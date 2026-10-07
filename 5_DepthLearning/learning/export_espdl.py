"""Export the trained depth residual MLP as an ESP32-S3 ESP-DL model."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

from .data import FEATURE_NAMES, load_many, make_windows
from .model import DepthResidualMLP

class NormalizedFeatureModel(nn.Module):
    """Accept normalized features; fold only output normalization into weights.

    Normalization must precede INT8 input quantization. Folding it into the
    first layer quantizes cm-scale signals together with 0..255 PWM values.
    """

    def __init__(self, checkpoint: dict) -> None:
        super().__init__()
        dimensions = tuple(int(value) for value in checkpoint["hidden_dims"])
        trained = DepthResidualMLP(len(checkpoint["input_mean"]), dimensions)
        trained.load_state_dict(checkpoint["model_state"])

        output_mean = float(checkpoint["output_mean"])
        output_std = float(checkpoint["output_std"])

        source = trained.net
        self.fc1 = nn.Linear(source[0].in_features, source[0].out_features)
        self.fc2 = nn.Linear(source[2].in_features, source[2].out_features)
        self.fc3 = nn.Linear(source[4].in_features, source[4].out_features)
        with torch.no_grad():
            self.fc1.load_state_dict(source[0].state_dict())
            self.fc2.load_state_dict(source[2].state_dict())
            self.fc3.weight.copy_(source[4].weight * output_std)
            self.fc3.bias.copy_(source[4].bias * output_std + output_mean)

    def forward(self, features: torch.Tensor) -> torch.Tensor:
        return self.fc3(torch.relu(self.fc2(torch.relu(self.fc1(features)))))


def _ensure_residual_output_range(graph, limit: float = 20.0) -> tuple[object, float]:
    output = graph.outputs["depth_residual"]
    operation = output.source_op
    config = operation.output_quant_config[operation.outputs.index(output)]
    if config.num_of_bits != 8 or int(config.offset.item()) != 0:
        raise ValueError("Depth output must use symmetric INT8 quantization")
    previous_scale = float(config.scale.item())
    required_scale = 2.0 ** math.ceil(math.log2(limit / min(-config.quant_min, config.quant_max)))
    config.scale = config.scale.new_tensor(max(previous_scale, required_scale))
    scale = float(config.scale.item())
    if config.quant_min * scale > -limit or config.quant_max * scale < limit:
        raise ValueError("Quantized output cannot represent the complete residual range")
    # Exact endpoint roundtrip: neither +20 nor -20 may clip to INT8 limits.
    endpoints = torch.tensor([-limit, limit])
    decoded = (endpoints / scale).round().clamp(config.quant_min, config.quant_max) * scale
    if not torch.equal(decoded, endpoints):
        raise ValueError("Residual endpoints are not represented exactly")
    return config, previous_scale


def _write_metadata(path: Path, checkpoint: dict, output_config) -> None:
    def cpp_float(value: float) -> str:
        value = format(float(value), ".9g")
        if "." not in value and "e" not in value:
            value += ".0"
        return value + "f"

    mean = ", ".join(cpp_float(x) for x in checkpoint["input_mean"])
    std = ", ".join(cpp_float(max(float(x), 1.0e-6)) for x in checkpoint["input_std"])
    text = (
        "// Generated with depth_model_espdl.espdl. Keep this header and model paired.\n"
        "#pragma once\n#include <cstdint>\n\nnamespace depth_model_config {\n"
        f"constexpr int kWindow = {int(checkpoint['window'])};\n"
        f"constexpr int kFeaturesPerFrame = {len(checkpoint['feature_names'])};\n"
        "constexpr int kInputDim = kWindow * kFeaturesPerFrame;\n"
        f"constexpr uint32_t kSampleIntervalMs = {int(checkpoint['sample_interval_ms'])};\n"
        "constexpr float kResidualLimit = 20.0f;\n"
        f"constexpr float kOutputScale = {cpp_float(output_config.scale.item())};\n"
        f"constexpr int kOutputQuantMin = {output_config.quant_min};\n"
        f"constexpr int kOutputQuantMax = {output_config.quant_max};\n"
        f"constexpr float kInputMean[kInputDim] = {{{mean}}};\n"
        f"constexpr float kInputStd[kInputDim] = {{{std}}};\n"
        "}  // namespace depth_model_config\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--csv", required=True, nargs="+", type=Path,
                        help="training CSVs used to build representative calibration windows")
    parser.add_argument("--onnx-output", required=True, type=Path)
    parser.add_argument("--espdl-output", required=True, type=Path)
    parser.add_argument("--metadata-output", required=True, type=Path)
    parser.add_argument("--calib-steps", type=int, default=128)
    args = parser.parse_args()

    try:
        from esp_ppq import TorchExecutor, TargetPlatform
        from esp_ppq.lib import Exporter
        from esp_ppq.api.espdl_interface import generate_test_value
        from esp_ppq.api import espdl_quantize_onnx
    except ImportError as exc:
        raise SystemExit("ESP-PPQ is required. Install it with: python -m pip install esp-ppq") from exc

    checkpoint = torch.load(args.model, map_location="cpu", weights_only=False)
    names = tuple(checkpoint["feature_names"])
    if names != FEATURE_NAMES or checkpoint.get("sample_interval_ms", 0) <= 0:
        raise SystemExit("Checkpoint must contain the matching feature order and sample interval")
    torch.set_num_threads(1)
    model = NormalizedFeatureModel(checkpoint).eval()
    input_dim = len(checkpoint["input_mean"])
    dummy = torch.zeros(1, input_dim, dtype=torch.float32)

    args.onnx_output.parent.mkdir(parents=True, exist_ok=True)
    args.espdl_output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model,
        dummy,
        args.onnx_output,
        input_names=["features"],
        output_names=["depth_residual"],
        opset_version=18,
        do_constant_folding=True,
        dynamo=False,
    )

    samples = load_many(args.csv, float(checkpoint["teacher_kp"]), float(checkpoint["teacher_kd"]),
                        int(checkpoint["sample_interval_ms"]))
    windows, targets, _ = make_windows(samples, int(checkpoint["window"]))
    if len(windows) != int(checkpoint["sample_count"]):
        raise SystemExit("Calibration dataset does not reproduce the trained window count")
    mean = torch.tensor(checkpoint["input_mean"], dtype=torch.float32)
    std = torch.tensor(checkpoint["input_std"], dtype=torch.float32).clamp_min(1.0e-6)
    normalized = (torch.tensor(windows, dtype=torch.float32) - mean) / std
    if not torch.isfinite(normalized).all():
        raise SystemExit("Calibration CSV produced non-finite feature values")
    # Cover all training conditions, instead of calibrating only the first
    # approach to a target. Validation data does not participate in calibration.
    train_count = int(checkpoint["train_count"])
    steps = min(args.calib_steps, train_count)
    if steps < 1:
        raise SystemExit("Calibration needs at least one training window")
    indices = torch.linspace(0, train_count - 1, steps=steps).round().long()
    calibration = normalized[indices]
    loader = DataLoader(TensorDataset(calibration), batch_size=1, shuffle=False)

    def collate_fn(batch):
        return batch[0]

    graph = espdl_quantize_onnx(
        onnx_import_file=str(args.onnx_output),
        espdl_export_file=str(args.espdl_output),
        calib_dataloader=loader,
        calib_steps=steps,
        input_shape=[1, input_dim],
        target="esp32s3",
        num_of_bits=8,
        collate_fn=collate_fn,
        device="cpu",
        export_test_values=True,
        skip_export=True,
        error_report=False,
        verbose=1,
    )
    # Fix the calibrated output scale BEFORE serializing the ESP-DL binary.
    # A scale of 0.125 clips INT8 output at -16/+15.875 despite firmware's ±20 limit.
    output_config, previous_output_scale = _ensure_residual_output_range(graph)
    test_values = generate_test_value(graph, "cpu", calibration[0].unsqueeze(0))
    Exporter(platform=TargetPlatform.ESPDL_S3_INT8).export(
        file_path=str(args.espdl_output), graph=graph,
        values_for_test=test_values, export_config=True,
    )
    serialized = json.loads(args.espdl_output.with_suffix(".json").read_text(encoding="utf-8"))
    serialized_config = serialized["configs"]["/fc3/Gemm"]["depth_residual"]
    serialized_scale = serialized["values"][str(serialized_config["dominator"])]["scale"]
    if abs(serialized_scale - float(output_config.scale.item())) > 1.0e-8:
        raise ValueError("Exported output scale does not match the verified graph")
    eval_start = int(checkpoint["eval_start_index"])
    eval_inputs = normalized[eval_start:]
    truth = torch.tensor(targets[eval_start:], dtype=torch.float32).reshape(-1, 1)
    executor = TorchExecutor(graph=graph, device="cpu")
    with torch.no_grad():
        float_predictions = model(eval_inputs).clamp(-20.0, 20.0)
        quant_predictions = torch.cat([
            executor.forward(inputs=frame.unsqueeze(0))[0]
            for frame in eval_inputs
        ]).clamp(-20.0, 20.0)
    if not torch.isfinite(quant_predictions).all():
        raise SystemExit("Quantized validation produced non-finite predictions")
    error = quant_predictions - truth
    float_error = float_predictions - truth
    input_config = graph.operations["/fc1/Gemm"].input_quant_config[0]
    report = {
        "eval_count": len(eval_inputs),
        "calibration_count": steps,
        "normalized_inputs": True,
        "feature_names": list(names),
        "sample_interval_ms": int(checkpoint["sample_interval_ms"]),
        "input_quantization_scale": float(input_config.scale.item()),
        "previous_output_scale": previous_output_scale,
        "output_quantization_scale": float(output_config.scale.item()),
        "output_representable_min": output_config.quant_min * float(output_config.scale.item()),
        "output_representable_max": output_config.quant_max * float(output_config.scale.item()),
        "residual_endpoint_roundtrip": [-20.0, 20.0],
        "eval_predictions_beyond_old_range": int((quant_predictions.abs() > 16.0).sum().item()),
        "float_mae": float(float_error.abs().mean().item()),
        "quantized_mae": float(error.abs().mean().item()),
        "quantized_rmse": float(error.square().mean().sqrt().item()),
        "quantized_max_abs_error": float(error.abs().max().item()),
        "quantized_vs_float_mae": float((quant_predictions - float_predictions).abs().mean().item()),
    }
    _write_metadata(args.metadata_output, checkpoint, output_config)
    args.espdl_output.with_suffix(".validation.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    print(f"saved {args.onnx_output}")
    print(f"saved {args.espdl_output}")


if __name__ == "__main__":
    main()
