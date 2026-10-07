"""Export and verify the distilled policy using preserved MPC teacher windows."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

from ..export_espdl import NormalizedFeatureModel, _ensure_residual_output_range, _write_metadata
from .data import FEATURE_NAMES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", required=True, type=Path)
    args = parser.parse_args()
    out = args.model_dir
    checkpoint = torch.load(out / "depth_model.pt", map_location="cpu", weights_only=False)
    windows = torch.load(out / "policy_windows.pt", map_location="cpu", weights_only=False)
    assert checkpoint["feature_names"] == FEATURE_NAMES
    assert checkpoint["feature_schema"] == "model_based_base_pwm_v1"
    assert len(checkpoint["input_mean"]) == 25 and checkpoint["sample_interval_ms"] == 500
    from esp_ppq import TorchExecutor, TargetPlatform
    from esp_ppq.lib import Exporter
    from esp_ppq.api.espdl_interface import generate_test_value
    from esp_ppq.api import espdl_quantize_onnx
    torch.set_num_threads(1)
    model = NormalizedFeatureModel(checkpoint).eval()
    mean, std = torch.tensor(checkpoint["input_mean"]), torch.tensor(checkpoint["input_std"])
    normalize = lambda x: (x - mean) / std
    train = normalize(windows["train_inputs"])
    validation = normalize(windows["validation_inputs"])
    test = normalize(windows["test_inputs"])
    assert all(torch.isfinite(x).all() for x in (train, validation, test))
    onnx_path, espdl_path = out / "depth_model.onnx", out / "depth_model_espdl.espdl"
    torch.onnx.export(model, torch.zeros(1, 25), onnx_path, input_names=["features"],
                      output_names=["depth_residual"], opset_version=18, dynamo=False)
    steps = min(128, len(train))
    indices = torch.linspace(0, len(train) - 1, steps).round().long()
    calibration = train[indices]
    graph = espdl_quantize_onnx(onnx_import_file=str(onnx_path), espdl_export_file=str(espdl_path),
                              calib_dataloader=DataLoader(TensorDataset(calibration), batch_size=1),
                              calib_steps=steps, input_shape=[1, 25], target="esp32s3", num_of_bits=8,
                              collate_fn=lambda batch: batch[0], device="cpu", export_test_values=True,
                              skip_export=True, error_report=False, verbose=0)
    config, previous_scale = _ensure_residual_output_range(graph)
    Exporter(platform=TargetPlatform.ESPDL_S3_INT8).export(
        file_path=str(espdl_path), graph=graph,
        values_for_test=generate_test_value(graph, "cpu", calibration[0].unsqueeze(0)), export_config=True)
    serialized = json.loads(espdl_path.with_suffix(".json").read_text(encoding="utf-8"))
    output = serialized["configs"]["/fc3/Gemm"]["depth_residual"]
    assert abs(serialized["values"][str(output["dominator"])]["scale"] - config.scale.item()) < 1.0e-8
    executor = TorchExecutor(graph=graph, device="cpu")
    report = {"feature_names": FEATURE_NAMES, "sample_interval_ms": 500, "calibration_count": steps,
              "previous_output_scale": previous_scale, "output_scale": config.scale.item(),
              "output_representable_range": [config.quant_min * config.scale.item(), config.quant_max * config.scale.item()],
              "residual_endpoint_roundtrip": [-20.0, 20.0], "closed_loop_accuracy_measured": False}
    for name, x in (("validation", validation), ("test", test)):
        with torch.no_grad():
            floating = model(x).clamp(-20, 20).reshape(-1)
            quantized = torch.cat([executor.forward(inputs=row.unsqueeze(0))[0] for row in x]).clamp(-20, 20).reshape(-1)
        truth = windows[name + "_labels"]
        assert torch.isfinite(quantized).all()
        quant_difference = (floating - quantized).abs().mean().item()
        if quant_difference > 2.0:
            raise RuntimeError("Quantization changes policy by over 2 output points on average; refusing metadata")
        report[name] = {"count": len(x), "float_teacher_mae_output_points": (floating - truth).abs().mean().item(),
                        "quantized_teacher_mae_output_points": (quantized - truth).abs().mean().item(),
                        "quantized_vs_float_mae_output_points": quant_difference}
    header = out / "DepthResidualModelConfig.h"
    _write_metadata(header, checkpoint, config)
    text = header.read_text(encoding="utf-8")
    text += ("\n// This model uses causal base PWM, not telemetry PWM.\n#define SQUID_DEPTH_MODEL_BASED 1\n"
             "namespace depth_model_config {\n"
             f"constexpr float kGuardErrorCm = {checkpoint['guard_error_cm']:.1f}f;\n"
             f"constexpr float kGuardSpeedCmS = {checkpoint['guard_speed_cm_s']:.1f}f;\n"
             f"constexpr float kGuardNormalizedAbs = {checkpoint['guard_normalized_abs']:.1f}f;\n"
             "}\n")
    header.write_text(text, encoding="utf-8")
    (out / "depth_model_espdl.validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
