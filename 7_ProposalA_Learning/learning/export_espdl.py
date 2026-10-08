"""Export the normalized effort student, verify INT8, generate paired metadata."""
import hashlib
import json
import math
from pathlib import Path

import torch
from torch.utils.data import DataLoader, TensorDataset
from pipeline import Student, ROOT, FEATURES


def main():
    from esp_ppq import TorchExecutor, TargetPlatform
    from esp_ppq.lib import Exporter
    from esp_ppq.api.espdl_interface import generate_test_value
    from esp_ppq.api import espdl_quantize_onnx
    torch.set_num_threads(1)
    out=ROOT/"artifacts/hybrid_mpc_v1"
    checkpoint=torch.load(out/"student.pt",map_location="cpu",weights_only=False)
    windows=torch.load(out/"student_windows.pt",map_location="cpu",weights_only=False)
    assert checkpoint["feature_names"]==FEATURES and len(checkpoint["input_mean"])==34
    model=Student().eval();model.load_state_dict(checkpoint["state"])
    onnx=out/"proposal_student.onnx";espdl=out/"proposal_student.espdl"
    torch.onnx.export(model,torch.zeros(1,34),onnx,input_names=["features"],output_names=["effort_correction"],opset_version=18,dynamo=False)
    train=windows["inputs"][windows["split"]==0]
    calibration=train[torch.linspace(0,len(train)-1,min(128,len(train))).round().long()]
    graph=espdl_quantize_onnx(onnx_import_file=str(onnx),espdl_export_file=str(espdl),
        calib_dataloader=DataLoader(TensorDataset(calibration),batch_size=1),calib_steps=len(calibration),
        input_shape=[1,34],target="esp32s3",num_of_bits=8,collate_fn=lambda batch:batch[0],device="cpu",
        export_test_values=True,skip_export=True,error_report=False,verbose=0)
    output=graph.outputs["effort_correction"];operation=output.source_op
    config=operation.output_quant_config[operation.outputs.index(output)]
    assert config.num_of_bits==8 and int(config.offset.item())==0
    minimum_scale=2**math.ceil(math.log2(.3/min(-config.quant_min,config.quant_max)))
    config.scale=config.scale.new_tensor(max(float(config.scale.item()),minimum_scale))
    Exporter(platform=TargetPlatform.ESPDL_S3_INT8).export(file_path=str(espdl),graph=graph,
        values_for_test=generate_test_value(graph,"cpu",calibration[0].unsqueeze(0)),export_config=True)
    executor=TorchExecutor(graph=graph,device="cpu")
    report={"input_dim":34,"history_seconds":3,"sample_interval_ms":100,"limit_cm_s2":.3,
            "enabled":False,"output_scale":float(config.scale.item()),"calibration_count":len(calibration)}
    for name,split in (("validation",1),("test",2)):
        x=windows["inputs"][windows["split"]==split];truth=windows["labels"][windows["split"]==split]
        with torch.no_grad():
            floating=model(x).clamp(-.3,.3)
            quantized=torch.cat([executor.forward(inputs=row.unsqueeze(0))[0] for row in x]).clamp(-.3,.3)
        if not torch.isfinite(quantized).all():raise RuntimeError("Nonfinite quantized student")
        difference=(floating-quantized).abs().mean().item()
        if difference>.03:raise RuntimeError("INT8 drift exceeds .03cm/s^2; refusing paired metadata")
        report[name]={"count":len(x),"quantized_float_mae_cm_s2":difference,"quantized_teacher_mae_cm_s2":(quantized-truth).abs().mean().item()}
    report["espdl_sha256"]=hashlib.sha256(espdl.read_bytes()).hexdigest()
    # No automatic enablement from an offline simulation on three sessions.
    def cf(v):
        value=format(float(v),".9g")
        return value+(".0" if "." not in value and "e" not in value else "")+"f"
    mean=", ".join(cf(v) for v in checkpoint["input_mean"])
    std=", ".join(cf(v) for v in checkpoint["input_std"])
    text=("// Generated paired metadata; enable only after independent validation.\n#pragma once\n#include <cstdint>\n"
          "namespace proposal_model_config {\nconstexpr bool kEnabled = false;\nconstexpr int kInputDim = 34;\n"
          "constexpr int kHistory = 30;\nconstexpr uint32_t kIntervalMs = 100;\nconstexpr float kLimit = 0.3f;\n"
          "constexpr float kGuardError = 12.0f;\nconstexpr float kGuardSpeed = 6.0f;\nconstexpr float kGuardNormalized = 6.0f;\n"
          f"constexpr float kOutputScale = {cf(config.scale.item())};\n"
          f"constexpr float kInputMean[kInputDim] = {{{mean}}};\nconstexpr float kInputStd[kInputDim] = {{{std}}};\n}}\n")
    (out/"ProposalStudentConfig.h").write_text(text,encoding="utf-8")
    (out/"espdl_validation.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report,indent=2))


if __name__=="__main__":main()
