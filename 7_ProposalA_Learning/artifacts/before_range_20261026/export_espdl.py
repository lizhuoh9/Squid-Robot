"""Export the normalized effort student, verify INT8, generate paired metadata."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import torch
from torch.utils.data import DataLoader, TensorDataset
from pipeline import Student, Hybrid, read_data, paired_evaluation, ROOT, FEATURES, SCHEMA_VERSION, HISTORY, DT


def paired_config(checkpoint, output_scale, checkpoint_hash, input_scale, limit=.3, enabled=False):
    def cf(value):
        text=format(float(value),".9g")
        return text+(".0" if "." not in text and "e" not in text else "")+"f"
    mean=", ".join(cf(v) for v in checkpoint["input_mean"])
    std=", ".join(cf(v) for v in checkpoint["input_std"])
    activation="experimental correction enabled by explicit user approval" if enabled else "correction disabled by default"
    return (f"// Generated paired metadata; {activation}.\n"
            "// Independent real-water acceptance is still pending.\n#pragma once\n#include <cstdint>\n"
            "namespace proposal_model_config {\n"
            f"constexpr bool kEnabled = {'true' if enabled else 'false'};\n"
            f"constexpr int kInputDim = {len(FEATURES)};\nconstexpr int kHistory = {HISTORY};\n"
            f"constexpr uint32_t kIntervalMs = {round(DT*1000)};\n"
            "constexpr int kCommandOffset = 4;\nconstexpr int kForwardIndex = kCommandOffset + kHistory;\n"
            "constexpr int kForwardHistoryOffset = kForwardIndex + 1;\n"
            f"constexpr float kLimit = {cf(limit)};\nconstexpr float kGuardError = 12.0f;\n"
            "constexpr float kGuardSpeed = 6.0f;\nconstexpr float kGuardNormalized = 6.0f;\n"
            f"constexpr char kSchemaVersion[] = \"{SCHEMA_VERSION}\";\n"
            f"constexpr char kCheckpointSha256[] = \"{checkpoint_hash}\";\n"
            f"constexpr float kInputScale = {cf(input_scale)};\n"
            f"constexpr float kOutputScale = {cf(output_scale)};\n"
            f"constexpr float kInputMean[kInputDim] = {{{mean}}};\n"
            f"constexpr float kInputStd[kInputDim] = {{{std}}};\n}}\n")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit",type=float,help="Runtime effort limit in cm/s^2; default is the checkpoint training limit")
    parser.add_argument("--enable",action="store_true",help="Explicit experimental activation; export defaults to disabled")
    args=parser.parse_args()
    from esp_ppq import TorchExecutor, TargetPlatform
    from esp_ppq.lib import Exporter
    from esp_ppq.api.espdl_interface import generate_test_value
    from esp_ppq.api import espdl_quantize_onnx
    torch.set_num_threads(1)
    out=ROOT/"artifacts/hybrid_mpc_v1"
    checkpoint_hash=hashlib.sha256((out/"student.pt").read_bytes()).hexdigest()
    checkpoint=torch.load(out/"student.pt",map_location="cpu",weights_only=False)
    training_limit=float(checkpoint["limit"])
    limit=training_limit if args.limit is None else args.limit
    if not math.isfinite(limit) or limit<=0:parser.error("--limit must be finite and positive")
    previous_export_path=out/"espdl_validation.json"
    previous_export=json.loads(previous_export_path.read_text(encoding="utf-8")) if previous_export_path.exists() else None
    windows=torch.load(out/"student_windows.pt",map_location="cpu",weights_only=False)
    input_dim=len(FEATURES)
    if (checkpoint["feature_names"]!=FEATURES or checkpoint.get("schema_version")!=SCHEMA_VERSION
            or windows.get("schema_version")!=SCHEMA_VERSION
            or len(checkpoint["input_mean"])!=input_dim or len(checkpoint["input_std"])!=input_dim
            or windows["inputs"].ndim!=2 or windows["inputs"].shape[1]!=input_dim):
        raise ValueError("Student checkpoint, window data and feature schema do not match")
    if not torch.isfinite(windows["inputs"]).all() or any(not math.isfinite(v) or v<=0 for v in checkpoint["input_std"]):
        raise ValueError("Nonfinite model inputs or invalid normalization")
    model=Student().eval();model.load_state_dict(checkpoint["state"])
    train=windows["inputs"][windows["split"]==0]
    if not len(train):raise ValueError("No training-only calibration data")
    calibration=train[torch.linspace(0,len(train)-1,min(128,len(train))).round().long()]
    with tempfile.TemporaryDirectory(prefix="proposal_export_",dir=out.parent) as scratch:
        staged=Path(scratch);onnx=staged/"proposal_student.onnx";espdl=staged/"proposal_student.espdl"
        torch.onnx.export(model,torch.zeros(1,input_dim),onnx,input_names=["features"],output_names=["effort_correction"],opset_version=18,dynamo=False)
        graph=espdl_quantize_onnx(onnx_import_file=str(onnx),espdl_export_file=str(espdl),
            calib_dataloader=DataLoader(TensorDataset(calibration),batch_size=1),calib_steps=len(calibration),
            input_shape=[1,input_dim],target="esp32s3",num_of_bits=8,collate_fn=lambda batch:batch[0],device="cpu",
            export_test_values=True,skip_export=True,error_report=False,verbose=0)
        output=graph.outputs["effort_correction"];operation=output.source_op
        config=operation.output_quant_config[operation.outputs.index(output)]
        assert config.num_of_bits==8 and int(config.offset.item())==0
        minimum_scale=2**math.ceil(math.log2(limit/min(-config.quant_min,config.quant_max)))
        config.scale=config.scale.new_tensor(max(float(config.scale.item()),minimum_scale))
        feature_variable=graph.inputs["features"]
        input_op=feature_variable.dest_ops[0]
        input_config=input_op.input_quant_config[input_op.inputs.index(feature_variable)]
        assert input_config.num_of_bits==8 and int(input_config.offset.item())==0
        # Every input accepted by the normalized +/-6 firmware guard must fit
        # INT8. Otherwise the worker would reject inputs accepted by the PC sim.
        minimum_input_scale=2**math.ceil(math.log2(6/127))
        input_config.scale=input_config.scale.new_tensor(max(float(input_config.scale.item()),minimum_input_scale))
        Exporter(platform=TargetPlatform.ESPDL_S3_INT8).export(file_path=str(espdl),graph=graph,
            values_for_test=generate_test_value(graph,"cpu",calibration[0].unsqueeze(0)),export_config=True)
        executor=TorchExecutor(graph=graph,device="cpu")
        report={"schema_version":SCHEMA_VERSION,"feature_names":FEATURES,"input_dim":input_dim,
                "history_seconds":HISTORY*DT,"sample_interval_ms":round(DT*1000),"limit_cm_s2":limit,
                "training_limit_cm_s2":training_limit,"weights_retrained":False,
                "enabled":args.enable,"experimental_user_override":args.enable,
                "output_scale":float(config.scale.item()),"calibration_count":len(calibration),
                "input_scale":float(input_config.scale.item()),
                "calibration_split":"train only, including selected DAgger samples",
                "student_checkpoint_sha256":checkpoint_hash}
        for name,split in (("validation",1),("test",2)):
            x=windows["inputs"][windows["split"]==split];truth=windows["labels"][windows["split"]==split]
            if not len(x):raise ValueError(f"No {name} windows for quantization validation")
            with torch.no_grad():
                floating=model(x).clamp(-limit,limit)
                quantized=torch.cat([executor.forward(inputs=row.unsqueeze(0))[0] for row in x]).clamp(-limit,limit)
            if not torch.isfinite(quantized).all():raise RuntimeError("Nonfinite quantized student")
            difference=(floating-quantized).abs()
            if difference.mean().item()>.03:raise RuntimeError("INT8 drift exceeds .03cm/s^2; refusing paired metadata")
            report[name]={"count":len(x),"quantized_float_mae_cm_s2":difference.mean().item(),
                          "quantized_float_max_error_cm_s2":difference.max().item(),
                          "quantized_teacher_mae_cm_s2":(quantized-truth).abs().mean().item(),
                          "float_output_min_cm_s2":floating.min().item(),"float_output_max_cm_s2":floating.max().item(),
                          "quantized_output_min_cm_s2":quantized.min().item(),"quantized_output_max_cm_s2":quantized.max().item()}
        class QuantizedStudent(torch.nn.Module):
            def forward(self,features):
                return executor.forward(inputs=features)[0]
        training_report=json.loads((out/"training_report.json").read_text(encoding="utf-8"))
        data=read_data([source["path"] for source in training_report["sources"]])
        if any(a["sha256"]!=b["sha256"] for a,b in zip(data["provenance"],training_report["sources"])):
            raise ValueError("Training source CSV changed since the checkpoint was generated")
        dynamics=torch.load(out/"dynamics_ensemble.pt",map_location="cpu",weights_only=False)
        if dynamics.get("schema_version")!=SCHEMA_VERSION:raise ValueError("Dynamics schema does not match")
        members=[]
        for state in dynamics["states"]:
            member=Hybrid().eval();member.load_state_dict(state);members.append(member)
        stats=(torch.tensor(checkpoint["input_mean"]),torch.tensor(checkpoint["input_std"]))
        report["heldout_quantized_model_rollout"]=paired_evaluation(
            data,(data["split"]==2).nonzero()[0][::4],members,QuantizedStudent().eval(),stats,correction_limit=limit)
        expected=training_report["student"]["heldout_model_rollout"]["evaluation_window_sha256"]
        if report["heldout_quantized_model_rollout"]["evaluation_window_sha256"]!=expected:
            raise ValueError("Quantized evaluation windows differ from the trained model evaluation")
        report["espdl_sha256"]=hashlib.sha256(espdl.read_bytes()).hexdigest()
        if previous_export:
            report["previous_export"]={key:previous_export[key] for key in
                                       ("espdl_sha256","limit_cm_s2","output_scale","test","heldout_quantized_model_rollout")
                                       if key in previous_export}
        (staged/"ProposalStudentConfig.h").write_text(paired_config(checkpoint,config.scale.item(),checkpoint_hash,
                                                                 input_config.scale.item(),limit,args.enable),encoding="utf-8")
        (staged/"espdl_validation.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
        status_path=out/"retraining_status.json"
        status=json.loads(status_path.read_text(encoding="utf-8")) if status_path.exists() else {}
        previous=status.pop("unchanged_deployment_files",None)
        if previous is not None:status["pre_export_deployment_files"]=previous
        status.update(espdl_requires_reexport=False,existing_export_is_stale=False,
                      firmware_requires_rebuild_after_reexport=True,robot_firmware_changed=False,
                      local_firmware_build_complete=False,robot_uploaded=False,student_runtime_ready_verified=False,
                      runtime_limit_cm_s2=limit,training_limit_cm_s2=training_limit,runtime_enabled=args.enable,
                      exported_student_checkpoint_sha256=checkpoint_hash,espdl_sha256=report["espdl_sha256"],
                      warning="Re-exported model and normalization header are paired. The changed runtime limit requires a local rebuild and has NOT been uploaded; deployment_report.json describes the last uploaded firmware.")
        (staged/"retraining_status.json").write_text(json.dumps(status,indent=2),encoding="utf-8")
        if hashlib.sha256((out/"student.pt").read_bytes()).hexdigest()!=checkpoint_hash:
            raise RuntimeError("Checkpoint changed during export; refusing to publish")
        for path in sorted(staged.iterdir(),key=lambda p:p.name=="retraining_status.json"):
            os.replace(path,out/path.name)
    print(json.dumps(report,indent=2))


if __name__=="__main__":main()
