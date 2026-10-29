"""Export the separate RL actor logits to INT8; tanh*limit runs in firmware.

Calibration uses only train rollouts. Frozen base model files are never edited.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import torch
from torch.utils.data import DataLoader,TensorDataset
import pipeline as p
from residual_rl import Actor,Env,LIMIT,OBS_DIM,EPISODE_STEPS,load_frozen,evaluate,acceptance


def require_current_actor(report,checkpoint,allow_unvalidated=False):
    artifact=report.get('policy_artifact','actor.pt')
    if artifact not in ('actor.pt','candidate.pt'):
        raise RuntimeError('Training report names an unknown policy artifact')
    if (report.get('export_allowed') is False or artifact!='actor.pt') and not allow_unvalidated:
        raise RuntimeError('Failed research candidate requires explicit --allow-unvalidated')
    if checkpoint['selected_update']!=report['selected_update']:
        raise ValueError('Actor and training report do not match')
    return artifact


@torch.no_grad()
def observations(data,ids,plant,student,stats,actor):
    env=Env(data,plant,student,stats,ids);values=[]
    for _ in range(EPISODE_STEPS):
        obs=env.observation();values.append(obs[env.valid].clone());env.step(actor(obs))
    return torch.cat(values)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--enable',action='store_true');parser.add_argument('--allow-unvalidated',action='store_true');args=parser.parse_args()
    if args.allow_unvalidated and not args.enable:parser.error('override requires --enable')
    from esp_ppq import TorchExecutor,TargetPlatform
    from esp_ppq.lib import Exporter
    from esp_ppq.api.espdl_interface import generate_test_value
    from esp_ppq.api import espdl_quantize_onnx
    torch.set_num_threads(1)
    out=p.ROOT/'artifacts/residual_rl_v1'
    report=json.loads((out/'training_report.json').read_text(encoding='utf-8'))
    artifact=report.get('policy_artifact','actor.pt')
    ck=torch.load(out/artifact,weights_only=False,map_location='cpu')
    require_current_actor(report,ck,args.allow_unvalidated)
    if ck['input_dim']!=OBS_DIM or ck['limit_cm_s2']!=LIMIT or ck['combined_correction_clipped']:raise ValueError('Actor contract mismatch')
    if args.enable and not report['acceptance_passed'] and not args.allow_unvalidated:raise RuntimeError('Failed policy requires explicit experimental override')
    data,members,student,stats,frozen=load_frozen()
    # PPO observes the floating student and dynamics checkpoint. ESP-DL files
    # are derived deployment artifacts and are re-exported independently.
    for path,digest in ck['frozen_sha256'].items():
        name=Path(path).name
        if name in ('student.pt','dynamics_ensemble.pt') and frozen.get(path)!=digest:
            raise ValueError(f'Frozen training model changed since PPO training: {name}')
    actor=Actor().eval();actor.load_state_dict(ck['state'])
    ids=np.flatnonzero(data['split']==0);ids=ids[np.linspace(0,len(ids)-1,min(128,len(ids))).round().astype(int)]
    x=observations(data,ids,members[0],student,stats,actor)
    if not len(x):raise ValueError('No eligible training calibration inputs')
    calibration=x[torch.linspace(0,len(x)-1,min(512,len(x))).round().long()]
    torch.onnx.export(actor.net,torch.zeros(1,OBS_DIM),out/'residual_rl.onnx',input_names=['features'],output_names=['rl_logit'],opset_version=18,dynamo=False)
    graph=espdl_quantize_onnx(onnx_import_file=str(out/'residual_rl.onnx'),espdl_export_file=str(out/'residual_rl.espdl'),
        calib_dataloader=DataLoader(TensorDataset(calibration),batch_size=1),calib_steps=len(calibration),input_shape=[1,OBS_DIM],
        target='esp32s3',num_of_bits=8,collate_fn=lambda batch:batch[0],device='cpu',export_test_values=True,skip_export=True,error_report=False,verbose=0)
    variable=graph.inputs['features'];op=variable.dest_ops[0];ic=op.input_quant_config[op.inputs.index(variable)]
    variable=graph.outputs['rl_logit'];op=variable.source_op;oc=op.output_quant_config[op.outputs.index(variable)]
    if ic.num_of_bits!=8 or oc.num_of_bits!=8 or int(ic.offset.item())!=0 or int(oc.offset.item())!=0:raise ValueError('INT8 quantization contract')
    ic.scale=ic.scale.new_tensor(max(float(ic.scale.item()),2**math.ceil(math.log2(6/127))))
    with torch.no_grad():logit_extent=max(4.,float(actor.net(calibration).abs().max())*1.25)
    oc.scale=oc.scale.new_tensor(max(float(oc.scale.item()),2**math.ceil(math.log2(logit_extent/127))))
    Exporter(platform=TargetPlatform.ESPDL_S3_INT8).export(file_path=str(out/'residual_rl.espdl'),graph=graph,values_for_test=generate_test_value(graph,'cpu',calibration[:1]),export_config=True)
    executor=TorchExecutor(graph=graph,device='cpu')
    class QuantizedActor:
        def __call__(self,obs):return LIMIT*torch.tanh(executor.forward(inputs=obs)[0].squeeze(-1))
    quant=QuantizedActor();result={'enabled':args.enable,'experimental_override':args.allow_unvalidated,'float_acceptance_passed':report['acceptance_passed'],
        'input_dim':OBS_DIM,'input_scale':float(ic.scale.item()),'logit_scale':float(oc.scale.item()),'rl_limit_cm_s2':LIMIT,
        'combined_correction_clipped':False,'calibration_split':'train only','calibration_count':len(calibration),
        'runtime_observation_contract':'125 normalized causal features, NN correction divided by 0.8, previous RL correction divided by 0.8, depth/40',
        'output_contract':'INT8 logit -> dequantize -> tanh -> multiply0.8'}
    with torch.no_grad():
        for name,split in [('validation',1),('test',2)]:
            ids=np.flatnonzero(data['split']==split)[::4]
            obs=observations(data,ids,members[0],student,stats,actor)
            obs=obs[torch.linspace(0,len(obs)-1,min(1024,len(obs))).round().long()]
            delta=(quant(obs)-actor(obs)).abs()
            drift=delta.mean().item()
            if not torch.isfinite(delta).all() or (drift>.04 and not args.allow_unvalidated):
                raise RuntimeError(f'RL INT8 output drift {drift:.6f} exceeds .04cm/s^2')
            result[name+'_quantization']={'count':len(obs),'mae_cm_s2':drift,'max_cm_s2':delta.max().item(),
                'threshold_cm_s2':.04,'threshold_passed':drift<=.04}
            reference=evaluate(data,ids,members,student,stats,None);candidate=evaluate(data,ids,members,student,stats,quant)
            result[name+'_baseline']=reference;result[name+'_quantized']=candidate;result[name+'_acceptance_checks']=acceptance(candidate,reference)
    result['quantized_acceptance_passed']=all(result['validation_acceptance_checks'].values()) and all(result['test_acceptance_checks'].values())
    if args.enable and not result['quantized_acceptance_passed'] and not args.allow_unvalidated:raise RuntimeError('Quantized policy failed release acceptance')
    result['policy_artifact']=artifact
    result['actor_sha256']=hashlib.sha256((out/artifact).read_bytes()).hexdigest();result['espdl_sha256']=hashlib.sha256((out/'residual_rl.espdl').read_bytes()).hexdigest()
    def cf(v):
        s=format(v,'.9g');return s+('.0' if '.' not in s and 'e' not in s else '')+'f'
    text=('// Generated experimental RL metadata; acceptance is reported separately.\n#pragma once\nnamespace residual_rl_config {\n'
          f'constexpr bool kEnabled = {str(args.enable).lower()};\nconstexpr int kInputDim = {OBS_DIM};\n'
          f'constexpr float kLimit = {cf(LIMIT)};\nconstexpr float kInputScale = {cf(float(ic.scale.item()))};\n'
          f'constexpr float kLogitScale = {cf(float(oc.scale.item()))};\n'
          f'constexpr char kBaseCheckpointSha256[] = "{hashlib.sha256((p.ROOT/"artifacts/hybrid_mpc_v1/student.pt").read_bytes()).hexdigest()}";\n'
          f'constexpr char kActorSha256[] = "{result["actor_sha256"]}";\n}}\n')
    (out/'ResidualRLConfig.h').write_text(text,encoding='utf-8')
    for path,h in frozen.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=h:raise RuntimeError('Base model changed during export')
    (out/'espdl_validation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ['enabled','quantized_acceptance_passed','validation_quantization','test_quantization','espdl_sha256']}),flush=True)


if __name__=='__main__':main()
