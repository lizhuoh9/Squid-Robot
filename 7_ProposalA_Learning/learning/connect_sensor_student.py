"""Distill the frozen sensor-history dynamics into a bounded ESP-DL student.

No backup, robot IO, dynamics retraining, or automatic activation.
"""
import json
import hashlib
from pathlib import Path
import tempfile
import os

import torch
import numpy as np
import pipeline as p
from fit_dynamics_only import run_separated_data


def main():
    torch.set_num_threads(1);np.random.seed(101)
    out=p.ROOT/'artifacts/hybrid_mpc_v1'
    prediction=json.loads((out/'dynamics_prediction_report.json').read_text(encoding='utf-8'))
    sources=prediction['sources']
    training=[s['path'] for s in sources if s['role']=='train']
    validation=next(s['path'] for s in sources if s['role']=='validation')
    test=next(s['path'] for s in sources if s['role']=='test')
    if any(hashlib.sha256(Path(s['path']).read_bytes()).hexdigest()!=s['sha256'] for s in sources):
        raise ValueError('Source CSV changed')
    data=run_separated_data(training,validation,test)
    data['policy']=np.c_[data['policy'],data['state_history'].numpy().reshape(-1,60)]
    data['student_feature_names']=p.SENSOR_FEATURES;data['student_schema_version']=p.SENSOR_STUDENT_SCHEMA
    dynamics_hash=hashlib.sha256((out/'dynamics_ensemble.pt').read_bytes()).hexdigest()
    if dynamics_hash!=prediction['checkpoint_sha256']:raise ValueError('Prediction checkpoint changed')
    dynamics=torch.load(out/'dynamics_ensemble.pt',map_location='cpu',weights_only=False)
    if dynamics.get('model_class')!='SensorHistoryHybrid':raise ValueError('Expected improved sensor-history dynamics')
    members=[]
    for state in dynamics['states']:
        model=p.SensorHistoryHybrid().eval();model.load_state_dict(state);members.append(model)
    deployment=json.loads((out/'deployment_report.json').read_text(encoding='utf-8'))
    previous_path=out/'student.pt'
    if hashlib.sha256(previous_path.read_bytes()).hexdigest()!=deployment['student_checkpoint_sha256']:
        # Read an already-existing deployed-model snapshot; do not create backups.
        previous_path=out.parent/'before_range_20261026/student.pt'
    previous_hash=hashlib.sha256(previous_path.read_bytes()).hexdigest() if previous_path.exists() else None
    old=None
    if previous_hash==deployment['student_checkpoint_sha256']:
        previous_checkpoint=torch.load(previous_path,map_location='cpu',weights_only=False)
        previous=p.Student(len(previous_checkpoint['feature_names'])).eval();previous.load_state_dict(previous_checkpoint['state'])
        old=(previous,(torch.tensor(previous_checkpoint['input_mean']),torch.tensor(previous_checkpoint['input_std'])))
    deployment_hash=hashlib.sha256((out/'deployment_report.json').read_bytes()).hexdigest()
    print('SENSOR MPC / STUDENT',np.bincount(data['split'],minlength=3),'INPUTS',len(p.SENSOR_FEATURES),flush=True)
    with tempfile.TemporaryDirectory(prefix='sensor_student_',dir=out.parent) as scratch:
        staged=Path(scratch)
        student_limit=.8
        student=p.distill(data,members,staged,250,dagger_rounds=2,correction_limit=student_limit,objective='range',previous=old)
        report={'schema_version':p.SENSOR_STUDENT_SCHEMA,'feature_names':p.SENSOR_FEATURES,
                'sources':data['provenance'],'split_counts':np.bincount(data['split'],minlength=3).tolist(),
                'split_protocol':'disjoint entire recordings; same sources/roles/windows as improved prediction evaluation',
                'objective':'range','prediction_checkpoint_sha256':dynamics_hash,'prediction_model_class':'SensorHistoryHybrid',
                'previous_student_sha256':previous_hash,'previous_student_reference_available':old is not None,'student':student,
                'evaluation_protocol':{'duration_s':4,'control_interval_ms':100,'physics_interval_ms':50,
                    'model_selection':'validation only; final test is not used to select a round',
                    'forward_forecast':'hold currently observed forward state, not future recorded flags',
                    'sensor_forecast':'only own predicted states update future sensor history; no future measured states',
                    'acceptance':'fixed steady-hold starting abs(error)<=2 and abs(speed)<=0.3, at least 20 windows; compare previous deployed model only if its exact checkpoint is available; otherwise rank validation peak and p95 directly',
                'teacher_candidates':f'81 coarse 4-move plans plus 9 local refinements; first move bounded by +/-{student_limit} cm/s^2',
                'student_correction_limit_cm_s2':student_limit},
                'deployment_gate':{'enabled':False,'real_water_student_validation':False,
                    'range_acceptance':student.get('range_acceptance'),'reason':'Offline candidate only; no upload or real-water acceptance'},
                'limitations':['4s model MPC uses dynamics validated on 3s predictions',
                    'future simulated speed/acceleration are model states, not a full Kalman sensor emulator',
                    'commanded/estimated actuators, not physical pump/valve feedback',
                    'initial pump phase and direction lock are assumed; 50ms simulation vs 5ms actual motion ticks',
                    'existing retrospective validation/test logs have been inspected before; not a prospective trial']}
        (staged/'training_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        for name in ('student.pt','student_windows.pt','teacher_labels.csv','training_report.json'):os.replace(staged/name,out/name)
    if hashlib.sha256((out/'dynamics_ensemble.pt').read_bytes()).hexdigest()!=dynamics_hash:raise RuntimeError('Dynamics modified')
    if hashlib.sha256((out/'deployment_report.json').read_bytes()).hexdigest()!=deployment_hash:raise RuntimeError('Deployment record modified')
    print(json.dumps({'test':student['heldout_model_rollout'],'acceptance':student.get('range_acceptance')},indent=2))


if __name__=='__main__':main()
