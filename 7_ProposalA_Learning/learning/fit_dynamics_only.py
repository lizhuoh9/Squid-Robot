"""Train prediction dynamics only. No MPC labels, student export or robot IO.

No backup files are created. Previous models stay in memory until validation
selection; test data is evaluated once after selecting the final ensemble.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from pipeline import ROOT,DT,HISTORY,HORIZON,SCHEMA_VERSION,DYNAMICS_DIM,DYNAMICS_COMMANDS,Hybrid,HistoryHybrid,SensorHistoryHybrid,read_data


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def run_separated_data(training,validation,test):
    """Use disjoint entire runs; preserve continuity checks within each run."""
    groups=[(Path(p),0) for p in training]+[(Path(validation),1),(Path(test),2)]
    resolved=[str(p.resolve()) for p,_ in groups]
    if len(set(resolved))!=len(groups) or len({digest(p) for p,_ in groups})!=len(groups):
        raise ValueError('Training, validation and test must be distinct runs')
    records=[read_data([path],exclude_duplicate_timestamps=True,include_state_history=True) for path,_ in groups]
    result={key:torch.cat([d[key] for d in records]) for key in ('x','u','forward_u','y')}
    if all('state_history' in d for d in records):result['state_history']=torch.cat([d['state_history'] for d in records])
    result.update(policy=np.concatenate([d['policy'] for d in records]),
                  times=np.concatenate([d['times'] for d in records]),
                  session=np.concatenate([np.full(len(d['times']),i) for i,d in enumerate(records)]),
                  split=np.concatenate([np.full(len(d['times']),split) for d,(_,split) in zip(records,groups)]),
                  provenance=[{**d['provenance'][0],'role':('train','validation','test')[split]} for d,(_,split) in zip(records,groups)])
    return result


def hold_masks(data,ids):
    slow=data['x'][ids,1].abs()<=.3
    close=torch.as_tensor(np.abs(data['policy'][ids,0])<=2)
    return slow,slow & close


def prediction_loss(pred,truth,weights=None):
    depth=pred[:,:,0]-truth[:,:,0]
    # Depth prediction is the target; filtered CSV speed is not true velocity.
    # Emphasize 0.5/1/2/3-second endpoints instead of fitting delayed speed.
    anchors=depth[:,[4,9,19,29]]
    rows=depth.square().mean(1)+anchors.square().mean(1)+.05*(pred[:,:,1]-truth[:,:,1]).square().mean(1)
    return rows.mean() if weights is None else (rows*weights).sum()/weights.sum()


@torch.no_grad()
def predict(members,data,ids):
    return torch.stack([model_forward(m,data,ids) for m in members]).mean(0)


def model_forward(model,data,ids):
    extra={'sensor_history':data['state_history'][ids]} if isinstance(model,SensorHistoryHybrid) else {}
    return model(data['x'][ids],data['u'][ids],data['forward_u'][ids],**extra)


def metrics(pred,data,ids):
    error=pred[:,:,0]-data['y'][ids,:,0]
    def group(mask):
        e=error[mask]
        result={'count':int(mask.sum()),'trajectory_mae_cm':e.abs().mean().item()}
        for time,tick in ((.5,4),(1,9),(2,19),(3,29)):
            values=e[:,tick]
            result[str(time)+'s']={'mae_cm':values.abs().mean().item(),
                                 'bias_cm':values.mean().item(),
                                 'p95_abs_cm':torch.quantile(values.abs(),.95).item()}
        return result
    result=group(np.ones(len(ids),bool))
    target=data['x'][ids,0].numpy()+40-data['policy'][ids,0]
    speed=data['x'][ids,1].numpy()
    history=data['x'][ids,DYNAMICS_COMMANDS].numpy()
    reversal=np.any(history[:,1:]*history[:,:-1]<0,axis=1)
    masks={**{f'target_{t}cm':np.isclose(target,t,atol=.01) for t in (30,40,50)},
           'rising':speed<-.3,'sinking':speed>.3,'near_stationary':np.abs(speed)<=.3,
           'recent_direction_change':reversal,'no_recent_direction_change':~reversal}
    masks['near_target_stationary']=(np.abs(speed)<=.3)&(np.abs(data['policy'][ids,0])<=2)
    result['by_condition']={name:group(mask) for name,mask in masks.items() if mask.any()}
    return result


def score(pred,data,ids):
    error=(pred[:,:,0]-data['y'][ids,:,0]).abs()
    slow,close=hold_masks(data,ids)
    low=error[slow][:,[9,19,29]].mean() if slow.any() else error.mean()
    hold=error[close][:,[9,19,29]].mean() if close.any() else low
    return (.1*error.mean()+.2*error[:,-1].mean()+.35*low+.35*hold).item()


def hold_acceptance(pred,baseline,data,ids):
    """Validation only: low-speed horizons may not regress; cap global tradeoff."""
    e=(pred[:,:,0]-data['y'][ids,:,0]).abs()
    old=(baseline[:,:,0]-data['y'][ids,:,0]).abs()
    slow,close=hold_masks(data,ids)
    checks={'enough_slow_samples':int(slow.sum())>=20}
    for name,mask in (('slow',slow),('near_target_slow',close)):
        if int(mask.sum())<20:continue
        for tick in (9,19,29):
            checks[f'{name}_{tick+1}_ticks']=e[mask,tick].mean().item()<=old[mask,tick].mean().item()+1e-6
    for tick in (9,19,29):
        checks[f'global_{tick+1}_ticks']=e[:,tick].mean().item()<=old[:,tick].mean().item()*1.02+1e-6
    checks['strict_slow_3s_improvement']=bool(slow.any()) and e[slow,29].mean().item()<old[slow,29].mean().item()-1e-6
    return {'passed':all(checks.values()),'checks':checks,'slow_count':int(slow.sum()),'near_target_slow_count':int(close.sum())}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--epochs',type=int,default=100)
    parser.add_argument('--validation-csv',type=Path,required=True)
    parser.add_argument('--test-csv',type=Path,required=True)
    args=parser.parse_args()
    if args.epochs<1:parser.error('epochs must be positive')
    torch.set_num_threads(1)
    out=ROOT/'artifacts/hybrid_mpc_v1'
    manifest=json.loads((out/'training_report.json').read_text(encoding='utf-8'))
    files=[Path(s['path']) for s in manifest['sources']]
    files=[p if p.is_absolute() else ROOT/p for p in files]
    if any(digest(p)!=s['sha256'] for p,s in zip(files,manifest['sources'])):
        raise ValueError('Source CSV changed since the previous training run')
    data=run_separated_data(files,args.validation_csv,args.test_csv)
    train=np.flatnonzero(data['split']==0);val=np.flatnonzero(data['split']==1);test=np.flatnonzero(data['split']==2)
    if min(len(train),len(val),len(test))<20:raise ValueError('Insufficient split windows')
    protected=[out/name for name in ('student.pt','student_windows.pt','teacher_labels.csv','proposal_student.espdl',
                                    'ProposalStudentConfig.h','espdl_validation.json','deployment_report.json')]
    protected.append(ROOT/'firmware/build-learning/squid_robot_a_recorder.bin')
    hashes={str(p):digest(p) for p in protected}
    checkpoint_path=out/'dynamics_ensemble.pt';old_hash=digest(checkpoint_path)
    checkpoint=torch.load(checkpoint_path,map_location='cpu',weights_only=False)
    classes={'Hybrid':Hybrid,'HistoryHybrid':HistoryHybrid,'SensorHistoryHybrid':SensorHistoryHybrid}
    old_class=classes.get(checkpoint.get('model_class','Hybrid'))
    if old_class is None:raise ValueError('Unsupported previous dynamics model class')
    old=[]
    for state in checkpoint['states']:
        model=old_class().eval();model.load_state_dict(state);old.append(model)
    old_val=predict(old,data,val)
    report={'schema_version':SCHEMA_VERSION,'task':'dynamics prediction only','backup_created':False,
            'sources':data['provenance'],'split_counts':np.bincount(data['split'],minlength=3).tolist(),
            'protocol':'Disjoint entire recording runs for train/validation/test, same fixed new evaluation windows for previous and candidate models. Retain original 3s window continuity exclusions. Validation-only epoch/ensemble selection; test after selection.',
            'previous_checkpoint_sha256':old_hash,'previous_validation':metrics(old_val,data,val),
            'epochs':args.epochs,'member_training':[],'objective':'causal_sensor_history_v4',
            'new_input':'Past 3s depth centered on current measured depth and past speed, causal zero-order hold; no new CSV columns',
            'limitations':['Observed filtered depth predictions, not raw physical depth ground truth',
                           'Future recorded PWM commands are known during prediction evaluation; not an open-loop control performance test',
                           'Evaluation runs were previously inspected during controller analyses, so this is retrospective run-separated evaluation, not prospective real-water acceptance',
                           'Not directly comparable to the previous 2.31cm result: evaluation windows changed',
                           'MPC/student artifacts have NOT been regenerated and must not be paired with new dynamics without relabeling']}
    print(json.dumps({'baseline_validation':report['previous_validation'],'splits':report['split_counts']}),flush=True)
    report['stationary_samples_by_split_target']={str(split):{str(target):int(np.count_nonzero((data['split']==split)&
        np.isclose(data['x'][:,0].numpy()+40-data['policy'][:,0],target,atol=.01)&(data['x'][:,1].abs().numpy()<=.3)))
        for target in (30,40,50)} for split in (0,1,2)}
    candidates=[]
    for index,seed in enumerate((17,29,43)):
        torch.manual_seed(seed);rng=np.random.default_rng(seed)
        model=SensorHistoryHybrid()
        # Fine tune the whole previous predictor, including learned residuals.
        shared={k:v for k,v in old[index].state_dict().items() if k in model.state_dict() and v.shape==model.state_dict()[k].shape}
        model.load_state_dict(shared,strict=False)
        optimizer=torch.optim.Adam(model.parameters(),lr=.0002,weight_decay=1e-4)
        reference=predict([old[index]],data,val)
        best=score(reference,data,val);best_epoch=0;saved=copy.deepcopy(model.state_dict())
        for epoch in range(args.epochs):
            model.train()
            for ids in np.array_split(rng.permutation(train),max(1,len(train)//128)):
                pred=model_forward(model,data,ids)
                slow,close=hold_masks(data,ids)
                weights=1+3*slow.float()+4*close.float()
                loss=prediction_loss(pred,data['y'][ids],weights)
                optimizer.zero_grad();loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),5);optimizer.step()
            model.eval()
            prediction=predict([model],data,val)
            value=score(prediction,data,val)
            gate=hold_acceptance(prediction,reference,data,val)
            if gate['passed'] and value<best:best=value;saved=copy.deepcopy(model.state_dict());best_epoch=epoch+1
            if (epoch+1)%10==0:print('DYNAMICS_V2',seed,epoch+1,'validation_score',round(value,5),'best_epoch',best_epoch,flush=True)
        model.load_state_dict(saved);model.eval();candidates.append(model)
        report['member_training'].append({'seed':seed,'best_epoch':best_epoch,'validation_score':best})
    # Validation-only selection includes the unchanged baseline as fallback.
    new_val=predict(candidates,data,val)
    weights=(0.,1.)
    gate=hold_acceptance(new_val,old_val,data,val)
    chosen=1. if gate['passed'] and score(new_val,data,val)<score(old_val,data,val) else 0.
    selected_val=old_val*(1-chosen)+new_val*chosen
    report['validation_selected_new_weight']=chosen
    report['candidate_validation']=metrics(new_val,data,val)
    report['validation_hold_acceptance']=gate
    report['selected_validation']=metrics(selected_val,data,val)
    # Freeze choice above, then inspect final test. Never reselect from test.
    old_test=predict(old,data,test);new_test=predict(candidates,data,test)
    report['previous_test']=metrics(old_test,data,test)
    report['candidate_test']=metrics(new_test,data,test)
    report['selected_test']=metrics(old_test*(1-chosen)+new_test*chosen,data,test)
    report['test_hold_acceptance_diagnostic']=hold_acceptance(old_test*(1-chosen)+new_test*chosen,old_test,data,test)
    output={'schema_version':SCHEMA_VERSION,'model_class':'SensorHistoryHybrid','dt':DT,'history':HISTORY,'horizon':HORIZON,
            'input_dim':DYNAMICS_DIM,'states':[m.state_dict() for m in candidates],
            'ensemble_new_weight':chosen,
            'requires_mpc_relabeling':True,'training_task':'causal_sensor_history_v4','sensor_history_shape':[30,2],
            'sensor_history_columns':['past_depth_minus_current_cm','past_speed_cm_s']}
    if chosen==0:
        report['checkpoint_replaced']=False
    else:
        torch.save(output,checkpoint_path)
        report['checkpoint_replaced']=True
    report['checkpoint_sha256']=digest(checkpoint_path)
    report['protected_files_unchanged']=all(digest(Path(p))==h for p,h in hashes.items())
    if not report['protected_files_unchanged']:raise RuntimeError('Protected MPC/student/firmware artifact changed')
    report['protected_file_sha256']=hashes
    report['prediction_improved_on_test']=report['selected_test']['3s']['mae_cm']<report['previous_test']['3s']['mae_cm']
    (out/'dynamics_prediction_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('validation_selected_new_weight','previous_test','selected_test','prediction_improved_on_test','protected_files_unchanged')},indent=2),flush=True)


if __name__=='__main__':main()
