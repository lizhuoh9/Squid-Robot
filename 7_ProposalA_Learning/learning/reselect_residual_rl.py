"""Select retained PPO policies by validation absolute maximum error only.

Does not run PPO, use test data for selection, or overwrite deployment actor
when acceptance fails. Useful when correcting a model-selection defect.
"""
import copy
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
import pipeline as p
import residual_rl as r


@torch.no_grad()
def main():
    torch.set_num_threads(1);out=p.ROOT/'artifacts/residual_rl_v1'
    report=json.loads((out/'training_report.json').read_text())
    data,members,student,stats,frozen=r.load_frozen();val=np.flatnonzero(data['split']==1)[::4]
    previous=torch.load(out/report['policy_artifact'],weights_only=False,map_location='cpu')
    final=torch.load(out/'ppo_training_state.pt',weights_only=False,map_location='cpu')
    actor=r.Actor();actor.initialize_feedback(*stats)
    choices=[('feedback initialization',0,copy.deepcopy(actor.state_dict())),
             ('previous validation selection',previous['selected_update'],previous['state']),
             ('final PPO training state',final['updates'],{k[len('actor.'):]:v for k,v in final['agent'].items() if k.startswith('actor.')})]
    baseline=r.evaluate(data,val,members,student,stats,None);evaluations=[];best=None
    for name,update,state in choices:
        actor.load_state_dict(state);evaluation=r.evaluate(data,val,members,student,stats,actor)
        rank=r.validation_rank(evaluation,baseline);evaluations.append({'name':name,'update':update,'validation':evaluation,'checks':r.acceptance(evaluation,baseline)})
        if best is None or rank>best[0]:best=(rank,name,update,copy.deepcopy(state),evaluation)
    _,name,update,state,validation=best;actor.load_state_dict(state)
    test=np.flatnonzero(data['split']==2)[::4];reference=r.evaluate(data,test,members,student,stats,None);candidate=r.evaluate(data,test,members,student,stats,actor)
    vchecks=r.acceptance(validation,baseline);checks=r.acceptance(candidate,reference);eligible=all(vchecks.values()) and all(checks.values())
    previous.update(state=state,selected_update=update,eligible=eligible)
    artifact='actor.pt' if eligible else 'candidate.pt';torch.save(previous,out/artifact)
    report.update(selected_update=update,selected_policy_origin=name,policy_artifact=artifact,export_allowed=eligible,acceptance_passed=eligible,
        selection='validation absolute maximum error in cm, then two-sided peak sum, then MAE; acceptance checks have first priority',
        validation_baseline=baseline,validation_candidate=validation,validation_acceptance_checks=vchecks,test_baseline=reference,test_candidate=candidate,acceptance_checks=checks,
        retained_policy_reselection=evaluations,firmware_connected=False,espdl_exported=False,uploaded=False)
    report['limitations'].append('Retained-policy reselection uses validation only; test has been inspected repeatedly during development and is not a pristine independent holdout')
    for path,digest in frozen.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:raise RuntimeError('Frozen baseline changed')
    (out/'training_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'selected':name,'update':update,'eligible':eligible,'baseline_hold':reference['hold'],'new_hold':candidate['hold']}),flush=True)


if __name__=='__main__':main()
