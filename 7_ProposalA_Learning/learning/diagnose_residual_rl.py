"""Matched diagnostic replay, never used for policy selection or robot IO."""
import json
import hashlib
import numpy as np
import torch
import residual_rl as r
import pipeline as p


@torch.no_grad()
def main():
    torch.set_num_threads(1)
    out=p.ROOT/'artifacts/residual_rl_v1'
    report=json.loads((out/'training_report.json').read_text())
    data,members,student,stats,frozen=r.load_frozen()
    policies={'baseline':None}
    paths={'previously_built_candidate':out/'actor.pt','new_candidate':out/report['policy_artifact']}
    hashes={}
    for name,path in paths.items():
        ck=torch.load(path,weights_only=False,map_location='cpu')
        if ck['frozen_sha256']!=frozen:
            if name=='previously_built_candidate':continue
            raise ValueError('Current candidate does not match the frozen baseline')
        actor=r.Actor().eval();actor.load_state_dict(ck['state']);policies[name]=actor
        hashes[name]=hashlib.sha256(path.read_bytes()).hexdigest()
    ids=np.flatnonzero(data['split']==2)[::4]
    result={'note':'Diagnostic test replay after selection; not a new independent holdout or model-selection input',
            'policy_sha256':hashes,'frozen_models_sha256':frozen,'by_horizon':{}}
    for steps in (40,120):
        result['by_horizon'][str(steps/10)]={name:r.evaluate(data,ids,members,student,stats,actor,steps) for name,actor in policies.items()}
    # Record a matched validation trace for the previous candidate's worst
    # shallow hold excursion, to expose action timing instead of just averages.
    ids=np.flatnonzero(data['split']==1)[::4]
    ids=ids[(abs(data['policy'][ids,0])<=2)&(abs(data['policy'][ids,1])<=.3)]
    traces={}
    for k,plant in enumerate(members):
        for name,actor in policies.items():
            env=r.Env(data,plant,student,stats,ids);values=[]
            for _ in range(40):
                obs=env.observation();env.step(actor(obs) if actor else torch.zeros(len(ids)))
                values.append(torch.stack((env.state[:,0]-env.target,env.state[:,1],env.nn,env.prevrl,env.previous),1))
            traces[k,name]=torch.stack(values)
    trace_policy='previously_built_candidate' if 'previously_built_candidate' in policies else 'new_candidate'
    k=max(range(len(members)),key=lambda k:float((-traces[k,trace_policy][:,:,0]).max()))
    pos=torch.nonzero(-traces[k,trace_policy][:,:,0]==(-traces[k,trace_policy][:,:,0]).max())[0]
    i=int(pos[1]);result['validation_trace']={'reference_policy':trace_policy,'source_session':int(data['session'][ids[i]]),'elapsed_s':float(data['times'][ids[i]]),'ensemble_member':k,
        'fields':['depth_minus_target_cm','speed_cm_s','nn_cm_s2','rl_cm_s2','actual_signed_command'],
        'samples':[{'seconds':(t+1)/10,**{name:traces[k,name][t,i].tolist() for name in policies}} for t in range(40)]}
    (out/'diagnostics.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({h:{name:v['hold'] for name,v in values.items()} for h,values in result['by_horizon'].items()}),flush=True)


if __name__=='__main__':main()
