"""Offline PPO residual on frozen Proposal A + frozen sensor-history student.

No robot IO, firmware editing, ESP-DL export or existing-model replacement.
The separate actor outputs +/-0.8; the NN+RL sum is NOT clipped.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.distributions import Normal

import pipeline as p
from fit_dynamics_only import run_separated_data

LIMIT = .8
OBS_DIM = 128


def shallow_stage_cost(error, speed):
    shallow=(-error).clamp_min(0)
    projected_shallow=(-error-1.5*speed).clamp_min(0)
    return error.square()+12*shallow.square()+.8*speed.square()+4*projected_shallow.square()


def acceptance(candidate, reference):
    checks={k:candidate['all'][k]<=reference['all'][k]+1e-6 for k in
            ('mae_cm','p95_abs_cm','shallow_max_cm','deep_max_cm','direction_switches_per_4s')}
    checks['shallow_mean_strictly_smaller']=candidate['all']['shallow_mean_cm']<reference['all']['shallow_mean_cm']-1e-4
    for key in ('mae_cm','shallow_mean_cm','deep_max_cm','direction_switches_per_4s'):
        checks['hold_'+key+'_not_worse']=candidate['hold'][key]<=reference['hold'][key]+1e-6
    # Overall maximum may be set by an initially far-shallow state outside
    # the action guard. Require actual peak reduction in the fixed hold cohort.
    checks['hold_shallow_max_strictly_smaller']=candidate['hold']['shallow_max_cm']<reference['hold']['shallow_max_cm']-.01
    return checks


def validation_rank(candidate, reference):
    checks=acceptance(candidate,reference)
    shallow_score=(candidate['all']['shallow_mean_cm']+candidate['hold']['shallow_mean_cm']
                   +.5*candidate['hold']['shallow_max_cm'])
    return (all(checks.values()),-shallow_score,candidate['all']['reward'])


def combine(nn_correction, rl_correction):
    return nn_correction.clamp(-.5,.5) + rl_correction.clamp(-LIMIT,LIMIT)


class Actor(nn.Module):
    def __init__(self):
        super().__init__()
        self.net=nn.Sequential(nn.Linear(OBS_DIM,32),nn.ReLU(),nn.Linear(32,16),nn.ReLU(),nn.Linear(16,1))
        nn.init.zeros_(self.net[-1].weight);nn.init.zeros_(self.net[-1].bias)

    def forward(self,obs):
        return LIMIT*torch.tanh(self.net(obs).squeeze(-1))


class Agent(nn.Module):
    def __init__(self):
        super().__init__();self.actor=Actor()
        self.critic=nn.Sequential(nn.Linear(OBS_DIM,64),nn.Tanh(),nn.Linear(64,32),nn.Tanh(),nn.Linear(32,1))
        self.logstd=nn.Parameter(torch.tensor(-1.2))

    def distribution(self,obs):
        return Normal(self.actor.net(obs).squeeze(-1),self.logstd.clamp(-3,0).exp())


class Env:
    """100ms control / 50ms physics, same pulse phase and frozen base policy.

    Matches current A gains, trim update, residual guard and bias attenuation.
    200ms actuator reversal lock remains an unverified simulation assumption.
    """
    def __init__(self,data,plant,student,stats,ids,factors=None):
        self.plant=plant;self.student=student;self.mean,self.std=stats;self.factors=factors
        self.f=torch.tensor(data['policy'][ids],dtype=torch.float32)
        self.z0=data['x'][ids,0]+40;self.target=self.z0-self.f[:,0]
        x=torch.cat((self.z0[:,None]-40,self.f[:,1:3],self.f[:,4:65]),1)
        sensor=self.f[:,65:].reshape(-1,30,2)
        self.state=plant.initial(x,sensor);self.history=x[:,p.DYNAMICS_COMMANDS].clone()
        self.fhistory=x[:,p.DYNAMICS_FORWARD_HISTORY].clone();self.forward=self.f[:,p.POLICY_FORWARD]
        self.pastz=sensor[:,:,0]+self.z0[:,None];self.pastv=sensor[:,:,1].clone()
        self.integral=((self.f[:,3]-1.1-.03*(self.z0-40))/p.CONTROL_KI).clamp(-p.CONTROL_I_LIMIT/p.CONTROL_KI,p.CONTROL_I_LIMIT/p.CONTROL_KI)
        zero=torch.zeros(len(ids));self.last=torch.sign(self.history[:,-1]);self.previous=self.history[:,-1].clone()
        self.mod_start=-torch.tensor(p.phase_for_rows(data,ids),dtype=torch.float32)
        self.on_until=zero.clone();self.active=torch.sign(self.previous);self.lock_until=zero.clone()
        self.prevrl=zero.clone();self.tick=0;self.switches=zero.clone();self.energy=zero.clone()
        self.shallow_peak=(-(self.state[:,0]-self.target)).clamp_min(0)

    def observation(self):
        z,v,b=self.state.unbind(1)
        self.v=self.f[:,1] if self.tick==0 else v
        accel=self.f[:,2] if self.tick==0 else b-self.plant.constants()[1]*v*v.abs()
        e=z-self.target;self.bias=1.1+.03*(z-40)+p.CONTROL_KI*self.integral
        past=torch.stack((self.pastz-z[:,None],self.pastv),2).flatten(1)
        fi=torch.cat((torch.stack((e,self.v,accel,self.bias),1),self.history,self.forward[:,None],self.fhistory,past),1)
        norm=(fi-self.mean)/self.std
        self.valid=(e.abs()<=12)&(self.v.abs()<=6)&(norm.abs().amax(1)<=6)&torch.isfinite(norm).all(1)
        nnvalue=self.student(norm).squeeze(1).clamp(-.5,.5)
        self.nn=p.guard_ascent_correction(e,self.v,torch.where(self.valid,nnvalue,torch.zeros_like(nnvalue)))
        return torch.cat((norm.clamp(-6,6),self.nn[:,None]/.5,self.prevrl[:,None]/LIMIT,z[:,None]/40),1)

    def step(self,action):
        action=torch.where(self.valid,action.clamp(-LIMIT,LIMIT),torch.zeros_like(action))
        total=combine(self.nn,action);z=self.state[:,0];e=z-self.target
        effort=-p.CONTROL_KP*e-p.CONTROL_KD*self.v-p.effective_control_bias(e,self.v,self.bias)+total
        req=torch.where(effort>0,effort/.75,effort/4.5);saturated=req.abs()>=1;duty=req.clamp(-1,1)
        ilim=p.CONTROL_I_LIMIT/p.CONTROL_KI
        self.integral=torch.where(saturated,self.integral,(self.integral+e*.1).clamp(-ilim,ilim))
        duty=torch.where((self.last!=0)&(torch.sign(duty)!=self.last)&(duty.abs()<.05),torch.zeros_like(duty),duty)
        self.last=torch.where(duty!=0,torch.sign(duty),self.last)
        sampled_z=z.clone();sampled_v=self.v.clone();average=torch.zeros_like(duty);cost=torch.zeros_like(duty)
        for _ in range(2):
            mag=duty.abs();window=(mag>0)&(mag<80/255)&(self.tick-self.mod_start>=20)
            self.on_until=torch.where(window,self.tick+torch.floor(1000*mag/(80/255)+.5)/50,self.on_until)
            self.mod_start=torch.where(window,torch.full_like(mag,self.tick),self.mod_start)
            applied=torch.where(mag>=80/255,mag,torch.where(self.tick<self.on_until,torch.full_like(mag,80/255),torch.zeros_like(mag)))
            command=applied*torch.sign(duty);direction=torch.sign(command)
            change=(direction!=0)&(self.active!=0)&(direction!=self.active)
            self.switches+=change.float();self.lock_until=torch.where(change,torch.full_like(mag,self.tick+4),self.lock_until)
            self.active=torch.where(direction!=0,direction,self.active)
            command=torch.where(self.tick<self.lock_until,torch.zeros_like(command),command)
            self.state=self.plant.step(self.state,command,self.history,dt=.05,factors=self.factors,forward_active=self.forward,forward_history=self.fhistory)
            error=self.state[:,0]-self.target;speed=self.state[:,1]
            cost+=.05*(shallow_stage_cost(error,speed)+.1*action.square())
            peak=torch.maximum(self.shallow_peak,(-error).clamp_min(0))
            cost+=3*(peak.square()-self.shallow_peak.square());self.shallow_peak=peak
            cost+=.15*change.float()+.02*(command-self.previous).abs()
            self.energy+=command.abs()*.05;self.previous=command;average+=command*.5;self.tick+=1
        cost+=.03*(action-self.prevrl).square();self.prevrl=action
        self.pastz=torch.cat((self.pastz[:,1:],sampled_z[:,None]),1)
        self.pastv=torch.cat((self.pastv[:,1:],sampled_v[:,None]),1)
        self.history=torch.cat((self.history[:,1:],average[:,None]),1)
        self.fhistory=torch.cat((self.fhistory[:,1:],self.forward[:,None]),1)
        return -cost


@torch.no_grad()
def evaluate(data,ids,members,student,stats,actor):
    records=[]
    for plant in members:
        env=Env(data,plant,student,stats,ids);errors=[];actions=[];reward=torch.zeros(len(ids))
        for _ in range(40):
            obs=env.observation();action=actor(obs) if actor is not None else torch.zeros(len(ids))
            reward+=env.step(action);errors.append(env.state[:,0]-env.target);actions.append(env.prevrl.clone())
        records.append((torch.stack(errors),env.switches,env.energy,torch.stack(actions),reward))
    err=torch.stack([r[0] for r in records]);actions=torch.stack([r[3] for r in records])
    def metrics(mask):
        x=err[:,:,mask];ac=actions[:,:,mask]
        return {'windows':int(mask.sum()),'mae_cm':x.abs().mean().item(),'rmse_cm':x.square().mean().sqrt().item(),
                'p95_abs_cm':torch.quantile(x.abs().flatten(),.95).item(),'max_abs_cm':x.abs().max().item(),
                'shallow_mean_cm':(-x).clamp_min(0).mean().item(),'shallow_max_cm':(-x).clamp_min(0).max().item(),
                'deep_mean_cm':x.clamp_min(0).mean().item(),'deep_max_cm':x.clamp_min(0).max().item(),
                'direction_switches_per_4s':torch.stack([r[1] for r in records])[:,mask].mean().item(),
                'pump_duty_seconds':torch.stack([r[2] for r in records])[:,mask].mean().item(),
                'rl_mean_cm_s2':ac.mean().item(),'rl_abs_mean_cm_s2':ac.abs().mean().item(),'rl_abs_max_cm_s2':ac.abs().max().item(),
                'reward':torch.stack([r[4] for r in records])[:,mask].mean().item()}
    result={'all':metrics(np.ones(len(ids),bool))}
    close=(np.abs(data['policy'][ids,0])<=2)&(np.abs(data['policy'][ids,1])<=.3)
    if close.any():result['hold']=metrics(close)
    targets=(data['x'][ids,0]+40).numpy()-data['policy'][ids,0]
    result['by_target']={str(t):metrics(np.isclose(targets,t,atol=.01)) for t in (30,40,50) if np.isclose(targets,t,atol=.01).any()}
    return result


def load_frozen():
    out=p.ROOT/'artifacts/hybrid_mpc_v1'
    report=json.loads((out/'dynamics_prediction_report.json').read_text(encoding='utf-8'));sources=report['sources']
    for s in sources:
        if hashlib.sha256(Path(s['path']).read_bytes()).hexdigest()!=s['sha256']:raise ValueError('CSV changed')
    data=run_separated_data([s['path'] for s in sources if s['role']=='train'],next(s['path'] for s in sources if s['role']=='validation'),next(s['path'] for s in sources if s['role']=='test'))
    data['policy']=np.c_[data['policy'],data['state_history'].numpy().reshape(-1,60)]
    ck=torch.load(out/'dynamics_ensemble.pt',weights_only=False,map_location='cpu');members=[]
    for state in ck['states']:
        model=p.SensorHistoryHybrid().eval();model.load_state_dict(state);model.requires_grad_(False);members.append(model)
    ck=torch.load(out/'student.pt',weights_only=False,map_location='cpu');student=p.Student(125).eval();student.load_state_dict(ck['state']);student.requires_grad_(False)
    stats=(torch.tensor(ck['input_mean']),torch.tensor(ck['input_std']))
    frozen={str(out/n):hashlib.sha256((out/n).read_bytes()).hexdigest() for n in ('student.pt','dynamics_ensemble.pt','proposal_student.espdl','ProposalStudentConfig.h')}
    return data,members,student,stats,frozen


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--updates',type=int,default=240);parser.add_argument('--envs',type=int,default=64);args=parser.parse_args()
    if args.updates<1 or args.envs<1:parser.error('updates and envs must be positive')
    torch.set_num_threads(1);torch.manual_seed(2701);rng=np.random.default_rng(2701)
    data,members,student,stats,frozen=load_frozen();train=np.flatnonzero(data['split']==0);val=np.flatnonzero(data['split']==1)[::4];test=np.flatnonzero(data['split']==2)[::4]
    agent=Agent();optimizer=torch.optim.Adam(agent.parameters(),lr=3e-4)
    baseline=evaluate(data,val,members,student,stats,None);best_score=None;best=None;history=[]
    near=train[(np.abs(data['policy'][train,0])<=4)&(np.abs(data['policy'][train,1])<=6)]
    if not len(near):raise ValueError('No near-target training seeds')
    for update in range(1,args.updates+1):
        near_count=int(.75*args.envs)
        ids=np.r_[rng.choice(near,near_count,replace=True),rng.choice(train,args.envs-near_count,replace=True)]
        plant=members[(update-1)%len(members)];factors=p.DOMAINS[(update-1)%len(p.DOMAINS)]
        obsbuf=[];latentbuf=[];lpbuf=[];vbuf=[];rewards=[]
        with torch.no_grad():
            env=Env(data,plant,student,stats,ids,factors)
            for tick in range(40):
                obs=env.observation();dist=agent.distribution(obs);latent=dist.sample();value=agent.critic(obs).squeeze(1)
                reward=env.step(LIMIT*torch.tanh(latent));obsbuf.append(obs);latentbuf.append(latent);lpbuf.append(dist.log_prob(latent));vbuf.append(value);rewards.append(reward)
            # Fixed reward scaling keeps value gradients from overwhelming
            # normalized policy gradients; physical evaluation stays unscaled.
            reward=torch.stack(rewards)/50;values=torch.stack(vbuf);adv=torch.zeros_like(reward);gae=torch.zeros(args.envs)
            for t in reversed(range(40)):
                # Fixed 4s model horizon is an episodic terminal, not a measured terminal.
                nxt=values[t+1] if t<39 else torch.zeros_like(gae)
                delta=reward[t]+.99*nxt-values[t];gae=delta+.99*.95*gae;adv[t]=gae
            returns=adv+values;adv=(adv-adv.mean())/(adv.std()+1e-6)
        obs=torch.stack(obsbuf).reshape(-1,OBS_DIM);latent=torch.stack(latentbuf).flatten();oldlp=torch.stack(lpbuf).flatten();adv=adv.flatten();returns=returns.flatten()
        for _ in range(4):
            for idx in torch.randperm(len(obs)).split(256):
                dist=agent.distribution(obs[idx]);ratio=(dist.log_prob(latent[idx])-oldlp[idx]).exp()
                actorloss=-torch.minimum(ratio*adv[idx],ratio.clamp(.8,1.2)*adv[idx]).mean()
                value=agent.critic(obs[idx]).squeeze(1);criticloss=.5*(value-returns[idx]).square().mean()
                loss=actorloss+criticloss-.002*dist.entropy().mean()
                if not torch.isfinite(loss):raise RuntimeError('Nonfinite PPO loss')
                optimizer.zero_grad();loss.backward();torch.nn.utils.clip_grad_norm_(agent.parameters(),.5);optimizer.step()
        if update%20==0 or update==args.updates:
            evaluation=evaluate(data,val,members,student,stats,agent.actor)
            checks=acceptance(evaluation,baseline);rank=validation_rank(evaluation,baseline)
            history.append({'update':update,'validation':evaluation,'acceptance_checks':checks})
            if best_score is None or rank>best_score:best_score=rank;best=copy.deepcopy(agent.actor.state_dict());selected=update
            print(json.dumps({'update':update,'validation_passed':all(checks.values()),'mae_cm':evaluation['all']['mae_cm'],
                              'shallow_mean_cm':evaluation['all']['shallow_mean_cm'],'hold_shallow_max_cm':evaluation['hold']['shallow_max_cm']}),flush=True)
    # Keep the final optimizer paired with its own final actor, independently
    # of the earlier actor selected for inference by validation.
    final_training_state=copy.deepcopy(agent.state_dict())
    agent.actor.load_state_dict(best);validation=evaluate(data,val,members,student,stats,agent.actor)
    reference=evaluate(data,test,members,student,stats,None);candidate=evaluate(data,test,members,student,stats,agent.actor)
    checks=acceptance(candidate,reference);validation_checks=acceptance(validation,baseline)
    for path,digest in frozen.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:raise RuntimeError('Frozen base changed')
    out=p.ROOT/'artifacts/residual_rl_v1';out.mkdir(parents=True,exist_ok=True)
    checkpoint={'state':agent.actor.state_dict(),'input_dim':OBS_DIM,'limit_cm_s2':LIMIT,'combined_correction_clipped':False,
                'observation':f'125 baseline-normalized causal features clipped +/-6, applied NN/.5, previous RL/{LIMIT}, depth/40',
                'baseline_input_mean':stats[0].tolist(),'baseline_input_std':stats[1].tolist(),'frozen_sha256':frozen,'selected_update':selected,
                'eligible':all(checks.values()) and all(validation_checks.values())}
    torch.save(checkpoint,out/'actor.pt')
    torch.save({'agent':final_training_state,'optimizer':optimizer.state_dict(),'updates':args.updates,
                'note':'Final optimization state, not necessarily the validation-selected inference actor; RNG states are not saved'},out/'ppo_training_state.pt')
    report={'algorithm':'PPO clipped policy gradient with GAE, tanh Gaussian actions','seed':2701,'updates':args.updates,'parallel_envs':args.envs,
            'simulated_control_steps':args.updates*args.envs*40,'episode_s':4,'control_ms':100,'physics_ms':50,'domains':p.DOMAINS,
            'rl_limit_cm_s2':LIMIT,'nn_limit_cm_s2':.5,'combined_correction_clipped':False,'actuator_duty_limit':[-1,1],
            'selected_update':selected,'selection':'validation shallow acceptance first, then shallow error/peak score; test once after selection. If no policy passes, retain failed research candidate and mark ineligible.',
            'training_seed_sampling':'75% near-target abs(error)<=4cm and abs(speed)<=6cm/s, 25% all train; no validation/test seeds',
            'reward_training_scale':.02,'sources':data['provenance'],'frozen_sha256':frozen,
            'validation_baseline':baseline,'validation_candidate':validation,'test_baseline':reference,'test_candidate':candidate,'acceptance_checks':checks,
            'validation_acceptance_checks':validation_checks,'acceptance_passed':all(checks.values()) and all(validation_checks.values()),'training_history':history,'firmware_connected':False,'espdl_exported':False,'uploaded':False,
            'limitations':['Model-based simulation only, not evidence of water performance','4s episodes may miss longer oscillations; repeated source development limits independence',
                           'Forward state is held constant; no future recorded commands or sensors','200ms actuator lock is an unverified assumption; sensor delay is not explicitly reproduced',
                           'Dynamics frozen from older data; current controller changes are simulated, not refitted','No online learning, new robot data collection or firmware modification'],
            'reward':'negative sum dt*(e^2+12*relu(-e)^2+0.8*v^2+4*relu(-e-1.5*v)^2+0.1*rl^2)+3*increase(shallow_peak^2)+0.15*direction_switch+0.02*abs(delta_actual_command)+0.03*delta_rl^2 per control tick'}
    (out/'training_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'output':str(out),'acceptance_passed':report['acceptance_passed'],'baseline_mae':reference['all']['mae_cm'],'rl_mae':candidate['all']['mae_cm']}),flush=True)


if __name__=='__main__':main()
