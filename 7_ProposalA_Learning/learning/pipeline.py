"""Research-only hybrid dynamics, closed-loop MPC labels, and bounded student.

All outputs stay in this project. Never sends robot commands or enables a model.
Temporal splits embargo the entire past/future window. Recorded PWM is a command,
not measured actuator feedback. The 200ms actuator lock is a simulation assumption.
"""
import argparse
import copy
import csv
import hashlib
import itertools
import json
import os
from pathlib import Path
import tempfile

import numpy as np
import torch
from torch import nn

ROOT = Path(__file__).resolve().parents[1]
DT, HISTORY, HORIZON = .1, 30, 30
FEATURES = ["depth_error_cm", "speed_cm_s", "accel_cm_s2", "proposal_bias_cm_s2"] + [f"applied_command_{i:02d}" for i in range(HISTORY)]
FILES = [ROOT / "data" / f"capture-20261008T{s}Z.csv" for s in ("073354205522", "074026357113", "074725226418")]


def read_data(files=None):
    files = [Path(path) for path in (FILES if files is None else files)]
    if not files:
        raise ValueError("At least one capture CSV is required")
    samples, sessions, provenance = [], [], []
    split_bounds_by_session = []
    for session, path in enumerate(files):
        with path.open(encoding="utf-8-sig") as handle:
            rows = list(csv.DictReader(handle))
        if not rows:
            raise ValueError(f"Empty CSV: {path}")
        columns = set(rows[0])
        is_ram_capture = {"depth_valid", "depth_age_ms", "proposal_bias_cm_s2", "pwm_commanded", "direction_commanded"} <= columns
        is_pc_control = {"depth_cm", "depth_speed_cm_s", "depth_accel_cm_s2", "target_depth_cm", "pid_base", "buoyancy_pwm", "valve_e_on", "valve_f_on", "forward_active"} <= columns
        if not (is_ram_capture or is_pc_control):
            raise ValueError(f"Unsupported control CSV schema: {path}")
        times = np.array([float(r["elapsed_s"]) for r in rows])
        gaps = np.diff(times)
        if np.any(gaps <= 0):
            raise ValueError(f"Nonmonotonic timestamps: {path}")
        if is_ram_capture:
            if np.max(gaps) > .1:
                raise ValueError("Nonmonotonic/missing capture slots")
            if any(r["depth_valid"] != "1" or float(r["depth_age_ms"]) > 250 for r in rows):
                raise ValueError("Invalid/stale sensor samples: split segments before learning")
            raw = np.array([[float(r[k]) for k in ("depth_cm", "depth_speed_cm_s", "depth_accel_cm_s2", "proposal_bias_cm_s2")] for r in rows])
            target = np.array([float(r["target_depth_cm"]) if r["target_valid"] == "1" else 0 for r in rows])
            signed = np.array([float(r["pwm_commanded"]) / 255 * (1 if r["direction_commanded"] == "1" else -1 if r["direction_commanded"] == "2" else 0) for r in rows])
            max_gap_for_window = .100001
            target_counts = {}
            for row in rows:
                if row["target_valid"] == "1":
                    key = str(float(row["target_depth_cm"]))
                    target_counts[key] = target_counts.get(key, 0) + 1
            fresh_count = sum(r["depth_fresh"] == "1" for r in rows)
            data_format = "ram_capture"
        else:
            # PC console's existing 200ms CSV: E+F means rise (negative depth
            # direction); neither means sink when PWM is active. One-valve
            # states are ambiguous and are excluded instead of guessed.
            raw = np.array([[float(r[k]) for k in ("depth_cm", "depth_speed_cm_s", "depth_accel_cm_s2", "pid_base")] for r in rows])
            # The PC logger leaves the target blank before a depth command is
            # issued. Treat those rows as invalid samples instead of aborting
            # the full-session training run.
            target = np.array([float(r["target_depth_cm"]) if r["target_depth_cm"].strip() else np.nan for r in rows])
            pwm = np.array([float(r["buoyancy_pwm"]) for r in rows])
            valve_e = np.array([int(r["valve_e_on"]) for r in rows])
            valve_f = np.array([int(r["valve_f_on"]) for r in rows])
            forward = np.array([int(r["forward_active"]) for r in rows])
            ambiguous = valve_e != valve_f
            if np.any((valve_e > 1) | (valve_f > 1) | (forward > 1) | (pwm < 0) | (pwm > 255)):
                raise ValueError(f"Invalid actuator fields in PC CSV: {path}")
            signed_direction = np.where(valve_e == 1, -1.0, 1.0)
            signed = (pwm / 255.0) * signed_direction
            signed[pwm == 0] = 0.0
            signed[ambiguous | (forward == 1)] = np.nan
            valid_rows = np.isfinite(raw).all(axis=1) & np.isfinite(target) & (target > 0) & np.isfinite(signed)
            if not valid_rows.all():
                # Keep elapsed-time gaps intact; invalid rows cannot be bridged
                # by the later window-continuity check.
                signed[~valid_rows] = np.nan
            max_gap_for_window = .300001
            target_counts = {}
            for value in target[valid_rows]:
                key = str(float(value))
                target_counts[key] = target_counts.get(key, 0) + 1
            fresh_count = None  # The PC CSV does not carry a sensor-fresh flag.
            data_format = "pc_control_200ms"
        duration = float(times[-1] - times[0])
        split_bounds = (times[0] + duration * .6, times[0] + duration * .8)
        split_bounds_by_session.append(split_bounds)
        grid = np.arange(times[0] + DT, times[-1] - DT, DT)
        # Causal zero-order hold, NOT interpolation using a future sensor value.
        ids = np.searchsorted(times, grid, side="right") - 1
        states, targets = raw[ids], target[ids]
        # Time-weighted actual command during each future integration interval.
        duty = []
        for t in grid:
            edges = np.r_[t, times[(times > t) & (times < t + DT)], t + DT]
            j = np.searchsorted(times, edges[:-1], side="right") - 1
            action = signed[j]
            duty.append(np.nan if not np.isfinite(action).all() else np.sum(np.diff(edges) * action) / DT)
        duty = np.array(duty)
        # Future depth labels are interpolated only as targets. Model inputs
        # remain causal zero-order holds from the last available PC sample.
        future_depth = np.interp(grid, times, raw[:, 0])
        if not np.isfinite(states).all() or not np.isfinite(duty).any():
            raise ValueError("Nonfinite state")
        for i in range(HISTORY, len(grid) - HORIZON):
            t = grid[i]
            # Whole 3s history AND 3s target sequence belong to only one block.
            first, second = split_bounds
            split = 0 if t + 3 <= first else 1 if t - 3 >= first and t + 3 <= second else 2 if t - 3 >= second and t + 3 <= times[-1] else -1
            window_start, window_end = t - HISTORY * DT, t + HORIZON * DT
            first_gap = max(0, int(np.searchsorted(times, window_start, side="right") - 1))
            last_gap = min(len(gaps), int(np.searchsorted(times, window_end, side="left")))
            continuous = last_gap <= first_gap or np.max(gaps[first_gap:last_gap]) <= max_gap_for_window
            window_duty = duty[i - HISTORY:i + HORIZON]
            if (split < 0 or not continuous or not np.isfinite(targets[i]) or not 0 < targets[i] <= 100
                    or not np.isfinite(window_duty).all()):
                continue
            z, v, a, bias = states[i]
            history = duty[i - HISTORY:i]
            samples.append((np.r_[z - 40, v, a, history], duty[i:i + HORIZON],
                            np.c_[future_depth[i + 1:i + HORIZON + 1], states[i + 1:i + HORIZON + 1, 1]],
                            np.r_[z - targets[i], v, a, bias, history], split, session, t))
        sessions.append((grid, states, targets, duty))
        provenance.append({"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "format": data_format,
                           "record_count": len(rows), "fresh_sensor_count": fresh_count,
                           "duration_s": duration, "median_interval_s": float(np.median(gaps)),
                           "max_interval_s": float(np.max(gaps)), "target_valid_row_counts_cm": target_counts})
    if not samples:
        raise ValueError("No eligible training windows: check target-valid duration and capture length")
    x, u, y, policy, split, session, t = zip(*samples)
    return dict(x=torch.tensor(np.array(x), dtype=torch.float32), u=torch.tensor(np.array(u), dtype=torch.float32),
                y=torch.tensor(np.array(y), dtype=torch.float32), policy=np.array(policy, dtype=np.float32),
                split=np.array(split), session=np.array(session), times=np.array(t), provenance=provenance,
                split_bounds=split_bounds_by_session)


class Hybrid(nn.Module):
    def __init__(self):
        super().__init__()
        self.latent = nn.Sequential(nn.Linear(33, 24), nn.Tanh(), nn.Linear(24, 1))
        self.residual = nn.Sequential(nn.Linear(5, 24), nn.Tanh(), nn.Linear(24, 1))
        self.physical = nn.Parameter(torch.tensor([0., -1., -1., 0., -.5, 0.]))
        nn.init.zeros_(self.latent[-1].weight); nn.init.zeros_(self.latent[-1].bias)
        nn.init.zeros_(self.residual[-1].weight); nn.init.zeros_(self.residual[-1].bias)

    def constants(self):
        s = torch.sigmoid(self.physical)
        return .5 + 3.5*s[0], .01 + .25*s[1], .3 + 5.7*s[2], .3 + 11.7*s[3], -1 + 4*s[4], -.03 + .09*s[5]

    def initial(self, x):
        _, drag, _, _, _, _ = self.constants()
        normalized = x / x.new_tensor([40., 6., 4.] + [1.]*30)
        z, v, a = x[:, 0] + 40, x[:, 1], x[:, 2]
        # Estimate hidden buoyancy from acceleration and the preceding 3s actions.
        b = a + drag*v*v.abs() + 2*torch.tanh(self.latent(normalized).squeeze(1))
        return torch.stack((z, v, b), 1)

    def step(self, state, u, history, dt=DT, factors=None):
        tau, drag, sink, rise, rest, alpha = self.constants()
        if factors is not None:
            tau, sink, rise, rest = tau*factors[0], sink*factors[1], rise*factors[2], rest+factors[3]
        z, v, b = state.unbind(1)
        correction = torch.tanh(self.residual(torch.stack(((z-40)/40, v/6, b/4, u, history.mean(1)), 1)).squeeze(1))
        eq = rest + alpha*(z-40) + torch.where(u >= 0, sink*u, rise*u)
        bnext = b + dt*((eq-b)/tau + correction)
        acceleration = b - drag*v*v.abs()
        vnext = v + dt*acceleration
        return torch.stack((z + dt*v + .5*dt*dt*acceleration, vnext, bnext), 1)

    def forward(self, x, actions):
        state, history, values = self.initial(x), x[:, 3:], []
        for u in actions.unbind(1):
            state = self.step(state, u, history)
            values.append(state[:, :2])
            history = torch.cat((history[:, 1:], u[:, None]), 1)
        return torch.stack(values, 1)


def fit_dynamics(data, out, epochs):
    train = np.flatnonzero(data["split"] == 0); val = np.flatnonzero(data["split"] == 1); test = np.flatnonzero(data["split"] == 2)
    members, summaries = [], []
    for seed in (17, 29, 43):
        torch.manual_seed(seed); rng = np.random.default_rng(seed)
        model = Hybrid(); opt = torch.optim.Adam(model.parameters(), lr=.002, weight_decay=1e-4)
        best, best_state, best_epoch = float("inf"), None, 0
        boot = rng.choice(train, len(train), replace=True)
        for epoch in range(epochs):
            for ids in np.array_split(rng.permutation(boot), max(1, len(boot)//128)):
                pred = model(data["x"][ids], data["u"][ids]); truth = data["y"][ids]
                loss = ((pred-truth)/torch.tensor([2., 1.])).square().mean()
                opt.zero_grad();loss.backward();nn.utils.clip_grad_norm_(model.parameters(), 5);opt.step()
            with torch.no_grad():
                value = (model(data["x"][val], data["u"][val])-data["y"][val]).square().mean().item()
            if value < best:best, best_state, best_epoch = value, copy.deepcopy(model.state_dict()), epoch+1
            if (epoch+1)%20 == 0:print("DYNAMICS",seed,epoch+1,"validation_loss",round(value,4),flush=True)
        model.load_state_dict(best_state);model.eval();members.append(model)
        summaries.append({"seed": seed, "best_epoch": best_epoch, "physical_tau_drag_sink_rise_rest_alpha": [p.item() for p in model.constants()]})
    torch.save({"states": [m.state_dict() for m in members], "dt": DT, "history": HISTORY, "horizon": HORIZON}, out/"dynamics_ensemble.pt")
    report = {}
    for name, ids in (("validation",val),("test",test)):
        with torch.no_grad():pred = torch.stack([m(data["x"][ids],data["u"][ids]) for m in members]).mean(0)
        times = torch.arange(1,HORIZON+1)*DT
        cv = data["x"][ids,0,None] + 40 + data["x"][ids,1,None]*times
        error = pred[:,:,0]-data["y"][ids,:,0];baseline=cv-data["y"][ids,:,0]
        report[name] = {"count":len(ids),"depth_trajectory_mae_cm":error.abs().mean().item(),"depth_3s_mae_cm":error[:,-1].abs().mean().item(),"constant_velocity_trajectory_mae_cm":baseline.abs().mean().item(),"constant_velocity_3s_mae_cm":baseline[:,-1].abs().mean().item()}
    report["members"] = summaries
    return members, report


@torch.no_grad()
def teacher_rollout(members, features, plans, factors=None, student=None, stats=None):
    """4s closed-loop A: 50ms motion, 100ms control, pulses + 200ms lock.

    PWM/valve state is simulated, not measured. Restored modulator phase and
    direction-lock state are unknown in recorded logs: use randomized phases.
    """
    n = len(features); candidates = len(plans); f = torch.tensor(features).repeat_interleave(candidates,0)
    p = torch.tensor(plans,dtype=torch.float32).repeat(n,1)
    # Teacher features carry absolute z as an additional final column.
    target = f[:,-1] - f[:,0]
    x = torch.cat((f[:,-1,None]-40,f[:,1:3],f[:,4:34]),1)
    states = [m.initial(x) for m in members]
    histories = [x[:,3:].clone() for _ in members]
    count=n*candidates
    costs=torch.zeros(count);integ=[((f[:,3]-1.1-.03*(f[:,-1]-40))/.01).clamp(-150,150).clone() for _ in members]
    duty=[torch.zeros(count) for _ in members];last=[torch.sign(x[:,-1]) for _ in members]
    # All competing action plans MUST start at the same modulator phase.
    phases=(torch.arange(n)%20).repeat_interleave(candidates);on_until=[torch.zeros(count) for _ in members]
    active_dir=[torch.sign(x[:,-1]) for _ in members];lock_until=[torch.zeros(count) for _ in members]
    previous=[x[:,-1].clone() for _ in members]
    averaged=[torch.zeros(count) for _ in members]
    maximum_uncertainty=torch.zeros(count)
    student_history=[f[:,4:34].clone() for _ in members]
    for tick in range(80):
        seconds=tick*.05;control_tick=tick%2==0
        for k,m in enumerate(members):
            z,v,b=states[k].unbind(1);e=z-target
            if control_tick:
                bias=1.1+.03*(z-40)+.01*integ[k]
                correction=p[:,min(3,int(seconds))]
                if student is not None:
                    accel=b-m.constants()[1]*v*v.abs()
                    fi=torch.cat((torch.stack((e,v,accel,bias),1),student_history[k]),1)
                    norm=(fi-stats[0])/stats[1]
                    correction=student(norm).squeeze(1).clamp(-.3,.3)
                    guard=(e.abs()<=12)&(v.abs()<=6)&(norm.abs().amax(1)<=6)
                    correction=torch.where(guard,correction,torch.zeros_like(correction))
                desired=-.09*e-.60*v;effort=desired-bias+correction
                req=torch.where(effort>0,effort/.75,effort/4.5)
                saturated=req.abs()>=1;req=req.clamp(-1,1)
                integ[k]=torch.where(saturated,integ[k],(integ[k]+e*.1).clamp(-150,150))
                req=torch.where((last[k]!=0)&(torch.sign(req)!=last[k])&(req.abs()<.05),torch.zeros_like(req),req)
                last[k]=torch.where(req!=0,torch.sign(req),last[k]);duty[k]=req
            mag=duty[k].abs();window=((tick+phases)%20)==0
            on_until[k]=torch.where(window,tick+20*mag/(80/255),on_until[k])
            applied=torch.where(mag>=80/255,mag,torch.where(tick<on_until[k],torch.full_like(mag,80/255),torch.zeros_like(mag)))
            command=applied*torch.sign(duty[k]);direction=torch.sign(command)
            change=(direction!=0)&(active_dir[k]!=0)&(direction!=active_dir[k])
            lock_until[k]=torch.where(change,torch.full_like(mag,seconds+.2),lock_until[k])
            active_dir[k]=torch.where(direction!=0,direction,active_dir[k])
            command=torch.where(seconds<lock_until[k],torch.zeros_like(command),command)
            states[k]=m.step(states[k],command,histories[k],dt=.05,factors=factors)
            costs+=((states[k][:,0]-target).square()+.8*states[k][:,1].square()+.2*p[:,min(3,int(seconds))].square()+.1*(command-previous[k]).abs())/len(members)*.05
            previous[k]=command
            averaged[k]+=command*.5
            if tick%2:
                histories[k]=torch.cat((histories[k][:,1:],averaged[k][:,None]),1)
                student_history[k]=torch.cat((student_history[k][:,1:],averaged[k][:,None]),1)
                averaged[k].zero_()
        uncertainty=torch.stack([state[:,0] for state in states]).std(0)
        maximum_uncertainty=torch.maximum(maximum_uncertainty,uncertainty)
        costs+=uncertainty.square()*.1
    return costs.reshape(n,candidates),maximum_uncertainty.reshape(n,candidates)


class Student(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc1=nn.Linear(34,32);self.fc2=nn.Linear(32,16);self.fc3=nn.Linear(16,1)
    def forward(self,x):return self.fc3(torch.relu(self.fc2(torch.relu(self.fc1(x)))))


def distill(data,members,out,epochs):
    # Enumerate 3^4 sequences; first move is repeated for 1s then replanned.
    plans=np.array(list(itertools.product((-.3,0.,.3),repeat=4)),dtype=np.float32)
    zero=np.flatnonzero(np.all(plans==0,axis=1))[0]
    labels=np.zeros(len(data["policy"]),np.float32);improvement=[]
    # Eleven parameter cases; observational data cannot establish real coverage.
    domains=[(1.,1.,1.,0.),(.8,1.,1.,0.),(1.2,1.,1.,0.),(1.,.8,1.,0.),(1.,1.2,1.,0.),(1.,1.,.8,0.),(1.,1.,1.2,0.),(1.,1.,1.,-.2),(1.,1.,1.,.2),(.8,.8,.8,-.2),(1.2,1.2,1.2,.2)]
    for begin in range(0,len(labels),32):
        ids=np.arange(begin,min(begin+32,len(labels)))
        features=np.c_[data["policy"][ids],data["x"][ids,0].numpy()+40]
        cost,uncertain=teacher_rollout(members,features,plans,factors=domains[(begin//32)%len(domains)])
        best=cost.argmin(1);gain=(cost[:,zero]-cost[torch.arange(len(ids)),best])/(cost[:,zero]+1e-6)
        chosen=plans[best.numpy(),0].copy()
        good=(gain.numpy()>.02)&(uncertain[torch.arange(len(ids)),best].numpy()<1.)&(np.abs(features[:,0])<=12)&(np.abs(features[:,1])<=6)
        chosen[~good]=0;labels[ids]=chosen;improvement.extend(gain.tolist())
        if begin%128==0:print("MPC LABELS",begin,"/",len(labels),flush=True)
    train=data["split"]==0;val=data["split"]==1;test=data["split"]==2
    mean=data["policy"][train].mean(0);std=data["policy"][train].std(0).clip(.05)
    x=torch.tensor((data["policy"]-mean)/std);y=torch.tensor(labels[:,None]);torch.manual_seed(71)
    model=Student();opt=torch.optim.Adam(model.parameters(),lr=.001,weight_decay=1e-4)
    best=float("inf");saved=None
    for epoch in range(epochs):
        for ids in np.array_split(np.random.default_rng(epoch).permutation(np.flatnonzero(train)),8):
            loss=(model(x[ids])-y[ids]).square().mean();opt.zero_grad();loss.backward();opt.step()
        with torch.no_grad():loss=(model(x[val])-y[val]).square().mean().item()
        if loss<best:best=loss;saved=copy.deepcopy(model.state_dict())
    model.load_state_dict(saved);model.eval()
    torch.save({"state":saved,"input_mean":mean.tolist(),"input_std":std.tolist(),"feature_names":FEATURES,"limit":.3,"enabled":False,"dt":DT,"history":HISTORY},out/"student.pt")
    torch.save({"inputs":x,"labels":y,"split":torch.tensor(data["split"]),"raw_features":torch.tensor(data["policy"])},out/"student_windows.pt")
    with (out/"teacher_labels.csv").open("w",newline="",encoding="utf-8") as handle:
        writer=csv.writer(handle);writer.writerow(["source_session","elapsed_s","split","effort_correction_cm_s2"])
        writer.writerows(zip(data["session"],data["times"],data["split"],labels))
    report={"teacher": "4-second 4-move exhaustive MPC, closed-loop A; 11 randomized physical cases", "nonzero_labels":int(np.count_nonzero(labels)),"total_labels":len(labels),"mean_candidate_plan_cost_improvement_fraction":float(np.mean(improvement)),"label_limit_cm_s2":.3}
    for name,mask in (("validation",val),("test",test)):
        with torch.no_grad():report[name+"_student_teacher_mae_cm_s2"]=(model(x[mask]).clamp(-.3,.3)-y[mask]).abs().mean().item()
    # Same starting states, same nominal ensemble, re-evaluated student every 100ms.
    ids=np.flatnonzero(test)[::4]
    features=np.c_[data["policy"][ids],data["x"][ids,0].numpy()+40]
    base,_=teacher_rollout(members,features,np.zeros((1,4),np.float32))
    learned,_=teacher_rollout(members,features,np.zeros((1,4),np.float32),student=model,stats=(torch.tensor(mean),torch.tensor(std)))
    report["heldout_model_rollout"]={"count":len(ids),"base_cost":base.mean().item(),"student_cost":learned.mean().item(),"fraction_better":(learned<base).float().mean().item(),"real_robot_validation":False}
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--epochs",type=int,default=80);parser.add_argument("--csv",action="append",type=Path,help="CSV input; repeat only when intentionally combining sessions");args=parser.parse_args()
    torch.set_num_threads(1);np.random.seed(101)
    out=ROOT/"artifacts/hybrid_mpc_v1";out.mkdir(parents=True,exist_ok=True)
    previous_path=out/"training_report.json"
    previous=json.loads(previous_path.read_text(encoding="utf-8")) if previous_path.exists() else None
    deployment_files=[out/name for name in ("proposal_student.espdl", "proposal_student.onnx", "ProposalStudentConfig.h", "espdl_validation.json")]
    unchanged_hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in deployment_files if path.exists()}
    data=read_data(args.csv);split_counts=np.bincount(data["split"],minlength=3)
    if np.any(split_counts < 20):
        raise ValueError(f"Refusing to overwrite trained artifacts: each temporal split needs at least 20 windows; got {split_counts.tolist()}")
    print("SPLITS",split_counts,"SOURCES",len(data["provenance"]),flush=True)
    with tempfile.TemporaryDirectory(prefix="hybrid_mpc_train_",dir=out.parent) as scratch:
        staged=Path(scratch)
        members,dynamics=fit_dynamics(data,staged,args.epochs)
        student=distill(data,members,staged,250)
        beats_cv=dynamics["test"]["depth_3s_mae_cm"]<dynamics["test"]["constant_velocity_3s_mae_cm"]*.9
        beats_base=student["heldout_model_rollout"]["student_cost"]<student["heldout_model_rollout"]["base_cost"]*.95
        report={"sources":data["provenance"],"split_counts":split_counts.tolist(),"split_protocol":"Per-session 60/20/20 time blocks with a 3s embargo on both sides of each boundary; every 3s history and prediction window stays within its block", "split_bounds_s":data["split_bounds"],"dynamics":dynamics,"student":student,
                "deployment_gate":{"enabled":False,"dynamics_beats_constant_velocity":beats_cv,"student_model_rollout_beats_A":beats_base,"sufficient_independent_data":False,"open_loop_test_complete":False,"real_water_student_validation":False,"reason":f"Only {len(data['provenance'])} short observational closed-loop session(s); no independent/open-loop plant identification or real-water student validation"},
                "limitations":["single capture session; temporal splits are correlated and do not establish generalization to a new run", "commanded PWM and estimated valves, not actual actuator feedback", "estimated acceleration has filter lag", "MPC modulator phase and actuator lock are assumed; hardware feedback unavailable", "4s MPC extrapolates a dynamics model trained on 3s prediction windows", "domain randomization is simulated, not validation of 11 real operating conditions"]}
        (staged/"training_report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
        status={"training_sources":len(data["provenance"]),"student_checkpoint_sha256":hashlib.sha256((staged/"student.pt").read_bytes()).hexdigest(),
                "espdl_requires_reexport":True,"firmware_requires_rebuild_after_reexport":True,"robot_firmware_changed":False,
                "existing_export_is_stale":True,"unchanged_deployment_files":unchanged_hashes,
                "warning":"New PT models use only the selected CSV. Existing ESP-DL/ONNX/header belong to the previous training and must not be paired with these checkpoints.",
                "previous_dynamics_test":previous["dynamics"]["test"] if previous else None,
                "previous_student_test_mae_cm_s2":previous["student"]["test_student_teacher_mae_cm_s2"] if previous else None,
                "previous_model_rollout":previous["student"]["heldout_model_rollout"] if previous else None}
        (staged/"retraining_status.json").write_text(json.dumps(status,indent=2),encoding="utf-8")
        output_names=("dynamics_ensemble.pt","student.pt","student_windows.pt","teacher_labels.csv","training_report.json","retraining_status.json")
        for name in output_names:
            os.replace(staged/name,out/name)
    assert all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest for path,digest in unchanged_hashes.items())
    print(json.dumps(report,indent=2),flush=True)


if __name__=="__main__":main()
