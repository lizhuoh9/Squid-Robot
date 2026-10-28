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
CONTROL_KP, CONTROL_KI, CONTROL_KD = .081, .009, .54
CONTROL_I_LIMIT = 1.5
SCHEMA_VERSION = "forward_history_dagger_v2"
FEATURES = (["depth_error_cm", "speed_cm_s", "accel_cm_s2", "proposal_bias_cm_s2"]
            + [f"applied_command_{i:02d}" for i in range(HISTORY)]
            + ["forward_active"] + [f"forward_history_{i:02d}" for i in range(HISTORY)])
POLICY_COMMANDS = slice(4, 4 + HISTORY)
POLICY_FORWARD = 4 + HISTORY
POLICY_FORWARD_HISTORY = slice(5 + HISTORY, 5 + 2 * HISTORY)
DYNAMICS_COMMANDS = slice(3, 3 + HISTORY)
DYNAMICS_FORWARD = 3 + HISTORY
DYNAMICS_FORWARD_HISTORY = slice(4 + HISTORY, 4 + 2 * HISTORY)
DYNAMICS_DIM = 4 + 2 * HISTORY
SENSOR_STUDENT_SCHEMA = "sensor_history_student_v5"
SENSOR_FEATURES = FEATURES + [name for i in range(HISTORY) for name in
                             (f"past_depth_delta_{i:02d}",f"past_speed_{i:02d}")]
FILES = [ROOT / "data" / f"capture-20261008T{s}Z.csv" for s in ("073354205522", "074026357113", "074725226418")]


def read_data(files=None, pc_max_gap=.300001, exclude_duplicate_timestamps=False, include_state_history=False):
    files = [Path(path) for path in (FILES if files is None else files)]
    if not files:
        raise ValueError("At least one capture CSV is required")
    samples, sessions, provenance, forward_actions, state_histories = [], [], [], [], []
    split_bounds_by_session = []
    for session, path in enumerate(files):
        forward_window_counts = np.zeros(3, dtype=int)
        with path.open(encoding="utf-8-sig") as handle:
            rows = list(csv.DictReader(handle))
        if not rows:
            raise ValueError(f"Empty CSV: {path}")
        columns = set(rows[0])
        is_ram_capture = {"depth_valid", "depth_age_ms", "proposal_bias_cm_s2", "pwm_commanded", "direction_commanded"} <= columns
        is_pc_control = not is_ram_capture and {"depth_cm", "depth_speed_cm_s", "depth_accel_cm_s2", "target_depth_cm", "pid_base", "buoyancy_pwm", "valve_e_on", "valve_f_on", "forward_active"} <= columns
        if not (is_ram_capture or is_pc_control):
            raise ValueError(f"Unsupported control CSV schema: {path}")
        times = np.array([float(r["elapsed_s"]) for r in rows])
        original_record_count=len(rows)
        duplicate_times=np.unique(times[1:][np.diff(times)==0])
        if exclude_duplicate_timestamps and len(duplicate_times) and not np.any(np.diff(times)<0):
            # Host may receive different sensor frames within the same ms.
            # Keep the final event for ordering, but exclude all learning
            # windows touching this ambiguous instant below. Never edit CSV.
            keep=np.r_[np.diff(times)!=0,True]
            rows=[row for row,valid in zip(rows,keep) if valid]
            times=times[keep]
        gaps = np.diff(times)
        if np.any(gaps <= 0):
            raise ValueError(f"Nonmonotonic timestamps: {path}")
        if "forward_active" not in columns:
            raise ValueError(f"Forward-state history is required: {path}")
        forward = np.array([int(r["forward_active"]) for r in rows])
        if np.any((forward < 0) | (forward > 1)):
            raise ValueError(f"Invalid forward state: {path}")
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
            # states are ambiguous and excluded. Forward state is modeled
            # separately from the signed vertical command.
            raw = np.array([[float(r[k]) if r[k].strip() else np.nan for k in ("depth_cm", "depth_speed_cm_s", "depth_accel_cm_s2", "pid_base")] for r in rows])
            # The PC logger leaves the target blank before a depth command is
            # issued. Treat those rows as invalid samples instead of aborting
            # the full-session training run.
            target = np.array([float(r["target_depth_cm"]) if r["target_depth_cm"].strip() else np.nan for r in rows])
            pwm = np.array([float(r["buoyancy_pwm"]) for r in rows])
            valve_e = np.array([int(r["valve_e_on"]) for r in rows])
            valve_f = np.array([int(r["valve_f_on"]) for r in rows])
            ambiguous = valve_e != valve_f
            if np.any((valve_e > 1) | (valve_f > 1) | (forward > 1) | (pwm < 0) | (pwm > 255)):
                raise ValueError(f"Invalid actuator fields in PC CSV: {path}")
            signed_direction = np.where(valve_e == 1, -1.0, 1.0)
            signed = (pwm / 255.0) * signed_direction
            signed[pwm == 0] = 0.0
            signed[ambiguous] = np.nan
            valid_rows = np.isfinite(raw).all(axis=1) & np.isfinite(target) & (target > 0) & np.isfinite(signed)
            if not valid_rows.all():
                # Keep elapsed-time gaps intact; invalid rows cannot be bridged
                # by the later window-continuity check.
                signed[~valid_rows] = np.nan
            max_gap_for_window = pc_max_gap
            target_counts = {}
            for value in target[valid_rows]:
                key = str(float(value))
                target_counts[key] = target_counts.get(key, 0) + 1
            fresh_count = None  # The PC CSV does not carry a sensor-fresh flag.
            data_format = "pc_control_200ms"
        if exclude_duplicate_timestamps and len(duplicate_times):
            signed[np.isin(times,duplicate_times)]=np.nan
        duration = float(times[-1] - times[0])
        split_bounds = (times[0] + duration * .6, times[0] + duration * .8)
        split_bounds_by_session.append(split_bounds)
        grid = np.arange(times[0] + DT, times[-1] - DT, DT)
        # Causal zero-order hold, NOT interpolation using a future sensor value.
        ids = np.searchsorted(times, grid, side="right") - 1
        states, targets, forward_now = raw[ids], target[ids], forward[ids]
        # Time-weighted actual command during each future integration interval.
        duty, forward_duty = [], []
        for t in grid:
            edges = np.r_[t, times[(times > t) & (times < t + DT)], t + DT]
            j = np.searchsorted(times, edges[:-1], side="right") - 1
            action = signed[j]
            duty.append(np.nan if not np.isfinite(action).all() else np.sum(np.diff(edges) * action) / DT)
            forward_duty.append(np.sum(np.diff(edges) * forward[j]) / DT)
        duty = np.array(duty)
        forward_duty = np.array(forward_duty)
        # Future depth labels are interpolated only as targets. Model inputs
        # remain causal zero-order holds from the last available PC sample.
        future_depth = np.interp(grid, times, raw[:, 0])
        if not np.isfinite(duty).any():
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
            future_values=np.c_[future_depth[i + 1:i + HORIZON + 1], states[i + 1:i + HORIZON + 1, 1]]
            if (split < 0 or not continuous or not np.isfinite(targets[i]) or not 0 < targets[i] <= 100
                    or not np.isfinite(window_duty).all() or not np.isfinite(states[i]).all()
                    or not np.isfinite(future_values).all()):
                continue
            z, v, a, bias = states[i]
            history = duty[i - HISTORY:i]
            forward_history = forward_duty[i - HISTORY:i]
            samples.append((np.r_[z - 40, v, a, history, forward_now[i], forward_history], duty[i:i + HORIZON],
                            future_values,
                            np.r_[z - targets[i], v, a, bias, history, forward_now[i], forward_history], split, session, t))
            forward_actions.append(forward_duty[i:i + HORIZON])
            if include_state_history:
                # Strictly past causal sensor holds. Center depth on the
                # current observed depth, not on any future/target value.
                state_histories.append(np.c_[states[i-HISTORY:i,0]-z,states[i-HISTORY:i,1]])
            stop = np.searchsorted(times, window_end, side="right")
            if np.any(forward[first_gap:stop] == 1):
                forward_window_counts[split] += 1
        sessions.append((grid, states, targets, duty))
        provenance.append({"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "format": data_format,
                           "record_count": original_record_count, "fresh_sensor_count": fresh_count,
                           "excluded_duplicate_timestamp_count":len(duplicate_times) if exclude_duplicate_timestamps else 0,
                           "duration_s": duration, "median_interval_s": float(np.median(gaps)),
                           "max_interval_s": float(np.max(gaps)), "target_valid_row_counts_cm": target_counts,
                           "forward_active_row_count": int(np.count_nonzero(forward == 1)),
                           "forward_active_valid_row_count": int(np.count_nonzero((forward == 1) & valid_rows)) if is_pc_control else None,
                           "forward_active_window_counts": forward_window_counts.tolist()})
    if not samples:
        raise ValueError("No eligible training windows: check target-valid duration and capture length")
    x, u, y, policy, split, session, t = zip(*samples)
    data=dict(x=torch.tensor(np.array(x), dtype=torch.float32), u=torch.tensor(np.array(u), dtype=torch.float32),
                forward_u=torch.tensor(np.array(forward_actions), dtype=torch.float32),
                y=torch.tensor(np.array(y), dtype=torch.float32), policy=np.array(policy, dtype=np.float32),
                split=np.array(split), session=np.array(session), times=np.array(t), provenance=provenance,
                split_bounds=split_bounds_by_session)
    if include_state_history:
        data['state_history']=torch.tensor(np.array(state_histories),dtype=torch.float32)
        if not torch.isfinite(data['state_history']).all():raise ValueError('Nonfinite causal sensor history')
    return data


class Hybrid(nn.Module):
    def __init__(self):
        super().__init__()
        self.latent = nn.Sequential(nn.Linear(DYNAMICS_DIM, 24), nn.Tanh(), nn.Linear(24, 1))
        self.residual = nn.Sequential(nn.Linear(7, 24), nn.Tanh(), nn.Linear(24, 1))
        self.physical = nn.Parameter(torch.tensor([0., -1., -1., 0., -.5, 0.]))
        nn.init.zeros_(self.latent[-1].weight); nn.init.zeros_(self.latent[-1].bias)
        nn.init.zeros_(self.residual[-1].weight); nn.init.zeros_(self.residual[-1].bias)

    def constants(self):
        s = torch.sigmoid(self.physical)
        return .5 + 3.5*s[0], .01 + .25*s[1], .3 + 5.7*s[2], .3 + 11.7*s[3], -1 + 4*s[4], -.03 + .09*s[5]

    def initial(self, x):
        _, drag, _, _, _, _ = self.constants()
        normalized = x / x.new_tensor([40., 6., 4.] + [1.]*(DYNAMICS_DIM - 3))
        z, v, a = x[:, 0] + 40, x[:, 1], x[:, 2]
        # Estimate hidden buoyancy from acceleration and the preceding 3s actions.
        b = a + drag*v*v.abs() + 2*torch.tanh(self.latent(normalized).squeeze(1))
        return torch.stack((z, v, b), 1)

    def step(self, state, u, history, dt=DT, factors=None, forward_active=None, forward_history=None):
        tau, drag, sink, rise, rest, alpha = self.constants()
        if factors is not None:
            tau, sink, rise, rest = tau*factors[0], sink*factors[1], rise*factors[2], rest+factors[3]
        z, v, b = state.unbind(1)
        forward_active = torch.zeros_like(u) if forward_active is None else forward_active
        forward_mean = torch.zeros_like(u) if forward_history is None else forward_history.mean(1)
        correction = torch.tanh(self.residual(torch.stack(((z-40)/40, v/6, b/4, u, history.mean(1), forward_active, forward_mean), 1)).squeeze(1))
        eq = rest + alpha*(z-40) + torch.where(u >= 0, sink*u, rise*u)
        bnext = b + dt*((eq-b)/tau + correction)
        acceleration = b - drag*v*v.abs()
        vnext = v + dt*acceleration
        return torch.stack((z + dt*v + .5*dt*dt*acceleration, vnext, bnext), 1)

    def forward(self, x, actions, forward_actions=None):
        state, history, values = self.initial(x), x[:, DYNAMICS_COMMANDS], []
        forward_history = x[:, DYNAMICS_FORWARD_HISTORY]
        if forward_actions is None:
            forward_actions = x[:, DYNAMICS_FORWARD, None].expand_as(actions)
        for u, fw in zip(actions.unbind(1), forward_actions.unbind(1)):
            state = self.step(state, u, history, forward_active=fw, forward_history=forward_history)
            values.append(state[:, :2])
            history = torch.cat((history[:, 1:], u[:, None]), 1)
            forward_history = torch.cat((forward_history[:, 1:], fw[:, None]), 1)
        return torch.stack(values, 1)


class HistoryHybrid(Hybrid):
    """Dynamics-only v2: ordered action history and initial velocity correction.

    The original Hybrid remains loadable for frozen comparisons. This class
    retains the same physical state/step interface but has different weights;
    MPC labels and students must be regenerated before using this checkpoint.
    """
    def __init__(self):
        super().__init__()
        self.velocity = nn.Sequential(nn.Linear(DYNAMICS_DIM,32),nn.Tanh(),nn.Linear(32,1))
        self.residual = nn.Sequential(nn.Linear(18,32),nn.Tanh(),nn.Linear(32,1))
        nn.init.zeros_(self.velocity[-1].weight);nn.init.zeros_(self.velocity[-1].bias)
        nn.init.zeros_(self.residual[-1].weight);nn.init.zeros_(self.residual[-1].bias)

    def initial(self,x):
        state=super().initial(x)
        normalized=x/x.new_tensor([40.,6.,4.]+[1.]*(DYNAMICS_DIM-3))
        dv=2*torch.tanh(self.velocity(normalized).squeeze(1))
        return torch.stack((state[:,0],state[:,1]+dv,state[:,2]),1)

    def step(self,state,u,history,dt=DT,factors=None,forward_active=None,forward_history=None):
        tau,drag,sink,rise,rest,alpha=self.constants()
        if factors is not None:tau,sink,rise,rest=tau*factors[0],sink*factors[1],rise*factors[2],rest+factors[3]
        z,v,b=state.unbind(1)
        fw=torch.zeros_like(u) if forward_active is None else forward_active
        fmean=torch.zeros_like(u) if forward_history is None else forward_history.mean(1)
        # Separate rise/sink averages: signed means otherwise cancel opposite
        # pump bursts. Ordered windows retain recent vs delayed pump effects.
        encoded=[]
        for ticks in (2,5,10,20,30):
            window=history[:,-ticks:]
            encoded.extend((window.clamp_min(0).mean(1),(-window).clamp_min(0).mean(1)))
        switches=(history[:,1:]*history[:,:-1]<0).float().mean(1)
        inputs=torch.stack(((z-40)/40,v/6,b/4,u,fw,fmean,history[:,-1],switches,*encoded),1)
        correction=torch.tanh(self.residual(inputs).squeeze(1))
        equilibrium=rest+alpha*(z-40)+torch.where(u>=0,sink*u,rise*u)
        bnext=b+dt*((equilibrium-b)/tau+correction)
        acceleration=b-drag*v*v.abs()
        return torch.stack((z+dt*v+.5*dt*dt*acceleration,v+dt*acceleration,bnext),1)


class SensorHistoryHybrid(HistoryHybrid):
    """Prediction-only v4: infer hidden state from past depth/speed as well.

    The zero-initialized encoder preserves a frozen HistoryHybrid exactly
    before training. No future measurements are consumed by the encoder.
    """
    def __init__(self):
        super().__init__()
        self.sensor_encoder=nn.Sequential(nn.Linear(HISTORY*2,32),nn.Tanh(),nn.Linear(32,2))
        nn.init.zeros_(self.sensor_encoder[-1].weight);nn.init.zeros_(self.sensor_encoder[-1].bias)

    def initial(self,x,sensor_history=None):
        if sensor_history is None or sensor_history.shape!=(len(x),HISTORY,2):
            raise ValueError('SensorHistoryHybrid requires [batch,30,2] past depth/speed history')
        state=super().initial(x)
        normalized=sensor_history/sensor_history.new_tensor([6.,6.])
        delta=.75*torch.tanh(self.sensor_encoder(normalized.flatten(1)))
        return torch.stack((state[:,0],state[:,1]+delta[:,0],state[:,2]+delta[:,1]),1)

    def forward(self,x,actions,forward_actions=None,sensor_history=None):
        state=self.initial(x,sensor_history);history=x[:,DYNAMICS_COMMANDS]
        forward_history=x[:,DYNAMICS_FORWARD_HISTORY];values=[]
        if forward_actions is None:forward_actions=x[:,DYNAMICS_FORWARD,None].expand_as(actions)
        for u,fw in zip(actions.unbind(1),forward_actions.unbind(1)):
            state=self.step(state,u,history,forward_active=fw,forward_history=forward_history)
            values.append(state[:,:2])
            history=torch.cat((history[:,1:],u[:,None]),1)
            forward_history=torch.cat((forward_history[:,1:],fw[:,None]),1)
        return torch.stack(values,1)


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
                pred = model(data["x"][ids], data["u"][ids], data["forward_u"][ids]); truth = data["y"][ids]
                loss = ((pred-truth)/torch.tensor([2., 1.])).square().mean()
                opt.zero_grad();loss.backward();nn.utils.clip_grad_norm_(model.parameters(), 5);opt.step()
            with torch.no_grad():
                value = (model(data["x"][val], data["u"][val], data["forward_u"][val])-data["y"][val]).square().mean().item()
            if value < best:best, best_state, best_epoch = value, copy.deepcopy(model.state_dict()), epoch+1
            if (epoch+1)%20 == 0:print("DYNAMICS",seed,epoch+1,"validation_loss",round(value,4),flush=True)
        model.load_state_dict(best_state);model.eval();members.append(model)
        summaries.append({"seed": seed, "best_epoch": best_epoch, "physical_tau_drag_sink_rise_rest_alpha": [p.item() for p in model.constants()]})
    torch.save({"states": [m.state_dict() for m in members], "dt": DT, "history": HISTORY, "horizon": HORIZON,
                "schema_version": SCHEMA_VERSION, "input_dim": DYNAMICS_DIM}, out/"dynamics_ensemble.pt")
    report = {}
    for name, ids in (("validation",val),("test",test)):
        with torch.no_grad():pred = torch.stack([m(data["x"][ids],data["u"][ids],data["forward_u"][ids]) for m in members]).mean(0)
        times = torch.arange(1,HORIZON+1)*DT
        cv = data["x"][ids,0,None] + 40 + data["x"][ids,1,None]*times
        error = pred[:,:,0]-data["y"][ids,:,0];baseline=cv-data["y"][ids,:,0]
        report[name] = {"count":len(ids),"depth_trajectory_mae_cm":error.abs().mean().item(),"depth_3s_mae_cm":error[:,-1].abs().mean().item(),"constant_velocity_trajectory_mae_cm":baseline.abs().mean().item(),"constant_velocity_3s_mae_cm":baseline[:,-1].abs().mean().item()}
    report["members"] = summaries
    return members, report


@torch.no_grad()
def guard_ascent_correction(error, speed, correction):
    """Match firmware: block negative residual at/above target at any speed."""
    blocked = (error <= 0) & (correction < 0)
    return torch.where(blocked, torch.zeros_like(correction), correction)


def effective_control_bias(error, speed, bias):
    """Reduce positive bias while shallow at any speed; preserve trim state."""
    weight=(-error/2).clamp(0,1)
    reduction=torch.minimum(bias.clamp_min(0),torch.full_like(bias,.2))*weight
    return bias-reduction


def teacher_rollout(members, features, plans, factors=None, student=None, stats=None,
                    phase_ids=None, start_context=None, return_details=False, collect_ticks=(), correction_limit=.3, objective="legacy"):
    """4s closed-loop A: 50ms motion, 100ms control, pulses + 200ms lock.

    PWM/valve state is simulated, not measured. Restored modulator phase and
    direction-lock state are unknown in recorded logs: use randomized phases.
    """
    if not np.isfinite(correction_limit) or correction_limit<=0:
        raise ValueError("Correction limit must be finite and positive")
    if objective not in ("legacy", "range"):
        raise ValueError("Unknown control objective")
    n = len(features); candidates = plans.shape[-2]; f = torch.as_tensor(features,dtype=torch.float32).repeat_interleave(candidates,0)
    p = (torch.tensor(plans,dtype=torch.float32).reshape(n*candidates,4) if plans.ndim==3
         else torch.tensor(plans,dtype=torch.float32).repeat(n,1))
    sensor_enabled=features.shape[1]==len(SENSOR_FEATURES)+1
    if features.shape[1] not in (len(FEATURES)+1,len(SENSOR_FEATURES)+1):raise ValueError('Invalid rollout feature dimension')
    if collect_ticks and (candidates != 1 or len(members) != 1 or any(t % 2 for t in collect_ticks)):
        raise ValueError("DAgger snapshots require one plant, one policy and complete 100ms intervals")
    # Teacher features carry absolute z as an additional final column.
    target = f[:,-1] - f[:,0]
    x = torch.cat((f[:,-1,None]-40,f[:,1:3],f[:,4:len(FEATURES)]),1)
    count=n*candidates
    def restored(name,default):
        return (torch.as_tensor(start_context[name],dtype=torch.float32).repeat_interleave(candidates,0).clone()
                if start_context is not None else default.clone())
    sensor=f[:,len(FEATURES):-1].reshape(n*candidates,HISTORY,2) if sensor_enabled else None
    states=[restored("state",m.initial(x,sensor) if isinstance(m,SensorHistoryHybrid) else m.initial(x)) for m in members]
    sensor_depths=[restored("sensor_depths",sensor[:,:,0]+f[:,-1,None]) for _ in members] if sensor_enabled else []
    sensor_speeds=[restored("sensor_speeds",sensor[:,:,1]) for _ in members] if sensor_enabled else []
    histories=[restored("history",x[:,DYNAMICS_COMMANDS]) for _ in members]
    forward_histories=[restored("forward_history",x[:,DYNAMICS_FORWARD_HISTORY]) for _ in members]
    forward_current=restored("forward",f[:,POLICY_FORWARD])
    costs=torch.zeros(count)
    integral_limit=CONTROL_I_LIMIT/CONTROL_KI
    integ=[restored("integral",((f[:,3]-1.1-.03*(f[:,-1]-40))/CONTROL_KI).clamp(-integral_limit,integral_limit)) for _ in members]
    duty=[restored("duty",torch.zeros(count)) for _ in members]
    last=[restored("last",torch.sign(x[:,DYNAMICS_COMMANDS][:,-1])) for _ in members]
    # All competing action plans MUST start at the same modulator phase.
    phases=restored("phase",(torch.arange(n) if phase_ids is None else torch.as_tensor(phase_ids)).repeat_interleave(candidates)%20)
    mod_start=[restored("mod_start",-phases) for _ in members]
    on_until=[restored("on_until",torch.zeros(count)) for _ in members]
    active_dir=[restored("active_dir",torch.sign(x[:,DYNAMICS_COMMANDS][:,-1])) for _ in members]
    lock_until=[restored("lock_until",torch.zeros(count)) for _ in members]
    previous=[restored("previous",x[:,DYNAMICS_COMMANDS][:,-1]) for _ in members]
    averaged=[torch.zeros(count) for _ in members]
    maximum_uncertainty=torch.zeros(count)
    corrections=[restored("correction",torch.zeros(count)) for _ in members]
    components={key:torch.zeros(count) for key in ("depth","speed","correction","command_change","uncertainty","outside_band","braking","correction_change","peak")}
    error_history=[]
    previous_correction=[torch.zeros(count) for _ in members]
    sampled_depth=[torch.zeros(count) for _ in members]
    sampled_speed=[torch.zeros(count) for _ in members]
    absolute_error=torch.zeros(count);within_one=torch.zeros(count);maximum_error=torch.zeros(count)
    final_error=torch.zeros(count);snapshots=[]
    for tick in range(80):
        seconds=tick*.05;control_tick=tick%2==0
        for k,m in enumerate(members):
            z,v,b=states[k].unbind(1);e=z-target
            if control_tick:
                # The initial controller observation is measured, not the
                # predictor's latent velocity/buoyancy estimate. Subsequent
                # observations are predictions (sensor filtering is assumed).
                observed_v=f[:,1] if tick==0 and start_context is None else v
                observed_a=f[:,2] if tick==0 and start_context is None else b-m.constants()[1]*v*v.abs()
                sampled_depth[k]=z.clone();sampled_speed[k]=observed_v.clone()
                previous_correction[k]=corrections[k].clone()
                bias=1.1+.03*(z-40)+CONTROL_KI*integ[k]
                fi=torch.cat((torch.stack((e,observed_v,observed_a,bias),1),histories[k],forward_current[:,None],forward_histories[k]),1)
                if sensor_enabled:
                    past=torch.stack((sensor_depths[k]-z[:,None],sensor_speeds[k]),2).flatten(1)
                    fi=torch.cat((fi,past),1)
                if tick in collect_ticks:
                    context={"state":states[k].clone(),"integral":integ[k].clone(),"duty":duty[k].clone(),
                             "last":last[k].clone(),"phase":((tick+phases)%20).clone(),
                             "on_until":(on_until[k]-tick).clamp_min(0),"active_dir":active_dir[k].clone(),
                             "lock_until":(lock_until[k]-seconds).clamp_min(0),"previous":previous[k].clone(),
                             "history":histories[k].clone(),"forward_history":forward_histories[k].clone(),
                             "forward":forward_current.clone(),"mod_start":mod_start[k]-tick,
                             "correction":corrections[k].clone()}
                    if sensor_enabled:context.update(sensor_depths=sensor_depths[k].clone(),sensor_speeds=sensor_speeds[k].clone())
                    snapshots.append({"tick":tick,"features":torch.cat((fi,z[:,None]),1).clone(),"context":context})
                correction=p[:,min(3,int(seconds))].clamp(-correction_limit,correction_limit)
                if student is not None:
                    norm=(fi[:,:len(stats[0])]-stats[0])/stats[1]
                    correction=student(norm).squeeze(1).clamp(-correction_limit,correction_limit)
                    guard=(e.abs()<=12)&(observed_v.abs()<=6)&(norm.abs().amax(1)<=6)
                    correction=torch.where(guard,correction,torch.zeros_like(correction))
                correction=guard_ascent_correction(e,observed_v,correction)
                corrections[k]=correction
                desired=-CONTROL_KP*e-CONTROL_KD*observed_v
                effort=desired-effective_control_bias(e,observed_v,bias)+correction
                req=torch.where(effort>0,effort/.75,effort/4.5)
                saturated=req.abs()>=1;req=req.clamp(-1,1)
                integ[k]=torch.where(saturated,integ[k],(integ[k]+e*.1).clamp(-integral_limit,integral_limit))
                req=torch.where((last[k]!=0)&(torch.sign(req)!=last[k])&(req.abs()<.05),torch.zeros_like(req),req)
                last[k]=torch.where(req!=0,torch.sign(req),last[k]);duty[k]=req
            mag=duty[k].abs()
            # Match firmware: only low-duty requests start a modulation window.
            window=(mag>0)&(mag<80/255)&(tick-mod_start[k]>=20)
            on_until[k]=torch.where(window,tick+20*mag/(80/255),on_until[k])
            mod_start[k]=torch.where(window,torch.full_like(mag,tick),mod_start[k])
            applied=torch.where(mag>=80/255,mag,torch.where(tick<on_until[k],torch.full_like(mag,80/255),torch.zeros_like(mag)))
            command=applied*torch.sign(duty[k]);direction=torch.sign(command)
            change=(direction!=0)&(active_dir[k]!=0)&(direction!=active_dir[k])
            lock_until[k]=torch.where(change,torch.full_like(mag,seconds+.2),lock_until[k])
            active_dir[k]=torch.where(direction!=0,direction,active_dir[k])
            command=torch.where(seconds<lock_until[k],torch.zeros_like(command),command)
            states[k]=m.step(states[k],command,histories[k],dt=.05,factors=factors,
                             forward_active=forward_current,forward_history=forward_histories[k])
            err=(states[k][:,0]-target).abs()
            weight=.05/len(members)
            stage={"depth":err.square()*weight,"speed":.8*states[k][:,1].square()*weight,
                   "correction":.2*corrections[k].square()*weight,
                   "command_change":.1*(command-previous[k]).abs()*weight}
            if objective == "range":
                # Stronger braking near the target; penalize leaving +/-2cm
                # and a 2-second constant-speed projection before crossing it.
                stage["speed"]=2.4*states[k][:,1].square()*weight
                stage["outside_band"]=3.0*torch.relu(err-2.0).square()*weight
                projected=states[k][:,0]-target+2.0*states[k][:,1]
                stage["braking"]=.6*projected.square()*weight
                if control_tick:
                    stage["correction_change"]=.3*(corrections[k]-previous_correction[k]).square()/len(members)
            for key,value in stage.items():
                components[key]+=value;costs+=value
            absolute_error+=err*weight/4
            within_one+=(err<=1).float()*weight/4
            maximum_error=torch.maximum(maximum_error,err)
            if return_details:
                error_history.append(err.clone())
            if tick==79:final_error+=err/len(members)
            previous[k]=command
            averaged[k]+=command*.5
            if tick%2:
                if sensor_enabled:
                    sensor_depths[k]=torch.cat((sensor_depths[k][:,1:],sampled_depth[k][:,None]),1)
                    sensor_speeds[k]=torch.cat((sensor_speeds[k][:,1:],sampled_speed[k][:,None]),1)
                histories[k]=torch.cat((histories[k][:,1:],averaged[k][:,None]),1)
                forward_histories[k]=torch.cat((forward_histories[k][:,1:],forward_current[:,None]),1)
                averaged[k].zero_()
        uncertainty=torch.stack([state[:,0] for state in states]).std(0,unbiased=False)
        maximum_uncertainty=torch.maximum(maximum_uncertainty,uncertainty)
        costs+=uncertainty.square()*.1
        components["uncertainty"]+=uncertainty.square()*.1
    if objective == "range":
        components["peak"]=1.5*maximum_error.square()
        costs+=components["peak"]
    result=(costs.reshape(n,candidates),maximum_uncertainty.reshape(n,candidates))
    if return_details or collect_ticks:
        details={"cost_components":{key:value.reshape(n,candidates) for key,value in components.items()},
                 "depth_mae_cm":absolute_error.reshape(n,candidates),
                 "depth_rmse_cm":(components["depth"]/4).sqrt().reshape(n,candidates),
                 "max_abs_error_cm":maximum_error.reshape(n,candidates),
                 "p95_abs_error_cm":torch.quantile(torch.stack(error_history),.95,dim=0).reshape(n,candidates) if return_details else None,
                 "final_abs_error_cm":final_error.reshape(n,candidates),
                 "within_1cm_fraction":within_one.reshape(n,candidates),"snapshots":snapshots}
        return (*result,details)
    return result


class Student(nn.Module):
    def __init__(self,input_dim=None):
        super().__init__()
        self.fc1=nn.Linear(len(FEATURES) if input_dim is None else input_dim,32);self.fc2=nn.Linear(32,16);self.fc3=nn.Linear(16,1)
    def forward(self,x):return self.fc3(torch.relu(self.fc2(torch.relu(self.fc1(x)))))


DOMAINS=[(1.,1.,1.,0.),(.8,1.,1.,0.),(1.2,1.,1.,0.),(1.,.8,1.,0.),(1.,1.2,1.,0.),
         (1.,1.,.8,0.),(1.,1.,1.2,0.),(1.,1.,1.,-.2),(1.,1.,1.,.2),(.8,.8,.8,-.2),(1.2,1.2,1.2,.2)]
PLANS=np.array(list(itertools.product((-.3,0.,.3),repeat=4)),dtype=np.float32)


def phase_for_rows(data,ids):
    return (data["session"][ids]*997+np.rint(data["times"][ids]/DT).astype(np.int64))%20


def label_batch(members,features,factors,phase_ids=None,context=None,correction_limit=.3,objective="legacy"):
    plans=PLANS*(correction_limit/.3)
    cost,uncertain=teacher_rollout(members,features,plans,factors=factors,
                                  phase_ids=phase_ids,start_context=context,correction_limit=correction_limit,objective=objective)
    zero=np.flatnonzero(np.all(plans==0,axis=1))[0]
    best=cost.argmin(1);ids=torch.arange(len(features))
    best_cost=cost[ids,best];best_uncertainty=uncertain[ids,best]
    labels=plans[best.numpy(),0].copy()
    if features.shape[1]==len(SENSOR_FEATURES)+1:
        # Coarse MPC followed by eight local +/-0.25 refinements, rather than
        # forcing every nonzero teacher label to one of the output limits.
        selected=plans[best.numpy()]
        local=np.repeat(selected[:,None,:],9,axis=1)
        for move in range(4):
            local[:,1+2*move,move]-=correction_limit/2
            local[:,2+2*move,move]+=correction_limit/2
        local=np.clip(local,-correction_limit,correction_limit)
        refined,spread=teacher_rollout(members,features,local,factors=factors,phase_ids=phase_ids,
                                      start_context=context,correction_limit=correction_limit,objective=objective)
        winner=refined.argmin(1)
        best_cost=refined[ids,winner];best_uncertainty=spread[ids,winner]
        labels=local[np.arange(len(features)),winner.numpy(),0].copy()
    gain=(cost[:,zero]-best_cost)/(cost[:,zero]+1e-6)
    good=(gain.numpy()>.02)&(best_uncertainty.numpy()<1.)&(np.abs(features[:,0])<=12)&(np.abs(features[:,1])<=6)
    labels[~good]=0
    return labels,gain.numpy()


def fit_student(features,labels,split,mean,std,epochs,initial=None):
    torch.manual_seed(71)
    model=Student(features.shape[1])
    if initial is not None:model.load_state_dict(initial)
    x=torch.tensor((features-mean)/std);y=torch.tensor(labels[:,None])
    train=np.flatnonzero(split==0);val=split==1
    opt=torch.optim.Adam(model.parameters(),lr=.001,weight_decay=1e-4)
    best=float("inf");saved=None
    for epoch in range(epochs):
        for ids in np.array_split(np.random.default_rng(epoch).permutation(train),max(1,len(train)//128)):
            loss=(model(x[ids])-y[ids]).square().mean()
            opt.zero_grad();loss.backward();nn.utils.clip_grad_norm_(model.parameters(),5);opt.step()
        with torch.no_grad():loss=(model(x[val])-y[val]).square().mean().item()
        if loss<best:best=loss;saved=copy.deepcopy(model.state_dict())
    model.load_state_dict(saved);model.eval()
    return model


def paired_evaluation(data,ids,members,model,stats,correction_limit=.3,objective="legacy"):
    features=np.c_[data["policy"][ids],data["x"][ids,0].numpy()+40]
    phases=phase_for_rows(data,ids);zero=np.zeros((1,4),np.float32)
    base,_,base_detail=teacher_rollout(members,features,zero,phase_ids=phases,return_details=True,objective=objective)
    learned,_,detail=teacher_rollout(members,features,zero,student=model,stats=stats,phase_ids=phases,
                                   return_details=True,correction_limit=correction_limit,objective=objective)
    def metrics(mask):
        b=base[:,0][mask];s=learned[:,0][mask]
        result={"count":int(np.count_nonzero(mask)),"base_cost":b.mean().item(),"student_cost":s.mean().item(),
                "cost_improvement_fraction":((b.mean()-s.mean())/(b.mean()+1e-6)).item(),
                "fraction_better":(s<b).float().mean().item()}
        for prefix,values in (("base",base_detail),("student",detail)):
            result[prefix+"_cost_components"]={k:v[:,0][mask].mean().item() for k,v in values["cost_components"].items()}
            for key in ("depth_mae_cm","depth_rmse_cm","p95_abs_error_cm","max_abs_error_cm","final_abs_error_cm","within_1cm_fraction"):
                selected=values[key][:,0][mask]
                result[prefix+"_"+key]=(selected.max() if key=="max_abs_error_cm" else selected.mean()).item()
        return result
    result=metrics(np.ones(len(ids),dtype=bool));result["real_robot_validation"]=False
    result["cost_version"]="range_v3" if objective=="range" else "v2_actual_applied_correction"
    result["objective"]=objective
    result["runtime_limit_cm_s2"]=correction_limit
    result["evaluation_window_sha256"]=hashlib.sha256(np.c_[data["session"][ids],data["times"][ids]].astype("<f8").tobytes()).hexdigest()
    result["by_forward_state"]={}
    for active in (0,1):
        mask=data["policy"][ids,POLICY_FORWARD]==active
        if mask.any():result["by_forward_state"][str(active)]=metrics(mask)
    result["by_target_cm"]={}
    steady=(np.abs(data['policy'][ids,0])<=2)&(np.abs(data['policy'][ids,1])<=.3)
    if steady.any():result['steady_hold']=metrics(steady)
    targets=features[:,-1]-features[:,0]
    for target in (30,40,50):
        mask=np.isclose(targets,target,atol=.01)
        if mask.any():result["by_target_cm"][str(target)]=metrics(mask)
    return result


def range_acceptance(candidate, reference, tolerance=1e-6):
    """Compare identical rollout populations, never costs with different weights."""
    if candidate["evaluation_window_sha256"] != reference["evaluation_window_sha256"]:
        raise ValueError("Range comparison requires identical evaluation windows")
    keys=("depth_mae_cm", "p95_abs_error_cm", "max_abs_error_cm")
    checks={}
    for key in keys:
        value=candidate["student_"+key]
        checks["beats_A_"+key]=value<=candidate["base_"+key]+tolerance
        checks["beats_previous_"+key]=value<=reference["student_"+key]+tolerance
    checks["strictly_smaller_p95"]=candidate["student_p95_abs_error_cm"]<reference["student_p95_abs_error_cm"]-tolerance
    checks["strictly_smaller_max"]=candidate["student_max_abs_error_cm"]<reference["student_max_abs_error_cm"]-tolerance
    return {"passed":all(checks.values()),"checks":checks}


def sensor_student_acceptance(candidate,reference,tolerance=1e-6):
    """Predefined steady-hold cohort; large initial descents stay in global MAE."""
    if candidate['evaluation_window_sha256']!=reference['evaluation_window_sha256']:
        raise ValueError('Acceptance windows differ')
    c=candidate.get('steady_hold');r=reference.get('steady_hold')
    if not c or not r or c['count']<20:return {'passed':False,'checks':{'enough_fixed_steady_hold_windows':False}}
    checks={'enough_fixed_steady_hold_windows':True,
            'overall_mae_not_worse_than_A':candidate['student_depth_mae_cm']<=candidate['base_depth_mae_cm']+tolerance,
            'overall_mae_not_worse_than_previous':candidate['student_depth_mae_cm']<=reference['student_depth_mae_cm']+tolerance}
    for key in ('depth_mae_cm','p95_abs_error_cm','max_abs_error_cm'):
        checks['hold_beats_A_'+key]=c['student_'+key]<=c['base_'+key]+tolerance
        checks['hold_beats_previous_'+key]=c['student_'+key]<=r['student_'+key]+tolerance
    checks['hold_p95_strictly_smaller']=c['student_p95_abs_error_cm']<r['student_p95_abs_error_cm']-tolerance
    checks['hold_max_strictly_smaller']=c['student_max_abs_error_cm']<r['student_max_abs_error_cm']-tolerance
    return {'passed':all(checks.values()),'checks':checks,'steady_hold_count':c['count'],
            'cohort':'starting abs(depth error)<=2cm and abs(speed)<=0.3cm/s; fixed before evaluation'}


@torch.no_grad()
def collect_dagger(data,members,model,stats,iteration,correction_limit=.3,objective="legacy"):
    # Only train-block starting states. No validation/test trajectories or
    # future recorded forward switches are used to generate student inputs.
    train=np.flatnonzero(data["split"]==0)
    rng=np.random.default_rng(900+iteration)
    seeds=rng.choice(train,min(96,len(train)),replace=False)
    samples=[]
    for group,begin in enumerate(range(0,len(seeds),16)):
        ids=seeds[begin:begin+16]
        features=np.c_[data["policy"][ids],data["x"][ids,0].numpy()+40]
        factors=DOMAINS[(iteration*6+group)%len(DOMAINS)]
        plant=members[(iteration+group)%len(members)]
        _,_,details=teacher_rollout([plant],features,np.zeros((1,4),np.float32),student=model,stats=stats,
                                   factors=factors,phase_ids=phase_for_rows(data,ids),collect_ticks=(20,40,60),correction_limit=correction_limit,objective=objective)
        for snapshot in details["snapshots"]:
            f=snapshot["features"].numpy()
            normalized=(f[:,:len(stats[0])]-stats[0].numpy())/stats[1].numpy()
            valid=(np.isfinite(f).all(1)&(np.abs(f[:,0])<=12)&(np.abs(f[:,1])<=6)
                   &(f[:,-1]>0)&(f[:,-1]<=100)&(np.abs(normalized).max(1)<=6))
            if not valid.any():continue
            context={key:value[valid] for key,value in snapshot["context"].items()}
            labels,_=label_batch(members,f[valid],factors,context=context,correction_limit=correction_limit,objective=objective)
            samples.append((f[valid,:len(stats[0])],labels,data["session"][ids[valid]],
                            data["times"][ids[valid]]+snapshot["tick"]*.05))
    if not samples:
        return (np.empty((0,len(stats[0])),np.float32),np.empty(0,np.float32),np.empty(0,int),np.empty(0))
    return tuple(np.concatenate([sample[i] for sample in samples]) for i in range(4))


def distill(data,members,out,epochs,dagger_rounds=2,correction_limit=.3,objective="legacy",previous=None):
    features=data["policy"].copy();split=data["split"].copy()
    names=data.get('student_feature_names',FEATURES)
    schema=data.get('student_schema_version',SCHEMA_VERSION)
    labels=np.zeros(len(features),np.float32);improvement=[]
    for begin in range(0,len(labels),32):
        ids=np.arange(begin,min(begin+32,len(labels)))
        teacher_features=np.c_[features[ids],data["x"][ids,0].numpy()+40]
        labels[ids],gain=label_batch(members,teacher_features,DOMAINS[(begin//32)%len(DOMAINS)],phase_for_rows(data,ids),correction_limit=correction_limit,objective=objective)
        improvement.extend(gain.tolist())
        if begin%256==0:print("MPC LABELS",begin,"/",len(labels),flush=True)
    train=split==0
    mean=features[train].mean(0);std=features[train].std(0).clip(.05)
    stats=(torch.tensor(mean),torch.tensor(std));origin=np.zeros(len(features),int)
    sessions=data["session"].copy();times=data["times"].copy()
    validation_ids=np.flatnonzero(data["split"]==1)[::4]
    previous_validation=paired_evaluation(data,validation_ids,members,*previous,correction_limit=correction_limit,objective=objective) if previous else None
    best_rank=None
    best_cost=float("inf");best_state=None;best_size=0;best_round=0;round_reports=[]
    model=None
    for iteration in range(dagger_rounds+1):
        generated=0
        if iteration:
            extra,extra_labels,extra_sessions,extra_times=collect_dagger(data,members,model,stats,iteration,correction_limit,objective)
            generated=len(extra)
            features=np.r_[features,extra];labels=np.r_[labels,extra_labels]
            split=np.r_[split,np.zeros(generated,int)];origin=np.r_[origin,np.full(generated,iteration)]
            sessions=np.r_[sessions,extra_sessions];times=np.r_[times,extra_times]
        model=fit_student(features,labels,split,mean,std,epochs,model.state_dict() if model is not None else None)
        validation=paired_evaluation(data,validation_ids,members,model,stats,correction_limit,objective)
        acceptance=sensor_student_acceptance if len(names)==len(SENSOR_FEATURES) else range_acceptance
        gate=acceptance(validation,previous_validation) if previous_validation else None
        ranking=validation.get('steady_hold',validation) if len(names)==len(SENSOR_FEATURES) else validation
        rank=(not gate["passed"], ranking["student_p95_abs_error_cm"]+ranking["student_max_abs_error_cm"],validation["student_depth_mae_cm"]) if objective=="range" and gate else (validation["student_cost"],)
        round_reports.append({"round":iteration,"new_train_only_simulated_samples":generated,
                              "validation_cost":validation["student_cost"],
                              "validation_cost_improvement_fraction":validation["cost_improvement_fraction"],
                              "validation_range_acceptance":gate,"validation_metrics":validation})
        print("STUDENT ROUND",iteration,"DAGGER",generated,"VALIDATION IMPROVEMENT",round(validation["cost_improvement_fraction"]*100,3),flush=True)
        if best_rank is None or rank<best_rank:
            best_rank=rank
            best_cost=validation["student_cost"];best_state=copy.deepcopy(model.state_dict())
            best_size=len(features);best_round=iteration
    model.load_state_dict(best_state);model.eval()
    features,labels,split,origin,sessions,times=(value[:best_size] for value in (features,labels,split,origin,sessions,times))
    x=torch.tensor((features-mean)/std);y=torch.tensor(labels[:,None])
    torch.save({"state":best_state,"input_mean":mean.tolist(),"input_std":std.tolist(),"feature_names":names,
                "schema_version":schema,"limit":correction_limit,"objective":objective,"enabled":False,"dt":DT,"history":HISTORY,
                "selected_dagger_round":best_round},out/"student.pt")
    torch.save({"inputs":x,"labels":y,"split":torch.tensor(split),"raw_features":torch.tensor(features),
                "dagger_round":torch.tensor(origin),"source_session":torch.tensor(sessions),
                "schema_version":schema},out/"student_windows.pt")
    with (out/"teacher_labels.csv").open("w",newline="",encoding="utf-8") as handle:
        writer=csv.writer(handle);writer.writerow(["source_session","elapsed_s","split","effort_correction_cm_s2","dagger_round","origin"])
        writer.writerows(zip(sessions,times,split,labels,origin,np.where(origin==0,"recorded","simulated_train_only")))
    report={"teacher":"4-second 4-move exhaustive MPC, closed-loop A; 11 randomized physical cases",
            "nonzero_labels":int(np.count_nonzero(labels)),"total_labels":len(labels),
            "mean_candidate_plan_cost_improvement_fraction":float(np.mean(improvement)),"label_limit_cm_s2":correction_limit,"objective":objective,
            "input_dim":features.shape[1],"dagger":{"rounds":round_reports,"selected_round":best_round,
            "selected_simulated_train_samples":int(np.count_nonzero(origin)),
            "selection":"validation only: range acceptance vs A and previous student, then p95+max error; final test after selection" if objective=="range" else "validation paired rollout cost only; test is evaluated after final selection"}}
    for name,value in (("validation",1),("test",2)):
        mask=split==value
        with torch.no_grad():report[name+"_student_teacher_mae_cm_s2"]=(model(x[mask]).clamp(-correction_limit,correction_limit)-y[mask]).abs().mean().item()
    test_ids=np.flatnonzero(data["split"]==2)[::4]
    report["heldout_model_rollout"]=paired_evaluation(data,test_ids,members,model,stats,correction_limit,objective)
    if previous:
        report["previous_student_same_model_test"]=paired_evaluation(data,test_ids,members,*previous,correction_limit=correction_limit,objective=objective)
        acceptance=sensor_student_acceptance if len(names)==len(SENSOR_FEATURES) else range_acceptance
        report["range_acceptance"]=acceptance(report["heldout_model_rollout"],report["previous_student_same_model_test"])
        report["validation_range_acceptance"]=round_reports[best_round]["validation_range_acceptance"]
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--epochs",type=int,default=80)
    parser.add_argument("--dagger-rounds",type=int,default=2)
    parser.add_argument("--objective",choices=("legacy","range"),default="legacy")
    parser.add_argument("--limit",type=float,default=.3)
    parser.add_argument("--previous-model-dir",type=Path)
    parser.add_argument("--csv",action="append",type=Path,help="CSV input; repeat only when intentionally combining sessions");args=parser.parse_args()
    if args.epochs<1 or args.dagger_rounds<0:parser.error("epochs must be positive and dagger-rounds nonnegative")
    if not np.isfinite(args.limit) or args.limit<=0:parser.error("limit must be finite and positive")
    if args.objective=="range" and args.previous_model_dir is None:parser.error("range objective requires a frozen previous model directory")
    torch.set_num_threads(1);np.random.seed(101)
    out=ROOT/"artifacts/hybrid_mpc_v1";out.mkdir(parents=True,exist_ok=True)
    previous_path=out/"training_report.json"
    previous=json.loads(previous_path.read_text(encoding="utf-8")) if previous_path.exists() else None
    deployment_files=[out/name for name in ("proposal_student.espdl", "proposal_student.onnx", "ProposalStudentConfig.h", "espdl_validation.json")]
    unchanged_hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in deployment_files if path.exists()}
    data=read_data(args.csv);split_counts=np.bincount(data["split"],minlength=3)
    previous_student=None
    if args.previous_model_dir:
        checkpoint=torch.load(args.previous_model_dir/"student.pt",map_location="cpu",weights_only=False)
        if checkpoint["feature_names"]!=FEATURES:raise ValueError("Previous student feature mismatch")
        old_model=Student().eval();old_model.load_state_dict(checkpoint["state"])
        previous_student=(old_model,(torch.tensor(checkpoint["input_mean"]),torch.tensor(checkpoint["input_std"])))
    if np.any(split_counts < 20):
        raise ValueError(f"Refusing to overwrite trained artifacts: each temporal split needs at least 20 windows; got {split_counts.tolist()}")
    print("SPLITS",split_counts,"SOURCES",len(data["provenance"]),flush=True)
    with tempfile.TemporaryDirectory(prefix="hybrid_mpc_train_",dir=out.parent) as scratch:
        staged=Path(scratch)
        members,dynamics=fit_dynamics(data,staged,args.epochs)
        student=distill(data,members,staged,250,args.dagger_rounds,args.limit,args.objective,previous_student)
        beats_cv=dynamics["test"]["depth_3s_mae_cm"]<dynamics["test"]["constant_velocity_3s_mae_cm"]*.9
        beats_base=student["heldout_model_rollout"]["student_cost"]<student["heldout_model_rollout"]["base_cost"]*.95
        report={"schema_version":SCHEMA_VERSION,"feature_names":FEATURES,"sources":data["provenance"],"split_counts":split_counts.tolist(),"split_protocol":"Per-session 60/20/20 time blocks with a 3s embargo on both sides of each boundary; every 3s history and prediction window stays within its block", "split_bounds_s":data["split_bounds"],"dynamics":dynamics,"student":student,
                "evaluation_protocol":{"duration_s":4,"control_interval_ms":100,"physics_interval_ms":50,
                "test_selection":"all original eligible test-block windows, stride 4, same order as v1",
                "phase_selection":"deterministic from source session and timestamp; identical for A and student",
                "forward_forecast":"hold the currently observed forward state; no recorded future forward switches in MPC or student rollouts",
                "cost_formula":"integral(e^2 + 0.8*v^2 + 0.2*actually_applied_correction^2 + 0.1*abs(delta_command)) dt + sum(0.1*population_variance(ensemble_depth))",
                "model_selection":"validation only; final test is not used to select a DAgger round",
                "error_metrics":"4-second simulated tracking errors, not real-water steady-state error or settling time"},
                "deployment_gate":{"enabled":False,"dynamics_beats_constant_velocity":beats_cv,"student_model_rollout_beats_A":beats_base,"sufficient_independent_data":False,"open_loop_test_complete":False,"real_water_student_validation":False,"reason":f"Only {len(data['provenance'])} short observational closed-loop session(s); no independent/open-loop plant identification or real-water student validation"},
                "limitations":["within-session temporal splits are correlated and do not establish generalization to a new run", "forward flag and 3s history are inputs, but forward speed/PWM and separate causal excitation are unavailable", "commanded PWM and estimated valves, not actual actuator feedback", "estimated acceleration has filter lag", "initial recorded MPC modulator phase and actuator lock are assumed; DAgger carries simulated controller/actuator state", "4s MPC extrapolates a dynamics model trained on 3s prediction windows", "domain randomization and DAgger samples are simulated, not validation of real operating conditions", "cost v2 fixes the actual-correction penalty and uses population ensemble variance; previous v1 cost numbers are not directly comparable"]}
        report["objective"]=args.objective
        report["evaluation_protocol"]["cost_formula"]=("integral(e^2 + 2.4*v^2 + 3*relu(abs(e)-2)^2 + 0.6*(e+2*v)^2 + 0.2*correction^2 + 0.1*abs(delta_command))dt + 0.3*sum(delta_correction^2) + 1.5*max_abs_error^2 + ensemble uncertainty" if args.objective=="range" else report["evaluation_protocol"]["cost_formula"])
        report["evaluation_protocol"]["error_metrics"]="4-second simulated tracking: MAE and p95 averaged over starting windows; max is worst across all windows, time steps and ensemble members. Not real-water steady-state range."
        report["deployment_gate"]["range_acceptance"]=student.get("range_acceptance")
        report["previous_student_sha256"]=hashlib.sha256((args.previous_model_dir/"student.pt").read_bytes()).hexdigest() if args.previous_model_dir else None
        (staged/"training_report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
        status={"training_sources":len(data["provenance"]),"student_checkpoint_sha256":hashlib.sha256((staged/"student.pt").read_bytes()).hexdigest(),
                "espdl_requires_reexport":True,"firmware_requires_rebuild_after_reexport":True,"robot_firmware_changed":False,
                "existing_export_is_stale":True,"unchanged_deployment_files":unchanged_hashes,
                "schema_version":SCHEMA_VERSION,"student_input_dim":len(FEATURES),"firmware_feature_update_required":True,
                "warning":"New local PT artifacts require matching ESP-DL re-export and firmware build. Robot firmware is unchanged; deployment_report.json remains the authoritative upload record.",
                "previous_costs_directly_comparable":False,
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
