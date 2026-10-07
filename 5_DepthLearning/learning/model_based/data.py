"""Causal transitions; no future observations appear in policy inputs."""
from __future__ import annotations

import csv
import hashlib
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FEATURE_NAMES = ["depth_err_cm", "depth_speed_cm_s", "depth_accel_cm_s2",
                 "u_base", "base_pwm_commanded"]
WINDOW = 5
DT = 0.5


def base_pwm(output):
    """Same rounded 8-point trigger and 80..255 PWM mapping as firmware."""
    output = np.asarray(output)
    magnitude = np.minimum(np.abs(output), 100.0)
    return np.where(magnitude < 8.0, 0.0,
                    np.floor(80.0 + (magnitude - 8.0) * 175.0 / 92.0 + 0.5))


def effort(output):
    return np.sign(output) * base_pwm(output) / 255.0


@dataclass
class Dataset:
    policy: np.ndarray
    dynamics: np.ndarray
    outcome: np.ndarray
    state: np.ndarray
    sessions: list[str]
    rows: list[dict]
    sources: list[dict]


def read_dataset(paths: list[Path]) -> Dataset:
    policies, dynamics, outcomes, states, sessions, audit = [], [], [], [], [], []
    sources = []
    for path in paths:
        path = path.resolve()
        sources.append({"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
        history = []
        segment = 0
        previous = None
        with path.open(encoding="utf-8-sig", newline="") as handle:
            for line, row in enumerate(csv.DictReader(handle), 2):
                keys = ["elapsed_s", "depth_cm", "depth_speed_cm_s", "depth_accel_cm_s2",
                        "target_depth_cm", "pid_base", "pid_output", "buoyancy_pwm", "forward_active"]
                try:
                    t, depth, speed, accel, target, base, output, pwm, forward = [float(row[k]) for k in keys]
                except (ValueError, KeyError, TypeError):
                    history, previous = [], None
                    segment += 1
                    continue
                valid = (all(math.isfinite(x) for x in (t, depth, speed, accel, target, base, output, pwm, forward))
                         and 0 < target < 100 and 0 <= depth < 100 and abs(base) <= 80.01
                         and abs(output) <= 100.01 and 0 <= pwm <= 255 and forward == 0
                         and abs(speed) <= 10 and abs(accel) <= 20)
                if not valid:
                    history, previous = [], None
                    segment += 1
                    continue
                if previous is not None and (target != previous[4] or not 0.40 <= t - previous[0] <= 0.60):
                    history = []
                    segment += 1
                current = (t, depth, speed, accel, target, base, output, pwm)
                # The logged command is treated as applied until the next telemetry
                # sample. High-rate controller changes are unobserved: record this
                # approximation rather than claiming exact system identification.
                if len(history) >= WINDOW:
                    past = history[-WINDOW:]
                    last = past[-1]
                    frame = [[r[4] - r[1], r[2], r[3], r[5], float(base_pwm(r[5]))] for r in past]
                    action_history = [float(np.sign(r[6]) * r[7] / 255.0) for r in past[-3:]]
                    policies.append(np.asarray(frame).reshape(-1))
                    dynamics.append([last[2], last[3], *action_history])
                    elapsed = t - last[0]
                    outcomes.append([(depth - last[1]) * DT / elapsed, (speed - last[2]) * DT / elapsed])
                    states.append([last[4] - last[1], last[2], last[3], last[5], *action_history])
                    session = f"{path.stem}-segment{segment:03d}-target{target:g}"
                    sessions.append(session)
                    audit.append({"source": str(path), "input_last_line": line - 1, "outcome_line": line,
                                  "session": session, "elapsed_s": last[0], "depth_cm": last[1],
                                  "target_depth_cm": last[4], "pid_base": last[5], "pid_output": last[6]})
                history.append(current)
                previous = current
    if not policies:
        raise ValueError("No valid five-frame transitions at 500 ms")
    return Dataset(*(np.asarray(x, dtype=np.float32) for x in (policies, dynamics, outcomes, states)),
                   sessions, audit, sources)


def split_sessions(dataset: Dataset):
    unique = list(dict.fromkeys(dataset.sessions))
    if len(unique) < 3:
        raise ValueError("At least three independent target/gap segments are required")
    validation_sessions = set(unique[max(1, int(len(unique) * 0.8)):])
    validation = np.array([s in validation_sessions for s in dataset.sessions])
    train = ~validation
    if train.sum() < 100 or validation.sum() < 30:
        raise ValueError("Insufficient train/validation transitions")
    return train, validation
