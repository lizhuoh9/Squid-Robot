"""Five-feature CSV loading and sliding windows for depth residual learning."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

FEATURE_NAMES = (
    "depth_err_cm", "depth_speed_cm_s", "depth_accel_cm_s2",
    "u_base", "buoyancy_pwm_commanded",
)


@dataclass
class Sample:
    session_id: str
    timestamp_ms: float
    dt_ms: float
    features: list[float]
    residual: float


def _number(row: dict[str, str], *names: str) -> float | None:
    for name in names:
        try:
            value = float(row.get(name, ""))
        except (ValueError, TypeError):
            continue
        if math.isfinite(value):
            return value
    return None


def load_csv(path: str | Path, teacher_kp: float, teacher_kd: float,
             sample_interval_ms: int = 500) -> list[Sample]:
    """Read PD teacher samples; preserve prepared CSV session boundaries."""
    if sample_interval_ms < 1:
        raise ValueError("sample interval must be positive")
    path = Path(path)
    samples: list[Sample] = []
    previous_timestamp = None
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError(f"{path}: CSV has no header")
        for row in reader:
            valid = _number(row, "depth_valid")
            if valid is not None and valid < 0.5:
                continue
            values = (
                _number(row, "elapsed_s"),
                _number(row, "filtered_depth_cm", "depth_cm"),
                _number(row, "depth_speed_cm_s"),
                _number(row, "depth_accel_cm_s2"),
                _number(row, "target_depth_cm"),
                _number(row, "u_base", "pid_base"),
                _number(row, "buoyancy_pwm_commanded", "buoyancy_pwm"),
            )
            if any(value is None for value in values):
                continue
            elapsed, depth, speed, accel, target, base, pwm = values
            if not 0 <= target <= 100:
                continue
            timestamp = elapsed * 1000.0
            if previous_timestamp is not None and timestamp <= previous_timestamp:
                continue
            dt = timestamp - previous_timestamp if previous_timestamp is not None else float(sample_interval_ms)
            previous_timestamp = timestamp
            error = target - depth
            residual = max(-20.0, min(20.0, teacher_kp * error - teacher_kd * speed))
            samples.append(Sample(str(row.get("session_id") or path.stem), timestamp, dt,
                                  [error, speed, accel, base, pwm], residual))
    if not samples:
        raise ValueError(f"{path}: no valid depth samples after filtering")
    return samples


def load_many(paths: Iterable[str | Path], teacher_kp: float, teacher_kd: float,
              sample_interval_ms: int = 500) -> list[Sample]:
    samples: list[Sample] = []
    for path in paths:
        samples.extend(load_csv(path, teacher_kp, teacher_kd, sample_interval_ms))
    return samples


def make_windows(samples: list[Sample], window: int = 5) -> tuple[list[list[float]], list[float], list[str]]:
    """Create chronological windows without crossing session boundaries."""
    if window < 1:
        raise ValueError("window must be positive")
    features: list[list[float]] = []
    targets: list[float] = []
    sessions: list[str] = []
    by_session: dict[str, list[Sample]] = {}
    for sample in samples:
        by_session.setdefault(sample.session_id, []).append(sample)
    for session_id, rows in by_session.items():
        rows.sort(key=lambda row: row.timestamp_ms)
        for index in range(window - 1, len(rows)):
            history = rows[index - window + 1:index + 1]
            features.append([value for row in history for value in row.features])
            targets.append(rows[index].residual)
            sessions.append(session_id)
    if not features:
        raise ValueError(f"not enough samples for a {window}-frame window")
    return features, targets, sessions
