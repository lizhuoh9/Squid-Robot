"""Read-only causal data, split, export, and firmware-input contract checks."""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from .data import FEATURE_NAMES, base_pwm, effort, read_dataset, split_sessions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", type=Path, required=True)
    args = parser.parse_args()
    out = args.model_dir
    checkpoint = torch.load(out / "depth_model.pt", map_location="cpu", weights_only=False)
    for source in checkpoint["training_sources"] + checkpoint["test_sources"]:
        assert hashlib.sha256(Path(source["path"]).read_bytes()).hexdigest() == source["sha256"]
    data = read_dataset([Path(s["path"]) for s in checkpoint["training_sources"]])
    train, validation = split_sessions(data)
    windows = torch.load(out / "policy_windows.pt", map_location="cpu", weights_only=False)
    assert set(np.array(data.sessions)[train]).isdisjoint(set(np.array(data.sessions)[validation]))
    assert np.array_equal(data.policy[train], windows["train_inputs"].numpy())
    assert np.array_equal(data.policy[validation], windows["validation_inputs"].numpy())
    assert checkpoint["feature_names"] == FEATURE_NAMES
    assert np.array_equal(base_pwm(np.array([-100, -80, -8, -7.99, 0, 7.99, 8, 80, 100])),
                          [255, 217, 80, 0, 0, 0, 80, 217, 255])
    assert np.all(np.abs(effort(np.linspace(-100, 100, 1001))) <= 1)
    for i, row in enumerate(data.rows):
        assert row["input_last_line"] < row["outcome_line"]
        frames = data.policy[i].reshape(5, 5)
        assert np.array_equal(frames[:, 4], base_pwm(frames[:, 3]))
    for file in Path(__file__).parent.glob("*.py"):
        ast.parse(file.read_text(encoding="utf-8"))
    report = json.loads((out / "depth_model_espdl.validation.json").read_text(encoding="utf-8"))
    assert report["output_representable_range"][0] <= -20
    assert report["output_representable_range"][1] >= 20
    assert "#define SQUID_DEPTH_MODEL_BASED 1" in (out / "DepthResidualModelConfig.h").read_text(encoding="utf-8")
    print("PASS: source hashes, causal windows, disjoint sessions, base PWM mapping, Python syntax, INT8 +/-20 range")


if __name__ == "__main__":
    main()
