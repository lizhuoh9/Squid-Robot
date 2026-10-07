"""Learn dynamics, search short-horizon corrections and distill an ESP-DL policy."""
from __future__ import annotations

import argparse
import copy
import csv
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

from ..model import DepthResidualMLP
from .data import DT, FEATURE_NAMES, WINDOW, Dataset, effort, read_dataset, split_sessions


def metric(prediction, truth):
    delta = np.asarray(prediction) - np.asarray(truth)
    return {"mae": np.mean(np.abs(delta), axis=0).tolist(),
            "rmse": np.sqrt(np.mean(delta ** 2, axis=0)).tolist()}


def fit(model, x, y, train, validation, epochs, seed, bootstrap=False):
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    train_ids = np.flatnonzero(train)
    if bootstrap:
        train_ids = rng.choice(train_ids, len(train_ids), replace=True)
    xmean, xstd = x[train].mean(0), x[train].std(0).clip(1.0e-6)
    ymean, ystd = y[train].mean(0), y[train].std(0).clip(1.0e-6)
    xt = torch.tensor((x - xmean) / xstd)
    yt = torch.tensor((y - ymean) / ystd)
    optimizer = torch.optim.Adam(model.parameters(), lr=1.0e-3, weight_decay=1.0e-4)
    best, best_epoch, state = float("inf"), 0, None
    for epoch in range(epochs):
        model.train()
        for ids in np.array_split(rng.permutation(train_ids), max(1, len(train_ids) // 64)):
            optimizer.zero_grad()
            loss = nn.functional.smooth_l1_loss(model(xt[ids]), yt[ids])
            loss.backward()
            optimizer.step()
        model.eval()
        with torch.no_grad():
            val = nn.functional.mse_loss(model(xt[validation]), yt[validation]).item()
        if val < best:
            best, best_epoch, state = val, epoch + 1, copy.deepcopy(model.state_dict())
    model.load_state_dict(state)
    model.eval()
    return {"model_state": state, "input_mean": xmean.tolist(), "input_std": xstd.tolist(),
            "output_mean": ymean.tolist(), "output_std": ystd.tolist(), "best_epoch": best_epoch}


class Ensemble:
    def __init__(self, members):
        self.members = members

    @torch.no_grad()
    def predict(self, x):
        x = torch.tensor(np.asarray(x, dtype=np.float32))
        predictions = []
        for model, stats in self.members:
            normalized = (x - torch.tensor(stats["input_mean"])) / torch.tensor(stats["input_std"])
            predictions.append((model(normalized) * torch.tensor(stats["output_std"])
                                + torch.tensor(stats["output_mean"])).numpy())
        values = np.stack(predictions)
        return values.mean(0), values.std(0)


def rollout(ensemble, states, corrections, horizon=6):
    """Local held-base/held-correction shooting, NOT a full PID closed-loop replay."""
    state = states.copy()
    errors, speed, accel, base = [state[:, i].copy() for i in range(4)]
    actions = state[:, 4:7].copy()
    cost = np.zeros(len(state), np.float32)
    total = np.clip(base + corrections, -100, 100)
    drive = effort(total)
    for step in range(horizon):
        # Replace current action only; older commands retain their causal order.
        actions[:, -1] = drive
        x = np.column_stack([speed, accel, actions])
        change, uncertainty = ensemble.predict(x)
        errors -= change[:, 0]
        speed += change[:, 1]
        accel = np.clip(change[:, 1] / DT, -20, 20)
        cost += (errors ** 2 + 2.0 * speed ** 2 + 5.0 * np.sum(uncertainty ** 2, axis=1))
        actions = np.column_stack([actions[:, 1:], drive])
    cost += 2.0 * errors ** 2 + 0.2 * (corrections / 20.0) ** 2
    return cost


def teacher(ensemble, dataset, dynamic_train, train_dynamics):
    candidates = np.arange(-20, 21, 2, dtype=np.float32)
    choices = np.repeat(dataset.state[:, None, :], len(candidates), axis=1).reshape(-1, 7)
    residuals = np.tile(candidates, len(dataset.state))
    scores = rollout(ensemble, choices, residuals).reshape(-1, len(candidates))
    labels = candidates[np.argmin(scores, axis=1)]
    zero_cost = scores[:, len(candidates) // 2]
    improvement = zero_cost - scores.min(1)
    # Conservative support check: nearest observed training state/action, not
    # validation/test outcomes. One-step uncertainty is not a safety certificate.
    candidate_inputs = np.column_stack([dataset.state[:, 1:3], dataset.state[:, 4:6],
                                       effort(np.clip(dataset.state[:, 3] + labels, -100, 100))])
    mean = train_dynamics[dynamic_train].mean(0)
    std = train_dynamics[dynamic_train].std(0).clip(0.1)
    bank = (train_dynamics[dynamic_train] - mean) / std
    nearest = []
    for chunk in np.array_split((candidate_inputs - mean) / std, max(1, len(labels) // 100)):
        nearest.extend(np.sqrt(((chunk[:, None, :] - bank[None, :, :]) ** 2).mean(2)).min(1))
    supported = np.array(nearest) <= 1.5
    near = (np.abs(dataset.state[:, 0]) <= 15) & (np.abs(dataset.state[:, 1]) <= 5)
    useful = improvement > 0.01 * np.maximum(zero_cost, 1.0)
    enabled = supported & near & useful
    labels = np.where(enabled, labels, 0).astype(np.float32)
    return labels, {"enabled_count": int(enabled.sum()), "total_count": len(labels),
                    "unsupported_count": int((~supported).sum()),
                    "label_min": float(labels.min()), "label_max": float(labels.max()),
                    "mean_predicted_cost_reduction_enabled": float(improvement[enabled].mean()) if enabled.any() else 0.0}


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-csv", nargs="+", type=Path, required=True)
    parser.add_argument("--test-csv", nargs="+", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=400)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.epochs < 1 or set(p.resolve() for p in args.train_csv) & set(p.resolve() for p in args.test_csv):
        raise ValueError("Positive epochs and disjoint train/test CSVs required")
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    data = read_dataset(args.train_csv)
    test = read_dataset(args.test_csv)
    train, validation = split_sessions(data)
    out = args.output_dir
    if out.exists() and any(out.iterdir()):
        raise ValueError("Output directory is nonempty; choose a new path to preserve prior artifacts")
    out.mkdir(parents=True, exist_ok=True)
    members, saved = [], []
    for member in range(3):
        torch.manual_seed(args.seed + member)
        model = nn.Sequential(nn.Linear(5, 32), nn.ReLU(), nn.Linear(32, 16), nn.ReLU(), nn.Linear(16, 2))
        stats = fit(model, data.dynamics, data.outcome, train, validation, args.epochs, args.seed + member, True)
        members.append((model, stats))
        saved.append(stats)
        print(f"dynamics member={member} best_epoch={stats['best_epoch']}", flush=True)
    ensemble = Ensemble(members)
    validation_predictions, _ = ensemble.predict(data.dynamics[validation])
    test_predictions, _ = ensemble.predict(test.dynamics)
    baseline = np.column_stack([data.dynamics[validation, 0] * DT, np.zeros(validation.sum())])
    test_baseline = np.column_stack([test.dynamics[:, 0] * DT, np.zeros(len(test.outcome))])
    validation_metrics = metric(validation_predictions, data.outcome[validation])
    baseline_metrics = metric(baseline, data.outcome[validation])
    validated = sum(validation_metrics["rmse"]) < sum(baseline_metrics["rmse"])
    labels, teacher_report = teacher(ensemble, data, train, data.dynamics)
    # Test teacher uses the FROZEN training ensemble, not test outcomes.
    test_labels, test_teacher_report = teacher(ensemble, test, train, data.dynamics)
    print(f"teacher enabled={teacher_report['enabled_count']}/{len(labels)} dynamics_validated={validated}", flush=True)
    if not validated or np.std(labels[train]) < 0.1:
        write_json(out / "failed_validation.json", {"dynamics": validation_metrics, "baseline": baseline_metrics,
                                                    "teacher": teacher_report})
        raise RuntimeError("Dynamics did not beat validation baseline or teacher is degenerate; refusing policy promotion")
    torch.save({"members": saved, "input_names": ["speed", "acceleration", "effort_t_minus_2", "effort_t_minus_1", "effort_t"],
                "output_names": ["delta_depth_cm", "delta_speed_cm_s"], "sample_interval_ms": 500}, out / "dynamics_model.pt")
    torch.manual_seed(args.seed)
    policy = DepthResidualMLP(25, (24, 12))
    stats = fit(policy, data.policy, labels[:, None], train, validation, args.epochs, args.seed)
    stats["output_mean"] = stats["output_mean"][0]
    stats["output_std"] = stats["output_std"][0]
    with torch.no_grad():
        def predict(x):
            z = (torch.tensor(x) - torch.tensor(stats["input_mean"])) / torch.tensor(stats["input_std"])
            return (policy(z) * stats["output_std"] + stats["output_mean"]).clamp(-20, 20).numpy().reshape(-1)
        val_policy, test_policy = predict(data.policy[validation]), predict(test.policy)
    checkpoint = {**stats, "window": 5, "feature_names": FEATURE_NAMES, "hidden_dims": [24, 12],
                  "sample_interval_ms": 500, "feature_schema": "model_based_base_pwm_v1", "seed": args.seed,
                  "sample_count": len(data.policy), "train_count": int(train.sum()), "eval_count": int(validation.sum()),
                  "training_sources": data.sources, "test_sources": test.sources, "residual_limit": 20.0,
                  "guard_error_cm": 15.0, "guard_speed_cm_s": 5.0, "guard_normalized_abs": 6.0}
    torch.save(checkpoint, out / "depth_model.pt")
    write_json(out / "depth_manifest.json", {k: v for k, v in checkpoint.items() if k != "model_state"})
    torch.save({"train_inputs": torch.tensor(data.policy[train]), "validation_inputs": torch.tensor(data.policy[validation]),
                "validation_labels": torch.tensor(labels[validation]), "test_inputs": torch.tensor(test.policy),
                "test_labels": torch.tensor(test_labels)}, out / "policy_windows.pt")
    with (out / "teacher_labels.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=[*data.rows[0].keys(), "split", "residual_label"])
        writer.writeheader()
        for i, row in enumerate(data.rows):
            writer.writerow({**row, "split": "train" if train[i] else "validation", "residual_label": float(labels[i])})
    report = {"method": "bootstrap neural dynamics ensemble -> six-step held-action search -> bounded MLP distillation",
              "training_transitions": int(train.sum()), "validation_transitions": int(validation.sum()),
              "test_transitions": len(test.policy), "training_sessions": sorted(set(np.array(data.sessions)[train])),
              "validation_sessions": sorted(set(np.array(data.sessions)[validation])),
              "dynamics_validation": validation_metrics, "constant_velocity_validation": baseline_metrics,
              "dynamics_test": metric(test_predictions, test.outcome), "constant_velocity_test": metric(test_baseline, test.outcome),
              "teacher": teacher_report, "test_teacher": test_teacher_report,
              "policy_validation": metric(val_policy[:, None], labels[validation, None]),
              "policy_test": metric(test_policy[:, None], test_labels[:, None]),
              "closed_loop_accuracy_measured": False,
              "limitations": ["500 ms output snapshots approximate interval actions; high-rate output is unobserved",
                              "All supplied sessions have forward_active=0; moving-mode accuracy is unvalidated",
                              "Teacher holds PID base fixed over 3 s, not a full closed-loop PID simulation",
                              "Bootstrap disagreement and support checks do not prove counterfactual action accuracy",
                              "Predictive cost reductions are simulated, not measured depth improvements"]}
    write_json(out / "training_report.json", report)
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
