"""Train the depth residual MLP from PC console CSV files."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

from .data import FEATURE_NAMES, load_many, make_windows
from .model import DepthResidualMLP


def _stats(values: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    mean = values.mean(dim=0)
    std = values.std(dim=0, unbiased=False).clamp_min(1.0e-6)
    return mean, std


def main() -> None:
    parser = argparse.ArgumentParser(description="Train depth residuals from bounded PD-teacher labels")
    parser.add_argument("--csv", nargs="+", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--window", type=int, default=5)
    parser.add_argument("--hidden-dims", type=int, nargs=2, default=[24, 12])
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=1.0e-3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--sample-interval-ms", type=int, required=True)
    parser.add_argument("--split-by-session", action="store_true")
    parser.add_argument("--teacher-kp", type=float, required=True,
                        help="teacher residual output points per cm of depth error")
    parser.add_argument("--teacher-kd", type=float, required=True,
                        help="teacher residual output points per cm/s of depth speed")
    args = parser.parse_args()
    if args.epochs < 1 or args.sample_interval_ms < 1:
        raise SystemExit("epochs and sample interval must be positive")
    torch.manual_seed(args.seed)
    torch.set_num_threads(1)

    samples = load_many(args.csv, args.teacher_kp, args.teacher_kd, args.sample_interval_ms)
    features, targets, sessions = make_windows(samples, args.window)
    if len(features) < 32:
        raise SystemExit(f"need at least 32 windows; found {len(features)}")

    split = max(1, min(len(features) - 1, int(len(features) * 0.8)))
    if args.split_by_session:
        while split > 0 and sessions[split - 1] == sessions[split]:
            split -= 1
        if split == 0:
            raise SystemExit("Session holdout needs at least two independent sessions")
    # Adjacent windows share raw frames. Purge the training boundary when
    # validation starts inside the same session, keeping those frames disjoint.
    purge_count = args.window - 1 if sessions[split - 1] == sessions[split] else 0
    train_end = split - purge_count
    if train_end < 1:
        raise SystemExit("not enough independent windows for training and validation")
    x = torch.tensor(features, dtype=torch.float32)
    y = torch.tensor(targets, dtype=torch.float32).reshape(-1, 1)
    x_train, x_eval = x[:train_end], x[split:]
    y_train, y_eval = y[:train_end], y[split:]
    input_mean, input_std = _stats(x_train)
    output_mean, output_std = _stats(y_train)
    x_train = (x_train - input_mean) / input_std
    x_eval = (x_eval - input_mean) / input_std
    y_train_norm = (y_train - output_mean) / output_std
    y_eval_norm = (y_eval - output_mean) / output_std

    model = DepthResidualMLP(x.shape[1], tuple(args.hidden_dims))
    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)
    loss_fn = nn.SmoothL1Loss()
    loader = DataLoader(TensorDataset(x_train, y_train_norm), batch_size=args.batch_size, shuffle=True)

    best_loss = float("inf")
    best_epoch = 0
    best_state = None
    for epoch in range(args.epochs):
        model.train()
        for batch_x, batch_y in loader:
            optimizer.zero_grad()
            loss = loss_fn(model(batch_x), batch_y)
            loss.backward()
            optimizer.step()
        model.eval()
        with torch.no_grad():
            eval_loss = loss_fn(model(x_eval), y_eval_norm).item()
        if eval_loss < best_loss:
            best_loss = eval_loss
            best_epoch = epoch + 1
            best_state = copy.deepcopy(model.state_dict())
        if (epoch + 1) % 50 == 0 or epoch == 0:
            print(f"epoch={epoch + 1} eval_loss={eval_loss:.6f}")

    model.load_state_dict(best_state)
    with torch.no_grad():
        prediction = (model(x_eval) * output_std + output_mean).clamp(-20.0, 20.0)
        error = prediction - y_eval
        eval_mae = error.abs().mean().item()
        eval_rmse = error.square().mean().sqrt().item()
        eval_max_abs_error = error.abs().max().item()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    checkpoint = {
        "model_state": model.state_dict(),
        "input_mean": input_mean.tolist(),
        "input_std": input_std.tolist(),
        "output_mean": float(output_mean.item()),
        "output_std": float(output_std.item()),
        "window": args.window,
        "feature_names": list(FEATURE_NAMES),
        "hidden_dims": list(args.hidden_dims),
        "sample_count": len(features),
        "raw_sample_count": len(samples),
        "train_count": train_end,
        "eval_count": len(features) - split,
        "eval_start_index": split,
        "purged_window_count": purge_count,
        "sample_interval_ms": args.sample_interval_ms,
        "seed": args.seed,
        "best_epoch": best_epoch,
        "eval_loss": best_loss,
        "eval_mae": eval_mae,
        "eval_rmse": eval_rmse,
        "eval_max_abs_error": eval_max_abs_error,
        "sessions": sorted(set(sessions)),
        "teacher_kp": args.teacher_kp,
        "teacher_kd": args.teacher_kd,
        "feature_schema": "depth_pd_v1",
        "validation_split": "whole_session" if args.split_by_session else "purged_chronological",
        "training_sessions": sorted(set(sessions[:train_end])),
        "validation_sessions": sorted(set(sessions[split:])),
        "source_csvs": [str(Path(p)) for p in args.csv],
    }
    torch.save(checkpoint, output_dir / "depth_model.pt")
    (output_dir / "depth_manifest.json").write_text(
        json.dumps({key: value for key, value in checkpoint.items() if key != "model_state"}, indent=2),
        encoding="utf-8",
    )
    print(f"saved {output_dir / 'depth_model.pt'}")
    print(f"best_epoch={best_epoch} eval_mae={eval_mae:.6f} eval_rmse={eval_rmse:.6f}")


if __name__ == "__main__":
    main()
