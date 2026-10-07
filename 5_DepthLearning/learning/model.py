"""The depth-only MLP used for offline training and firmware export."""

from __future__ import annotations

import torch
from torch import nn


class DepthResidualMLP(nn.Module):
    def __init__(self, input_dim: int = 25, hidden_dims: tuple[int, int] = (24, 12)) -> None:
        super().__init__()
        first, second = hidden_dims
        self.net = nn.Sequential(
            nn.Linear(input_dim, first),
            nn.ReLU(),
            nn.Linear(first, second),
            nn.ReLU(),
            nn.Linear(second, 1),
        )

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.net(inputs)
