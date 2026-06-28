"""GPS sampling utilities to simulate sparse GPS updates."""

from __future__ import annotations

import numpy as np


def sample_gps_every_distance(
    east: np.ndarray,
    north: np.ndarray,
    up: np.ndarray,
    sample_distance_m: float,
) -> np.ndarray:
    """Return indices where cumulative horizontal path length exceeds sample_distance."""
    positions = np.column_stack([east, north, up])
    if len(positions) < 2:
        return np.array([0], dtype=int)

    diffs = np.diff(positions, axis=0)
    segment_lengths = np.linalg.norm(diffs[:, :2], axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(segment_lengths)])

    indices = [0]
    next_target = sample_distance_m
    for idx, dist in enumerate(cumulative):
        if dist >= next_target:
            indices.append(idx)
            next_target += sample_distance_m

    if indices[-1] != len(positions) - 1:
        indices.append(len(positions) - 1)
    return np.unique(np.array(indices, dtype=int))


def build_sparse_gps_mask(
    east: np.ndarray,
    north: np.ndarray,
    up: np.ndarray,
    sample_distance_m: float,
) -> np.ndarray:
    """Boolean mask indicating frames where GPS measurements are available."""
    mask = np.zeros(len(east), dtype=bool)
    mask[sample_gps_every_distance(east, north, up, sample_distance_m)] = True
    return mask
