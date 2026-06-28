"""Timestamp synchronization utilities."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class SyncPair:
    source_idx: int
    target_idx: int
    time_diff: float


def nearest_neighbor_sync(
    source_times: np.ndarray,
    target_times: np.ndarray,
    max_time_diff: float,
) -> list[SyncPair]:
    """Match timestamps using nearest-neighbor with a maximum difference threshold."""
    pairs: list[SyncPair] = []
    used_targets: set[int] = set()

    for src_idx, src_time in enumerate(source_times):
        diffs = np.abs(target_times - src_time)
        tgt_idx = int(np.argmin(diffs))
        min_diff = float(diffs[tgt_idx])
        if min_diff <= max_time_diff and tgt_idx not in used_targets:
            pairs.append(SyncPair(src_idx, tgt_idx, min_diff))
            used_targets.add(tgt_idx)
    return pairs


def sync_arrays_by_pairs(
    source_array: np.ndarray,
    target_array: np.ndarray,
    pairs: list[SyncPair],
) -> tuple[np.ndarray, np.ndarray]:
    src = np.array([source_array[p.source_idx] for p in pairs])
    tgt = np.array([target_array[p.target_idx] for p in pairs])
    return src, tgt
