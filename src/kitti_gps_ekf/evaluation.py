"""Trajectory evaluation: ATE and RPE metrics."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from .alignment import umeyama_alignment, apply_sim3


@dataclass
class MetricSummary:
    rmse: float
    mean: float
    median: float
    std: float
    max_error: float


def align_trajectory_se3(
    estimate: np.ndarray,
    reference: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    rotation, translation, scale = umeyama_alignment(estimate, reference, with_scale=False)
    aligned = apply_sim3(estimate, rotation, translation, scale)
    return aligned, rotation, translation, scale


def compute_ate(
    estimate: np.ndarray,
    reference: np.ndarray,
    align: bool = True,
) -> tuple[MetricSummary, np.ndarray, np.ndarray]:
    estimate = np.asarray(estimate, dtype=np.float64)
    reference = np.asarray(reference, dtype=np.float64)
    n = min(len(estimate), len(reference))
    estimate = estimate[:n]
    reference = reference[:n]

    if align:
        estimate, _, _, _ = align_trajectory_se3(estimate, reference)

    errors = np.linalg.norm(estimate - reference, axis=1)
    summary = MetricSummary(
        rmse=float(np.sqrt(np.mean(errors ** 2))),
        mean=float(np.mean(errors)),
        median=float(np.median(errors)),
        std=float(np.std(errors)),
        max_error=float(np.max(errors)),
    )
    return summary, errors, estimate


def _relative_transform(
    pos_i: np.ndarray,
    quat_i: np.ndarray,
    pos_j: np.ndarray,
    quat_j: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    rot_i = Rotation.from_quat(quat_i).as_matrix()
    rot_j = Rotation.from_quat(quat_j).as_matrix()
    delta_rot = rot_i.T @ rot_j
    delta_trans = rot_i.T @ (pos_j - pos_i)
    delta_quat = Rotation.from_matrix(delta_rot).as_quat()
    return delta_trans, delta_quat


def compute_rpe(
    estimate_positions: np.ndarray,
    estimate_quaternions: np.ndarray,
    reference_positions: np.ndarray,
    reference_quaternions: np.ndarray,
    delta: int = 1,
    align: bool = True,
) -> MetricSummary:
    n = min(len(estimate_positions), len(reference_positions))
    est_pos = estimate_positions[:n]
    est_quat = estimate_quaternions[:n]
    ref_pos = reference_positions[:n]
    ref_quat = reference_quaternions[:n]

    if align:
        est_pos, _, _, _ = align_trajectory_se3(est_pos, ref_pos)

    trans_errors = []
    rot_errors = []
    for i in range(0, n - delta):
        est_rel_t, est_rel_q = _relative_transform(est_pos[i], est_quat[i], est_pos[i + delta], est_quat[i + delta])
        ref_rel_t, ref_rel_q = _relative_transform(ref_pos[i], ref_quat[i], ref_pos[i + delta], ref_quat[i + delta])
        trans_errors.append(np.linalg.norm(est_rel_t - ref_rel_t))
        est_rot = Rotation.from_quat(est_rel_q)
        ref_rot = Rotation.from_quat(ref_rel_q)
        rot_errors.append((est_rot.inv() * ref_rot).magnitude())

    trans_errors = np.asarray(trans_errors)
    rot_errors = np.asarray(rot_errors)
    combined = np.sqrt(trans_errors ** 2 + rot_errors ** 2)
    return MetricSummary(
        rmse=float(np.sqrt(np.mean(combined ** 2))),
        mean=float(np.mean(combined)),
        median=float(np.median(combined)),
        std=float(np.std(combined)),
        max_error=float(np.max(combined)),
    )


def evaluate_trajectory(
    estimate_positions: np.ndarray,
    estimate_quaternions: np.ndarray,
    reference_positions: np.ndarray,
    reference_quaternions: np.ndarray,
    timestamps: np.ndarray | None = None,
    rpe_deltas: list[int] | None = None,
) -> tuple[dict, pd.DataFrame, pd.DataFrame]:
    if rpe_deltas is None:
        rpe_deltas = [1, 10, 100]

    ate_summary, ate_errors, aligned = compute_ate(estimate_positions, reference_positions)
    rpe_rows = []
    for delta in rpe_deltas:
        summary = compute_rpe(
            estimate_positions,
            estimate_quaternions,
            reference_positions,
            reference_quaternions,
            delta=delta,
            align=True,
        )
        rpe_rows.append(
            {
                "delta_frames": delta,
                "rmse": summary.rmse,
                "mean": summary.mean,
                "median": summary.median,
                "std": summary.std,
                "max_error": summary.max_error,
            }
        )

    ate_df = pd.DataFrame(
        {
            "frame": np.arange(len(ate_errors)),
            "ate_error_m": ate_errors,
            "timestamp": timestamps[: len(ate_errors)] if timestamps is not None else np.arange(len(ate_errors)),
        }
    )
    rpe_df = pd.DataFrame(rpe_rows)
    metrics = {"ate": ate_summary, "rpe": {row["delta_frames"]: row for row in rpe_rows}}
    return metrics, ate_df, rpe_df
