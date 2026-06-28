"""Umeyama Sim(3)/SE(3) alignment between trajectories."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def umeyama_alignment(
    source: np.ndarray,
    target: np.ndarray,
    with_scale: bool = False,
) -> tuple[np.ndarray, np.ndarray, float]:
    """
    Estimate transform target ≈ scale * R @ source + t using Umeyama (1991).

    Returns rotation R (3x3), translation t (3,), and scale s.
    """
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if source.shape != target.shape or source.shape[1] != 3:
        raise ValueError("source and target must be Nx3 arrays with same shape")

    n = source.shape[0]
    mu_src = source.mean(axis=0)
    mu_tgt = target.mean(axis=0)
    src_centered = source - mu_src
    tgt_centered = target - mu_tgt

    cov = (tgt_centered.T @ src_centered) / n
    u, singular_values, vt = np.linalg.svd(cov)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1
        rotation = u @ vt

    if with_scale:
        var_src = np.sum(src_centered ** 2) / n
        scale = float(singular_values.sum() / var_src) if var_src > 0 else 1.0
    else:
        scale = 1.0

    translation = mu_tgt - scale * rotation @ mu_src
    return rotation, translation, scale


def apply_sim3(
    points: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
    scale: float = 1.0,
) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    return (scale * (rotation @ points.T).T) + translation


def align_poses_to_enu(
    slam_positions: np.ndarray,
    gps_positions: np.ndarray,
    with_scale: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Align SLAM positions to GPS ENU frame."""
    rotation, translation, scale = umeyama_alignment(
        slam_positions, gps_positions, with_scale=with_scale
    )
    aligned = apply_sim3(slam_positions, rotation, translation, scale)
    return aligned, rotation, translation, scale


def transform_orientation(
    quaternions: np.ndarray,
    rotation: np.ndarray,
) -> np.ndarray:
    """Apply fixed world rotation to SLAM orientations."""
    r_align = Rotation.from_matrix(rotation)
    transformed = []
    for quat in quaternions:
        r_pose = Rotation.from_quat(quat)
        r_new = r_align * r_pose
        transformed.append(r_new.as_quat())
    return np.asarray(transformed, dtype=np.float64)
