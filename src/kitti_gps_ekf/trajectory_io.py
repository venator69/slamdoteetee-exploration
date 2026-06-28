"""Trajectory I/O utilities."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def load_times(sequence_dir: Path) -> np.ndarray:
    times = []
    for line in (sequence_dir / "times.txt").read_text().strip().splitlines():
        if line.strip():
            times.append(float(line.split()[0]))
    return np.asarray(times, dtype=np.float64)


def kitti_pose_matrix_to_position_quaternion(matrix_row: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convert KITTI 3x4 pose row to position and quaternion [qx,qy,qz,qw]."""
    rotation = matrix_row.reshape(3, 4)[:, :3]
    position = matrix_row.reshape(3, 4)[:, 3]
    quat = Rotation.from_matrix(rotation).as_quat()
    return position, quat


def load_kitti_trajectory(
    traj_file: Path,
    timestamps: np.ndarray | None = None,
    latency: float = 0.0,
) -> dict[str, np.ndarray]:
    """Load ORB-SLAM3 KITTI-format trajectory (12 values per line)."""
    rows = []
    for line in traj_file.read_text().strip().splitlines():
        if line.strip():
            rows.append(np.fromstring(line, sep=" "))
    poses = np.asarray(rows, dtype=np.float64)
    positions = []
    quaternions = []
    for row in poses:
        pos, quat = kitti_pose_matrix_to_position_quaternion(row)
        positions.append(pos)
        quaternions.append(quat)

    n = len(positions)
    if timestamps is None:
        timestamps = np.arange(n, dtype=np.float64)
    else:
        timestamps = timestamps[:n]

    return {
        "timestamp": timestamps,
        "position": np.asarray(positions),
        "quaternion": np.asarray(quaternions),
        "latency": np.full(n, latency, dtype=np.float64),
    }


def load_pose_trajectory_txt(traj_file: Path) -> dict[str, np.ndarray]:
    """
    Load trajectory text file with columns:
    timestamp x y z qx qy qz qw [latency]
    """
    lines = [line for line in traj_file.read_text().strip().splitlines() if line.strip()]
    numeric_lines = []
    for line in lines:
        if line.strip().startswith("#") or line.strip().startswith("timestamp"):
            continue
        values = np.fromstring(line, sep=" ")
        if values.size >= 8:
            numeric_lines.append(values)
    data = np.asarray(numeric_lines, dtype=np.float64)
    if data.ndim == 1:
        data = data[None, :]
    result = {
        "timestamp": data[:, 0],
        "position": data[:, 1:4],
        "quaternion": data[:, 4:8],
    }
    if data.shape[1] >= 9:
        result["latency"] = data[:, 8]
    return result


def load_trajectory_auto(traj_file: Path, timestamps: np.ndarray | None = None) -> dict[str, np.ndarray]:
    """Load ORB-SLAM3 KITTI (12 cols) or pose text (8+ cols) trajectory."""
    first_data_line = None
    for line in traj_file.read_text().strip().splitlines():
        if not line.strip() or line.strip().startswith("#") or line.strip().startswith("timestamp"):
            continue
        values = np.fromstring(line, sep=" ")
        if values.size >= 8:
            first_data_line = values
            break
    if first_data_line is None:
        raise ValueError(f"No valid trajectory data in {traj_file}")
    if first_data_line.size == 12:
        return load_kitti_trajectory(traj_file, timestamps=timestamps)
    slam = load_pose_trajectory_txt(traj_file)
    if timestamps is not None:
        slam["timestamp"] = timestamps[: len(slam["position"])]
    return slam


def save_pose_trajectory_txt(
    output_file: Path,
    timestamps: np.ndarray,
    positions: np.ndarray,
    quaternions: np.ndarray,
    latency: np.ndarray | None = None,
) -> None:
    output_file.parent.mkdir(parents=True, exist_ok=True)
    n = len(timestamps)
    if latency is None:
        latency = np.zeros(n, dtype=np.float64)
    data = np.column_stack([timestamps, positions, quaternions, latency])
    np.savetxt(output_file, data, fmt="%.9f", header="timestamp x y z qx qy qz qw latency", comments="# ")


def save_gps_trajectory_txt(
    output_file: Path,
    timestamps: np.ndarray,
    east: np.ndarray,
    north: np.ndarray,
    up: np.ndarray,
) -> None:
    output_file.parent.mkdir(parents=True, exist_ok=True)
    data = np.column_stack([timestamps, east, north, up])
    np.savetxt(output_file, data, fmt="%.9f")


def load_kitti_ground_truth(poses_file: Path) -> dict[str, np.ndarray]:
    """Load KITTI odometry ground truth poses."""
    rows = []
    for line in poses_file.read_text().strip().splitlines():
        if line.strip():
            rows.append(np.fromstring(line, sep=" "))
    poses = np.asarray(rows, dtype=np.float64)
    positions = []
    quaternions = []
    for row in poses:
        pos, quat = kitti_pose_matrix_to_position_quaternion(row)
        positions.append(pos)
        quaternions.append(quat)
    return {
        "position": np.asarray(positions),
        "quaternion": np.asarray(quaternions),
    }
