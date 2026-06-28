#!/usr/bin/env python3
"""Compare ORB-SLAM3, EKF fusion, and GPS against KITTI ground truth."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from kitti_gps_ekf.pipeline import load_config, run_fusion_for_mode
from kitti_gps_ekf.trajectory_io import load_kitti_ground_truth, load_times, save_pose_trajectory_txt


def _is_kitti_trajectory(traj_file: Path) -> bool:
    for line in traj_file.read_text().strip().splitlines():
        if not line.strip():
            continue
        values = np.fromstring(line, sep=" ")
        return values.size == 12
    return False


def resolve_orbslam3_trajectory(config: dict, sequence: str, orbslam_mode: str) -> Path:
    orbslam_root = Path(config["dataset"]["orbslam3_root"])
    log_traj = orbslam_root / "logs" / f"kitti_{sequence}_{orbslam_mode}" / "CameraTrajectory.txt"
    if log_traj.exists():
        return log_traj

    root_traj = orbslam_root / "CameraTrajectory.txt"
    if root_traj.exists() and _is_kitti_trajectory(root_traj):
        log_traj.parent.mkdir(parents=True, exist_ok=True)
        log_traj.write_bytes(root_traj.read_bytes())
        return log_traj

    raise FileNotFoundError(
        f"ORB-SLAM3 trajectory not found: {log_traj}\n"
        f"Run ORB-SLAM3 first:\n"
        f"  ./scripts/run_orbslam3.sh {sequence} {orbslam_mode}"
    )


def build_synthetic_trajectory(config: dict, sequence: str, output_file: Path) -> Path:
    root = Path(config["dataset"]["root"])
    gt = load_kitti_ground_truth(root / "odometry" / "dataset" / "poses" / f"{sequence}.txt")
    times = load_times(root / "odometry" / "dataset" / "sequences" / sequence)

    rng = np.random.default_rng(42)
    noise = rng.normal(0, 0.05, size=gt["position"].shape)
    synthetic_pos = gt["position"] + noise
    save_pose_trajectory_txt(output_file, times, synthetic_pos, gt["quaternion"])
    return output_file


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare SLAM, EKF fusion, and GPS against KITTI ground truth."
    )
    parser.add_argument("--sequence", default="00")
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument(
        "--slam-source",
        choices=["orbslam3", "synthetic"],
        default="orbslam3",
        help="SLAM input: real ORB-SLAM3 trajectory (default) or synthetic GT+noise",
    )
    parser.add_argument(
        "--orbslam3-mode",
        choices=["vanilla", "drift_gps"],
        default="vanilla",
        help="ORB-SLAM3 run mode when --slam-source=orbslam3",
    )
    parser.add_argument("--slam-traj", type=Path, help="Override SLAM trajectory file")
    args = parser.parse_args()

    project = args.project_root
    config_path = project / "config" / "fusion_config.yaml"
    config = load_config(config_path)

    if args.slam_source == "orbslam3":
        results = project / "results" / args.sequence / "vanilla_gps"
        slam_traj = args.slam_traj or resolve_orbslam3_trajectory(
            config, args.sequence, args.orbslam3_mode
        )
        title = "ORB-SLAM3 comparison complete:"
        slam_label = "ORB-SLAM3 ATE RMSE"
    else:
        results = project / "results" / args.sequence / "synthetic_test"
        results.mkdir(parents=True, exist_ok=True)
        slam_traj = args.slam_traj or build_synthetic_trajectory(
            config, args.sequence, results / "synthetic_slam.txt"
        )
        title = "Synthetic test complete:"
        slam_label = "SLAM ATE RMSE"

    results.mkdir(parents=True, exist_ok=True)

    summary = run_fusion_for_mode(
        config,
        args.sequence,
        "vanilla_gps",
        results,
        slam_traj,
        config_path=config_path,
        plot_gt_to_slam=args.slam_source == "synthetic",
        slam_plot_label="SLAM Synthetic" if args.slam_source == "synthetic" else "ORB-SLAM3",
    )
    print(title)
    print(f"  {slam_label}: {summary['slam_ate_rmse']:.4f} m")
    print(f"  EKF ATE RMSE:  {summary['ate_rmse']:.4f} m")
    print(f"  GPS ATE RMSE:  {summary['gps_ate_rmse']:.4f} m")
    print(f"  Results: {results}")


if __name__ == "__main__":
    main()
