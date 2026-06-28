"""Main GPS-assisted ORB-SLAM3 EKF fusion pipeline."""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from .alignment import align_poses_to_enu, transform_orientation
from .ekf import EkfConfig, run_ekf_fusion
from .evaluation import compute_ate, compute_rpe, evaluate_trajectory
from .gps_sampler import build_sparse_gps_mask
from .oxts_parser import load_oxts_sequence, oxts_to_gps_trajectory, oxts_to_local_pose
from .synchronization import nearest_neighbor_sync
from .trajectory_io import (
    load_kitti_ground_truth,
    load_times,
    load_trajectory_auto,
    save_gps_trajectory_txt,
    save_pose_trajectory_txt,
)
from .visualization import plot_error_analysis, plot_trajectories


def _resolve_dataset_path(config: dict, key: str, env_name: str) -> str:
    env_value = os.environ.get(env_name, "").strip()
    if env_value:
        return env_value
    value = str(config.get("dataset", {}).get(key, "")).strip()
    if value and not value.startswith("/path/to/"):
        return value
    raise ValueError(
        f"Set {env_name} or dataset.{key} in fusion_config.yaml "
        f"(see config/env.example)"
    )


def load_config(config_path: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    dataset = config.setdefault("dataset", {})
    dataset["root"] = _resolve_dataset_path(config, "root", "KITTI_DATASET_ROOT")
    dataset["orbslam3_root"] = _resolve_dataset_path(
        config, "orbslam3_root", "ORB_SLAM3_ROOT"
    )
    return config


def get_trajectory_fraction(config: dict) -> float:
    fraction = float(config.get("trajectory", {}).get("fraction", 1.0))
    if not 0.0 < fraction <= 1.0:
        raise ValueError(f"trajectory.fraction must be in (0, 1], got {fraction}")
    return fraction


def get_max_frames(config: dict, total_frames: int) -> int:
    return max(1, int(total_frames * get_trajectory_fraction(config)))


def get_paths(config: dict, sequence: str) -> dict[str, Path]:
    root = Path(config["dataset"]["root"])
    mapping = config["sequence_mapping"][sequence]
    raw_drive = root / "raw" / mapping["date"] / mapping["drive"]
    odom_seq = root / "odometry" / "dataset" / "sequences" / sequence
    return {
        "raw_drive": raw_drive,
        "odom_seq": odom_seq,
        "poses_gt": root / "odometry" / "dataset" / "poses" / f"{sequence}.txt",
        "orbslam3_root": Path(config["dataset"]["orbslam3_root"]),
        "yaml": mapping["yaml"],
    }


def align_slam_to_gps(
    slam: dict[str, np.ndarray],
    gps: dict[str, np.ndarray],
    max_time_diff: float,
    alignment_samples: int,
    with_scale: bool = False,
) -> dict[str, np.ndarray]:
    pairs = nearest_neighbor_sync(slam["timestamp"], gps["timestamp"], max_time_diff)
    if len(pairs) < 3:
        raise RuntimeError("Insufficient synchronized samples for Umeyama alignment")

    use_pairs = pairs[: min(alignment_samples, len(pairs))]
    slam_pts = np.array([slam["position"][p.source_idx] for p in use_pairs])
    gps_pts = np.array(
        [
            [gps["east"][p.target_idx], gps["north"][p.target_idx], gps["up"][p.target_idx]]
            for p in use_pairs
        ]
    )

    _, rotation, translation, scale = align_poses_to_enu(slam_pts, gps_pts, with_scale=with_scale)
    aligned_positions = (scale * (rotation @ slam["position"].T).T) + translation
    aligned_quaternions = transform_orientation(slam["quaternion"], rotation)
    return {
        "timestamp": slam["timestamp"],
        "position": aligned_positions,
        "quaternion": aligned_quaternions,
        "rotation": rotation,
        "translation": translation,
        "scale": scale,
    }


def run_fusion_for_mode(
    config: dict,
    sequence: str,
    mode: str,
    results_dir: Path,
    slam_traj_file: Path,
    config_path: Path | None = None,
    plot_gt_to_slam: bool = False,
    slam_plot_label: str = "ORB-SLAM3",
) -> dict:
    paths = get_paths(config, sequence)
    times = load_times(paths["odom_seq"])
    max_frames = get_max_frames(config, len(times))
    times = times[:max_frames]
    slam = load_trajectory_auto(slam_traj_file, timestamps=times)
    slam = {key: value[:max_frames] for key, value in slam.items() if isinstance(value, np.ndarray)}
    oxts_samples = load_oxts_sequence(paths["raw_drive"], max_frames=max_frames)
    gps = oxts_to_gps_trajectory(oxts_samples)

    sync_pairs = nearest_neighbor_sync(
        slam["timestamp"], gps["timestamp"], config["synchronization"]["max_time_diff_s"]
    )
    if not sync_pairs:
        raise RuntimeError("No synchronized SLAM/GPS pairs found")

    aligned_slam = align_slam_to_gps(
        slam,
        gps,
        max_time_diff=config["synchronization"]["max_time_diff_s"],
        alignment_samples=config["synchronization"]["alignment_samples"],
        with_scale=False,
    )

    gps_positions = np.column_stack([gps["east"], gps["north"], gps["up"]])
    gps_mask = build_sparse_gps_mask(
        gps["east"],
        gps["north"],
        gps["up"],
        config["gps"]["sample_distance_m"],
    )

    ekf_config = EkfConfig(
        process_position_std=config["ekf"]["process_position_std"],
        process_velocity_std=config["ekf"]["process_velocity_std"],
        process_orientation_std=config["ekf"]["process_orientation_std"],
        gps_position_std=config["gps"]["position_std"],
        slam_position_std=config["slam"]["position_std"],
        slam_orientation_std=config["slam"]["orientation_std"],
        initial_position_std=config["ekf"]["initial_position_std"],
        initial_velocity_std=config["ekf"]["initial_velocity_std"],
        initial_orientation_std=config["ekf"]["initial_orientation_std"],
        imu_enabled=config["imu"]["enabled"],
    )

    if mode in {"vanilla_gps", "drift_gps"}:
        fused = run_ekf_fusion(
            aligned_slam["timestamp"],
            aligned_slam["position"],
            aligned_slam["quaternion"],
            gps_positions,
            gps_mask,
            imu_accel=np.column_stack([gps["ax"], gps["ay"], gps["az"]]),
            imu_gyro=np.column_stack([gps["wx"], gps["wy"], gps["wz"]]),
            config=ekf_config,
        )
    else:
        fused = {
            "timestamp": aligned_slam["timestamp"],
            "position": aligned_slam["position"],
            "quaternion": aligned_slam["quaternion"],
        }

    gt = load_kitti_ground_truth(paths["poses_gt"])
    gt_positions = gt["position"][:max_frames]
    gt_quaternions = gt["quaternion"][:max_frames]

    save_pose_trajectory_txt(
        results_dir / "orbslam3_traj.txt",
        aligned_slam["timestamp"],
        aligned_slam["position"],
        aligned_slam["quaternion"],
    )
    save_gps_trajectory_txt(
        results_dir / "gps_traj.txt",
        gps["timestamp"],
        gps["east"],
        gps["north"],
        gps["up"],
    )
    save_pose_trajectory_txt(
        results_dir / "ekf_traj.txt",
        fused["timestamp"],
        fused["position"],
        fused["quaternion"],
    )
    if config_path is not None:
        shutil.copy2(config_path, results_dir / "fusion_config.yaml")

    metrics, ate_df, rpe_df = evaluate_trajectory(
        fused["position"],
        fused["quaternion"],
        gt_positions,
        gt_quaternions,
        timestamps=fused["timestamp"],
        rpe_deltas=config["evaluation"]["rpe_deltas"],
    )
    ate_df.to_csv(results_dir / "ate_report.csv", index=False)
    rpe_df.to_csv(results_dir / "rpe_report.csv", index=False)

    slam_metrics, slam_ate_df, slam_rpe_df = evaluate_trajectory(
        aligned_slam["position"],
        aligned_slam["quaternion"],
        gt_positions,
        gt_quaternions,
        timestamps=aligned_slam["timestamp"],
        rpe_deltas=config["evaluation"]["rpe_deltas"],
    )
    gps_metrics, _, _ = evaluate_trajectory(
        oxts_to_local_pose(oxts_samples)["position"][: len(gt_positions)],
        oxts_to_local_pose(oxts_samples)["quaternion"][: len(gt_positions)],
        gt_positions,
        gt_quaternions,
        timestamps=gps["timestamp"][: len(gt_positions)],
        rpe_deltas=config["evaluation"]["rpe_deltas"],
    )

    ekf_label = (
        "Drift Corrector + EKF Fusion"
        if mode == "drift_gps"
        else "EKF Fusion"
    )
    slam_plot_position = slam["position"] if plot_gt_to_slam else aligned_slam["position"]
    ekf_plot_position = (
        aligned_slam["position"] if mode == "vanilla" else fused["position"]
    )
    plot_kwargs = {
        "align_gps_to_ground_truth": False,
        "align_estimates_to_ground_truth": not plot_gt_to_slam,
    }
    if plot_gt_to_slam:
        plot_kwargs.update(
            {
                "plot_frame": "slam",
                "slam_reference": slam["position"],
            }
        )

    plot_trajectories(
        results_dir / "trajectory_plot.png",
        gt_positions,
        {
            slam_plot_label: slam_plot_position,
            ekf_label: ekf_plot_position,
        },
        gps=gps_positions,
        title=f"KITTI Seq {sequence} - {mode}",
        **plot_kwargs,
    )
    plot_error_analysis(
        results_dir / "error_plot.png",
        ate_df["timestamp"].to_numpy(),
        ate_df["ate_error_m"].to_numpy(),
        {
            "ORB-SLAM3": slam_ate_df["ate_error_m"].to_numpy(),
            "EKF Fusion": ate_df["ate_error_m"].to_numpy(),
        },
        title=f"KITTI Seq {sequence} Error Analysis",
    )

    summary = {
        "sequence": sequence,
        "mode": mode,
        "ate_rmse": metrics["ate"].rmse,
        "rpe_rmse": metrics["rpe"][config["evaluation"]["rpe_deltas"][1]]["rmse"],
        "slam_ate_rmse": slam_metrics["ate"].rmse,
        "slam_rpe_rmse": slam_rpe_df.iloc[1]["rmse"] if len(slam_rpe_df) > 1 else slam_rpe_df.iloc[0]["rmse"],
        "gps_ate_rmse": gps_metrics["ate"].rmse,
        "gps_rpe_rmse": gps_metrics["rpe"][config["evaluation"]["rpe_deltas"][1]]["rmse"],
    }
    write_evaluation_summary(results_dir, summary, config, sequence, mode)
    return summary


_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "fusion_config.yaml"


def write_evaluation_summary(
    results_dir: Path,
    summary: dict,
    config: dict,
    sequence: str,
    mode: str,
) -> None:
    lines = [
        f"# KITTI Sequence {sequence} Evaluation Summary",
        "",
        f"**Mode:** {mode}",
        "",
        "## Metrics",
        "",
        f"- ATE RMSE (fused): {summary['ate_rmse']:.4f} m",
        f"- RPE RMSE (fused, delta=10): {summary['rpe_rmse']:.4f}",
        f"- ATE RMSE (ORB-SLAM3): {summary['slam_ate_rmse']:.4f} m",
        f"- ATE RMSE (GPS only): {summary['gps_ate_rmse']:.4f} m",
        "",
        "## Configuration",
        "",
        f"- GPS sample distance: {config['gps']['sample_distance_m']} m",
        f"- GPS position std: {config['gps']['position_std']} m",
        f"- SLAM position std: {config['slam']['position_std']} m",
        "",
        "## Output Files",
        "",
        "- orbslam3_traj.txt",
        "- gps_traj.txt",
        "- ekf_traj.txt",
        "- ate_report.csv",
        "- rpe_report.csv",
        "- trajectory_plot.png",
        "- error_plot.png",
    ]
    (results_dir / "evaluation_summary.md").write_text("\n".join(lines), encoding="utf-8")


def compare_all_methods(
    config: dict,
    sequence: str,
    results_dir: Path,
) -> pd.DataFrame:
    gt_path = Path(config["dataset"]["root"]) / "odometry" / "dataset" / "poses" / f"{sequence}.txt"
    gt = load_kitti_ground_truth(gt_path)
    max_frames = get_max_frames(config, len(gt["position"]))
    gt = {
        "position": gt["position"][:max_frames],
        "quaternion": gt["quaternion"][:max_frames],
    }
    rpe_deltas = config.get("evaluation", {}).get("rpe_deltas", [1, 10, 100])

    method_specs = [
        ("ORB-SLAM3 Vanilla", results_dir / "vanilla", "slam"),
        ("Drift Corrector Only", results_dir / "drift_only", "slam"),
        ("Vanilla + GPS EKF", results_dir / "vanilla_gps", "ekf"),
        ("Drift Corrector + GPS EKF", results_dir / "drift_gps", "ekf"),
        ("GPS Only", results_dir / "vanilla_gps", "gps"),
    ]

    rows = []
    for label, result_dir, method_name in method_specs:
        if method_name == "gps":
            gps_pose = oxts_to_local_pose(
                load_oxts_sequence(
                    Path(config["dataset"]["root"])
                    / "raw"
                    / config["sequence_mapping"][sequence]["date"]
                    / config["sequence_mapping"][sequence]["drive"],
                    max_frames=len(gt["position"]),
                )
            )
            positions = gps_pose["position"]
            quaternions = gps_pose["quaternion"]
        else:
            if not result_dir.exists():
                continue
            traj_file = result_dir / ("ekf_traj.txt" if method_name == "ekf" else "orbslam3_traj.txt")
            if not traj_file.exists():
                continue
            traj = np.loadtxt(traj_file, skiprows=1)
            positions = traj[:, 1:4]
            quaternions = traj[:, 4:8]

        ate, _, _ = compute_ate(positions, gt["position"][: len(positions)])
        row = {"Method": label, "ATE RMSE [m]": ate.rmse}
        for delta in rpe_deltas:
            rpe = compute_rpe(
                positions,
                quaternions,
                gt["position"][: len(positions)],
                gt["quaternion"][: len(positions)],
                delta=delta,
            )
            row[f"RPE RMSE (d={delta})"] = rpe.rmse
        rows.append(row)

    df = pd.DataFrame(rows)
    results_dir.mkdir(parents=True, exist_ok=True)
    df.to_csv(results_dir / "comparison_table.csv", index=False)

    md_lines = [
        f"# KITTI Sequence {sequence} — Comparison Table",
        "",
        f"Trajectory fraction: {get_trajectory_fraction(config):.0%}",
        "",
        "| Method | ATE RMSE [m] | "
        + " | ".join(f"RPE d={d}" for d in rpe_deltas)
        + " |",
        "|--------|-------------|" + "|".join("----------:" for _ in rpe_deltas) + "|",
    ]
    for row in rows:
        md_lines.append(
            f"| {row['Method']} | {row['ATE RMSE [m]']:.4f} | "
            + " | ".join(f"{row[f'RPE RMSE (d={d})']:.4f}" for d in rpe_deltas)
            + " |"
        )
    (results_dir / "comparison_table.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")
    return df


def main() -> None:
    parser = argparse.ArgumentParser(description="KITTI GPS-assisted ORB-SLAM3 EKF fusion")
    parser.add_argument("--config", type=Path, default=_CONFIG_PATH)
    parser.add_argument("--sequence", type=str, required=True)
    parser.add_argument(
        "--mode",
        choices=["vanilla", "vanilla_gps", "drift_gps", "all"],
        default="all",
    )
    parser.add_argument("--slam-traj", type=Path, help="ORB-SLAM3 CameraTrajectory.txt")
    parser.add_argument("--results-dir", type=Path, default=None)
    args = parser.parse_args()

    config = load_config(args.config)
    base_results = args.results_dir or Path(__file__).resolve().parents[2] / "results" / args.sequence

    modes = (
        ["vanilla", "vanilla_gps", "drift_gps"]
        if args.mode == "all"
        else [args.mode]
    )

    summaries = []
    for mode in modes:
        mode_dir = base_results / mode if args.mode == "all" else base_results
        mode_dir.mkdir(parents=True, exist_ok=True)
        if args.slam_traj is None:
            slam_traj = (
                Path(config["dataset"]["orbslam3_root"])
                / "logs"
                / f"kitti_{args.sequence}_{mode}"
                / "CameraTrajectory.txt"
            )
        else:
            slam_traj = args.slam_traj
        if not slam_traj.exists():
            raise FileNotFoundError(f"SLAM trajectory not found: {slam_traj}")
        summary = run_fusion_for_mode(
            config, args.sequence, mode, mode_dir, slam_traj, config_path=args.config
        )
        summaries.append(summary)

    if args.mode == "all" and len(summaries) == 3:
        compare_all_methods(config, args.sequence, base_results)


if __name__ == "__main__":
    main()
