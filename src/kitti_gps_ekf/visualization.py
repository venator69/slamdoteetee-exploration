"""Trajectory and error visualization."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .evaluation import align_trajectory_se3


def _align_to_reference(
    estimate: np.ndarray,
    reference: np.ndarray,
) -> np.ndarray:
    """SE(3)-align estimate trajectory to a reference for visualization."""
    n = min(len(estimate), len(reference))
    aligned, _, _, _ = align_trajectory_se3(estimate[:n], reference[:n])
    return aligned


def plot_trajectories(
    output_file: Path,
    ground_truth: np.ndarray,
    trajectories: dict[str, np.ndarray],
    gps: np.ndarray | None = None,
    title: str = "KITTI Trajectory Comparison",
    align_gps_to_ground_truth: bool = False,
    align_estimates_to_ground_truth: bool = True,
    plot_frame: str = "ground_truth",
    slam_reference: np.ndarray | None = None,
    y_lim: tuple[float, float] | None = (-20.0, 0.0),
) -> None:
    if plot_frame == "slam":
        if slam_reference is None:
            raise ValueError("slam_reference is required when plot_frame='slam'")
        reference = np.asarray(slam_reference, dtype=np.float64)
        gt_xy = _align_to_reference(ground_truth, reference)[:, :2]
        gt_label = "Ground Truth (→ SLAM)"
        align_estimates = False
    else:
        reference = None
        gt_xy = np.asarray(ground_truth[:, :2], dtype=np.float64)
        gt_label = "Ground Truth"
        align_estimates = align_estimates_to_ground_truth

    plot_gps = None
    gps_label = "GPS (ENU)"
    if gps is not None:
        gps_xy = np.asarray(gps[:, :2], dtype=np.float64)
        if plot_frame == "slam" and reference is not None:
            plot_gps = _align_to_reference(gps, reference)[:, :2]
            gps_label = "GPS (→ SLAM)"
        elif align_gps_to_ground_truth:
            plot_gps = _align_to_reference(gps, ground_truth)[:, :2]
            gps_label = "GPS (aligned)"
        else:
            plot_gps = gps_xy

    plot_trajectories_dict: dict[str, np.ndarray] = {}
    for name, traj in trajectories.items():
        traj_xy = np.asarray(traj[:, :2], dtype=np.float64)
        if plot_frame == "slam" and reference is not None:
            if name in {"SLAM Synthetic", "ORB-SLAM3"}:
                plot_xy = reference[:, :2]
            else:
                plot_xy = _align_to_reference(traj, reference)[:, :2]
        elif align_estimates:
            plot_xy = _align_to_reference(traj, ground_truth)[:, :2]
        else:
            plot_xy = traj_xy
        plot_trajectories_dict[name] = plot_xy

    fig, ax = plt.subplots(figsize=(12, 8))
    ax.plot(gt_xy[:, 0], gt_xy[:, 1], "k-", linewidth=2, label=gt_label)
    if plot_gps is not None:
        ax.plot(plot_gps[:, 0], plot_gps[:, 1], "g--", linewidth=1.5, alpha=0.7, label=gps_label)
    colors = {
        "SLAM Synthetic": "blue",
        "ORB-SLAM3": "blue",
        "EKF Fusion": "red",
        "Drift Corrector + EKF Fusion": "orange",
        "GPS Only": "green",
    }
    for name, traj_xy in plot_trajectories_dict.items():
        color = colors.get(name, None)
        ax.plot(traj_xy[:, 0], traj_xy[:, 1], label=name, color=color, linewidth=1.5)
    if plot_frame == "slam":
        xlabel = "X [m] (SLAM frame)"
        ylabel = "Y [m] (SLAM frame)"
    elif align_estimates and not align_gps_to_ground_truth:
        xlabel = "X [m] (GT frame; GPS in ENU)"
        ylabel = "Y [m] (GT frame; GPS in ENU)"
    elif align_estimates:
        xlabel = "X [m] (GT frame)"
        ylabel = "Y [m] (GT frame)"
    else:
        xlabel = "X [m]"
        ylabel = "Y [m]"
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if y_lim is not None:
        ax.set_ylim(y_lim[0], y_lim[1])
        x_values = [gt_xy[:, 0]]
        if plot_gps is not None:
            x_values.append(plot_gps[:, 0])
        for traj_xy in plot_trajectories_dict.values():
            x_values.append(traj_xy[:, 0])
        x_all = np.concatenate(x_values)
        x_pad = max(5.0, 0.05 * (x_all.max() - x_all.min()))
        ax.set_xlim(x_all.min() - x_pad, x_all.max() + x_pad)
    else:
        ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    output_file.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_file, dpi=150)
    plt.close(fig)


def plot_error_analysis(
    output_file: Path,
    timestamps: np.ndarray,
    ate_errors: np.ndarray,
    method_errors: dict[str, np.ndarray],
    title: str = "Trajectory Error Analysis",
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(timestamps, ate_errors, "r-", label="ATE error")
    axes[0].set_ylabel("ATE [m]")
    axes[0].set_title("Position Error vs Time")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    cumulative_ate = np.cumsum(ate_errors) / np.arange(1, len(ate_errors) + 1)
    axes[1].plot(timestamps, cumulative_ate, "b-", label="Cumulative mean ATE")
    axes[1].set_ylabel("ATE [m]")
    axes[1].set_title("ATE vs Time")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    for name, errors in method_errors.items():
        drift = np.cumsum(errors) / np.arange(1, len(errors) + 1)
        axes[2].plot(timestamps[: len(drift)], drift, label=f"{name} drift")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Drift [m]")
    axes[2].set_title("Drift Accumulation vs Time")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    fig.suptitle(title)
    fig.tight_layout()
    output_file.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_file, dpi=150)
    plt.close(fig)


def save_comparison_table(
    output_file: Path,
    rows: list[dict],
) -> pd.DataFrame:
    df = pd.DataFrame(rows)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_file, index=False)
    return df
