#!/usr/bin/env python3
"""Build latency comparison: drift-only vs drift + YOLOv8n (CPU/Hailo) on TUM desk."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd


def parse_tracking_stats(stats_file: Path) -> pd.DataFrame:
    rows = []
    for line in stats_file.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 9:
            continue
        rows.append(
            {
                "frame": len(rows),
                "slam_total_ms": float(parts[-1]),
            }
        )
    return pd.DataFrame(rows)


def load_frame_tracking(frame_file: Path) -> pd.DataFrame:
    rows = []
    for line in frame_file.read_text().splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        frame, timestamp, track_s = line.split()
        rows.append(
            {
                "frame": int(frame),
                "timestamp_s": float(timestamp),
                "slam_track_s": float(track_s),
                "slam_track_ms": float(track_s) * 1000.0,
            }
        )
    return pd.DataFrame(rows)


def summarize(name: str, slam_ms: np.ndarray, yolo_ms: np.ndarray | None = None) -> dict:
    yolo = np.zeros_like(slam_ms) if yolo_ms is None else yolo_ms[: len(slam_ms)]
    pipeline = slam_ms + yolo
    return {
        "Pipeline": name,
        "Frames": len(slam_ms),
        "SLAM mean [ms]": float(np.mean(slam_ms)),
        "SLAM median [ms]": float(np.median(slam_ms)),
        "YOLO mean [ms]": float(np.mean(yolo)),
        "YOLO median [ms]": float(np.median(yolo)),
        "Pipeline mean [ms]": float(np.mean(pipeline)),
        "Pipeline median [ms]": float(np.median(pipeline)),
        "Pipeline max [ms]": float(np.max(pipeline)),
    }


def add_yolo_pipeline(
    rows: list[dict],
    slam_ms: np.ndarray,
    yolo_dir: Path,
    label: str,
    combined_frames: list[dict] | None = None,
) -> None:
    yolo_file = yolo_dir / "yolo_latency.csv"
    if not yolo_file.exists():
        return

    yolo_df = pd.read_csv(yolo_file)
    n = min(len(slam_ms), len(yolo_df))
    if combined_frames is not None:
        for i in range(n):
            combined_frames.append(
                {
                    "frame": i,
                    "pipeline": label,
                    "slam_ms": float(slam_ms[i]),
                    "yolo_ms": float(yolo_df["yolo_ms"].to_numpy()[i]),
                }
            )
    rows.append(
        summarize(
            label,
            slam_ms[:n],
            yolo_df["yolo_ms"].to_numpy()[:n],
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()

    project = args.project_root
    drift_dir = project / "results/drift_only"
    out_dir = project / "results"

    frame_file = drift_dir / "FrameTrackingTimes.txt"
    stats_file = drift_dir / "TrackingTimeStats.txt"

    if frame_file.exists():
        slam_df = load_frame_tracking(frame_file)
        slam_ms = slam_df["slam_track_ms"].to_numpy()
    elif stats_file.exists():
        slam_df = parse_tracking_stats(stats_file)
        slam_ms = slam_df["slam_total_ms"].to_numpy()
    else:
        raise FileNotFoundError(f"Missing SLAM timing outputs in {drift_dir}")

    rows = [summarize("ORB-SLAM3 Drift ON", slam_ms)]
    combined_frames: list[dict] = []

    add_yolo_pipeline(
        rows,
        slam_ms,
        project / "results/drift_yolo_cpu",
        "Drift ON + YOLOv8n CPU (COCO)",
        combined_frames,
    )
    add_yolo_pipeline(
        rows,
        slam_ms,
        project / "results/drift_yolo_hailo",
        "Drift ON + YOLOv8n Hailo (COCO)",
        combined_frames,
    )

    if combined_frames:
        combined = pd.DataFrame(combined_frames)
        combined["pipeline_ms"] = combined["slam_ms"] + combined["yolo_ms"]
        combined.to_csv(out_dir / "combined_latency.csv", index=False)

    df = pd.DataFrame(rows)
    df.to_csv(out_dir / "latency_comparison.csv", index=False)

    md = [
        "# TUM freiburg1 desk — Tracking Latency Comparison",
        "",
        "Drift corrector: ON (`TUM1_lc_on_mpc_on.yaml`)",
        "YOLO: YOLOv8n, COCO pretrained",
        "",
        "| Pipeline | SLAM mean [ms] | SLAM median [ms] | YOLO mean [ms] | Pipeline mean [ms] | Pipeline median [ms] |",
        "|----------|----------------|------------------|----------------|--------------------|-----------------------|",
    ]
    for row in rows:
        md.append(
            f"| {row['Pipeline']} | {row['SLAM mean [ms]']:.2f} | {row['SLAM median [ms]']:.2f} | "
            f"{row['YOLO mean [ms]']:.2f} | {row['Pipeline mean [ms]']:.2f} | {row['Pipeline median [ms]']:.2f} |"
        )
    (out_dir / "latency_comparison.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    print(df.to_string(index=False))
    print(f"\nCSV: {out_dir / 'latency_comparison.csv'}")
    print(f"MD:  {out_dir / 'latency_comparison.md'}")


if __name__ == "__main__":
    main()
