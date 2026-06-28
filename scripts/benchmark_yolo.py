#!/usr/bin/env python3
"""Benchmark YOLOv8n (COCO) inference latency on TUM desk RGB frames."""

from __future__ import annotations

import argparse
import os
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

try:
    from ultralytics import YOLO
except ImportError:
    sys.path.insert(0, str(Path.home() / ".local/lib/python3.12/site-packages"))
    from ultralytics import YOLO

DEFAULT_HEF = Path("/path/to/yolov8n.hef")
DEFAULT_PT = Path("/path/to/yolov8n.pt")


def resolve_model_path(env_name: str, cli_path: Path | None, fallback: Path) -> Path:
    if cli_path is not None:
        return cli_path
    env_value = os.environ.get(env_name, "").strip()
    if env_value:
        return Path(env_value)
    if str(fallback).startswith("/path/to/"):
        raise ValueError(
            f"Set {env_name} in config/env.local or pass --model/--hef "
            f"(see config/env.example)"
        )
    return fallback


def hailo_detector_src() -> Path:
    raw = os.environ.get("YOLO_DETECTOR_SRC", "").strip()
    if not raw:
        raise RuntimeError(
            "Set YOLO_DETECTOR_SRC to the yolo_detector package for the Hailo backend "
            "(see config/env.example)"
        )
    return Path(raw)


def load_associations(assoc_file: Path) -> list[tuple[float, str]]:
    rows: list[tuple[float, str]] = []
    for line in assoc_file.read_text().splitlines():
        if not line.strip():
            continue
        parts = line.split()
        rows.append((float(parts[0]), parts[1]))
    return rows


def create_hailo_backend(hef_path: Path, confidence: float, input_size: int):
    sys.path.insert(0, str(hailo_detector_src()))
    from yolo_detector.hailo_subprocess import create_hailo_backend as _create

    return _create(str(hef_path), confidence, input_size)


def parse_hailortcli_benchmark(hef_path: Path) -> dict[str, float]:
    proc = subprocess.run(
        ["hailortcli", "benchmark", str(hef_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    output = proc.stdout + proc.stderr
    hw_match = re.search(r"HW Latency:\s*([\d.]+)\s*ms", output)
    fps_matches = re.findall(r"FPS:\s*([\d.]+)", output)
    streaming_fps = float(fps_matches[-1]) if fps_matches else 0.0
    hw_ms = float(hw_match.group(1)) if hw_match else 0.0
    streaming_ms = 1000.0 / streaming_fps if streaming_fps > 0 else 0.0
    return {
        "hw_latency_ms": hw_ms,
        "streaming_fps": streaming_fps,
        "streaming_ms": streaming_ms,
    }


def benchmark_cpu(
    pairs: list[tuple[float, str]],
    seq_dir: Path,
    model_path: Path,
    imgsz: int,
    warmup: int,
) -> tuple[list[dict], dict]:
    model = YOLO(str(model_path))
    warmup_img = cv2.imread(str(seq_dir / pairs[0][1]))
    for _ in range(warmup):
        model.predict(warmup_img, imgsz=imgsz, device="cpu", verbose=False)

    records = []
    for frame_idx, (timestamp, rgb_rel) in enumerate(pairs):
        image = cv2.imread(str(seq_dir / rgb_rel))
        if image is None:
            raise FileNotFoundError(seq_dir / rgb_rel)

        t0 = time.perf_counter()
        results = model.predict(image, imgsz=imgsz, device="cpu", verbose=False)
        infer_ms = (time.perf_counter() - t0) * 1000.0
        n_det = len(results[0].boxes) if results and results[0].boxes is not None else 0
        records.append(
            {
                "frame": frame_idx,
                "timestamp_s": timestamp,
                "yolo_ms": infer_ms,
                "detections": n_det,
            }
        )
    return records, {"method": "ultralytics_cpu", "model": str(model_path)}


def benchmark_hailo(
    pairs: list[tuple[float, str]],
    seq_dir: Path,
    hef_path: Path,
    imgsz: int,
    warmup: int,
) -> tuple[list[dict], dict]:
    try:
        backend = create_hailo_backend(hef_path, 0.25, imgsz)
    except Exception as exc:
        raise RuntimeError(
            "Hailo Python bindings unavailable. Install hailo-all or hailort cp313 wheel, "
            f"then retry. Original error: {exc}"
        ) from exc

    warmup_img = cv2.imread(str(seq_dir / pairs[0][1]))
    try:
        for _ in range(warmup):
            backend.predict(warmup_img)

        records = []
        for frame_idx, (timestamp, rgb_rel) in enumerate(pairs):
            image = cv2.imread(str(seq_dir / rgb_rel))
            if image is None:
                raise FileNotFoundError(seq_dir / rgb_rel)

            t0 = time.perf_counter()
            detections, _ = backend.predict(image)
            infer_ms = (time.perf_counter() - t0) * 1000.0
            records.append(
                {
                    "frame": frame_idx,
                    "timestamp_s": timestamp,
                    "yolo_ms": infer_ms,
                    "detections": len(detections),
                }
            )
    finally:
        backend.close()

    return records, {"method": "hailo_e2e", "model": str(hef_path)}


def benchmark_hailo_proxy(
    pairs: list[tuple[float, str]],
    seq_dir: Path,
    hef_path: Path,
    imgsz: int,
) -> tuple[list[dict], dict]:
    """Fallback when hailo_platform is missing: CPU preprocess + hailortcli streaming latency."""
    stats = parse_hailortcli_benchmark(hef_path)
    proxy_ms = stats["streaming_ms"]
    if proxy_ms <= 0:
        proxy_ms = stats["hw_latency_ms"]

    records = []
    for frame_idx, (timestamp, rgb_rel) in enumerate(pairs):
        image = cv2.imread(str(seq_dir / rgb_rel))
        if image is None:
            raise FileNotFoundError(seq_dir / rgb_rel)

        t0 = time.perf_counter()
        resized = cv2.resize(image, (imgsz, imgsz))
        _ = resized.astype(np.uint8)
        prep_ms = (time.perf_counter() - t0) * 1000.0
        infer_ms = prep_ms + proxy_ms
        records.append(
            {
                "frame": frame_idx,
                "timestamp_s": timestamp,
                "yolo_ms": infer_ms,
                "detections": -1,
            }
        )

    meta = {
        "method": "hailortcli_proxy",
        "model": str(hef_path),
        "note": (
            "hailo_platform missing; latency = CPU resize + hailortcli streaming latency "
            f"({proxy_ms:.2f} ms)"
        ),
        **stats,
    }
    return records, meta


def write_summary(out_dir: Path, backend: str, records: list[dict], meta: dict) -> None:
    df = pd.DataFrame(records)
    df.to_csv(out_dir / "yolo_latency.csv", index=False)

    summary = {
        "backend": backend,
        "method": meta.get("method", backend),
        "model": meta.get("model", ""),
        "dataset": "TUM freiburg1 desk (COCO)",
        "frames": len(df),
        "mean_yolo_ms": float(df["yolo_ms"].mean()),
        "median_yolo_ms": float(df["yolo_ms"].median()),
        "std_yolo_ms": float(df["yolo_ms"].std()),
        "max_yolo_ms": float(df["yolo_ms"].max()),
        "mean_detections": float(df["detections"].mean()),
    }
    if "note" in meta:
        summary["note"] = meta["note"]
    for key in ("hw_latency_ms", "streaming_fps", "streaming_ms"):
        if key in meta:
            summary[key] = meta[key]

    lines = [f"{k}: {v}" for k, v in summary.items()]
    (out_dir / "yolo_summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--backend", choices=("cpu", "hailo"), default="cpu")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--hef", type=Path, default=None)
    parser.add_argument("--allow-hailo-proxy", action="store_true")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--warmup", type=int, default=5)
    args = parser.parse_args()

    project = args.project_root
    model_path = resolve_model_path("YOLO_PT_MODEL", args.model, DEFAULT_PT)
    hef_path = resolve_model_path("YOLO_HEF_MODEL", args.hef, DEFAULT_HEF)
    seq_dir = project / "data/rgbd_dataset_freiburg1_desk"
    assoc = seq_dir / "associations.txt"
    out_dir = project / f"results/drift_yolo_{args.backend}"
    out_dir.mkdir(parents=True, exist_ok=True)

    pairs = load_associations(assoc)
    if args.backend == "cpu":
        records, meta = benchmark_cpu(pairs, seq_dir, model_path, args.imgsz, args.warmup)
    else:
        try:
            records, meta = benchmark_hailo(pairs, seq_dir, hef_path, args.imgsz, args.warmup)
        except RuntimeError:
            if not args.allow_hailo_proxy:
                raise
            print("WARNING: falling back to hailortcli proxy latency (hailo_platform missing)")
            records, meta = benchmark_hailo_proxy(pairs, seq_dir, hef_path, args.imgsz)

    write_summary(out_dir, args.backend, records, meta)


if __name__ == "__main__":
    main()
