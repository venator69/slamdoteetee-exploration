# TUM RGB-D Latency Benchmark

Branch **`tum_latency_benchmark`** in [ROBOTICS-STEI-ITB/slamdoteetee-exploration](https://github.com/ROBOTICS-STEI-ITB/slamdoteetee-exploration).

Measures per-frame latency on the TUM `freiburg1_desk` RGB-D sequence for:

1. **ORB-SLAM3 RGB-D** with drift corrector enabled (baseline)
2. **YOLOv8n** object detection on CPU (Ultralytics)
3. **YOLOv8n** on Hailo NPU (or `hailortcli` proxy when Python bindings are missing)

Results are combined into `results/latency_comparison.csv` and `results/latency_comparison.md`.

## Prerequisites

- Ubuntu 22.04+ (tested on Raspberry Pi 5 with Hailo-8)
- ORB-SLAM3 built with `Examples/RGB-D/rgbd_tum` and drift-corrector presets
- Python 3.10+ with `opencv-python`, `pandas`, `numpy`
- For CPU YOLO: `ultralytics` and `yolov8n.pt`
- For Hailo YOLO: `yolov8n.hef`, `hailortcli`, and optional `yolo_detector` package

## Setup

```bash
git clone -b tum_latency_benchmark https://github.com/ROBOTICS-STEI-ITB/slamdoteetee-exploration.git
cd slamdoteetee-exploration

mkdir -p config
cp config/env.example config/env.local
# Edit ORB_SLAM3_ROOT and YOLO model paths

pip install opencv-python pandas numpy ultralytics
```

### Dataset

The TUM desk sequence (~713 MB) is **not** committed. Download with:

```bash
./scripts/prepare_tum_desk.sh
```

This creates `data/rgbd_dataset_freiburg1_desk/` with `associations.txt`.

## Path configuration

Copy `config/env.example` to `config/env.local`:

| Variable | Purpose |
|----------|---------|
| `ORB_SLAM3_ROOT` | Built ORB-SLAM3 install (required) |
| `YOLO_PT_MODEL` | Path to `yolov8n.pt` (CPU benchmark) |
| `YOLO_HEF_MODEL` | Path to `yolov8n.hef` (Hailo benchmark) |
| `YOLO_DETECTOR_SRC` | Directory containing `yolo_detector` package (Hailo e2e) |

## Run

Full benchmark (prepare dataset → ORB-SLAM3 → YOLO CPU → YOLO Hailo → comparison):

```bash
./scripts/run_full_benchmark.sh
```

Individual steps:

```bash
./scripts/prepare_tum_desk.sh
./scripts/run_drift_only.sh
python3 scripts/benchmark_yolo.py --project-root . --backend cpu
python3 scripts/benchmark_yolo.py --project-root . --backend hailo --allow-hailo-proxy
python3 scripts/compare_latency.py --project-root .
```

## Project structure

```
├── config/env.example      # Copy to env.local (gitignored)
├── data/                   # TUM desk sequence (gitignored, download via script)
├── results/                # Cached benchmark outputs from a prior Pi run
├── scripts/
│   ├── lib/paths.sh        # Loads config/env.local
│   ├── prepare_tum_desk.sh
│   ├── run_drift_only.sh   # ORB-SLAM3 drift-corrector latency
│   ├── benchmark_yolo.py   # YOLO CPU / Hailo latency
│   ├── compare_latency.py  # Merge into comparison table
│   └── run_full_benchmark.sh
```

## Cached results

The committed `results/` directory contains metrics from a reference run. Model paths in summary files use `/path/to/...` placeholders; re-run scripts locally for machine-specific logs.

## Other exploration branches

| Branch | Purpose |
|--------|---------|
| `kitti_gps_ekf_fusion` | KITTI GPS + ORB-SLAM3 EKF fusion |
| `ros2_test` | ROS 2 Jazzy + RealSense D455 + ORB-SLAM3 RGB-D |

## License

MIT — TUM RGB-D dataset, ORB-SLAM3, and YOLOv8 have their own licenses.
