# GPS-Assisted ORB-SLAM3 EKF Fusion (KITTI)

Branch **`kitti_gps_ekf_fusion`** in [ROBOTICS-STEI-ITB/slamdoteetee-exploration](https://github.com/ROBOTICS-STEI-ITB/slamdoteetee-exploration).

GPS-assisted visual-inertial fusion for KITTI using ORB-SLAM3 stereo odometry, OXTS GPS/IMU, and an Extended Kalman Filter.

## What this branch contains

- Python EKF fusion library (`src/kitti_gps_ekf/`)
- ORB-SLAM3 YAML presets for KITTI (`config/orbslam3/`)
- Shell scripts for dataset prep, ORB-SLAM3 runs, fusion, and evaluation
- Sample `prepared_sequences/00` thumbnails and cached `results/00` from a prior run

## Prerequisites

- Ubuntu 22.04+ (tested on Raspberry Pi 5)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) built with `stereo_kitti` example
- KITTI odometry + raw synced drives (sequences 00, 05, 07)
- Python 3.10+

## Setup

```bash
git clone -b kitti_gps_ekf_fusion https://github.com/ROBOTICS-STEI-ITB/slamdoteetee-exploration.git
cd slamdoteetee-exploration

cp config/env.example config/env.local
# Edit config/env.local with your ORB-SLAM3 and KITTI dataset paths

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
export PYTHONPATH=src:$PYTHONPATH
```

### Dataset layout

Point `KITTI_DATASET_ROOT` at a directory with:

```
KITTI-RGBD-GPS/
├── odometry/dataset/sequences/00..21/
├── odometry/dataset/poses/
└── raw/<date>/<drive>_sync/
```

Download helpers: `./scripts/download_kitti_seq05_07.sh`

## Quick start

```bash
source .venv/bin/activate
export PYTHONPATH=src:$PYTHONPATH

# Full pipeline on sequence 00 (prepare → ORB-SLAM3 → EKF → plots)
./scripts/run_full_pipeline.sh 00
```

Individual steps:

```bash
./scripts/prepare_kitti_sequence.sh 00
./scripts/run_orbslam3.sh 00 vanilla
./scripts/run_orbslam3.sh 00 drift_gps
./scripts/run_fusion_only.sh 00
```

## Operating modes

| Mode | Description |
|------|-------------|
| `vanilla` | ORB-SLAM3 trajectory only (no GPS in EKF) |
| `vanilla_gps` | ORB-SLAM3 + sparse GPS fusion |
| `drift_gps` | Drift-corrector ORB-SLAM3 + GPS fusion |

## Project structure

```
├── config/
│   ├── env.example           # Copy to env.local (gitignored)
│   ├── fusion_config.yaml    # Noise models, sequence mapping
│   └── orbslam3/             # KITTI YAML presets
├── docs/                     # EKF derivation, coordinates, sync
├── prepared_sequences/       # Symlinked stereo frames per sequence
├── results/                  # Trajectories, ATE/RPE, plots
├── scripts/
│   ├── lib/paths.sh          # Resolves KITTI_DATASET_ROOT / ORB_SLAM3_ROOT
│   └── run_full_pipeline.sh
└── src/kitti_gps_ekf/        # Fusion library
```

## Configuration

Paths are **not** hardcoded. Set them in `config/env.local` or export:

- `KITTI_DATASET_ROOT` — KITTI odometry + raw data root
- `ORB_SLAM3_ROOT` — built ORB-SLAM3 install directory

Fusion noise and sync parameters: `config/fusion_config.yaml`

## Documentation

- [KITTI execution guide](docs/KITTI_EXECUTION.md)
- [EKF derivation](docs/EKF_DERIVATION.md)
- [Coordinate transforms](docs/COORDINATE_TRANSFORMS.md)
- [Sensor synchronization](docs/SENSOR_SYNC.md)

## Other exploration branches

| Branch | Purpose |
|--------|---------|
| `tum_latency_benchmark` | TUM RGB-D latency: drift corrector vs YOLO (CPU/Hailo) |
| `ros2_test` | ROS 2 Jazzy + RealSense D455 + ORB-SLAM3 RGB-D |

## License

MIT — KITTI dataset and ORB-SLAM3 have their own licenses.
