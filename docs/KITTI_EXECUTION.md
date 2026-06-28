# KITTI Execution Instructions

## Prerequisites

- Ubuntu 22.04+
- ORB-SLAM3 built with `Examples/Stereo/stereo_kitti`
- KITTI odometry dataset + raw synced drives for sequences 00, 05, 07
- Python 3.10+ virtual environment

## Path configuration

Copy `config/env.example` to `config/env.local` and set:

```bash
export KITTI_DATASET_ROOT=/path/to/KITTI-RGBD-GPS
export ORB_SLAM3_ROOT=/path/to/ORB_SLAM3
```

All scripts source `scripts/lib/paths.sh`, which loads `config/env.local` and exports these variables.

## Dataset layout

```
${KITTI_DATASET_ROOT}/
├── odometry/dataset/
│   ├── sequences/00..21/   # times.txt, calib.txt
│   └── poses/00..10.txt    # ground truth
└── raw/
    ├── 2011_10_03/2011_10_03_drive_0027_sync/  # seq 00
    ├── 2011_09_30/2011_09_30_drive_0018_sync/  # seq 05
    └── 2011_09_30/2011_09_30_drive_0027_sync/  # seq 07
```

## Setup

```bash
cp config/env.example config/env.local
# edit paths in config/env.local

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
export PYTHONPATH=src:$PYTHONPATH
```

## Sequence preparation

Links raw grayscale images (`image_00`, `image_01`) into odometry format (`image_0`, `image_1`):

```bash
./scripts/prepare_kitti_sequence.sh 00
./scripts/prepare_kitti_sequence.sh 05
./scripts/prepare_kitti_sequence.sh 07
```

## Run ORB-SLAM3

| Mode | Command | YAML |
|------|---------|------|
| Vanilla | `./scripts/run_orbslam3.sh 00 vanilla` | KITTI00-02.yaml |
| Drift corrector | `./scripts/run_orbslam3.sh 00 drift_gps` | KITTI_drift_corrector.yaml |

Trajectories are written to `${ORB_SLAM3_ROOT}/logs/kitti_<seq>_<mode>/`.

## Run EKF fusion

```bash
source .venv/bin/activate
python -m kitti_gps_ekf.pipeline \
  --sequence 00 \
  --mode vanilla_gps \
  --slam-traj "${ORB_SLAM3_ROOT}/logs/kitti_00_vanilla/CameraTrajectory.txt" \
  --results-dir results/00/vanilla_gps
```

Or run all fusion modes after ORB-SLAM3 finishes:

```bash
./scripts/run_fusion_only.sh 00
```

## Full pipeline

```bash
./scripts/run_full_pipeline.sh 00
```

## Download missing sequences

If raw images for sequences 05 or 07 are missing:

```bash
./scripts/download_kitti_seq05_07.sh
```

Or download individual archives from [KITTI raw data](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/).

## Expected output

```
results/00/
├── vanilla/
├── vanilla_gps/
├── drift_gps/
├── drift_only/
└── comparison_table.csv
```

Each mode directory contains trajectories, ATE/RPE CSVs, plots, and a snapshot `fusion_config.yaml`.

## Sequence mapping

| Seq | Raw drive | YAML |
|-----|-----------|------|
| 00 | 2011_10_03_drive_0027 | KITTI00-02.yaml |
| 05 | 2011_09_30_drive_0018 | KITTI04-12.yaml |
| 07 | 2011_09_30_drive_0027 | KITTI04-12.yaml |

## Runtime notes

- ORB-SLAM3 on full sequence 00 can take 1–3 hours on Raspberry Pi 4/5.
- EKF fusion and evaluation complete in seconds.
- Monitor progress in `${ORB_SLAM3_ROOT}/logs/kitti_XX_MODE/run.log`.
