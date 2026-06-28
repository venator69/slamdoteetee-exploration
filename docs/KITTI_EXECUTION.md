# KITTI Execution Instructions

## Prerequisites

- Ubuntu 22.04
- ORB-SLAM3 built at `/media/slamet/EpsteinFile3/dev/ORB_SLAM3`
- KITTI RGBD+GPS dataset at `/media/slamet/EpsteinFile3/SLAM-Datasets/KITTI-RGBD-GPS`
- Python 3.10+ virtual environment

## Dataset Layout

```
KITTI-RGBD-GPS/
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
cd /home/slamet/Dev/kitti_gps_ekf_fusion
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
export PYTHONPATH=src:$PYTHONPATH
```

## Sequence Preparation

Links raw grayscale images (`image_00`, `image_01`) into odometry format (`image_0`, `image_1`):

```bash
./scripts/prepare_kitti_sequence.sh 00
./scripts/prepare_kitti_sequence.sh 05
./scripts/prepare_kitti_sequence.sh 07
```

## Run ORB-SLAM3

Three modes:

| Mode | Command | YAML |
|------|---------|------|
| Vanilla | `./scripts/run_orbslam3.sh 00 vanilla` | KITTI00-02.yaml |
| Drift Corrector | `./scripts/run_orbslam3.sh 00 drift_gps` | presets/KITTI_drift_corrector.yaml |

## Run EKF Fusion

```bash
source .venv/bin/activate
python -m kitti_gps_ekf.pipeline \
  --sequence 00 \
  --mode vanilla_gps \
  --slam-traj /media/slamet/EpsteinFile3/dev/ORB_SLAM3/logs/kitti_00_vanilla/CameraTrajectory.txt \
  --results-dir results/00/vanilla_gps
```

## Full Pipeline (all modes)

```bash
./scripts/run_full_pipeline.sh 00
```

## Download Missing Sequences

If raw images for sequences 05 or 07 are missing:

```bash
bash /media/slamet/EpsteinFile3/SLAM-Datasets/KITTI-RGBD-GPS/download_kitti_rgbd_gps.sh
```

Or download individual archives:

```bash
wget -c -O downloads/2011_09_30_drive_0018_sync.zip \
  https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_30_drive_0018/2011_09_30_drive_0018_sync.zip
unzip downloads/2011_09_30_drive_0018_sync.zip -d raw/
```

## Expected Output

```
results/00/
├── vanilla/
│   ├── orbslam3_traj.txt
│   ├── gps_traj.txt
│   ├── ekf_traj.txt
│   ├── ate_report.csv
│   ├── rpe_report.csv
│   ├── trajectory_plot.png
│   ├── error_plot.png
│   ├── fusion_config.yaml
│   └── evaluation_summary.md
├── vanilla_gps/
├── drift_gps/
└── comparison_table.csv
```

## Sequence Mapping

| Seq | Raw Drive | Frames | YAML |
|-----|-----------|--------|------|
| 00 | 2011_10_03_drive_0027 | 4541 | KITTI00-02.yaml |
| 05 | 2011_09_30_drive_0018 | 2761 | KITTI04-12.yaml |
| 07 | 2011_09_30_drive_0027 | 1101 | KITTI04-12.yaml |

## Tuning

Edit `config/fusion_config.yaml`:

```yaml
gps:
  sample_distance_m: 10.0
  position_std: 2.0
slam:
  position_std: 0.1
  orientation_std: 0.05
```

## Runtime Notes

- ORB-SLAM3 on KITTI seq 00 (~4541 frames) takes 1-3 hours on Raspberry Pi 4/5.
- EKF fusion and evaluation complete in seconds.
- Use `tee` logs in `ORB_SLAM3/logs/kitti_XX_MODE/run.log` to monitor progress.
