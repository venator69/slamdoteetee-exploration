# GPS-Assisted ORB-SLAM3 EKF Fusion Pipeline

Production-quality GPS-assisted visual-inertial fusion for KITTI using ORB-SLAM3, OXTS GPS/IMU, and an Extended Kalman Filter.

## Features

- ORB-SLAM3 stereo odometry on KITTI (EpsteinFile3 SSD)
- OXTS GPS/IMU parsing with LLA → ECEF → ENU conversion
- Sparse GPS sampling (every 10 m) to simulate GPS-denied regions
- Umeyama SE(3) alignment of SLAM to GPS ENU frame
- EKF fusion of ORB-SLAM3 pose + GPS + IMU
- Three operating modes: vanilla, vanilla+GPS, drift corrector+GPS
- ATE/RPE evaluation with trajectory and error plots

## Quick Start

```bash
cd /home/slamet/Dev/kitti_gps_ekf_fusion
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
export PYTHONPATH=src:$PYTHONPATH

# Full pipeline on KITTI sequence 00
./scripts/run_full_pipeline.sh 00
```

## Project Structure

```
kitti_gps_ekf_fusion/
├── config/fusion_config.yaml    # Noise models, sync params
├── docs/                        # EKF derivation, transforms, sync
├── scripts/                     # KITTI prep, ORB-SLAM3, full pipeline
├── src/kitti_gps_ekf/           # Python fusion library
│   ├── coordinates.py           # LLA/ECEF/ENU
│   ├── oxts_parser.py           # KITTI OXTS reader
│   ├── gps_sampler.py           # 10 m GPS sampling
│   ├── alignment.py             # Umeyama alignment
│   ├── synchronization.py       # Timestamp sync
│   ├── ekf.py                   # Extended Kalman Filter
│   ├── evaluation.py            # ATE/RPE metrics
│   ├── visualization.py         # Matplotlib plots
│   └── pipeline.py              # Main orchestrator
└── results/                     # Per-sequence outputs
```

## Documentation

- [EKF Derivation](docs/EKF_DERIVATION.md)
- [Coordinate Transforms](docs/COORDINATE_TRANSFORMS.md)
- [Sensor Synchronization](docs/SENSOR_SYNC.md)
- [KITTI Execution](docs/KITTI_EXECUTION.md)

## Configuration

All noise covariances are tunable via `config/fusion_config.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gps.position_std` | 2.0 m | GPS measurement noise |
| `slam.position_std` | 0.1 m | SLAM position noise |
| `slam.orientation_std` | 0.05 rad | SLAM orientation noise |
| `gps.sample_distance_m` | 10.0 m | GPS update interval |
| `synchronization.max_time_diff_s` | 0.05 s | Max sync offset |

## License

MIT — KITTI dataset and ORB-SLAM3 have their own licenses.
