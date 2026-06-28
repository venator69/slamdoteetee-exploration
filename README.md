# ROS 2 ORB-SLAM3 Workspace (RealSense RGB-D)

Branch **`ros2_test`** in [ROBOTICS-STEI-ITB/slamdoteetee-exploration](https://github.com/ROBOTICS-STEI-ITB/slamdoteetee-exploration).

Colcon workspace for **ROS 2 Jazzy** with `ros2_orb_slam3`: monocular, stereo, and **RGB-D** nodes, plus a Python bridge for dashboard pose streaming.

Designed for **Intel RealSense D455** on a single Raspberry Pi (localhost DDS discovery).

## What this branch contains

- `src/ros2_orb_slam3/` — ORB-SLAM3 ROS 2 package (RGB-D, stereo, mono examples)
- `scripts/ros_pipeline_env.sh` — workspace-relative environment setup
- `cyclonedds.xml` — CycloneDDS config for solo-Pi ROS discovery

**Not committed** (build or download locally):

- `build/`, `install/`, `log/` — colcon artifacts
- ORB vocabulary tarball — extract under `src/ros2_orb_slam3/orb_slam3/Vocabulary/`
- Large test datasets and map databases

## Prerequisites

- Ubuntu 24.04 + ROS 2 Jazzy
- Intel RealSense SDK 2 (`realsense2_camera` ROS package)
- Pangolin, Eigen, OpenCV (see package README in `src/ros2_orb_slam3/README.md`)

## Setup

```bash
git clone -b ros2_test https://github.com/ROBOTICS-STEI-ITB/slamdoteetee-exploration.git ros2_test
cd ros2_test

# ORB vocabulary (required once)
cd src/ros2_orb_slam3/orb_slam3/Vocabulary
tar -xf ORBvoc.txt.tar.gz   # download from ORB-SLAM3 repo if missing
cd ../../../..

source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros2_orb_slam3
source install/setup.bash
```

## Environment

Source the shared pipeline script in every terminal (paths are relative to this workspace):

```bash
source scripts/ros_pipeline_env.sh
```

Override defaults with environment variables:

| Variable | Default |
|----------|---------|
| `ROS_DOMAIN_ID` | `42` |
| `ORBSLAM3_VOCAB` | `src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt` |
| `ORBSLAM3_CONFIG` | `src/ros2_orb_slam3/orb_slam3/config/RGB-D/RealSense_D455.yaml` |

## Run RGB-D (RealSense D455)

Terminal 1 — camera:

```bash
source scripts/ros_pipeline_env.sh
ros2 launch realsense2_camera rs_launch.py enable_depth:=true enable_color:=true
```

Terminal 2 — ORB-SLAM3 RGB-D node:

```bash
source scripts/ros_pipeline_env.sh
ros2 run ros2_orb_slam3 rgbd
```

Terminal 3 — pose bridge (optional, for web dashboard):

```bash
source scripts/ros_pipeline_env.sh
ros2 run ros2_orb_slam3 ros_backend_bridge.py
```

## Project structure

```
ros2_test/
├── cyclonedds.xml
├── scripts/ros_pipeline_env.sh
└── src/ros2_orb_slam3/
    ├── orb_slam3/              # ORB-SLAM3 core + configs
    ├── scripts/                # mono_driver, ros_backend_bridge
    └── src/                    # rgbd_example, stereo_example, mono_example
```

## Integration with slamdoteetee dashboard

Pair this workspace with the main [slamdoteetee](https://github.com/ROBOTICS-STEI-ITB/slamdoteetee) web dashboard. The dashboard backend reads pose from the ROS bridge HTTP API when `ros_backend_bridge.py` is running.

## Other exploration branches

| Branch | Purpose |
|--------|---------|
| `kitti_gps_ekf_fusion` | KITTI GPS + ORB-SLAM3 EKF fusion |
| `tum_latency_benchmark` | TUM RGB-D latency vs YOLO (CPU/Hailo) |

## License

GPLv3 (ORB-SLAM3 / ros2_orb_slam3 package) — see `src/ros2_orb_slam3/README.md`.
