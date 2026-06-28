# Sensor Synchronization Strategy

## Data Sources

| Sensor | Source | Timestamp |
|--------|--------|-----------|
| ORB-SLAM3 | `sequences/XX/times.txt` | Seconds from sequence start |
| GPS/IMU (OXTS) | `raw/.../oxts/timestamps.txt` | Converted to seconds from first OXTS sample |
| Ground truth | KITTI odometry poses | Frame-indexed (no explicit timestamp) |

## Timestamp Normalization

1. **ORB-SLAM3**: Uses KITTI odometry `times.txt` directly as frame timestamps.
2. **OXTS**: Absolute ISO timestamps are converted to relative seconds from the first measurement, matching the odometry time base.
3. **Ground truth**: Associated by frame index (same count as odometry frames).

## Nearest-Neighbor Matching

For each ORB-SLAM3 timestamp \(t_s\), find the OXTS timestamp \(t_g\) minimizing \(|t_s - t_g|\).

A pair is accepted only if:

\[
|t_s - t_g| \leq \tau_{max}
\]

Default \(\tau_{max} = 50\) ms (configurable in `fusion_config.yaml`).

Unmatched samples are discarded.

## Alignment Subset

The first 100 accepted synchronized pairs are used for Umeyama SE(3) alignment between SLAM and GPS ENU frames.

## GPS Sparse Sampling

To simulate GPS-denied regions, GPS measurements are retained only when the cumulative horizontal path length exceeds 10 m intervals (configurable via `gps.sample_distance_m`).

Between GPS updates, the EKF relies on:
- ORB-SLAM3 pose measurements (every frame)
- IMU propagation (acceleration + gyroscope from OXTS)

## EKF Update Schedule

For each synchronized frame \(k\):

1. **Predict** using IMU (or constant velocity) over \(\Delta t = t_k - t_{k-1}\)
2. **SLAM update** with aligned ORB-SLAM3 pose
3. **GPS update** only if frame \(k\) is in the sparse GPS mask

## Implementation

- `synchronization.nearest_neighbor_sync()`
- `gps_sampler.build_sparse_gps_mask()`
- `pipeline.align_slam_to_gps()`
