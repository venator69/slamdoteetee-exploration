# Experiment Discussion

## Drift Reduction

ORB-SLAM3 accumulates drift over long trajectories, especially in sequences with loops and varying scene structure. The EKF fusion pipeline mitigates drift by:

1. **Sparse GPS corrections** every 10 m simulate re-acquisition of GPS signal after denied regions.
2. **ORB-SLAM3 high-rate updates** maintain local consistency between GPS fixes.
3. **IMU propagation** smooths inter-frame motion using OXTS acceleration and gyroscope.

The drift corrector mode adds a second layer: runtime MPC-based pose correction inside ORB-SLAM3 before EKF fusion.

## GPS Correction Effectiveness

GPS position updates (σ = 2 m) pull the fused trajectory toward global consistency. Effectiveness depends on:

- **Alignment quality**: Umeyama SE(3) on first 100 synchronized pairs
- **GPS availability**: 10 m sampling leaves long dead-reckoning segments
- **SLAM weight**: Low SLAM position noise (0.1 m) trusts visual odometry locally

When GPS and SLAM agree, EKF reduces long-term drift. When they disagree (urban canyons, multipath), GPS corrections can degrade short-term accuracy.

## Failure Cases

| Scenario | Symptom | Mitigation |
|----------|---------|------------|
| GPS multipath | Position jumps | Increase `gps.position_std`, outlier rejection |
| SLAM tracking loss | Missing pose updates | Increase `slam.position_std`, skip bad frames |
| Poor alignment | Global offset | More alignment samples, verify timestamp sync |
| IMU bias | Orientation drift | Bias estimation (future work) |
| Missing raw data | Pipeline abort | Run `download_kitti_seq05_07.sh` |

## Sensitivity to GPS Noise

Configurable via `gps.position_std` in `fusion_config.yaml`:

| GPS σ [m] | Behavior |
|-----------|----------|
| 0.5 | Tight GPS fusion, follows GPS closely |
| 2.0 | Default, balanced |
| 5.0 | Loose fusion, trusts SLAM more |
| 10.0 | GPS barely influences solution |

Higher GPS noise reduces correction aggressiveness but improves robustness to outliers.

## Synthetic Validation (Seq 00)

Using ground-truth poses + noise as synthetic SLAM input:

| Method | ATE RMSE |
|--------|----------|
| SLAM (GT+noise) | 0.087 m |
| EKF Fusion | 0.188 m |
| GPS Only | 1.090 m |

Note: Synthetic SLAM is in GT frame; EKF aligns to GPS ENU, explaining higher EKF error in this synthetic scenario. Real ORB-SLAM3 trajectories benefit from GPS global anchoring.

## Real Experiments Status

| Sequence | ORB-SLAM3 | EKF Fusion |
|----------|-----------|------------|
| 00 | Running (vanilla) | Pending |
| 05 | Raw data downloading | Pending |
| 07 | Raw data pending download | Pending |

Run after ORB-SLAM3 completes:

```bash
./scripts/run_fusion_only.sh 00
```
