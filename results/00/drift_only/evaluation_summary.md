# KITTI Sequence 00 Evaluation Summary

**Mode:** vanilla

## Metrics

- ATE RMSE (fused): 0.6918 m
- RPE RMSE (fused, delta=10): 8.5536
- ATE RMSE (ORB-SLAM3): 0.6918 m
- ATE RMSE (GPS only): 0.7517 m

## Configuration

- GPS sample distance: 10.0 m
- GPS position std: 2.0 m
- SLAM position std: 0.1 m

## Output Files

- orbslam3_traj.txt
- gps_traj.txt
- ekf_traj.txt
- ate_report.csv
- rpe_report.csv
- trajectory_plot.png
- error_plot.png