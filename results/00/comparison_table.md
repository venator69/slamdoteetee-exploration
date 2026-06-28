# KITTI Sequence 00 — Comparison Table

Trajectory fraction: 20%

| Method | ATE RMSE [m] | RPE d=1 | RPE d=10 | RPE d=100 |
|--------|-------------|----------:|----------:|----------:|
| ORB-SLAM3 Vanilla | 0.8103 | 0.8519 | 8.5084 | 76.9622 |
| Drift Corrector Only | 0.6918 | 0.8565 | 8.5536 | 77.3518 |
| Vanilla + GPS EKF | 0.8113 | 0.9027 | 9.0742 | 82.8364 |
| Drift Corrector + GPS EKF | 0.7002 | 0.9053 | 9.0989 | 83.0385 |
| GPS Only | 0.7517 | 0.6855 | 6.8101 | 60.4442 |
