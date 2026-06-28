# TUM freiburg1 desk — Tracking Latency Comparison

Drift corrector: ON (`TUM1_lc_on_mpc_on.yaml`)
YOLO: YOLOv8n, COCO pretrained

| Pipeline | SLAM mean [ms] | SLAM median [ms] | YOLO mean [ms] | Pipeline mean [ms] | Pipeline median [ms] |
|----------|----------------|------------------|----------------|--------------------|-----------------------|
| ORB-SLAM3 Drift ON | 77.61 | 70.04 | 0.00 | 77.61 | 70.04 |
| Drift ON + YOLOv8n CPU (COCO) | 77.61 | 70.04 | 607.81 | 685.41 | 658.15 |
| Drift ON + YOLOv8n Hailo (COCO) | 77.61 | 70.04 | 16.27 | 93.88 | 87.07 |
