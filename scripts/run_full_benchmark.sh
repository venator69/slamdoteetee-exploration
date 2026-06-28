#!/usr/bin/env bash
# Full latency benchmark: TUM desk, drift ON vs drift ON + YOLOv8n (CPU + Hailo).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
mkdir -p "${PROJECT_ROOT}/results"

echo "=== 1/5 Prepare TUM desk ==="
bash "${SCRIPT_DIR}/prepare_tum_desk.sh"

echo "=== 2/5 ORB-SLAM3 drift-only latency ==="
bash "${SCRIPT_DIR}/run_drift_only.sh"

echo "=== 3/5 YOLOv8n COCO latency (CPU) ==="
/usr/bin/python3 "${SCRIPT_DIR}/benchmark_yolo.py" \
  --project-root "${PROJECT_ROOT}" \
  --backend cpu

echo "=== 4/5 YOLOv8n COCO latency (Hailo) ==="
/usr/bin/python3 "${SCRIPT_DIR}/benchmark_yolo.py" \
  --project-root "${PROJECT_ROOT}" \
  --backend hailo \
  --allow-hailo-proxy

echo "=== 5/5 Comparison table ==="
/usr/bin/python3 "${SCRIPT_DIR}/compare_latency.py" --project-root "${PROJECT_ROOT}"

echo "Done: ${PROJECT_ROOT}/results/latency_comparison.csv"
