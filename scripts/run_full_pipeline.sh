#!/usr/bin/env bash
# Full GPS-assisted ORB-SLAM3 EKF fusion pipeline for KITTI.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SEQ="${1:-00}"

echo "=== Step 1: Prepare KITTI sequence ${SEQ} ==="
bash "${SCRIPT_DIR}/prepare_kitti_sequence.sh" "${SEQ}"

echo "=== Step 2: Run ORB-SLAM3 (vanilla) ==="
bash "${SCRIPT_DIR}/run_orbslam3.sh" "${SEQ}" vanilla

echo "=== Step 3: Run ORB-SLAM3 (drift corrector) ==="
bash "${SCRIPT_DIR}/run_orbslam3.sh" "${SEQ}" drift_gps

echo "=== Step 4-5: EKF fusion, evaluation, comparison ==="
bash "${SCRIPT_DIR}/run_fusion_only.sh" "${SEQ}"

echo "Pipeline complete: ${PROJECT_ROOT}/results/${SEQ}"
