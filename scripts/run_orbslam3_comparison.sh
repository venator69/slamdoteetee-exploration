#!/usr/bin/env bash
# Compare real ORB-SLAM3, EKF fusion, and GPS against KITTI ground truth.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export PYTHONPATH="${PROJECT_ROOT}/src:${PYTHONPATH:-}"

SEQ="${1:-00}"
MODE="${2:-vanilla}"

"${PROJECT_ROOT}/.venv/bin/python" "${SCRIPT_DIR}/run_synthetic_test.py" \
  --sequence "${SEQ}" \
  --project-root "${PROJECT_ROOT}" \
  --slam-source orbslam3 \
  --orbslam3-mode "${MODE}"
