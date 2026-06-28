#!/usr/bin/env bash
# Run EKF fusion after ORB-SLAM3 trajectories are available.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/paths.sh"
VENV="${PROJECT_ROOT}/.venv/bin/python"
export PYTHONPATH="${PROJECT_ROOT}/src:${PYTHONPATH:-}"

SEQ="${1:-00}"
CONFIG="${PROJECT_ROOT}/config/fusion_config.yaml"
RESULTS="${PROJECT_ROOT}/results/${SEQ}"
ORB_LOG="${ORB_SLAM3_ROOT}/logs"

for MODE in vanilla vanilla_gps drift_gps; do
  case "${MODE}" in
    vanilla) TRAJ="${ORB_LOG}/kitti_${SEQ}_vanilla/CameraTrajectory.txt" ;;
    vanilla_gps) TRAJ="${ORB_LOG}/kitti_${SEQ}_vanilla/CameraTrajectory.txt" ;;
    drift_gps) TRAJ="${ORB_LOG}/kitti_${SEQ}_drift_gps/CameraTrajectory.txt" ;;
  esac
  if [[ ! -f "${TRAJ}" ]]; then
    echo "Missing trajectory for ${MODE}: ${TRAJ}"
    exit 1
  fi
  echo "Running fusion: ${MODE}"
  "${VENV}" -m kitti_gps_ekf.pipeline \
    --config "${CONFIG}" \
    --sequence "${SEQ}" \
    --mode "${MODE}" \
    --slam-traj "${TRAJ}" \
    --results-dir "${RESULTS}/${MODE}"
done

DRIFT_TRAJ="${ORB_LOG}/kitti_${SEQ}_drift_gps/CameraTrajectory.txt"
if [[ -f "${DRIFT_TRAJ}" ]]; then
  echo "Running drift corrector only (no EKF): drift_only"
  "${VENV}" -m kitti_gps_ekf.pipeline \
    --config "${CONFIG}" \
    --sequence "${SEQ}" \
    --mode vanilla \
    --slam-traj "${DRIFT_TRAJ}" \
    --results-dir "${RESULTS}/drift_only"
fi

"${VENV}" "${SCRIPT_DIR}/compare_results.py" --sequence "${SEQ}" --project-root "${PROJECT_ROOT}"
echo "Fusion complete: ${RESULTS}"
