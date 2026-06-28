#!/usr/bin/env bash
# Wait for ORB-SLAM3 to finish, then run ORB-SLAM3 vs EKF vs GPS comparison.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/paths.sh"

SEQ="${1:-00}"
MODE="${2:-vanilla}"
ORB_ROOT="${ORB_SLAM3_ROOT}"
LOG_TRAJ="${ORB_ROOT}/logs/kitti_${SEQ}_${MODE}/CameraTrajectory.txt"
ROOT_TRAJ="${ORB_ROOT}/CameraTrajectory.txt"
POLL_SEC="${POLL_SEC:-30}"

echo "Waiting for ORB-SLAM3 trajectory (seq=${SEQ}, mode=${MODE})..."
while true; do
  if [[ -f "${LOG_TRAJ}" ]]; then
    break
  fi
  if [[ -f "${ROOT_TRAJ}" ]] && awk 'NF==12 {found=1; exit} END {exit !found}' "${ROOT_TRAJ}"; then
    mkdir -p "$(dirname "${LOG_TRAJ}")"
    cp -f "${ROOT_TRAJ}" "${LOG_TRAJ}"
    break
  fi
  if ! pgrep -f "stereo_kitti.*prepared_sequences/${SEQ}" >/dev/null; then
    if [[ -f "${ROOT_TRAJ}" ]] && awk 'NF==12 {found=1; exit} END {exit !found}' "${ROOT_TRAJ}"; then
      mkdir -p "$(dirname "${LOG_TRAJ}")"
      cp -f "${ROOT_TRAJ}" "${LOG_TRAJ}"
      break
    fi
    echo "ORB-SLAM3 is not running and no KITTI trajectory was found."
    exit 1
  fi
  sleep "${POLL_SEC}"
done

echo "Trajectory ready: ${LOG_TRAJ}"
bash "${SCRIPT_DIR}/run_orbslam3_comparison.sh" "${SEQ}" "${MODE}"
