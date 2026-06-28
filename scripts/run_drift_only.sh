#!/usr/bin/env bash
# ORB-SLAM3 RGB-D on TUM desk with DriftStabilizer ON (latency baseline).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ORB_ROOT="/media/slamet/EpsteinFile3/dev/ORB_SLAM3"
SEQ_DIR="${PROJECT_ROOT}/data/rgbd_dataset_freiburg1_desk"
ASSOC="${SEQ_DIR}/associations.txt"
YAML="${ORB_ROOT}/Examples/RGB-D/presets/TUM1_lc_on_mpc_on.yaml"
VOCAB="${ORB_ROOT}/Vocabulary/ORBvoc.txt"
OUT="${PROJECT_ROOT}/results/drift_only"
LOG="${OUT}/run.log"

bash "${SCRIPT_DIR}/prepare_tum_desk.sh"

mkdir -p "${OUT}"
cd "${ORB_ROOT}"
export LD_LIBRARY_PATH="${ORB_ROOT}/lib:${ORB_ROOT}/Thirdparty/DBoW2/lib:${ORB_ROOT}/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"
export ORBSLAM_NO_VIEWER=1

echo "Running ORB-SLAM3 RGB-D (drift corrector ON) on TUM desk..."
echo "  YAML: ${YAML}"
echo "  Seq:  ${SEQ_DIR}"

./Examples/RGB-D/rgbd_tum "${VOCAB}" "${YAML}" "${SEQ_DIR}" "${ASSOC}" 2>&1 | tee "${LOG}"

cp -f FrameTrackingTimes.txt "${OUT}/FrameTrackingTimes.txt"
cp -f TrackingTimeStats.txt "${OUT}/TrackingTimeStats.txt" 2>/dev/null || true
cp -f CameraTrajectory.txt "${OUT}/CameraTrajectory.txt" 2>/dev/null || true

echo "Saved: ${OUT}"
