#!/usr/bin/env bash
# Run ORB-SLAM3 stereo_kitti on a KITTI odometry sequence.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/paths.sh"
CONFIG="${PROJECT_ROOT}/config/fusion_config.yaml"

SEQ="${1:-00}"
MODE="${2:-vanilla}"   # vanilla | drift_gps

ORB_ROOT="${ORB_SLAM3_ROOT}"
VOCAB="${ORB_ROOT}/Vocabulary/ORBvoc.txt"
KITTI_ROOT="${KITTI_DATASET_ROOT}"
PREPARED_ROOT="${PROJECT_ROOT}/prepared_sequences"
SEQ_DIR="${PREPARED_ROOT}/${SEQ}"
LOG_DIR="${ORB_ROOT}/logs/kitti_${SEQ}_${MODE}"
mkdir -p "${LOG_DIR}"

YAML_NAME="$(python3 -c "import yaml; c=yaml.safe_load(open('${CONFIG}')); print(c['sequence_mapping']['${SEQ}']['yaml'])")"
YAML="${PROJECT_ROOT}/config/orbslam3/${YAML_NAME}"

if [[ "${MODE}" == "drift_gps" ]]; then
  YAML="${PROJECT_ROOT}/config/orbslam3/KITTI_drift_corrector.yaml"
fi

FRACTION="$(python3 -c "import yaml; c=yaml.safe_load(open('${CONFIG}')); print(c.get('trajectory', {}).get('fraction', 1.0))")"
EXPECTED_FRAMES="$(python3 -c "from pathlib import Path; seq='${SEQ}'; root=Path('${KITTI_ROOT}'); n=len((root/'odometry'/'dataset'/'sequences'/seq/'times.txt').read_text().strip().splitlines()); import yaml; c=yaml.safe_load(open('${CONFIG}')); f=float(c.get('trajectory', {}).get('fraction', 1.0)); print(max(1, int(n*f)))")"
ACTUAL_FRAMES=0
if [[ -f "${SEQ_DIR}/times.txt" ]]; then
  ACTUAL_FRAMES="$(wc -l < "${SEQ_DIR}/times.txt" | tr -d ' ')"
fi

if [[ ! -f "${SEQ_DIR}/image_0/000000.png" ]] || [[ "${ACTUAL_FRAMES}" != "${EXPECTED_FRAMES}" ]]; then
  echo "Preparing sequence ${SEQ} for ${EXPECTED_FRAMES} frames (fraction=${FRACTION})..."
  bash "${SCRIPT_DIR}/prepare_kitti_sequence.sh" "${SEQ}"
fi

cd "${ORB_ROOT}"
export LD_LIBRARY_PATH="${ORB_ROOT}/lib:${ORB_ROOT}/Thirdparty/DBoW2/lib:${ORB_ROOT}/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"

echo "Running ORB-SLAM3 stereo_kitti on sequence ${SEQ} (mode=${MODE}, frames=${EXPECTED_FRAMES}, fraction=${FRACTION})"
echo "  Vocabulary: ${VOCAB}"
echo "  Settings:   ${YAML}"
echo "  Sequence:   ${SEQ_DIR}"
echo "  Output:     ${LOG_DIR}"

./Examples/Stereo/stereo_kitti "${VOCAB}" "${YAML}" "${SEQ_DIR}" 2>&1 | tee "${LOG_DIR}/run.log"
cp -f CameraTrajectory.txt "${LOG_DIR}/CameraTrajectory.txt"
cp -f TrackingTimeStats.txt "${LOG_DIR}/TrackingTimeStats.txt" 2>/dev/null || true
echo "Trajectory saved to ${LOG_DIR}/CameraTrajectory.txt"
