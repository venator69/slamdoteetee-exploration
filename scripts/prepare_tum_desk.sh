#!/usr/bin/env bash
# Download and prepare TUM RGB-D freiburg1 desk sequence.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATA_ROOT="${PROJECT_ROOT}/data"
SEQ_NAME="rgbd_dataset_freiburg1_desk"
SEQ_DIR="${DATA_ROOT}/${SEQ_NAME}"
TGZ="${DATA_ROOT}/${SEQ_NAME}.tgz"
URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg1/${SEQ_NAME}.tgz"

mkdir -p "${DATA_ROOT}"

associations_ready() {
  [[ -s "${SEQ_DIR}/associations.txt" ]]
}

dataset_ready() {
  [[ -d "${SEQ_DIR}/rgb" && -d "${SEQ_DIR}/depth" ]] || return 1
  [[ -n "$(ls -A "${SEQ_DIR}/rgb" 2>/dev/null)" ]] || return 1
  associations_ready || [[ -f "${SEQ_DIR}/rgb.txt" && -f "${SEQ_DIR}/depth.txt" ]]
}

if dataset_ready; then
  echo "TUM desk ready: ${SEQ_DIR}"
  exit 0
fi

if [[ ! -f "${TGZ}" ]]; then
  echo "Downloading TUM desk (~713 MB)..."
  wget -O "${TGZ}" "${URL}"
fi

echo "Extracting ${TGZ}..."
tar -xzf "${TGZ}" -C "${DATA_ROOT}"

if ! associations_ready; then
  if [[ -f "${SEQ_DIR}/rgb.txt" && -f "${SEQ_DIR}/depth.txt" ]]; then
    python3 "${SCRIPT_DIR}/make_associations.py" "${SEQ_DIR}"
  else
    echo "Missing rgb.txt/depth.txt; provide associations manually under ${SEQ_DIR}"
    exit 1
  fi
fi

echo "Prepared: ${SEQ_DIR} ($(find "${SEQ_DIR}/rgb" -type f | wc -l) RGB frames)"
