#!/usr/bin/env bash
# Resolve ORB-SLAM3 and optional YOLO paths from config/env.local.
set -euo pipefail

_paths_lib_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${_paths_lib_dir}/../.." && pwd)"
export PROJECT_ROOT

_env_local="${PROJECT_ROOT}/config/env.local"
if [[ -f "${_env_local}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${_env_local}"
  set +a
fi

: "${ORB_SLAM3_ROOT:?Set ORB_SLAM3_ROOT in config/env.local (see config/env.example)}"
export ORB_SLAM3_ROOT

export YOLO_PT_MODEL="${YOLO_PT_MODEL:-}"
export YOLO_HEF_MODEL="${YOLO_HEF_MODEL:-}"
export YOLO_DETECTOR_SRC="${YOLO_DETECTOR_SRC:-}"
