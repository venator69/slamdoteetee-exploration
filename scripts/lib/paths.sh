#!/usr/bin/env bash
# Resolve dataset and ORB-SLAM3 paths from config/env.local or fusion_config.yaml.
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

_read_yaml_path() {
  local key="$1"
  python3 - "${PROJECT_ROOT}/config/fusion_config.yaml" "${key}" <<'PY'
import sys
from pathlib import Path
import yaml

config_path = Path(sys.argv[1])
key = sys.argv[2]
config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
value = config.get("dataset", {}).get(key, "") or ""
print(value)
PY
}

if [[ -z "${KITTI_DATASET_ROOT:-}" ]]; then
  _cfg="$(_read_yaml_path root)"
  if [[ -n "${_cfg}" && "${_cfg}" != /path/to/* ]]; then
    export KITTI_DATASET_ROOT="${_cfg}"
  fi
fi

if [[ -z "${ORB_SLAM3_ROOT:-}" ]]; then
  _cfg="$(_read_yaml_path orbslam3_root)"
  if [[ -n "${_cfg}" && "${_cfg}" != /path/to/* ]]; then
    export ORB_SLAM3_ROOT="${_cfg}"
  fi
fi

: "${KITTI_DATASET_ROOT:?Set KITTI_DATASET_ROOT in config/env.local (see config/env.example)}"
: "${ORB_SLAM3_ROOT:?Set ORB_SLAM3_ROOT in config/env.local (see config/env.example)}"
export KITTI_DATASET_ROOT ORB_SLAM3_ROOT
