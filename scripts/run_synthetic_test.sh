#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export PYTHONPATH="${PROJECT_ROOT}/src:${PYTHONPATH:-}"
"${PROJECT_ROOT}/.venv/bin/python" "${SCRIPT_DIR}/run_synthetic_test.py" \
  --sequence "${1:-00}" \
  --project-root "${PROJECT_ROOT}" \
  --slam-source synthetic
