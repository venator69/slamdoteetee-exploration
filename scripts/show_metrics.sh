#!/usr/bin/env bash
# Tampilkan metrics ATE/RPE untuk ketiga pipeline mode.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SEQ="${1:-00}"
RESULTS="${PROJECT_ROOT}/results/${SEQ}"

echo "=============================================="
echo "  KITTI Seq ${SEQ} — Metrics Summary"
echo "=============================================="
echo ""

show_mode() {
  local mode="$1"
  local label="$2"
  local dir="${RESULTS}/${mode}"

  echo "── ${label} (${mode}) ──"
  if [[ ! -d "${dir}" ]]; then
    echo "  [belum dijalankan]"
    echo ""
    return
  fi

  if [[ -f "${dir}/evaluation_summary.md" ]]; then
    grep -E "ATE RMSE|RPE RMSE" "${dir}/evaluation_summary.md" | sed 's/^/- /'
  else
    echo "  [evaluation_summary.md tidak ditemukan]"
  fi

  if [[ -f "${dir}/rpe_report.csv" ]]; then
    echo "  RPE detail (delta=10):"
    awk -F, 'NR==1 || $1==10 {print "    delta="$1"  rmse="$2"  mean="$3"  median="$4}' "${dir}/rpe_report.csv"
  fi
  echo ""
}

show_mode "vanilla"       "1. ORB-SLAM3 Vanilla"
show_mode "drift_only"    "2. Drift Corrector Only"
show_mode "vanilla_gps"   "3. Vanilla + GPS EKF"
show_mode "drift_gps"     "4. Drift Corrector + GPS EKF"

if [[ -f "${RESULTS}/comparison_table.csv" ]]; then
  echo "── Comparison Table ──"
  column -t -s, "${RESULTS}/comparison_table.csv" 2>/dev/null || cat "${RESULTS}/comparison_table.csv"
  echo ""
fi

if [[ -f "${RESULTS}/comparison_table.md" ]]; then
  echo "Markdown: ${RESULTS}/comparison_table.md"
  echo ""
fi

echo "Plot tersedia di:"
for mode in vanilla drift_only vanilla_gps drift_gps; do
  dir="${RESULTS}/${mode}"
  [[ -f "${dir}/trajectory_plot.png" ]] && echo "  ${dir}/trajectory_plot.png"
  [[ -f "${dir}/error_plot.png" ]]      && echo "  ${dir}/error_plot.png"
done
