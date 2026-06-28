#!/usr/bin/env bash
# Download missing KITTI raw sequences for 05 and 07.
set -euo pipefail

BASE="/media/slamet/EpsteinFile3/SLAM-Datasets/KITTI-RGBD-GPS"
DL="${BASE}/downloads"
RAW="${BASE}/raw"
S3="https://s3.eu-central-1.amazonaws.com/avg-kitti"

mkdir -p "${DL}" "${RAW}"

download_and_unzip() {
  local url="$1"
  local outdir="$2"
  local fname
  fname="$(basename "${url}")"
  echo "Downloading ${fname}"
  wget -c -O "${DL}/${fname}" "${url}"
  echo "Extracting ${fname}"
  unzip -o "${DL}/${fname}" -d "${outdir}"
}

SEQUENCES=(
  "2011_09_30_drive_0018"
  "2011_09_30_drive_0027"
)

for seq in "${SEQUENCES[@]}"; do
  if [[ -d "${RAW}/2011_09_30/${seq}_sync" ]]; then
    echo "Already present: ${seq}_sync"
    continue
  fi
  download_and_unzip "${S3}/raw_data/${seq}/${seq}_sync.zip" "${RAW}"
done

echo "Download complete."
