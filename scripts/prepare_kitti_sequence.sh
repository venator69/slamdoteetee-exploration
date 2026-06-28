#!/usr/bin/env bash
# Prepare KITTI odometry sequence directory with image symlinks from raw synced data.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG="${PROJECT_ROOT}/config/fusion_config.yaml"

SEQ="${1:-00}"
PREPARED_ROOT="${PROJECT_ROOT}/prepared_sequences"
python3 - "${SEQ}" "${CONFIG}" "${PREPARED_ROOT}" <<'PY'
import sys
from pathlib import Path
import yaml

seq = sys.argv[1]
config_path = Path(sys.argv[2])
config = yaml.safe_load(config_path.read_text())
root = Path(config["dataset"]["root"])
mapping = config["sequence_mapping"][seq]
raw_drive = root / "raw" / mapping["date"] / mapping["drive"]
odom_seq = root / "odometry" / "dataset" / "sequences" / seq
prepared_root = Path(sys.argv[3]) if len(sys.argv) > 3 else Path.home() / "Dev/kitti_gps_ekf_fusion/prepared_sequences"
prepared_seq = prepared_root / seq

if not raw_drive.exists():
    raise SystemExit(f"Raw drive not found: {raw_drive}. Download KITTI raw data first.")

times = (odom_seq / "times.txt").read_text().strip().splitlines()
n_frames_total = len(times)
fraction = float(config.get("trajectory", {}).get("fraction", 1.0))
n_frames = max(1, int(n_frames_total * fraction))
shutil = __import__("shutil")
prepared_seq.mkdir(parents=True, exist_ok=True)

calib_src = odom_seq / "calib.txt"
calib_dst = prepared_seq / "calib.txt"
if calib_dst.exists() or calib_dst.is_symlink():
    calib_dst.unlink()
calib_dst.symlink_to(calib_src.resolve())

times_dst = prepared_seq / "times.txt"
if times_dst.exists() or times_dst.is_symlink():
    times_dst.unlink()
times_dst.write_text("\n".join(times[:n_frames]) + "\n", encoding="utf-8")

for cam_raw, cam_odom in [("image_00", "image_0"), ("image_01", "image_1")]:
    src_dir = raw_drive / cam_raw / "data"
    dst_dir = prepared_seq / cam_odom
    dst_dir.mkdir(parents=True, exist_ok=True)
    for stale in dst_dir.glob("*.png"):
        stale.unlink()
    for i in range(n_frames):
        src = src_dir / f"{i:010d}.png"
        dst = dst_dir / f"{i:06d}.png"
        if not src.exists():
            raise SystemExit(f"Missing source image: {src}")
        if dst.exists() or dst.is_symlink():
            dst.unlink()
        dst.symlink_to(src.resolve())

print(
    f"Prepared sequence {seq}: {n_frames}/{n_frames_total} stereo frames "
    f"(fraction={fraction}) linked in {prepared_seq}"
)
PY
