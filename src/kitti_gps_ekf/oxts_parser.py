"""KITTI OXTS GPS/IMU data parser."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .coordinates import lla_to_enu


OXTS_FIELDS = [
    "lat",
    "lon",
    "alt",
    "roll",
    "pitch",
    "yaw",
    "vn",
    "ve",
    "vf",
    "vl",
    "vu",
    "ax",
    "ay",
    "az",
    "af",
    "al",
    "au",
    "wx",
    "wy",
    "wz",
    "wf",
    "wl",
    "wu",
    "posacc",
    "velacc",
    "navstat",
    "numsats",
    "posmode",
    "velmode",
    "orimode",
]


@dataclass
class OxtsSample:
    timestamp: float
    lat: float
    lon: float
    alt: float
    roll: float
    pitch: float
    yaw: float
    vn: float
    ve: float
    vf: float
    vl: float
    vu: float
    ax: float
    ay: float
    az: float
    wx: float
    wy: float
    wz: float


def parse_oxts_line(line: str) -> np.ndarray:
    values = np.fromstring(line.strip(), sep=" ")
    if values.size != 30:
        raise ValueError(f"Expected 30 OXTS values, got {values.size}")
    return values


def load_oxts_timestamps(raw_drive_dir: Path) -> list[float]:
    """Load OXTS timestamps as seconds relative to the first sample."""
    ts_file = raw_drive_dir / "oxts" / "timestamps.txt"
    lines = ts_file.read_text().strip().splitlines()
    absolute = [datetime.fromisoformat(line.strip()) for line in lines if line.strip()]
    t0 = absolute[0].timestamp()
    return [dt.timestamp() - t0 for dt in absolute]


def load_oxts_sequence(raw_drive_dir: Path, max_frames: int | None = None) -> list[OxtsSample]:
    """Load synchronized OXTS measurements for a raw KITTI drive."""
    data_dir = raw_drive_dir / "oxts" / "data"
    timestamps = load_oxts_timestamps(raw_drive_dir)
    files = sorted(data_dir.glob("*.txt"))
    if max_frames is not None:
        files = files[:max_frames]
        timestamps = timestamps[:max_frames]

    samples: list[OxtsSample] = []
    for idx, file_path in enumerate(files):
        values = parse_oxts_line(file_path.read_text())
        samples.append(
            OxtsSample(
                timestamp=timestamps[idx],
                lat=values[0],
                lon=values[1],
                alt=values[2],
                roll=values[3],
                pitch=values[4],
                yaw=values[5],
                vn=values[6],
                ve=values[7],
                vf=values[8],
                vl=values[9],
                vu=values[10],
                ax=values[12],
                ay=values[13],
                az=values[14],
                wx=values[17],
                wy=values[18],
                wz=values[19],
            )
        )
    return samples


def oxts_to_local_pose(samples: list[OxtsSample]) -> dict[str, np.ndarray]:
    """
    Convert OXTS to local metric poses (KITTI devkit style).
    Uses Mercator projection for horizontal position, relative to first frame.
    """
    if not samples:
        raise ValueError("No OXTS samples provided")

    lat0 = samples[0].lat
    scale = np.cos(np.deg2rad(lat0))

    def mercator(lat_deg: float, lon_deg: float) -> tuple[float, float]:
        er = 6378137.0
        mx = scale * lon_deg * np.pi * er / 180.0
        my = scale * er * np.log(np.tan((90.0 + lat_deg) * np.pi / 360.0))
        return mx, my

    mx0, my0 = mercator(samples[0].lat, samples[0].lon)
    z0 = samples[0].alt

    positions = []
    quaternions = []
    timestamps = []
    for sample in samples:
        mx, my = mercator(sample.lat, sample.lon)
        pos = np.array([mx - mx0, my - my0, sample.alt - z0], dtype=np.float64)
        rot = Rotation.from_euler("ZYX", [sample.yaw, sample.pitch, sample.roll]).as_quat()
        positions.append(pos)
        quaternions.append(rot)
        timestamps.append(sample.timestamp)

    return {
        "timestamp": np.asarray(timestamps, dtype=np.float64),
        "position": np.asarray(positions, dtype=np.float64),
        "quaternion": np.asarray(quaternions, dtype=np.float64),
    }


def oxts_to_gps_trajectory(samples: list[OxtsSample]) -> dict[str, np.ndarray]:
    """Convert OXTS samples to timestamped ENU GPS trajectory."""
    lat = np.array([s.lat for s in samples], dtype=np.float64)
    lon = np.array([s.lon for s in samples], dtype=np.float64)
    alt = np.array([s.alt for s in samples], dtype=np.float64)
    timestamps = np.array([s.timestamp for s in samples], dtype=np.float64)

    enu = lla_to_enu(lat, lon, alt)
    return {
        "timestamp": timestamps,
        "east": enu[:, 0],
        "north": enu[:, 1],
        "up": enu[:, 2],
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "roll": np.array([s.roll for s in samples]),
        "pitch": np.array([s.pitch for s in samples]),
        "yaw": np.array([s.yaw for s in samples]),
        "vn": np.array([s.vn for s in samples]),
        "ve": np.array([s.ve for s in samples]),
        "vf": np.array([s.vf for s in samples]),
        "ax": np.array([s.ax for s in samples]),
        "ay": np.array([s.ay for s in samples]),
        "az": np.array([s.az for s in samples]),
        "wx": np.array([s.wx for s in samples]),
        "wy": np.array([s.wy for s in samples]),
        "wz": np.array([s.wz for s in samples]),
    }
