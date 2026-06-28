"""Coordinate frame transformations: LLA, ECEF, ENU."""

from __future__ import annotations

import numpy as np

# WGS84 ellipsoid constants
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """Convert geodetic coordinates (degrees, meters) to ECEF [x, y, z]."""
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return np.array([x, y, z], dtype=np.float64)


def ecef_to_enu(ecef: np.ndarray, origin_lla: np.ndarray) -> np.ndarray:
    """Convert ECEF point(s) to local ENU relative to origin LLA."""
    ecef = np.atleast_2d(ecef)
    origin_ecef = lla_to_ecef(origin_lla[0], origin_lla[1], origin_lla[2])
    diff = ecef - origin_ecef

    lat0 = np.deg2rad(origin_lla[0])
    lon0 = np.deg2rad(origin_lla[1])
    sin_lat = np.sin(lat0)
    cos_lat = np.cos(lat0)
    sin_lon = np.sin(lon0)
    cos_lon = np.cos(lon0)

    rotation = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=np.float64,
    )
    enu = (rotation @ diff.T).T
    return enu[0] if enu.shape[0] == 1 else enu


def lla_to_enu(
    lat_deg: np.ndarray,
    lon_deg: np.ndarray,
    alt_m: np.ndarray,
    origin_lla: np.ndarray | None = None,
) -> np.ndarray:
    """Convert LLA arrays to ENU using the first sample as origin by default."""
    lat_deg = np.asarray(lat_deg, dtype=np.float64)
    lon_deg = np.asarray(lon_deg, dtype=np.float64)
    alt_m = np.asarray(alt_m, dtype=np.float64)

    if origin_lla is None:
        origin_lla = np.array([lat_deg[0], lon_deg[0], alt_m[0]], dtype=np.float64)

    ecef = np.column_stack(
        [
            lla_to_ecef(lat, lon, alt)[None, :]
            for lat, lon, alt in zip(lat_deg, lon_deg, alt_m)
        ]
    ).reshape(-1, 3)
    return ecef_to_enu(ecef, origin_lla)


def enu_to_ecef(enu: np.ndarray, origin_lla: np.ndarray) -> np.ndarray:
    """Convert ENU point(s) to ECEF."""
    enu = np.atleast_2d(enu)
    origin_ecef = lla_to_ecef(origin_lla[0], origin_lla[1], origin_lla[2])

    lat0 = np.deg2rad(origin_lla[0])
    lon0 = np.deg2rad(origin_lla[1])
    sin_lat = np.sin(lat0)
    cos_lat = np.cos(lat0)
    sin_lon = np.sin(lon0)
    cos_lon = np.cos(lon0)

    rotation = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=np.float64,
    )
    ecef = (rotation.T @ enu.T).T + origin_ecef
    return ecef[0] if ecef.shape[0] == 1 else ecef
