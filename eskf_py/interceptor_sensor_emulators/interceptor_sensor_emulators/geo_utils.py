"""
Geodetic utility functions.

Provides conversions between WGS-84 LLA (latitude, longitude, altitude),
ECEF, and local NED coordinates.
"""

import numpy as np
from typing import Tuple

# WGS-84 ellipsoid constants
_A = 6378137.0  # semi-major axis (m)
_F = 1.0 / 298.257223563  # flattening
_B = _A * (1.0 - _F)  # semi-minor axis
_E2 = 1.0 - (_B / _A) ** 2  # first eccentricity squared


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """
    Convert WGS-84 geodetic coordinates to ECEF.

    Parameters
    ----------
    lat_deg : float  – Latitude in degrees.
    lon_deg : float  – Longitude in degrees.
    alt_m   : float  – Altitude above WGS-84 ellipsoid in metres.

    Returns
    -------
    np.ndarray, shape (3,)  – ECEF coordinates [x, y, z] in metres.
    """
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)

    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    N = _A / np.sqrt(1.0 - _E2 * sin_lat ** 2)  # prime vertical radius

    x = (N + alt_m) * cos_lat * cos_lon
    y = (N + alt_m) * cos_lat * sin_lon
    z = (N * (1.0 - _E2) + alt_m) * sin_lat

    return np.array([x, y, z], dtype=np.float64)


def _ecef_to_ned_rotation(lat_deg: float, lon_deg: float) -> np.ndarray:
    """Return the 3×3 rotation matrix from ECEF to NED at a reference LLA."""
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)

    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    # Rotation matrix ECEF → NED
    R = np.array([
        [-sin_lat * cos_lon, -sin_lat * sin_lon,  cos_lat],
        [-sin_lon,            cos_lon,             0.0    ],
        [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat],
    ], dtype=np.float64)

    return R


def ecef_to_ned(
    ecef: np.ndarray,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
) -> np.ndarray:
    """
    Convert ECEF position to NED relative to a reference point.

    Parameters
    ----------
    ecef        : np.ndarray, shape (3,)  – ECEF position [x, y, z] in metres.
    ref_lat_deg : float – Reference latitude in degrees.
    ref_lon_deg : float – Reference longitude in degrees.
    ref_alt_m   : float – Reference altitude in metres.

    Returns
    -------
    np.ndarray, shape (3,) – NED position [north, east, down] in metres.
    """
    ref_ecef = lla_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    d_ecef = ecef - ref_ecef
    R = _ecef_to_ned_rotation(ref_lat_deg, ref_lon_deg)
    return R @ d_ecef


def lla_to_ned(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
) -> np.ndarray:
    """
    Convert LLA to local NED coordinates relative to a reference LLA.

    Parameters
    ----------
    lat_deg, lon_deg, alt_m       : Target position (deg, deg, m).
    ref_lat_deg, ref_lon_deg, ref_alt_m : Reference/origin position (deg, deg, m).

    Returns
    -------
    np.ndarray, shape (3,)  – NED position [north, east, down] in metres.
    """
    ecef = lla_to_ecef(lat_deg, lon_deg, alt_m)
    return ecef_to_ned(ecef, ref_lat_deg, ref_lon_deg, ref_alt_m)
