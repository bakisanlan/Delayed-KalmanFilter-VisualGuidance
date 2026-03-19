"""
geodesy.py
==========
Minimal WGS84 utilities for LLA/ECEF/NED conversions.
"""

from __future__ import annotations

import math

import numpy as np


class NEDFrame:
    """Convert between geodetic, ECEF, and local NED coordinates."""

    WGS84_A = 6378137.0
    WGS84_F = 1.0 / 298.257223563
    WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

    def __init__(self, ref_lla_deg):
        self.ref_lla = self._as_vector(ref_lla_deg)
        self.ref_ecef = self.lla_to_ecef(self.ref_lla)
        self._R_ecef_to_ned = self._ecef_to_ned_rotation(
            self.ref_lla[0], self.ref_lla[1])

    @staticmethod
    def _as_vector(vec) -> np.ndarray:
        arr = np.asarray(vec, dtype=float).reshape(3)
        return arr

    @classmethod
    def lla_to_ecef(cls, lla_deg) -> np.ndarray:
        lat_deg, lon_deg, alt_m = cls._as_vector(lla_deg)
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)

        sin_lat = math.sin(lat)
        cos_lat = math.cos(lat)
        sin_lon = math.sin(lon)
        cos_lon = math.cos(lon)

        N = cls.WGS84_A / math.sqrt(1.0 - cls.WGS84_E2 * sin_lat * sin_lat)

        x = (N + alt_m) * cos_lat * cos_lon
        y = (N + alt_m) * cos_lat * sin_lon
        z = (N * (1.0 - cls.WGS84_E2) + alt_m) * sin_lat
        return np.array([x, y, z], dtype=float)

    @staticmethod
    def _ecef_to_ned_rotation(lat_deg: float, lon_deg: float) -> np.ndarray:
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)

        sin_lat = math.sin(lat)
        cos_lat = math.cos(lat)
        sin_lon = math.sin(lon)
        cos_lon = math.cos(lon)

        return np.array([
            [-sin_lat * cos_lon, -sin_lat * sin_lon,  cos_lat],
            [-sin_lon,            cos_lon,            0.0],
            [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat],
        ], dtype=float)

    def ecef_to_ned(self, ecef_xyz) -> np.ndarray:
        ecef = self._as_vector(ecef_xyz)
        return self._R_ecef_to_ned @ (ecef - self.ref_ecef)

    def ned_to_ecef(self, ned_xyz) -> np.ndarray:
        ned = self._as_vector(ned_xyz)
        return self.ref_ecef + self._R_ecef_to_ned.T @ ned

    def lla_to_ned(self, lla_deg) -> np.ndarray:
        return self.ecef_to_ned(self.lla_to_ecef(lla_deg))
