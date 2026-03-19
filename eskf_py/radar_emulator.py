"""
radar_emulator.py
=================
Python translation of RadarEmulator.m.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

if __package__ in (None, ''):
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from eskf_py.geodesy import NEDFrame


class RadarEmulator:
    """Physically-motivated radar measurement emulator in spherical space."""

    def __init__(
        self,
        radar_lla,
        ref_lla0,
        dt: float,
        rmse_range: float = 3.0,
        rmse_azimuth: float = math.radians(1.3),
        rmse_elevation: float = math.radians(3.3),
        rmse_doppler: float = 0.01,
        rng: np.random.Generator | None = None,
    ):
        self.rmse_range = float(rmse_range)
        self.rmse_azimuth = float(rmse_azimuth)
        self.rmse_elevation = float(rmse_elevation)
        self.rmse_doppler = float(rmse_doppler)
        self.radar_lla = np.asarray(radar_lla, dtype=float).reshape(3)
        self.ref_lla0 = np.asarray(ref_lla0, dtype=float).reshape(3)
        self.dt = float(dt)
        self._rng = rng if rng is not None else np.random.default_rng()
        self._radar_frame = NEDFrame(self.radar_lla)
        self._ref_frame = NEDFrame(self.ref_lla0)

        if self.dt <= 0.0:
            raise ValueError("dt must be positive.")
        if self.rmse_range <= 0.0:
            raise ValueError("rmse_range must be positive.")
        if self.rmse_azimuth <= 0.0:
            raise ValueError("rmse_azimuth must be positive.")
        if self.rmse_elevation <= 0.0:
            raise ValueError("rmse_elevation must be positive.")
        if self.rmse_doppler <= 0.0:
            raise ValueError("rmse_doppler must be positive.")

    def emulate_measurement(self, target_lla, target_vel_ned):
        """Return noisy target position/velocity in ref_lla0 NED."""
        target_lla = np.asarray(target_lla, dtype=float).reshape(3)
        target_vel_ned = np.asarray(target_vel_ned, dtype=float).reshape(3)

        p_true_radar = self._radar_frame.lla_to_ned(target_lla)
        r_true = float(np.linalg.norm(p_true_radar))
        if r_true <= 1e-3:
            raise ValueError("Target and radar are effectively co-located.")

        x, y, z = p_true_radar
        r_xy = math.hypot(x, y)
        r = math.sqrt(x * x + y * y + z * z)
        theta = math.atan2(y, x)
        phi = math.atan2(-z, r_xy)

        r_m = max(r + self.rmse_range * self._rng.standard_normal(), 0.01)
        theta_m = theta + self.rmse_azimuth * self._rng.standard_normal()
        phi_m = phi + self.rmse_elevation * self._rng.standard_normal()

        R_pos, R_vel = self.get_cartesian_covariance(r_m, theta_m, phi_m)

        #print(R_pos)
        #print(R_vel)

        cos_phi_m = math.cos(phi_m)
        x_n_m = r_m * cos_phi_m * math.cos(theta_m)
        x_e_m = r_m * cos_phi_m * math.sin(theta_m)
        x_d_m = -r_m * math.sin(phi_m)
        p_noisy_ned_radar = np.array([x_n_m, x_e_m, x_d_m], dtype=float)

        noisy_ecef = self._radar_frame.ned_to_ecef(p_noisy_ned_radar)
        noisy_pos_ned0 = self._ref_frame.ecef_to_ned(noisy_ecef)

        v_n, v_e, v_d = target_vel_ned
        u_r = p_true_radar / r
        rdot = float(np.dot(target_vel_ned, u_r))

        if r_xy < 1e-6:
            thetadot = 0.0
            phidot = 0.0
        else:
            thetadot = (x * v_e - y * v_n) / (r_xy * r_xy)
            rdot_xy = (x * v_n + y * v_e) / r_xy
            phidot = (r_xy * (-v_d) - (-z) * rdot_xy) / (r_xy * r)

        rdot_m = rdot + self.rmse_doppler * self._rng.standard_normal()
        thetadot_m = thetadot + (self.rmse_azimuth / self.dt) * self._rng.standard_normal()
        phidot_m = phidot + (self.rmse_elevation / self.dt) * self._rng.standard_normal()

        cp_m = math.cos(phi_m)
        sp_m = math.sin(phi_m)
        ct_m = math.cos(theta_m)
        st_m = math.sin(theta_m)

        # # DEBUG NO NOISE
        # r_m = r
        # rdot_m = rdot 
        # thetadot_m = thetadot 
        # phidot_m = phidot 

        # cp_m = math.cos(phi)
        # sp_m = math.sin(phi)
        # ct_m = math.cos(theta)
        # st_m = math.sin(theta)
    

        v_n_m = (
            rdot_m * cp_m * ct_m
            - r_m * phidot_m * sp_m * ct_m
            - r_m * thetadot_m * cp_m * st_m
        )
        v_e_m = (
            rdot_m * cp_m * st_m
            - r_m * phidot_m * sp_m * st_m
            + r_m * thetadot_m * cp_m * ct_m
        )
        v_d_m = -rdot_m * sp_m - r_m * phidot_m * cp_m
        noisy_vel_ned0 = np.array([v_n_m, v_e_m, v_d_m], dtype=float)

        return noisy_pos_ned0, noisy_vel_ned0, R_pos, R_vel

    def get_cartesian_covariance(self, r: float, theta: float, phi: float):
        cp = math.cos(phi)
        sp = math.sin(phi)
        ct = math.cos(theta)
        st = math.sin(theta)

        jac = np.array([
            [cp * ct, -r * cp * st, -r * sp * ct],
            [cp * st,  r * cp * ct, -r * sp * st],
            [-sp,      0.0,         -r * cp],
        ], dtype=float)

        s_pos = np.diag([
            self.rmse_range ** 2,
            self.rmse_azimuth ** 2,
            self.rmse_elevation ** 2,
        ])
        s_vel = np.diag([
            self.rmse_doppler ** 2,
            (self.rmse_azimuth / self.dt) ** 2,
            (self.rmse_elevation / self.dt) ** 2,
        ])

        r_pos = jac @ s_pos @ jac.T
        r_vel = jac @ s_vel @ jac.T
        r_pos = 0.5 * (r_pos + r_pos.T)
        r_vel = 0.5 * (r_vel + r_vel.T)
        return r_pos, r_vel

    def getCartesianCovariance(self, r: float, theta: float, phi: float):
        """MATLAB-compatible alias."""
        return self.get_cartesian_covariance(r, theta, phi)
