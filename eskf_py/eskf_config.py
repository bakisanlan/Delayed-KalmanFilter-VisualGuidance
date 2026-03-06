"""
eskf_config.py
==============
YAML configuration loader for the ESKF.

Mirrors eskf_config.cpp / eskf_config.hpp.
"""

from __future__ import annotations
import yaml
import numpy as np
from eskf_py.eskf_types import ESKFParams


def load_config(filename: str) -> ESKFParams:
    """Load ESKFParams from a YAML file."""
    with open(filename, 'r') as f:
        cfg = yaml.safe_load(f)
    return _parse_config(cfg)


def load_config_from_dict(cfg: dict) -> ESKFParams:
    """Build ESKFParams from an already-parsed YAML dict."""
    return _parse_config(cfg)


# ---------------------------------------------------------------------------
def _get(node: dict | None, key: str, default):
    """Safe getter with default."""
    if node is None:
        return default
    return node.get(key, default)


def _parse_rotation_matrix(arr) -> np.ndarray:
    """Parse a flat 9-element list into a 3x3 rotation matrix."""
    if arr is None or len(arr) != 9:
        return np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=float)
    R = np.array(arr, dtype=float).reshape(3, 3)
    return R


def _parse_config(cfg: dict) -> ESKFParams:
    p = ESKFParams()

    # Topics
    topics = cfg.get('topics', {})
    p.topic_imu   = _get(topics, 'imu',          p.topic_imu)
    p.topic_mag   = _get(topics, 'mag',           p.topic_mag)
    p.topic_image = _get(topics, 'image',         p.topic_image)
    p.topic_radar = _get(topics, 'radar',         p.topic_radar)
    p.topic_odom  = _get(topics, 'output_state',  p.topic_odom)
    p.topic_pbar  = _get(topics, 'output_pose',   p.topic_pbar)

    # Timing
    timing = cfg.get('timing', {})
    if timing:
        imu_rate  = _get(timing, 'imu_rate_hz',  200.0)
        eskf_rate = _get(timing, 'eskf_rate_hz', 200.0)
        p.dt_imu   = 1.0 / imu_rate
        p.dt_eskf  = 1.0 / eskf_rate
        delay_ms   = _get(timing, 'image_delay_ms', 80.0)
        p.image_delay       = delay_ms / 1000.0
        p.image_timeout_sec = _get(timing, 'image_timeout_sec', p.image_timeout_sec)

    # IMU noise
    imu_noise = cfg.get('imu_noise', {})
    p.sigma_a_n     = _get(imu_noise, 'sigma_a_n',     p.sigma_a_n)
    p.sigma_omega_n = _get(imu_noise, 'sigma_omega_n', p.sigma_omega_n)
    p.sigma_a_w     = _get(imu_noise, 'sigma_a_w',     p.sigma_a_w)
    p.sigma_omega_w = _get(imu_noise, 'sigma_omega_w', p.sigma_omega_w)

    # Measurement noise
    meas = cfg.get('measurement_noise', {})
    p.sigma_img       = _get(meas, 'image_sigma',     p.sigma_img)
    p.sigma_radar_pos = _get(meas, 'radar_sigma_pos', p.sigma_radar_pos)
    p.sigma_radar_vel = _get(meas, 'radar_sigma_vel', p.sigma_radar_vel)

    # Radar
    radar = cfg.get('radar', {})
    p.use_vr = _get(radar, 'use_vr', p.use_vr)

    # Initial covariance
    init_cov = cfg.get('initial_covariance', {})
    p.init_sigma_attitude = _get(init_cov, 'attitude',   p.init_sigma_attitude)
    p.init_sigma_position = _get(init_cov, 'position',   p.init_sigma_position)
    p.init_sigma_velocity = _get(init_cov, 'velocity',   p.init_sigma_velocity)
    p.init_sigma_pbar     = _get(init_cov, 'pbar',       p.init_sigma_pbar)
    p.init_sigma_bgyr     = _get(init_cov, 'gyro_bias',  p.init_sigma_bgyr)
    p.init_sigma_bacc     = _get(init_cov, 'accel_bias', p.init_sigma_bacc)

    # Camera
    camera = cfg.get('camera', {})
    if camera and 'R_b2c' in camera:
        p.R_b2c = _parse_rotation_matrix(camera['R_b2c'])

    # Chi-square gating
    chi2 = cfg.get('chi2_gating', {})
    p.enable_false_detection_image = _get(chi2, 'enable_false_detection_image', p.enable_false_detection_image)
    p.enable_false_detection_radar = _get(chi2, 'enable_false_detection_radar', p.enable_false_detection_radar)
    p.chi2_threshold_image         = _get(chi2, 'image_threshold',              p.chi2_threshold_image)
    p.chi2_threshold_radar         = _get(chi2, 'radar_threshold',              p.chi2_threshold_radar)

    # History
    hist = cfg.get('history', {})
    p.history_length = _get(hist, 'buffer_length', p.history_length)

    # Constants
    consts = cfg.get('constants', {})
    p.gravity = _get(consts, 'gravity', p.gravity)

    # ZUPT
    zupt = cfg.get('zupt', {})
    p.zupt_alpha          = _get(zupt, 'alpha',          p.zupt_alpha)
    p.chi2_threshold_zupt = _get(zupt, 'chi2_threshold', p.chi2_threshold_zupt)

    # Logging
    logging = cfg.get('logging', {})
    p.log_enabled   = _get(logging, 'enable',    p.log_enabled)
    p.log_rate_hz   = _get(logging, 'rate_hz',   p.log_rate_hz)
    p.log_file_path = _get(logging, 'file_path', p.log_file_path)

    # Magnetometer
    mag = cfg.get('magnetometer', {})
    p.enable_mag = _get(mag, 'enable', p.enable_mag)
    if 'B_ned' in mag and len(mag['B_ned']) == 3:
        p.B_ned = np.array(mag['B_ned'], dtype=float)
    p.sigma_mag_n                = _get(mag, 'sigma_mag_n',         p.sigma_mag_n)
    p.sigma_mag_w                = _get(mag, 'sigma_mag_w',         p.sigma_mag_w)
    p.init_sigma_bmag            = _get(mag, 'init_sigma_bmag',     p.init_sigma_bmag)
    p.enable_false_detection_mag = _get(mag, 'enable_false_detection', p.enable_false_detection_mag)
    p.chi2_threshold_mag         = _get(mag, 'chi2_threshold',      p.chi2_threshold_mag)

    return p


def print_config(params: ESKFParams) -> None:
    """Print configuration summary to stdout."""
    print("\n============ ESKF Configuration ============")
    print(f"Timing:")
    print(f"  IMU rate:      {1.0/params.dt_imu:.1f} Hz")
    print(f"  ESKF rate:     {1.0/params.dt_eskf:.1f} Hz")
    print(f"  Image delay:   {params.image_delay*1000:.1f} ms")
    print(f"  Image timeout: {params.image_timeout_sec:.1f} s")
    print(f"IMU Noise:")
    print(f"  sigma_omega_n: {params.sigma_omega_n:.6f} rad/s")
    print(f"  sigma_a_n:     {params.sigma_a_n:.6f} m/s²")
    print(f"  sigma_omega_w: {params.sigma_omega_w:.2e} rad/s√s")
    print(f"  sigma_a_w:     {params.sigma_a_w:.2e} m/s²√s")
    print(f"Measurement Noise:")
    print(f"  Image:         {params.sigma_img:.4f}")
    print(f"  Radar pos:     {params.sigma_radar_pos:.2f} m")
    print(f"  Radar vel:     {params.sigma_radar_vel:.2f} m/s")
    print(f"Radar: use_vr={params.use_vr}")
    print(f"Gravity: {params.gravity:.4f} m/s²")
    print(f"Initial Covariance (1σ):")
    print(f"  Attitude: {params.init_sigma_attitude:.4f} rad")
    print(f"  Position: {params.init_sigma_position:.2f} m")
    print(f"  Velocity: {params.init_sigma_velocity:.2f} m/s")
    print(f"  Pbar:     {params.init_sigma_pbar:.4f}")
    print(f"  Gyro bias:{params.init_sigma_bgyr:.5f} rad/s")
    print(f"  Acc bias: {params.init_sigma_bacc:.4f} m/s²")
    print(f"History buffer: {params.history_length} entries")
    print(f"Chi² Gating:")
    print(f"  Image: {'enabled' if params.enable_false_detection_image else 'disabled'} thresh={params.chi2_threshold_image:.2f}")
    print(f"  Radar: {'enabled' if params.enable_false_detection_radar else 'disabled'} thresh={params.chi2_threshold_radar:.2f}")
    print(f"ZUPT: alpha={params.zupt_alpha:.2f}  thresh={params.chi2_threshold_zupt:.2f}")
    print(f"Logging: {'enabled' if params.log_enabled else 'disabled'}"
          + (f"  rate={params.log_rate_hz:.1f} Hz  file={params.log_file_path}" if params.log_enabled else ""))
    print(f"Magnetometer: {'enabled' if params.enable_mag else 'disabled'}")
    if params.enable_mag:
        print(f"  B_ned: {params.B_ned}")
        print(f"  sigma_n={params.sigma_mag_n:.3f}  sigma_w={params.sigma_mag_w:.5f}")
        print(f"  init_bmag={params.init_sigma_bmag:.2f}")
    print("=============================================\n")
