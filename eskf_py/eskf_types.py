"""
eskf_types.py
=============
Type definitions, state indices, and constants for the ESKF.

Mirrors eskf_types.hpp.

Nominal state (21-D): [q(4), pr(3), vr(3), pbar(2), bgyr(3), bacc(3), bmag(3)]
Error  state (20-D): [dθ(3), dpr(3), dvr(3), dpbar(2), dbgyr(3), dbacc(3), dbmag(3)]
"""

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

# ============================================================================
# Nominal State Indices  (21-D)
# ============================================================================
Q_START      = 0;  Q_SIZE      = 4   # quaternion [w, x, y, z]
PR_START     = 4;  PR_SIZE     = 3   # relative position in earth frame
VR_START     = 7;  VR_SIZE     = 3   # relative velocity in earth frame
PBAR_START   = 10; PBAR_SIZE   = 2   # normalized image coordinates
BGYR_START   = 12; BGYR_SIZE   = 3   # gyroscope bias
BACC_START   = 15; BACC_SIZE   = 3   # accelerometer bias
BMAG_START   = 18; BMAG_SIZE   = 3   # magnetometer bias
NOMINAL_SIZE = 21

# ============================================================================
# Error State Indices  (20-D)
# ============================================================================
DTHETA_START = 0;  DTHETA_SIZE = 3   # attitude error (axis-angle)
DPR_START    = 3;  DPR_SIZE    = 3   # position error
DVR_START    = 6;  DVR_SIZE    = 3   # velocity error
DPBAR_START  = 9;  DPBAR_SIZE  = 2   # image feature error
DBGYR_START  = 11; DBGYR_SIZE  = 3   # gyro bias error
DBACC_START  = 14; DBACC_SIZE  = 3   # accel bias error
DBMAG_START  = 17; DBMAG_SIZE  = 3   # mag bias error
ERROR_SIZE   = 20

# ============================================================================
# Process Noise Indices  (15-D)
# ============================================================================
OMEGA_N_START = 0;  OMEGA_N_SIZE = 3   # gyro measurement noise
A_N_START     = 3;  A_N_SIZE     = 3   # accel measurement noise
OMEGA_W_START = 6;  OMEGA_W_SIZE = 3   # gyro bias random walk
A_W_START     = 9;  A_W_SIZE     = 3   # accel bias random walk
MAG_W_START   = 12; MAG_W_SIZE   = 3   # mag bias random walk
NOISE_SIZE    = 15

# ============================================================================
# Physical Constants
# ============================================================================
DEFAULT_GRAVITY      = 9.81
E3                   = np.array([0.0, 0.0, 1.0])   # NED gravity direction unit vector
MIN_DEPTH            = 0.1                           # minimum camera depth [m]
SMALL_ANGLE_THRESHOLD = 1e-10                        # threshold for small-angle approx

# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class IMUMeasurement:
    """IMU measurement containing gyroscope and accelerometer data."""
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [rad/s]
    accel: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [m/s²]
    timestamp: float = 0.0                                            # [s]


@dataclass
class StateHistoryEntry:
    """State history entry for delayed measurement handling."""
    x: np.ndarray = field(default_factory=lambda: np.zeros(NOMINAL_SIZE))  # nominal state
    P: np.ndarray = field(default_factory=lambda: np.zeros((ERROR_SIZE, ERROR_SIZE)))  # covariance
    imu: IMUMeasurement = field(default_factory=IMUMeasurement)
    timestamp: float = 0.0


@dataclass
class ESKFParams:
    """Configuration parameters for the ESKF (mirrors ESKFParams struct)."""

    # ROS2 topic names
    topic_imu:   str = "/mavros/imu/data_raw"
    topic_mag:   str = "/mavros/imu/mag"
    topic_image: str = "/yolo/target"
    topic_radar: str = "/radar/pr"
    topic_odom:  str = "/eskf/odom"
    topic_pbar:  str = "/eskf/pbar"

    # Timing
    dt_imu:           float = 1.0 / 200.0   # IMU sampling period [s]
    dt_eskf:          float = 1.0 / 200.0   # ESKF update period [s]
    image_delay:      float = 0.080          # Image processing delay [s]
    image_timeout_sec: float = 2.0           # Freeze pbar after this [s]

    # IMU noise (continuous-time)
    sigma_omega_n: float = 0.01     # gyro noise std [rad/s]
    sigma_a_n:     float = 0.1      # accel noise std [m/s²]
    sigma_omega_w: float = 1e-5     # gyro bias walk [rad/s√s]
    sigma_a_w:     float = 1e-4     # accel bias walk [m/s²√s]

    # Measurement noise
    sigma_img:        float = 0.005  # image coordinate noise
    sigma_radar_pos:  float = 1.0    # radar position noise [m]
    sigma_radar_vel:  float = 0.5    # radar velocity noise [m/s]

    # Camera-to-body rotation (body → camera)
    R_b2c: np.ndarray = field(default_factory=lambda: np.array([
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 0]], dtype=float))

    # History buffer length (for delayed measurements)
    history_length: int = 25

    # Initial covariance standard deviations
    init_sigma_attitude: float = 0.05   # [rad]
    init_sigma_position: float = 3.0    # [m]
    init_sigma_velocity: float = 0.5    # [m/s]
    init_sigma_pbar:     float = 0.1    # normalized
    init_sigma_bgyr:     float = 0.005  # [rad/s]
    init_sigma_bacc:     float = 0.05   # [m/s²]

    # Chi-square gating
    enable_false_detection_image: bool  = True
    enable_false_detection_radar: bool  = True
    chi2_threshold_image:         float = 18.42   # 2 DoF
    chi2_threshold_radar:         float = 27.86   # 6 DoF (or 3 DoF if use_vr=False)
    use_vr:                       bool  = False    # use radar velocity

    # Physical constants
    gravity: float = 9.81   # [m/s²]

    # ZUPT
    zupt_alpha:          float = 1.0    # noise inflation for chi-square test
    chi2_threshold_zupt: float = 12.59  # 6 DoF, 95%

    # Data logging
    log_enabled:    bool  = False
    log_rate_hz:    float = 20.0
    log_file_path:  str   = "log/eskf_log.csv"

    # Magnetometer
    enable_mag:                 bool      = True
    B_ned:                      np.ndarray = field(
        default_factory=lambda: np.array([0.5097, 0.0541, 0.8545]))
    sigma_mag_n:                float = 0.10
    sigma_mag_w:                float = 0.0002
    init_sigma_bmag:            float = 1.0
    enable_false_detection_mag: bool  = True
    chi2_threshold_mag:         float = 16.27   # 3 DoF


# ============================================================================
# State Accessor Helpers
# ============================================================================

def get_quaternion(x: np.ndarray) -> np.ndarray:
    """Return quaternion [w, x, y, z] from nominal state."""
    return x[Q_START:Q_START + Q_SIZE].copy()


def set_quaternion(x: np.ndarray, q: np.ndarray) -> None:
    """Set quaternion [w, x, y, z] in nominal state (in-place)."""
    x[Q_START:Q_START + Q_SIZE] = q


def get_position(x: np.ndarray) -> np.ndarray:
    return x[PR_START:PR_START + PR_SIZE].copy()


def get_velocity(x: np.ndarray) -> np.ndarray:
    return x[VR_START:VR_START + VR_SIZE].copy()


def get_pbar(x: np.ndarray) -> np.ndarray:
    return x[PBAR_START:PBAR_START + PBAR_SIZE].copy()


def get_gyro_bias(x: np.ndarray) -> np.ndarray:
    return x[BGYR_START:BGYR_START + BGYR_SIZE].copy()


def get_accel_bias(x: np.ndarray) -> np.ndarray:
    return x[BACC_START:BACC_START + BACC_SIZE].copy()


def get_mag_bias(x: np.ndarray) -> np.ndarray:
    return x[BMAG_START:BMAG_START + BMAG_SIZE].copy()
