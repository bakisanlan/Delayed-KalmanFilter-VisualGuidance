"""
Measurement function for the camera emulator.

Takes interceptor LLA, target LLA, and body-to-NED attitude as inputs,
and computes the target's pixel location using the pinhole+radtan camera
model.  Returns ``None`` when the target is outside the camera FOV.
"""

import numpy as np
from typing import Optional, Tuple

from .camera_model import PinholeRadtanCamera
from .geo_utils import lla_to_ned


R_ENU_TO_NED = np.array([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
], dtype=np.float64)

R_FRD_TO_FLU = np.array([
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
], dtype=np.float64)


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    Convert a unit quaternion (ROS convention: x, y, z, w) to a 3×3 rotation
    matrix.  The returned matrix is R_body2ned (rotates body-frame vectors
    into the NED frame).

    Parameters
    ----------
    qx, qy, qz, qw : float
        Quaternion components (Hamilton convention, scalar-last).

    Returns
    -------
    np.ndarray, shape (3, 3)
    """
    # Normalise
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    R = np.array([
        [1 - 2 * (qy**2 + qz**2),     2 * (qx*qy - qz*qw),     2 * (qx*qz + qy*qw)],
        [    2 * (qx*qy + qz*qw), 1 - 2 * (qx**2 + qz**2),     2 * (qy*qz - qx*qw)],
        [    2 * (qx*qz - qy*qw),     2 * (qy*qz + qx*qw), 1 - 2 * (qx**2 + qy**2)],
    ], dtype=np.float64)

    return R


def mavros_enu_flu_to_ned_frd_rotation(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    Convert a MAVROS local odometry quaternion into the R_body2ned matrix.

    MAVROS publishes local orientation in ENU with the vehicle body expressed
    in FLU. The camera emulator expects body-to-NED with the body expressed
    in FRD.
    """
    R_flu2enu = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    return R_ENU_TO_NED @ R_flu2enu @ R_FRD_TO_FLU


def measure(
    camera: PinholeRadtanCamera,
    interceptor_lla: Tuple[float, float, float],
    target_lla: Tuple[float, float, float],
    R_body2ned: np.ndarray,
) -> Optional[Tuple[float, float]]:
    """
    Compute the target pixel location as seen by the interceptor's camera.

    Pipeline
    --------
    1. Target LLA → NED relative to interceptor LLA.
    2. NED → body frame  :  p_body = R_body2ned^T  @ p_ned
    3. Body → camera     :  p_cam  = R_c2b^T       @ p_body
    4. Project p_cam → pixel (u, v) with radtan distortion.
    5. Check FOV; add noise.

    Parameters
    ----------
    camera : PinholeRadtanCamera
        The camera model instance.
    interceptor_lla : (lat_deg, lon_deg, alt_m)
        Interceptor position in geodetic coordinates.
    target_lla : (lat_deg, lon_deg, alt_m)
        Target position in geodetic coordinates.
    R_body2ned : np.ndarray, shape (3, 3)
        Rotation matrix from body frame to NED frame.

    Returns
    -------
    (u, v) : tuple of float, or None
        Noisy pixel coordinates if target is in FOV, else ``None``.
    """
    # 1. Target position in NED (relative to interceptor)
    p_ned = lla_to_ned(
        target_lla[0], target_lla[1], target_lla[2],
        interceptor_lla[0], interceptor_lla[1], interceptor_lla[2],
    )

    # 2. NED → body frame
    R_ned2body = R_body2ned.T
    p_body = R_ned2body @ p_ned

    # 3. Body → camera frame
    p_cam = camera.R_b2c @ p_body

    # 4. Project to pixel
    result = camera.project(p_cam)
    if result is None:
        return None  # Behind camera

    u, v = result

    # 5. FOV check
    if not camera.is_in_fov(u, v):
        return None

    # 6. Add realistic pixel noise
    u, v = camera.add_noise(u, v)

    return (u, v)
