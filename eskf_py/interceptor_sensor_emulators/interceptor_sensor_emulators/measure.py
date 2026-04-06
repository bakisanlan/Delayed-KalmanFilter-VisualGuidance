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
    max_range: float = 100.0,
    false_detection_prob: float = 0.0,
    dropout_prob_at_max_range: float = 0.0,
) -> Tuple[Optional[Tuple[float, float]], Optional[Tuple[float, float]], dict]:
    """
    Compute the target pixel location as seen by the interceptor's camera.

    Includes emulation of:
    - Maximal range limitation
    - Distance-dependent dropout
    - False detections in the FOV

    Returns
    -------
    (pbar_meas, pbar_true, info_dict)
        pbar_meas: Noisy/imperfect pixel coordinates if detected (or randomly generated), else None.
        pbar_true: Perfect target coordinates if mathematically inside the FOV, else None.
        info_dict: Dictionary containing extra tracking info like 'dist' and 'p_cam'.
    """
    pbar_true = None
    pbar_meas = None

    # 1. Target position in NED (relative to interceptor)
    p_ned = lla_to_ned(
        target_lla[0], target_lla[1], target_lla[2],
        interceptor_lla[0], interceptor_lla[1], interceptor_lla[2],
    )
    dist = np.linalg.norm(p_ned)

    # 2. NED → body frame
    R_ned2body = R_body2ned.T
    p_body = R_ned2body @ p_ned

    # 3. Body → camera frame
    p_cam = camera.R_b2c @ p_body

    # 4. Project to pixel
    result = camera.project(p_cam)
    if result is not None:
        u_true, v_true = result
        
        # 5. FOV check for the True Location
        if camera.is_in_fov(u_true, v_true):
            pbar_true = ((u_true - camera.cx) / camera.fx,
                         (v_true - camera.cy) / camera.fy)

            # --- Evaluate Perfect Emulation Physical Limits ---
            # Max range limitation & distance-dependent discontinuity
            if dist <= max_range:
                p_drop = dropout_prob_at_max_range * (dist / max_range)
                if np.random.rand() > p_drop:
                    # 6. Add realistic pixel noise
                    u_noisy, v_noisy = camera.add_noise(u_true, v_true)
                    pbar_meas = ((u_noisy - camera.cx) / camera.fx,
                                 (v_noisy - camera.cy) / camera.fy)

    # --- Evaluate False Detections (happens independently of target) ---
    if np.random.rand() < false_detection_prob:
        # Generate random coordinates safely within the FOV
        u_fd = np.random.uniform(0, camera.image_width)
        v_fd = np.random.uniform(0, camera.image_height)
        pbar_meas = ((u_fd - camera.cx) / camera.fx,
                     (v_fd - camera.cy) / camera.fy)

    info_dict = {
        'dist': dist,
        'p_cam': p_cam,
    }
    return pbar_meas, pbar_true, info_dict
