"""
eskf_math.py
============
Mathematical utilities for the ESKF.

Mirrors eskf_math.hpp: skew-symmetric matrices, quaternion exponential map,
Rodrigues rotation, IBVS Jacobians, and process noise computation.
"""

from __future__ import annotations
import numpy as np
from eskf_py.eskf_types import (
    ESKFParams, SMALL_ANGLE_THRESHOLD, MIN_DEPTH,
    OMEGA_N_START, A_N_START, OMEGA_W_START, A_W_START, MAG_W_START,
)


# ============================================================================
# Skew-Symmetric Matrix
# ============================================================================

def skew(v: np.ndarray) -> np.ndarray:
    """
    Compute 3x3 skew-symmetric (cross-product) matrix from a 3-vector:
        S * x = v × x
    """
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    return np.array([
        [  0.0, -vz,  vy],
        [  vz,  0.0, -vx],
        [ -vy,   vx, 0.0],
    ], dtype=float)


# ============================================================================
# Quaternion Operations  (stored as [w, x, y, z])
# ============================================================================

def exp_quaternion(delta_theta: np.ndarray) -> np.ndarray:
    """
    Quaternion exponential map: convert rotation vector to unit quaternion.
        q = [cos(|δθ|/2);  sin(|δθ|/2) * δθ/|δθ|]

    Returns array [w, x, y, z].
    """
    theta = np.linalg.norm(delta_theta)

    if theta < SMALL_ANGLE_THRESHOLD:
        # Small-angle approximation: q ≈ [1, δθ/2]
        q = np.array([1.0,
                      0.5 * delta_theta[0],
                      0.5 * delta_theta[1],
                      0.5 * delta_theta[2]])
    else:
        half = 0.5 * theta
        axis = delta_theta / theta
        s = np.sin(half)
        q = np.array([np.cos(half), s * axis[0], s * axis[1], s * axis[2]])

    return q / np.linalg.norm(q)


def exp_rotation(phi: np.ndarray) -> np.ndarray:
    """
    Rotation matrix exponential map (Rodrigues' formula).
        R = I·cos(θ) + sin(θ)·[u]× + (1-cos(θ))·u·uᵀ

    @param phi Rotation vector (axis-angle)
    @return 3x3 rotation matrix
    """
    theta = np.linalg.norm(phi)

    if theta < SMALL_ANGLE_THRESHOLD:
        return np.eye(3) + skew(phi)

    u = phi / theta
    u_skew = skew(u)
    c = np.cos(theta)
    s = np.sin(theta)
    return c * np.eye(3) + s * u_skew + (1.0 - c) * np.outer(u, u)


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Hamilton product: q_result = q1 ⊗ q2  (both [w, x, y, z]).

    Applies q2 rotation first, then q1 — matching Eigen convention.
    Returns normalized quaternion.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    q = np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])
    return q / np.linalg.norm(q)


def quaternion_to_rotation(q: np.ndarray) -> np.ndarray:
    """
    Convert unit quaternion [w, x, y, z] to 3x3 rotation matrix (body → earth).
    """
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=float)


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """
    Normalize quaternion and ensure positive scalar (w > 0).
    q and -q represent the same rotation; positive convention avoids ambiguity.
    """
    q_n = q / np.linalg.norm(q)
    if q_n[0] < 0:
        q_n = -q_n
    return q_n


# ============================================================================
# IBVS (Image-Based Visual Servoing) Jacobians
# ============================================================================

def compute_Lv(pbar_x: float, pbar_y: float, p_c_z: float) -> np.ndarray:
    """
    Compute IBVS velocity Jacobian Lv (2x3).
        pbar_dot (velocity) = Lv * v_c

        Lv = [[-1/pz,    0,   px/pz],
              [    0, -1/pz,  py/pz]]
    """
    inv_z = 1.0 / max(p_c_z, MIN_DEPTH)
    return np.array([
        [-inv_z,    0.0,  pbar_x * inv_z],
        [   0.0, -inv_z,  pbar_y * inv_z],
    ], dtype=float)


def compute_Lw(pbar_x: float, pbar_y: float) -> np.ndarray:
    """
    Compute IBVS angular velocity Jacobian Lw (2x3).
        pbar_dot (rotation) = Lw * omega_c

        Lw = [[ px*py,  -(1+px²),  py],
              [(1+py²),  -px*py,  -px]]
    """
    px, py = pbar_x, pbar_y
    return np.array([
        [px * py,       -(1.0 + px * px),  py],
        [(1.0 + py * py), -px * py,        -px],
    ], dtype=float)


def compute_Apbar(pbar_x: float, pbar_y: float,
                  v_c: np.ndarray, omega_c: np.ndarray,
                  p_c_z: float) -> np.ndarray:
    """
    Compute A_pbar Jacobian for δpbar_dot/δpbar (2x2).
    """
    vz = float(v_c[2])
    wx, wy, wz = float(omega_c[0]), float(omega_c[1]), float(omega_c[2])
    z_inv = 1.0 / max(p_c_z, MIN_DEPTH)
    px, py = pbar_x, pbar_y
    return np.array([
        [vz * z_inv + py * wx - 2.0 * px * wy,   px * wx + wz],
        [-py * wy - wz,                            vz * z_inv + 2.0 * py * wx - px * wy],
    ], dtype=float)


def compute_Apcz(pbar_x: float, pbar_y: float,
                 v_c: np.ndarray, p_c_z: float) -> np.ndarray:
    """
    Compute A_pc_z Jacobian for δpbar_dot/δp_c_z (2,).
    """
    vx, vy, vz = float(v_c[0]), float(v_c[1]), float(v_c[2])
    z_sq = max(p_c_z, MIN_DEPTH) ** 2
    return np.array([
        (vx - pbar_x * vz) / z_sq,
        (vy - pbar_y * vz) / z_sq,
    ], dtype=float)


# ============================================================================
# Process Noise Covariance
# ============================================================================

def compute_discrete_process_noise(params: ESKFParams) -> np.ndarray:
    """
    Compute discrete process noise covariance Qd (15x15).

    Block diagonal:
        Θ_i = σ²_ωn · Δt²  (attitude)
        V_i = σ²_an · Δt²  (velocity)
        Ω_i = σ²_ωw · Δt   (gyro bias RW)
        A_i = σ²_aw · Δt   (accel bias RW)
        M_i = σ²_mw · Δt   (mag bias RW)
    """
    dt = params.dt_eskf
    Qd = np.zeros((15, 15))

    theta_i = params.sigma_omega_n ** 2 * dt ** 2
    v_i     = params.sigma_a_n     ** 2 * dt ** 2
    omega_i = params.sigma_omega_w ** 2 * dt
    a_i     = params.sigma_a_w     ** 2 * dt
    m_i     = params.sigma_mag_w   ** 2 * dt

    Qd[OMEGA_N_START:OMEGA_N_START+3, OMEGA_N_START:OMEGA_N_START+3] = theta_i * np.eye(3)
    Qd[A_N_START    :A_N_START    +3, A_N_START    :A_N_START    +3] = v_i     * np.eye(3)
    Qd[OMEGA_W_START:OMEGA_W_START+3, OMEGA_W_START:OMEGA_W_START+3] = omega_i * np.eye(3)
    Qd[A_W_START    :A_W_START    +3, A_W_START    :A_W_START    +3] = a_i     * np.eye(3)
    Qd[MAG_W_START  :MAG_W_START  +3, MAG_W_START  :MAG_W_START  +3] = m_i     * np.eye(3)

    return Qd
