"""
eskf_jacobian.py
================
ESKF Jacobian computation for error-state propagation.

Mirrors eskf_jacobian.cpp / eskf_jacobian.hpp.

Computes:
    Fc  (20x20): continuous-time error-state Jacobian
    Gc  (20x15): continuous-time noise Jacobian
    Fd  (20x20): discrete-time state transition
    Gd  (20x15): discrete-time noise Jacobian

Error state:  δx = [δθ(3), δpr(3), δvr(3), δpbar(2), δbgyr(3), δbacc(3), δbmag(3)]
Noise vector: n  = [n_ω(3), n_a(3), ω_w(3), a_w(3),  m_w(3)]
"""

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

from eskf_py.eskf_types import (
    # nominal indices
    PR_START, VR_START, PBAR_START, BGYR_START, BACC_START,
    # error-state indices
    DTHETA_START, DPR_START, DVR_START, DPBAR_START,
    DBGYR_START, DBACC_START, DBMAG_START,
    ERROR_SIZE, NOISE_SIZE,
    # noise indices
    OMEGA_N_START, A_N_START, OMEGA_W_START, A_W_START, MAG_W_START,
    # accessors
    get_quaternion, get_position, get_velocity, get_pbar,
    get_gyro_bias, get_accel_bias,
    MIN_DEPTH,
)
from eskf_py.eskf_math import (
    skew, quaternion_to_rotation,
    compute_Lv, compute_Lw, compute_Apbar, compute_Apcz,
    exp_rotation,
)


@dataclass
class ESKFJacobians:
    """All four ESKF Jacobians."""
    Fc: np.ndarray = field(default_factory=lambda: np.zeros((ERROR_SIZE, ERROR_SIZE)))
    Gc: np.ndarray = field(default_factory=lambda: np.zeros((ERROR_SIZE, NOISE_SIZE)))
    Fd: np.ndarray = field(default_factory=lambda: np.eye(ERROR_SIZE))
    Gd: np.ndarray = field(default_factory=lambda: np.zeros((ERROR_SIZE, NOISE_SIZE)))


def compute_eskf_jacobians(x_nominal: np.ndarray,
                            omega_meas: np.ndarray,
                            a_meas: np.ndarray,
                            dt: float,
                            R_b2c: np.ndarray) -> ESKFJacobians:
    """
    Compute Fc, Gc, Fd, Gd for the current nominal state.

    Direct translation of computeESKFJacobians() in eskf_jacobian.cpp.
    """
    jac = ESKFJacobians()

    # Extract nominal states
    q       = get_quaternion(x_nominal)
    p_r     = get_position(x_nominal)
    v_r     = get_velocity(x_nominal)
    pbar    = get_pbar(x_nominal)
    b_gyr   = get_gyro_bias(x_nominal)
    b_acc   = get_accel_bias(x_nominal)

    pbar_x, pbar_y = float(pbar[0]), float(pbar[1])

    # Rotation matrices
    R_b2e = quaternion_to_rotation(q)          # body → earth
    R_e2b = R_b2e.T                            # earth → body

    # Bias-corrected IMU
    omega  = omega_meas - b_gyr    # corrected angular velocity
    a_body = a_meas    - b_acc     # corrected specific force

    # Camera-frame quantities
    p_c   = R_b2c @ R_e2b @ (-p_r)
    p_c_z = max(float(p_c[2]), MIN_DEPTH)
    v_c   = R_b2c @ R_e2b @ v_r
    omega_c = R_b2c @ omega

    # IBVS sub-Jacobians
    A_pbar  = compute_Apbar(pbar_x, pbar_y, v_c, omega_c, p_c_z)
    Lv      = compute_Lv(pbar_x, pbar_y, p_c_z)
    Lw      = compute_Lw(pbar_x, pbar_y)
    A_pc_z  = compute_Apcz(pbar_x, pbar_y, v_c, p_c_z)

    # ========================================================================
    # Build Fc (20x20)
    # ========================================================================
    Fc = np.zeros((ERROR_SIZE, ERROR_SIZE))

    # δθ_dot = -skew(ω)·δθ - δbgyr
    Fc[DTHETA_START:DTHETA_START+3, DTHETA_START:DTHETA_START+3] = -skew(omega)
    Fc[DTHETA_START:DTHETA_START+3, DBGYR_START :DBGYR_START +3] = -np.eye(3)

    # δpr_dot = δvr
    Fc[DPR_START:DPR_START+3, DVR_START:DVR_START+3] = np.eye(3)

    # δvr_dot = -R·skew(a_body)·δθ - R·δbacc
    Fc[DVR_START:DVR_START+3, DTHETA_START:DTHETA_START+3] = -R_b2e @ skew(a_body)
    Fc[DVR_START:DVR_START+3, DBACC_START :DBACC_START +3] = -R_b2e

    # δpbar_dot sub-block
    e3 = np.array([0.0, 0.0, 1.0])

    dvc_ddtheta  =  R_b2c @ skew(R_e2b @ v_r)         # (3x3)
    dvc_ddvr     =  R_b2c @ R_e2b                       # (3x3)
    dwc_ddbgyr   = -R_b2c                               # (3x3)

    dpcz_ddtheta = -(e3 @ R_b2c @ skew(R_e2b @ p_r))  # (3,)  row vector
    dpcz_ddpr    = -(e3 @ R_b2c @ R_e2b)               # (3,)  row vector

    # ∂δpbar/∂δθ   (2x3)
    Fc[DPBAR_START:DPBAR_START+2, DTHETA_START:DTHETA_START+3] = (
        Lv @ dvc_ddtheta + np.outer(A_pc_z, dpcz_ddtheta)
    )
    # ∂δpbar/∂δpr  (2x3)
    Fc[DPBAR_START:DPBAR_START+2, DPR_START:DPR_START+3] = (
        np.outer(A_pc_z, dpcz_ddpr)
    )
    # ∂δpbar/∂δvr  (2x3)
    Fc[DPBAR_START:DPBAR_START+2, DVR_START:DVR_START+3] = Lv @ dvc_ddvr

    # ∂δpbar/∂δpbar (2x2)
    Fc[DPBAR_START:DPBAR_START+2, DPBAR_START:DPBAR_START+2] = A_pbar

    # ∂δpbar/∂δbgyr (2x3)
    Fc[DPBAR_START:DPBAR_START+2, DBGYR_START:DBGYR_START+3] = Lw @ dwc_ddbgyr

    # δbgyr_dot, δbacc_dot, δbmag_dot = 0 (handled by noise)

    # ========================================================================
    # Build Gc (20x15)
    # ========================================================================
    Gc = np.zeros((ERROR_SIZE, NOISE_SIZE))

    Gc[DTHETA_START:DTHETA_START+3, OMEGA_N_START:OMEGA_N_START+3] = -np.eye(3)
    Gc[DVR_START   :DVR_START   +3, A_N_START    :A_N_START    +3] = -R_b2e
    Gc[DPBAR_START :DPBAR_START +2, OMEGA_N_START:OMEGA_N_START+3] =  Lw @ (-R_b2c)
    Gc[DBGYR_START :DBGYR_START +3, OMEGA_W_START:OMEGA_W_START+3] =  np.eye(3)
    Gc[DBACC_START :DBACC_START +3, A_W_START    :A_W_START    +3] =  np.eye(3)
    Gc[DBMAG_START :DBMAG_START +3, MAG_W_START  :MAG_W_START  +3] =  np.eye(3)

    # ========================================================================
    # Discretization
    # ========================================================================
    # Fd ≈ I + Fc*dt  (first-order), with exact exponential for attitude block
    Fd = np.eye(ERROR_SIZE) + Fc * dt

    # Exact attitude discretization: exp(-skew(ω)·dt)
    omega_dt = omega * dt
    Phi_theta = exp_rotation(-omega_dt)
    Fd[DTHETA_START:DTHETA_START+3, DTHETA_START:DTHETA_START+3] = Phi_theta

    # First-order attitude-to-bias coupling
    Fd[DTHETA_START:DTHETA_START+3, DBGYR_START:DBGYR_START+3] = -np.eye(3) * dt

    Gd = Gc * dt

    jac.Fc = Fc
    jac.Gc = Gc
    jac.Fd = Fd
    jac.Gd = Gd
    return jac


def compute_zupt_jacobian(x_nominal: np.ndarray,
                           gravity_ned: np.ndarray) -> np.ndarray:
    """
    Compute 6x20 ZUPT measurement Jacobian.

    Row 0-2 (accel): H1 = [skew(R'g)  0  0  0  0  -I  0]
    Row 3-5 (gyro):  H2 = [0          0  0  0  -I  0  0]
    """
    q     = get_quaternion(x_nominal)
    R_b2e = quaternion_to_rotation(q)
    g_body = R_b2e.T @ gravity_ned

    H = np.zeros((6, ERROR_SIZE))
    H[0:3, DTHETA_START:DTHETA_START+3] = skew(g_body)
    H[0:3, DBACC_START :DBACC_START +3] = -np.eye(3)
    H[3:6, DBGYR_START :DBGYR_START +3] = -np.eye(3)
    return H


def compute_mag_jacobian(x_nominal: np.ndarray,
                          B_ned: np.ndarray) -> np.ndarray:
    """
    Compute 3x20 magnetometer measurement Jacobian.

    dh/dδθ   = skew(R_e2b * B_ned)
    dh/dδbmag = -I₃
    """
    q     = get_quaternion(x_nominal)
    R_b2e = quaternion_to_rotation(q)
    B_body = R_b2e.T @ B_ned

    H = np.zeros((3, ERROR_SIZE))
    H[0:3, DTHETA_START:DTHETA_START+3] = skew(B_body)
    H[0:3, DBMAG_START :DBMAG_START +3] = -np.eye(3)
    return H
