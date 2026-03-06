"""
eskf_core.py
============
Error-State Kalman Filter core class.

Mirrors eskf_core.cpp / eskf_core.hpp.

State dimensions:
    Nominal: 21  [q(4), pr(3), vr(3), pbar(2), bgyr(3), bacc(3), bmag(3)]
    Error:   20  [dθ(3), dpr(3), dvr(3), dpbar(2), dbgyr(3), dbacc(3), dbmag(3)]
"""

from __future__ import annotations
import math
import numpy as np
from collections import deque

from eskf_py.eskf_types import (
    ESKFParams, IMUMeasurement, StateHistoryEntry,
    NOMINAL_SIZE, ERROR_SIZE,
    # nominal indices
    Q_START, PR_START, VR_START, PBAR_START, BGYR_START, BACC_START, BMAG_START,
    # error indices
    DTHETA_START, DPR_START, DVR_START, DPBAR_START,
    DBGYR_START, DBACC_START, DBMAG_START,
    MIN_DEPTH,
    # accessors
    get_quaternion, set_quaternion, get_position, get_velocity,
    get_pbar, get_gyro_bias, get_accel_bias, get_mag_bias,
)
from eskf_py.eskf_math import (
    skew, exp_quaternion, exp_rotation,
    quaternion_multiply, quaternion_to_rotation, normalize_quaternion,
    compute_Lv, compute_Lw,
    compute_discrete_process_noise,
)
from eskf_py.eskf_jacobian import (
    compute_eskf_jacobians, compute_zupt_jacobian, compute_mag_jacobian,
)


# ============================================================================
# Free Functions
# ============================================================================

def compute_image_features(p_r: np.ndarray,
                            q: np.ndarray,
                            R_b2c: np.ndarray) -> np.ndarray:
    """
    Compute normalized image features from relative position.

    p_c = R_b2c * R_e2b * (-p_r)
    pbar = [p_c[0]/p_c[2], p_c[1]/p_c[2]]
    """
    R_b2e = quaternion_to_rotation(q)
    p_c = R_b2c @ R_b2e.T @ (-p_r)
    p_c_z = max(float(p_c[2]), MIN_DEPTH)
    return np.array([p_c[0] / p_c_z, p_c[1] / p_c_z])


def create_initial_state(q_true: np.ndarray,
                          p_r_true: np.ndarray,
                          v_r_true: np.ndarray,
                          pbar_true: np.ndarray,
                          euler_error_deg: np.ndarray | None = None,
                          pos_error: np.ndarray | None = None,
                          vel_error: np.ndarray | None = None,
                          pbar_error: np.ndarray | None = None) -> np.ndarray:
    """Create a nominal state with optional errors applied."""
    euler_error_deg = euler_error_deg if euler_error_deg is not None else np.zeros(3)
    pos_error       = pos_error       if pos_error       is not None else np.zeros(3)
    vel_error       = vel_error       if vel_error       is not None else np.zeros(3)
    pbar_error      = pbar_error      if pbar_error      is not None else np.zeros(2)

    x = np.zeros(NOMINAL_SIZE)

    if np.linalg.norm(euler_error_deg) > 0:
        euler_rad = euler_error_deg * math.pi / 180.0
        # ZYX convention: yaw * pitch * roll
        cr, sr = math.cos(euler_rad[2]/2), math.sin(euler_rad[2]/2)
        cp, sp = math.cos(euler_rad[1]/2), math.sin(euler_rad[1]/2)
        cy, sy = math.cos(euler_rad[0]/2), math.sin(euler_rad[0]/2)
        q_roll  = np.array([cr, sr, 0, 0])
        q_pitch = np.array([cp, 0, sp, 0])
        q_yaw   = np.array([cy, 0, 0, sy])
        dq = quaternion_multiply(q_yaw, quaternion_multiply(q_pitch, q_roll))
        q_est = normalize_quaternion(quaternion_multiply(q_true, dq))
    else:
        q_est = q_true.copy()

    set_quaternion(x, q_est)
    x[PR_START:PR_START+3] = p_r_true + pos_error
    x[VR_START:VR_START+3] = v_r_true + vel_error
    x[PBAR_START:PBAR_START+2] = pbar_true + pbar_error
    # biases start at zero
    return x


# ============================================================================
# ErrorStateKalmanFilter
# ============================================================================

class ErrorStateKalmanFilter:
    """
    Error-State Kalman Filter for visual guidance.

    Mirrors the C++ ErrorStateKalmanFilter class exactly.
    """

    def __init__(self, params: ESKFParams):
        self.params = params

        self._x = np.zeros(NOMINAL_SIZE)
        self._P = np.zeros((ERROR_SIZE, ERROR_SIZE))

        self._current_time = 0.0

        # Derived timing
        self._samples_per_eskf = max(1, round(params.dt_eskf / params.dt_imu))

        # Discrete process noise
        self._Qd = compute_discrete_process_noise(params)

        # Gravity vector in NED
        self._gravity_ned = np.array([0.0, 0.0, params.gravity])

        # Measurement noise covariances
        self._R_img = params.sigma_img ** 2 * np.eye(2)

        self._R_radar = np.zeros((6, 6))
        self._R_radar[:3, :3] = params.sigma_radar_pos ** 2 * np.eye(3)
        self._R_radar[3:, 3:] = params.sigma_radar_vel ** 2 * np.eye(3)

        self._R_mag = params.sigma_mag_n ** 2 * np.eye(3)

        self._R_zupt = np.zeros((6, 6))
        self._R_zupt[:3, :3] = params.sigma_a_n     ** 2 * np.eye(3)
        self._R_zupt[3:, 3:] = params.sigma_omega_n ** 2 * np.eye(3)

        # Fixed measurement matrices
        self._H_img   = self._get_image_measurement_matrix()
        self._H_radar = self._get_radar_measurement_matrix()

        # Initial covariance
        self._P = self._create_initial_covariance()

        # History buffer (deque of StateHistoryEntry)
        self._history: deque[StateHistoryEntry] = deque(
            [StateHistoryEntry() for _ in range(params.history_length)],
            maxlen=params.history_length,
        )

        # IMU accumulation
        self._omega_accum = np.zeros(3)
        self._accel_accum = np.zeros(3)
        self._imu_count   = 0

        # ZUPT state
        self._zupt_enabled       = True
        self._zupt_triggered_once = False
        self._radar_received     = False

        # Pbar observability
        self._first_image_received = False
        self._pbar_frozen          = False

    # ========================================================================
    # Reset
    # ========================================================================

    def reset(self, x_init: np.ndarray, P_init: np.ndarray) -> None:
        self._x = x_init.copy()
        self._P = P_init.copy()
        self._current_time = 0.0
        self._omega_accum  = np.zeros(3)
        self._accel_accum  = np.zeros(3)
        self._imu_count    = 0
        for entry in self._history:
            entry.x[:] = self._x
            entry.P[:] = self._P
            entry.timestamp = 0.0

    # ========================================================================
    # IMU Accumulation
    # ========================================================================

    def accumulate_imu(self, omega: np.ndarray, accel: np.ndarray) -> None:
        self._omega_accum += omega
        self._accel_accum += accel
        self._imu_count   += 1

    def is_ready_for_prediction(self) -> bool:
        return self._imu_count >= self._samples_per_eskf

    def get_averaged_imu(self) -> IMUMeasurement:
        avg = IMUMeasurement()
        if self._imu_count > 0:
            avg.omega = self._omega_accum / self._imu_count
            avg.accel = self._accel_accum / self._imu_count
        self._omega_accum = np.zeros(3)
        self._accel_accum = np.zeros(3)
        self._imu_count   = 0
        return avg

    # ========================================================================
    # Prediction Step
    # ========================================================================

    def predict(self, omega_meas: np.ndarray,
                a_meas: np.ndarray,
                timestamp: float) -> None:
        self._current_time = timestamp

        # 1. Propagate nominal state
        x_new = self._predict_nominal_state(self._x, omega_meas, a_meas, self.params.dt_eskf)

        # 2. Compute Jacobians
        jac = compute_eskf_jacobians(self._x, omega_meas, a_meas,
                                     self.params.dt_eskf, self.params.R_b2c)

        # 3. Propagate error covariance  P_new = Fd*P*Fd' + Gc*Qd*Gc'
        P_new = jac.Fd @ self._P @ jac.Fd.T + jac.Gc @ self._Qd @ jac.Gc.T

        # 4. Freeze pbar if not yet observed or timed out
        if not self._first_image_received or self._pbar_frozen:
            # Zero cross-correlations between pbar and all other states
            ps, pe = DPBAR_START, DPBAR_START + 2
            P_new[ps:pe, :ps]     = 0.0
            P_new[ps:pe, pe:]     = 0.0
            P_new[:ps, ps:pe]     = 0.0
            P_new[pe:,  ps:pe]    = 0.0
            # Restore pbar diagonal
            P_new[ps:pe, ps:pe]   = self._P[ps:pe, ps:pe]
            # Keep pbar state unchanged
            x_new[PBAR_START:PBAR_START+2] = self._x[PBAR_START:PBAR_START+2]

        self._x = x_new
        self._P = P_new

        self._update_history(IMUMeasurement(omega=omega_meas.copy(),
                                            accel=a_meas.copy(),
                                            timestamp=timestamp))

    # ========================================================================
    # Image Correction (with delay)
    # ========================================================================

    def correct_image(self, z_pbar: np.ndarray, delay_steps: int) -> np.ndarray:
        """
        Delayed image measurement correction.

        Looks up state at (current - delay_steps) from history, applies
        KF update there, then re-propagates to current time.
        """
        innovation = np.zeros(2)

        hist_list = list(self._history)
        idx_delayed = len(hist_list) - delay_steps - 1
        if idx_delayed < 0 or idx_delayed >= len(hist_list):
            return innovation

        x_prior = hist_list[idx_delayed].x
        P_prior = hist_list[idx_delayed].P

        z_pred  = get_pbar(x_prior)
        innovation = z_pbar - z_pred

        PH_T = P_prior @ self._H_img.T          # (20, 2)
        S    = self._H_img @ PH_T + self._R_img  # (2, 2)
        S_inv = np.linalg.inv(S)

        # Chi-square gating
        d2 = float(innovation @ S_inv @ innovation)
        if self.params.enable_false_detection_image and d2 > self.params.chi2_threshold_image:
            print(f"[IMAGE]: Rejected (d²={d2:.2f} > {self.params.chi2_threshold_image:.2f})")
            return np.zeros(2)

        K       = PH_T @ S_inv                   # (20, 2)
        delta_x = K @ innovation                  # (20,)

        x_corrected = self._inject_error_state(x_prior, delta_x)

        I_KH = np.eye(ERROR_SIZE) - K @ self._H_img
        P_corrected = I_KH @ P_prior @ I_KH.T + K @ self._R_img @ K.T
        P_corrected = self._reset_covariance(P_corrected, delta_x)

        self._repropagate(x_corrected, P_corrected, idx_delayed)

        if not self._first_image_received:
            self._first_image_received = True
            print("[IMAGE]: First measurement received — pbar propagation enabled")

        return innovation

    # ========================================================================
    # Radar Correction (no delay)
    # ========================================================================

    def correct_radar(self, z_radar: np.ndarray) -> np.ndarray:
        """Radar correction: 6D (pos+vel) or 3D (pos only) depending on use_vr."""
        if self.params.use_vr:
            # === 6D update ===
            z_pred = np.concatenate([get_position(self._x), get_velocity(self._x)])
            innovation = z_radar - z_pred

            PH_T = self._P @ self._H_radar.T        # (20, 6)
            S    = self._H_radar @ PH_T + self._R_radar
            S_inv = np.linalg.inv(S)

            d2 = float(innovation @ S_inv @ innovation)
            if self.params.enable_false_detection_radar and d2 > self.params.chi2_threshold_radar:
                print(f"[RADAR]: Rejected (d²={d2:.2f})")
                return np.zeros(6)

            K       = PH_T @ S_inv
            delta_x = K @ innovation
            self._x = self._inject_error_state(self._x, delta_x)

            I_KH = np.eye(ERROR_SIZE) - K @ self._H_radar
            P_up = I_KH @ self._P @ I_KH.T + K @ self._R_radar @ K.T
            self._P = self._reset_covariance(P_up, delta_x)
            return innovation

        else:
            # === 3D position-only update ===
            H_pos  = self._H_radar[:3, :]            # (3, 20)
            R_pos  = self._R_radar[:3, :3]            # (3, 3)
            z_pred = get_position(self._x)
            innovation_pos = z_radar[:3] - z_pred

            PH_T = self._P @ H_pos.T                  # (20, 3)
            S    = H_pos @ PH_T + R_pos
            S_inv = np.linalg.inv(S)

            chi2_3dof = 16.27  # chi2inv(0.9999, 3)
            d2 = float(innovation_pos @ S_inv @ innovation_pos)
            if self.params.enable_false_detection_radar and d2 > chi2_3dof:
                print(f"[RADAR]: Rejected pos-only (d²={d2:.2f})")
                return np.zeros(6)

            K       = PH_T @ S_inv                    # (20, 3)
            delta_x = K @ innovation_pos
            self._x = self._inject_error_state(self._x, delta_x)

            I_KH = np.eye(ERROR_SIZE) - K @ H_pos
            P_up = I_KH @ self._P @ I_KH.T + K @ R_pos @ K.T
            self._P = self._reset_covariance(P_up, delta_x)

            innovation_full = np.zeros(6)
            innovation_full[:3] = innovation_pos
            return innovation_full

    # ========================================================================
    # Magnetometer Correction
    # ========================================================================

    def correct_mag(self, z_mag: np.ndarray) -> np.ndarray:
        """Magnetometer correction with chi-square gating."""
        H_mag = compute_mag_jacobian(self._x, self.params.B_ned)

        R_b2e = quaternion_to_rotation(get_quaternion(self._x))
        z_pred = R_b2e.T @ self.params.B_ned - get_mag_bias(self._x)
        innovation = z_mag - z_pred

        PH_T = self._P @ H_mag.T               # (20, 3)
        S    = H_mag @ PH_T + self._R_mag
        S_inv = np.linalg.inv(S)

        d2 = float(innovation @ S_inv @ innovation)
        if self.params.enable_false_detection_mag and d2 > self.params.chi2_threshold_mag:
            print(f"[MAG]: Rejected (d²={d2:.2f} > {self.params.chi2_threshold_mag:.2f})")
            return np.zeros(3)

        K       = PH_T @ S_inv
        delta_x = K @ innovation
        self._x = self._inject_error_state(self._x, delta_x)

        I_KH = np.eye(ERROR_SIZE) - K @ H_mag
        P_up = I_KH @ self._P @ I_KH.T + K @ self._R_mag @ K.T
        self._P = self._reset_covariance(P_up, delta_x)

        return innovation

    # ========================================================================
    # ZUPT (Zero Velocity Update)
    # ========================================================================

    def detect_zupt(self, omega_meas: np.ndarray, a_meas: np.ndarray) -> bool:
        """Check if platform is stationary using chi-square test."""
        if not self._zupt_enabled:
            return False

        R_b2e = quaternion_to_rotation(get_quaternion(self._x))
        b_gyr = get_gyro_bias(self._x)
        b_acc = get_accel_bias(self._x)

        z_tilde = np.concatenate([
            -(a_meas - b_acc + R_b2e.T @ self._gravity_ned),
            -(omega_meas - b_gyr),
        ])

        H = compute_zupt_jacobian(self._x, self._gravity_ned)
        S = H @ self._P @ H.T + self.params.zupt_alpha * self._R_zupt
        chi2 = float(z_tilde @ np.linalg.inv(S) @ z_tilde)

        return chi2 < self.params.chi2_threshold_zupt

    def correct_zupt(self, omega_meas: np.ndarray, a_meas: np.ndarray) -> np.ndarray:
        """Apply zero velocity update correction."""
        if not self._zupt_enabled:
            return np.zeros(6)

        R_b2e = quaternion_to_rotation(get_quaternion(self._x))
        b_gyr = get_gyro_bias(self._x)
        b_acc = get_accel_bias(self._x)

        innovation = np.concatenate([
            -(a_meas - b_acc + R_b2e.T @ self._gravity_ned),
            -(omega_meas - b_gyr),
        ])

        H = compute_zupt_jacobian(self._x, self._gravity_ned)
        PH_T = self._P @ H.T               # (20, 6)
        S    = H @ PH_T + self._R_zupt
        K    = PH_T @ np.linalg.inv(S)

        delta_x = K @ innovation
        self._x = self._inject_error_state(self._x, delta_x)

        I_KH = np.eye(ERROR_SIZE) - K @ H
        P_up = I_KH @ self._P @ I_KH.T + K @ self._R_zupt @ K.T
        self._P = self._reset_covariance(P_up, delta_x)

        self._zupt_triggered_once = True
        return innovation

    def disable_zupt(self) -> None:
        self._zupt_enabled = False

    def notify_radar_received(self) -> None:
        self._radar_received = True

    def set_pbar_frozen(self, frozen: bool) -> None:
        self._pbar_frozen = frozen

    # ========================================================================
    # State Access
    # ========================================================================

    def get_state(self)     -> np.ndarray: return self._x.copy()
    def get_covariance(self) -> np.ndarray: return self._P.copy()
    def get_timestamp(self)  -> float:      return self._current_time
    def get_params(self)    -> ESKFParams:  return self.params

    def get_quaternion(self) -> np.ndarray: return get_quaternion(self._x)
    def get_position(self)   -> np.ndarray: return get_position(self._x)
    def get_velocity(self)   -> np.ndarray: return get_velocity(self._x)
    def get_pbar(self)       -> np.ndarray: return get_pbar(self._x)
    def get_gyro_bias(self)  -> np.ndarray: return get_gyro_bias(self._x)
    def get_accel_bias(self) -> np.ndarray: return get_accel_bias(self._x)
    def get_mag_bias(self)   -> np.ndarray: return get_mag_bias(self._x)

    def get_covariance_diagonal(self) -> np.ndarray: return np.diag(self._P)

    def is_zupt_enabled(self)     -> bool: return self._zupt_enabled
    def was_zupt_triggered(self)  -> bool: return self._zupt_triggered_once
    def was_radar_received(self)  -> bool: return self._radar_received
    def was_image_received(self)  -> bool: return self._first_image_received
    def is_pbar_frozen(self)      -> bool: return self._pbar_frozen

    # ========================================================================
    # Private Helpers
    # ========================================================================

    def _predict_nominal_state(self, x: np.ndarray,
                                omega_meas: np.ndarray,
                                a_meas: np.ndarray,
                                dt: float) -> np.ndarray:
        """Propagate nominal state using IMU. Mirrors predictNominalState()."""
        q      = get_quaternion(x)
        p_r    = get_position(x)
        v_r    = get_velocity(x)
        pbar   = get_pbar(x)
        b_gyr  = get_gyro_bias(x)
        b_acc  = get_accel_bias(x)
        b_mag  = get_mag_bias(x)

        omega  = omega_meas - b_gyr
        a_body = a_meas     - b_acc

        R_b2e  = quaternion_to_rotation(q)

        # Quaternion update
        q_new = normalize_quaternion(
            quaternion_multiply(q, exp_quaternion(omega * dt))
        )

        # Velocity update
        a_world  = R_b2e @ a_body + self._gravity_ned
        v_r_new  = v_r + a_world * dt

        # Position update (trapezoidal)
        p_r_new  = p_r + 0.5 * (v_r + v_r_new) * dt

        # Image feature update
        R_e2b  = R_b2e.T
        p_c    = self.params.R_b2c @ R_e2b @ (-p_r)
        p_c_z  = max(float(p_c[2]), MIN_DEPTH)
        v_c    = self.params.R_b2c @ R_e2b @ v_r
        w_c    = self.params.R_b2c @ omega

        Lv   = compute_Lv(float(pbar[0]), float(pbar[1]), p_c_z)
        Lw   = compute_Lw(float(pbar[0]), float(pbar[1]))
        pbar_new = pbar + (Lv @ v_c + Lw @ w_c) * dt

        # Biases unchanged
        x_new = np.zeros(NOMINAL_SIZE)
        set_quaternion(x_new, q_new)
        x_new[PR_START  :PR_START  +3] = p_r_new
        x_new[VR_START  :VR_START  +3] = v_r_new
        x_new[PBAR_START:PBAR_START+2] = pbar_new
        x_new[BGYR_START:BGYR_START+3] = b_gyr
        x_new[BACC_START:BACC_START+3] = b_acc
        x_new[BMAG_START:BMAG_START+3] = b_mag
        return x_new

    def _inject_error_state(self, x_nominal: np.ndarray,
                             delta_x: np.ndarray) -> np.ndarray:
        """Inject error state into nominal state. x_corrected = x ⊕ δx."""
        x_c = x_nominal.copy()

        # Attitude: q_corrected = q ⊗ exp_q(δθ)
        delta_theta = delta_x[DTHETA_START:DTHETA_START+3]
        dq = exp_quaternion(delta_theta)
        q_nom = get_quaternion(x_nominal)
        q_corrected = normalize_quaternion(quaternion_multiply(q_nom, dq))
        set_quaternion(x_c, q_corrected)

        # Additive corrections
        x_c[PR_START  :PR_START  +3] += delta_x[DPR_START  :DPR_START  +3]
        x_c[VR_START  :VR_START  +3] += delta_x[DVR_START  :DVR_START  +3]
        x_c[PBAR_START:PBAR_START+2] += delta_x[DPBAR_START:DPBAR_START+2]
        x_c[BGYR_START:BGYR_START+3] += delta_x[DBGYR_START:DBGYR_START+3]
        x_c[BACC_START:BACC_START+3] += delta_x[DBACC_START:DBACC_START+3]
        x_c[BMAG_START:BMAG_START+3] += delta_x[DBMAG_START:DBMAG_START+3]
        return x_c

    def _reset_covariance(self, P: np.ndarray,
                           delta_x: np.ndarray) -> np.ndarray:
        """Reset covariance after error injection: P_reset = G*P*G'."""
        delta_theta = delta_x[DTHETA_START:DTHETA_START+3]
        G = np.eye(ERROR_SIZE)
        G[:3, :3] = np.eye(3) - 0.5 * skew(delta_theta)
        return G @ P @ G.T

    def _repropagate(self, x_start: np.ndarray,
                     P_start: np.ndarray,
                     idx_start: int) -> None:
        """Re-propagate from corrected delayed state to current time."""
        x_rep = x_start.copy()
        P_rep = P_start.copy()

        hist_list = list(self._history)
        for j in range(idx_start, len(hist_list) - 1):
            omega_meas = hist_list[j].imu.omega
            a_meas     = hist_list[j].imu.accel

            x_rep = self._predict_nominal_state(x_rep, omega_meas, a_meas,
                                                self.params.dt_eskf)
            jac   = compute_eskf_jacobians(x_rep, omega_meas, a_meas,
                                           self.params.dt_eskf, self.params.R_b2c)
            P_rep = jac.Fd @ P_rep @ jac.Fd.T + jac.Gc @ self._Qd @ jac.Gc.T

        self._x = x_rep
        self._P = P_rep

    def _update_history(self, imu: IMUMeasurement) -> None:
        """Add current state to history buffer (deque auto-drops oldest)."""
        entry = StateHistoryEntry(
            x=self._x.copy(),
            P=self._P.copy(),
            imu=imu,
            timestamp=self._current_time,
        )
        self._history.append(entry)

    def _get_image_measurement_matrix(self) -> np.ndarray:
        """H_img: identity on pbar block. (2x20)"""
        H = np.zeros((2, ERROR_SIZE))
        H[:2, DPBAR_START:DPBAR_START+2] = np.eye(2)
        return H

    def _get_radar_measurement_matrix(self) -> np.ndarray:
        """H_radar: identity on [pr; vr] blocks. (6x20)"""
        H = np.zeros((6, ERROR_SIZE))
        H[:3, DPR_START:DPR_START+3] = np.eye(3)
        H[3:, DVR_START:DVR_START+3] = np.eye(3)
        return H

    def _create_initial_covariance(self) -> np.ndarray:
        """Build initial P from ESKFParams sigmas."""
        p = self.params
        P = np.zeros((ERROR_SIZE, ERROR_SIZE))
        P[DTHETA_START:DTHETA_START+3, DTHETA_START:DTHETA_START+3] = p.init_sigma_attitude ** 2 * np.eye(3)
        P[DPR_START   :DPR_START   +3, DPR_START   :DPR_START   +3] = p.init_sigma_position ** 2 * np.eye(3)
        P[DVR_START   :DVR_START   +3, DVR_START   :DVR_START   +3] = p.init_sigma_velocity ** 2 * np.eye(3)
        P[DPBAR_START :DPBAR_START +2, DPBAR_START :DPBAR_START +2] = p.init_sigma_pbar      ** 2 * np.eye(2)
        P[DBGYR_START :DBGYR_START +3, DBGYR_START :DBGYR_START +3] = p.init_sigma_bgyr      ** 2 * np.eye(3)
        P[DBACC_START :DBACC_START +3, DBACC_START :DBACC_START +3] = p.init_sigma_bacc      ** 2 * np.eye(3)
        P[DBMAG_START :DBMAG_START +3, DBMAG_START :DBMAG_START +3] = p.init_sigma_bmag      ** 2 * np.eye(3)
        return P

    def _clamp_pbar_covariance(self,
                                max_var: float = 10.0,
                                min_var: float = 1e-6) -> None:
        """Clamp pbar covariance diagonal to prevent divergence."""
        for i in range(DPBAR_START, DPBAR_START + 2):
            if not math.isfinite(self._P[i, i]):
                self._P[i, i] = min_var
            elif self._P[i, i] < 0:
                self._P[i, i] = abs(self._P[i, i])
            elif self._P[i, i] > max_var:
                self._P[i, i] = max_var
                for j in range(ERROR_SIZE):
                    if j != i:
                        self._P[i, j] = 0.0
                        self._P[j, i] = 0.0
