"""
eskf_test_harness.py
====================
Standalone simulation test for the ESKF — no ROS2 required.

Mirrors eskf_test_harness.cpp exactly.

Usage:
    python eskf_test_harness.py [config_file]
    python eskf_test_harness.py ../eskf_cpp/config/eskf_params.yaml
"""

import sys
import time
import math
import csv
import numpy as np

from eskf_py.eskf_core import ErrorStateKalmanFilter, compute_image_features, create_initial_state
from eskf_py.eskf_config import load_config, print_config
from eskf_py.eskf_types import ESKFParams, IMUMeasurement
from eskf_py.eskf_math import (
    quaternion_to_rotation, quaternion_multiply, normalize_quaternion, exp_quaternion,
)


# ============================================================================
# IMU Simulator
# ============================================================================

class IMUSimulator:
    """Simulates noisy IMU with evolving biases. Matches C++ IMUSimulator."""

    def __init__(self, params: ESKFParams, seed: int = 35):
        self._params = params
        self._rng    = np.random.default_rng(seed)
        self._omega_b = np.array([0.005, -0.003,  0.002])   # initial gyro bias
        self._a_b     = np.array([0.02,  -0.01,   0.015])   # initial accel bias

    def measure(self, omega_true: np.ndarray, a_true: np.ndarray) -> IMUMeasurement:
        dt = self._params.dt_imu
        s_ow = self._params.sigma_omega_w * math.sqrt(dt)
        s_aw = self._params.sigma_a_w     * math.sqrt(dt)

        self._omega_b += s_ow * self._rng.standard_normal(3)
        self._a_b     += s_aw * self._rng.standard_normal(3)

        n_omega = self._params.sigma_omega_n * self._rng.standard_normal(3)
        n_a     = self._params.sigma_a_n     * self._rng.standard_normal(3)

        return IMUMeasurement(
            omega=omega_true + self._omega_b + n_omega,
            accel=a_true    + self._a_b     + n_a,
        )

    def get_gyro_bias(self)  -> np.ndarray: return self._omega_b.copy()
    def get_accel_bias(self) -> np.ndarray: return self._a_b.copy()

    def reset(self):
        self._omega_b = np.array([0.005, -0.003,  0.002])
        self._a_b     = np.array([0.02,  -0.01,   0.015])


# ============================================================================
# True State
# ============================================================================

class TrueState:
    def __init__(self):
        self.q           = np.array([1.0, 0.0, 0.0, 0.0])  # identity
        self.p_int       = np.array([0.0, 0.0, -65.0])
        self.v_int       = np.zeros(3)
        self.p_tgt       = np.array([50.0, 10.0, -40.0])
        self.v_tgt       = np.zeros(3)
        self.omega_true  = np.zeros(3)
        self.a_body_true = np.zeros(3)


def propagate_true_state(state: TrueState, t: float, dt: float,
                          R_b2c: np.ndarray) -> None:
    """Propagate ground-truth dynamics. Mirrors C++ propagateTrueState()."""
    state.omega_true = np.array([1.0, 2.0, 3.0]) * (0.01 * math.sin(0.5 * t))

    R_b2e = quaternion_to_rotation(state.q)
    R_e2b = R_b2e.T
    gravity_vec = np.array([0.0, 0.0, 9.81])
    maneuver = np.array([6.0, 0.4, -0.5]) * (0.1 * math.sin(0.5 * t))
    state.a_body_true = R_e2b @ (-gravity_vec) + maneuver

    # Attitude
    state.q = normalize_quaternion(
        quaternion_multiply(state.q, exp_quaternion(state.omega_true * dt))
    )

    # Velocity / position
    a_world = R_b2e @ state.a_body_true + gravity_vec
    v_int_new = state.v_int + a_world * dt
    state.p_int = state.p_int + 0.5 * (state.v_int + v_int_new) * dt
    state.v_int = v_int_new

    # Target drifts slowly
    state.p_tgt += state.v_tgt * dt


# ============================================================================
# Error Metrics
# ============================================================================

class ErrorMetrics:
    def __init__(self):
        self.pos_error:     list[float] = []
        self.vel_error:     list[float] = []
        self.att_error_deg: list[float] = []
        self.bgyr_error:    list[float] = []
        self.bacc_error:    list[float] = []

    @staticmethod
    def rmse(errors: list[float]) -> float:
        arr = [e for e in errors if math.isfinite(e)]
        if not arr:
            return 0.0
        return math.sqrt(sum(e*e for e in arr) / len(arr))

    def print_summary(self):
        print("\n=== ESKF Performance Statistics ===")
        print(f"Position RMSE:   {self.rmse(self.pos_error):.4f} m")
        print(f"Velocity RMSE:   {self.rmse(self.vel_error):.4f} m/s")
        print(f"Attitude RMSE:   {self.rmse(self.att_error_deg):.4f} deg")
        print(f"Gyro Bias RMSE:  {self.rmse(self.bgyr_error):.6f} rad/s")
        print(f"Accel Bias RMSE: {self.rmse(self.bacc_error):.6f} m/s²")
        print("===================================")


# ============================================================================
# Main
# ============================================================================

def main():
    print("=== ESKF Python Test Harness ===\n")

    # Load config
    params = ESKFParams()
    if len(sys.argv) > 1:
        print(f"Loading config from: {sys.argv[1]}")
        try:
            params = load_config(sys.argv[1])
        except Exception as e:
            print(f"Error loading config: {e}. Using defaults.")
    print_config(params)

    # Simulation parameters
    t_total          = 25.0
    dt_imu           = params.dt_imu
    dt_eskf          = params.dt_eskf
    dt_image         = 1.0 / 30.0
    dt_radar         = 1.0 / 0.5
    t_delay          = params.image_delay
    D                = round(t_delay / dt_eskf)

    N_imu            = int(t_total / dt_imu) + 1
    eskf_sample_idx  = round(dt_eskf  / dt_imu)
    image_sample_idx = round(dt_image / dt_imu)
    radar_sample_idx = round(dt_radar / dt_imu)

    print(f"Simulation: {t_total}s  IMU: {1/dt_imu:.0f} Hz  ESKF: {1/dt_eskf:.0f} Hz")
    print(f"Image delay: {t_delay*1000:.0f} ms ({D} ESKF cycles)\n")

    # Initial true state
    truth = TrueState()
    p_r_true  = truth.p_int - truth.p_tgt
    v_r_true  = truth.v_int - truth.v_tgt
    pbar_true = compute_image_features(p_r_true, truth.q, params.R_b2c)

    # Create ESKF
    eskf = ErrorStateKalmanFilter(params)
    x_init = create_initial_state(
        truth.q, p_r_true, v_r_true, pbar_true,
        euler_error_deg=np.array([1.0, 1.0, 1.0]),
        pos_error=np.array([0.5, 0.5, 0.5]),
        vel_error=np.array([0.1, -0.05, 0.0]),
        pbar_error=np.array([0.01, -0.02]),
    )
    eskf.reset(x_init, eskf.get_covariance())

    imu_sim = IMUSimulator(params, seed=35)
    rng     = np.random.default_rng(42)
    metrics = ErrorMetrics()

    # CSV output
    csv_path = "py_results.csv"
    csv_file = open(csv_path, 'w', newline='')
    writer   = csv.writer(csv_file)
    writer.writerow([
        'time',
        'px_est','py_est','pz_est','vx_est','vy_est','vz_est',
        'qw_est','qx_est','qy_est','qz_est',
        'pbarx_est','pbary_est',
        'bgx_est','bgy_est','bgz_est','bax_est','bay_est','baz_est',
        'bmx_est','bmy_est','bmz_est',
        'px_true','py_true','pz_true','vx_true','vy_true','vz_true',
        'qw_true','qx_true','qy_true','qz_true',
        'pbarx_true','pbary_true',
        'bgx_true','bgy_true','bgz_true','bax_true','bay_true','baz_true',
        'bmx_true','bmy_true','bmz_true',
    ] + [f'P_{i}' for i in range(20)])

    print("Starting simulation...")
    t_start = time.perf_counter()
    eskf_update_counter  = 0
    image_update_counter = 0
    radar_update_counter = 0
    nan_detected = False

    for k in range(N_imu):
        t = k * dt_imu

        # Ground truth propagation
        propagate_true_state(truth, t, dt_imu, params.R_b2c)
        p_r_true  = truth.p_int - truth.p_tgt
        v_r_true  = truth.v_int - truth.v_tgt
        pbar_true = compute_image_features(p_r_true, truth.q, params.R_b2c)

        # IMU measurement
        imu_meas = imu_sim.measure(truth.omega_true, truth.a_body_true)

        # Accumulate IMU
        eskf.accumulate_imu(imu_meas.omega, imu_meas.accel)
        eskf_update_counter += 1

        # Prediction
        if eskf_update_counter >= eskf_sample_idx:
            eskf_update_counter = 0
            avg = eskf.get_averaged_imu()
            eskf.predict(avg.omega, avg.accel, t)

        # Image correction (with delay)
        image_update_counter += 1
        if image_update_counter >= image_sample_idx and k > D:
            image_update_counter = 0
            R_b2e = quaternion_to_rotation(truth.q)
            p_cam = params.R_b2c @ R_b2e.T @ (-p_r_true)
            if p_cam[2] > 2.0:
                noise = params.sigma_img * rng.standard_normal(2)
                z_pbar = np.array([p_cam[0]/p_cam[2], p_cam[1]/p_cam[2]]) + noise
                eskf.correct_image(z_pbar, D)

        # Radar correction (0.5 Hz)
        radar_update_counter += 1
        if radar_update_counter >= radar_sample_idx:
            radar_update_counter = 0
            z_radar = np.concatenate([
                p_r_true + params.sigma_radar_pos * rng.standard_normal(3),
                v_r_true + params.sigma_radar_vel * rng.standard_normal(3),
            ])
            eskf.correct_radar(z_radar)

        # Errors
        pos_est  = eskf.get_position()
        vel_est  = eskf.get_velocity()
        q_est    = eskf.get_quaternion()
        bgyr_est = eskf.get_gyro_bias()
        bacc_est = eskf.get_accel_bias()

        pos_err = float(np.linalg.norm(pos_est - p_r_true))
        vel_err = float(np.linalg.norm(vel_est - v_r_true))

        # Attitude error from quaternion
        q_err = quaternion_multiply(
            np.array([truth.q[0], -truth.q[1], -truth.q[2], -truth.q[3]]),  # conjugate
            q_est)
        att_err_deg = 2.0 * math.degrees(math.acos(min(1.0, abs(float(q_err[0])))))

        bgyr_err = float(np.linalg.norm(bgyr_est - imu_sim.get_gyro_bias()))
        bacc_err = float(np.linalg.norm(bacc_est - imu_sim.get_accel_bias()))

        if not nan_detected and (not math.isfinite(pos_err) or not math.isfinite(vel_err)):
            nan_detected = True
            print(f"\n!!! NaN detected at k={k}, t={t:.3f}")

        metrics.pos_error.append(pos_err)
        metrics.vel_error.append(vel_err)
        metrics.att_error_deg.append(att_err_deg)
        metrics.bgyr_error.append(bgyr_err)
        metrics.bacc_error.append(bacc_err)

        # CSV (every 10th sample)
        if k % 10 == 0:
            pbar_est  = eskf.get_pbar()
            bmag_est  = eskf.get_mag_bias()
            bg_true   = imu_sim.get_gyro_bias()
            ba_true   = imu_sim.get_accel_bias()
            bm_true   = np.zeros(3)
            P_diag    = eskf.get_covariance_diagonal()

            writer.writerow(
                [f'{t:.6f}'] +
                [f'{v:.6f}' for v in pos_est] +
                [f'{v:.6f}' for v in vel_est] +
                [f'{v:.6f}' for v in q_est] +
                [f'{v:.6f}' for v in pbar_est] +
                [f'{v:.6f}' for v in bgyr_est] +
                [f'{v:.6f}' for v in bacc_est] +
                [f'{v:.6f}' for v in bmag_est] +
                [f'{v:.6f}' for v in p_r_true] +
                [f'{v:.6f}' for v in v_r_true] +
                [f'{v:.6f}' for v in truth.q] +
                [f'{v:.6f}' for v in pbar_true] +
                [f'{v:.6f}' for v in bg_true] +
                [f'{v:.6f}' for v in ba_true] +
                [f'{v:.6f}' for v in bm_true] +
                [f'{v:.6f}' for v in P_diag]
            )

        # Progress
        if k % (N_imu // 10) == 0:
            print(f"Progress: {100 * k // N_imu}%")

    elapsed_ms = (time.perf_counter() - t_start) * 1000
    csv_file.close()

    print(f"\nSimulation Complete!")
    print(f"Execution time:  {elapsed_ms:.0f} ms")
    print(f"Processing rate: {N_imu * 1000 / elapsed_ms:.0f} Hz")
    metrics.print_summary()
    print(f"\nResults saved to: {csv_path}")


if __name__ == '__main__':
    main()
