"""
eskf_ros2_test.py
=================
ROS2 integration test node — publishes synthetic sensor topics to drive eskf_node.py.

Run in two terminals:
    Terminal 1:  ros2 run eskf_py eskf_node --ros-args -p config_file:=config/eskf_params.yaml
    Terminal 2:  ros2 run eskf_py eskf_ros2_test [config_file]

All dummy topics use the same message types and topic names as the real system
so this node is a drop-in replacement for the real hardware.

Radar topic (/radar/pr as geometry_msgs/Vector3) matches the exact format
published by relative_position_node.py.
"""

import sys
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

from eskf_py.eskf_config import load_config, print_config
from eskf_py.eskf_types  import ESKFParams
from eskf_py.eskf_math   import (
    quaternion_to_rotation, quaternion_multiply,
    normalize_quaternion, exp_quaternion,
)
from eskf_py.eskf_core   import compute_image_features


# ============================================================================
# True State Propagator (same as test_harness)
# ============================================================================

class TrueState:
    def __init__(self):
        self.q           = np.array([1.0, 0.0, 0.0, 0.0])
        self.p_int       = np.array([0.0, 0.0, -65.0])
        self.v_int       = np.zeros(3)
        self.p_tgt       = np.array([50.0, 10.0, -40.0])
        self.v_tgt       = np.zeros(3)
        self.omega_true  = np.zeros(3)
        self.a_body_true = np.zeros(3)


def propagate_true_state(state: TrueState, t: float, dt: float) -> None:
    state.omega_true = np.array([1.0, 2.0, 3.0]) * (0.01 * math.sin(0.5 * t))
    R_b2e = quaternion_to_rotation(state.q)
    R_e2b = R_b2e.T
    gravity  = np.array([0.0, 0.0, 9.81])
    maneuver = np.array([6.0, 0.4, -0.5]) * (0.1 * math.sin(0.5 * t))
    state.a_body_true = R_e2b @ (-gravity) + maneuver
    state.q = normalize_quaternion(
        quaternion_multiply(state.q, exp_quaternion(state.omega_true * dt)))
    a_world   = R_b2e @ state.a_body_true + gravity
    v_int_new = state.v_int + a_world * dt
    state.p_int = state.p_int + 0.5 * (state.v_int + v_int_new) * dt
    state.v_int = v_int_new
    state.p_tgt += state.v_tgt * dt


# ============================================================================
# IMU Simulator
# ============================================================================

class IMUSimulator:
    def __init__(self, params: ESKFParams, seed: int = 35):
        self._params  = params
        self._rng     = np.random.default_rng(seed)
        self._omega_b = np.array([0.005, -0.003, 0.002])
        self._a_b     = np.array([0.02,  -0.01,  0.015])

    def measure(self, omega_true, a_true):
        dt = self._params.dt_imu
        self._omega_b += self._params.sigma_omega_w * math.sqrt(dt) * self._rng.standard_normal(3)
        self._a_b     += self._params.sigma_a_w     * math.sqrt(dt) * self._rng.standard_normal(3)
        n_w = self._params.sigma_omega_n * self._rng.standard_normal(3)
        n_a = self._params.sigma_a_n     * self._rng.standard_normal(3)
        return (omega_true + self._omega_b + n_w,
                a_true    + self._a_b     + n_a)

    def get_gyro_bias(self):  return self._omega_b.copy()
    def get_accel_bias(self): return self._a_b.copy()


# ============================================================================
# RMSE Tracker
# ============================================================================

class RMSETracker:
    def __init__(self):
        self._pos_sq_sum  = 0.0
        self._vel_sq_sum  = 0.0
        self._att_sq_sum  = 0.0
        self._count       = 0

    def update(self, pos_err: float, vel_err: float, att_err_deg: float):
        self._pos_sq_sum += pos_err ** 2
        self._vel_sq_sum += vel_err ** 2
        self._att_sq_sum += att_err_deg ** 2
        self._count      += 1

    def print_summary(self):
        n = max(1, self._count)
        print(f"\n=== Integration Test RMSE ({self._count} samples) ===")
        print(f"  Position RMSE: {math.sqrt(self._pos_sq_sum/n):.4f} m")
        print(f"  Velocity RMSE: {math.sqrt(self._vel_sq_sum/n):.4f} m/s")
        print(f"  Attitude RMSE: {math.sqrt(self._att_sq_sum/n):.4f} deg")
        print("================================================")


# ============================================================================
# Integration test node
# ============================================================================

class ESKFRos2TestNode(Node):
    """Publishes all dummy sensor topics and verifies eskf_node.py output."""

    def __init__(self, params: ESKFParams):
        super().__init__('eskf_ros2_test')
        self._params  = params
        self._rng     = np.random.default_rng(42)
        self._truth   = TrueState()
        self._imu_sim = IMUSimulator(params, seed=35)
        self._rmse    = RMSETracker()

        # Simulation time
        self._t            = 0.0
        self._sim_duration = 60.0   # seconds of synthetic data
        self._armed_until  = 30.0   # disarm after this many sim-seconds

        # Publishers
        self._imu_pub   = self.create_publisher(Imu,           params.topic_imu,   10)
        self._mag_pub   = self.create_publisher(MagneticField, params.topic_mag,    10)
        self._image_pub = self.create_publisher(Point,         params.topic_image,  10)
        self._radar_pub = self.create_publisher(Vector3,       params.topic_radar,  10)
        self._state_pub = self.create_publisher(State,         '/mavros/state',     10)

        # Subscribers (verify output)
        self._odom_sub = self.create_subscription(
            Odometry, params.topic_odom, self._odom_callback, 10)
        self._pbar_sub = self.create_subscription(
            Point, params.topic_pbar, self._pbar_callback, 10)

        # Timers matching sensor rates
        dt_imu   = params.dt_imu
        dt_image = 1.0 / 30.0
        dt_radar = 1.0 / 2.0       # 2 Hz (faster than real 0.5 Hz for quicker test)
        dt_mag   = 1.0 / 50.0
        dt_state = 1.0

        self._imu_timer   = self.create_timer(dt_imu,   self._imu_timer_cb)
        self._image_timer = self.create_timer(dt_image, self._image_timer_cb)
        self._radar_timer = self.create_timer(dt_radar, self._radar_timer_cb)
        self._mag_timer   = self.create_timer(dt_mag,   self._mag_timer_cb)
        self._state_timer = self.create_timer(dt_state, self._state_timer_cb)

        # Image delay queue: list of (send_at_t, z_pbar)
        self._image_queue: list = []
        self._image_delay = params.image_delay

        # Latest estimated state from eskf_node
        self._latest_pos  = None
        self._latest_vel  = None
        self._latest_q    = None

        self.get_logger().info(
            f"[TEST]: Publishing dummy sensors — {self._sim_duration}s simulation")
        self.get_logger().info(
            f"[TEST]: Radar on {params.topic_radar} (same format as relative_position_node.py)")

    # =========================================================================
    # IMU Timer  (200 Hz)
    # =========================================================================

    def _imu_timer_cb(self):
        if self._t > self._sim_duration:
            self._rmse.print_summary()
            self.get_logger().info("[TEST]: Simulation complete. Shutting down.")
            rclpy.shutdown()
            return

        dt = self._params.dt_imu
        propagate_true_state(self._truth, self._t, dt)

        omega_m, accel_m = self._imu_sim.measure(
            self._truth.omega_true, self._truth.a_body_true)

        # Publish FLU (MAVROS convention): negate Y and Z for FRD→FLU
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angular_velocity.x  =  float(omega_m[0])
        msg.angular_velocity.y  = -float(omega_m[1])
        msg.angular_velocity.z  = -float(omega_m[2])
        msg.linear_acceleration.x =  float(accel_m[0])
        msg.linear_acceleration.y = -float(accel_m[1])
        msg.linear_acceleration.z = -float(accel_m[2])
        self._imu_pub.publish(msg)

        # Evaluate RMSE if we have odometry
        if self._latest_pos is not None:
            p_r_true = self._truth.p_int - self._truth.p_tgt
            v_r_true = self._truth.v_int - self._truth.v_tgt
            pos_err  = float(np.linalg.norm(self._latest_pos - p_r_true))
            vel_err  = float(np.linalg.norm(self._latest_vel - v_r_true))
            # attitude error
            q_est = self._latest_q
            q_true = self._truth.q
            q_conj = np.array([q_true[0], -q_true[1], -q_true[2], -q_true[3]])
            q_err  = quaternion_multiply(q_conj, q_est)
            att_deg = 2.0 * math.degrees(math.acos(min(1.0, abs(float(q_err[0])))))
            self._rmse.update(pos_err, vel_err, att_deg)

        self._t += dt

    # =========================================================================
    # Image Timer  (30 Hz with delay)
    # =========================================================================

    def _image_timer_cb(self):
        p_r_true = self._truth.p_int - self._truth.p_tgt
        R_b2e = quaternion_to_rotation(self._truth.q)
        p_cam = self._params.R_b2c @ R_b2e.T @ (-p_r_true)

        if p_cam[2] > 2.0:
            noise = self._params.sigma_img * self._rng.standard_normal(2)
            pbar  = np.array([p_cam[0]/p_cam[2], p_cam[1]/p_cam[2]]) + noise
            # Schedule delayed publish
            send_at = self._t + self._image_delay
            self._image_queue.append((send_at, pbar))

        # Flush any images whose delay has elapsed
        ready   = [(t, p) for t, p in self._image_queue if self._t >= t]
        pending = [(t, p) for t, p in self._image_queue if self._t <  t]
        self._image_queue = pending

        for _, pbar in ready:
            msg = Point()
            msg.x = float(pbar[0])
            msg.y = float(pbar[1])
            msg.z = 0.0
            self._image_pub.publish(msg)

    # =========================================================================
    # Radar Timer  (2 Hz)
    # Matches geometry_msgs/Vector3 on /radar/pr — same as relative_position_node.py
    # =========================================================================

    def _radar_timer_cb(self):
        p_r_true = self._truth.p_int - self._truth.p_tgt
        noise    = self._params.sigma_radar_pos * self._rng.standard_normal(3)
        z        = p_r_true + noise

        msg = Vector3()
        msg.x = float(z[0])   # North (NED)
        msg.y = float(z[1])   # East
        msg.z = float(z[2])   # Down
        self._radar_pub.publish(msg)

    # =========================================================================
    # Magnetometer Timer  (50 Hz)
    # =========================================================================

    def _mag_timer_cb(self):
        R_b2e  = quaternion_to_rotation(self._truth.q)
        # True mag field in body frame: R_e2b * B_ned (normalized)
        B_body = R_b2e.T @ self._params.B_ned
        noise  = self._params.sigma_mag_n * self._rng.standard_normal(3)
        z_body = B_body + noise

        # Scale to Tesla (typical ~50 μT), then publish in FLU (negate Y/Z)
        scale  = 50e-6   # approx field magnitude in Tesla
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.magnetic_field.x =  float(z_body[0] * scale)
        msg.magnetic_field.y = -float(z_body[1] * scale)   # FRD→FLU
        msg.magnetic_field.z = -float(z_body[2] * scale)
        self._mag_pub.publish(msg)

    # =========================================================================
    # MAVROS State Timer  (1 Hz)
    # =========================================================================

    def _state_timer_cb(self):
        msg = State()
        msg.armed     = (self._t < self._armed_until)
        msg.connected = True
        msg.mode      = 'OFFBOARD'
        self._state_pub.publish(msg)

        if math.isclose(self._t, self._armed_until, abs_tol=2.0):
            if msg.armed:
                self.get_logger().info("[TEST]: Will disarm at t={:.0f}s".format(self._armed_until))

    # =========================================================================
    # Output Subscribers
    # =========================================================================

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        o = msg.pose.pose.orientation
        self._latest_pos = np.array([p.x, p.y, p.z])
        self._latest_vel = np.array([v.x, v.y, v.z])
        self._latest_q   = np.array([o.w, o.x, o.y, o.z])

    def _pbar_callback(self, msg: Point):
        pass   # could log pbar tracking here


# ============================================================================
# Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)

    params = ESKFParams()
    if len(sys.argv) > 1:
        try:
            params = load_config(sys.argv[1])
        except Exception as e:
            print(f"[TEST]: Config load failed ({e}), using defaults.")

    node = ESKFRos2TestNode(params)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._rmse.print_summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
