"""
eskf_node.py
============
ROS2 Python node for the Error-State Kalman Filter.

Direct translation of eskf_node.cpp, using rclpy.

Subscribers:
    /mavros/imu/data_raw  (sensor_msgs/Imu)           – IMU in FLU → converted to FRD
    /mavros/imu/mag       (sensor_msgs/MagneticField)  – Magnetometer
    /yolo/target          (geometry_msgs/Point)         – Image feature (pbar)
    /radar/pr             (geometry_msgs/Vector3)        – Relative position from radar
    /mavros/state         (mavros_msgs/State)            – Armed state for mission-complete

Publishers:
    /eskf/odom  (nav_msgs/Odometry)   – pose + velocity + covariance
    /eskf/pbar  (geometry_msgs/Point) – estimated image features

Usage:
    ros2 run eskf_py eskf_node --ros-args -p config_file:=config/eskf_params.yaml
"""

import os
import math
import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

from eskf_py.eskf_core import ErrorStateKalmanFilter, compute_image_features
from eskf_py.eskf_config import load_config, print_config
from eskf_py.eskf_types import (
    ESKFParams, IMUMeasurement,
    DTHETA_START, DPR_START, DVR_START, DPBAR_START,
    BGYR_START, BACC_START, BMAG_START,
    PR_START, VR_START,
)
from eskf_py.eskf_math import quaternion_to_rotation


class ESKFNode(Node):
    """ROS2 node wrapping ErrorStateKalmanFilter. Mirrors C++ ESKFNode."""

    def __init__(self):
        super().__init__('eskf_node')

        # Parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('print_level', 'INFO')

        # Load config
        self._params = ESKFParams()
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        if config_file:
            try:
                self._params = load_config(config_file)
                self.get_logger().info(f"[CONFIG]: Loaded from {config_file}")
            except Exception as e:
                self.get_logger().warning(f"[CONFIG]: Failed ({e}). Using defaults.")
        print_config(self._params)

        # Create ESKF
        self._eskf = ErrorStateKalmanFilter(self._params)

        # Timing
        self._samples_per_eskf = max(1, round(self._params.dt_eskf / self._params.dt_imu))
        self._delay_steps = round(self._params.image_delay / self._params.dt_eskf)

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subscribers
        self._imu_sub = self.create_subscription(
            Imu, self._params.topic_imu, self._imu_callback, sensor_qos)
        self._mag_sub = self.create_subscription(
            MagneticField, self._params.topic_mag, self._mag_callback, sensor_qos)
        self._image_sub = self.create_subscription(
            Point, self._params.topic_image, self._image_callback, reliable_qos)
        self._radar_sub = self.create_subscription(
            Vector3, self._params.topic_radar, self._radar_callback, reliable_qos)
        self._state_sub = self.create_subscription(
            State, '/mavros/state', self._state_callback, reliable_qos)

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, self._params.topic_odom, 10)
        self._pbar_pub = self.create_publisher(Point, self._params.topic_pbar, 10)

        # Timers
        self._diag_timer      = self.create_timer(1.0,  self._diagnostics_callback)
        self._log_flush_timer = self.create_timer(30.0, self._log_flush_callback)

        # State
        self._is_initialized      = False
        self._imu_count           = 0
        self._first_imu_received  = False
        self._first_mag_received  = False
        self._first_image_received = False
        self._last_image_time     = 0.0
        self._prediction_count    = 0
        self._was_armed           = False

        # IMU buffer for static initialization (400 samples ~ 2s at 200 Hz)
        self._init_imu_buffer: list[dict] = []
        self._init_buffer_size = 400

        # Logging
        self._log_file    = None
        self._log_skip_counter  = 0
        self._log_skip_samples  = 1
        if self._params.log_enabled:
            self._init_logging()

        self.get_logger().info(
            f"[ESKF]: Node initialized\n"
            f"  IMU: {self._params.topic_imu}\n"
            f"  Image: {self._params.topic_image}\n"
            f"  Radar: {self._params.topic_radar}\n"
            f"  Samples/update: {self._samples_per_eskf}\n"
            f"  Image delay: {self._delay_steps} steps"
        )
        self.get_logger().warning("[ESKF]: Waiting for first radar measurement...")

    def destroy_node(self):
        if self._log_file:
            self._log_file.close()
            self.get_logger().info("[LOG]: File closed")
        super().destroy_node()

    # =========================================================================
    # IMU Callback
    # =========================================================================

    def _imu_callback(self, msg: Imu):
        if not self._first_imu_received:
            self._first_imu_received = True
            self.get_logger().info("[IMU]: First message received")

        # FLU (MAVROS) → FRD (ESKF): negate Y and Z
        omega = np.array([ msg.angular_velocity.x,
                          -msg.angular_velocity.y,
                          -msg.angular_velocity.z])
        accel = np.array([ msg.linear_acceleration.x,
                          -msg.linear_acceleration.y,
                          -msg.linear_acceleration.z])

        # Buffer IMU before initialization
        if not self._is_initialized:
            if len(self._init_imu_buffer) < self._init_buffer_size:
                self._init_imu_buffer.append({'omega': omega, 'accel': accel})
                if len(self._init_imu_buffer) % 100 == 0:
                    self.get_logger().info(
                        f"[INIT]: Buffering IMU... {len(self._init_imu_buffer)}/{self._init_buffer_size}")
            return

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ZUPT logic
        if self._eskf.is_zupt_enabled():
            if self._eskf.detect_zupt(omega, accel):
                self._eskf.correct_zupt(omega, accel)
            else:
                if self._eskf.was_radar_received() and self._eskf.was_zupt_triggered():
                    self._eskf.disable_zupt()
                    self.get_logger().info("[ZUPT]: Disabled — platform moving")

        # Accumulate and predict
        self._eskf.accumulate_imu(omega, accel)
        self._imu_count += 1

        if self._imu_count >= self._samples_per_eskf:
            avg = self._eskf.get_averaged_imu()
            self._eskf.predict(avg.omega, avg.accel, timestamp)
            self._imu_count = 0
            self._prediction_count += 1

            # Image timeout check
            if self._last_image_time > 0.0:
                elapsed = timestamp - self._last_image_time
                if elapsed > self._params.image_timeout_sec and not self._eskf.is_pbar_frozen():
                    self._eskf.set_pbar_frozen(True)
                    self.get_logger().warning(
                        f"[IMAGE]: Timeout ({elapsed:.1f}s) — pbar frozen")

            self._publish_state(msg.header.stamp)

    # =========================================================================
    # Image Callback
    # =========================================================================

    def _image_callback(self, msg: Point):
        if not self._first_image_received:
            self._first_image_received = True
            self.get_logger().info(f"[IMAGE]: First message — pbar=[{msg.x:.4f}, {msg.y:.4f}]")

        if not self._is_initialized:
            return

        timestamp = self.get_clock().now().nanoseconds * 1e-9
        if self._eskf.is_pbar_frozen():
            self._eskf.set_pbar_frozen(False)
            self.get_logger().info("[IMAGE]: Resumed after timeout")

        self._last_image_time = timestamp
        z_pbar = np.array([msg.x, msg.y])
        self._eskf.correct_image(z_pbar, self._delay_steps)

    # =========================================================================
    # Radar Callback
    # =========================================================================

    def _radar_callback(self, msg: Vector3):
        z_radar = np.array([msg.x, msg.y, msg.z, 0.0, 0.0, 0.0])

        if not self._is_initialized:
            self._initialize_filter(z_radar)
            return

        self._eskf.notify_radar_received()
        self._eskf.correct_radar(z_radar)

    # =========================================================================
    # Magnetometer Callback
    # =========================================================================

    def _mag_callback(self, msg: MagneticField):
        if not self._params.enable_mag:
            return

        if not self._first_mag_received:
            self._first_mag_received = True
            mag_uT = math.sqrt(msg.magnetic_field.x**2 +
                                msg.magnetic_field.y**2 +
                                msg.magnetic_field.z**2) * 1e-3  # T → μT (×1e6 / ×1e3)
            self.get_logger().info(f"[MAG]: First message — |B|≈{mag_uT:.2f} μT")

        if not self._is_initialized:
            return

        # FLU → FRD
        z_raw = np.array([ msg.magnetic_field.x,
                           -msg.magnetic_field.y,
                           -msg.magnetic_field.z])
        mag_norm = np.linalg.norm(z_raw)
        if mag_norm < 1e-6:
            return
        z_mag = z_raw / mag_norm  # normalize to unit vector

        self._eskf.correct_mag(z_mag)

    # =========================================================================
    # MAVROS State Callback
    # =========================================================================

    def _state_callback(self, msg: State):
        currently_armed = msg.armed
        if self._was_armed and not currently_armed:
            self.get_logger().info("[STATE]: Drone DISARMED — mission complete")
            if self._log_file:
                self._log_file.flush()
                self._log_file.close()
                self._log_file = None
                self.get_logger().info("[LOG]: File saved and closed")
            rclpy.shutdown()
        if currently_armed and not self._was_armed:
            self.get_logger().info("[STATE]: Drone ARMED — mission started")
        self._was_armed = currently_armed

    # =========================================================================
    # Filter Initialization (first radar measurement)
    # =========================================================================

    def _initialize_filter(self, z_radar: np.ndarray):
        self.get_logger().info(
            f"\n========== ESKF INITIALIZATION ==========\n"
            f"[RADAR]: First measurement pos=[{z_radar[0]:.2f}, {z_radar[1]:.2f}, {z_radar[2]:.2f}]")

        q_init = self._compute_initial_attitude()

        from eskf_py.eskf_types import (NOMINAL_SIZE, PR_START, VR_START,
                                         PBAR_START, BGYR_START, BACC_START, BMAG_START)
        import numpy as np
        x_init = np.zeros(NOMINAL_SIZE)
        from eskf_py.eskf_types import set_quaternion
        set_quaternion(x_init, q_init)
        x_init[PR_START:PR_START+3] = z_radar[:3]
        x_init[VR_START:VR_START+3] = z_radar[3:]

        self._eskf.reset(x_init, self._eskf.get_covariance())
        self._is_initialized = True
        self._eskf.notify_radar_received()
        self._init_imu_buffer.clear()

        self.get_logger().info("[ESKF]: Filter RUNNING\n==========================================")

    def _compute_initial_attitude(self) -> np.ndarray:
        """Estimate roll/pitch from averaged gravity in buffered IMU samples."""
        if not self._init_imu_buffer:
            self.get_logger().warning("[INIT]: No IMU samples — using identity quaternion")
            return np.array([1.0, 0.0, 0.0, 0.0])

        accels = np.array([s['accel'] for s in self._init_imu_buffer])
        accel_avg = accels.mean(axis=0)
        variance = float(np.mean(np.sum((accels - accel_avg)**2, axis=1)))

        self.get_logger().info(
            f"[INIT]: {len(self._init_imu_buffer)} samples, variance={variance:.4f}\n"
            f"  Gravity measured: {accel_avg}")
        if variance > 1.0:
            self.get_logger().warning("[INIT]: High variance — platform may not be stationary!")

        g = -accel_avg / (np.linalg.norm(accel_avg) + 1e-9)
        roll  = math.atan2(float(g[1]), float(g[2]))
        pitch = math.atan2(-float(g[0]), math.sqrt(float(g[1])**2 + float(g[2])**2))

        self.get_logger().info(
            f"[INIT]: roll={math.degrees(roll):.1f}° pitch={math.degrees(pitch):.1f}°")

        # Build quaternion from ZYX euler angles (yaw=0)
        cr, sr = math.cos(roll/2),  math.sin(roll/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cy, sy = 1.0, 0.0             # yaw = 0

        # q = q_yaw * q_pitch * q_roll
        q = np.array([
            cy*cp*cr + sy*sp*sr,
            cy*cp*sr - sy*sp*cr,
            cy*sp*cr + sy*cp*sr,
            sy*cp*cr - cy*sp*sr,
        ])
        return q / np.linalg.norm(q)

    # =========================================================================
    # State Publisher
    # =========================================================================

    def _publish_state(self, stamp):
        q    = self._eskf.get_quaternion()
        pos  = self._eskf.get_position()
        vel  = self._eskf.get_velocity()
        pbar = self._eskf.get_pbar()
        P_diag = self._eskf.get_covariance_diagonal()

        # Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'world'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])

        # Pose covariance: position then orientation
        for i in range(3):
            odom.pose.covariance[i * 7]     = float(P_diag[DPR_START + i])
            odom.pose.covariance[(i+3) * 7] = float(P_diag[DTHETA_START + i])

        odom.twist.twist.linear.x  = float(vel[0])
        odom.twist.twist.linear.y  = float(vel[1])
        odom.twist.twist.linear.z  = float(vel[2])
        for i in range(3):
            odom.twist.covariance[i * 7] = float(P_diag[DVR_START + i])

        self._odom_pub.publish(odom)

        # Pbar
        pbar_msg = Point()
        pbar_msg.x = float(pbar[0])
        pbar_msg.y = float(pbar[1])
        pbar_msg.z = 0.0
        self._pbar_pub.publish(pbar_msg)

        # CSV logging
        if self._log_file:
            self._log_skip_counter += 1
            if self._log_skip_counter >= self._log_skip_samples:
                self._log_skip_counter = 0
                ts = stamp.sec + stamp.nanosec * 1e-9
                R = quaternion_to_rotation(q)
                # ZYX Euler → yaw, pitch, roll
                e = _rotation_to_euler_zyx(R)
                ed = np.degrees(e)

                bgyr = self._eskf.get_gyro_bias()
                bacc = self._eskf.get_accel_bias()
                bmag = self._eskf.get_mag_bias()

                row = [ts] + ed.tolist() + pos.tolist() + vel.tolist() + \
                      pbar.tolist() + bgyr.tolist() + bacc.tolist() + bmag.tolist() + \
                      P_diag.tolist()
                self._log_file.write(','.join(f'{v:.6f}' for v in row) + '\n')

    # =========================================================================
    # Timers
    # =========================================================================

    def _diagnostics_callback(self):
        if not self._is_initialized:
            self.get_logger().warning("[ESKF]: Waiting for radar...")
            return

        hz = self._prediction_count
        self._prediction_count = 0

        P_diag = self._eskf.get_covariance_diagonal()
        q   = self._eskf.get_quaternion()
        pos = self._eskf.get_position()
        vel = self._eskf.get_velocity()
        pb  = self._eskf.get_pbar()

        R = quaternion_to_rotation(q)
        e = np.degrees(_rotation_to_euler_zyx(R))

        self.get_logger().info(
            f"[STATE]: att=[{e[0]:.1f},{e[1]:.1f},{e[2]:.1f}]° "
            f"pos=[{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}]m "
            f"vel=[{vel[0]:.2f},{vel[1]:.2f},{vel[2]:.2f}]m/s "
            f"pbar=[{pb[0]:.4f},{pb[1]:.4f}]  {hz:.0f} Hz"
        )

    def _log_flush_callback(self):
        if self._log_file:
            self._log_file.flush()
            self.get_logger().debug("[LOG]: Periodic flush (30 s)")

    # =========================================================================
    # Logging Setup
    # =========================================================================

    def _init_logging(self):
        eskf_rate = 1.0 / self._params.dt_eskf
        self._log_skip_samples = max(1, round(eskf_rate / self._params.log_rate_hz))

        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'log')
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, f'eskf_{ts}.csv')

        try:
            self._log_file = open(log_path, 'w')
            header = ('timestamp,roll_deg,pitch_deg,yaw_deg,x,y,z,vx,vy,vz,'
                      'pbar_x,pbar_y,bgyr_x,bgyr_y,bgyr_z,bacc_x,bacc_y,bacc_z,'
                      'bmag_x,bmag_y,bmag_z,' +
                      ','.join([f'P_{i}' for i in range(20)]))
            self._log_file.write(header + '\n')
            actual_rate = eskf_rate / self._log_skip_samples
            self.get_logger().info(f"[LOG]: Logging to {log_path} @ {actual_rate:.1f} Hz")
        except IOError as e:
            self.get_logger().error(f"[LOG]: Failed to open {log_path}: {e}")
            self._log_file = None


# ============================================================================
# Helpers
# ============================================================================

def _rotation_to_euler_zyx(R: np.ndarray) -> np.ndarray:
    """ZYX Euler angles [yaw, pitch, roll] from rotation matrix, in radians."""
    pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0]**2 + R[1, 0]**2))
    if abs(math.cos(pitch)) < 1e-6:
        yaw  = math.atan2(R[0, 1], R[1, 1])
        roll = 0.0
    else:
        yaw  = math.atan2(R[1, 0], R[0, 0])
        roll = math.atan2(R[2, 1], R[2, 2])
    return np.array([yaw, pitch, roll])


# ============================================================================
# Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ESKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
