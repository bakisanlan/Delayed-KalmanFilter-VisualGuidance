"""
ROS2 camera emulator node.

Subscribes to interceptor/target LLA and interceptor local odometry, then
publishes the target's projected pixel coordinates at a configurable rate.
"""

import csv
import datetime
import glob
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

from .camera_model import PinholeRadtanCamera
from .measure import measure, mavros_enu_flu_to_ned_frd_rotation


class CameraEmulatorNode(Node):
    """Emulates a forward-looking pinhole+radtan camera on the interceptor."""

    def __init__(self):
        super().__init__('camera_emulator_node')

        # ------------------------------------------------------------------ #
        # Declare parameters (with defaults)
        # ------------------------------------------------------------------ #
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fx', 320.0)
        self.declare_parameter('fy', 320.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('dist_coeffs', [-0.1, 0.01, 0.0, 0.0])
        self.declare_parameter('noise_sigma', 1.5)
        self.declare_parameter('max_range', 100.0)
        self.declare_parameter('false_detection_prob', 0.1)
        self.declare_parameter('dropout_prob_at_max_range', 0.5)
        self.declare_parameter('delay_ms', 80)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('visualize_target', False)
        self.declare_parameter('log_dir', '/home/ituarc/Documents/GitHub/Delayed-KalmanFilter-VisualGuidance/eskf_py/interceptor_sensor_emulators/log/camera')

        # Topic names (easy to remap later)
        self.declare_parameter('interceptor_lla_topic',
                               '/interceptor/global_position/global')
        self.declare_parameter('target_lla_topic',
                               '/target/global_position/global')
        self.declare_parameter('interceptor_local_odom_topic',
                               '/interceptor/global_position/local')
        self.declare_parameter('target_pixel_topic',
                               '/interceptor/camera/target_pixel')
        self.declare_parameter('target_pixel_true_topic',
                               '/interceptor/camera/target_pbar_true')
        self.declare_parameter('eskf_pbar_topic',
                               '/eskf_reduced/pbar')
        self.declare_parameter('state_topic',
                               '/interceptor/state')

        # ------------------------------------------------------------------ #
        # Read parameters
        # ------------------------------------------------------------------ #
        img_w = self.get_parameter('image_width').value
        img_h = self.get_parameter('image_height').value
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value
        dist_coeffs = self.get_parameter('dist_coeffs').value
        noise_sigma = self.get_parameter('noise_sigma').value
        self.max_range = self.get_parameter('max_range').value
        self.false_detection_prob = self.get_parameter('false_detection_prob').value
        self.dropout_prob_at_max_range = self.get_parameter('dropout_prob_at_max_range').value
        self.delay_ms = self.get_parameter('delay_ms').value
        rate_hz = self.get_parameter('rate_hz').value
        self.visualize_target = self.get_parameter('visualize_target').value
        self._log_dir = str(self.get_parameter('log_dir').value)

        self._txt_log_file = None
        self._setup_txt_logging()

        interceptor_lla_topic = self.get_parameter('interceptor_lla_topic').value
        target_lla_topic = self.get_parameter('target_lla_topic').value
        interceptor_local_odom_topic = self.get_parameter('interceptor_local_odom_topic').value
        target_pixel_topic = self.get_parameter('target_pixel_topic').value
        target_pixel_true_topic = self.get_parameter('target_pixel_true_topic').value
        eskf_pbar_topic = self.get_parameter('eskf_pbar_topic').value
        state_topic = self.get_parameter('state_topic').value

        # ------------------------------------------------------------------ #
        # Build camera model
        # ------------------------------------------------------------------ #
        K = np.array([
            [fx,  0.0, cx],
            [0.0, fy,  cy],
            [0.0, 0.0, 1.0],
        ])

        # Camera-to-body rotation:
        #   cam-z → body-x (forward)
        #   cam-x → body-y (right)
        #   cam-y → body-z (down)
        R_c2b = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0],
        ], dtype=np.float64)

        self.camera = PinholeRadtanCamera(
            K=K,
            dist_coeffs=np.array(dist_coeffs),
            image_width=img_w,
            image_height=img_h,
            R_c2b=R_c2b,
            noise_sigma=noise_sigma,
        )

        self._log_info(
            f'Camera: {img_w}×{img_h}, fx={fx}, fy={fy}, '
            f'cx={cx}, cy={cy}, dist={dist_coeffs}, σ={noise_sigma} px'
        )

        # ------------------------------------------------------------------ #
        # State variables (latest received data)
        # ------------------------------------------------------------------ #
        self._interceptor_lla = None  # (lat, lon, alt)
        self._target_lla = None       # (lat, lon, alt)
        self._R_body2ned = None       # 3×3
        self._eskf_pbar = None        # (px, py)

        self._is_armed = False
        self._csv_file = None
        self._csv_writer = None

        # ------------------------------------------------------------------ #
        # QoS – best-effort for high-rate sensor data
        # ------------------------------------------------------------------ #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ------------------------------------------------------------------ #
        # Subscriptions
        # ------------------------------------------------------------------ #
        self.create_subscription(
            NavSatFix, interceptor_lla_topic,
            self._interceptor_lla_cb, sensor_qos)
        self.create_subscription(
            NavSatFix, target_lla_topic,
            self._target_lla_cb, sensor_qos)
        self.create_subscription(
            Odometry, interceptor_local_odom_topic,
            self._interceptor_local_odom_cb, sensor_qos)
        self.create_subscription(
            Point, eskf_pbar_topic,
            self._eskf_pbar_cb, 10)
        self.create_subscription(
            State, state_topic,
            self._state_callback, sensor_qos)

        self._log_info(
            f'Subscribed to:\n'
            f'  interceptor LLA : {interceptor_lla_topic}\n'
            f'  target LLA      : {target_lla_topic}\n'
            f'  interceptor odom: {interceptor_local_odom_topic}\n'
            f'  interceptor state: {state_topic}\n'
            f'  ESKF pbar       : {eskf_pbar_topic}'
        )

        # ------------------------------------------------------------------ #
        # Publisher
        # ------------------------------------------------------------------ #
        self.pixel_pub = self.create_publisher(
            PointStamped, target_pixel_topic, 10)
        self.pixel_true_pub = self.create_publisher(
            PointStamped, target_pixel_true_topic, 10)
        self._log_info(f'Publishing target pbar on: {target_pixel_topic}')
        self._log_info(f'Publishing true target pbar on: {target_pixel_true_topic}')

        # ------------------------------------------------------------------ #
        # Timer
        # ------------------------------------------------------------------ #
        period = 1.0 / rate_hz
        self.create_timer(period, self._timer_cb)
        self._log_info(f'Timer running at {rate_hz} Hz')

        # Queue for delayed publishing
        self._publish_queue = []
        self.create_timer(0.01, self._process_publish_queue)  # 100 Hz queue processor

    # ===================================================================== #
    # Callbacks
    # ===================================================================== #

    def _setup_txt_logging(self):
        try:
            os.makedirs(self._log_dir, exist_ok=True)
            existing_files = glob.glob(os.path.join(self._log_dir, '*-*.log'))
            max_count = 0
            for f in existing_files:
                basename = os.path.basename(f)
                parts = basename.split('-')
                if len(parts) >= 2 and parts[0].isdigit():
                    max_count = max(max_count, int(parts[0]))
            
            next_count = max_count + 1
            now_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"{next_count:04d}-{now_str}.log"
            filepath = os.path.join(self._log_dir, filename)
            
            self._txt_log_file = open(filepath, 'w')
            self._log_info(f"Text logging initialized at {filepath}")
        except Exception as e:
            self.get_logger().error(f"[CAMERA LOG]: Failed to start text logging: {e}")
            self._txt_log_file = None

    def _write_txt_log(self, msg: str):
        if self._txt_log_file is not None:
            try:
                self._txt_log_file.write(f"[{datetime.datetime.now().isoformat()}] {msg}\n")
                self._txt_log_file.flush()
            except Exception:
                pass

    def _log_info(self, msg: str):
        self.get_logger().info(msg)
        self._write_txt_log(f"[INFO] {msg}")

    def _log_warning(self, msg: str):
        self.get_logger().warning(msg)
        self._write_txt_log(f"[WARNING] {msg}")

    def _log_error(self, msg: str):
        self.get_logger().error(msg)
        self._write_txt_log(f"[ERROR] {msg}")

    def _state_callback(self, msg: State):
        if msg.armed and not self._is_armed:
            self._is_armed = True
            self._start_logging()
        elif not msg.armed and self._is_armed:
            self._is_armed = False
            self._stop_logging()

    def _start_logging(self):
        try:
            os.makedirs(self._log_dir, exist_ok=True)
            existing_files = glob.glob(os.path.join(self._log_dir, '*-*.csv'))
            max_count = 0
            for f in existing_files:
                basename = os.path.basename(f)
                parts = basename.split('-')
                if len(parts) >= 2 and parts[0].isdigit():
                    max_count = max(max_count, int(parts[0]))
            
            next_count = max_count + 1
            now_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"{next_count:04d}-{now_str}.csv"
            filepath = os.path.join(self._log_dir, filename)
            
            self._csv_file = open(filepath, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'timestamp',
                'true_pbar_x', 'true_pbar_y',
                'meas_pbar_x', 'meas_pbar_y',
            ])
            self._log_info(f"[CAMERA LOG]: Started logging to {filepath}")
        except Exception as e:
            self._log_error(f"[CAMERA LOG]: Failed to start logging: {e}")
            self._csv_file = None
            self._csv_writer = None

    def _stop_logging(self):
        if self._csv_file is not None:
            try:
                self._csv_file.close()
                self._log_info("[CAMERA LOG]: Stopped logging")
            except Exception as e:
                self._log_error(f"[CAMERA LOG]: Failed to close log file: {e}")
            self._csv_file = None
            self._csv_writer = None

    def _interceptor_lla_cb(self, msg: NavSatFix):
        self._interceptor_lla = (msg.latitude, msg.longitude, msg.altitude)

    def _target_lla_cb(self, msg: NavSatFix):
        self._target_lla = (msg.latitude, msg.longitude, msg.altitude)

    def _interceptor_local_odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return

        self._R_body2ned = mavros_enu_flu_to_ned_frd_rotation(
            q.x, q.y, q.z, q.w,
        )

    def _eskf_pbar_cb(self, msg: Point):
        self._eskf_pbar = (msg.x, msg.y)

    def _timer_cb(self):
    
        # Wait for all inputs
        if self._interceptor_lla is None:
            return
        if self._target_lla is None:
            return
        if self._R_body2ned is None:
            return


        # Compute pixel
        result_meas, result_true = measure(
            camera=self.camera,
            interceptor_lla=self._interceptor_lla,
            target_lla=self._target_lla,
            R_body2ned=self._R_body2ned,
            max_range=self.max_range,
            false_detection_prob=self.false_detection_prob,
            dropout_prob_at_max_range=self.dropout_prob_at_max_range,
        )

        # Publish measured detection (with noise, dropout, and false detection)
        now = self.get_clock().now()
        target_publish_time = now + Duration(nanoseconds=int(self.delay_ms * 1e6))

        if self._csv_writer is not None:
            try:
                now_sec = now.nanoseconds * 1e-9
                tpx = result_true[0] if result_true else np.nan
                tpy = result_true[1] if result_true else np.nan
                mpx = result_meas[0] if result_meas else np.nan
                mpy = result_meas[1] if result_meas else np.nan
                self._csv_writer.writerow([now_sec, tpx, tpy, mpx, mpy])
                self._csv_file.flush()
            except Exception as e:
                self._log_warning(f"[CAMERA LOG]: Failed to write to CSV log: {e}")

        msg_meas = None
        if result_meas is not None:
            pbar_x, pbar_y = result_meas
            msg_meas = PointStamped()
            msg_meas.header.stamp = now.to_msg()
            msg_meas.header.frame_id = 'camera'
            msg_meas.point.x = pbar_x
            msg_meas.point.y = pbar_y
            msg_meas.point.z = 0.0

        if result_true is not None:
            pbar_true_x, pbar_true_y = result_true
            msg_true = PointStamped()
            msg_true.header.stamp = now.to_msg()
            msg_true.header.frame_id = 'camera'
            msg_true.point.x = pbar_true_x
            msg_true.point.y = pbar_true_y
            msg_true.point.z = 0.0
            self.pixel_true_pub.publish(msg_true)

        if msg_meas is not None:
            self._publish_queue.append((target_publish_time, msg_meas))

    def _process_publish_queue(self):
        now = self.get_clock().now()
        while self._publish_queue and self._publish_queue[0][0] <= now:
            _, msg_meas = self._publish_queue.pop(0)
            if msg_meas is not None:
                self.pixel_pub.publish(msg_meas)




# ---------------------------------------------------------------------- #
# Entry point
# ---------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = CameraEmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._txt_log_file is not None:
            try:
                node._txt_log_file.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
