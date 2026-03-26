"""
target_state_bridge.py
======================
Publish target LLA and NED velocity in the JSON format expected by
RadarEmulatorNode._target_state_callback.
"""

from __future__ import annotations

import datetime
import glob
import json
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class TargetStateBridgeNode(Node):
    """Bridge target MAVROS topics to compact JSON telemetry."""

    def __init__(self):
        super().__init__('target_state_bridge')

        self.declare_parameter('gps_topic', '/target/global_position/global')
        self.declare_parameter('local_odom_topic', '/target/global_position/local')
        self.declare_parameter('output_topic', '/target/telemetry/state')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('velocity_frame', 'enu')
        self.declare_parameter('max_state_age_sec', 1.0)
        self.declare_parameter('log_dir', '/home/ituarc/Documents/GitHub/Delayed-KalmanFilter-VisualGuidance/eskf_py/interceptor_sensor_emulators/log/target_bridge')

        self._gps_topic = self.get_parameter('gps_topic').value
        self._local_odom_topic = self.get_parameter('local_odom_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._velocity_frame = str(self.get_parameter('velocity_frame').value).strip().lower()
        self._max_state_age = float(self.get_parameter('max_state_age_sec').value)
        self._log_dir = str(self.get_parameter('log_dir').value)

        if self._publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        if self._velocity_frame not in {'enu', 'ned'}:
            raise ValueError("velocity_frame must be 'enu' or 'ned'")

        self._lat: float | None = None
        self._lon: float | None = None
        self._alt: float | None = None
        self._vn: float | None = None
        self._ve: float | None = None
        self._vd: float | None = None
        self._gps_time: float | None = None
        self._vel_time: float | None = None
        self._warn_timestamps: dict[str, float] = {}
        self._publish_count = 0
        self._gps_msg_count = 0
        self._odom_msg_count = 0
        self._rate_start_time = self._now_sec()

        self._txt_log_file = None
        self._setup_txt_logging()

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._gps_sub = self.create_subscription(
            NavSatFix,
            self._gps_topic,
            self._gps_callback,
            qos_profile_sensor_data,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self._local_odom_topic,
            self._local_odom_callback,
            qos_profile_sensor_data,
        )
        self._pub = self.create_publisher(String, self._output_topic, reliable_qos)
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_state)

        self._log_info(
            "[TARGET]: Bridge initialized\n"
            f"  gps_topic: {self._gps_topic}\n"
            f"  local_odom_topic: {self._local_odom_topic}\n"
            f"  output_topic: {self._output_topic}\n"
            f"  publish_rate: {self._publish_rate:.2f} Hz\n"
            f"  velocity_frame: {self._velocity_frame}"
        )

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
            self.get_logger().error(f"[TARGET LOG]: Failed to start text logging: {e}")
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

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_throttled(self, key: str, message: str, interval: float = 5.0):
        now = self._now_sec()
        if now - self._warn_timestamps.get(key, -float('inf')) >= interval:
            self._warn_timestamps[key] = now
            self._log_warning(message)

    def _gps_callback(self, msg: NavSatFix):
        self._gps_msg_count += 1
        self._lat = float(msg.latitude)
        self._lon = float(msg.longitude)
        self._alt = float(msg.altitude)
        self._gps_time = self._now_sec()

    def _local_odom_callback(self, msg: Odometry):
        self._odom_msg_count += 1
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        vz = float(msg.twist.twist.linear.z)

        if self._velocity_frame == 'enu':
            self._vn = vy
            self._ve = vx
            self._vd = -vz
        else:
            self._vn = vx
            self._ve = vy
            self._vd = vz

        self._vel_time = self._now_sec()

    def _is_stale(self, stamp: float | None) -> bool:
        if stamp is None:
            return True
        return (self._now_sec() - stamp) > self._max_state_age

    def _publish_state(self):
        if self._lat is None or self._lon is None or self._alt is None:
            self._warn_throttled('gps', 'Waiting for target GPS data.')
            return
        if self._vn is None or self._ve is None or self._vd is None:
            self._warn_throttled('vel', 'Waiting for target velocity data.')
            return
        if self._is_stale(self._gps_time):
            self._warn_throttled('gps_stale', 'Target GPS data is stale; skipping publish.')
            return
        if self._is_stale(self._vel_time):
            self._warn_throttled('vel_stale', 'Target velocity data is stale; skipping publish.')
            return

        payload = {
            'lat': self._lat,
            'lon': self._lon,
            'alt': self._alt,
            'vn': self._vn,
            've': self._ve,
            'vd': self._vd,
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self._pub.publish(msg)

        self._publish_count += 1
        if self._publish_count % max(1, round(self._publish_rate * 5.0)) == 0:
            now = self._now_sec()
            dt = now - self._rate_start_time
            gps_hz = self._gps_msg_count / dt if dt > 0 else 0.0
            odom_hz = self._odom_msg_count / dt if dt > 0 else 0.0

            self._log_info(
                "[TARGET]: Published state "
                f"lat={self._lat:.6f}, lon={self._lon:.6f}, alt={self._alt:.2f}, "
                f"vn={self._vn:.2f}, ve={self._ve:.2f}, vd={self._vd:.2f}, "
                f"gps={gps_hz:.1f}Hz, odom={odom_hz:.1f}Hz"
            )

            self._gps_msg_count = 0
            self._odom_msg_count = 0
            self._rate_start_time = now


def main(args=None):
    rclpy.init(args=args)
    node = TargetStateBridgeNode()
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
