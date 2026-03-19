"""
target_state_bridge.py
======================
Publish target LLA and NED velocity in the JSON format expected by
RadarEmulatorNode._target_state_callback.
"""

from __future__ import annotations

import json

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

        self._gps_topic = self.get_parameter('gps_topic').value
        self._local_odom_topic = self.get_parameter('local_odom_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._velocity_frame = str(self.get_parameter('velocity_frame').value).strip().lower()
        self._max_state_age = float(self.get_parameter('max_state_age_sec').value)

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

        self.get_logger().info(
            "[TARGET]: Bridge initialized\n"
            f"  gps_topic: {self._gps_topic}\n"
            f"  local_odom_topic: {self._local_odom_topic}\n"
            f"  output_topic: {self._output_topic}\n"
            f"  publish_rate: {self._publish_rate:.2f} Hz\n"
            f"  velocity_frame: {self._velocity_frame}"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_throttled(self, key: str, message: str, interval: float = 5.0):
        now = self._now_sec()
        if now - self._warn_timestamps.get(key, -float('inf')) >= interval:
            self._warn_timestamps[key] = now
            self.get_logger().warning(message)

    def _gps_callback(self, msg: NavSatFix):
        self._lat = float(msg.latitude)
        self._lon = float(msg.longitude)
        self._alt = float(msg.altitude)
        self._gps_time = self._now_sec()

    def _local_odom_callback(self, msg: Odometry):
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
            self.get_logger().info(
                "[TARGET]: Published state "
                f"lat={self._lat:.6f}, lon={self._lon:.6f}, alt={self._alt:.2f}, "
                f"vn={self._vn:.2f}, ve={self._ve:.2f}, vd={self._vd:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TargetStateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
