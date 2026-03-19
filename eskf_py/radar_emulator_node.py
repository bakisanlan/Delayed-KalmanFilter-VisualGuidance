"""
radar_emulator_node.py
======================
ROS2 node wrapper around the Python RadarEmulator.
"""

from __future__ import annotations

import json
import math
import pathlib
import re
import sys

import numpy as np
import rclpy
import yaml
from mavros_msgs.msg import HomePosition
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String

if __package__ in (None, ''):
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from eskf_py.geodesy import NEDFrame
from eskf_py.radar_emulator import RadarEmulator


class RadarEmulatorNode(Node):
    """Subscribe to target telemetry and publish noisy radar measurements."""

    _ALIASES = {
        'lat': ('lat', 'latitude'),
        'lon': ('lon', 'lng', 'longitude'),
        'alt': ('alt', 'altitude'),
        'vn': ('vn', 'v_n', 'vel_n', 'velocity_north', 'north_velocity'),
        've': ('ve', 'v_e', 'vel_e', 'velocity_east', 'east_velocity'),
        'vd': ('vd', 'v_d', 'vel_d', 'velocity_down', 'down_velocity'),
    }

    @staticmethod
    def _default_config_path() -> pathlib.Path:
        return pathlib.Path(__file__).resolve().parent / 'config' / 'radar_emulator_params.yaml'

    @classmethod
    def _load_yaml_params(cls, config_path: pathlib.Path) -> dict:
        if not config_path.exists():
            return {}

        with config_path.open('r', encoding='utf-8') as handle:
            config = yaml.safe_load(handle) or {}

        node_block = (
            config.get('/radar_emulator_node')
            or config.get('radar_emulator_node')
            or {}
        )
        ros_params = node_block.get('ros__parameters', {})
        if not isinstance(ros_params, dict):
            return {}
        return ros_params

    def __init__(self):
        super().__init__('radar_emulator_node')

        self._config_path = self._default_config_path()
        self._yaml_params = self._load_yaml_params(self._config_path)

        self.declare_parameter(
            'target_state_topic',
            self._yaml_params.get('target_state_topic', '/target/telemetry/state'))
        self.declare_parameter(
            'home_position_topic',
            self._yaml_params.get('home_position_topic', '/mavros/home_position/home'))
        self.declare_parameter(
            'odom_topic',
            self._yaml_params.get('odom_topic', '/radar/emulated_measurement'))
        self.declare_parameter(
            'frame_id',
            self._yaml_params.get('frame_id', 'ref_ned'))
        self.declare_parameter(
            'child_frame_id',
            self._yaml_params.get('child_frame_id', 'radar_target'))
        self.declare_parameter(
            'radar_lla',
            self._yaml_params.get('radar_lla', [39.778354, 32.315964, 0.0]))
        self.declare_parameter(
            'dt',
            self._yaml_params.get('dt', 0.5))
        self.declare_parameter(
            'max_state_age_sec',
            self._yaml_params.get('max_state_age_sec', 2.0))
        self.declare_parameter(
            'velocity_source',
            self._yaml_params.get('velocity_source', 'auto'))
        self.declare_parameter(
            'rmse_range',
            self._yaml_params.get('rmse_range', 3.0))
        self.declare_parameter(
            'rmse_azimuth_deg',
            self._yaml_params.get('rmse_azimuth_deg', 1.3))
        self.declare_parameter(
            'rmse_elevation_deg',
            self._yaml_params.get('rmse_elevation_deg', 3.3))
        self.declare_parameter(
            'rmse_doppler',
            self._yaml_params.get('rmse_doppler', 0.5))
        self.declare_parameter(
            'random_seed',
            self._yaml_params.get('random_seed', -1))

        self._target_state_topic = self.get_parameter('target_state_topic').value
        self._home_position_topic = self.get_parameter('home_position_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._child_frame_id = self.get_parameter('child_frame_id').value
        self._radar_lla = np.asarray(self.get_parameter('radar_lla').value, dtype=float).reshape(3)
        self._dt = float(self.get_parameter('dt').value)
        self._max_state_age = float(self.get_parameter('max_state_age_sec').value)
        self._velocity_source = str(self.get_parameter('velocity_source').value).strip().lower()
        self._seed = int(self.get_parameter('random_seed').value)
        self._noise_params = {
            'rmse_range': float(self.get_parameter('rmse_range').value),
            'rmse_azimuth': math.radians(float(self.get_parameter('rmse_azimuth_deg').value)),
            'rmse_elevation': math.radians(float(self.get_parameter('rmse_elevation_deg').value)),
            'rmse_doppler': float(self.get_parameter('rmse_doppler').value),
        }

        if self._velocity_source not in {'auto', 'reported', 'estimated'}:
            raise ValueError("velocity_source must be one of: auto, reported, estimated")

        self._rng = np.random.default_rng(None if self._seed < 0 else self._seed)
        self._ref_lla0: np.ndarray | None = None
        self._ref_frame: NEDFrame | None = None
        self._emulator: RadarEmulator | None = None
        self._target_lla: np.ndarray | None = None
        self._target_vel_ned: np.ndarray | None = None
        self._prev_target_lla: np.ndarray | None = None
        self._prev_target_update_time: float | None = None
        self._last_target_update_time: float | None = None
        self._warn_timestamps: dict[str, float] = {}

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._home_sub = self.create_subscription(
            HomePosition,
            self._home_position_topic,
            self.home_position_callback,
            qos_profile_sensor_data,
        )
        self._target_sub = self.create_subscription(
            String,
            self._target_state_topic,
            self._target_state_callback,
            reliable_qos,
        )
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, reliable_qos)
        self._timer = self.create_timer(self._dt, self._publish_measurement)

        self.get_logger().info(
            "[RADAR]: Node initialized\n"
            f"  config_file: {self._config_path}\n"
            f"  home_position_topic: {self._home_position_topic}\n"
            f"  target_state_topic: {self._target_state_topic}\n"
            f"  odom_topic: {self._odom_topic}\n"
            f"  dt: {self._dt:.3f} s\n"
            f"  velocity_source: {self._velocity_source}\n"
            f"  radar_lla: {self._radar_lla.tolist()}"
        )
        self.get_logger().warning("[RADAR]: Waiting for /mavros/home_position/home before creating RadarEmulator")

    def _create_emulator(self):
        if self._ref_lla0 is None:
            return

        self._emulator = RadarEmulator(
            radar_lla=self._ref_lla0,  #no offset for now
            ref_lla0=self._ref_lla0,
            dt=self._dt,
            rmse_range=self._noise_params['rmse_range'],
            rmse_azimuth=self._noise_params['rmse_azimuth'],
            rmse_elevation=self._noise_params['rmse_elevation'],
            rmse_doppler=self._noise_params['rmse_doppler'],
            rng=self._rng,
        )
        self._ref_frame = NEDFrame(self._ref_lla0)

    def home_position_callback(self, msg: HomePosition):
        ref_lla0 = np.array([
            msg.geo.latitude,
            msg.geo.longitude,
            msg.geo.altitude,
        ], dtype=float)

        if self._ref_lla0 is not None and np.allclose(self._ref_lla0, ref_lla0):
            return

        ref_changed = self._ref_lla0 is not None
        self._ref_lla0 = ref_lla0
        self._create_emulator()
        self._target_vel_ned = None

        if ref_changed:
            self.get_logger().warning(
                f"[RADAR]: Home position changed. Recreated emulator with ref_lla0={self._ref_lla0.tolist()}"
            )
        else:
            self.get_logger().info(
                f"[RADAR]: Home position received. ref_lla0={self._ref_lla0.tolist()}"
            )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _warn_throttled(self, key: str, message: str, interval: float = 5.0):
        now = self._now_sec()
        if now - self._warn_timestamps.get(key, -float('inf')) >= interval:
            self._warn_timestamps[key] = now
            self.get_logger().warning(message)

    def _extract_numeric(self, raw: str, aliases) -> float | None:
        for alias in aliases:
            pattern = rf'"?{re.escape(alias)}"?\s*:?\s*(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)'
            match = re.search(pattern, raw, flags=re.IGNORECASE)
            if match:
                return float(match.group(1))
        return None

    def _extract_value(self, data: dict, raw: str, key: str) -> float | None:
        aliases = self._ALIASES[key]
        for alias in aliases:
            if alias in data:
                return float(data[alias])
        return self._extract_numeric(raw, aliases)

    def _parse_target_state(self, raw: str):
        try:
            payload = json.loads(raw)
            if isinstance(payload, dict):
                data = {str(key).lower(): value for key, value in payload.items()}
            else:
                data = {}
        except json.JSONDecodeError:
            data = {}

        lat = self._extract_value(data, raw, 'lat')
        lon = self._extract_value(data, raw, 'lon')
        alt = self._extract_value(data, raw, 'alt')
        if lat is None or lon is None or alt is None:
            return None, None

        reported_vel = None
        vn = self._extract_value(data, raw, 'vn')
        ve = self._extract_value(data, raw, 've')
        vd = self._extract_value(data, raw, 'vd')

        if vn is not None and ve is not None and vd is not None:
            reported_vel = np.array([vn, ve, vd], dtype=float)

        return np.array([lat, lon, alt], dtype=float), reported_vel

    def _estimate_velocity(self, target_lla: np.ndarray, now_sec: float) -> np.ndarray | None:
        if self._ref_frame is None:
            return None
        if self._prev_target_lla is None or self._prev_target_update_time is None:
            return None

        dt_obs = now_sec - self._prev_target_update_time
        if dt_obs <= 1e-3:
            return None

        prev_ned = self._ref_frame.lla_to_ned(self._prev_target_lla)
        curr_ned = self._ref_frame.lla_to_ned(target_lla)
        return (curr_ned - prev_ned) / dt_obs

    def _select_velocity(
        self,
        reported_vel: np.ndarray | None,
        estimated_vel: np.ndarray | None,
    ) -> np.ndarray | None:

        if self._velocity_source == 'reported':
            return reported_vel
        if self._velocity_source == 'estimated':
            return estimated_vel
        return reported_vel if reported_vel is not None else estimated_vel

    def _target_state_callback(self, msg: String):
        now_sec = self._now_sec()
        target_lla, reported_vel = self._parse_target_state(msg.data)
        if target_lla is None:
            self._warn_throttled(
                'parse',
                'Target state does not contain valid lat/lon/alt fields.',
            )
            return

        estimated_vel = self._estimate_velocity(target_lla, now_sec)
        target_vel_ned = self._select_velocity(reported_vel, estimated_vel)

        self._prev_target_lla = target_lla.copy()
        self._prev_target_update_time = now_sec
        self._target_lla = target_lla
        self._last_target_update_time = now_sec
        self._target_vel_ned = target_vel_ned

        if self._target_vel_ned is None:
            self._warn_throttled(
                'velocity',
                "Waiting for target velocity. Provide vn/ve/vd in telemetry or let two position samples arrive.",
            )

    def _publish_measurement(self):
        if self._emulator is None:
            self._warn_throttled('home', 'Waiting for HomePosition before publishing radar measurements.')
            return
        if self._target_lla is None:
            self._warn_throttled('target', 'Waiting for target telemetry.')
            return
        if self._target_vel_ned is None:
            self._warn_throttled('target_vel', 'Waiting for a usable target velocity estimate.')
            return
        if self._last_target_update_time is None:
            return

        age = self._now_sec() - self._last_target_update_time
        if age > self._max_state_age:
            self._warn_throttled(
                'stale',
                f"Skipping publish because target state is stale ({age:.2f}s old).",
            )
            return

        try:
            pos_ned0, vel_ned0, R_pos, R_vel = self._emulator.emulate_measurement(
                self._target_lla,
                self._target_vel_ned,
            )
        except ValueError as exc:
            self._warn_throttled('emulate', f"Radar emulation skipped: {exc}")
            return

        # print(f"velocity source: {self._velocity_source}, reported_vel: {self._target_vel_ned}, estimated_vel: {vel_ned0}")

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.child_frame_id = self._child_frame_id

        msg.pose.pose.position.x = float(pos_ned0[0])
        msg.pose.pose.position.y = float(pos_ned0[1])
        msg.pose.pose.position.z = float(pos_ned0[2])
        msg.pose.pose.orientation.w = 1.0 # No orientation information from radar

        msg.twist.twist.linear.x = float(vel_ned0[0])
        msg.twist.twist.linear.y = float(vel_ned0[1])
        msg.twist.twist.linear.z = float(vel_ned0[2])


        pose_cov = np.zeros((6, 6), dtype=float)
        twist_cov = np.zeros((6, 6), dtype=float)
        pose_cov[:3, :3] = R_pos
        twist_cov[:3, :3] = R_vel
        pose_cov[3:, 3:] = np.eye(3) * 1.0e9  # Large covariance for orientation (not estimated)
        twist_cov[3:, 3:] = np.eye(3) * 1.0e9 # Large covariance for angular velocity (not estimated)


        msg.pose.covariance = pose_cov.flatten().tolist()
        msg.twist.covariance = twist_cov.flatten().tolist()

        self.get_logger().info(
            f"Publishing radar measurement: pos_ned0={pos_ned0}, vel_ned0={vel_ned0}"
        )
        

        self._odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RadarEmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
