"""
Relative Position Publisher Node

This ROS2 node subscribes to global position topics for both interceptor (uas_1) 
and target (uas_2) drones, converts their LLA coordinates to NED using a fixed 
reference point (LLA0), computes the relative position (interceptor_ned - target_ned),
and publishes the result as a Vector3 message.

Topics:
    Subscribed:
        - /mavros/global_position/global (interceptor lat/lon)
        - /mavros/global_position/rel_alt (interceptor altitude)
        - /target/telemetry/state (JSON String with lat, lon, alt for target)
    Published:
        - /radar/pr (geometry_msgs/Vector3) - relative position in NED

Usage:
    ros2 run visual_guidance relative_position_node
    
    # Or with custom parameters:
    ros2 run visual_guidance relative_position_node --ros-args \
        -p publish_rate:=20.0 \
        -p lla0_lat:=41.10007741761694 \
        -p lla0_lon:=29.025102556944116 \
        -p lla0_alt:=0.0
"""

import json
import re
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String


class NEDConverter:
    """
    Converts LLA (Latitude, Longitude, Altitude) to NED (North, East, Down) 
    coordinates relative to a fixed reference point (LLA0).
    """
    
    # WGS84 constants
    WGS84_A = 6378137.0  # Semi-major axis (m)
    WGS84_F = 1 / 298.257223563  # Flattening
    WGS84_E2 = 2 * WGS84_F - WGS84_F ** 2  # Eccentricity squared
    
    def __init__(self, ref_lat: float, ref_lon: float, ref_alt: float = 0.0):
        """Initialize NED converter with a reference point (LLA0)."""
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt = ref_alt
        
        # Pre-compute reference ECEF coordinates
        self._ref_ecef = self._lla_to_ecef(ref_lat, ref_lon, ref_alt)
        
        # Pre-compute rotation matrix from ECEF to NED
        self._R_ecef_to_ned = self._compute_ecef_to_ned_rotation(ref_lat, ref_lon)
    
    def _lla_to_ecef(self, lat: float, lon: float, alt: float) -> np.ndarray:
        """Convert LLA to ECEF coordinates."""
        lat_rad = np.deg2rad(lat)
        lon_rad = np.deg2rad(lon)
        
        N = self.WGS84_A / np.sqrt(1 - self.WGS84_E2 * np.sin(lat_rad) ** 2)
        
        X = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
        Y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
        Z = ((1 - self.WGS84_E2) * N + alt) * np.sin(lat_rad)
        
        return np.array([X, Y, Z])
    
    def _compute_ecef_to_ned_rotation(self, lat: float, lon: float) -> np.ndarray:
        """Compute rotation matrix from ECEF to NED frame."""
        lat_rad = np.deg2rad(lat)
        lon_rad = np.deg2rad(lon)
        
        sin_lat = np.sin(lat_rad)
        cos_lat = np.cos(lat_rad)
        sin_lon = np.sin(lon_rad)
        cos_lon = np.cos(lon_rad)
        
        R = np.array([
            [-sin_lat * cos_lon, -sin_lat * sin_lon,  cos_lat],  # North
            [-sin_lon,            cos_lon,            0      ],  # East
            [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat]   # Down
        ])
        
        return R
    
    def lla_to_ned(self, lat: float, lon: float, alt: float) -> np.ndarray:
        """Convert LLA coordinates to NED relative to reference point."""
        point_ecef = self._lla_to_ecef(lat, lon, alt)
        delta_ecef = point_ecef - self._ref_ecef
        ned = self._R_ecef_to_ned @ delta_ecef
        return ned


class RelativePositionNode(Node):
    """
    ROS2 Node that computes and publishes relative position between 
    interceptor and target in NED coordinates.
    """
    
    def __init__(self):
        super().__init__('relative_position_node')
        
        # Declare parameters with defaults
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('lla0_lat', 39.778354)
        self.declare_parameter('lla0_lon', 32.315964)
        self.declare_parameter('lla0_alt', 0.0)
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        lla0_lat = self.get_parameter('lla0_lat').value
        lla0_lon = self.get_parameter('lla0_lon').value
        lla0_alt = self.get_parameter('lla0_alt').value
        
        # Initialize NED converter with reference point
        self.ned_converter = NEDConverter(lla0_lat, lla0_lon, lla0_alt)
        
        self.get_logger().info(
            f"Initialized NED converter with LLA0: "
            f"lat={lla0_lat:.10f}, lon={lla0_lon:.10f}, alt={lla0_alt:.1f}"
        )
        
        # State storage for interceptor (uas_1)
        self.interceptor_lat = 0.0
        self.interceptor_lon = 0.0
        self.interceptor_alt = 0.0
        self.interceptor_has_gps = False
        self.interceptor_has_alt = False
        
        # State storage for target (from /target/state JSON)
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.target_alt = 0.0
        self.target_has_data = False
        
        # QoS profiles
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=1)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST,depth=10)
        
        # === Subscribers ===
        # Interceptor (uas_1) global position
        self.interceptor_gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self._interceptor_gps_callback,
            sensor_qos
        )
        
        # Interceptor (uas_1) relative altitude
        self.interceptor_alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self._interceptor_alt_callback,
            sensor_qos
        )
        
        # Target state from /target/state (JSON String)
        self.target_state_sub = self.create_subscription(
            String,
            '/target/telemetry/state',
            self._target_state_callback,
            reliable_qos
        )
        
        # === Publisher ===
        self.relative_pos_pub = self.create_publisher(
            Vector3,
            '/radar/pr',
            reliable_qos
        )
        
        # Create timer for periodic publishing
        timer_period = 1.0 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self._publish_relative_position)
        
        self.get_logger().info(
            f"RelativePositionNode started. Publishing at {self.publish_rate} Hz to /radar/pr"
        )
    
    # =========================================================================
    # Callback Methods
    # =========================================================================
    
    def _interceptor_gps_callback(self, msg: NavSatFix):
        """Handle interceptor (uas_1) GPS updates."""
        self.interceptor_lat = msg.latitude
        self.interceptor_lon = msg.longitude
        if abs(msg.latitude) > 0.1 and abs(msg.longitude) > 0.1:
            if not self.interceptor_has_gps:
                self.get_logger().info(f"Interceptor GPS received: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")
            self.interceptor_has_gps = True
    
    def _interceptor_alt_callback(self, msg: Float64):
        """Handle interceptor (uas_1) relative altitude updates."""
        self.interceptor_alt = msg.data
        if not self.interceptor_has_alt:
            self.get_logger().info(f"Interceptor alt received: {msg.data:.2f}m")
        self.interceptor_has_alt = True
    
    # Regex pattern to extract numeric values for lat, lon, alt from corrupted JSON.
    # Matches key names with possible missing quotes/colons and captures the number.
    _FIELD_PATTERNS = {
        'lat': re.compile(r'"?la?t"?\s*:?\s*(-?\d+\.?\d*)'),
        'lon': re.compile(r'"?lo?n"?\s*:?\s*(-?\d+\.?\d*)'),
        'alt': re.compile(r'"?al?t"?\s*:?\s*(-?\d+\.?\d*)'),
    }

    def _parse_with_regex(self, raw_data: str) -> dict:
        """Fallback regex parser for corrupted JSON messages."""
        result = {}
        for key, pattern in self._FIELD_PATTERNS.items():
            match = pattern.search(raw_data)
            if match:
                result[key] = float(match.group(1))
        return result

    def _target_state_callback(self, msg: String):
        """Handle target state updates from JSON message."""
        try:
            raw_data = msg.data
            
            #print(raw_data)
            
            # Try standard JSON parsing first
            try:
                data = json.loads(raw_data)
            except json.JSONDecodeError:
                # Fallback: extract values with regex from corrupted JSON
                data = self._parse_with_regex(raw_data)
                
                #print(data)
                if 'lat' not in data or 'lon' not in data or 'alt' not in data:
                    self.get_logger().warning(
                        f"Could not extract lat/lon/alt from corrupted msg: '{msg.data[:120]}'"
                    )

                    return
                self.get_logger().debug(
                    f"Recovered corrupted JSON via regex: lat={data['lat']}, lon={data['lon']}, alt={data['alt']}"
                )

            self.target_lat = data.get('lat', 0.0)
            self.target_lon = data.get('lon', 0.0)
            self.target_alt = data.get('alt', 0.0)
            if abs(self.target_lat) > 0.1 and abs(self.target_lon) > 0.1:
                if not self.target_has_data:
                    self.get_logger().info(f"Target data received: lat={self.target_lat:.6f}, lon={self.target_lon:.6f}, alt={self.target_alt:.2f}m")
                self.target_has_data = True
        except Exception as e:
            self.get_logger().warning(f"Error processing target state: {e}")
    
    # =========================================================================
    # Publishing Method
    # =========================================================================
    
    def _publish_relative_position(self):
        """Compute and publish relative position (interceptor_ned - target_ned)."""
        # Check if we have valid data from both drones
        if not (self.interceptor_has_gps and self.target_has_data):
            self.get_logger().debug(
                f"Waiting for data: interceptor={self.interceptor_has_gps}, "
                f"target={self.target_has_data}"
            )
            return
        
        # Convert both positions to NED
        interceptor_ned = self.ned_converter.lla_to_ned(
            self.interceptor_lat,
            self.interceptor_lon,
            self.interceptor_alt
        )
        
        target_ned = self.ned_converter.lla_to_ned(
            self.target_lat,
            self.target_lon,
            self.target_alt
        )
        
        # Compute relative position: interceptor_ned - target_ned
        relative_ned = interceptor_ned - target_ned
        
        # Create and publish message
        msg = Vector3()
        msg.x = float(relative_ned[0])  # North
        msg.y = float(relative_ned[1])  # East
        msg.z = float(relative_ned[2])  # Down
        
        self.relative_pos_pub.publish(msg)
        
        # Log periodically (every 50 publishes = ~5 seconds at 10Hz)
        if not hasattr(self, '_publish_count'):
            self._publish_count = 0
        self._publish_count += 1
        
        # if self._publish_count % 1 == 1:
        self.get_logger().info(
            f"Interceptor NED: N={interceptor_ned[0]:.2f}m, "
            f"E={interceptor_ned[1]:.2f}m, D={interceptor_ned[2]:.2f}m"
        )
        self.get_logger().info(
            f"Target NED: N={target_ned[0]:.2f}m, "
            f"E={target_ned[1]:.2f}m, D={target_ned[2]:.2f}m"
        )
        self.get_logger().info(
            f"Relative NED: N={relative_ned[0]:.2f}m, "
            f"E={relative_ned[1]:.2f}m, D={relative_ned[2]:.2f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = RelativePositionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
