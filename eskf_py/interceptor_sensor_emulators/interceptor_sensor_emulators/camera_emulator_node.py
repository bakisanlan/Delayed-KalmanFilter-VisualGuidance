"""
ROS2 camera emulator node.

Subscribes to interceptor/target LLA and interceptor local odometry, then
publishes the target's projected pixel coordinates at a configurable rate.
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry

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
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('visualize_target', False)

        # Topic names (easy to remap later)
        self.declare_parameter('interceptor_lla_topic',
                               '/interceptor/global_position/global')
        self.declare_parameter('target_lla_topic',
                               '/target/global_position/global')
        self.declare_parameter('interceptor_local_odom_topic',
                               '/interceptor/global_position/local')
        self.declare_parameter('target_pixel_topic',
                               '/interceptor/camera/target_pixel')
        self.declare_parameter('eskf_pbar_topic',
                               '/eskf_reduced/pbar')

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
        rate_hz = self.get_parameter('rate_hz').value
        self.visualize_target = self.get_parameter('visualize_target').value

        interceptor_lla_topic = self.get_parameter('interceptor_lla_topic').value
        target_lla_topic = self.get_parameter('target_lla_topic').value
        interceptor_local_odom_topic = self.get_parameter('interceptor_local_odom_topic').value
        target_pixel_topic = self.get_parameter('target_pixel_topic').value
        eskf_pbar_topic = self.get_parameter('eskf_pbar_topic').value

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

        self.get_logger().info(
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

        self.get_logger().info(
            f'Subscribed to:\n'
            f'  interceptor LLA : {interceptor_lla_topic}\n'
            f'  target LLA      : {target_lla_topic}\n'
            f'  interceptor odom: {interceptor_local_odom_topic}\n'
            f'  ESKF pbar       : {eskf_pbar_topic}'
        )

        # ------------------------------------------------------------------ #
        # Publisher
        # ------------------------------------------------------------------ #
        self.pixel_pub = self.create_publisher(
            PointStamped, target_pixel_topic, 10)
        self.get_logger().info(f'Publishing target pbar on: {target_pixel_topic}')

        # ------------------------------------------------------------------ #
        # Timer
        # ------------------------------------------------------------------ #
        period = 1.0 / rate_hz
        self.create_timer(period, self._timer_cb)
        self.get_logger().info(f'Timer running at {rate_hz} Hz')

    # ===================================================================== #
    # Callbacks
    # ===================================================================== #

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
        result = measure(
            camera=self.camera,
            interceptor_lla=self._interceptor_lla,
            target_lla=self._target_lla,
            R_body2ned=self._R_body2ned,
        )

        if result is None:
            # Target not in FOV – do not publish
            return

        pbar_x, pbar_y = result

        # Publish
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.point.x = pbar_x
        msg.point.y = pbar_y
        msg.point.z = 0.0  # unused, reserved for future (e.g. confidence)
        self.pixel_pub.publish(msg)

        if self.visualize_target:


            img = np.zeros((self.camera.image_height, self.camera.image_width, 3), dtype=np.uint8)
            
            # Map normalized coordinates back to pixels for OpenCV drawing
            u_pixel = int(pbar_x * self.camera.fx + self.camera.cx)
            v_pixel = int(pbar_y * self.camera.fy + self.camera.cy)
            center = (u_pixel, v_pixel)
            
            cv2.circle(img, center, 5, (0, 0, 255), -1) # Red circle
            cv2.line(img, (center[0] - 15, center[1]), (center[0] + 15, center[1]), (0, 255, 0), 1) # Green crosshair
            cv2.line(img, (center[0], center[1] - 15), (center[0], center[1] + 15), (0, 255, 0), 1)
            
            # Plot Estimated pbar from ESKF
            if self._eskf_pbar is not None:
                est_pbar_x, est_pbar_y = self._eskf_pbar
                est_u_pixel = int(est_pbar_x * self.camera.fx + self.camera.cx)
                est_v_pixel = int(est_pbar_y * self.camera.fy + self.camera.cy)
                est_center = (est_u_pixel, est_v_pixel)
                
                cv2.circle(img, est_center, 5, (255, 0, 0), -1) # Blue circle
                cv2.line(img, (est_center[0] - 15, est_center[1]), (est_center[0] + 15, est_center[1]), (255, 255, 0), 1) # Cyan crosshair
                
            cv2.imshow("Camera Emulator Target", img)
            cv2.waitKey(1)


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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
