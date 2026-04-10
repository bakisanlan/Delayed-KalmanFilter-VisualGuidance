#!/usr/bin/env python3

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import RegionOfInterest
# Add other message types if needed for Est target topics

class VisualizationNode(Node):
    """Visualizes the camera view and 2D target trajectories."""

    def __init__(self):
        super().__init__('visualization_node')

        # ------------------------------------------------------------------ #
        # Declare parameters (with defaults)
        # ------------------------------------------------------------------ #
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fx', 320.0)
        self.declare_parameter('fy', 320.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('visualize_target', True)
        self.declare_parameter('show_2d_trajectory_plot', True)

        # Topic names
        self.declare_parameter('target_pixel_topic', '/interceptor/camera/target_pbar')
        self.declare_parameter('target_pixel_true_topic', '/interceptor/camera/target_pbar_true')
        self.declare_parameter('eskf_pbar_topic', '/eskf_reduced/pbar')
        self.declare_parameter('inference_roi_topic', '/interceptor/camera/inference_roi')
        
        # Estimated target from ESKF (NED frame):
        self.declare_parameter('gps_target_topic', '/target/global_position/local')  
        self.declare_parameter('est_target_topic', '/eskf_reduced/odom')

        # ------------------------------------------------------------------ #
        # Read parameters
        # ------------------------------------------------------------------ #
        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        rate_hz = self.get_parameter('rate_hz').value
        self.visualize_target = self.get_parameter('visualize_target').value
        self.show_2d_trajectory_plot = self.get_parameter('show_2d_trajectory_plot').value

        target_pixel_topic = self.get_parameter('target_pixel_topic').value
        target_pixel_true_topic = self.get_parameter('target_pixel_true_topic').value
        eskf_pbar_topic = self.get_parameter('eskf_pbar_topic').value
        inference_roi_topic = self.get_parameter('inference_roi_topic').value
        gps_target_topic = self.get_parameter('gps_target_topic').value
        est_target_topic = self.get_parameter('est_target_topic').value

        # ------------------------------------------------------------------ #
        # State variables
        # ------------------------------------------------------------------ #
        self._target_pbar_meas = None # (px, py) Imperfect Measurement
        self._target_pbar_true = None # (px, py) Ground truth
        self._eskf_pbar = None    # (px, py) Estimated
        self._inference_roi = None # RegionOfInterest message

        # Trajectory history for 2D plots
        self.gps_trajectory = [] # List of (x, y, z)
        self.est_trajectory = [] # List of (x, y, z)
        self._windows_open = False
        self._2d_plots_opened = False

        # Plot parameters
        self.plot_size = 500
        self.plot_bg_color = (20, 20, 20)  # Dark background

        # ------------------------------------------------------------------ #
        # Subscriptions
        # ------------------------------------------------------------------ #
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(PointStamped, target_pixel_topic, self._target_pixel_meas_cb, sensor_qos)
        self.create_subscription(PointStamped, target_pixel_true_topic, self._target_pixel_true_cb, sensor_qos)
        self.create_subscription(Point, eskf_pbar_topic, self._eskf_pbar_cb, 10)
        self.create_subscription(RegionOfInterest, inference_roi_topic, self._inference_roi_cb, sensor_qos)
        
        self.create_subscription(Odometry, gps_target_topic, self._gps_target_cb, sensor_qos)
        self.create_subscription(Odometry, est_target_topic, self._est_target_cb, sensor_qos)

        self.get_logger().info('Visualization Node Started.')
        
        # ------------------------------------------------------------------ #
        # Timer
        # ------------------------------------------------------------------ #
        period = 1.0 / rate_hz
        self.create_timer(period, self._timer_cb)

    # ===================================================================== #
    # Callbacks
    # ===================================================================== #

    def _target_pixel_meas_cb(self, msg: PointStamped):
        self._target_pbar_meas = (msg.point.x, msg.point.y)

    def _target_pixel_true_cb(self, msg: PointStamped):
        self._target_pbar_true = (msg.point.x, msg.point.y)

    def _eskf_pbar_cb(self, msg: Point):
        self._eskf_pbar = (msg.x, msg.y)

    def _inference_roi_cb(self, msg: RegionOfInterest):
        self._inference_roi = msg

    def _gps_target_cb(self, msg: Odometry):
        # Extract ENU position and convert to NED
        x_enu = msg.pose.pose.position.x
        y_enu = msg.pose.pose.position.y
        z_enu = msg.pose.pose.position.z
        
        # ENU to NED: North=y, East=x, Down=-z
        x_ned = y_enu
        y_ned = x_enu
        z_ned = -z_enu

        self.gps_trajectory.append((x_ned, y_ned, z_ned))
        if len(self.gps_trajectory) > 500:
            self.gps_trajectory.pop(0)

    def _est_target_cb(self, msg: Odometry):
        # Position is already in NED frame
        x_ned = msg.pose.pose.position.x
        y_ned = msg.pose.pose.position.y
        z_ned = msg.pose.pose.position.z
        
        self.est_trajectory.append((x_ned, y_ned, z_ned))
        if len(self.est_trajectory) > 500:
            self.est_trajectory.pop(0)

    def _timer_cb(self):
        self.visualize_target = self.get_parameter('visualize_target').value
        if not self.visualize_target:
            if self._windows_open:
                cv2.destroyAllWindows()
                self._windows_open = False
            return
            
        self._windows_open = True
        self._render_camera_view()
        
        # Optionally show 2D trajectory plot
        self.show_2d_trajectory_plot = self.get_parameter('show_2d_trajectory_plot').value
        if self.show_2d_trajectory_plot:
            self._render_horizontal_plot()
            self._render_vertical_plot()
            self._2d_plots_opened = True
        elif self._2d_plots_opened:
            # If windows might be open from previous runs and toggle is turned off
            try:
                cv2.destroyWindow("Horizontal Plot")
                cv2.destroyWindow("Vertical Plot")
            except cv2.error:
                pass
            self._2d_plots_opened = False
            
        cv2.waitKey(1)

    def _render_camera_view(self):
        img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)

        
        # Plot Imperfect Measured Target
        if self._target_pbar_meas is not None:
            pbar_x, pbar_y = self._target_pbar_meas
            if pbar_x is not None and pbar_y is not None and not np.isnan(pbar_x) and not np.isnan(pbar_y):
                u_pixel = int(pbar_x * self.fx + self.cx)
                v_pixel = int(pbar_y * self.fy + self.cy)
                center = (u_pixel, v_pixel)
                
                cv2.circle(img, center, 6, (0, 255, 255), -1) # Yellow circle

        # Plot Uncertainty Rectangle (Inference ROI)
        if self._inference_roi is not None:
            roi = self._inference_roi
            pt1 = (roi.x_offset, roi.y_offset)
            pt2 = (roi.x_offset + roi.width, roi.y_offset + roi.height)
            cv2.rectangle(img, pt1, pt2, (255, 255, 0), 2)  # Cyan colour (BGR)

        # Plot Perfect Ground Truth Target
        if self._target_pbar_true is not None:
            pbar_x, pbar_y = self._target_pbar_true
            if pbar_x is not None and pbar_y is not None and not np.isnan(pbar_x) and not np.isnan(pbar_y):
                u_pixel = int(pbar_x * self.fx + self.cx)
                v_pixel = int(pbar_y * self.fy + self.cy)
                center = (u_pixel, v_pixel)
                
                cv2.circle(img, center, 5, (0, 0, 255), -1) # Red circle
                cv2.line(img, (center[0] - 15, center[1]), (center[0] + 15, center[1]), (0, 255, 0), 1) # Green crosshair
                cv2.line(img, (center[0], center[1] - 15), (center[0], center[1] + 15), (0, 255, 0), 1)
        
        # Plot Estimated Target from ESKF
        if self._eskf_pbar is not None:
            est_pbar_x, est_pbar_y = self._eskf_pbar
            if est_pbar_x is not None and est_pbar_y is not None and not np.isnan(est_pbar_x) and not np.isnan(est_pbar_y):
                est_u_pixel = int(est_pbar_x * self.fx + self.cx)
                est_v_pixel = int(est_pbar_y * self.fy + self.cy)
                est_center = (est_u_pixel, est_v_pixel)
                
                cv2.circle(img, est_center, 5, (255, 0, 0), -1) # Blue circle
                cv2.line(img, (est_center[0] - 15, est_center[1]), (est_center[0] + 15, est_center[1]), (255, 255, 0), 1) # Cyan crosshair
            
        cv2.imshow("Camera View", img)

    def _compute_scale_offset(self, x_idx, y_idx):
        all_pts = []
        if self.gps_trajectory:
            all_pts.extend(self.gps_trajectory)
        if self.est_trajectory:
            all_pts.extend(self.est_trajectory)
            
        if not all_pts:
            return 3.0, 0.0, 0.0
            
        xs = [p[x_idx] for p in all_pts if not np.isnan(p).any()]
        ys = [p[y_idx] for p in all_pts if not np.isnan(p).any()]
        
        if not xs or not ys:
            return 3.0, 0.0, 0.0
            
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        range_x = max(max_x - min_x, 20.0)
        range_y = max(max_y - min_y, 20.0)
        
        # Fit within 400x400 to leave 50px margins
        scale = min(400.0 / range_x, 400.0 / range_y)
        
        center_x = (max_x + min_x) / 2.0
        center_y = (max_y + min_y) / 2.0
        
        return scale, center_x, center_y

    def _draw_trajectory(self, img, trajectory, color, x_idx, y_idx, scale, cx, cy):
        if not trajectory:
            return
        
        points = []
        offset = self.plot_size / 2.0 # center of 500x500 img
        for p in trajectory:
            if np.isnan(p).any():
                continue
            x_px = int((p[x_idx] - cx) * scale + offset)
            y_px = int(-(p[y_idx] - cy) * scale + offset)
            points.append((x_px, y_px))
            
        if not points:
            return
            
        for i in range(1, len(points)):
            cv2.line(img, points[i-1], points[i], color, 2)
        # Draw current position
        cv2.circle(img, points[-1], 5, color, -1)

    def _render_horizontal_plot(self):
        img = np.full((self.plot_size, self.plot_size, 3), self.plot_bg_color, dtype=np.uint8)
        
        cv2.putText(img, "Horizontal (X-Y) Target Pos", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, "GPS Target", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(img, "Est Target", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        if self.gps_trajectory and self.est_trajectory:
            err_x = self.est_trajectory[-1][0] - self.gps_trajectory[-1][0]
            err_y = self.est_trajectory[-1][1] - self.gps_trajectory[-1][1]
            cv2.putText(img, f"Err X: {err_x:.2f} m", (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(img, f"Err Y: {err_y:.2f} m", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Plot X (idx 0) vs Y (idx 1)
        scale, cx, cy = self._compute_scale_offset(0, 1)
        self._draw_trajectory(img, self.gps_trajectory, (0, 0, 255), 0, 1, scale, cx, cy)
        self._draw_trajectory(img, self.est_trajectory, (255, 0, 0), 0, 1, scale, cx, cy)

        cv2.imshow("Horizontal Plot", img)

    def _render_vertical_plot(self):
        img = np.full((self.plot_size, self.plot_size, 3), self.plot_bg_color, dtype=np.uint8)
        
        cv2.putText(img, "Vertical (X-Z) Target Pos", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, "GPS Target", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(img, "Est Target", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        if self.gps_trajectory and self.est_trajectory:
            err_x = self.est_trajectory[-1][0] - self.gps_trajectory[-1][0]
            err_z = self.est_trajectory[-1][2] - self.gps_trajectory[-1][2]
            cv2.putText(img, f"Err X: {err_x:.2f} m", (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(img, f"Err Z: {err_z:.2f} m", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Plot X (idx 0) vs Z (idx 2)
        scale, cx, cy = self._compute_scale_offset(0, 2)
        self._draw_trajectory(img, self.gps_trajectory, (0, 0, 255), 0, 2, scale, cx, cy)
        self._draw_trajectory(img, self.est_trajectory, (255, 0, 0), 0, 2, scale, cx, cy)

        cv2.imshow("Vertical Plot", img)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
