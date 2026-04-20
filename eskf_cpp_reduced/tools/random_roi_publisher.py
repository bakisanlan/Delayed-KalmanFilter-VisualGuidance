#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest
import random
import math

class RandomROIPublisher(Node):
    def __init__(self):
        super().__init__('random_roi_publisher')
        
        # Publisher for ROI
        # You can change the topic name here
        self.roi_pub = self.create_publisher(RegionOfInterest, '/interceptor/camera/inference_roi', 10)
        
        # Publish at 10 Hz
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.roi_distance_threshold = 40.0
        
        self.get_logger().info('Random ROI Publisher started on topic /eskf_reduced/roi')

    def timer_callback(self):
        # Simulate pbar covariance (sigma_x, sigma_y)
        # Using a single sigma so width and height are the same
        sigma = random.uniform(0.01, 0.15)
        sigma_x = sigma
        sigma_y = sigma
        
        # Simulate pbar position as normalized coordinates
        pbar_x = random.uniform(-1.0, 1.0)
        pbar_y = random.uniform(-1.0, 1.0)
        
        # Camera intrinsics parameters
        fx = 1000.0
        fy = 1000.0
        cx = 320.0
        cy = 240.0
        
        # Calculate normalized bounds
        min_x_norm = pbar_x - 3.0 * sigma_x
        min_y_norm = pbar_y - 3.0 * sigma_y
        width_norm = 6.0 * sigma_x
        height_norm = 6.0 * sigma_y
        
        # Convert to pixel bounds (image coordinates)
        min_x = min_x_norm * fx + cx
        min_y = min_y_norm * fy + cy
        width = width_norm * fx
        height = height_norm * fy
        
        # Simulate a distance between interceptor and target
        distance = random.uniform(10.0, 100.0)

        # Create and populate the message
        roi_msg = RegionOfInterest()
        roi_msg.x_offset = int(max(0.0, round(min_x)))
        roi_msg.y_offset = int(max(0.0, round(min_y)))
        roi_msg.width = int(max(0.0, round(width)))
        roi_msg.height = int(max(0.0, round(height)))
        
        # The boolean parameter for whether ROI will be used or not
        roi_msg.do_rectify = not (distance < self.roi_distance_threshold)

        self.roi_pub.publish(roi_msg)
        
        # Log occasionally for debugging
        if random.random() < 0.1:
            self.get_logger().info(f'Published ROI: dist={distance:.1f}m, do_rectify={roi_msg.do_rectify}, width={roi_msg.width}, height={roi_msg.height}')

def main(args=None):
    rclpy.init(args=args)
    
    node = RandomROIPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
