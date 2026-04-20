#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
import yaml
import subprocess
import os
import time
from datetime import datetime
import signal
import sys

class RosbagRecorder(Node):
    def __init__(self):
        super().__init__('rosbag_recorder')
        
        self.config_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'eskf_reduced_params.yaml')
        self.log_dir = os.path.join(
            os.path.dirname(__file__), '..', 'log')
        
        self.topics = []
        self.state_topic = "/mavros/state"
        self.load_config()
        
        self.bag_process = None
        self.was_armed = False
        
        self.get_logger().info(f"Subscribing to state topic: {self.state_topic}")
        self.subscription = self.create_subscription(
            State,
            self.state_topic,
            self.state_callback,
            10)
        
        self.start_recording()

    def load_config(self):
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
            if 'topics' in config:
                topics_dict = config['topics']
                self.topics = list(topics_dict.values())
                if 'interceptor_state' in topics_dict:
                    self.state_topic = topics_dict['interceptor_state']

    def get_counter(self):
        counter_file = os.path.join(self.log_dir, 'log_counter.txt')
        counter = "0000"
        if os.path.exists(counter_file):
            with open(counter_file, 'r') as f:
                current_count = int(f.read().strip())
                counter = f"{current_count - 1:04d}"
        return counter

    def start_recording(self):
        counter = self.get_counter()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"{counter}-{timestamp}"
        bag_dir = os.path.join(self.log_dir, 'rosbags', bag_name)
        
        # Ensure rosbags directory exists
        os.makedirs(os.path.join(self.log_dir, 'rosbags'), exist_ok=True)
        
        cmd = ['ros2', 'bag', 'record', '-o', bag_dir] + self.topics
        self.get_logger().info(f"Starting rosbag record: {' '.join(cmd)}")
        self.bag_process = subprocess.Popen(cmd)

    def stop_recording(self):
        if self.bag_process:
            self.get_logger().info("Stopping rosbag record...")
            self.bag_process.send_signal(signal.SIGINT)
            self.bag_process.wait()
            self.get_logger().info("Rosbag record stopped.")
            self.bag_process = None

    def state_callback(self, msg):
        if msg.armed:
            if not self.was_armed:
                self.get_logger().info("Vehicle ARMED.")
                self.was_armed = True
        else:
            if self.was_armed:
                self.get_logger().info("Vehicle DISARMED after being armed. Stopping recording and exiting.")
                self.stop_recording()
                rclpy.shutdown()
                sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
    finally:
        node.stop_recording()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
