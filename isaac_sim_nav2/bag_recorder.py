#!/usr/bin/env python3
"""
Rosbag recording automation for Isaac Sim Nav2 testing
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import rosbag2_py
from datetime import datetime
import os
import signal
import sys


class BagRecorder(Node):
    def __init__(self):
        super().__init__('isaac_bag_recorder')
        
        self.declare_parameter('output_dir', '/tmp/isaac_nav_bags')
        self.declare_parameter('topics', [
            '/scan', '/odom', '/tf', '/tf_static', '/cmd_vel',
            '/goal_pose', '/amcl_pose',
            '/local_costmap/costmap', '/global_costmap/costmap',
            '/navigate_to_pose/_action/feedback',
            '/navigate_to_pose/_action/status'
        ])
        
        self.output_dir = self.get_parameter('output_dir').value
        self.topics = self.get_parameter('topics').value
        
        # Create output directory
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.bag_path = os.path.join(self.output_dir, f'test_session_{timestamp}')
        os.makedirs(self.bag_path, exist_ok=True)
        
        # Setup bag writer
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=self.bag_path,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        
        # Topic type mapping
        self.topic_types = {}
        self.subscribers = []
        
        # Setup subscriptions
        for topic in self.topics:
            self.create_subscription_for_topic(topic)
        
        self.get_logger().info(f"Recording to: {self.bag_path}")
        self.get_logger().info(f"Topics: {self.topics}")
        
        # Handle shutdown gracefully
        signal.signal(signal.SIGINT, self.shutdown_handler)
    
    def create_subscription_for_topic(self, topic):
        """Create subscription and register topic type"""
        # This is a simplified version - full implementation would use topic type introspection
        sub = self.create_subscription(
            String,  # Placeholder - actual implementation needs proper type
            topic,
            lambda msg, t=topic: self.topic_callback(msg, t),
            10
        )
        self.subscribers.append(sub)
        
        topic_info = rosbag2_py._storage.TopicMetadata(
            name=topic,
            type='std_msgs/msg/String',  # Placeholder
            serialization_format='cdr'
        )
        self.writer.create_topic(topic_info)
    
    def topic_callback(self, msg, topic_name):
        """Write message to bag"""
        self.writer.write(
            topic_name,
            serialize_message(msg),
            self.get_clock().now().nanoseconds
        )
    
    def shutdown_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        self.get_logger().info("Shutting down recorder...")
        self.writer.close()
        self.get_logger().info(f"Bag saved to: {self.bag_path}")
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    recorder = BagRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.writer.close()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
