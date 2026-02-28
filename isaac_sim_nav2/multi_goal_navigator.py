#!/usr/bin/env python3
"""
isaac_sim_multi_goal_nav.py
Multi-goal navigation node for ROS2 Nav2 in Isaac Sim
Supports rosbag replay testing and dynamic obstacle handling
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import yaml
import time
from typing import List, Dict, Optional
from datetime import datetime
import os


class IsaacSimMultiGoalNavigator(Node):
    def __init__(self):
        super().__init__('isaac_sim_multi_goal_navigator')
        
        # Parameters
        self.declare_parameter('goal_file', '/tmp/goals.yaml')
        self.declare_parameter('iteration_count', 1)
        self.declare_parameter('obstacle_timeout', 30.0)  # seconds to wait for obstacle clear
        self.declare_parameter('record_bag', False)
        self.declare_parameter('bag_path', '/tmp/nav2_test')
        
        self.goal_file = self.get_parameter('goal_file').value
        self.iteration_count = self.get_parameter('iteration_count').value
        self.obstacle_timeout = self.get_parameter('obstacle_timeout').value
        self.record_bag = self.get_parameter('record_bag').value
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers/Subscribers for monitoring
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        
        # State tracking
        self.current_goal_idx = 0
        self.goals = self.load_goals()
        self.current_iteration = 0
        self.failures = []
        self.start_time = None
        self.current_pose = None
        
        self.get_logger().info(f"Loaded {len(self.goals)} goals from {self.goal_file}")
        
    def load_goals(self) -> List[Dict]:
        """Load waypoints from YAML file"""
        try:
            with open(self.goal_file, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('goals', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load goals: {e}")
            return []
    
    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
    
    def set_initial_pose(self, x: float, y: float, yaw: float):
        """Publish initial pose to AMCL for localization reset"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        # Simplified quaternion - use proper conversion in production
        msg.pose.pose.orientation.z = yaw
        msg.pose.pose.orientation.w = 1.0
        
        # High covariance for initial uncertainty
        msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
        
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f"Set initial pose: ({x}, {y})")
        time.sleep(1.0)  # Wait for AMCL to process
    
    def send_goal(self, goal_pose: Dict) -> Optional:
        """Send navigation goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available")
            return None
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_pose['x']
        goal_msg.pose.pose.position.y = goal_pose['y']
        goal_msg.pose.pose.orientation.z = goal_pose.get('qz', 0.0)
        goal_msg.pose.pose.orientation.w = goal_pose.get('qw', 1.0)
        
        # Behavior tree override for dynamic obstacle handling
        if 'behavior_tree' in goal_pose:
            goal_msg.behavior_tree = goal_pose['behavior_tree']
        
        self.get_logger().info(f"Sending goal {self.current_goal_idx}: ({goal_pose['x']}, {goal_pose['y']})")
        self.start_time = time.time()
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.log_failure('Goal rejected by action server')
            self.process_next_goal()
            return
            
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        elapsed = time.time() - self.start_time
        
        # Check result
        if hasattr(result, 'error_code') and result.error_code != 0:
            self.get_logger().error(f"Navigation failed with error code: {result.error_code}")
            self.log_failure(f'Navigation error code: {result.error_code}')
        else:
            self.get_logger().info(f"Goal reached in {elapsed:.2f}s")
            self.goal_reached_pub.publish(Bool(data=True))
            
        self.process_next_goal()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Log estimated time remaining or distance
        if hasattr(feedback, 'estimated_time_remaining'):
            remaining = feedback.estimated_time_remaining.sec
            self.get_logger().debug(f"ETA: {remaining}s")
    
    def log_failure(self, reason: str):
        """Document failure modes for tuning analysis"""
        failure = {
            'timestamp': datetime.now().isoformat(),
            'goal_idx': self.current_goal_idx,
            'goal': self.goals[self.current_goal_idx] if self.current_goal_idx < len(self.goals) else None,
            'reason': reason,
            'pose': {
                'x': self.current_pose.position.x if self.current_pose else None,
                'y': self.current_pose.position.y if self.current_pose else None
            }
        }
        self.failures.append(failure)
        self.get_logger().error(f"Failure logged: {reason}")
    
    def process_next_goal(self):
        """Move to next goal or iteration"""
        self.current_goal_idx += 1
        
        if self.current_goal_idx >= len(self.goals):
            self.current_iteration += 1
            if self.current_iteration >= self.iteration_count:
                self.get_logger().info("All iterations complete")
                self.save_report()
                return
            self.current_goal_idx = 0
            self.get_logger().info(f"Starting iteration {self.current_iteration + 1}")
        
        # Delay between goals for clean rosbag separation
        time.sleep(2.0)
        self.send_goal(self.goals[self.current_goal_idx])
    
    def save_report(self):
        """Save failure analysis report"""
        report = {
            'test_date': datetime.now().isoformat(),
            'total_goals': len(self.goals) * self.iteration_count,
            'failures': self.failures,
            'failure_rate': len(self.failures) / (len(self.goals) * self.iteration_count) if self.goals else 0
        }
        
        report_path = os.path.join(self.bag_path, 'failure_report.yaml')
        os.makedirs(self.bag_path, exist_ok=True)
        with open(report_path, 'w') as f:
            yaml.dump(report, f, default_flow_style=False)
        self.get_logger().info(f"Report saved to {report_path}")
    
    def start_navigation(self):
        """Begin navigation sequence"""
        if not self.goals:
            self.get_logger().error("No goals loaded")
            return
            
        # Set initial pose if provided
        if 'initial_pose' in self.goals[0]:
            init = self.goals[0]['initial_pose']
            self.set_initial_pose(init['x'], init['y'], init['yaw'])
        
        self.send_goal(self.goals[0])


def main(args=None):
    rclpy.init(args=args)
    navigator = IsaacSimMultiGoalNavigator()
    
    # Wait for navigation stack to be ready
    navigator.get_logger().info("Waiting 5s for Nav2 stack to initialize...")
    time.sleep(5.0)
    
    navigator.start_navigation()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.save_report()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
