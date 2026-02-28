#!/usr/bin/env python3
"""
spawn_obstacles.py - Dynamic obstacle spawner for Isaac Sim testing
Spawns moving obstacles to test navigation in dynamic environments
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import random
import math
import argparse
import sys


class DynamicObstacleSimulator(Node):
    """
    Simulates dynamic obstacles by publishing fake odometry messages.
    In a real Isaac Sim setup, this would interface with the USD stage.
    """
    
    def __init__(self, num_obstacles=3, speed=0.3, arena_size=10.0):
        super().__init__('dynamic_obstacle_simulator')
        
        self.num_obstacles = num_obstacles
        self.speed = speed
        self.arena_size = arena_size
        
        # Obstacle state
        self.obstacles = []
        for i in range(num_obstacles):
            self.obstacles.append({
                'id': i,
                'x': random.uniform(-arena_size/2, arena_size/2),
                'y': random.uniform(-arena_size/2, arena_size/2),
                'yaw': random.uniform(0, 2 * math.pi),
                'linear_vel': speed,
                'angular_vel': random.uniform(-0.5, 0.5)
            })
        
        # Publishers for each obstacle
        self.odom_pubs = []
        for i in range(num_obstacles):
            pub = self.create_publisher(Odometry, f'/obstacle_{i}/odom', 10)
            self.odom_pubs.append(pub)
        
        # Timer for obstacle updates
        self.timer = self.create_timer(0.1, self.update_obstacles)  # 10 Hz
        
        self.get_logger().info(f"Spawned {num_obstacles} dynamic obstacles at {speed} m/s")
    
    def update_obstacles(self):
        """Update obstacle positions and publish odometry"""
        for i, obs in enumerate(self.obstacles):
            # Update position
            obs['x'] += obs['linear_vel'] * math.cos(obs['yaw']) * 0.1
            obs['y'] += obs['linear_vel'] * math.sin(obs['yaw']) * 0.1
            obs['yaw'] += obs['angular_vel'] * 0.1
            
            # Boundary check - bounce off walls
            if abs(obs['x']) > self.arena_size / 2:
                obs['yaw'] = math.pi - obs['yaw']
                obs['x'] = max(-self.arena_size/2, min(self.arena_size/2, obs['x']))
            
            if abs(obs['y']) > self.arena_size / 2:
                obs['yaw'] = -obs['yaw']
                obs['y'] = max(-self.arena_size/2, min(self.arena_size/2, obs['y']))
            
            # Publish odometry
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'map'
            odom.child_frame_id = f'obstacle_{i}_base'
            
            odom.pose.pose.position.x = obs['x']
            odom.pose.pose.position.y = obs['y']
            odom.pose.pose.orientation.z = math.sin(obs['yaw'] / 2)
            odom.pose.pose.orientation.w = math.cos(obs['yaw'] / 2)
            
            odom.twist.twist.linear.x = obs['linear_vel']
            odom.twist.twist.angular.z = obs['angular_vel']
            
            self.odom_pubs[i].publish(odom)


def main(args=None):
    parser = argparse.ArgumentParser(description='Spawn dynamic obstacles for testing')
    parser.add_argument('--count', type=int, default=3, help='Number of obstacles')
    parser.add_argument('--speed', type=float, default=0.3, help='Obstacle speed (m/s)')
    parser.add_argument('--arena', type=float, default=10.0, help='Arena size (m)')
    
    # Parse known args to handle ROS2 args
    parsed_args, unknown = parser.parse_known_args()
    
    rclpy.init(args=args)
    
    node = DynamicObstacleSimulator(
        num_obstacles=parsed_args.count,
        speed=parsed_args.speed,
        arena_size=parsed_args.arena
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
