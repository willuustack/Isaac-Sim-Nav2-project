#!/usr/bin/env python3
"""
Utility functions for parsing and validating navigation goals
"""

import yaml
import math
from typing import List, Dict, Tuple, Optional
from geometry_msgs.msg import Quaternion


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles to quaternion (x, y, z, w)"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return (x, y, z, w)


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return (roll, pitch, yaw)


def validate_goal(goal: Dict) -> Tuple[bool, str]:
    """Validate a single goal dictionary"""
    required = ['x', 'y']
    
    for field in required:
        if field not in goal:
            return False, f"Missing required field: {field}"
        if not isinstance(goal[field], (int, float)):
            return False, f"Field {field} must be numeric"
    
    # Validate orientation if provided
    if 'qx' in goal or 'qy' in goal or 'qz' in goal or 'qw' in goal:
        orientation_fields = ['qx', 'qy', 'qz', 'qw']
        for field in orientation_fields:
            if field not in goal:
                return False, f"Partial orientation specified, missing: {field}"
    
    return True, "Valid"


def load_and_validate_goals(file_path: str) -> Tuple[List[Dict], List[str]]:
    """Load goals from YAML and validate them"""
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
    except Exception as e:
        return [], [f"Failed to load file: {e}"]
    
    goals = data.get('goals', [])
    errors = []
    valid_goals = []
    
    for i, goal in enumerate(goals):
        is_valid, msg = validate_goal(goal)
        if is_valid:
            valid_goals.append(goal)
        else:
            errors.append(f"Goal {i}: {msg}")
    
    return valid_goals, errors


def calculate_path_length(goals: List[Dict]) -> float:
    """Calculate total path length through all goals"""
    if len(goals) < 2:
        return 0.0
    
    total = 0.0
    for i in range(len(goals) - 1):
        dx = goals[i+1]['x'] - goals[i]['x']
        dy = goals[i+1]['y'] - goals[i]['y']
        total += math.sqrt(dx*dx + dy*dy)
    
    return total


def interpolate_goals(goals: List[Dict], num_points: int = 10) -> List[Dict]:
    """Interpolate additional waypoints between goals for smoother navigation"""
    if len(goals) < 2 or num_points < 2:
        return goals
    
    interpolated = []
    
    for i in range(len(goals) - 1):
        start = goals[i]
        end = goals[i+1]
        
        for j in range(num_points):
            t = j / num_points
            x = start['x'] + t * (end['x'] - start['x'])
            y = start['y'] + t * (end['y'] - start['y'])
            
            interpolated.append({
                'x': x,
                'y': y,
                'qw': 1.0,
                'description': f"Interpolated: {start.get('description', 'unknown')} -> {end.get('description', 'unknown')}"
            })
    
    interpolated.append(goals[-1])
    return interpolated
