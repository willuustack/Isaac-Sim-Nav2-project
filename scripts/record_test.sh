#!/bin/bash
# record_test.sh - Record ROS2 bag for Isaac Sim Nav2 testing
# Usage: ./record_test.sh [duration_seconds]

SESSION_NAME="isaac_nav_test"
BAG_DIR="/tmp/isaac_nav_bags/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BAG_DIR"

DURATION=${1:-300}  # Default 5 minutes

echo "Starting rosbag recording..."
echo "Output directory: $BAG_DIR"
echo "Duration: ${DURATION}s (Ctrl+C to stop early)"
echo ""

# Record essential topics for replay and debugging
ros2 bag record \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /cmd_vel \
  /goal_pose \
  /initialpose \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/status \
  /navigate_to_pose/_action/results \
  /local_costmap/costmap \
  /global_costmap/costmap \
  /amcl_pose \
  /particlecloud \
  /plan \
  /local_plan \
  -o "$BAG_DIR" &
  
BAG_PID=$!
echo "Recording PID: $BAG_PID"

# Wait for specified duration or until interrupted
sleep "$DURATION"

if kill -0 $BAG_PID 2>/dev/null; then
    echo ""
    echo "Stopping recording..."
    kill $BAG_PID
    wait $BAG_PID 2>/dev/null
fi

echo ""
echo "========================================="
echo "Bag saved to: $BAG_DIR"
echo "========================================="
