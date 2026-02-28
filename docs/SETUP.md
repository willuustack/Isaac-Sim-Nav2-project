# Isaac Sim Nav2 Setup Guide

## Prerequisites

### System Requirements

- **OS**: Ubuntu 22.04 LTS
- **GPU**: NVIDIA GPU with 8GB+ VRAM (RTX 3070 or better recommended)
- **RAM**: 32GB+ recommended
- **Storage**: 50GB+ free space

### Software Requirements

1. **NVIDIA Isaac Sim** 4.0 or later
   - Download from [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
   - Install via Omniverse Launcher

2. **ROS2 Humble Hawksbill**
   ```bash
   # Follow official installation
   # https://docs.ros.org/en/humble/Installation.html
   ```

3. **Nav2 Packages**
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

## Installation Steps

### 1. Clone and Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/isaac_sim_nav2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select isaac_sim_nav2 --symlink-install
source install/setup.bash
```

### 2. Isaac Sim Configuration

#### Enable ROS2 Bridge

1. Launch Isaac Sim
2. Go to `Window → Extensions`
3. Search for "ROS2"
4. Enable the following:
   - `omni.isaac.ros2_bridge`
   - `omni.isaac.ros2_bridge_ui`

#### Verify ROS2 Environment

In Isaac Sim's Script Editor, run:
```python
import os
print(os.environ.get('ROS_DISTRO'))
# Should print: humble
```

### 3. Robot Configuration

#### Action Graph Setup

Create the following Action Graphs in Isaac Sim:

**Odometry Graph:**
```
[On Playback Tick] → [Isaac Compute Odometry]
                           ↓ (Chassis Prim: /World/robot/base_link)
[ROS2 Publish Odometry] → /odom topic
                           ↓
[ROS2 Publish Raw Transform Tree] → odom -> base_link
```

**TF Tree Graph:**
```
[On Playback Tick] → [ROS2 Publish Transform Tree]
                           ↓ (Target Prim: /World/robot, Parent Prim: /World/robot/base_link)
[TF Publisher] → base_link -> wheels/sensors
```

**RTX LiDAR Graph:**
```
[On Playback Tick] → [Isaac Create RTX Lidar]
                           ↓ (config_file: sick_hesai.json or similar)
[Isaac RTX Lidar Helper] → [ROS2 Publish LaserScan] → /scan topic
                        → [ROS2 Publish PointCloud2] → /pointcloud topic
```

## Verification

### Test Topic Publishing

```bash
# In a terminal, check if topics are published
ros2 topic list

# Expected output:
# /scan
# /odom
# /tf
# /tf_static
# /cmd_vel
```

### Test TF Tree

```bash
ros2 run tf2_tools view_frames
# Open frames.pdf to verify the TF tree
```

### Echo Sensor Data

```bash
# Check LiDAR data
ros2 topic echo /scan --once

# Check odometry
ros2 topic echo /odom --once
```

## Common Setup Issues

### Issue: ROS2 topics not appearing

**Solution:**
1. Verify ROS2 bridge is enabled in Isaac Sim
2. Check that `ROS_DOMAIN_ID` matches between Isaac Sim and your terminal
3. Ensure `RMW_IMPLEMENTATION` is consistent

### Issue: TF tree broken

**Solution:**
1. Check that `odom` frame is published
2. Verify `base_link` exists in robot hierarchy
3. Ensure `map` -> `odom` -> `base_link` chain is complete

### Issue: Slow performance

**Solution:**
1. Reduce LiDAR resolution in Isaac Sim
2. Lower physics simulation rate
3. Disable unnecessary sensors

## Next Steps

Once setup is complete, proceed to:
- [Running Navigation](README.md#running-navigation)
- [Configuration Tuning](README.md#configuration-tuning)
- [Testing & Validation](README.md#testing--validation)
