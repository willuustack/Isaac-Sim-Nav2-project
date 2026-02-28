# Isaac Sim Nav2 Troubleshooting Guide

## Common Failure Modes

### 1. TF Timeouts

**Symptoms:**
- AMCL warnings about transform timeouts
- Robot pose jumps or drifts
- RViz shows "Transform [map] -> [base_link] does not exist"

**Root Causes:**
- Isaac Sim physics timestep drift
- Network latency in ROS2 bridge
- High computational load

**Solutions:**
```yaml
# In nav2_params_isaac.yaml
amcl:
  ros__parameters:
    transform_tolerance: 1.0  # Increase from default 0.1
    update_min_d: 0.25        # Reduce update frequency
    update_min_a: 0.2
```

### 2. Localization Jumps

**Symptoms:**
- AMCL pose suddenly jumps to wrong location
- Particle cloud disperses
- Robot appears to teleport in RViz

**Root Causes:**
- LiDAR dropouts
- Incorrect odometry
- AMCL parameters too aggressive

**Solutions:**
```yaml
amcl:
  ros__parameters:
    min_particles: 1000       # Increase particle count
    max_particles: 5000
    alpha1: 0.2               # Increase motion model noise
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
```

**Debug Commands:**
```bash
# Check LiDAR rate
ros2 topic hz /scan

# Check odometry consistency
ros2 topic echo /odom
```

### 3. Oscillation in Narrow Corridors

**Symptoms:**
- Robot oscillates left/right in corridors
- Path planning creates zigzag patterns
- Controller overshoots

**Root Causes:**
- DWB critic weights unbalanced
- Path alignment too aggressive
- Goal alignment interfering

**Solutions:**
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      PathAlign.scale: 16.0      # Reduce from 32.0
      PathAlign.forward_point_distance: 0.5  # Increase lookahead
      GoalAlign.scale: 12.0      # Reduce from 24.0
      vx_samples: 10             # Reduce for smoother trajectories
```

### 4. Costmap Not Updating

**Symptoms:**
- Obstacles don't appear in costmap
- Robot collides with visible obstacles
- Costmap shows stale data

**Root Causes:**
- LiDAR topic not subscribed
- `inf_is_valid` setting incorrect
- Update frequency too low

**Solutions:**
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      voxel_layer:
        scan:
          topic: /scan
          inf_is_valid: false    # Set based on your LiDAR
          clearing: true
          marking: true
      update_frequency: 5.0      # Ensure reasonable rate
```

### 5. Navigation Fails to Start

**Symptoms:**
- Nav2 nodes crash on startup
- Action server not available
- Parameters fail to load

**Root Causes:**
- Missing dependencies
- Parameter file syntax errors
- Map file not found

**Solutions:**
```bash
# Verify all packages installed
rosdep install --from-paths src --ignore-src -r -y

# Check parameter file syntax
python3 -c "import yaml; yaml.safe_load(open('config/nav2_params_isaac.yaml'))"

# Verify map file exists
ls -la install/isaac_sim_nav2/share/isaac_sim_nav2/maps/
```

### 6. Robot Stuck in Recovery

**Symptoms:**
- Continuous spin/backup cycles
- Never reaches goal
- Recovery behaviors loop

**Root Causes:**
- Goal inside obstacle
- Costmap inflation too large
- Recovery timeout too short

**Solutions:**
```yaml
behavior_server:
  ros__parameters:
    simulate_ahead_time: 3.0     # Increase simulation time
    
local_costmap:
  local_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 0.5    # Reduce inflation
        cost_scaling_factor: 5.0
```

## Debugging Tools

### RViz Configuration

Use the provided RViz config:
```bash
rviz2 -d install/isaac_sim_nav2/share/isaac_sim_nav2/config/rviz/nav2_isaac_view.rviz
```

Key displays to monitor:
- **AMCL Pose**: Shows localization uncertainty
- **Particle Cloud**: Shows AMCL particle distribution
- **Global/Local Costmap**: Shows obstacle detection
- **Global/Local Plan**: Shows planned paths

### Command Line Tools

```bash
# Monitor TF tree
ros2 run tf2_tools view_frames

# Check topic frequencies
ros2 topic hz /scan /odom /tf

# Echo action feedback
ros2 topic echo /navigate_to_pose/_action/feedback

# Check node status
ros2 node list
ros2 node info /amcl
```

### Groot Behavior Tree Monitoring

Enable Groot monitoring in parameters:
```yaml
bt_navigator:
  ros__parameters:
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
```

Connect with Groot to visualize behavior tree execution.

## Performance Optimization

### Reduce CPU Load

```yaml
# Lower controller frequency
controller_server:
  ros__parameters:
    controller_frequency: 10.0  # Down from 20.0

# Reduce costmap resolution
local_costmap:
  local_costmap:
    ros__parameters:
      resolution: 0.1  # Up from 0.05
```

### Isaac Sim Specific

1. Reduce LiDAR rays in Isaac Sim
2. Lower physics simulation rate
3. Disable unnecessary sensors
4. Use simplified collision meshes

## Logging and Diagnostics

### Enable Debug Logging

```bash
# Set log level for specific nodes
ros2 service call /amcl/set_logger_level rcl_interfaces/srv/SetLoggerLevel \
  "{logger_name: 'amcl', level: DEBUG}"
```

### Record Diagnostic Bag

```bash
ros2 bag record /diagnostics /diagnostics_agg \
  /local_costmap/costmap /global_costmap/costmap \
  /scan /odom /amcl_pose
```

## Getting Help

If issues persist:

1. Check [ROS Answers](https://answers.ros.org/)
2. Visit [Nav2 Documentation](https://navigation.ros.org/)
3. Post on [Isaac Sim Forums](https://forums.developer.nvidia.com/c/omniverse/isaac-sim)
4. Include the following in bug reports:
   - Isaac Sim version
   - ROS2 distro
   - GPU model
   - Relevant log files
   - Rosbag of failure
