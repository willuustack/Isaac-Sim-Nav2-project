# Isaac Sim Autonomous Navigation Bot

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-4.2-green.svg)](https://developer.nvidia.com/isaac-sim)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ROS2 Nav2 autonomous navigation stack implementation in NVIDIA Isaac Sim with validation in cluttered environments.

## Features

- **Full Nav2 Integration**: AMCL + DWB Local Planner + NavFn Global Planner
- **Multi-Goal Navigation**: Sequential waypoint execution with Python API
- **Automated Testing**: Rosbag recording/playback for regression testing
- **Cluttered Environment Validation**: Tuned for complex warehouse scenes
- **Failure Analysis**: Comprehensive logging and reporting tools

## Quick Start

### Prerequisites

- **NVIDIA Isaac Sim** 4.0+ (tested on 4.2)
- **ROS2 Humble** (native or via Isaac Sim ROS2 bridge)
- **Ubuntu** 22.04 (recommended)

### Installation

```bash
# Clone repository
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/isaac_sim_nav2.git

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
cd ~/ros2_ws
colcon build --packages-select isaac_sim_nav2 --symlink-install

# Source
source install/setup.bash
```

### Isaac Sim Setup

1. Enable ROS2 bridge in Isaac Sim:
   - `Window → Extensions → Search "ROS2" → Enable`
2. Verify ROS2 environment matches your system (Humble)
3. Load the warehouse scene:
   ```bash
   # In Isaac Sim
   File → Open → Select worlds/warehouse_cluttered.usda
   ```
4. Verify TF and sensor topics:
   ```bash
   ros2 topic list
   # Should see: /scan, /odom, /tf, /tf_static
   ```

### Running Navigation

```bash
# Terminal 1: Launch Nav2 stack
ros2 launch isaac_sim_nav2 isaac_nav2.launch.py

# Terminal 2: Run multi-goal test (optional)
ros2 run isaac_sim_nav2 multi_goal_nav \
  --ros-args -p goal_file:=install/isaac_sim_nav2/share/isaac_sim_nav2/tests/test_goals.yaml
```

## Project Structure

```
isaac_sim_nav2/
├── config/              # Nav2 parameters tuned for Isaac Sim
├── isaac_sim_nav2/      # Python nodes
│   ├── multi_goal_navigator.py
│   ├── bag_recorder.py
│   └── utils/
├── launch/              # ROS2 launch files
├── maps/                # Occupancy grid maps
├── scripts/             # Helper bash scripts
├── tests/               # Goal configurations and test data
└── worlds/              # Isaac Sim USD stages
```

## Configuration Tuning

Key Isaac Sim-specific adjustments in `config/nav2_params_isaac.yaml`:

| Parameter | Default Value | Reason |
|-----------|---------------|--------|
| `transform_tolerance` | 1.0 | Handles Isaac Sim TF latency |
| `movement_time_allowance` | 20.0 | Generous timeouts for cluttered envs |
| `update_frequency` | 5.0 | Balances performance/fidelity |
| `voxel_layer.z_voxels` | 16 | 3D obstacle detection |

## Testing & Validation

### Record Test Session

```bash
./scripts/record_test.sh
```

### Replay for Regression

```bash
./scripts/replay_test.sh path/to/bag
```

### Dynamic Obstacles

```bash
# Spawn moving obstacles programmatically
python3 scripts/spawn_obstacles.py --count 5 --speed 0.5
```

## Troubleshooting

See `docs/TROUBLESHOOTING.md` for common issues:

- **TF timeouts**: Increase `transform_tolerance`
- **Localization jumps**: Check `/scan` topic rate (should be ~10Hz)
- **Oscillation in narrow corridors**: Reduce `PathAlign.scale`

## Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

## License

Distributed under the MIT License. See `LICENSE` for more information.

## Acknowledgments

- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)
- [ROS2 Navigation Stack](https://navigation.ros.org/)
- Isaac ROS community
