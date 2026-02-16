# DWB Local Planner for ESP32 Robot

Complete Dynamic Window Approach (DWB) local planner implementation for differential drive robots with ESP32 controller.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Architecture](#architecture)
- [Configuration](#configuration)
- [Usage Examples](#usage-examples)
- [Tuning Guide](#tuning-guide)
- [API Reference](#api-reference)

---

## 🎯 Overview

This is a production-ready DWB local planner that:
- Plans collision-free trajectories in real-time
- Follows global paths while avoiding obstacles
- Adapts to different navigation scenarios with configuration profiles
- Works with ESP32-based differential drive robots
- Supports real-time visualization

## ✨ Features

### Core Capabilities
- ✅ **Dynamic Window Approach**: Plans trajectories within robot's dynamic constraints
- ✅ **Obstacle Avoidance**: Real-time collision checking and avoidance
- ✅ **Path Following**: Follows global paths with smooth local adjustments
- ✅ **Multiple Cost Functions**: Path distance, goal distance, obstacle proximity, velocity, smoothness
- ✅ **Configurable Profiles**: Pre-tuned profiles for different scenarios
- ✅ **Real-time Visualization**: See planning in action (matplotlib)
- ✅ **Simulation Mode**: Test without hardware

### Navigation Profiles
1. **Default**: Balanced performance
2. **Cautious**: High safety margins, slow movement
3. **Aggressive**: Fast goal-reaching
4. **Tight Spaces**: Precision in narrow areas
5. **Open Space**: Fast travel in open areas
6. **Smooth**: Smooth trajectories for carrying objects
7. **Parking**: High-precision final positioning
8. **Recovery**: Escape from stuck situations

---

## 📦 Installation

### 1. Hardware Requirements
- ESP32 board with firmware (provided)
- Differential drive robot with:
  - Two DC motors with encoders
  - MPU6050 IMU
  - Motor driver (H-bridge)

### 2. Software Requirements
```bash
# Python 3.7+
pip install pyserial numpy matplotlib

# Optional: For ROS integration
pip install rospkg rospy
```

### 3. File Structure
```
dwb_navigation/
├── dwb_planner.py          # Core DWB implementation
├── dwb_config.py           # Configuration profiles
├── dwb_visualizer.py       # Real-time visualization
├── dwb_integrated.py       # Complete examples
├── firmware.ino            # ESP32 firmware
└── README.md               # This file
```

---

## 🚀 Quick Start

### Basic Navigation

```python
from dwb_integrated import *

# 1. Connect to robot
robot = ESP32RobotController('/dev/ttyUSB0', 115200)
robot.start()

# 2. Setup environment
scanner = SimulatedLaserScanner()
scanner.add_obstacle(1.0, 0.5, 0.2)  # x, y, radius

# 3. Create planner with profile
config = get_config("default")  # or "cautious", "aggressive", etc.
planner = ConfigurableDWBPlanner(robot, scanner, config)

# 4. Set goal
waypoints = [
    (0.0, 0.0, 0.0),           # Start
    (2.0, 0.0, 0.0),           # Goal
]
path = SimpleGlobalPlanner.plan_path(waypoints)
planner.set_global_path(path)

# 5. Navigate
while not planner.goal_reached:
    cmd_vel = planner.compute_velocity_command()
    robot.send_velocity(cmd_vel)
    time.sleep(0.05)  # 20 Hz

robot.stop()
```

### Run Pre-built Scenarios

```bash
# Simple corridor navigation
python dwb_integrated.py simple_corridor

# Tight spaces with cautious profile
python dwb_integrated.py tight_corridor cautious

# With visualization
python dwb_integrated.py obstacle_field aggressive --viz
```

---

## 🏗️ Architecture

### System Overview

```
┌─────────────────┐
│  Global Planner │ (A* / waypoints)
└────────┬────────┘
         │ path
         ▼
┌─────────────────┐
│  DWB Planner    │ ◄── Laser Scan / Obstacles
│  - Sample vels  │
│  - Simulate     │
│  - Score        │
│  - Select best  │
└────────┬────────┘
         │ cmd_vel
         ▼
┌─────────────────┐
│  ESP32 Robot    │
│  - Motor ctrl   │
│  - Odometry     │
│  - IMU          │
└─────────────────┘
```

### DWB Algorithm Flow

1. **Velocity Sampling**: Generate velocity samples within dynamic window
   - Respects acceleration limits
   - Samples linear and angular velocities

2. **Trajectory Simulation**: Forward simulate each velocity
   - Constant velocity model
   - Configurable time horizon
   - Differential drive kinematics

3. **Collision Checking**: Check each trajectory for collisions
   - Robot radius + safety margin
   - Against all nearby obstacles

4. **Trajectory Scoring**: Score valid trajectories with multiple critics:
   - **Path Distance**: Stay close to global path
   - **Goal Distance**: Progress toward goal
   - **Obstacle Cost**: Maintain safe distance
   - **Velocity Cost**: Prefer higher speeds
   - **Smoothness Cost**: Avoid jerky movements

5. **Command Selection**: Choose lowest-cost trajectory

---

## ⚙️ Configuration

### Using Profiles

```python
from dwb_config import get_config, list_profiles

# List all available profiles
list_profiles()

# Get a specific profile
config = get_config("aggressive")

# Use in planner
planner = ConfigurableDWBPlanner(robot, scanner, config)
```

### Creating Custom Profiles

```python
from dwb_config import create_custom_config

config = create_custom_config(
    max_linear_vel=0.6,
    max_angular_vel=2.5,
    weight_obstacle=20.0,
    weight_smoothness=10.0,
    sim_time=2.5
)
```

### Key Parameters

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `max_linear_vel` | Maximum forward velocity | 0.5 m/s | 0.1-2.0 |
| `max_angular_vel` | Maximum rotation rate | 2.0 rad/s | 0.5-4.0 |
| `sim_time` | Trajectory prediction time | 2.0 s | 1.0-4.0 |
| `weight_path_distance` | Path following weight | 32.0 | 1-100 |
| `weight_goal_distance` | Goal attraction weight | 24.0 | 1-100 |
| `weight_obstacle` | Obstacle avoidance weight | 10.0 | 1-100 |
| `weight_velocity` | Speed preference weight | 1.0 | 0-20 |
| `weight_smoothness` | Smooth motion weight | 5.0 | 0-30 |

---

## 📚 Usage Examples

### Example 1: Simple Point-to-Point

```python
# Navigate to a single point
waypoints = [(0, 0, 0), (1.5, 0.5, 0)]
controller = ScenarioNavigationController(robot, scanner)
scenario = NavigationScenario(
    name="Point to Point",
    waypoints=waypoints,
    obstacles=[(0.7, 0.3, 0.15)],
    recommended_profile="default"
)
controller.run_scenario(scenario)
```

### Example 2: Multi-Waypoint Path

```python
# Navigate through multiple waypoints
waypoints = [
    (0.0, 0.0, 0.0),
    (1.0, 0.0, 0.0),
    (1.0, 1.0, math.pi/2),
    (0.0, 1.0, math.pi),
]

path = SimpleGlobalPlanner.plan_path(waypoints)
planner.set_global_path(path)

# Execute
while not planner.goal_reached:
    cmd = planner.compute_velocity_command()
    robot.send_velocity(cmd)
    time.sleep(0.05)
```

### Example 3: With Real-time Visualization

```python
from dwb_visualizer import DWBVisualizer

# Create visualizer
viz = DWBVisualizer(planner, robot)

# Run in separate thread
import threading
viz_thread = threading.Thread(target=lambda: viz.start(interval=50))
viz_thread.start()

# Continue navigation...
```

### Example 4: Dynamic Obstacle Avoidance

```python
# Add obstacles dynamically
scanner.add_obstacle(1.0, 0.5, 0.2)

# Navigate - planner automatically avoids new obstacles
while not planner.goal_reached:
    # Add dynamic obstacles based on sensor readings
    # sensor_obstacles = get_lidar_obstacles()
    # for obs in sensor_obstacles:
    #     scanner.add_obstacle(obs.x, obs.y, obs.radius)
    
    cmd = planner.compute_velocity_command()
    robot.send_velocity(cmd)
    time.sleep(0.05)
```

---

## 🎛️ Tuning Guide

### Problem: Robot moves too slowly
**Solution**:
- Increase `max_linear_vel`
- Increase `weight_velocity`
- Decrease `weight_smoothness`
- Try "aggressive" or "open_space" profile

### Problem: Robot oscillates or wobbles
**Solution**:
- Decrease `max_angular_vel`
- Increase `weight_smoothness`
- Decrease angular sampling rate
- Try "smooth" profile

### Problem: Robot gets too close to obstacles
**Solution**:
- Increase `weight_obstacle`
- Increase `safety_margin`
- Increase `robot_radius`
- Try "cautious" profile

### Problem: Robot can't navigate tight spaces
**Solution**:
- Decrease `robot_radius` (if safe)
- Increase angular samples
- Increase `sim_time` for better planning
- Try "tight_spaces" profile

### Problem: Robot doesn't follow path closely
**Solution**:
- Increase `weight_path_distance`
- Decrease `weight_goal_distance`
- Check global path quality

### Problem: Robot gets stuck
**Solution**:
- Switch to "recovery" profile
- Allow reverse motion (`min_linear_vel < 0`)
- Reduce obstacle weights temporarily
- Increase angular velocity limits

---

## 🔧 API Reference

### Classes

#### `ESP32RobotController`
Interface to ESP32 robot hardware.

```python
robot = ESP32RobotController(port, baud_rate)
robot.start()                    # Start telemetry reading
robot.send_velocity(twist)       # Send velocity command
pose = robot.get_pose()          # Get current pose
robot.stop()                     # Stop and cleanup
```

#### `DWBLocalPlanner`
Core DWB planning algorithm.

```python
planner = DWBLocalPlanner(robot, scanner)
planner.set_global_path(path)             # Set path to follow
cmd = planner.compute_velocity_command()  # Get next command
is_done = planner.goal_reached            # Check if goal reached
```

#### `SimulatedLaserScanner`
Simulated laser scanner (replace with real sensor).

```python
scanner = SimulatedLaserScanner()
scanner.add_obstacle(x, y, radius)
obstacles = scanner.get_obstacles_in_range(pose, max_range)
```

### Data Structures

```python
@dataclass
class Pose:
    x: float      # meters
    y: float      # meters
    theta: float  # radians

@dataclass
class Twist:
    linear: float   # m/s
    angular: float  # rad/s

@dataclass
class Trajectory:
    poses: List[Pose]
    twist: Twist
    cost: float
```

---

## 🐛 Troubleshooting

### Serial Connection Issues
```bash
# List available ports
ls /dev/tty*

# Check permissions (Linux)
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0

# Windows: Check Device Manager for COM port
```

### Import Errors
```bash
# Install missing dependencies
pip install pyserial numpy matplotlib

# Verify installation
python -c "import serial; print('OK')"
```

### Performance Issues
- Reduce `LOOP_RATE` if CPU limited
- Reduce `vel_samples` and `angular_samples`
- Decrease `sim_time`
- Use simpler cost functions

---

## 📊 Performance Metrics

Typical performance on ESP32:
- Planning frequency: 20 Hz
- Velocity samples: 15 linear × 20 angular = 300 trajectories
- Planning latency: ~30-50ms
- Memory usage: ~5KB for trajectories

---

## 🤝 Contributing

To add new features:
1. Create custom cost functions in `DWBLocalPlanner._score_trajectory()`
2. Add new configuration profiles to `dwb_config.py`
3. Define new scenarios in `dwb_integrated.py`

---

## 📄 License

MIT License - feel free to use in your projects!

---

## 🙏 Acknowledgments

Based on ROS Navigation Stack's DWB Local Planner
- Original paper: Fox et al. "The Dynamic Window Approach to Collision Avoidance"
- ROS DWB: github.com/ros-planning/navigation2

---

## 📞 Support

For issues and questions:
1. Check this README
2. Review configuration profiles
3. Try simulation mode for debugging
4. Adjust tuning parameters

**Happy Navigating! 🤖**
