# SLAM and Mapping Guide - LidarBot

> **Detailed guide for generating, saving, and managing occupancy grid maps**
> 
> Version: 1.0 | Last Updated: 2026-02-16

---

## Overview

Mapping is the process of using the robot's sensors (LiDAR and Odometry) to create a representation of its environment. For the LidarBot, we use **slam_toolbox**, a highly capable and robust SLAM package for ROS 2.

### Mapping Mode
Our primary mapping mode is **Online Asynchronous Mapping**. This allows the robot to build a map in real-time while prioritizing sensor data over consistency, making it ideal for the Raspberry Pi's processing constraints.

---

## Prerequisites

1. **Hardware Setup**:
   - Raspberry Pi 4 (or similar) on the robot.
   - YDLidar connected and functional.
   - Odometry (IMU + Encoders) active and calibrated.

2. **Laptop Setup**:
   - Laptop on the same Wi-Fi network as the robot.
   - ROS 2 Humble installed on the laptop.
   - `slam_toolbox` and `rviz2` installed.

---

## Step 1: Launching Mapping on the Robot

Run the following command on the **Raspberry Pi**:

```bash
ros2 launch lidarbot_bringup autonomous_navigation.launch.py
```

This launch file brings up:
- ✅ Hardware Interface (Motors/Encoders)
- ✅ YDLidar Driver
- ✅ SLAM Toolbox (Asynchronous Mapping Mode)

### Manual Launch (Fragmented)
If you want to run ONLY mapping without Nav2:

```bash
# 1. Start Hardware & LiDAR
ros2 launch lidarbot_bringup lidarbot_check_hardware.launch.py use_ros2_control:=True

# 2. Start SLAM Toolbox
ros2 launch lidarbot_slam online_async_launch.py
```

---

## Step 2: Visualization on Laptop

To see the map being built in real-time, you should use **RViz2** on your laptop.

### Configuration

1. **Set ROS Domain ID**: Ensure both Pi and Laptop share the same `ROS_DOMAIN_ID`:
   ```bash
   export ROS_DOMAIN_ID=X  # Replace X with your ID
   ```

2. **Launch RViz2 with the mapping config**:
   ```bash
   ros2 run rviz2 rviz2 -d ~/autonomous_mobile_robotics/src/lidarbot/lidarbot_slam/rviz/mapper_params_online_async.rviz
   ```

### What to Look For in RViz
- **Map Topic**: `/map` (Occupancy Grid)
- **LaserScan**: `/scan` (LiDAR points)
- **Robot Model**: Shows `base_link` relative to `map`
- **TF Tree**: Look for the `map -> odom` transform being generated.

---

## Step 3: Mapping Best Practices

1. **Slow and Steady**: Drive the robot slowly (e.g., 0.15 m/s). Rapid turns can cause "map smearing" or loss of localization.
2. **Loop Closure**: Re-visit known locations (loop closure). SLAM Toolbox will recognize the area and correct accumulated drift.
3. **Scan Overlap**: Ensure successive LiDAR scans have enough overlap.
4. **Environment Features**: SLAM works best in areas with distinct features (boxes, corners, furniture). Long, flat, featureless hallways are challenging.

---

## Step 4: Saving the Map

Once you are satisfied with your map, you need to save it for future use (e.g., for localization-only navigation).

### Method 1: Using SLAM Toolbox Plugin (Visual)
1. Open RViz2 on your laptop.
2. Ensure the **SlamToolboxPlugin** is visible (Panels -> Add New Panel -> SlamToolboxPlugin).
3. Type the map name in the **Save Map** field.
4. Click **Save Map**.

### Method 2: Command Line (Reliable)
Run this command from your **laptop or Pi**:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map_name
```

**Output Files:**
- `my_map_name.yaml`: Configuration file containing map metadata (resolution, origin).
- `my_map_name.pgm`: The actual map image (Portable Gray Map).

---

## Step 5: Loading a Saved Map

To use a previously saved map for navigation (without re-mapping):

1. **Update `slam_toolbox` parameters**:
   Edit `src/lidarbot/lidarbot_slam/config/mapper_params_online_async.yaml`:
   ```yaml
   mode: localization
   map_file_name: /home/pi/my_map_name
   ```

2. **Launch with the new config**:
   ```bash
   ros2 launch lidarbot_bringup autonomous_navigation.launch.py
   ```

---

## Troubleshooting Mapping Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| **Map is "smearing"** | Robot turning too fast | Slow down rotation velocity. |
| **Map is disjointed** | Poor odometry | Calibrate wheel separation/radius in `controllers.yaml`. |
| **No map appearing in RViz** | Topic mismatch | Check `ros2 topic list`. Ensure it's `/map`. |
| **Transforms failing** | TF tree broken | Check if `robot_state_publisher` is running. |
| **Scan not clearing space** | Raytrace parameter | Check `raytrace_max_range` in costmap configuration. |

---

## Advanced: Mapping Tuning

Key parameters in `mapper_params_online_async.yaml`:

- `resolution`: Default `0.05`m (5cm per pixel). Lowering this (e.g., `0.02`) makes maps more detailed but increases CPU usage.
- `minimum_travel_distance`: Distance the robot must move before adding a new scan to the map.
- `do_loop_closing`: Set to `true` (default) for best long-term accuracy.
