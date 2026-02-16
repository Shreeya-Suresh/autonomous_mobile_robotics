# Lidarbot Teleop - Complete Execution & Debugging Walkthrough

## System Overview

Your teleop system consists of:
1. **ESP32 Firmware** - Motor control, encoders, IMU (MPU6050)
2. **Serial Hardware Interface** - ROS2 ↔ ESP32 communication
3. **diff_drive_controller** - Differential drive kinematics
4. **teleop_twist_keyboard** - Keyboard control input
5. **YDLidar** - LiDAR sensor (optional for teleop)

**Data Flow:**
```
Keyboard → teleop_twist_keyboard → /diff_controller/cmd_vel_unstamped
                                           ↓
                                    diff_drive_controller
                                           ↓
                                    Serial Hardware Interface
                                           ↓
                                    ESP32 (V,left_vel,right_vel)
                                           ↓
                                    Motors + Encoders
                                           ↓
                                    ESP32 (D,left_enc,right_enc,yaw)
                                           ↓
                                    Serial Hardware Interface
                                           ↓
                                    /diff_controller/odom
```

---

## Prerequisites

### 1. Hardware Setup
- [ ] ESP32 connected to laptop via USB
- [ ] Motors powered (external power supply)
- [ ] Encoders connected to ESP32
- [ ] IMU (MPU6050) connected via I2C
- [ ] YDLidar connected (optional)

### 2. Firmware Flashed
```bash
# Verify ESP32 is detected
ls /dev/ttyUSB* /dev/ttyACM*
# Should show: /dev/ttyUSB0 or /dev/ttyACM0

# Test serial communication
screen /dev/ttyUSB0 115200
# Should see: "ESP32 READY" and periodic "D,..." messages
# Press Ctrl+A then K to exit screen
```

### 3. User Permissions
```bash
# Add user to dialout group (for serial port access)
sudo usermod -aG dialout $USER
# Log out and log back in for changes to take effect

# Verify permissions
groups
# Should include: dialout
```

### 4. ROS2 Workspace Built
```bash
cd ~/autonomous_mobile_robotics
colcon build --symlink-install
source install/setup.bash
```

---

## Step-by-Step Execution

### Step 1: Launch Teleop System

```bash
# Terminal 1: Launch teleop
cd ~/autonomous_mobile_robotics
source install/setup.bash
ros2 launch lidarbot_teleop teleop.launch.py
```

**Expected Output:**
```
[INFO] [robot_state_publisher]: Publishing robot description
[INFO] [ros2_control_node]: Loaded lidarbot_serial_base/LidarbotSerialHardware
[INFO] [ros2_control_node]: Successful connection to port /dev/ttyUSB0
[INFO] [spawner-X]: Loaded controller 'diff_controller'
[INFO] [spawner-X]: Loaded controller 'joint_broadcaster'
[INFO] [spawner-X]: Loaded controller 'imu_broadcaster'
[INFO] [ydlidar_ros2_driver_node]: [YDLIDAR] Connection established
```

**⚠️ If xterm window doesn't appear:**
- Install xterm: `sudo apt install xterm`
- Or manually run teleop in separate terminal (see Alternative Method below)

### Step 2: Control Robot with Keyboard

In the **xterm window** (or separate terminal):

```
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```

**Controls:**
- `i` - Forward
- `,` - Backward
- `j` - Rotate left
- `l` - Rotate right
- `k` - Stop
- `u` - Forward + left
- `o` - Forward + right

### Step 3: Verify Robot Movement

**Physical Verification:**
1. Press `i` - Both wheels should spin forward
2. Press `,` - Both wheels should spin backward
3. Press `j` - Left wheel backward, right wheel forward (rotate left)
4. Press `l` - Left wheel forward, right wheel backward (rotate right)
5. Press `k` - Both wheels stop

---

## Verification & Debugging

### Check 1: Controllers are Active

```bash
# Terminal 2
ros2 control list_controllers
```

**Expected Output:**
```
diff_controller[active]
joint_broadcaster[active]
imu_broadcaster[active]
```

**❌ If controllers show [inactive]:**
```bash
# Manually activate
ros2 control set_controller_state diff_controller active
ros2 control set_controller_state joint_broadcaster active
ros2 control set_controller_state imu_broadcaster active
```

---

### Check 2: Serial Communication

```bash
# Check hardware interface is receiving data
ros2 topic echo /joint_states --once
```

**Expected Output:**
```yaml
header:
  stamp:
    sec: ...
    nanosec: ...
  frame_id: ''
name:
- left_wheel_joint
- right_wheel_joint
position: [0.123, 0.456]  # Should change when wheels move
velocity: [0.0, 0.0]
effort: []
```

**❌ If no data or stale timestamps:**
- Check ESP32 connection: `ls /dev/ttyUSB*`
- Check serial port in URDF (should be `/dev/ttyUSB0` or `/dev/ttyACM0`)
- Restart ros2_control_node

---

### Check 3: Velocity Commands

```bash
# Monitor velocity commands
ros2 topic echo /diff_controller/cmd_vel_unstamped
```

**Expected Output (when pressing 'i'):**
```yaml
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

**❌ If no messages:**
- Teleop keyboard not running
- Check topic remapping in launch file
- Manually test: `ros2 topic pub /diff_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`

---

### Check 4: Odometry is Publishing

```bash
# Check odometry topic
ros2 topic echo /diff_controller/odom --once
```

**Expected Output:**
```yaml
header:
  stamp: ...
  frame_id: odom
child_frame_id: base_footprint
pose:
  pose:
    position:
      x: 0.0  # Should change when robot moves
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
twist:
  twist:
    linear:
      x: 0.0  # Should match commanded velocity
    angular:
      z: 0.0
```

**❌ If position doesn't change:**
- Encoders not connected properly
- Encoder counts not incrementing (check firmware)
- Wheel parameters incorrect in `controllers.yaml`

---

### Check 5: IMU Data

```bash
# Check IMU topic
ros2 topic echo /imu_broadcaster/imu --once
```

**Expected Output:**
```yaml
header:
  stamp: ...
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0  # Yaw - should change when robot rotates
  w: 1.0
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0  # Should be non-zero when rotating
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 9.81  # Gravity
```

---

### Check 6: TF Transforms

```bash
# Verify transform tree
ros2 run tf2_tools view_frames
# Opens frames.pdf showing TF tree

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_footprint
```

**Expected Output:**
```
At time ...
- Translation: [x, y, z]
- Rotation: in Quaternion [x, y, z, w]
```

**❌ If transform not available:**
- `enable_odom_tf: false` in controllers.yaml (see Odometry TF Issue below)
- Check robot_state_publisher is running
- Verify URDF has correct frame definitions

---

### Check 7: Topic Rates

```bash
# Check publishing rates
ros2 topic hz /diff_controller/odom
ros2 topic hz /joint_states
ros2 topic hz /imu_broadcaster/imu
ros2 topic hz /scan  # If LiDAR connected
```

**Expected Rates:**
- `/diff_controller/odom`: ~50 Hz
- `/joint_states`: ~30 Hz
- `/imu_broadcaster/imu`: ~100 Hz
- `/scan`: ~10 Hz

---

## Debugging Common Issues

### Issue 1: Robot Doesn't Move

**Symptoms:** Keyboard commands received, but motors don't spin

**Debug Steps:**
```bash
# 1. Check velocity commands are published
ros2 topic echo /diff_controller/cmd_vel_unstamped

# 2. Check controller is processing commands
ros2 topic echo /diff_controller/odom

# 3. Monitor serial port directly
screen /dev/ttyUSB0 115200
# Look for "V,..." commands being sent

# 4. Check ESP32 is receiving commands
# In screen session, manually send: V,1.0,1.0
# Motors should spin
```

**Possible Causes:**
- Motors not powered (external power supply)
- Serial port permissions
- Wrong serial port in URDF
- Firmware not flashed correctly
- Motor driver connections

---

### Issue 2: Robot Moves in Wrong Direction

**Symptoms:** Forward command makes robot go backward, or rotation is reversed

**Debug Steps:**
```bash
# Test individual wheels
ros2 topic pub /diff_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
# Both wheels should spin forward

# If reversed:
# - Check motor wiring (swap motor leads)
# - Check encoder direction in firmware
# - Verify wheel_separation sign in controllers.yaml
```

**Potential Issues Identified:**
1. **Motor direction pins** - May need to invert logic in firmware
2. **Encoder counting** - ISR may be incrementing when should decrement
3. **Wheel separation** - Sign may be incorrect

---

### Issue 3: Odometry Drift

**Symptoms:** Robot position in RViz doesn't match physical position

**Debug Steps:**
```bash
# 1. Check encoder counts are incrementing
ros2 topic echo /joint_states

# 2. Verify wheel parameters
# In controllers.yaml:
#   wheel_separation: 0.130  # Distance between wheels (meters)
#   wheel_radius: 0.0325     # Wheel radius (meters)

# 3. Measure actual robot parameters
# - Wheel diameter with ruler
# - Wheelbase (center to center of wheels)

# 4. Test encoder accuracy
# - Mark wheel position
# - Rotate wheel exactly 1 revolution
# - Check encoder count change
# - Should be close to ticks_per_revolution (1084 in firmware)
```

**Potential Issues Identified:**
1. **Wheel parameters** - May not match physical robot
2. **Encoder resolution** - 1084 ticks/rev may be incorrect
3. **Wheel slippage** - Odometry assumes no slip

---

## Visualizing in RViz

```bash
# Terminal 3: Launch RViz
ros2 run rviz2 rviz2
```

**Add Displays:**
1. **RobotModel**
   - Description Topic: `/robot_description`
   
2. **TF**
   - Shows coordinate frames
   
3. **LaserScan** (if LiDAR connected)
   - Topic: `/scan`
   - Size: 0.05
   
4. **Odometry**
   - Topic: `/diff_controller/odom`
   - Keep: 100
   - Shaft Length: 0.3

**Set Fixed Frame:** `odom`

**Expected Behavior:**
- Robot model should appear
- When you drive with keyboard, robot should move in RViz
- Odometry trail should show path

---

## Understanding the Odometry TF Issue

### What is the Issue?

In `controllers.yaml` (line 36), you have:
```yaml
enable_odom_tf: false
```

This means **diff_drive_controller does NOT publish the `odom → base_footprint` transform**.

### Why is This a Problem?

**For teleop:** Not a major issue - you can still drive the robot.

**For SLAM/mapping:** **This WILL cause problems!**

### How SLAM/Mapping Works

SLAM algorithms (like slam_toolbox) need a complete TF tree:

```
map
 └── odom (published by SLAM)
      └── base_footprint (NEEDS to be published!)
           └── base_link (published by robot_state_publisher)
                ├── imu_link
                ├── lidar_link
                └── wheel_links
```

**The `odom → base_footprint` transform represents:**
- Robot's position based on wheel encoders (dead reckoning)
- Updated continuously at high frequency (~50 Hz)
- Accumulates drift over time

**The `map → odom` transform represents:**
- Correction from SLAM to account for drift
- Updated when SLAM finds loop closures
- Keeps robot localized in global map

### What Happens Without `odom → base_footprint`?

**Scenario 1: Only diff_drive_controller running (teleop)**
- ❌ No `odom → base_footprint` transform
- ✅ Can still drive robot (velocity commands work)
- ❌ No odometry visualization in RViz
- ❌ Cannot run SLAM

**Scenario 2: With robot_localization (EKF) running**
- ✅ EKF publishes `odom → base_footprint` by fusing wheel odom + IMU
- ✅ Better odometry (less drift)
- ✅ Can run SLAM
- ⚠️ **But you're not currently launching EKF!**

**Scenario 3: With `enable_odom_tf: true`**
- ✅ diff_drive_controller publishes `odom → base_footprint`
- ✅ Can run SLAM immediately
- ⚠️ Odometry only from wheels (no IMU fusion)
- ⚠️ More drift than with EKF

### Will This Affect SLAM/Mapping?

**YES - SLAM will NOT work without `odom → base_footprint` transform!**

**Error you'll see:**
```
[slam_toolbox]: Waiting for odom → base_footprint transform...
[slam_toolbox]: Transform timeout
```

### Solutions (Choose ONE)

#### Option 1: Enable Odom TF in diff_drive_controller (Simple)

**Change in `controllers.yaml`:**
```yaml
enable_odom_tf: true  # Changed from false
```

**Pros:**
- ✅ Simple, one-line change
- ✅ SLAM works immediately
- ✅ No additional nodes needed

**Cons:**
- ⚠️ Odometry from wheels only (no IMU fusion)
- ⚠️ More drift than with EKF

**When to use:** Quick testing, simple navigation

---

#### Option 2: Use robot_localization (EKF) (Recommended)

**Add to your launch file:**
```python
# Launch EKF node
start_ekf_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[ekf_params_file],
)
```

**Pros:**
- ✅ Fuses wheel odom + IMU for better accuracy
- ✅ Less drift
- ✅ More robust odometry

**Cons:**
- ⚠️ Requires EKF configuration file
- ⚠️ Additional node to manage

**When to use:** Production, accurate navigation, SLAM

---

### Checking Current TF Tree

```bash
# View all transforms
ros2 run tf2_tools view_frames

# Check if odom → base_footprint exists
ros2 run tf2_ros tf2_echo odom base_footprint
```

**If you see:**
```
Lookup would require extrapolation into the future
```
or
```
Could not find a connection between 'odom' and 'base_footprint'
```

**Then you NEED to enable odom TF or launch EKF!**

---

### For SLAM/Mapping - Action Required

**Before running SLAM, you MUST do ONE of:**

1. **Enable odom TF:**
   ```yaml
   # In controllers.yaml
   enable_odom_tf: true
   ```

2. **Launch EKF:**
   ```bash
   # Create/use EKF config file
   ros2 launch lidarbot_navigation ekf.launch.py
   ```

**Without this, SLAM will fail with transform errors!**

---

## Alternative: Manual Teleop Launch

If xterm doesn't work:

```bash
# Terminal 1: Launch hardware only
ros2 launch lidarbot_bringup lidarbot_check_hardware.launch.py

# Terminal 2: Run teleop keyboard manually
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/diff_controller/cmd_vel_unstamped
```

---

## Testing Checklist

- [ ] ESP32 firmware flashed and responding
- [ ] Serial port accessible (`/dev/ttyUSB0` or `/dev/ttyACM0`)
- [ ] User in `dialout` group
- [ ] ROS2 workspace built and sourced
- [ ] Teleop launches without errors
- [ ] Controllers show `[active]` status
- [ ] Keyboard commands publish to `/diff_controller/cmd_vel_unstamped`
- [ ] Motors respond to keyboard input
- [ ] Robot moves in correct directions (forward/backward/rotate)
- [ ] Encoders increment when wheels spin
- [ ] Odometry publishes and updates
- [ ] IMU publishes orientation and angular velocity
- [ ] **TF tree check:** `odom → base_footprint` exists (for SLAM)
- [ ] RViz shows robot model and movement
- [ ] LiDAR publishes scan data (if connected)

---

## Summary

Your teleop system is well-structured with proper separation of concerns.

**Current Status:**
- ✅ Teleop works for manual driving
- ⚠️ **Odometry TF disabled** - will prevent SLAM/mapping
- ✅ IMU integrated
- ✅ Safety timeout in firmware

**Before running SLAM/mapping:**
- **MUST enable `odom → base_footprint` transform**
- Either set `enable_odom_tf: true` OR launch robot_localization (EKF)
- Without this, SLAM will fail with transform errors

**Recommended next steps:**
1. Test teleop with this walkthrough
2. Verify all checks pass
3. For SLAM: Enable odom TF or configure EKF
4. Calibrate wheel parameters if needed
