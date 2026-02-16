# Vision System

RGB-D vision system for detecting colored objects and shapes using the ascamera package.

## Scripts

### 1. RGB-D Object Detector (`rgbd_object_detector.py`)
Detects colored objects on a flat game field by filtering objects closer than the background using depth data.

**Features:**
- Synchronizes RGB and depth streams
- Filters objects by depth (closer than background)
- Detects colored objects using HSV ranges
- Draws bounding boxes with depth information

**Usage:**
```bash
# Launch camera first
ros2 launch ascamera ascamera.launch.py

# Run detector
python3 rgbd_object_detector.py

# With parameters
ros2 run vision_system rgbd_object_detector --ros-args \
  -p color_preset:=blue \
  -p background_margin_mm:=50.0 \
  -p min_area:=500
```

**Parameters:**
- `color_preset`: Color to detect (red, green, blue, yellow, etc.)
- `background_margin_mm`: Depth margin below background (default: 50mm)
- `min_area`: Minimum object area in pixels (default: 500)
- `max_area`: Maximum object area in pixels (default: 50000)

---

### 2. Color Calibration Tool (`color_calibration_tool.py`)
Interactive tool to find optimal HSV ranges for different colors.

**Usage:**
```bash
python3 color_calibration_tool.py
```

**Controls:**
- Adjust HSV sliders to isolate target color
- Press `s` to save current values
- Press `l` to load saved values
- Press `1-8` to switch color presets
- Press `q` to quit

**Presets:** Saved to `color_presets.json`

---

### 4. ROS Color Detection (`ros_color_detection.py`) [NEW]
A ROS-compatible version of the color detector that subscribes to live camera topics.

**Usage:**
```bash
# Run with default Pi camera topic
python3 ros_color_detection.py

# Run with Ascamera RGB topic
python3 ros_color_detection.py --ros-args -p camera_topic:=/ascamera/color0/image_raw
```

**Features:**
- Real-time HSV filtering via ROS parameters
- Publishes binary mask to `/vision/color_mask`
- Processes data directly from the robot's hardware interface

---

### 5. Shape Detector (`shape_detector.py`)

**Shapes Detected:**
- **Circle**: Circularity > 0.75
- **Triangle**: 3 vertices
- **Square**: 4 vertices, aspect ratio ~1
- **Rectangle**: 4 vertices, aspect ratio ≠ 1

---

## Camera Topics

**RGB:**
- `/ascamera/color0/image_raw` - RGB image
- `/ascamera/color0/camera_info` - Camera info

**Depth:**
- `/ascamera/depth0/image_raw` - Depth image
- `/ascamera/depth0/camera_info` - Depth camera info
- `/ascamera/depth0/points` - Point cloud

**Vision Output:**
- `/vision/detected_objects` - Annotated RGB-D detections
- `/vision/detected_shapes` - Annotated shape detections

---

## Dependencies

```bash
pip3 install opencv-python numpy
```

ROS2 packages:
- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `message_filters`

---

## Remote Display (Raspberry Pi → Laptop)

When running vision scripts on a Raspberry Pi and viewing windows on your laptop via SSH.

### Understanding X11 Forwarding

**X11** is the display server protocol used by Linux. **X11 forwarding** tunnels graphical output through SSH, allowing OpenCV windows to appear on your laptop while the code runs on the Raspberry Pi.

### Setup Steps

#### 1. On Your Laptop (SSH Client)

**Linux/Mac:**
```bash
# Connect with X11 forwarding enabled
ssh -X pi@<raspberry_pi_ip>

# Or for trusted X11 forwarding (faster)
ssh -Y pi@<raspberry_pi_ip>
```

**Windows:**
Install an X server first:
- **VcXsrv** (recommended): https://sourceforge.net/projects/vcxsrv/
- **Xming**: https://sourceforge.net/projects/xming/

Then use:
```bash
# In PowerShell or WSL
ssh -X pi@<raspberry_pi_ip>
```

#### 2. On Raspberry Pi (SSH Server)

Enable X11 forwarding in SSH config:
```bash
sudo nano /etc/ssh/sshd_config
```

Ensure these lines are set:
```
X11Forwarding yes
X11DisplayOffset 10
X11UseLocalhost yes
```

Restart SSH service:
```bash
sudo systemctl restart ssh
```

#### 3. Verify X11 Forwarding

After SSH connection, check DISPLAY variable:
```bash
echo $DISPLAY
# Should show something like: localhost:10.0
```

Test with a simple GUI:
```bash
xclock
# A clock window should appear on your laptop
```

### Running Vision Scripts Remotely

```bash
# 1. SSH with X11 forwarding
ssh -X pi@raspberry_pi_ip

# 2. Navigate to workspace
cd ~/autonomous_mobile_robotics

# 3. Source ROS2
source install/setup.bash

# 4. Launch camera
ros2 launch ascamera ascamera.launch.py &

# 5. Run vision script
python3 src/vision_system/rgbd_object_detector.py
# Windows will appear on your laptop!
```

### Troubleshooting X11

**Problem: "cannot open display"**
```bash
# Check DISPLAY is set
echo $DISPLAY

# If empty, reconnect with -X flag
exit
ssh -X pi@raspberry_pi_ip
```

**Problem: Slow/laggy windows**
- Use `-Y` instead of `-X` for faster (but less secure) forwarding
- Reduce image resolution if possible
- Use `-C` for compression: `ssh -XC pi@raspberry_pi_ip`

**Problem: Windows don't appear (Windows OS)**
- Ensure VcXsrv/Xming is running
- Check firewall allows X server connections
- Restart X server and reconnect

### 4. Background Depth Filtering Logic
In `rgbd_object_detector.py`, we use a unique filtering approach:
1. **Sample Edges**: The script samples the outermost pixels of the depth frame (top, bottom, left, right).
2. **Median Depth**: It calculates the median value of these samples to estimate the "background" (floor) distance.
3. **Subtraction Mask**: Any pixel further than `background_depth - margin` is masked out. 
4. **Result**: Only objects physically standing *above* the floor or background are processed by the color detector.

---

## Troubleshooting Depth-RGB Issues

- **Objects not being detected**: The `background_margin_mm` might be too high (filtering out small objects) or too low (noise from the floor). Try adjusting it:
  ```bash
  ros2 run vision_system rgbd_object_detector --ros-args -p background_margin_mm:=30.0
  ```
- **Depth Map is Black**: Ensure you are using `rqt_image_view` with the "Normalize" checkbox enabled, or use RViz2 with appropriate range scaling.
- **Misalignment**: If the color box and physical object don't match, check that you are subscribing to the *registered* depth topic (if available) or ensure the ascamera registration features are enabled in `ascamera.launch.py`.

---

## Quick Start



```bash
# 1. Launch camera
ros2 launch ascamera ascamera.launch.py

# 2. Calibrate colors (optional)
python3 color_calibration_tool.py
# Adjust sliders, press 's' to save

# 3. Run object detector
python3 rgbd_object_detector.py

# 4. Run shape detector (in another terminal)
python3 shape_detector.py
```

---

## Color Presets

Default HSV ranges in `color_presets.json`:
- Red: H[0-10], S[100-255], V[100-255]
- Green: H[35-85], S[50-255], V[50-255]
- Blue: H[90-130], S[50-255], V[50-255]
- Yellow: H[20-35], S[100-255], V[100-255]
- Orange: H[10-25], S[100-255], V[100-255]
- Purple: H[130-160], S[50-255], V[50-255]
- White: H[0-179], S[0-40], V[200-255]
- Black: H[0-179], S[0-255], V[0-40]

Adjust using the calibration tool and save for your specific lighting conditions.
