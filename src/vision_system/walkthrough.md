# Vision System Implementation - Walkthrough

## Summary
Successfully implemented a complete **RGB-D vision system** for detecting colored objects and shapes using the `ascamera` package. This system integrates depth sensing with computer vision to isolate foreground objects from their background effectively.

---

## Files Created

### 1. RGB-D Object Detector
**File:** `rgbd_object_detector.py`

**Features:**
* Subscribes to `/ascamera/color0/image_raw` and `/ascamera/depth0/image_raw`.
* Synchronizes RGB and depth streams using `ApproximateTimeSynchronizer`.
* Estimates background depth from image edges to create a dynamic clipping plane.
* Filters objects based on a depth threshold (keeping only those closer than the background).
* Detects colored objects using calibrated HSV color ranges.
* Draws bounding boxes with depth labels.
* Publishes results to `/vision/detected_objects`.

**Key Algorithm:**
1. Estimate background depth from edges.
2. Set $threshold = background\_depth - margin$.
3. Create HSV color mask.
4. For each contour:
   - Get median depth in bounding box.
   - If $depth < threshold$: Draw bounding box.

### 2. Color Calibration Tool
**File:** `color_calibration_tool.py`

**Features:**
* Interactive HSV trackbars for real-time tuning.
* Real-time mask preview to visualize isolation.
* Save/load presets to `color_presets.json`.
* **8 color presets:** Red, green, blue, yellow, orange, purple, white, and black.
* Keyboard shortcuts for quick preset switching.

**Workflow:**
1. Run tool.
2. Adjust H/S/V sliders until target color is isolated.
3. Press **'s'** to save to `color_presets.json`.

### 3. Shape Detector
**File:** `shape_detector.py`

**Features:**
* Detects circles, triangles, squares, and rectangles.
* Uses Canny edge detection and contour hierarchy.
* **Classifies shapes using:**
  * **Circularity:** $\frac{4\pi \times Area}{Perimeter^2}$
  * **Polygon approximation:** Vertex counting via `approxPolyDP`.
  * **Aspect ratio:** Width/height comparison for squares vs. rectangles.
* Color-coded bounding boxes and shape count summary overlay.

**Classification Logic:**
* **Circle:** Circularity $> 0.75$
* **Triangle:** 3 vertices
* **Square:** 4 vertices, aspect ratio $0.9 - 1.1$
* **Rectangle:** 4 vertices, aspect ratio $\neq 1$

### 4. Supporting Files
* **`color_presets.json`**: Default HSV ranges for 8 colors; updated by calibration tool.
* **`README.md`**: Usage instructions, parameter documentation, and quick start guide.

---

## Technical Details

### Message Synchronization
RGB and depth streams are synchronized using `message_filters` to ensure spatial-temporal alignment:

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber

rgb_sub = Subscriber(self, Image, '/ascamera/color0/image_raw')
depth_sub = Subscriber(self, Image, '/ascamera/depth0/image_raw')

sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], 
                                   queue_size=10, slop=0.1)
sync.registerCallback(self.rgbd_callback)
```

---

## Remote Display Setup (Raspberry Pi → Laptop)

### Why X11 Forwarding?

When your camera is on a Raspberry Pi and you're connected via SSH from your laptop, you need **X11 forwarding** to display OpenCV windows on your laptop while the code runs on the Pi.

**X11** is the Linux display server protocol. **X11 forwarding** tunnels graphical output through SSH.

### Quick Setup

**On Laptop (Linux/Mac):**
```bash
ssh -X pi@<raspberry_pi_ip>
```

**On Laptop (Windows):**
1. Install **VcXsrv** or **Xming** (X server for Windows)
2. Start the X server
3. Connect: `ssh -X pi@<raspberry_pi_ip>`

**On Raspberry Pi:**
```bash
sudo nano /etc/ssh/sshd_config
# Ensure: X11Forwarding yes
sudo systemctl restart ssh
```

**Verify Connection:**
```bash
echo $DISPLAY  # Should show: localhost:10.0
xclock         # Test window (should appear on laptop)
```

### Running Vision Scripts Remotely

```bash
# 1. SSH with X11 forwarding
ssh -X pi@raspberry_pi_ip

# 2. Navigate to workspace
cd ~/autonomous_mobile_robotics
source install/setup.bash

# 3. Launch camera
ros2 launch ascamera ascamera.launch.py &

# 4. Run vision script
python3 src/vision_system/rgbd_object_detector.py
# Windows will appear on your laptop!
```

### Performance Tips

- **Faster forwarding**: Use `ssh -Y` (trusted, less secure)
- **Compressed**: Use `ssh -XC` for slower networks
- **Disable windows**: `-p show_windows:=false` if too slow

### Alternative: ROS2 Image Topics

If X11 is too slow, view images via ROS2 topics:

```bash
# On Pi: Run without windows
python3 rgbd_object_detector.py --ros-args -p show_windows:=false

# On Laptop: View in RViz
ros2 run rviz2 rviz2
# Add Image display → Topic: /vision/detected_objects
```

### Troubleshooting

**"cannot open display":**
```bash
echo $DISPLAY  # Check if set
# If empty, reconnect with -X
```

**Slow/laggy windows:**
- Use `ssh -Y` instead of `-X`
- Use compression: `ssh -XC`
- Reduce resolution or disable windows

**Windows don't appear (Windows OS):**
- Ensure VcXsrv/Xming is running
- Check firewall settings
- Restart X server

---