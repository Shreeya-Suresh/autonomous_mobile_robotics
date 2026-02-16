# LidarBot Color Visualization Scripts

Three Python scripts for color detection and depth visualization with your Ascamera RGB-D system.

## 📁 Files

1. **`color_identifier.py`** - Detects and identifies specific colors in images
2. **`color_depth_visualizer.py`** - Combines color detection with depth information
3. **`color_depth_ros_node.py`** - ROS2 node for real-time analysis

---

## 🚀 Quick Start

### Prerequisites

```bash
pip install opencv-python numpy --break-system-packages

# For ROS2 integration:
pip install cv-bridge --break-system-packages
```

---

## 1️⃣ Color Identification Script

Detects specific colors (red, blue, green, yellow, orange, purple, cyan, white, black) in images.

### Usage Examples

**Process a single image:**
```bash
python3 color_identifier.py --input test_image.jpg --colors red blue green
```

**Live camera feed:**
```bash
python3 color_identifier.py --camera --colors red blue yellow
```

**Save annotated output:**
```bash
python3 color_identifier.py --input image.jpg --output result.jpg
```

**Adjust sensitivity:**
```bash
python3 color_identifier.py --input image.jpg --min-area 1000
```

### Features
- Detects 9 different colors using HSV color space
- Filters out noise with minimum area threshold
- Shows bounding boxes, centers, and coverage percentage
- Works with images or live camera feed

### Output Example
```
==============================================================
Detected 3 color region(s):
==============================================================

1. RED
   Position: (245, 180)
   Area: 1523 pixels (2.37% of image)
   Bounding Box: x=200, y=150, w=90, h=60

2. BLUE
   Position: (450, 320)
   Area: 2810 pixels (4.39% of image)
   Bounding Box: x=410, y=280, w=80, h=80
```

---

## 2️⃣ Color + Depth Visualizer

Combines color detection with depth information from RGB-D camera.

### Usage Examples

**Basic usage:**
```bash
python3 color_depth_visualizer.py --rgb color.jpg --depth depth.png
```

**Specify colors to detect:**
```bash
python3 color_depth_visualizer.py \
  --rgb color.jpg \
  --depth depth.png \
  --colors red blue green yellow
```

**Adjust depth filtering:**
```bash
# Objects must be 100mm closer than background to be detected
python3 color_depth_visualizer.py \
  --rgb color.jpg \
  --depth depth.png \
  --background-margin 100
```

**Save visualization:**
```bash
python3 color_depth_visualizer.py \
  --rgb color.jpg \
  --depth depth.png \
  --output result.png
```

### Features
- Estimates background depth automatically
- Filters objects based on depth (foreground only)
- Calculates median, min, max depth for each object
- Side-by-side RGB + depth visualization
- Shows distance in millimeters and centimeters

### Output Example
```
======================================================================
COLOR + DEPTH ANALYSIS
======================================================================
Background depth: 2450mm (245.0cm)
Detected objects: 2
======================================================================

1. RED
   Distance: 550mm (55.0cm)
   From background: 1900mm closer
   Position: (320, 240)
   Area: 1823 pixels
   Depth range: 520-580mm

2. BLUE
   Distance: 720mm (72.0cm)
   From background: 1730mm closer
   Position: (180, 310)
   Area: 2156 pixels
   Depth range: 690-750mm
```

---

## 3️⃣ ROS2 Integration Node

Real-time color and depth analysis by subscribing to Ascamera topics.

### Launch with Default Settings

```bash
ros2 run vision_system color_depth_ros_node.py
```

### Launch with Custom Parameters

```bash
ros2 run vision_system color_depth_ros_node.py --ros-args \
  -p colors:="['red','blue','green']" \
  -p min_area:=800 \
  -p background_margin_mm:=100 \
  -p show_visualization:=true
```

### Custom Topics

If your Ascamera uses different topic names:
```bash
ros2 run vision_system color_depth_ros_node.py --ros-args \
  -p rgb_topic:="/camera/color/image" \
  -p depth_topic:="/camera/depth/image"
```

### Features
- Subscribes to `/ascamera/color0/image_raw` and `/ascamera/depth0/image_raw`
- Processes at 10 Hz
- Logs detections to console
- Shows live visualization window
- Handles 16UC1 depth format (millimeters)

---

## 🎨 Supported Colors

The scripts detect these colors using HSV ranges:

| Color  | HSV Range |
|--------|-----------|
| Red    | 0-10, 170-180 (wraps around) |
| Blue   | 100-130 |
| Green  | 40-80 |
| Yellow | 20-30 |
| Orange | 10-20 |
| Purple | 130-170 |
| Cyan   | 80-100 |
| White  | Low saturation, high value |
| Black  | Low value |

---

## 📊 Understanding Depth Data

### Depth Image Format
- **Encoding**: 16UC1 (16-bit unsigned integer, 1 channel)
- **Units**: Millimeters
- **Range**: 0-65535mm (0 = invalid/no data)
- **Typical Range**: 200-4000mm (20cm - 4m)

### How Background Estimation Works
1. Samples the edges of the depth image (top, bottom, left, right)
2. Takes the median depth value from these edges
3. Objects closer than `background - margin` are considered foreground

### Depth Visualization
- **Red/Yellow**: Close objects (~200-1000mm)
- **Green**: Mid-range (~1000-2500mm)
- **Blue/Purple**: Far objects (~2500-4000mm)

---

## 🔧 Adjustable Parameters

### Color Identification
- `--min-area`: Minimum contour area in pixels (default: 500)
  - Increase to ignore small noise
  - Decrease to detect tiny objects

### Depth Processing
- `--background-margin`: How much closer than background (default: 50mm)
  - Increase to only detect very close objects
  - Decrease to include objects near the background

---

## 🐛 Troubleshooting

### "No colors detected"
- Try lowering `--min-area`
- Check if colors are in the supported list
- Verify image is loaded correctly
- Try adjusting lighting conditions

### "Depth image all black in visualization"
- Objects might be too close (< 20cm) or too far (> 4m)
- Check depth image with: `ros2 run rqt_image_view rqt_image_view`
- Enable "Normalize" checkbox to see raw depth data

### "ROS node not receiving images"
- Verify topics: `ros2 topic list`
- Check if Ascamera is running: `ros2 launch ascamera ascamera.launch.py`
- Echo topic to verify data: `ros2 topic echo /ascamera/color0/image_raw --once`

### "Depth and RGB misaligned"
- This is normal if cameras are not calibrated
- The Ascamera should provide aligned depth automatically
- Verify you're using the aligned depth topic

---

## 📝 Integration with Your Vision System

To integrate these into your existing `vision_system` package:

```bash
# Copy scripts to your package
cp color_identifier.py ~/ros2_ws/src/lidarbot/vision_system/scripts/
cp color_depth_visualizer.py ~/ros2_ws/src/lidarbot/vision_system/scripts/
cp color_depth_ros_node.py ~/ros2_ws/src/lidarbot/vision_system/scripts/

# Make executable
chmod +x ~/ros2_ws/src/lidarbot/vision_system/scripts/*.py

# Rebuild
cd ~/ros2_ws
colcon build --packages-select vision_system
source install/setup.bash
```

---

## 🎯 Example Workflow

### 1. Test color detection on a saved image
```bash
python3 color_identifier.py --input test.jpg --colors red blue
```

### 2. Capture RGB and depth from ROS
```bash
# Terminal 1: Launch Ascamera
ros2 launch ascamera ascamera.launch.py

# Terminal 2: Save images
ros2 run image_view image_saver --ros-args \
  -r image:=/ascamera/color0/image_raw
ros2 run image_view image_saver --ros-args \
  -r image:=/ascamera/depth0/image_raw
```

### 3. Process saved images
```bash
python3 color_depth_visualizer.py \
  --rgb color_image.jpg \
  --depth depth_image.png \
  --colors red blue green
```

### 4. Run live with ROS
```bash
ros2 run vision_system color_depth_ros_node.py --ros-args \
  -p colors:="['red','blue']" \
  -p min_area:=1000
```

---

## 📖 API Usage

You can also import these as modules:

```python
from color_identifier import ColorIdentifier
from color_depth_visualizer import ColorDepthVisualizer

# Color only
detector = ColorIdentifier(min_area=800)
results = detector.identify_colors(rgb_image, ['red', 'blue'])

# Color + Depth
visualizer = ColorDepthVisualizer(min_area=800, background_margin_mm=100)
results = visualizer.process_rgbd(rgb_image, depth_image, ['red', 'blue'])

# Access detections
for detection in results['detections']:
    print(f"Found {detection['color']} at {detection['center']}")
    print(f"  Distance: {detection['depth_mm']}mm")
```

---

## 📚 Resources

- OpenCV Color Spaces: https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
- ROS2 Image Transport: https://github.com/ros-perception/image_transport_tutorials
- CV Bridge: http://wiki.ros.org/cv_bridge

---

## 💡 Tips

1. **Lighting**: HSV color detection works best with consistent lighting
2. **Calibration**: Adjust HSV ranges if colors aren't detected properly
3. **Performance**: Reduce resolution if processing is slow
4. **Accuracy**: Use depth filtering to eliminate background confusion
5. **Testing**: Always test with sample images before live deployment

---

## 🔮 Future Enhancements

- [ ] Auto-calibration of HSV ranges
- [ ] Multi-color tracking across frames
- [ ] 3D position estimation using depth
- [ ] Export detections to CSV/JSON
- [ ] ROS service for on-demand detection
- [ ] Deep learning integration (YOLO + depth)

---

Happy detecting! 🎨📏🤖
