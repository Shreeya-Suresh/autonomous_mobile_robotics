# Camera System Guide - LidarBot

> **Setup and operation guide for Raspberry Pi Camera and Ascamera (RGB-D)**
> 
> Version: 1.0 | Last Updated: 2026-02-16

---

## Overview

The LidarBot supports two camera configurations:
1. **Standard RPi Camera (v1.3/v2)**: Used for basic vision and teleoperation.
2. **Ascamera (RGB-D)**: Used for advanced vision tasks like object detection and shape classification using depth information.

---

## 1. Raspberry Pi Camera (v4l2)

Used for standard color streaming.

### Launching on the Robot (Pi)

```bash
ros2 launch lidarbot_bringup camera_launch.py
```

### Configuration
- **Package**: `v4l2_camera`
- **Output**: 640x480 resolution
- **Topic**: `/camera/image_raw`

---

## 2. Ascamera RGB-D System

Used for depth-aware vision applications.

### Launching on the Robot (Pi)

```bash
ros2 launch ascamera ascamera.launch.py
```

### Key Topics
- `/ascamera/color0/image_raw`: RGB Stream
- `/ascamera/depth0/image_raw`: Depth Stream (millimeters)
- `/ascamera/depth0/points`: 3D Point Cloud

---

## 3. Depth Imaging Deep Dive

The Ascamera (RGB-D) provides spatial perception by measuring the distance to objects in the environment.

### Data Formats and Encoding
- **16UC1 (Default)**: Unsigned 16-bit integer. Each pixel value represents the distance in **millimeters**.
    - Max range: 65,535 mm.
    - Standard for the `/ascamera/depth0/image_raw` topic.
- **32FC1**: 32-bit floating point. Each pixel value represents the distance in **meters**.
    - Common for processed point clouds or specific vision nodes.

### Depth-RGB Alignment
Since the RGB and Depth sensors are physically offset, ROS uses **Registration** to align the pixels.
- Topic: `/ascamera/depth0/image_raw` is usually aligned with `/ascamera/color0/image_raw`.

---

## 4. Laptop Visualization (Depth & RGB)

### Method 1: RViz2 (Professional Visualization)
RViz2 is the best tool for seeing 3D data.

1. **Launch RViz2 on Laptop**:
   ```bash
   ros2 run rviz2 rviz2
   ```
2. **Fixed Frame**: Set `Global Options -> Fixed Frame` to `camera_link` or `base_link`.
3. **Add Displays**:
   - **Image**: Add `Image` display. Set topic to `/ascamera/color0/image_raw` (RGB).
   - **Depth Map**: Add `Image` display. Set topic to `/ascamera/depth0/image_raw`. 
     *Note: It might look black if the range scaling isn't set.*
   - **Point Cloud**: Add `PointCloud2`. Set topic to `/ascamera/depth0/points`.
     - *Tip: Set "Style" to "Points" and "Size" to "0.01" to see the 3D structure.*

### Method 2: rqt_image_view (Quick Check)
```bash
ros2 run rqt_image_view rqt_image_view
```
- Select `/ascamera/depth0/image_raw` from the dropdown.
- Check the "Normalize" box to automatically scale the depth values for visibility (converts high-range integers to gray/white).

---

## 5. Vision System Integration

Our `vision_system` package uses depth data for intelligent filtering.

### Depth-Based Filtering Logic
In `rgbd_object_detector.py`, we filter objects based on their distance relative to the background:
1. **Estimate Background**: Samples the edges of the depth image to find the median "floor" distance.
2. **ROI Masking**: Only pixels closer than the `background_margin_mm` are considered potential objects.
3. **Distance Calculation**: The median depth of a detected color contour is calculated to provide real-time distance: 
   `"blue ball at 450mm"`.

### Running Vision Scripts with Depth
```bash
# SSH into Pi with X11
ssh -X pi@<robot_ip>

# Start the RGB-D detector
ros2 run vision_system rgbd_object_detector --ros-args -p color_preset:=red
```

---

## 6. Troubleshooting Camera / Depth Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| **No image in RViz** | Frame mismatch | Set "Global Options -> Fixed Frame" to `base_link` or `odom`. |
| **Laggy video** | Network bandwidth | Reduce resolution or frame rate in the launch file. |
| **Device not found** | Permissions or cabling | Run `ls /dev/video*`. Use `sudo chmod 666 /dev/video0`. |
| **X11 Windows don't appear** | SSH flag missing | Reconnect with `ssh -X` or `ssh -Y`. |
| **Ascamera depth is black** | Invalid range | Objects might be too close (min ~10cm) or too far. |

---

## Developer Performance Tip
To view images without the overhead of RViz, use:
```bash
ros2 run rqt_image_view rqt_image_view
```
This tool is lightweight and provides a simple dropdown to select camera topics.
