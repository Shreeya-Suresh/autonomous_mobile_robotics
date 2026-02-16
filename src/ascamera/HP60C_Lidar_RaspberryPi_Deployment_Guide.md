# HP60C Depth Camera + YDLidar Complete Deployment Guide

**Platform:** Raspberry Pi\
**OS:** Ubuntu 22.04\
**ROS Version:** ROS2 Humble

------------------------------------------------------------------------

# 1️⃣ System Preparation

## Update System

``` bash
sudo apt update
sudo apt upgrade -y
```

## Install Required ROS Dependencies

``` bash
sudo apt install -y     ros-humble-pcl-conversions     ros-humble-pcl-ros     ros-humble-image-transport     ros-humble-example-interfaces     usbutils     v4l-utils
```

------------------------------------------------------------------------

# 2️⃣ Workspace Setup

``` bash
cd ~
mkdir -p ros2_ws/src
cd ros2_ws
```

Place your repository inside:

    ros2_ws/src/autonomous_mobile_robotics

------------------------------------------------------------------------

# 3️⃣ Fix HP60C Configuration Path (CRITICAL)

Your workspace structure likely looks like:

    ros2_ws
     └── src
         └── autonomous_mobile_robotics
             └── src
                 └── ascamera

The launch file must point to:

    /home/rpi/ros2_ws/src/autonomous_mobile_robotics/src/ascamera/configurationfiles

## Edit Launch File

``` bash
nano ~/ros2_ws/src/autonomous_mobile_robotics/src/ascamera/launch/hp60c.launch.py
```

Find:

``` python
{"confiPath": "/home/rpi/ros2_ws/src/autonomous_mobile_robotics/ascamera/configurationfiles"},
```

Replace with:

``` python
{"confiPath": "/home/rpi/ros2_ws/src/autonomous_mobile_robotics/src/ascamera/configurationfiles"},
```

Save and exit.

------------------------------------------------------------------------

# 4️⃣ Build Workspace

``` bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

------------------------------------------------------------------------

# 5️⃣ Fix Camera Permissions

If you get `uvc_open: Access denied`:

``` bash
sudo usermod -aG video $USER
sudo reboot
```

Temporary fix:

``` bash
sudo chmod 666 /dev/video*
```

Verify devices:

``` bash
ls /dev/video*
```

------------------------------------------------------------------------

# 6️⃣ Launch HP60C Camera

``` bash
source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py
```

Check topics:

``` bash
ros2 topic list | grep ascamera
```

Test stream:

``` bash
ros2 topic echo /ascamera_hp60c/rgb/image_raw
```

------------------------------------------------------------------------

# 7️⃣ Launch RViz for Camera

In a new terminal:

``` bash
rviz2
```

Add:

-   Image display → `/ascamera_hp60c/rgb/image_raw`
-   PointCloud2 display → `/ascamera_hp60c/depth/points`

------------------------------------------------------------------------

# 8️⃣ Fix YDLidar Serial Port

Check ports:

``` bash
ls /dev/ttyUSB*
```

If permission error:

``` bash
sudo usermod -aG dialout $USER
sudo reboot
```

Or temporary:

``` bash
sudo chmod 666 /dev/ttyUSB0
```

------------------------------------------------------------------------

# 9️⃣ Launch YDLidar

``` bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

Or from your robot bringup:

``` bash
ros2 launch lidarbot_bringup lidarbot_check_hardware.launch.py
```

------------------------------------------------------------------------

# 🔟 Verify Lidar Data

``` bash
ros2 topic list | grep scan
ros2 topic echo /scan
```

In RViz:

-   Add LaserScan
-   Topic: `/scan`
-   Fixed Frame: `laser_frame`

------------------------------------------------------------------------

# 🚀 Final Working Order

Terminal 1:

``` bash
ros2 launch ascamera hp60c.launch.py
```

Terminal 2:

``` bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

Terminal 3:

``` bash
rviz2
```

------------------------------------------------------------------------

# ✅ Expected Working Topics

Camera: - /ascamera_hp60c/rgb/image_raw -
/ascamera_hp60c/depth/image_raw - /ascamera_hp60c/depth/points

Lidar: - /scan

------------------------------------------------------------------------

# 🧠 If Camera Segfaults (-11)

It is almost always: - Wrong `confiPath` - Missing configurationfiles
directory - Missing video permissions

------------------------------------------------------------------------

System should now be fully operational.
