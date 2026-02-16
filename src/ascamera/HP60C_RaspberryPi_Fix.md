# HP60C Depth Camera Fix on Raspberry Pi (ROS 2 Humble)

## Problem

When launching:

    ros2 launch ascamera hp60c.launch.py

The following error appears:

    can't not open dir:/home/rpi/ros2_ws/src/autonomous_mobile_robotics/ascamera/configurationfiles
    cannot find config file
    process has died (exit code -11)

------------------------------------------------------------------------

## Root Cause

Your workspace structure is:

    ros2_ws
     └── src
         └── autonomous_mobile_robotics
             └── src
                 └── ascamera
                     └── configurationfiles

But the launch file was pointing to:

    /home/rpi/ros2_ws/src/autonomous_mobile_robotics/ascamera/configurationfiles

Notice the missing `src/` in the middle.

Correct path must be:

    /home/rpi/ros2_ws/src/autonomous_mobile_robotics/src/ascamera/configurationfiles

------------------------------------------------------------------------

# Step-by-Step Fix

## 1️⃣ Edit Launch File

Open:

    nano ~/ros2_ws/src/autonomous_mobile_robotics/src/ascamera/launch/hp60c.launch.py

Find this parameter:

``` python
{"confiPath": "..."},
```

Replace it with:

``` python
{"confiPath": "/home/rpi/ros2_ws/src/autonomous_mobile_robotics/src/ascamera/configurationfiles"},
```

Save and exit.

------------------------------------------------------------------------

## 2️⃣ Clean and Rebuild Workspace

    cd ~/ros2_ws
    rm -rf build install log
    colcon build --symlink-install
    source install/setup.bash

------------------------------------------------------------------------

## 3️⃣ Launch Camera

    ros2 launch ascamera hp60c.launch.py

If successful, you should see:

    open camera success
    start stream success

------------------------------------------------------------------------

## 4️⃣ Verify Camera Topics

Open a new terminal:

    source ~/ros2_ws/install/setup.bash
    ros2 topic list

Look for:

    /ascamera_hp60c/color/image_raw
    /ascamera_hp60c/depth/image_raw
    /ascamera_hp60c/points

Check frame rate:

    ros2 topic hz /ascamera_hp60c/color/image_raw

------------------------------------------------------------------------

## 5️⃣ View in RViz

    rviz2

Add:

-   Image → topic `/ascamera_hp60c/color/image_raw`
-   PointCloud2 → topic `/ascamera_hp60c/points`

Set Fixed Frame to:

    camera_link

or

    ascamera_hp60c_camera_link

------------------------------------------------------------------------

# Why It Was Crashing

When the config file path was wrong:

-   SDK failed to load config
-   Continued with null pointer
-   Caused segmentation fault (exit code -11)

Fixing the path resolves the crash.

------------------------------------------------------------------------

✅ After this fix, HP60C should stream properly on Raspberry Pi with ROS
2 Humble.
