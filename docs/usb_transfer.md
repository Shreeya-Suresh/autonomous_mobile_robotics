# Offline ROS 2 Workspace Transfer & Build Guide

Ubuntu Server 22.04 (No Internet) → Laptop (With Internet) → USB → Back
to Server

This document explains how to safely transfer, update, and rebuild your
ROS 2 workspace when your robot machine has no internet. It also covers
directory mismatches, common failures, and deep troubleshooting.

------------------------------------------------------------------------

# 1. Current Directory Situation

Robot (offline machine):

    ros2_ws/
     └── src/
         └── autonomous_mobile_robotics/
             └── src/
                 ├── pkg1
                 ├── pkg2
                 └── ...

Laptop:

    ros2_ws/
     └── src/
         ├── pkg1
         ├── pkg2
         └── ...

Only packages matter --- not build artifacts.

------------------------------------------------------------------------

# 2. Golden Rules

Never transfer:

    build/
    install/
    log/

Only transfer:

    src/

------------------------------------------------------------------------

# 3. Workflow Overview

1.  Copy workspace → USB → Laptop\
2.  Modify code on laptop\
3.  Build & test on laptop\
4.  Copy updated src → USB\
5.  Replace src on robot\
6.  Build on robot

------------------------------------------------------------------------

# 4. Prepare Workspace on Robot

``` bash
cd ~/ros2_ws
rm -rf build install log
```

------------------------------------------------------------------------

# 5. Copy to USB

Find USB:

``` bash
lsblk
```

Mount:

``` bash
sudo mount /dev/sda1 /mnt
```

Copy:

``` bash
cp -r ~/ros2_ws /mnt/
sync
```

Unmount:

``` bash
sudo umount /mnt
```

------------------------------------------------------------------------

# 6. Copy to Laptop

``` bash
cp -r /media/<user>/<usb>/ros2_ws ~/
```

------------------------------------------------------------------------

# 7. Fix Directory Structure (Optional)

Flatten:

``` bash
cd ~/ros2_ws/src/autonomous_mobile_robotics/src
mv * ../../
cd ../..
rm -rf autonomous_mobile_robotics
```

------------------------------------------------------------------------

# 8. Install Dependencies on Laptop

``` bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

If needed:

``` bash
sudo rosdep init
rosdep update
```

------------------------------------------------------------------------

# 9. Build on Laptop

``` bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Verify:

``` bash
ros2 pkg list
```

------------------------------------------------------------------------

# 10. Copy Back to Robot

``` bash
rm -rf ~/ros2_ws/src
cp -r /usb/ros2_ws/src ~/ros2_ws/
```

------------------------------------------------------------------------

# 11. Build on Robot

``` bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

------------------------------------------------------------------------

# 12. Offline Dependency Install

On laptop:

``` bash
apt download ros-humble-<package>
```

On robot:

``` bash
sudo dpkg -i ros-humble-*.deb
```

------------------------------------------------------------------------

# 13. Environment Setup

``` bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

------------------------------------------------------------------------

# 14. Common Problems

## ros2 not found

    source /opt/ros/humble/setup.bash

## colcon not found

    sudo apt install python3-colcon-common-extensions

## Package not detected

Check:

    package.xml
    CMakeLists.txt

Rebuild after cleaning:

    rm -rf build install log
    colcon build

------------------------------------------------------------------------

# 15. Verify System

``` bash
ros2 pkg list
ros2 node list
ros2 topic list
```

------------------------------------------------------------------------

# 16. USB Safety

Always:

``` bash
sync
sudo umount /mnt
```

------------------------------------------------------------------------

# 17. Recommended Workflow

Laptop = Development\
Robot = Runtime

Process:

    Code → Laptop Build → Test → Copy src → Robot Build → Run

------------------------------------------------------------------------

# END
