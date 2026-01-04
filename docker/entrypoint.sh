#!/usr/bin/env bash
set -e

# ---------- Source ROS ----------
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ROS Humble not found!"
    exit 1
fi

# ---------- Source workspace overlay if built ----------
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# ---------- Execute command ----------
exec "$@"
