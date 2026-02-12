#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Simple teleop launch - just hardware + keyboard control.
    No twist_mux, direct connection to diff_controller.
    """

    # Package paths
    pkg_bringup = FindPackageShare(package="lidarbot_bringup").find("lidarbot_bringup")

    # 1. Hardware interface (robot_state_publisher, controller_manager, controllers, lidar)
    start_hardware_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_bringup, "launch", "lidarbot_check_hardware.launch.py")]
        ),
        launch_arguments={
            "use_sim_time": "False",
            "use_ros2_control": "True",
        }.items(),
    )

    # 2. Keyboard teleop - directly publishes to diff_controller
    start_teleop_keyboard_cmd = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_keyboard",
        output="screen",
        prefix="xterm -e",  # Run in separate terminal for keyboard input
        remappings=[
            ("cmd_vel", "diff_controller/cmd_vel_unstamped"),
        ],
    )

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(start_hardware_cmd)
    ld.add_action(start_teleop_keyboard_cmd)

    return ld
