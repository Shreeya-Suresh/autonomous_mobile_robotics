#!/usr/bin/env python3
"""
Autonomous Navigation Launch File - Mapless SLAM + Nav2

Brings up the complete autonomous navigation stack:
- Hardware interface (motors, encoders, IMU)
- YDLidar
- SLAM (slam_toolbox in mapping mode)
- Nav2 stack (without AMCL)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package paths
    pkg_bringup = FindPackageShare(package="lidarbot_bringup").find("lidarbot_bringup")
    pkg_slam = FindPackageShare(package="lidarbot_slam").find("lidarbot_slam")
    pkg_navigation = FindPackageShare(package="lidarbot_navigation").find("lidarbot_navigation")

    # Configuration files
    nav2_params_file = os.path.join(pkg_navigation, "config", "nav2_params_mapless.yaml")
    slam_params_file = os.path.join(pkg_slam, "config", "mapper_params_online_async.yaml")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    # =====================================================================
    # 1. Hardware Interface
    # =====================================================================
    # Launches: robot_state_publisher, controller_manager, controllers, YDLidar
    start_hardware_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_bringup, "launch", "lidarbot_check_hardware.launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_ros2_control": "True",
        }.items(),
    )

    # =====================================================================
    # 2. SLAM - slam_toolbox (mapping mode)
    # =====================================================================
    # Provides: map->odom transform, /map topic
    start_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_slam, "launch", "online_async_launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    # =====================================================================
    # 3. Nav2 Stack (mapless)
    # =====================================================================
    # Launches: controller_server, planner_server, behavior_server,
    #           bt_navigator, velocity_smoother, waypoint_follower
    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_navigation, "launch", "navigation_launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "autostart": "true",
        }.items(),
    )

    # =====================================================================
    # 4. RViz for Visualization (Optional)
    # =====================================================================
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_slam, "rviz", "mapper_params_online_async.rviz")],
        condition=lambda context: False,  # Set to True to auto-launch RViz
    )

    # Build launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes in order
    ld.add_action(start_hardware_cmd)
    ld.add_action(start_slam_cmd)
    ld.add_action(start_navigation_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
