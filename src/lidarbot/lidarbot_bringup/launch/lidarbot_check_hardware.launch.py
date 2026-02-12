import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Path retrieval
    pkg_path = FindPackageShare(package="lidarbot_bringup").find("lidarbot_bringup")
    pkg_description = FindPackageShare(package="lidarbot_description").find("lidarbot_description")
    
    controller_params_file = os.path.join(pkg_path, "config/controllers.yaml")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        name="use_ros2_control",
        default_value="True",
        description="Use ros2_control if true",
    )

    # 1. Robot State Publisher (URDF)
    start_robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, "launch", "robot_state_publisher_launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
        }.items(),
    )

    robot_description = Command(
        ["ros2 param get --hide-type /robot_state_publisher robot_description"]
    )

    # 2. Controller Manager (The Hardware Interface)
    start_controller_manager_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_params_file],
    )

    # 3. Controllers
    start_diff_controller_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
    )

    start_joint_broadcaster_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster", "--controller-manager", "/controller_manager"],
    )

    start_imu_broadcaster_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Delayed start of spawners
    start_delayed_controller_manager = TimerAction(
        period=2.0, actions=[start_controller_manager_cmd]
    )

    start_delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_diff_controller_cmd],
        )
    )

    start_delayed_joint_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_joint_broadcaster_cmd],
        )
    )

    start_delayed_imu_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_controller_manager_cmd,
            on_start=[start_imu_broadcaster_cmd],
        )
    )
    
    # 4. YDLidar (Optional, but good for verification)
    start_lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_path, "launch", "ydlidar_launch.py")]
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_delayed_controller_manager)
    ld.add_action(start_delayed_diff_drive_spawner)
    ld.add_action(start_delayed_joint_broadcaster_spawner)
    ld.add_action(start_delayed_imu_broadcaster_spawner)
    ld.add_action(start_lidar_cmd)

    return ld
