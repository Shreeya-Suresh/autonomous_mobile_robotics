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

    # Paths
    pkg_bringup = FindPackageShare("lidarbot_bringup").find("lidarbot_bringup")
    pkg_description = FindPackageShare("lidarbot_description").find("lidarbot_description")
    pkg_navigation = FindPackageShare("lidarbot_navigation").find("lidarbot_navigation")

    # Config Files
    controller_params_file = os.path.join(pkg_bringup, "config/controllers.yaml")
    ekf_params_file = os.path.join(pkg_navigation, "config/ekf.yaml")
    nav2_params_file = os.path.join(pkg_navigation, "config/nav2_params.yaml")
    map_file = os.path.join(pkg_navigation, "maps", "real_map3.yaml")
    rviz_config_file = os.path.join(pkg_navigation, "rviz", "nav2_default_view.rviz")

    # Launch Configs
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")

    # Arguments
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
    
    declare_map_cmd = DeclareLaunchArgument(
        name="map",
        default_value=map_file,
        description="Full path to map file to load"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name="params_file",
        default_value=nav2_params_file,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    # ----------------------------------------
    # 1. HARDWARE & TRANSFORMS
    # ----------------------------------------
    
    # Robot State Publisher
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

    # Controller Manager
    start_controller_manager_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_params_file],
    )

    # Spawners
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

    # Delayed Spawners
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
    
    # Lidar
    start_lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_bringup, "launch", "ydlidar_launch.py")]
        )
    )
    
    # EKF (Provides odom -> base_link)
    start_ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file, {"use_sim_time": use_sim_time}],
        remappings=[("/odometry/filtered", "/odom")]
    )

    # ----------------------------------------
    # 2. LOCALIZATION (AMCL + Map Server)
    # ----------------------------------------
    
    start_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_navigation, "launch", "localization_launch.py")]
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": use_sim_time,
            "params_file": LaunchConfiguration("params_file"),
            "autostart": "True",  # Ensure AMCL auto-starts
        }.items(),
    )

    # ----------------------------------------
    # 3. NAVIGATION (Planner + Controller)
    # ----------------------------------------
    
    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_navigation, "launch", "navigation_launch.py")]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": LaunchConfiguration("params_file"),
            "autostart": "True",
        }.items(),
    )

    # ----------------------------------------
    # 4. RVIZ
    # ----------------------------------------
    
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    ld = LaunchDescription()

    # Args
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)

    # Nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_delayed_controller_manager)
    ld.add_action(start_delayed_diff_drive_spawner)
    ld.add_action(start_delayed_joint_broadcaster_spawner)
    ld.add_action(start_delayed_imu_broadcaster_spawner)
    ld.add_action(start_lidar_cmd)
    ld.add_action(start_ekf_cmd)
    
    ld.add_action(start_localization_cmd)
    ld.add_action(start_navigation_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
