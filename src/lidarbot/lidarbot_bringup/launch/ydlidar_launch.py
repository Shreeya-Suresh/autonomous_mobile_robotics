
import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_lidarbot_bringup = get_package_share_directory('lidarbot_bringup')
    
    # Parameter file
    params_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_lidarbot_bringup, 'config', 'ydlidar.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        namespace='/',
    )

    return LaunchDescription([
        params_declare,
        driver_node,
    ])
