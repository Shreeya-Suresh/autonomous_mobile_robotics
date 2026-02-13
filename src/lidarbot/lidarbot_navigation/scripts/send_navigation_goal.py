#!/usr/bin/env python3
"""
Send Navigation Goal - Simple script to send a goal pose to Nav2

Usage:
    python3 send_navigation_goal.py [x] [y] [yaw_degrees]

Example:
    python3 send_navigation_goal.py 2.0 1.5 90
"""

import sys
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


def create_goal_pose(navigator, x, y, yaw_degrees=0.0):
    """
    Create a PoseStamped goal in the map frame
    
    Args:
        navigator: BasicNavigator instance
        x: x position in meters
        y: y position in meters
        yaw_degrees: orientation in degrees (0 = facing +X axis)
    
    Returns:
        PoseStamped: Goal pose
    """
    import math
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # Position
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0
    
    # Orientation (convert yaw to quaternion)
    yaw_radians = math.radians(yaw_degrees)
    goal_pose.pose.orientation.z = math.sin(yaw_radians / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw_radians / 2.0)
    
    return goal_pose


def main():
    # Parse command line arguments
    if len(sys.argv) < 3:
        print("Usage: send_navigation_goal.py <x> <y> [yaw_degrees]")
        print("Example: send_navigation_goal.py 2.0 1.5 90")
        sys.exit(1)
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    # Initialize ROS2
    rclpy.init()
    navigator = BasicNavigator()
    
    # Wait for Nav2 to be ready
    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print("Nav2 is active!")
    
    # Create and send goal
    goal_pose = create_goal_pose(navigator, x, y, yaw)
    print(f"\nSending goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.1f}°")
    print(f"Goal frame: {goal_pose.header.frame_id}")
    
    navigator.goToPose(goal_pose)
    
    # Monitor progress
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f"Distance remaining: {feedback.distance_remaining:.2f} m")
            
            # Optionally cancel if taking too long
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #     navigator.cancelTask()
            #     print("Navigation timeout - cancelling task")
    
    # Check result
    result = navigator.getResult()
    if result == BasicNavigator.TaskResult.SUCCEEDED:
        print("\n✅ Goal succeeded!")
    elif result == BasicNavigator.TaskResult.CANCELED:
        print("\n❌ Goal was canceled!")
    elif result == BasicNavigator.TaskResult.FAILED:
        print("\n❌ Goal failed!")
    else:
        print(f"\n⚠️  Unknown result: {result}")
    
    # Shutdown
    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
