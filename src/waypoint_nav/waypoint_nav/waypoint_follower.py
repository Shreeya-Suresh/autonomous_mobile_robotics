#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def quat_to_yaw(q):
    """Convert quaternion to yaw (rad)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointFollower(Node):
    # FSM states
    ROTATE_TO_TARGET = 0
    DRIVE_TO_TARGET = 1
    ALIGN_FINAL_HEADING = 2
    DONE = 3

    def __init__(self):
        super().__init__('waypoint_follower')

        # -------------------------------
        # Waypoints (x, y, theta)
        # -------------------------------
        self.waypoints = [
            (0.0, 0.0, 0.0),          # Home
            (1.0, 0.0, 0.0),          # Station A
            (1.0, 1.0, math.pi / 2),  # Station B
            (0.0, 1.0, math.pi),      # Station C
            (0.0, 0.0, 0.0),          # Home
        ]

        self.current_wp = 0
        self.state = self.ROTATE_TO_TARGET

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Obstacle detection
        self.obstacle_ahead = False

        # -------------------------------
        # Parameters (tuned for clarity)
        # -------------------------------
        self.angle_tolerance = math.radians(5.0)
        self.distance_tolerance = 0.05

        self.kp_ang = 1.5
        self.kp_lin = 0.6

        self.max_ang_vel = 1.0
        self.max_lin_vel = 0.4

        self.obstacle_dist_thresh = 0.4

        # -------------------------------
        # ROS interfaces
        # -------------------------------
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info('Waypoint follower started')

    # -----------------------------------
    # Callbacks
    # -----------------------------------
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)

    def scan_cb(self, msg):
        self.obstacle_ahead = False
        mid = len(msg.ranges) // 2
        window = msg.ranges[mid - 10: mid + 10]

        for r in window:
            if msg.range_min < r < self.obstacle_dist_thresh:
                self.obstacle_ahead = True
                return

    # -----------------------------------
    # FSM update
    # -----------------------------------
    def update(self):
        if self.state == self.DONE:
            self.publish_cmd(0.0, 0.0)
            return

        if self.current_wp >= len(self.waypoints):
            self.state = self.DONE
            self.get_logger().info('Mission complete')
            return

        tx, ty, ttheta = self.waypoints[self.current_wp]

        dx = tx - self.x
        dy = ty - self.y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.yaw)

        if self.state == self.ROTATE_TO_TARGET:
            if abs(heading_error) < self.angle_tolerance:
                self.publish_cmd(0.0, 0.0)
                self.state = self.DRIVE_TO_TARGET
            else:
                ang = self.kp_ang * heading_error
                ang = max(-self.max_ang_vel, min(self.max_ang_vel, ang))
                self.publish_cmd(0.0, ang)

        elif self.state == self.DRIVE_TO_TARGET:
            if self.obstacle_ahead:
                self.publish_cmd(0.0, 0.5)
                return

            if distance < self.distance_tolerance:
                self.publish_cmd(0.0, 0.0)
                self.state = self.ALIGN_FINAL_HEADING
            else:
                lin = self.kp_lin * distance
                lin = min(self.max_lin_vel, lin)

                ang = self.kp_ang * heading_error
                ang = max(-self.max_ang_vel, min(self.max_ang_vel, ang))

                self.publish_cmd(lin, ang)

        elif self.state == self.ALIGN_FINAL_HEADING:
            final_err = normalize_angle(ttheta - self.yaw)
            if abs(final_err) < self.angle_tolerance:
                self.publish_cmd(0.0, 0.0)
                self.current_wp += 1
                self.state = self.ROTATE_TO_TARGET
            else:
                ang = self.kp_ang * final_err
                ang = max(-self.max_ang_vel, min(self.max_ang_vel, ang))
                self.publish_cmd(0.0, ang)

    # -----------------------------------
    # Command publisher
    # -----------------------------------
    def publish_cmd(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
