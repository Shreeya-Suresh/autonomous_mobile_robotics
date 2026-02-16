#!/usr/bin/env python3
"""
Waypoint Navigation Controller for ESP32 Robot
Navigates through predefined waypoints while publishing odometry and scan data
"""

import serial
import time
import math
import threading
from dataclasses import dataclass
from typing import List, Tuple

# =====================
# CONFIGURATION
# =====================

SERIAL_PORT = '/dev/ttyUSB0'  # Change to your ESP32 port (COM3 on Windows)
BAUD_RATE = 115200
LOOP_RATE = 100  # Hz (matches firmware 10ms loop)

# Robot physical parameters
WHEEL_RADIUS = 0.033  # meters (33mm typical, MEASURE YOUR WHEELS!)
WHEEL_BASE = 0.15     # meters (distance between wheels, MEASURE THIS!)
ENCODER_CPR = 360     # Counts per revolution (MEASURE/CHECK YOUR ENCODERS!)

# Navigation parameters
WAYPOINT_TOLERANCE = 0.05  # meters (5cm - how close to get to waypoint)
ANGLE_TOLERANCE = 5.0      # degrees (heading tolerance)
MAX_LINEAR_VEL = 0.3       # m/s
MAX_ANGULAR_VEL = 1.0      # rad/s

# Control gains (tune these!)
KP_LINEAR = 1.0
KP_ANGULAR = 2.0
KD_ANGULAR = 0.1  # Optional: for smoother turning

# =====================
# WAYPOINTS (X, Y in meters)
# =====================

WAYPOINTS = [
    (0.0, 0.0),    # Point A (starting position)
    (1.0, 0.0),    # Point B (1 meter forward)
    (1.0, 1.0),    # Point C (1 meter left)
    (0.0, 1.0),    # Point D (1 meter back)
]

# Alternative waypoint patterns (uncomment to try):
# Square pattern
# WAYPOINTS = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]

# Figure-8 pattern (requires more waypoints)
# WAYPOINTS = [(0, 0), (0.5, 0.5), (1, 0), (0.5, -0.5), (0, 0)]

# =====================
# DATA STRUCTURES
# =====================

@dataclass
class Pose:
    """Robot pose in 2D"""
    x: float = 0.0      # meters
    y: float = 0.0      # meters
    theta: float = 0.0  # radians

@dataclass
class Odometry:
    """Odometry data"""
    pose: Pose
    left_encoder: int = 0
    right_encoder: int = 0
    linear_vel: float = 0.0   # m/s
    angular_vel: float = 0.0  # rad/s

@dataclass
class ScanData:
    """Simulated laser scan data (placeholder for future sensor)"""
    ranges: List[float] = None
    angle_min: float = -3.14159
    angle_max: float = 3.14159
    angle_increment: float = 0.0174533  # 1 degree

# =====================
# ROBOT CONTROLLER
# =====================

class ESP32RobotController:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)  # Wait for ESP32 to initialize
        
        self.odom = Odometry(pose=Pose())
        self.scan = ScanData(ranges=[1.0] * 360)  # Simulated scan
        
        self.prev_left = 0
        self.prev_right = 0
        self.prev_time = time.time()
        
        self.running = False
        self.read_thread = None
        
        # For angular velocity derivative (if using KD)
        self.prev_angular_error = 0.0
        
    def start(self):
        """Start reading telemetry from ESP32"""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        print("✓ Controller started")
        
    def stop(self):
        """Stop all motors and reading"""
        self.running = False
        self.send_velocity(0, 0)
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        self.ser.close()
        print("✓ Controller stopped")
        
    def _read_loop(self):
        """Background thread to read telemetry"""
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line.startswith('D,'):
                        self._parse_telemetry(line)
            except Exception as e:
                print(f"Read error: {e}")
                
    def _parse_telemetry(self, line: str):
        """Parse telemetry: D,left_enc,right_enc,yaw"""
        try:
            parts = line.split(',')
            left_enc = int(parts[1])
            right_enc = int(parts[2])
            yaw_deg = float(parts[3])
            
            # Update odometry
            self._update_odometry(left_enc, right_enc, yaw_deg)
            
        except (ValueError, IndexError) as e:
            print(f"Parse error: {e}")
            
    def _update_odometry(self, left_enc: int, right_enc: int, yaw_deg: float):
        """Update robot pose from encoder and IMU data"""
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            return
            
        # Encoder deltas
        d_left = left_enc - self.prev_left
        d_right = right_enc - self.prev_right
        
        # Convert encoder counts to distance (meters)
        dist_left = (d_left / ENCODER_CPR) * (2 * math.pi * WHEEL_RADIUS)
        dist_right = (d_right / ENCODER_CPR) * (2 * math.pi * WHEEL_RADIUS)
        
        # Dead reckoning odometry
        dist_center = (dist_left + dist_right) / 2.0
        # d_theta = (dist_right - dist_left) / WHEEL_BASE  # From encoders
        
        # Use IMU yaw (more accurate)
        yaw_rad = math.radians(yaw_deg)
        
        # Update pose
        self.odom.pose.x += dist_center * math.cos(self.odom.pose.theta)
        self.odom.pose.y += dist_center * math.sin(self.odom.pose.theta)
        self.odom.pose.theta = yaw_rad
        
        # Calculate velocities
        self.odom.linear_vel = dist_center / dt
        d_theta = yaw_rad - self.odom.pose.theta
        self.odom.angular_vel = d_theta / dt
        
        # Update previous values
        self.prev_left = left_enc
        self.prev_right = right_enc
        self.prev_time = current_time
        self.odom.left_encoder = left_enc
        self.odom.right_encoder = right_enc
        
    def send_velocity(self, linear: float, angular: float):
        """
        Send velocity command to ESP32
        linear: m/s
        angular: rad/s
        """
        # Convert to wheel velocities (differential drive kinematics)
        v_left = linear - (angular * WHEEL_BASE / 2.0)
        v_right = linear + (angular * WHEEL_BASE / 2.0)
        
        # Convert m/s to rad/s for wheels
        w_left = v_left / WHEEL_RADIUS
        w_right = v_right / WHEEL_RADIUS
        
        # Send command: V,left_rad/s,right_rad/s
        cmd = f"V,{w_left:.3f},{w_right:.3f}\n"
        self.ser.write(cmd.encode('utf-8'))
        
    def get_pose(self) -> Pose:
        """Get current robot pose"""
        return self.odom.pose
        
    def get_odometry(self) -> Odometry:
        """Get full odometry data"""
        return self.odom
        
    def get_scan(self) -> ScanData:
        """Get scan data (simulated for now)"""
        return self.scan
        
    def publish_data(self):
        """Print odometry and scan data (placeholder for ROS publishing)"""
        print(f"ODOM: x={self.odom.pose.x:.3f}m, y={self.odom.pose.y:.3f}m, "
              f"θ={math.degrees(self.odom.pose.theta):.1f}°, "
              f"v={self.odom.linear_vel:.2f}m/s, ω={self.odom.angular_vel:.2f}rad/s")
        # In ROS, you would publish to /odom topic here
        
        # Simulated scan (replace with actual sensor data)
        # print(f"SCAN: {len(self.scan.ranges)} points, min={min(self.scan.ranges):.2f}m")
        # In ROS, you would publish to /scan topic here

# =====================
# NAVIGATION CONTROLLER
# =====================

class WaypointNavigator:
    def __init__(self, robot: ESP32RobotController):
        self.robot = robot
        self.current_waypoint_idx = 0
        
    def distance_to_waypoint(self, waypoint: Tuple[float, float]) -> float:
        """Calculate Euclidean distance to waypoint"""
        pose = self.robot.get_pose()
        dx = waypoint[0] - pose.x
        dy = waypoint[1] - pose.y
        return math.sqrt(dx**2 + dy**2)
        
    def angle_to_waypoint(self, waypoint: Tuple[float, float]) -> float:
        """Calculate angle to waypoint (radians)"""
        pose = self.robot.get_pose()
        dx = waypoint[0] - pose.x
        dy = waypoint[1] - pose.y
        return math.atan2(dy, dx)
        
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def navigate_to_waypoint(self, waypoint: Tuple[float, float]) -> bool:
        """
        Navigate to a single waypoint using proportional control
        Returns True when waypoint is reached
        """
        distance = self.distance_to_waypoint(waypoint)
        
        if distance < WAYPOINT_TOLERANCE:
            return True
            
        # Calculate desired heading
        desired_angle = self.angle_to_waypoint(waypoint)
        current_angle = self.robot.get_pose().theta
        angle_error = self.normalize_angle(desired_angle - current_angle)
        
        # Proportional control
        linear_vel = KP_LINEAR * distance
        angular_vel = KP_ANGULAR * angle_error
        
        # Optional: Add derivative term for smoother turning
        # angular_vel += KD_ANGULAR * (angle_error - self.robot.prev_angular_error) / dt
        
        # Clamp velocities
        linear_vel = max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, linear_vel))
        angular_vel = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular_vel))
        
        # Reduce linear velocity when turning sharply
        if abs(angle_error) > math.radians(30):
            linear_vel *= 0.5
        if abs(angle_error) > math.radians(60):
            linear_vel = 0  # Turn in place first
            
        self.robot.send_velocity(linear_vel, angular_vel)
        return False
        
    def run(self, waypoints: List[Tuple[float, float]]):
        """Navigate through all waypoints"""
        print(f"\n{'='*60}")
        print(f"STARTING WAYPOINT NAVIGATION")
        print(f"{'='*60}")
        print(f"Total waypoints: {len(waypoints)}")
        for i, wp in enumerate(waypoints):
            print(f"  {chr(65+i)}: ({wp[0]:.2f}, {wp[1]:.2f})")
        print(f"{'='*60}\n")
        
        for idx, waypoint in enumerate(waypoints):
            print(f"\n>>> Navigating to waypoint {chr(65+idx)}: ({waypoint[0]:.2f}, {waypoint[1]:.2f})")
            
            # Navigate to waypoint
            rate = 1.0 / LOOP_RATE
            while True:
                if self.navigate_to_waypoint(waypoint):
                    print(f"✓ Reached waypoint {chr(65+idx)}!")
                    break
                    
                # Publish data
                self.robot.publish_data()
                time.sleep(rate)
                
            # Pause at waypoint
            self.robot.send_velocity(0, 0)
            time.sleep(1.0)
            
        print(f"\n{'='*60}")
        print(f"NAVIGATION COMPLETE!")
        print(f"{'='*60}\n")
        
        # Stop robot
        self.robot.send_velocity(0, 0)

# =====================
# ALTERNATIVE CONTROLLERS
# =====================

# Uncomment to try pure pursuit controller (more advanced)
"""
class PurePursuitNavigator(WaypointNavigator):
    def __init__(self, robot: ESP32RobotController, lookahead_distance: float = 0.3):
        super().__init__(robot)
        self.lookahead_distance = lookahead_distance
        
    def find_lookahead_point(self, waypoint: Tuple[float, float]) -> Tuple[float, float]:
        '''Find point on path at lookahead distance'''
        pose = self.robot.get_pose()
        dx = waypoint[0] - pose.x
        dy = waypoint[1] - pose.y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < self.lookahead_distance:
            return waypoint
            
        # Scale to lookahead distance
        scale = self.lookahead_distance / dist
        return (pose.x + dx * scale, pose.y + dy * scale)
        
    def navigate_to_waypoint(self, waypoint: Tuple[float, float]) -> bool:
        distance = self.distance_to_waypoint(waypoint)
        
        if distance < WAYPOINT_TOLERANCE:
            return True
            
        # Get lookahead point
        lookahead = self.find_lookahead_point(waypoint)
        
        # Calculate curvature to lookahead point
        pose = self.robot.get_pose()
        dx = lookahead[0] - pose.x
        dy = lookahead[1] - pose.y
        
        # Transform to robot frame
        dx_robot = dx * math.cos(-pose.theta) - dy * math.sin(-pose.theta)
        dy_robot = dx * math.sin(-pose.theta) + dy * math.cos(-pose.theta)
        
        # Calculate curvature
        curvature = 2 * dy_robot / (self.lookahead_distance ** 2)
        
        # Set velocities
        linear_vel = MAX_LINEAR_VEL
        angular_vel = curvature * linear_vel
        
        # Clamp angular velocity
        angular_vel = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular_vel))
        
        self.robot.send_velocity(linear_vel, angular_vel)
        return False
"""

# =====================
# MAIN PROGRAM
# =====================

def main():
    print("ESP32 Robot Waypoint Navigator")
    print("=" * 60)
    
    # Initialize robot controller
    try:
        robot = ESP32RobotController(SERIAL_PORT, BAUD_RATE)
        robot.start()
        time.sleep(1)  # Let telemetry stabilize
        
    except serial.SerialException as e:
        print(f"❌ Failed to connect to ESP32: {e}")
        print(f"Check port {SERIAL_PORT} and make sure ESP32 is connected.")
        return
        
    try:
        # Create navigator
        navigator = WaypointNavigator(robot)
        
        # Option 1: Run predefined waypoints
        navigator.run(WAYPOINTS)
        
        # Option 2: Single waypoint test (uncomment to try)
        # print("Testing single waypoint...")
        # navigator.run([(1.0, 0.0)])
        
        # Option 3: Interactive waypoint entry (uncomment to try)
        # custom_waypoints = []
        # while True:
        #     x = input("Enter X (or 'done'): ")
        #     if x.lower() == 'done':
        #         break
        #     y = input("Enter Y: ")
        #     custom_waypoints.append((float(x), float(y)))
        # navigator.run(custom_waypoints)
        
    except KeyboardInterrupt:
        print("\n\n⚠ Navigation interrupted by user")
        
    finally:
        # Clean shutdown
        robot.stop()
        print("Program terminated.")

if __name__ == "__main__":
    main()