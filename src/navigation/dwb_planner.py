#!/usr/bin/env python3
"""
Dynamic Window Approach (DWB) Local Planner for ESP32 Robot
Implements advanced local planning with obstacle avoidance and trajectory optimization
"""

import serial
import time
import math
import threading
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from collections import deque

# =====================
# CONFIGURATION
# =====================

SERIAL_PORT = '/dev/ttyUSB0'  # Change to your ESP32 port
BAUD_RATE = 115200
LOOP_RATE = 20  # Hz (DWB planning rate)

# Robot physical parameters
WHEEL_RADIUS = 0.033  # meters
WHEEL_BASE = 0.15     # meters
ENCODER_CPR = 360     # Counts per revolution

# Robot constraints (TUNE THESE!)
MAX_LINEAR_VEL = 0.5      # m/s
MIN_LINEAR_VEL = -0.2     # m/s (allow reverse)
MAX_ANGULAR_VEL = 2.0     # rad/s
MAX_LINEAR_ACC = 0.5      # m/s²
MAX_ANGULAR_ACC = 2.0     # rad/s²

# DWB parameters
SIM_TIME = 2.0            # seconds (trajectory simulation time)
SIM_GRANULARITY = 0.1     # seconds (time step for simulation)
VEL_SAMPLES = 15          # number of velocity samples
ANGULAR_SAMPLES = 20      # number of angular velocity samples

# Trajectory scoring weights (TUNE FOR BEHAVIOR!)
WEIGHT_PATH_DISTANCE = 32.0   # Follow global path
WEIGHT_GOAL_DISTANCE = 24.0   # Reach goal
WEIGHT_OBSTACLE = 10.0        # Avoid obstacles
WEIGHT_VELOCITY = 1.0         # Prefer higher velocities
WEIGHT_SMOOTHNESS = 5.0       # Smooth velocity changes

# Safety parameters
ROBOT_RADIUS = 0.2        # meters (for collision checking)
SAFETY_MARGIN = 0.05      # meters (extra safety buffer)
MIN_OBSTACLE_DISTANCE = 0.3  # meters (emergency stop distance)

# Navigation parameters
GOAL_TOLERANCE_XY = 0.1   # meters
GOAL_TOLERANCE_YAW = 0.15 # radians (~8 degrees)

# =====================
# DATA STRUCTURES
# =====================

@dataclass
class Pose:
    """Robot pose in 2D"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    
    def copy(self):
        return Pose(self.x, self.y, self.theta)

@dataclass
class Twist:
    """Velocity command"""
    linear: float = 0.0   # m/s
    angular: float = 0.0  # rad/s

@dataclass
class Odometry:
    """Odometry data"""
    pose: Pose
    twist: Twist = field(default_factory=Twist)
    left_encoder: int = 0
    right_encoder: int = 0

@dataclass
class Obstacle:
    """Point obstacle"""
    x: float
    y: float
    radius: float = 0.1

@dataclass
class Trajectory:
    """Simulated trajectory"""
    poses: List[Pose] = field(default_factory=list)
    twist: Twist = field(default_factory=Twist)
    cost: float = float('inf')
    
    # Individual cost components for debugging
    path_cost: float = 0.0
    goal_cost: float = 0.0
    obstacle_cost: float = 0.0
    velocity_cost: float = 0.0
    smoothness_cost: float = 0.0

@dataclass
class Path:
    """Global path (sequence of poses)"""
    poses: List[Pose] = field(default_factory=list)

# =====================
# SIMULATED LASER SCANNER
# =====================

class SimulatedLaserScanner:
    """
    Simulates a laser scanner with obstacles
    Replace this with real sensor data integration
    """
    def __init__(self):
        self.obstacles = []
        self.scan_range = 3.0  # meters
        self.num_rays = 360
        
    def add_obstacle(self, x: float, y: float, radius: float = 0.1):
        """Add an obstacle to the environment"""
        self.obstacles.append(Obstacle(x, y, radius))
        
    def get_scan(self, pose: Pose) -> List[float]:
        """
        Get simulated laser scan from current pose
        Returns list of ranges (one per ray)
        """
        ranges = [self.scan_range] * self.num_rays
        
        for i in range(self.num_rays):
            angle = pose.theta + (i - self.num_rays/2) * (2*math.pi / self.num_rays)
            
            # Cast ray and find closest obstacle
            for obs in self.obstacles:
                # Vector from robot to obstacle
                dx = obs.x - pose.x
                dy = obs.y - pose.y
                
                # Check if obstacle is in direction of ray
                obs_angle = math.atan2(dy, dx)
                angle_diff = abs(self._normalize_angle(obs_angle - angle))
                
                if angle_diff < math.radians(1):  # Within 1 degree
                    dist = math.sqrt(dx**2 + dy**2) - obs.radius
                    ranges[i] = min(ranges[i], max(0.0, dist))
                    
        return ranges
        
    def get_obstacles_in_range(self, pose: Pose, max_range: float) -> List[Obstacle]:
        """Get all obstacles within range of robot"""
        nearby = []
        for obs in self.obstacles:
            dx = obs.x - pose.x
            dy = obs.y - pose.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist <= max_range:
                nearby.append(obs)
        return nearby
        
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

# =====================
# ESP32 ROBOT INTERFACE
# =====================

class ESP32RobotController:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)
        
        self.odom = Odometry(pose=Pose(), twist=Twist())
        self.current_cmd = Twist()
        
        self.prev_left = 0
        self.prev_right = 0
        self.prev_time = time.time()
        
        self.running = False
        self.read_thread = None
        
        # Velocity history for smoothness calculation
        self.velocity_history = deque(maxlen=5)
        
    def start(self):
        """Start reading telemetry"""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        print("✓ ESP32 controller started")
        
    def stop(self):
        """Stop robot and close connection"""
        self.running = False
        self.send_velocity(Twist(0, 0))
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        self.ser.close()
        
    def _read_loop(self):
        """Background telemetry reading"""
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line.startswith('D,'):
                        self._parse_telemetry(line)
            except Exception as e:
                print(f"Read error: {e}")
                
    def _parse_telemetry(self, line: str):
        """Parse: D,left_enc,right_enc,yaw"""
        try:
            parts = line.split(',')
            left_enc = int(parts[1])
            right_enc = int(parts[2])
            yaw_deg = float(parts[3])
            
            self._update_odometry(left_enc, right_enc, yaw_deg)
            
        except (ValueError, IndexError):
            pass
            
    def _update_odometry(self, left_enc: int, right_enc: int, yaw_deg: float):
        """Update pose from encoders and IMU"""
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            return
            
        # Encoder deltas
        d_left = left_enc - self.prev_left
        d_right = right_enc - self.prev_right
        
        # Distance traveled
        dist_left = (d_left / ENCODER_CPR) * (2 * math.pi * WHEEL_RADIUS)
        dist_right = (d_right / ENCODER_CPR) * (2 * math.pi * WHEEL_RADIUS)
        dist_center = (dist_left + dist_right) / 2.0
        
        # Update pose using IMU yaw
        yaw_rad = math.radians(yaw_deg)
        self.odom.pose.x += dist_center * math.cos(yaw_rad)
        self.odom.pose.y += dist_center * math.sin(yaw_rad)
        self.odom.pose.theta = yaw_rad
        
        # Calculate velocities
        self.odom.twist.linear = dist_center / dt
        self.odom.twist.angular = (yaw_rad - self.prev_time) / dt if hasattr(self, 'prev_yaw') else 0.0
        self.prev_yaw = yaw_rad
        
        # Store velocity history
        self.velocity_history.append((self.odom.twist.linear, self.odom.twist.angular))
        
        # Update state
        self.prev_left = left_enc
        self.prev_right = right_enc
        self.prev_time = current_time
        self.odom.left_encoder = left_enc
        self.odom.right_encoder = right_enc
        
    def send_velocity(self, twist: Twist):
        """Send velocity command to ESP32"""
        # Convert to wheel velocities
        v_left = twist.linear - (twist.angular * WHEEL_BASE / 2.0)
        v_right = twist.linear + (twist.angular * WHEEL_BASE / 2.0)
        
        # Convert to rad/s
        w_left = v_left / WHEEL_RADIUS
        w_right = v_right / WHEEL_RADIUS
        
        cmd = f"V,{w_left:.3f},{w_right:.3f}\n"
        self.ser.write(cmd.encode('utf-8'))
        
        self.current_cmd = twist
        
    def get_pose(self) -> Pose:
        return self.odom.pose
        
    def get_odometry(self) -> Odometry:
        return self.odom
        
    def get_current_velocity(self) -> Twist:
        return self.odom.twist

# =====================
# DWB LOCAL PLANNER
# =====================

class DWBLocalPlanner:
    """
    Dynamic Window Approach Local Planner
    Plans collision-free trajectories that follow a global path
    """
    
    def __init__(self, robot: ESP32RobotController, scanner: SimulatedLaserScanner):
        self.robot = robot
        self.scanner = scanner
        self.global_path = Path()
        
        # Current goal
        self.current_goal = None
        self.goal_reached = False
        
        # Best trajectory for visualization
        self.best_trajectory = None
        
    def set_global_path(self, path: Path):
        """Set the global path to follow"""
        self.global_path = path
        if path.poses:
            self.current_goal = path.poses[-1]
            self.goal_reached = False
            
    def compute_velocity_command(self) -> Optional[Twist]:
        """
        Main DWB planning function
        Returns the best velocity command based on DWB algorithm
        """
        if not self.current_goal:
            return Twist(0, 0)
            
        # Check if goal reached
        if self._is_goal_reached():
            self.goal_reached = True
            return Twist(0, 0)
            
        current_pose = self.robot.get_pose()
        current_vel = self.robot.get_current_velocity()
        
        # Generate velocity samples within dynamic window
        velocity_samples = self._generate_velocity_samples(current_vel)
        
        if not velocity_samples:
            print("⚠ No valid velocity samples!")
            return Twist(0, 0)
            
        # Simulate trajectories and score them
        trajectories = []
        for twist in velocity_samples:
            traj = self._simulate_trajectory(current_pose, twist)
            
            # Check for collision
            if self._is_trajectory_safe(traj):
                self._score_trajectory(traj, current_pose, current_vel)
                trajectories.append(traj)
                
        if not trajectories:
            print("⚠ No collision-free trajectories! Emergency stop.")
            return Twist(0, 0)
            
        # Select best trajectory
        self.best_trajectory = min(trajectories, key=lambda t: t.cost)
        
        # Debug output
        self._print_debug_info(self.best_trajectory, len(trajectories))
        
        return self.best_trajectory.twist
        
    def _generate_velocity_samples(self, current_vel: Twist) -> List[Twist]:
        """
        Generate velocity samples within dynamic window
        Dynamic window respects acceleration limits
        """
        samples = []
        
        # Calculate achievable velocity ranges (dynamic window)
        dt = 1.0 / LOOP_RATE
        
        v_min = max(MIN_LINEAR_VEL, 
                   current_vel.linear - MAX_LINEAR_ACC * dt)
        v_max = min(MAX_LINEAR_VEL, 
                   current_vel.linear + MAX_LINEAR_ACC * dt)
        
        w_min = max(-MAX_ANGULAR_VEL, 
                   current_vel.angular - MAX_ANGULAR_ACC * dt)
        w_max = min(MAX_ANGULAR_VEL, 
                   current_vel.angular + MAX_ANGULAR_ACC * dt)
        
        # Sample velocities within dynamic window
        v_step = (v_max - v_min) / VEL_SAMPLES if VEL_SAMPLES > 1 else 0
        w_step = (w_max - w_min) / ANGULAR_SAMPLES if ANGULAR_SAMPLES > 1 else 0
        
        for i in range(VEL_SAMPLES):
            v = v_min + i * v_step
            for j in range(ANGULAR_SAMPLES):
                w = w_min + j * w_step
                samples.append(Twist(v, w))
                
        # Always include stop command
        samples.append(Twist(0, 0))
        
        return samples
        
    def _simulate_trajectory(self, start_pose: Pose, twist: Twist) -> Trajectory:
        """
        Simulate trajectory forward in time using constant velocity model
        """
        traj = Trajectory(twist=twist)
        
        pose = start_pose.copy()
        num_steps = int(SIM_TIME / SIM_GRANULARITY)
        
        for _ in range(num_steps):
            # Simple differential drive model
            pose.x += twist.linear * math.cos(pose.theta) * SIM_GRANULARITY
            pose.y += twist.linear * math.sin(pose.theta) * SIM_GRANULARITY
            pose.theta += twist.angular * SIM_GRANULARITY
            pose.theta = self._normalize_angle(pose.theta)
            
            traj.poses.append(pose.copy())
            
        return traj
        
    def _is_trajectory_safe(self, traj: Trajectory) -> bool:
        """
        Check if trajectory collides with obstacles
        """
        # Get nearby obstacles
        obstacles = self.scanner.get_obstacles_in_range(
            self.robot.get_pose(), 
            SIM_TIME * MAX_LINEAR_VEL + 1.0
        )
        
        for pose in traj.poses:
            for obs in obstacles:
                dx = obs.x - pose.x
                dy = obs.y - pose.y
                dist = math.sqrt(dx**2 + dy**2)
                
                # Check collision with safety margin
                if dist < (ROBOT_RADIUS + obs.radius + SAFETY_MARGIN):
                    return False
                    
        return True
        
    def _score_trajectory(self, traj: Trajectory, current_pose: Pose, current_vel: Twist):
        """
        Score trajectory using multiple critics
        Lower cost is better
        """
        # 1. Path distance cost (distance to global path)
        traj.path_cost = self._compute_path_distance_cost(traj)
        
        # 2. Goal distance cost (distance to goal)
        traj.goal_cost = self._compute_goal_distance_cost(traj)
        
        # 3. Obstacle cost (proximity to obstacles)
        traj.obstacle_cost = self._compute_obstacle_cost(traj)
        
        # 4. Velocity cost (prefer higher velocities)
        traj.velocity_cost = self._compute_velocity_cost(traj)
        
        # 5. Smoothness cost (prefer smooth velocity changes)
        traj.smoothness_cost = self._compute_smoothness_cost(traj, current_vel)
        
        # Total weighted cost
        traj.cost = (
            WEIGHT_PATH_DISTANCE * traj.path_cost +
            WEIGHT_GOAL_DISTANCE * traj.goal_cost +
            WEIGHT_OBSTACLE * traj.obstacle_cost +
            WEIGHT_VELOCITY * traj.velocity_cost +
            WEIGHT_SMOOTHNESS * traj.smoothness_cost
        )
        
    def _compute_path_distance_cost(self, traj: Trajectory) -> float:
        """
        Cost based on distance to global path
        """
        if not self.global_path.poses or not traj.poses:
            return 0.0
            
        # Use end of trajectory
        end_pose = traj.poses[-1]
        
        # Find closest point on global path
        min_dist = float('inf')
        for path_pose in self.global_path.poses:
            dx = path_pose.x - end_pose.x
            dy = path_pose.y - end_pose.y
            dist = math.sqrt(dx**2 + dy**2)
            min_dist = min(min_dist, dist)
            
        return min_dist
        
    def _compute_goal_distance_cost(self, traj: Trajectory) -> float:
        """
        Cost based on distance to goal
        """
        if not traj.poses or not self.current_goal:
            return 0.0
            
        end_pose = traj.poses[-1]
        dx = self.current_goal.x - end_pose.x
        dy = self.current_goal.y - end_pose.y
        
        return math.sqrt(dx**2 + dy**2)
        
    def _compute_obstacle_cost(self, traj: Trajectory) -> float:
        """
        Cost based on proximity to obstacles
        Higher cost for trajectories closer to obstacles
        """
        obstacles = self.scanner.get_obstacles_in_range(
            self.robot.get_pose(), 
            SIM_TIME * MAX_LINEAR_VEL + 1.0
        )
        
        if not obstacles:
            return 0.0
            
        min_dist = float('inf')
        for pose in traj.poses:
            for obs in obstacles:
                dx = obs.x - pose.x
                dy = obs.y - pose.y
                dist = math.sqrt(dx**2 + dy**2) - obs.radius - ROBOT_RADIUS
                min_dist = min(min_dist, dist)
                
        # Exponential cost for close obstacles
        if min_dist < MIN_OBSTACLE_DISTANCE:
            return 10.0  # Very high cost
        else:
            return 1.0 / max(min_dist, 0.1)  # Inverse distance
            
    def _compute_velocity_cost(self, traj: Trajectory) -> float:
        """
        Prefer higher forward velocities (negative cost)
        """
        return -abs(traj.twist.linear) / MAX_LINEAR_VEL
        
    def _compute_smoothness_cost(self, traj: Trajectory, current_vel: Twist) -> float:
        """
        Prefer smooth velocity changes
        """
        dv = abs(traj.twist.linear - current_vel.linear)
        dw = abs(traj.twist.angular - current_vel.angular)
        
        return dv + dw
        
    def _is_goal_reached(self) -> bool:
        """Check if robot has reached the goal"""
        if not self.current_goal:
            return False
            
        pose = self.robot.get_pose()
        
        dx = self.current_goal.x - pose.x
        dy = self.current_goal.y - pose.y
        dist = math.sqrt(dx**2 + dy**2)
        
        angle_diff = abs(self._normalize_angle(self.current_goal.theta - pose.theta))
        
        return (dist < GOAL_TOLERANCE_XY and angle_diff < GOAL_TOLERANCE_YAW)
        
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def _print_debug_info(self, best_traj: Trajectory, num_valid: int):
        """Print debug information"""
        pose = self.robot.get_pose()
        print(f"\n[DWB] Valid trajectories: {num_valid}")
        print(f"  Pose: ({pose.x:.2f}, {pose.y:.2f}, {math.degrees(pose.theta):.1f}°)")
        print(f"  Cmd: v={best_traj.twist.linear:.3f} m/s, ω={best_traj.twist.angular:.3f} rad/s")
        print(f"  Costs: total={best_traj.cost:.2f}, path={best_traj.path_cost:.2f}, "
              f"goal={best_traj.goal_cost:.2f}, obs={best_traj.obstacle_cost:.2f}")

# =====================
# GLOBAL PATH PLANNER
# =====================

class SimpleGlobalPlanner:
    """
    Simple global planner that creates straight-line paths between waypoints
    Replace with A* or other algorithms for complex environments
    """
    
    @staticmethod
    def plan_path(waypoints: List[Tuple[float, float, float]]) -> Path:
        """
        Create a path from waypoints
        waypoints: list of (x, y, theta) tuples
        """
        path = Path()
        
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            
            # Interpolate between waypoints
            num_points = 20
            for j in range(num_points + 1):
                t = j / num_points
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                theta = start[2] + t * (end[2] - start[2])
                
                path.poses.append(Pose(x, y, theta))
                
        return path

# =====================
# MAIN NAVIGATION CONTROLLER
# =====================

class DWBNavigationController:
    """
    High-level navigation controller using DWB local planner
    """
    
    def __init__(self, robot: ESP32RobotController, planner: DWBLocalPlanner):
        self.robot = robot
        self.planner = planner
        
    def navigate_to_waypoints(self, waypoints: List[Tuple[float, float, float]]):
        """
        Navigate through a sequence of waypoints
        waypoints: list of (x, y, theta) tuples
        """
        print(f"\n{'='*70}")
        print(f"STARTING DWB NAVIGATION")
        print(f"{'='*70}")
        print(f"Waypoints:")
        for i, wp in enumerate(waypoints):
            print(f"  {chr(65+i)}: ({wp[0]:.2f}, {wp[1]:.2f}, {math.degrees(wp[2]):.1f}°)")
        print(f"{'='*70}\n")
        
        # Generate global path
        global_path = SimpleGlobalPlanner.plan_path(waypoints)
        self.planner.set_global_path(global_path)
        
        # Execute navigation
        rate = 1.0 / LOOP_RATE
        start_time = time.time()
        
        while not self.planner.goal_reached:
            # Compute velocity command using DWB
            cmd_vel = self.planner.compute_velocity_command()
            
            # Send command to robot
            self.robot.send_velocity(cmd_vel)
            
            # Check timeout (safety)
            if time.time() - start_time > 120:  # 2 minute timeout
                print("⚠ Navigation timeout!")
                break
                
            time.sleep(rate)
            
        # Stop robot
        self.robot.send_velocity(Twist(0, 0))
        
        print(f"\n{'='*70}")
        print(f"NAVIGATION COMPLETE!")
        print(f"Time: {time.time() - start_time:.1f}s")
        print(f"{'='*70}\n")

# =====================
# MAIN PROGRAM
# =====================

def main():
    print("DWB Local Planner for ESP32 Robot")
    print("=" * 70)
    
    # Initialize components
    try:
        robot = ESP32RobotController(SERIAL_PORT, BAUD_RATE)
        robot.start()
        time.sleep(1)
        
        # Create simulated laser scanner
        scanner = SimulatedLaserScanner()
        
        # Add obstacles to environment (CUSTOMIZE THIS!)
        scanner.add_obstacle(0.5, 0.3, 0.15)   # Obstacle 1
        scanner.add_obstacle(0.8, -0.2, 0.1)   # Obstacle 2
        scanner.add_obstacle(1.2, 0.5, 0.2)    # Obstacle 3
        # scanner.add_obstacle(0.3, 0.7, 0.1)  # Uncomment to add more
        
        print(f"✓ Added {len(scanner.obstacles)} obstacles to environment")
        
        # Create DWB planner
        dwb_planner = DWBLocalPlanner(robot, scanner)
        
        # Create navigation controller
        navigator = DWBNavigationController(robot, dwb_planner)
        
        # Define waypoints with orientation (x, y, theta)
        waypoints = [
            (0.0, 0.0, 0.0),                    # Start
            (1.0, 0.0, 0.0),                    # Point B
            (1.5, 0.5, math.pi/4),              # Point C
            (1.5, 1.0, math.pi/2),              # Point D
            (0.5, 1.0, math.pi),                # Point E (return)
        ]
        
        # Alternative: Square pattern
        # waypoints = [
        #     (0, 0, 0),
        #     (1, 0, math.pi/2),
        #     (1, 1, math.pi),
        #     (0, 1, -math.pi/2),
        #     (0, 0, 0)
        # ]
        
        # Run navigation
        navigator.navigate_to_waypoints(waypoints)
        
    except serial.SerialException as e:
        print(f"❌ Failed to connect: {e}")
        return
        
    except KeyboardInterrupt:
        print("\n⚠ Navigation interrupted")
        
    finally:
        robot.stop()
        print("Program terminated")

# =====================
# TESTING & TUNING
# =====================

def test_trajectory_generation():
    """Test trajectory generation without robot"""
    print("\n=== Testing Trajectory Generation ===")
    
    class DummyRobot:
        def get_pose(self):
            return Pose(0, 0, 0)
        def get_current_velocity(self):
            return Twist(0, 0)
    
    scanner = SimulatedLaserScanner()
    scanner.add_obstacle(1.0, 0.0, 0.2)
    
    robot = DummyRobot()
    planner = DWBLocalPlanner(robot, scanner)
    
    # Set goal
    goal_path = Path()
    goal_path.poses.append(Pose(2.0, 0.0, 0.0))
    planner.set_global_path(goal_path)
    
    # Generate trajectories
    cmd = planner.compute_velocity_command()
    print(f"Best command: v={cmd.linear:.3f}, ω={cmd.angular:.3f}")
    
    if planner.best_trajectory:
        print(f"Trajectory has {len(planner.best_trajectory.poses)} poses")
        print(f"Total cost: {planner.best_trajectory.cost:.2f}")

if __name__ == "__main__":
    # Uncomment to run tests
    # test_trajectory_generation()
    
    # Run main navigation
    main()
