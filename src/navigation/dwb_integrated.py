#!/usr/bin/env python3
"""
Integrated DWB Navigation with Configuration Profiles
Complete example showing how to use DWB planner with different scenarios
"""

import serial
import time
import math
from typing import List, Tuple
from dwb_planner import (
    ESP32RobotController,
    DWBLocalPlanner,
    SimulatedLaserScanner,
    SimpleGlobalPlanner,
    Pose, Path, Twist
)
from dwb_config import get_config, list_profiles, DWBConfig

# =====================
# ENHANCED DWB PLANNER WITH CONFIG
# =====================

class ConfigurableDWBPlanner(DWBLocalPlanner):
    """DWB Planner that uses external configuration"""
    
    def __init__(self, robot: ESP32RobotController, scanner: SimulatedLaserScanner, 
                 config: DWBConfig):
        super().__init__(robot, scanner)
        self.config = config
        self.apply_config()
        
    def apply_config(self):
        """Apply configuration to planner"""
        # Update module-level constants (not ideal but works with existing code)
        import dwb_planner as dwb
        
        dwb.MAX_LINEAR_VEL = self.config.max_linear_vel
        dwb.MIN_LINEAR_VEL = self.config.min_linear_vel
        dwb.MAX_ANGULAR_VEL = self.config.max_angular_vel
        dwb.MAX_LINEAR_ACC = self.config.max_linear_acc
        dwb.MAX_ANGULAR_ACC = self.config.max_angular_acc
        
        dwb.SIM_TIME = self.config.sim_time
        dwb.SIM_GRANULARITY = self.config.sim_granularity
        dwb.VEL_SAMPLES = self.config.vel_samples
        dwb.ANGULAR_SAMPLES = self.config.angular_samples
        
        dwb.WEIGHT_PATH_DISTANCE = self.config.weight_path_distance
        dwb.WEIGHT_GOAL_DISTANCE = self.config.weight_goal_distance
        dwb.WEIGHT_OBSTACLE = self.config.weight_obstacle
        dwb.WEIGHT_VELOCITY = self.config.weight_velocity
        dwb.WEIGHT_SMOOTHNESS = self.config.weight_smoothness
        
        dwb.ROBOT_RADIUS = self.config.robot_radius
        dwb.SAFETY_MARGIN = self.config.safety_margin
        dwb.MIN_OBSTACLE_DISTANCE = self.config.min_obstacle_distance
        
        dwb.GOAL_TOLERANCE_XY = self.config.goal_tolerance_xy
        dwb.GOAL_TOLERANCE_YAW = self.config.goal_tolerance_yaw
        
        dwb.LOOP_RATE = self.config.loop_rate

# =====================
# SCENARIO DEFINITIONS
# =====================

class NavigationScenario:
    """Defines a navigation scenario with waypoints and obstacles"""
    
    def __init__(self, name: str, waypoints: List[Tuple[float, float, float]], 
                 obstacles: List[Tuple[float, float, float]], 
                 recommended_profile: str):
        self.name = name
        self.waypoints = waypoints  # (x, y, theta)
        self.obstacles = obstacles  # (x, y, radius)
        self.recommended_profile = recommended_profile
        
    def setup_environment(self, scanner: SimulatedLaserScanner):
        """Add obstacles to scanner"""
        scanner.obstacles.clear()
        for x, y, radius in self.obstacles:
            scanner.add_obstacle(x, y, radius)

# Predefined scenarios
SCENARIOS = {
    "simple_corridor": NavigationScenario(
        name="Simple Corridor",
        waypoints=[
            (0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (2.0, 2.0, math.pi/2),
        ],
        obstacles=[
            (1.0, 0.4, 0.15),
            (1.0, -0.4, 0.15),
            (2.0, 0.8, 0.15),
        ],
        recommended_profile="default"
    ),
    
    "tight_corridor": NavigationScenario(
        name="Tight Corridor",
        waypoints=[
            (0.0, 0.0, 0.0),
            (3.0, 0.0, 0.0),
        ],
        obstacles=[
            (0.8, 0.25, 0.1),
            (0.8, -0.25, 0.1),
            (1.5, 0.25, 0.1),
            (1.5, -0.25, 0.1),
            (2.2, 0.25, 0.1),
            (2.2, -0.25, 0.1),
        ],
        recommended_profile="tight_spaces"
    ),
    
    "open_field": NavigationScenario(
        name="Open Field",
        waypoints=[
            (0.0, 0.0, 0.0),
            (3.0, 0.0, 0.0),
            (3.0, 3.0, math.pi/2),
            (0.0, 3.0, math.pi),
            (0.0, 0.0, -math.pi/2),
        ],
        obstacles=[
            (1.5, 1.5, 0.2),  # Single obstacle in middle
        ],
        recommended_profile="open_space"
    ),
    
    "obstacle_field": NavigationScenario(
        name="Dense Obstacles",
        waypoints=[
            (0.0, 0.0, 0.0),
            (2.5, 0.0, 0.0),
            (2.5, 2.5, math.pi/2),
        ],
        obstacles=[
            (0.5, 0.3, 0.15),
            (1.0, -0.3, 0.12),
            (1.3, 0.4, 0.18),
            (1.8, 0.0, 0.15),
            (2.0, 0.5, 0.13),
            (2.2, -0.3, 0.16),
            (2.5, 1.2, 0.14),
        ],
        recommended_profile="cautious"
    ),
    
    "parking_spot": NavigationScenario(
        name="Parking Maneuver",
        waypoints=[
            (0.0, 0.0, 0.0),
            (1.5, 0.0, 0.0),
            (1.5, 0.5, math.pi/2),
            (1.0, 0.5, math.pi),  # Final parking position
        ],
        obstacles=[
            (0.5, 0.5, 0.2),
            (2.0, 0.5, 0.2),
        ],
        recommended_profile="parking"
    ),
}

# =====================
# NAVIGATION CONTROLLER
# =====================

class ScenarioNavigationController:
    """Navigation controller for scenario-based testing"""
    
    def __init__(self, robot: ESP32RobotController, scanner: SimulatedLaserScanner):
        self.robot = robot
        self.scanner = scanner
        self.planner = None
        
    def run_scenario(self, scenario: NavigationScenario, 
                    profile_name: str = None, 
                    visualize: bool = False):
        """
        Run a navigation scenario
        
        Args:
            scenario: NavigationScenario object
            profile_name: Config profile to use (None = use scenario recommendation)
            visualize: Whether to show visualization
        """
        # Use recommended profile if not specified
        if profile_name is None:
            profile_name = scenario.recommended_profile
            
        print(f"\n{'='*70}")
        print(f"SCENARIO: {scenario.name}")
        print(f"Profile: {profile_name}")
        print(f"{'='*70}")
        
        # Setup environment
        scenario.setup_environment(self.scanner)
        print(f"✓ Added {len(scenario.obstacles)} obstacles")
        
        # Create planner with config
        config = get_config(profile_name)
        self.planner = ConfigurableDWBPlanner(self.robot, self.scanner, config)
        
        # Generate global path
        global_path = SimpleGlobalPlanner.plan_path(scenario.waypoints)
        self.planner.set_global_path(global_path)
        
        print(f"✓ Generated path with {len(global_path.poses)} poses")
        print(f"\nWaypoints:")
        for i, wp in enumerate(scenario.waypoints):
            print(f"  {chr(65+i)}: ({wp[0]:.2f}, {wp[1]:.2f}, {math.degrees(wp[2]):.1f}°)")
        
        # Start visualization if requested
        if visualize:
            try:
                from dwb_visualizer import DWBVisualizer
                viz = DWBVisualizer(self.planner, self.robot)
                import threading
                viz_thread = threading.Thread(
                    target=lambda: viz.start(interval=50), 
                    daemon=True
                )
                viz_thread.start()
                print("✓ Visualization started")
            except ImportError:
                print("⚠ Visualization not available (install matplotlib)")
        
        # Execute navigation
        print(f"\n{'='*70}")
        print("STARTING NAVIGATION")
        print(f"{'='*70}\n")
        
        rate = 1.0 / config.loop_rate
        start_time = time.time()
        iteration = 0
        
        try:
            while not self.planner.goal_reached:
                # Compute velocity
                cmd_vel = self.planner.compute_velocity_command()
                
                # Send to robot
                self.robot.send_velocity(cmd_vel)
                
                iteration += 1
                
                # Timeout check
                if time.time() - start_time > 180:  # 3 minute timeout
                    print("\n⚠ Navigation timeout!")
                    break
                    
                time.sleep(rate)
                
        except KeyboardInterrupt:
            print("\n⚠ Navigation interrupted")
            
        finally:
            # Stop robot
            self.robot.send_velocity(Twist(0, 0))
            
            # Print statistics
            elapsed = time.time() - start_time
            final_pose = self.robot.get_pose()
            goal = scenario.waypoints[-1]
            
            dx = goal[0] - final_pose.x
            dy = goal[1] - final_pose.y
            final_error = math.sqrt(dx**2 + dy**2)
            
            print(f"\n{'='*70}")
            print("NAVIGATION COMPLETED")
            print(f"{'='*70}")
            print(f"Time: {elapsed:.1f}s")
            print(f"Iterations: {iteration}")
            print(f"Final position: ({final_pose.x:.3f}, {final_pose.y:.3f}, "
                  f"{math.degrees(final_pose.theta):.1f}°)")
            print(f"Goal: ({goal[0]:.3f}, {goal[1]:.3f}, {math.degrees(goal[2]):.1f}°)")
            print(f"Position error: {final_error:.3f}m")
            print(f"Success: {'YES' if self.planner.goal_reached else 'NO'}")
            print(f"{'='*70}\n")

# =====================
# MAIN PROGRAM
# =====================

def main():
    import sys
    
    print("DWB Navigation with Scenarios")
    print("="*70)
    
    # Parse command line arguments
    scenario_name = "simple_corridor"
    profile_name = None
    visualize = False
    port = '/dev/ttyUSB0'
    
    if len(sys.argv) > 1:
        scenario_name = sys.argv[1]
    if len(sys.argv) > 2:
        profile_name = sys.argv[2]
    if len(sys.argv) > 3 and sys.argv[3] == '--viz':
        visualize = True
    
    # List available scenarios
    print("\nAvailable scenarios:")
    for name, scenario in SCENARIOS.items():
        print(f"  - {name}: {scenario.name} (recommended: {scenario.recommended_profile})")
    
    if scenario_name not in SCENARIOS:
        print(f"\n❌ Scenario '{scenario_name}' not found")
        print("Usage: python dwb_integrated.py [scenario] [profile] [--viz]")
        return
        
    scenario = SCENARIOS[scenario_name]
    
    # Connect to robot
    try:
        print(f"\nConnecting to ESP32 on {port}...")
        robot = ESP32RobotController(port, 115200)
        robot.start()
        time.sleep(1)
        print("✓ Connected to robot")
        
    except serial.SerialException as e:
        print(f"❌ Failed to connect: {e}")
        print("\nRunning in SIMULATION MODE (no hardware)")
        
        # Create dummy robot for testing
        class DummyRobot:
            def __init__(self):
                self.pose = Pose(0, 0, 0)
                self.cmd = Twist(0, 0)
                
            def start(self): pass
            def stop(self): pass
            
            def get_pose(self):
                # Update pose based on commanded velocity
                dt = 0.05
                self.pose.x += self.cmd.linear * math.cos(self.pose.theta) * dt
                self.pose.y += self.cmd.linear * math.sin(self.pose.theta) * dt
                self.pose.theta += self.cmd.angular * dt
                return self.pose
                
            def get_current_velocity(self):
                return self.cmd
                
            def send_velocity(self, twist):
                self.cmd = twist
                
            def get_odometry(self):
                from dwb_planner import Odometry
                return Odometry(pose=self.pose, twist=self.cmd)
        
        robot = DummyRobot()
        robot.start()
    
    try:
        # Create scanner
        scanner = SimulatedLaserScanner()
        
        # Create controller
        controller = ScenarioNavigationController(robot, scanner)
        
        # Run scenario
        controller.run_scenario(scenario, profile_name, visualize)
        
        if visualize:
            print("\nClose visualization window to exit...")
            input("Press Enter to continue...")
        
    finally:
        robot.stop()
        print("Program terminated")

# =====================
# USAGE EXAMPLES
# =====================

def print_usage():
    """Print usage instructions"""
    print("""
╔════════════════════════════════════════════════════════════════════╗
║                    DWB NAVIGATION SYSTEM                           ║
║                     Usage Examples                                 ║
╚════════════════════════════════════════════════════════════════════╝

BASIC USAGE:
    python dwb_integrated.py [scenario] [profile] [--viz]

EXAMPLES:
    # Run simple corridor with default settings
    python dwb_integrated.py simple_corridor
    
    # Run tight corridor with custom profile
    python dwb_integrated.py tight_corridor cautious
    
    # Run with visualization
    python dwb_integrated.py obstacle_field aggressive --viz
    
    # List all profiles
    python dwb_config.py

AVAILABLE SCENARIOS:
    - simple_corridor: Basic navigation with few obstacles
    - tight_corridor: Narrow passage navigation
    - open_field: Large open space with minimal obstacles
    - obstacle_field: Dense obstacle navigation
    - parking_spot: Precision parking maneuver

AVAILABLE PROFILES:
    - default: Balanced performance
    - cautious: Safe, slow movement
    - aggressive: Fast, goal-oriented
    - tight_spaces: Precision in narrow areas
    - open_space: Fast travel in open areas
    - smooth: Smooth trajectories
    - parking: High precision positioning
    - recovery: Escape stuck situations

CUSTOM CONFIGURATION:
    Edit dwb_config.py to create your own profiles!
    
HARDWARE REQUIREMENTS:
    - ESP32 with firmware loaded
    - Connected to {port}
    - Encoders and IMU working
    
SIMULATION MODE:
    If no hardware detected, runs in simulation mode
    for testing and development.
""")

if __name__ == "__main__":
    import sys
    
    if '--help' in sys.argv or '-h' in sys.argv:
        print_usage()
    else:
        main()
