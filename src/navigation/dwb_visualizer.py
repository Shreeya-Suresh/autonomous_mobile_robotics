#!/usr/bin/env python3
"""
Real-time Visualization for DWB Planner
Shows robot position, trajectories, obstacles, and paths
Requires: matplotlib
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
import math
from typing import List, Optional
from dwb_planner import (
    Pose, Trajectory, Obstacle, Path, 
    ESP32RobotController, DWBLocalPlanner,
    ROBOT_RADIUS
)

class DWBVisualizer:
    """Real-time visualization of DWB planning"""
    
    def __init__(self, planner: DWBLocalPlanner, robot: ESP32RobotController):
        self.planner = planner
        self.robot = robot
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('DWB Local Planner Visualization')
        
        # Plot elements (initialized in animation)
        self.robot_circle = None
        self.robot_arrow = None
        self.trajectory_lines = []
        self.best_traj_line = None
        self.path_line = None
        self.goal_marker = None
        self.obstacle_circles = []
        
        # History for path tracking
        self.pose_history = []
        self.history_line = None
        
    def setup_plot(self):
        """Initialize plot elements"""
        # Robot representation
        self.robot_circle = patches.Circle((0, 0), ROBOT_RADIUS, 
                                          color='blue', alpha=0.5, zorder=10)
        self.ax.add_patch(self.robot_circle)
        
        # Robot heading arrow
        self.robot_arrow = patches.FancyArrow(0, 0, 0.2, 0, 
                                              width=0.05, color='blue', zorder=11)
        self.ax.add_patch(self.robot_arrow)
        
        # Global path
        self.path_line, = self.ax.plot([], [], 'g--', linewidth=2, 
                                       alpha=0.6, label='Global Path')
        
        # Best trajectory
        self.best_traj_line, = self.ax.plot([], [], 'r-', linewidth=3, 
                                            alpha=0.8, label='Best Trajectory')
        
        # Robot path history
        self.history_line, = self.ax.plot([], [], 'b-', linewidth=1, 
                                          alpha=0.4, label='Traveled Path')
        
        # Goal marker
        self.goal_marker, = self.ax.plot([], [], 'g*', markersize=20, 
                                        label='Goal')
        
        # Obstacles
        for obs in self.planner.scanner.obstacles:
            circle = patches.Circle((obs.x, obs.y), obs.radius, 
                                   color='red', alpha=0.7, zorder=5)
            self.ax.add_patch(circle)
            self.obstacle_circles.append(circle)
            
        self.ax.legend(loc='upper right')
        
    def update(self, frame):
        """Animation update function"""
        # Get current robot pose
        pose = self.robot.get_pose()
        
        # Update robot position
        self.robot_circle.center = (pose.x, pose.y)
        
        # Update robot heading arrow
        arrow_length = 0.2
        dx = arrow_length * math.cos(pose.theta)
        dy = arrow_length * math.sin(pose.theta)
        
        # Remove old arrow and create new one (matplotlib limitation)
        self.robot_arrow.remove()
        self.robot_arrow = patches.FancyArrow(
            pose.x, pose.y, dx, dy,
            width=0.05, color='blue', zorder=11
        )
        self.ax.add_patch(self.robot_arrow)
        
        # Update pose history
        self.pose_history.append((pose.x, pose.y))
        if len(self.pose_history) > 500:  # Keep last 500 points
            self.pose_history.pop(0)
            
        if self.pose_history:
            history_x, history_y = zip(*self.pose_history)
            self.history_line.set_data(history_x, history_y)
        
        # Update global path
        if self.planner.global_path.poses:
            path_x = [p.x for p in self.planner.global_path.poses]
            path_y = [p.y for p in self.planner.global_path.poses]
            self.path_line.set_data(path_x, path_y)
        
        # Update best trajectory
        if self.planner.best_trajectory and self.planner.best_trajectory.poses:
            traj_x = [p.x for p in self.planner.best_trajectory.poses]
            traj_y = [p.y for p in self.planner.best_trajectory.poses]
            self.best_traj_line.set_data(traj_x, traj_y)
            
            # Add arrow at end of trajectory
            if len(traj_x) > 1:
                end_pose = self.planner.best_trajectory.poses[-1]
                # Small arrow showing trajectory direction
                
        # Update goal marker
        if self.planner.current_goal:
            self.goal_marker.set_data([self.planner.current_goal.x], 
                                      [self.planner.current_goal.y])
        
        # Auto-scale view to follow robot
        margin = 2.0  # meters
        self.ax.set_xlim(pose.x - margin, pose.x + margin)
        self.ax.set_ylim(pose.y - margin, pose.y + margin)
        
        # Update title with current state
        cmd = self.planner.best_trajectory.twist if self.planner.best_trajectory else None
        if cmd:
            self.ax.set_title(
                f'DWB Planner | Pose: ({pose.x:.2f}, {pose.y:.2f}, {math.degrees(pose.theta):.1f}°) | '
                f'Cmd: v={cmd.linear:.2f} m/s, ω={cmd.angular:.2f} rad/s'
            )
        
        return [self.robot_circle, self.robot_arrow, self.best_traj_line, 
                self.path_line, self.history_line, self.goal_marker]
    
    def start(self, interval=100):
        """Start animation (interval in ms)"""
        self.setup_plot()
        
        # Set initial view
        if self.planner.scanner.obstacles:
            all_x = [obs.x for obs in self.planner.scanner.obstacles]
            all_y = [obs.y for obs in self.planner.scanner.obstacles]
            
            if self.planner.global_path.poses:
                all_x.extend([p.x for p in self.planner.global_path.poses])
                all_y.extend([p.y for p in self.planner.global_path.poses])
            
            margin = 1.0
            self.ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            self.ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        else:
            self.ax.set_xlim(-1, 3)
            self.ax.set_ylim(-1, 3)
        
        self.anim = FuncAnimation(self.fig, self.update, interval=interval, 
                                 blit=False, cache_frame_data=False)
        plt.show()

# =====================
# STANDALONE VISUALIZATION TEST
# =====================

def visualize_with_dummy_data():
    """Test visualization with dummy data"""
    print("Starting visualization test...")
    
    class DummyRobot:
        def __init__(self):
            self.pose = Pose(0, 0, 0)
            self.t = 0
            
        def get_pose(self):
            # Simulate robot moving in circle
            self.t += 0.05
            self.pose.x = math.cos(self.t)
            self.pose.y = math.sin(self.t)
            self.pose.theta = self.t + math.pi/2
            return self.pose
            
        def get_current_velocity(self):
            from dwb_planner import Twist
            return Twist(0.2, 0.5)
    
    from dwb_planner import SimulatedLaserScanner
    
    # Setup dummy environment
    robot = DummyRobot()
    scanner = SimulatedLaserScanner()
    scanner.add_obstacle(1.5, 0.5, 0.2)
    scanner.add_obstacle(0.5, 1.5, 0.15)
    
    planner = DWBLocalPlanner(robot, scanner)
    
    # Create path
    path = Path()
    for t in np.linspace(0, 2*math.pi, 50):
        path.poses.append(Pose(math.cos(t), math.sin(t), t))
    planner.set_global_path(path)
    
    # Start visualization
    viz = DWBVisualizer(planner, robot)
    viz.start(interval=50)

if __name__ == "__main__":
    # Run standalone visualization test
    visualize_with_dummy_data()
