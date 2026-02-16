#!/usr/bin/env python3
"""
DWB Planner Configuration Profiles
Different tuning sets for various navigation scenarios
"""

from dataclasses import dataclass
from typing import Dict

@dataclass
class DWBConfig:
    """DWB planner configuration parameters"""
    
    # Robot constraints
    max_linear_vel: float = 0.5
    min_linear_vel: float = -0.2
    max_angular_vel: float = 2.0
    max_linear_acc: float = 0.5
    max_angular_acc: float = 2.0
    
    # Trajectory simulation
    sim_time: float = 2.0
    sim_granularity: float = 0.1
    vel_samples: int = 15
    angular_samples: int = 20
    
    # Cost function weights
    weight_path_distance: float = 32.0
    weight_goal_distance: float = 24.0
    weight_obstacle: float = 10.0
    weight_velocity: float = 1.0
    weight_smoothness: float = 5.0
    
    # Safety parameters
    robot_radius: float = 0.2
    safety_margin: float = 0.05
    min_obstacle_distance: float = 0.3
    
    # Goal tolerances
    goal_tolerance_xy: float = 0.1
    goal_tolerance_yaw: float = 0.15
    
    # Planning rate
    loop_rate: int = 20  # Hz

# =====================
# PREDEFINED PROFILES
# =====================

PROFILES: Dict[str, DWBConfig] = {
    
    # Default balanced profile
    "default": DWBConfig(
        max_linear_vel=0.5,
        max_angular_vel=2.0,
        max_linear_acc=0.5,
        max_angular_acc=2.0,
        sim_time=2.0,
        vel_samples=15,
        angular_samples=20,
        weight_path_distance=32.0,
        weight_goal_distance=24.0,
        weight_obstacle=10.0,
        weight_velocity=1.0,
        weight_smoothness=5.0,
    ),
    
    # Cautious: High obstacle avoidance, slow movement
    "cautious": DWBConfig(
        max_linear_vel=0.3,
        min_linear_vel=-0.1,
        max_angular_vel=1.5,
        max_linear_acc=0.3,
        max_angular_acc=1.5,
        sim_time=3.0,  # Look further ahead
        vel_samples=20,  # More samples for safety
        angular_samples=25,
        weight_path_distance=20.0,
        weight_goal_distance=15.0,
        weight_obstacle=50.0,  # Very high obstacle avoidance
        weight_velocity=0.5,
        weight_smoothness=10.0,  # Smooth movements
        robot_radius=0.25,  # Larger safety bubble
        safety_margin=0.1,
        min_obstacle_distance=0.5,
    ),
    
    # Aggressive: Fast movement, prioritize goal reaching
    "aggressive": DWBConfig(
        max_linear_vel=0.8,
        min_linear_vel=-0.3,
        max_angular_vel=3.0,
        max_linear_acc=0.8,
        max_angular_acc=3.0,
        sim_time=1.5,  # Shorter prediction
        vel_samples=12,
        angular_samples=15,
        weight_path_distance=25.0,
        weight_goal_distance=40.0,  # Strong goal attraction
        weight_obstacle=5.0,  # Lower obstacle concern
        weight_velocity=5.0,  # Prefer high speeds
        weight_smoothness=2.0,
        robot_radius=0.18,
        safety_margin=0.03,
        min_obstacle_distance=0.2,
    ),
    
    # Tight spaces: Good for narrow corridors and dense obstacles
    "tight_spaces": DWBConfig(
        max_linear_vel=0.2,
        min_linear_vel=-0.15,
        max_angular_vel=1.0,
        max_linear_acc=0.2,
        max_angular_acc=1.0,
        sim_time=2.5,
        vel_samples=20,
        angular_samples=30,  # More angular samples for precision
        weight_path_distance=40.0,  # Stay close to path
        weight_goal_distance=20.0,
        weight_obstacle=30.0,
        weight_velocity=0.5,
        weight_smoothness=15.0,  # Very smooth
        robot_radius=0.22,
        safety_margin=0.08,
        min_obstacle_distance=0.4,
        goal_tolerance_xy=0.05,  # Tighter goal tolerance
    ),
    
    # Open space: Fast navigation in obstacle-free areas
    "open_space": DWBConfig(
        max_linear_vel=1.0,
        min_linear_vel=0.0,  # No reverse in open space
        max_angular_vel=2.5,
        max_linear_acc=1.0,
        max_angular_acc=2.5,
        sim_time=1.0,  # Short prediction
        vel_samples=10,
        angular_samples=12,
        weight_path_distance=30.0,
        weight_goal_distance=50.0,  # Strong goal focus
        weight_obstacle=3.0,  # Low obstacle weight
        weight_velocity=10.0,  # Maximize speed
        weight_smoothness=3.0,
        robot_radius=0.18,
        safety_margin=0.02,
        min_obstacle_distance=0.15,
        goal_tolerance_xy=0.15,  # Looser tolerance
    ),
    
    # Smooth: Prioritize smooth trajectories (good for carrying objects)
    "smooth": DWBConfig(
        max_linear_vel=0.4,
        max_angular_vel=1.5,
        max_linear_acc=0.3,
        max_angular_acc=1.0,
        sim_time=2.5,
        vel_samples=18,
        angular_samples=22,
        weight_path_distance=28.0,
        weight_goal_distance=22.0,
        weight_obstacle=12.0,
        weight_velocity=2.0,
        weight_smoothness=25.0,  # Very high smoothness
        robot_radius=0.2,
        safety_margin=0.05,
        min_obstacle_distance=0.35,
    ),
    
    # Parking: High precision for final positioning
    "parking": DWBConfig(
        max_linear_vel=0.15,
        min_linear_vel=-0.15,
        max_angular_vel=0.8,
        max_linear_acc=0.15,
        max_angular_acc=0.8,
        sim_time=3.0,
        vel_samples=25,
        angular_samples=35,
        weight_path_distance=45.0,
        weight_goal_distance=60.0,
        weight_obstacle=25.0,
        weight_velocity=0.2,
        weight_smoothness=20.0,
        robot_radius=0.2,
        safety_margin=0.06,
        min_obstacle_distance=0.35,
        goal_tolerance_xy=0.02,  # Very tight
        goal_tolerance_yaw=0.05,  # ~3 degrees
        loop_rate=30,  # Higher rate for precision
    ),
    
    # Recovery: For when robot gets stuck
    "recovery": DWBConfig(
        max_linear_vel=0.2,
        min_linear_vel=-0.2,  # Allow more reverse
        max_angular_vel=1.5,
        max_linear_acc=0.2,
        max_angular_acc=1.5,
        sim_time=2.0,
        vel_samples=18,
        angular_samples=25,
        weight_path_distance=15.0,  # Less path following
        weight_goal_distance=10.0,  # Less goal focus
        weight_obstacle=40.0,  # High obstacle avoidance
        weight_velocity=0.5,
        weight_smoothness=8.0,
        robot_radius=0.18,  # Smaller for wiggling out
        safety_margin=0.03,
        min_obstacle_distance=0.25,
    ),
}

# =====================
# HELPER FUNCTIONS
# =====================

def get_config(profile_name: str = "default") -> DWBConfig:
    """
    Get configuration by profile name
    
    Args:
        profile_name: Name of profile from PROFILES dict
        
    Returns:
        DWBConfig object
    """
    if profile_name not in PROFILES:
        print(f"⚠ Profile '{profile_name}' not found, using 'default'")
        profile_name = "default"
        
    return PROFILES[profile_name]

def list_profiles():
    """Print all available profiles with descriptions"""
    print("\n" + "="*70)
    print("AVAILABLE DWB CONFIGURATION PROFILES")
    print("="*70)
    
    descriptions = {
        "default": "Balanced performance for general navigation",
        "cautious": "High safety, slow movement, large safety margins",
        "aggressive": "Fast movement, prioritize reaching goal quickly",
        "tight_spaces": "Precise navigation in narrow areas",
        "open_space": "Fast travel in obstacle-free environments",
        "smooth": "Smooth trajectories for carrying objects",
        "parking": "High precision final positioning",
        "recovery": "Escape from stuck situations",
    }
    
    for name, desc in descriptions.items():
        config = PROFILES[name]
        print(f"\n[{name.upper()}]")
        print(f"  {desc}")
        print(f"  Max velocity: {config.max_linear_vel} m/s")
        print(f"  Max angular: {config.max_angular_vel} rad/s")
        print(f"  Obstacle weight: {config.weight_obstacle}")
        print(f"  Safety margin: {config.safety_margin} m")
    
    print("\n" + "="*70 + "\n")

def create_custom_config(**kwargs) -> DWBConfig:
    """
    Create custom config by overriding default parameters
    
    Example:
        config = create_custom_config(
            max_linear_vel=0.6,
            weight_obstacle=15.0
        )
    """
    base_config = PROFILES["default"]
    
    # Create dict from base config
    config_dict = {
        'max_linear_vel': base_config.max_linear_vel,
        'min_linear_vel': base_config.min_linear_vel,
        'max_angular_vel': base_config.max_angular_vel,
        'max_linear_acc': base_config.max_linear_acc,
        'max_angular_acc': base_config.max_angular_acc,
        'sim_time': base_config.sim_time,
        'sim_granularity': base_config.sim_granularity,
        'vel_samples': base_config.vel_samples,
        'angular_samples': base_config.angular_samples,
        'weight_path_distance': base_config.weight_path_distance,
        'weight_goal_distance': base_config.weight_goal_distance,
        'weight_obstacle': base_config.weight_obstacle,
        'weight_velocity': base_config.weight_velocity,
        'weight_smoothness': base_config.weight_smoothness,
        'robot_radius': base_config.robot_radius,
        'safety_margin': base_config.safety_margin,
        'min_obstacle_distance': base_config.min_obstacle_distance,
        'goal_tolerance_xy': base_config.goal_tolerance_xy,
        'goal_tolerance_yaw': base_config.goal_tolerance_yaw,
        'loop_rate': base_config.loop_rate,
    }
    
    # Override with custom parameters
    config_dict.update(kwargs)
    
    return DWBConfig(**config_dict)

def compare_profiles(*profile_names):
    """
    Compare multiple profiles side by side
    
    Example:
        compare_profiles("default", "aggressive", "cautious")
    """
    if not profile_names:
        profile_names = ["default", "aggressive", "cautious"]
    
    print("\n" + "="*100)
    print(f"PROFILE COMPARISON: {', '.join(profile_names)}")
    print("="*100)
    
    # Parameters to compare
    params = [
        ('max_linear_vel', 'Max Linear Vel (m/s)'),
        ('max_angular_vel', 'Max Angular Vel (rad/s)'),
        ('sim_time', 'Simulation Time (s)'),
        ('weight_path_distance', 'Path Weight'),
        ('weight_goal_distance', 'Goal Weight'),
        ('weight_obstacle', 'Obstacle Weight'),
        ('weight_velocity', 'Velocity Weight'),
        ('weight_smoothness', 'Smoothness Weight'),
        ('safety_margin', 'Safety Margin (m)'),
    ]
    
    # Print header
    print(f"\n{'Parameter':<30}", end='')
    for name in profile_names:
        print(f"{name:>15}", end='')
    print()
    print("-" * 100)
    
    # Print each parameter
    for param_name, param_label in params:
        print(f"{param_label:<30}", end='')
        for profile_name in profile_names:
            if profile_name in PROFILES:
                config = PROFILES[profile_name]
                value = getattr(config, param_name)
                print(f"{value:>15.2f}", end='')
            else:
                print(f"{'N/A':>15}", end='')
        print()
    
    print("="*100 + "\n")

# =====================
# USAGE EXAMPLES
# =====================

if __name__ == "__main__":
    # List all available profiles
    list_profiles()
    
    # Compare some profiles
    compare_profiles("default", "aggressive", "cautious", "tight_spaces")
    
    # Get a specific config
    config = get_config("aggressive")
    print(f"\nAggressive config max velocity: {config.max_linear_vel} m/s")
    
    # Create custom config
    custom = create_custom_config(
        max_linear_vel=0.7,
        weight_obstacle=20.0,
        sim_time=2.5
    )
    print(f"\nCustom config max velocity: {custom.max_linear_vel} m/s")
    print(f"Custom config obstacle weight: {custom.weight_obstacle}")
