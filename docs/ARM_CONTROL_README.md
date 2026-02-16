# Robot Arm & Gripper Control System

Complete control system for ESP32-based robot with differential drive, position-controlled arm, and servo gripper.

## 📋 Table of Contents

- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [System Architecture](#system-architecture)
- [Usage Guide](#usage-guide)
- [API Reference](#api-reference)
- [Mission Planning](#mission-planning)
- [Configuration](#configuration)
- [Examples](#examples)
- [Troubleshooting](#troubleshooting)

---

## 🎯 Overview

This system provides two levels of robot control:

### 1. **Arm Control** (`robot_arm_control.py`)
Flag-based control for arm manipulation and gripper operation
- Simple command interface
- Predefined positions
- Interactive mode
- Pick and place sequences

### 2. **Integrated Control** (`integrated_robot.py`)
Complete autonomous missions combining navigation and manipulation
- Mission planning
- Task sequencing
- Navigation + arm coordination
- Warehouse automation

---

## 🔧 Hardware Requirements

### ESP32 Robot Components

```
┌─────────────────────────────────────┐
│         ESP32 Main Board            │
└─────────────────┬───────────────────┘
                  │
        ┌─────────┴─────────┐
        │                   │
┌───────▼────────┐  ┌──────▼───────┐
│  Drive System  │  │ Manipulation │
│                │  │    System    │
│ • 2 DC Motors  │  │ • Arm Motor  │
│ • 2 Encoders   │  │ • Encoder    │
│ • H-Bridge     │  │ • Gripper    │
│ • MPU6050 IMU  │  │ • Servo      │
└────────────────┘  └──────────────┘
```

### Pin Configuration

**Drive System:**
- Motors: PWM pins 4, 6 | DIR pins 5, 7
- Encoders: Left (16, 17), Right (18, 13)
- IMU: I2C (SDA: 8, SCL: 9)

**Arm & Gripper:**
- Arm motor: PWM pin 11 | DIR pin 12
- Arm encoder: Pins 14, 15
- Gripper servo: Pin 10

### Required Components
- ESP32 development board
- 2× DC motors with encoders (drive wheels)
- 1× DC motor with encoder (arm position control)
- 1× Servo motor (gripper)
- MPU6050 IMU
- Motor drivers (H-bridges)
- Power supply

---

## 📦 Installation

### 1. Upload Firmware to ESP32

```arduino
// Use the provided firmware
// File: firmware_with_arm.ino
```

Upload using Arduino IDE:
1. Install ESP32 board support
2. Install libraries: Wire, I2Cdev, MPU6050
3. Select your ESP32 board
4. Upload the firmware

### 2. Install Python Dependencies

```bash
# Python 3.7+
pip install pyserial

# Optional: For visualization
pip install matplotlib numpy
```

### 3. Download Control Scripts

```bash
# Download these files:
robot_arm_control.py      # Flag-based control
integrated_robot.py       # Mission system
dwb_planner.py           # (Optional) Advanced navigation
```

### 4. Configure Serial Port

```python
# Edit in scripts:
SERIAL_PORT = '/dev/ttyUSB0'  # Linux
SERIAL_PORT = 'COM3'           # Windows
SERIAL_PORT = '/dev/cu.usbserial-XXX'  # Mac
```

---

## 🚀 Quick Start

### Method 1: Flag-Based Control (Simplest)

```bash
# Interactive mode - type commands
python robot_arm_control.py

# At the prompt:
Flag > pick_low
Flag > place_high
Flag > arm_home
Flag > quit
```

### Method 2: Single Flag Execution

```bash
# Execute one action
python robot_arm_control.py pick_low
python robot_arm_control.py place_mid
python robot_arm_control.py gripper_open
```

### Method 3: Run Demos

```bash
# Demo basic movements
python robot_arm_control.py demo_basic

# Demo pick and place
python robot_arm_control.py demo_pick

# Demo flag system
python robot_arm_control.py demo_flags
```

### Method 4: Execute Mission

```bash
# Run predefined mission
python integrated_robot.py simple_transfer
python integrated_robot.py warehouse

# List available missions
python integrated_robot.py list
```

---

## 🏗️ System Architecture

### Control Hierarchy

```
┌─────────────────────────────────────────────┐
│         Mission Layer                        │
│  (Complete tasks: warehouse, sorting)        │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│         Task Layer                           │
│  (Navigate, Pick, Place, Wait)               │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│         Action Layer                         │
│  (Flags: arm_low, gripper_close)             │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│         Command Layer                        │
│  (Serial: V,3.5,4.2  P,1500  G,C)           │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────▼──────────────────────────┐
│         ESP32 Firmware                       │
│  (Motor control, encoders, sensors)          │
└──────────────────────────────────────────────┘
```

### Communication Protocol

**Python → ESP32 (Commands):**
```
V,<left_rad/s>,<right_rad/s>  # Drive velocity
P,<encoder_count>              # Arm position
G,O                            # Gripper open
G,C                            # Gripper close
```

**ESP32 → Python (Telemetry):**
```
D,<left_enc>,<right_enc>,<arm_pos>  # 100 Hz updates
```

---

## 📖 Usage Guide

### A. Flag-Based Control

#### Available Flags

**Arm Position Flags:**
```
arm_home   - Move to home position (0)
arm_low    - Move to low position
arm_mid    - Move to mid position
arm_high   - Move to high position
```

**Gripper Flags:**
```
gripper_open   - Open gripper
gripper_close  - Close gripper
```

**Sequence Flags:**
```
pick_low    - Pick from low position
pick_mid    - Pick from mid position
pick_high   - Pick from high position
place_low   - Place at low position
place_mid   - Place at mid position
place_high  - Place at high position
```

**Utility Flags:**
```
status  - Print current robot state
```

#### Interactive Mode Example

```bash
$ python robot_arm_control.py

Flag > list              # Show all flags
Flag > arm_low           # Move to low position
Flag > gripper_close     # Close gripper
Flag > arm_high          # Move to high position
Flag > gripper_open      # Open gripper
Flag > status            # Show state
Flag > quit              # Exit
```

### B. Programmatic Control

```python
from robot_arm_control import RobotArmController, FlagController

# Initialize robot
robot = RobotArmController('/dev/ttyUSB0', 115200)
robot.start()

# Method 1: Direct control
robot.move_arm_to_named_position('low', wait=True)
robot.close_gripper()
robot.move_arm_to_named_position('high', wait=True)
robot.open_gripper()

# Method 2: Flag control
controller = FlagController(robot)
controller.set_flag('pick_low')
controller.set_flag('place_high')

# Method 3: Sequences
robot.pick_object('low')    # Complete pick sequence
robot.place_object('high')  # Complete place sequence

# Cleanup
robot.stop()
```

### C. Mission System

#### Mission Structure

```python
from integrated_robot import Mission, IntegratedRobotController

# Create mission
mission = Mission("My Task")

# Add tasks (they chain together)
mission.add_navigate(1.0, 0.0, 0.0, "go_to_pickup")
mission.add_pick('low', "grab_object")
mission.add_navigate(2.0, 1.0, 1.57, "go_to_dropoff")
mission.add_place('high', "release_object")
mission.add_wait(2.0, "pause")
mission.add_navigate(0.0, 0.0, 0.0, "return_home")

# Execute
robot = IntegratedRobotController('/dev/ttyUSB0', 115200)
robot.start()
robot.execute_mission(mission)
robot.stop()
```

#### Predefined Missions

**1. Simple Transfer**
```
1. Navigate to (1.0, 0.0)
2. Pick from low position
3. Navigate to (1.0, 1.0)
4. Place at low position
5. Return home
```

**2. Sort Objects**
```
1. Navigate to pickup station
2. Pick from floor
3. Navigate to shelf
4. Place on low shelf
(Repeat for mid and high shelves)
5. Return home
```

**3. Warehouse Task**
```
1. Navigate to station A
2. Pick package from mid shelf
3. Navigate to station B
4. Place package on low shelf
5. Navigate to station C
6. Pick return item
7. Navigate home
8. Place item on mid shelf
9. Stow arm
```

---

## 🔍 API Reference

### RobotArmController Class

```python
class RobotArmController:
    def __init__(self, port: str, baud: int)
    def start()
    def stop()
    
    # Arm control
    def move_arm_to_position(self, position: int)
    def move_arm_to_named_position(self, name: str, wait: bool)
    def wait_for_arm_position(self, timeout: float) -> bool
    def get_arm_position(self) -> int
    def is_arm_at_target(self) -> bool
    def home_arm(self, wait: bool)
    
    # Gripper control
    def open_gripper()
    def close_gripper()
    def set_gripper(self, state: GripperState)
    
    # Sequences
    def pick_object(self, arm_position: str, wait: bool)
    def place_object(self, arm_position: str, wait: bool)
    
    # Status
    def print_status()
```

### FlagController Class

```python
class FlagController:
    def __init__(self, robot: RobotArmController)
    def register_flag(self, flag_name: str, handler: Callable)
    def set_flag(self, flag_name: str) -> bool
    def list_flags()
```

### IntegratedRobotController Class

```python
class IntegratedRobotController:
    def __init__(self, port: str, baud: int)
    def start()
    def stop()
    
    # Navigation
    def navigate_to_pose(self, target: Pose, timeout: float) -> bool
    def send_drive_velocity(self, linear: float, angular: float)
    
    # Manipulation
    def move_arm_to_position(self, name: str, wait: bool) -> bool
    def open_gripper()
    def close_gripper()
    def pick_object(self, position: str) -> bool
    def place_object(self, position: str) -> bool
    
    # Mission execution
    def execute_mission(self, mission: Mission) -> bool
```

### Mission Class

```python
class Mission:
    def __init__(self, name: str)
    
    # Task builders (chainable)
    def add_navigate(self, x: float, y: float, theta: float, name: str)
    def add_pick(self, arm_position: str, name: str)
    def add_place(self, arm_position: str, name: str)
    def add_arm_move(self, position: str, name: str)
    def add_gripper(self, state: GripperState, name: str)
    def add_wait(self, duration: float, name: str)
```

---

## 🎯 Mission Planning

### Creating Custom Missions

```python
# Example: Assembly line task
assembly_mission = Mission("Assembly Line")

# Station 1: Get component A
assembly_mission.add_navigate(1.0, 0.0, 0.0, "station_1")
assembly_mission.add_pick('low', "get_component_a")

# Station 2: Get component B
assembly_mission.add_navigate(2.0, 0.0, 0.0, "station_2")
assembly_mission.add_arm_move('mid', "prepare_for_b")
assembly_mission.add_gripper(GripperState.OPEN, "open_for_dual")
# ... (Complex dual-gripper logic here)

# Assembly station
assembly_mission.add_navigate(1.5, 1.5, 1.57, "assembly")
assembly_mission.add_place('mid', "place_assembly")

# Quality check (wait for inspection)
assembly_mission.add_wait(3.0, "inspection")

# Return
assembly_mission.add_navigate(0.0, 0.0, 0.0, "home")
```

### Task Types

| Task Type | Description | Parameters |
|-----------|-------------|------------|
| `NAVIGATE` | Drive to position | x, y, theta |
| `PICK` | Execute pick sequence | arm_position |
| `PLACE` | Execute place sequence | arm_position |
| `ARM_MOVE` | Move arm only | position |
| `GRIPPER` | Control gripper | state (OPEN/CLOSED) |
| `WAIT` | Pause execution | duration (seconds) |

---

## ⚙️ Configuration

### Arm Position Configuration

Edit `ARM_POSITIONS` to match your robot:

```python
ARM_POSITIONS = {
    'home': 0,        # Stowed position
    'floor': 300,     # Pick from floor
    'low': 800,       # Low shelf (measure this!)
    'mid': 1800,      # Mid shelf (measure this!)
    'high': 3200,     # High shelf (measure this!)
    'max': 5000,      # Maximum extension
}
```

**How to measure:**
1. Manually move arm to desired position
2. Read encoder count from telemetry
3. Update ARM_POSITIONS with that value

### Tolerance Configuration

```python
ARM_POSITION_TOLERANCE = 15  # Encoder counts
# Smaller = more precise, but may timeout
# Larger = faster, but less accurate

WAYPOINT_TOLERANCE = 0.1     # meters
ANGLE_TOLERANCE = 0.15       # radians (~8 degrees)
```

### Serial Port Configuration

```python
# Linux
SERIAL_PORT = '/dev/ttyUSB0'  # or /dev/ttyACM0

# Windows - Check Device Manager
SERIAL_PORT = 'COM3'

# Mac
SERIAL_PORT = '/dev/cu.usbserial-XXXX'

BAUD_RATE = 115200  # Match firmware
```

---

## 💡 Examples

### Example 1: Simple Pick and Place

```python
#!/usr/bin/env python3
from robot_arm_control import RobotArmController

robot = RobotArmController('/dev/ttyUSB0', 115200)
robot.start()

try:
    # Pick from low shelf
    robot.open_gripper()
    robot.move_arm_to_named_position('low', wait=True)
    robot.close_gripper()
    
    # Move to high shelf
    robot.move_arm_to_named_position('high', wait=True)
    robot.open_gripper()
    
    # Return home
    robot.home_arm()
    
finally:
    robot.stop()
```

### Example 2: Multi-Position Transfer

```python
#!/usr/bin/env python3
from robot_arm_control import RobotArmController
import time

robot = RobotArmController('/dev/ttyUSB0', 115200)
robot.start()

positions = ['low', 'mid', 'high']

try:
    for pos in positions:
        print(f"Processing: {pos}")
        robot.pick_object(pos)
        time.sleep(1)
        
        # Move to next position (cycle)
        next_idx = (positions.index(pos) + 1) % len(positions)
        next_pos = positions[next_idx]
        robot.place_object(next_pos)
        time.sleep(1)
        
finally:
    robot.stop()
```

### Example 3: Conditional Control

```python
#!/usr/bin/env python3
from robot_arm_control import RobotArmController

robot = RobotArmController('/dev/ttyUSB0', 115200)
robot.start()

try:
    # Check current position
    current = robot.get_arm_position()
    
    if current < 500:
        print("Arm is low, moving to mid")
        robot.move_arm_to_named_position('mid')
    elif current < 2000:
        print("Arm is mid, moving to high")
        robot.move_arm_to_named_position('high')
    else:
        print("Arm is high, homing")
        robot.home_arm()
        
finally:
    robot.stop()
```

### Example 4: Warehouse Sorting

```python
#!/usr/bin/env python3
from integrated_robot import IntegratedRobotController, Mission
import math

robot = IntegratedRobotController('/dev/ttyUSB0', 115200)
robot.start()

# Sort 3 objects to different shelves
sorting = Mission("Warehouse Sorting")

for i, shelf in enumerate(['low', 'mid', 'high']):
    sorting.add_navigate(0.5, 0.0, 0.0, f"pickup_{i}")
    sorting.add_pick('floor', f"grab_item_{i}")
    sorting.add_navigate(2.0, 0.0, 0.0, f"dropoff_{i}")
    sorting.add_place(shelf, f"place_on_{shelf}")

sorting.add_navigate(0.0, 0.0, 0.0, "return_home")

try:
    success = robot.execute_mission(sorting)
    if success:
        print("✓ Sorting complete!")
finally:
    robot.stop()
```

### Example 5: Custom Flag Handler

```python
#!/usr/bin/env python3
from robot_arm_control import RobotArmController, FlagController

robot = RobotArmController('/dev/ttyUSB0', 115200)
robot.start()

controller = FlagController(robot)

# Register custom flag
def special_sequence():
    print("Executing special sequence")
    robot.move_arm_to_named_position('low')
    robot.close_gripper()
    robot.move_arm_to_named_position('high')
    robot.open_gripper()
    robot.move_arm_to_named_position('mid')

controller.register_flag('special', special_sequence)

try:
    # Use custom flag
    controller.set_flag('special')
finally:
    robot.stop()
```

### Example 6: Error Handling

```python
#!/usr/bin/env python3
from robot_arm_control import RobotArmController
import time

robot = RobotArmController('/dev/ttyUSB0', 115200)
robot.start()

try:
    # Try to reach position with timeout
    success = robot.move_arm_to_named_position('low', wait=True)
    
    if not success:
        print("❌ Failed to reach position")
        # Recovery action
        robot.home_arm()
        time.sleep(1)
        # Retry
        success = robot.move_arm_to_named_position('low', wait=True)
        
    if success:
        print("✓ Position reached, continuing...")
        robot.close_gripper()
        
except Exception as e:
    print(f"Error: {e}")
    robot.home_arm()  # Emergency home
    
finally:
    robot.stop()
```

---

## 🔧 Troubleshooting

### Problem: Arm not moving

**Possible causes:**
1. Wrong encoder counts in `ARM_POSITIONS`
2. Motor not connected
3. Encoder not working
4. PID gain too low

**Solutions:**
```python
# Test 1: Check telemetry
robot.print_status()  # See current arm_position

# Test 2: Move to small position
robot.move_arm_to_position(100)  # Small movement
time.sleep(2)
robot.print_status()  # Did it move?

# Test 3: Check firmware
# Upload fresh firmware
# Check serial monitor for "SYSTEM READY"
```

### Problem: Arm overshoots/oscillates

**Solution:** Tune PID gain in firmware

```arduino
// In firmware, adjust:
#define KP 0.5f  // Decrease if oscillating
#define KP 1.0f  // Increase if too slow
```

### Problem: Position timeout

**Possible causes:**
1. Target position unreachable
2. Tolerance too tight
3. Motor stalled

**Solutions:**
```python
# Increase tolerance
ARM_POSITION_TOLERANCE = 30  # Was 15

# Increase timeout
robot.move_arm_to_named_position('low', wait=True, timeout=15.0)

# Check if position is within range
target = ARM_POSITIONS['low']
if 0 <= target <= ARM_MAX_POSITION:
    robot.move_arm_to_position(target)
```

### Problem: Gripper not responding

**Solutions:**
```python
# Test gripper directly
robot.open_gripper()
time.sleep(2)
robot.close_gripper()
time.sleep(2)

# Check servo angles in firmware
#define GRIP_OPEN_ANGLE   40   // Adjust these
#define GRIP_CLOSE_ANGLE  180  // Adjust these
```

### Problem: Serial connection failed

**Solutions:**
```bash
# Linux: Check port permissions
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0

# List available ports
python -m serial.tools.list_ports

# Test connection
python -c "import serial; s=serial.Serial('/dev/ttyUSB0', 115200); print('OK')"
```

### Problem: Mission fails partway through

**Debug approach:**
```python
# Add debug prints
import logging
logging.basicConfig(level=logging.DEBUG)

# Test tasks individually
robot.navigate_to_pose(Pose(1.0, 0.0, 0.0))  # Test nav
robot.pick_object('low')  # Test pick
robot.place_object('high')  # Test place

# Run mission with recovery
try:
    robot.execute_mission(mission)
except Exception as e:
    print(f"Failed at: {e}")
    robot.home_arm()  # Safe position
    robot.send_drive_velocity(0, 0)  # Stop
```

### Problem: Arm position drift

**Cause:** Encoder not tracking properly

**Solutions:**
1. Check encoder wiring
2. Verify interrupt service routine (ISR)
3. Add encoder pullup resistors
4. Check for electrical noise

```python
# Monitor encoder over time
for i in range(100):
    pos = robot.get_arm_position()
    print(f"Position: {pos}")
    time.sleep(0.1)
# Position should be stable when not moving
```

---

## 📊 Performance Metrics

**Typical Performance:**
- Command latency: ~5-10ms
- Position accuracy: ±10 encoder counts
- Gripper response: ~0.5s
- Pick sequence time: ~3-4s
- Place sequence time: ~3-4s
- Navigation speed: 0.3 m/s (adjustable)

**Telemetry Rate:**
- Encoder updates: 100 Hz
- Arm position updates: 200 Hz (internal control)
- Serial telemetry: 100 Hz

---

## 🎓 Learning Resources

### Understanding the System

1. **Start with flags** - Simplest way to control
2. **Try direct control** - More flexibility
3. **Build sequences** - Combine actions
4. **Create missions** - Full automation

### Code Examples Progression

```
1. robot_arm_control.py demo_basic     # Learn basics
2. robot_arm_control.py demo_pick      # Pick/place
3. robot_arm_control.py interactive    # Manual control
4. integrated_robot.py simple_transfer # First mission
5. integrated_robot.py warehouse       # Complex mission
6. Create your own custom missions!
```

---

## 📝 Command Reference

### Command Line Usage

```bash
# ARM CONTROL
python robot_arm_control.py                    # Interactive
python robot_arm_control.py list               # List flags
python robot_arm_control.py <flag>             # Execute flag
python robot_arm_control.py demo_basic         # Basic demo
python robot_arm_control.py demo_pick          # Pick demo
python robot_arm_control.py demo_flags         # Flag demo

# INTEGRATED CONTROL
python integrated_robot.py list                # List missions
python integrated_robot.py <mission_name>      # Run mission
python integrated_robot.py simple_transfer     # Basic mission
python integrated_robot.py warehouse           # Complex mission

# WITH DWB NAVIGATION (if installed)
python dwb_integrated.py simple_corridor       # DWB scenario
python dwb_integrated.py --viz                 # With visualization
```

---

## 🚀 Next Steps

1. **Calibrate your robot**
   - Measure encoder counts for positions
   - Tune gripper open/close angles
   - Test pick/place sequences

2. **Create custom positions**
   - Add to `ARM_POSITIONS` dictionary
   - Register new flags if needed

3. **Build custom missions**
   - Start with simple transfer
   - Add complexity gradually
   - Test each task individually

4. **Integrate sensors**
   - Add obstacle detection
   - Implement force sensing
   - Add vision system

5. **Advanced features**
   - Combine with DWB navigation
   - Add multi-robot coordination
   - Implement trajectory planning

---

## 📄 License

MIT License - Free to use and modify

---

## 🙏 Support

For issues:
1. Check troubleshooting section
2. Verify hardware connections
3. Test with simple examples first
4. Check serial communication

---

## 📞 Quick Reference Card

```
╔═══════════════════════════════════════════════════════════╗
║                  QUICK REFERENCE                          ║
╠═══════════════════════════════════════════════════════════╣
║ CONNECT:    python robot_arm_control.py                  ║
║ FLAGS:      arm_low, arm_home, gripper_open              ║
║ SEQUENCES:  pick_low, place_high                         ║
║ MISSIONS:   python integrated_robot.py warehouse         ║
║ STATUS:     robot.print_status()                         ║
║ HELP:       python robot_arm_control.py --help           ║
╚═══════════════════════════════════════════════════════════╝

ARM POSITIONS:        GRIPPER STATES:
• home  (0)           • OPEN   (40°)
• floor (300)         • CLOSED (180°)
• low   (800)
• mid   (1800)        SERIAL PROTOCOL:
• high  (3200)        • V,w_left,w_right
• max   (5000)        • P,encoder_count
                      • G,O  or  G,C
```

---

**Happy Robot Building! 🤖**
