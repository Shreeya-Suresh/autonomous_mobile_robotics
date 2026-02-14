#!/usr/bin/env python3
"""
Color Calibration Tool

Interactive tool to find optimal HSV ranges for different colored objects.
Allows real-time adjustment of HSV thresholds and saving/loading presets.

Usage:
    python3 color_calibration_tool.py

Remote Display (SSH X11 Forwarding):
    ssh -X user@raspberry_pi_ip
    python3 color_calibration_tool.py

Controls:
    - Adjust HSV sliders to isolate target color
    - Press 's' to save current values to preset
    - Press 'l' to load preset
    - Press '1-8' to switch between color presets
    - Press 'q' to quit
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os


class ColorCalibrationTool(Node):
    def __init__(self):
        super().__init__('color_calibration_tool')
        
        self.bridge = CvBridge()
        
        # Current color preset
        self.current_preset = 'blue'
        self.preset_names = ['red', 'green', 'blue', 'yellow', 'orange', 'purple', 'white', 'black']
        
        # HSV ranges
        self.h_min = 90
        self.h_max = 130
        self.s_min = 50
        self.s_max = 255
        self.v_min = 50
        self.v_max = 255
        
        # Subscribe to RGB camera
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/color0/image_raw',
            self.image_callback,
            10
        )
        
        # Create windows and trackbars
        cv2.namedWindow('Original')
        cv2.namedWindow('Mask')
        cv2.namedWindow('HSV Trackbars')
        
        # Create trackbars
        cv2.createTrackbar('H Min', 'HSV Trackbars', self.h_min, 179, self.on_trackbar)
        cv2.createTrackbar('H Max', 'HSV Trackbars', self.h_max, 179, self.on_trackbar)
        cv2.createTrackbar('S Min', 'HSV Trackbars', self.s_min, 255, self.on_trackbar)
        cv2.createTrackbar('S Max', 'HSV Trackbars', self.s_max, 255, self.on_trackbar)
        cv2.createTrackbar('V Min', 'HSV Trackbars', self.v_min, 255, self.on_trackbar)
        cv2.createTrackbar('V Max', 'HSV Trackbars', self.v_max, 255, self.on_trackbar)
        
        self.get_logger().info('Color Calibration Tool started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  s - Save current HSV values')
        self.get_logger().info('  l - Load saved values')
        self.get_logger().info('  1-8 - Switch color preset')
        self.get_logger().info('  q - Quit')
        
        # Load existing presets
        self.load_presets()
    
    def on_trackbar(self, val):
        """Trackbar callback"""
        self.h_min = cv2.getTrackbarPos('H Min', 'HSV Trackbars')
        self.h_max = cv2.getTrackbarPos('H Max', 'HSV Trackbars')
        self.s_min = cv2.getTrackbarPos('S Min', 'HSV Trackbars')
        self.s_max = cv2.getTrackbarPos('S Max', 'HSV Trackbars')
        self.v_min = cv2.getTrackbarPos('V Min', 'HSV Trackbars')
        self.v_max = cv2.getTrackbarPos('V Max', 'HSV Trackbars')
    
    def load_presets(self):
        """Load color presets from JSON file"""
        presets_file = os.path.join(
            os.path.dirname(__file__),
            'color_presets.json'
        )
        
        try:
            if os.path.exists(presets_file):
                with open(presets_file, 'r') as f:
                    self.presets = json.load(f)
                self.get_logger().info(f'Loaded presets from {presets_file}')
            else:
                self.presets = {}
                self.get_logger().info('No presets file found, starting fresh')
        except Exception as e:
            self.get_logger().error(f'Error loading presets: {e}')
            self.presets = {}
    
    def save_presets(self):
        """Save current HSV values to preset"""
        presets_file = os.path.join(
            os.path.dirname(__file__),
            'color_presets.json'
        )
        
        # Update current preset
        self.presets[self.current_preset] = {
            'h_min': self.h_min,
            'h_max': self.h_max,
            's_min': self.s_min,
            's_max': self.s_max,
            'v_min': self.v_min,
            'v_max': self.v_max
        }
        
        try:
            with open(presets_file, 'w') as f:
                json.dump(self.presets, f, indent=2)
            self.get_logger().info(f'Saved "{self.current_preset}" preset to {presets_file}')
            self.get_logger().info(f'HSV: [{self.h_min}, {self.h_max}], [{self.s_min}, {self.s_max}], [{self.v_min}, {self.v_max}]')
        except Exception as e:
            self.get_logger().error(f'Error saving presets: {e}')
    
    def load_preset(self, preset_name):
        """Load HSV values from preset"""
        if preset_name in self.presets:
            preset = self.presets[preset_name]
            self.h_min = preset['h_min']
            self.h_max = preset['h_max']
            self.s_min = preset['s_min']
            self.s_max = preset['s_max']
            self.v_min = preset['v_min']
            self.v_max = preset['v_max']
            
            # Update trackbars
            cv2.setTrackbarPos('H Min', 'HSV Trackbars', self.h_min)
            cv2.setTrackbarPos('H Max', 'HSV Trackbars', self.h_max)
            cv2.setTrackbarPos('S Min', 'HSV Trackbars', self.s_min)
            cv2.setTrackbarPos('S Max', 'HSV Trackbars', self.s_max)
            cv2.setTrackbarPos('V Min', 'HSV Trackbars', self.v_min)
            cv2.setTrackbarPos('V Max', 'HSV Trackbars', self.v_max)
            
            self.get_logger().info(f'Loaded "{preset_name}" preset')
        else:
            self.get_logger().warn(f'Preset "{preset_name}" not found')
    
    def image_callback(self, msg):
        """Process incoming RGB image"""
        try:
            # Convert to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Create mask with current HSV range
            lower = np.array([self.h_min, self.s_min, self.v_min])
            upper = np.array([self.h_max, self.s_max, self.v_max])
            mask = cv2.inRange(hsv, lower, upper)
            
            # Apply morphological operations
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours for visualization
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw contours on original
            result = frame.copy()
            cv2.drawContours(result, contours, -1, (0, 255, 0), 2)
            
            # Add info text
            info_text = f'Preset: {self.current_preset} | HSV: [{self.h_min},{self.h_max}] [{self.s_min},{self.s_max}] [{self.v_min},{self.v_max}]'
            cv2.putText(result, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show images
            cv2.imshow('Original', result)
            cv2.imshow('Mask', mask)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('s'):
                self.save_presets()
            elif key == ord('l'):
                self.load_preset(self.current_preset)
            elif key == ord('q'):
                self.get_logger().info('Quitting...')
                rclpy.shutdown()
            elif ord('1') <= key <= ord('8'):
                # Switch preset
                idx = key - ord('1')
                if idx < len(self.preset_names):
                    self.current_preset = self.preset_names[idx]
                    self.get_logger().info(f'Switched to "{self.current_preset}" preset')
                    self.load_preset(self.current_preset)
            
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')


def main(args=None):
    # Check for X11 display (for remote SSH sessions)
    import os
    if 'DISPLAY' not in os.environ:
        print("ERROR: DISPLAY not set. For remote viewing via SSH, use: ssh -X user@host")
        print("This tool requires a display for interactive trackbars.")
        return
    
    rclpy.init(args=args)
    node = ColorCalibrationTool()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
