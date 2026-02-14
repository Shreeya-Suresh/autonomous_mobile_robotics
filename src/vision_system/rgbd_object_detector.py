#!/usr/bin/env python3
"""
RGB-D Object Detector

Detects colored objects (e.g., balls) on a flat game field by:
1. Filtering objects closer in depth than the background plane
2. Matching specific color criteria (HSV ranges)
3. Drawing bounding boxes with depth information

Topics:
- Subscribe: /ascamera/color0/image_raw (RGB)
- Subscribe: /ascamera/depth0/image_raw (Depth)
- Publish: /vision/detected_objects (annotated image)

Remote Display (SSH X11 Forwarding):
- Connect with: ssh -X user@raspberry_pi_ip
- Ensure DISPLAY variable is set
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
import json
import os


class RGBDObjectDetector(Node):
    def __init__(self):
        super().__init__('rgbd_object_detector')
        
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('background_margin_mm', 50.0)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('color_preset', 'blue')
        self.declare_parameter('show_windows', True)
        
        self.background_margin = self.get_parameter('background_margin_mm').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.color_preset = self.get_parameter('color_preset').value
        self.show_windows = self.get_parameter('show_windows').value
        
        # Load color presets
        self.color_ranges = self.load_color_presets()
        
        # Subscribers with message synchronization
        rgb_sub = Subscriber(self, Image, '/ascamera/color0/image_raw')
        depth_sub = Subscriber(self, Image, '/ascamera/depth0/image_raw')
        
        # Synchronize RGB and Depth messages
        self.sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.rgbd_callback)
        
        # Publisher for annotated image
        self.image_pub = self.create_publisher(Image, '/vision/detected_objects', 10)
        
        self.get_logger().info(f'RGB-D Object Detector started. Color: {self.color_preset}')
        self.get_logger().info(f'Subscribing to: /ascamera/color0/image_raw, /ascamera/depth0/image_raw')
    
    def load_color_presets(self):
        """Load color presets from JSON file or use defaults"""
        presets_file = os.path.join(
            os.path.dirname(__file__), 
            'color_presets.json'
        )
        
        # Default color ranges (HSV)
        default_presets = {
            "red": {"h_min": 0, "h_max": 10, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
            "red2": {"h_min": 170, "h_max": 180, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
            "green": {"h_min": 35, "h_max": 85, "s_min": 50, "s_max": 255, "v_min": 50, "v_max": 255},
            "blue": {"h_min": 90, "h_max": 130, "s_min": 50, "s_max": 255, "v_min": 50, "v_max": 255},
            "yellow": {"h_min": 20, "h_max": 35, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
            "orange": {"h_min": 10, "h_max": 25, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
            "purple": {"h_min": 130, "h_max": 160, "s_min": 50, "s_max": 255, "v_min": 50, "v_max": 255},
            "white": {"h_min": 0, "h_max": 179, "s_min": 0, "s_max": 40, "v_min": 200, "v_max": 255},
        }
        
        try:
            if os.path.exists(presets_file):
                with open(presets_file, 'r') as f:
                    presets = json.load(f)
                self.get_logger().info(f'Loaded color presets from {presets_file}')
                return presets
        except Exception as e:
            self.get_logger().warn(f'Failed to load presets: {e}. Using defaults.')
        
        return default_presets
    
    def estimate_background_depth(self, depth_image):
        """Estimate background depth by sampling edges of the image"""
        h, w = depth_image.shape
        
        # Sample from edges (assumed to be background)
        edge_samples = []
        
        # Top and bottom edges
        edge_samples.extend(depth_image[0:10, :].flatten())
        edge_samples.extend(depth_image[h-10:h, :].flatten())
        
        # Left and right edges
        edge_samples.extend(depth_image[:, 0:10].flatten())
        edge_samples.extend(depth_image[:, w-10:w].flatten())
        
        # Filter out zero/invalid depths
        valid_samples = [d for d in edge_samples if d > 0]
        
        if len(valid_samples) == 0:
            return None
        
        # Return median depth
        return np.median(valid_samples)
    
    def rgbd_callback(self, rgb_msg, depth_msg):
        """Process synchronized RGB and Depth images"""
        try:
            # Convert ROS images to OpenCV
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            
            # Depth can be 16UC1 or 32FC1
            if depth_msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
                depth_image = depth_image.astype(np.float32)  # Convert to float for processing
            elif depth_msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {depth_msg.encoding}')
                return
            
            # Estimate background depth
            background_depth = self.estimate_background_depth(depth_image)
            
            if background_depth is None:
                self.get_logger().warn('Could not estimate background depth')
                return
            
            # Set depth threshold (objects closer than background)
            depth_threshold = background_depth - self.background_margin
            
            # Convert RGB to HSV for color detection
            hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            
            # Get color range for current preset
            if self.color_preset not in self.color_ranges:
                self.get_logger().warn(f'Color preset "{self.color_preset}" not found')
                return
            
            color_range = self.color_ranges[self.color_preset]
            lower = np.array([color_range['h_min'], color_range['s_min'], color_range['v_min']])
            upper = np.array([color_range['h_max'], color_range['s_max'], color_range['v_max']])
            
            # Create color mask
            mask = cv2.inRange(hsv, lower, upper)
            
            # Morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Annotated image
            annotated = rgb_image.copy()
            detected_count = 0
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                
                # Filter by area
                if area < self.min_area or area > self.max_area:
                    continue
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(cnt)
                
                # Extract depth ROI
                depth_roi = depth_image[y:y+h, x:x+w]
                
                # Calculate median depth of object
                valid_depths = depth_roi[depth_roi > 0]
                if len(valid_depths) == 0:
                    continue
                
                object_depth = np.median(valid_depths)
                
                # Filter by depth (keep objects closer than background)
                if object_depth >= depth_threshold:
                    continue
                
                # Object passed all filters - draw bounding box
                detected_count += 1
                
                # Draw rectangle
                cv2.rectangle(annotated, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # Add label with depth
                label = f'{self.color_preset} {object_depth:.0f}mm'
                cv2.putText(annotated, label, (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Add info overlay
            info_text = f'Objects: {detected_count} | BG: {background_depth:.0f}mm | Threshold: {depth_threshold:.0f}mm'
            cv2.putText(annotated, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.image_pub.publish(annotated_msg)
            
            # Show windows if enabled
            if self.show_windows:
                cv2.imshow('RGB-D Object Detection', annotated)
                cv2.imshow('Color Mask', mask)
                
                # Normalize depth for visualization
                depth_viz = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_viz = depth_viz.astype(np.uint8)
                depth_viz = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)
                cv2.imshow('Depth', depth_viz)
                
                cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in rgbd_callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RGBDObjectDetector()
    
    # Check for X11 display (for remote SSH sessions)
    import os
    if node.show_windows and 'DISPLAY' not in os.environ:
        node.get_logger().warn("DISPLAY not set. For remote viewing via SSH, use: ssh -X user@host")
        node.get_logger().warn("Continuing without display windows...")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_windows:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

        if node.show_windows:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
