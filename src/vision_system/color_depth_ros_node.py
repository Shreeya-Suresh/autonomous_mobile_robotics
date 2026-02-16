#!/usr/bin/env python3
"""
ROS2 Integration for Color + Depth Visualization
Subscribes to Ascamera topics and runs real-time analysis

Usage:
    ros2 run vision_system color_depth_ros_node.py
    
    # Or with parameters:
    ros2 run vision_system color_depth_ros_node.py --ros-args \
        -p colors:="['red','blue','green']" \
        -p min_area:=800
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional


class ColorDepthROSNode(Node):
    """ROS2 node for real-time color and depth visualization"""
    
    COLOR_RANGES = {
        'red': [((0, 100, 100), (10, 255, 255)), ((170, 100, 100), (180, 255, 255))],
        'blue': [((100, 100, 100), (130, 255, 255))],
        'green': [((40, 50, 50), (80, 255, 255))],
        'yellow': [((20, 100, 100), (30, 255, 255))],
        'orange': [((10, 100, 100), (20, 255, 255))],
        'purple': [((130, 50, 50), (170, 255, 255))],
        'cyan': [((80, 100, 100), (100, 255, 255))],
    }
    
    def __init__(self):
        super().__init__('color_depth_visualizer')
        
        # Declare parameters
        self.declare_parameter('colors', ['red', 'blue', 'green', 'yellow'])
        self.declare_parameter('min_area', 500)
        self.declare_parameter('background_margin_mm', 50)
        self.declare_parameter('show_visualization', True)
        self.declare_parameter('rgb_topic', '/ascamera/color0/image_raw')
        self.declare_parameter('depth_topic', '/ascamera/depth0/image_raw')
        
        # Get parameters
        self.colors_to_detect = self.get_parameter('colors').value
        self.min_area = self.get_parameter('min_area').value
        self.background_margin = self.get_parameter('background_margin_mm').value
        self.show_viz = self.get_parameter('show_visualization').value
        
        # Initialize
        self.bridge = CvBridge()
        self.rgb_image: Optional[np.ndarray] = None
        self.depth_image: Optional[np.ndarray] = None
        self.last_detections = []
        
        # Create subscribers
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        
        self.rgb_sub = self.create_subscription(
            Image, rgb_topic, self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10
        )
        
        # Create timer for processing
        self.create_timer(0.1, self.process_callback)  # 10 Hz
        
        self.get_logger().info('Color+Depth Visualizer Node Started')
        self.get_logger().info(f'RGB Topic: {rgb_topic}')
        self.get_logger().info(f'Depth Topic: {depth_topic}')
        self.get_logger().info(f'Detecting colors: {self.colors_to_detect}')
    
    def rgb_callback(self, msg: Image):
        """Receive RGB image"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')
    
    def depth_callback(self, msg: Image):
        """Receive depth image"""
        try:
            # Depth is 16UC1 (millimeters)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def process_callback(self):
        """Process RGB-D data when both are available"""
        if self.rgb_image is None or self.depth_image is None:
            return
        
        try:
            # Process the images
            results = self.process_rgbd(
                self.rgb_image.copy(),
                self.depth_image.copy()
            )
            
            # Log detections
            if results['detections'] != self.last_detections:
                self.log_detections(results['detections'])
                self.last_detections = results['detections']
            
            # Show visualization if enabled
            if self.show_viz:
                cv2.imshow('Color+Depth Visualization', results['combined_viz'])
                cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
    
    def process_rgbd(self, rgb_image: np.ndarray, 
                     depth_image: np.ndarray) -> dict:
        """Process RGB-D data"""
        # Estimate background
        background_depth = self._estimate_background_depth(depth_image)
        
        # Create depth mask
        depth_threshold = background_depth - self.background_margin
        depth_mask = (depth_image > 0) & (depth_image < depth_threshold)
        
        # Convert to HSV
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        
        detections = []
        annotated_rgb = rgb_image.copy()
        
        # Detect colors
        for color_name in self.colors_to_detect:
            if color_name not in self.COLOR_RANGES:
                continue
            
            # Create masks
            color_mask = self._create_color_mask(hsv, color_name)
            combined_mask = cv2.bitwise_and(
                color_mask, color_mask,
                mask=depth_mask.astype(np.uint8) * 255
            )
            
            # Find contours
            contours, _ = cv2.findContours(
                combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                
                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2
                
                # Get depth
                roi_depth = depth_image[y:y+h, x:x+w]
                roi_mask = combined_mask[y:y+h, x:x+w]
                depth_values = roi_depth[roi_mask > 0]
                
                if len(depth_values) == 0:
                    continue
                
                median_depth = int(np.median(depth_values))
                
                detection = {
                    'color': color_name,
                    'center': (cx, cy),
                    'bbox': (x, y, w, h),
                    'area': area,
                    'depth_mm': median_depth
                }
                detections.append(detection)
                
                # Draw
                self._draw_detection(annotated_rgb, detection)
        
        # Create visualization
        depth_colormap = self._create_depth_colormap(depth_image)
        combined = self._create_combined_viz(
            annotated_rgb, depth_colormap, detections
        )
        
        return {
            'detections': detections,
            'annotated_rgb': annotated_rgb,
            'combined_viz': combined,
            'background_depth_mm': background_depth
        }
    
    def _estimate_background_depth(self, depth_image: np.ndarray) -> float:
        """Estimate background depth"""
        h, w = depth_image.shape
        edge_size = min(50, h // 10)
        
        edges = [
            depth_image[:edge_size, :],
            depth_image[-edge_size:, :],
            depth_image[:, :edge_size],
            depth_image[:, -edge_size:]
        ]
        
        edge_depths = []
        for edge in edges:
            valid = edge[edge > 0]
            if len(valid) > 0:
                edge_depths.extend(valid.flatten())
        
        return np.median(edge_depths) if edge_depths else 3000
    
    def _create_color_mask(self, hsv_image: np.ndarray, 
                          color_name: str) -> np.ndarray:
        """Create color mask"""
        masks = []
        for lower, upper in self.COLOR_RANGES[color_name]:
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            masks.append(mask)
        
        combined = masks[0]
        for mask in masks[1:]:
            combined = cv2.bitwise_or(combined, mask)
        
        kernel = np.ones((5, 5), np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
        
        return combined
    
    def _create_depth_colormap(self, depth_image: np.ndarray) -> np.ndarray:
        """Create depth visualization"""
        depth_norm = np.copy(depth_image).astype(np.float32)
        depth_norm[depth_norm == 0] = 4000
        depth_norm = np.clip(depth_norm, 200, 4000)
        depth_norm = ((depth_norm - 200) / 3800 * 255).astype(np.uint8)
        
        return cv2.applyColorMap(255 - depth_norm, cv2.COLORMAP_JET)
    
    def _draw_detection(self, image: np.ndarray, detection: dict):
        """Draw detection on image"""
        colors = {
            'red': (0, 0, 255), 'blue': (255, 0, 0), 'green': (0, 255, 0),
            'yellow': (0, 255, 255), 'orange': (0, 165, 255),
            'purple': (255, 0, 255), 'cyan': (255, 255, 0)
        }
        
        color = colors.get(detection['color'], (255, 255, 255))
        x, y, w, h = detection['bbox']
        cx, cy = detection['center']
        
        cv2.rectangle(image, (x, y), (x+w, y+h), color, 2)
        cv2.circle(image, (cx, cy), 5, color, -1)
        
        label = f"{detection['color']}: {detection['depth_mm']}mm"
        cv2.putText(image, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX,
                   0.5, color, 2)
    
    def _create_combined_viz(self, rgb: np.ndarray, depth: np.ndarray,
                            detections: list) -> np.ndarray:
        """Create combined visualization"""
        h, w = rgb.shape[:2]
        combined = np.zeros((h, w*2, 3), dtype=np.uint8)
        
        combined[:, :w] = rgb
        combined[:, w:] = depth
        
        cv2.putText(combined, f"Detections: {len(detections)}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return combined
    
    def log_detections(self, detections: list):
        """Log detection results"""
        if not detections:
            return
        
        self.get_logger().info(f'Detected {len(detections)} object(s):')
        for det in detections:
            self.get_logger().info(
                f"  - {det['color']}: {det['depth_mm']}mm "
                f"at ({det['center'][0]}, {det['center'][1]})"
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = ColorDepthROSNode()
    
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
