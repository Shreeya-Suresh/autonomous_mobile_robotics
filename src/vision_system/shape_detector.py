#!/usr/bin/env python3
"""
Shape Detector

Detects and classifies shapes (circle, square, triangle, rectangle) from RGB camera.
Uses contour approximation and geometric properties for classification.

Shapes detected:
- Circle: High circularity ratio
- Triangle: 3 vertices
- Square: 4 vertices, aspect ratio ~1
- Rectangle: 4 vertices, aspect ratio != 1

Usage:
    python3 shape_detector.py

Remote Display (SSH X11 Forwarding):
    ssh -X user@raspberry_pi_ip
    python3 shape_detector.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class ShapeDetector(Node):
    def __init__(self):
        super().__init__('shape_detector')
        
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('min_area', 500)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('circularity_threshold', 0.75)
        self.declare_parameter('show_windows', True)
        
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.circularity_threshold = self.get_parameter('circularity_threshold').value
        self.show_windows = self.get_parameter('show_windows').value
        
        # Subscribe to RGB camera
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/color0/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for annotated image
        self.image_pub = self.create_publisher(Image, '/vision/detected_shapes', 10)
        
        self.get_logger().info('Shape Detector started')
        self.get_logger().info(f'Subscribing to: /ascamera/color0/image_raw')
    
    def detect_shape(self, contour):
        """
        Classify shape based on contour properties
        
        Returns:
            tuple: (shape_name, color_for_drawing)
        """
        # Calculate perimeter and area
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        
        # Calculate circularity (4π * area / perimeter²)
        # Perfect circle = 1.0, square ≈ 0.785
        if perimeter == 0:
            return "unknown", (128, 128, 128)
        
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        
        # Check for circle first
        if circularity > self.circularity_threshold:
            return "circle", (255, 0, 255)  # Magenta
        
        # Approximate polygon
        epsilon = 0.04 * perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)
        vertices = len(approx)
        
        # Classify based on number of vertices
        if vertices == 3:
            return "triangle", (0, 255, 255)  # Yellow
        
        elif vertices == 4:
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h if h != 0 else 0
            
            # Square if aspect ratio close to 1
            if 0.9 <= aspect_ratio <= 1.1:
                return "square", (0, 255, 0)  # Green
            else:
                return "rectangle", (255, 255, 0)  # Cyan
        
        elif vertices == 5:
            return "pentagon", (255, 128, 0)  # Orange
        
        elif vertices == 6:
            return "hexagon", (128, 0, 255)  # Purple
        
        elif vertices > 6:
            # Many vertices could be a circle with noisy edges
            if circularity > 0.6:
                return "circle", (255, 0, 255)
            else:
                return "polygon", (128, 128, 128)  # Gray
        
        return "unknown", (128, 128, 128)
    
    def image_callback(self, msg):
        """Process incoming RGB image"""
        try:
            # Convert to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Edge detection
            edges = cv2.Canny(blurred, 50, 150)
            
            # Morphological closing to connect edges
            kernel = np.ones((3, 3), np.uint8)
            edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Annotated image
            annotated = frame.copy()
            shape_counts = {}
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                
                # Filter by area
                if area < self.min_area or area > self.max_area:
                    continue
                
                # Detect shape
                shape, color = self.detect_shape(cnt)
                
                # Count shapes
                shape_counts[shape] = shape_counts.get(shape, 0) + 1
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(cnt)
                
                # Draw contour
                cv2.drawContours(annotated, [cnt], -1, color, 2)
                
                # Draw bounding box
                cv2.rectangle(annotated, (x, y), (x+w, y+h), color, 2)
                
                # Add label
                label = f'{shape}'
                cv2.putText(annotated, label, (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                # Add area info
                area_text = f'{int(area)}px'
                cv2.putText(annotated, area_text, (x, y+h+20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # Add summary overlay
            y_offset = 30
            for shape, count in sorted(shape_counts.items()):
                summary_text = f'{shape}: {count}'
                cv2.putText(annotated, summary_text, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                y_offset += 30
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.image_pub.publish(annotated_msg)
            
            # Show windows if enabled
            if self.show_windows:
                cv2.imshow('Shape Detection', annotated)
                cv2.imshow('Edges', edges)
                cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetector()
    
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

    finally:
        if node.show_windows:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
