#!/usr/bin/env python3
"""
Color Identification Script for LidarBot Ascamera
Detects and identifies specific colors in RGB camera stream

Usage:
    python3 color_identifier.py --input /path/to/image.jpg
    # Or for ROS2 integration:
    ros2 run vision_system color_identifier
"""

import cv2
import numpy as np
import argparse
from typing import Dict, List, Tuple


class ColorIdentifier:
    """Identifies specific colors in images using HSV color space"""
    
    # HSV color ranges for common colors
    COLOR_RANGES = {
        'red': [
            # Red wraps around the HSV hue circle, so we need two ranges
            ((0, 100, 100), (10, 255, 255)),
            ((170, 100, 100), (180, 255, 255))
        ],
        'blue': [((100, 100, 100), (130, 255, 255))],
        'green': [((40, 50, 50), (80, 255, 255))],
        'yellow': [((20, 100, 100), (30, 255, 255))],
        'orange': [((10, 100, 100), (20, 255, 255))],
        'purple': [((130, 50, 50), (170, 255, 255))],
        'cyan': [((80, 100, 100), (100, 255, 255))],
        'white': [((0, 0, 200), (180, 30, 255))],
        'black': [((0, 0, 0), (180, 255, 50))],
    }
    
    def __init__(self, min_area: int = 500):
        """
        Initialize the color identifier
        
        Args:
            min_area: Minimum contour area to consider (filters noise)
        """
        self.min_area = min_area
        
    def identify_colors(self, image: np.ndarray, 
                       colors_to_detect: List[str] = None) -> Dict:
        """
        Identify colors in an image
        
        Args:
            image: BGR image from camera
            colors_to_detect: List of color names to detect (default: all)
            
        Returns:
            Dictionary with detection results
        """
        if colors_to_detect is None:
            colors_to_detect = list(self.COLOR_RANGES.keys())
            
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        results = {
            'detections': [],
            'annotated_image': image.copy()
        }
        
        for color_name in colors_to_detect:
            if color_name not in self.COLOR_RANGES:
                print(f"Warning: Unknown color '{color_name}', skipping...")
                continue
                
            # Create mask for this color
            mask = self._create_color_mask(hsv, color_name)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            
            # Process each contour
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                    
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate center
                cx = x + w // 2
                cy = y + h // 2
                
                # Get percentage of image covered
                image_area = image.shape[0] * image.shape[1]
                coverage = (area / image_area) * 100
                
                detection = {
                    'color': color_name,
                    'center': (cx, cy),
                    'bbox': (x, y, w, h),
                    'area': area,
                    'coverage_percent': coverage
                }
                results['detections'].append(detection)
                
                # Draw on annotated image
                self._draw_detection(results['annotated_image'], detection)
        
        return results
    
    def _create_color_mask(self, hsv_image: np.ndarray, 
                          color_name: str) -> np.ndarray:
        """Create a binary mask for a specific color"""
        masks = []
        for lower, upper in self.COLOR_RANGES[color_name]:
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            masks.append(mask)
        
        # Combine all masks for this color
        combined_mask = masks[0]
        for mask in masks[1:]:
            combined_mask = cv2.bitwise_or(combined_mask, mask)
        
        # Morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        
        return combined_mask
    
    def _draw_detection(self, image: np.ndarray, detection: Dict):
        """Draw detection on image"""
        color_name = detection['color']
        x, y, w, h = detection['bbox']
        cx, cy = detection['center']
        
        # Color for drawing (BGR)
        draw_colors = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'yellow': (0, 255, 255),
            'orange': (0, 165, 255),
            'purple': (255, 0, 255),
            'cyan': (255, 255, 0),
            'white': (255, 255, 255),
            'black': (0, 0, 0)
        }
        draw_color = draw_colors.get(color_name, (255, 255, 255))
        
        # Draw bounding box
        cv2.rectangle(image, (x, y), (x + w, y + h), draw_color, 2)
        
        # Draw center point
        cv2.circle(image, (cx, cy), 5, draw_color, -1)
        
        # Draw label
        label = f"{color_name}: {detection['coverage_percent']:.1f}%"
        
        # Background for text
        (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 
                                              0.6, 2)
        cv2.rectangle(image, (x, y - text_h - 10), (x + text_w, y), 
                     draw_color, -1)
        
        # Text color (white or black depending on background)
        text_color = (0, 0, 0) if color_name in ['white', 'yellow', 'cyan'] else (255, 255, 255)
        cv2.putText(image, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, text_color, 2)
    
    def print_results(self, results: Dict):
        """Print detection results to console"""
        detections = results['detections']
        
        if not detections:
            print("No colors detected.")
            return
        
        print(f"\n{'='*60}")
        print(f"Detected {len(detections)} color region(s):")
        print(f"{'='*60}")
        
        for i, det in enumerate(detections, 1):
            print(f"\n{i}. {det['color'].upper()}")
            print(f"   Position: ({det['center'][0]}, {det['center'][1]})")
            print(f"   Area: {det['area']} pixels ({det['coverage_percent']:.2f}% of image)")
            print(f"   Bounding Box: x={det['bbox'][0]}, y={det['bbox'][1]}, "
                  f"w={det['bbox'][2]}, h={det['bbox'][3]}")


def main():
    parser = argparse.ArgumentParser(
        description='Identify colors in images from LidarBot camera'
    )
    parser.add_argument('--input', '-i', 
                       help='Input image path (or use --camera for live feed)')
    parser.add_argument('--camera', '-c', action='store_true',
                       help='Use camera feed (device 0)')
    parser.add_argument('--colors', '-col', nargs='+',
                       default=['red', 'blue', 'green', 'yellow'],
                       help='Colors to detect (default: red blue green yellow)')
    parser.add_argument('--min-area', '-a', type=int, default=500,
                       help='Minimum area threshold (default: 500)')
    parser.add_argument('--output', '-o',
                       help='Save annotated image to this path')
    
    args = parser.parse_args()
    
    # Initialize detector
    detector = ColorIdentifier(min_area=args.min_area)
    
    if args.camera:
        # Live camera feed
        print("Opening camera feed... Press 'q' to quit")
        cap = cv2.VideoCapture(0)
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            # Detect colors
            results = detector.identify_colors(frame, args.colors)
            
            # Show annotated image
            cv2.imshow('Color Detection', results['annotated_image'])
            
            # Print detections
            if results['detections']:
                print(f"\rDetected: {len(results['detections'])} regions", end='')
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
    elif args.input:
        # Single image
        image = cv2.imread(args.input)
        if image is None:
            print(f"Error: Could not load image from {args.input}")
            return
        
        print(f"Processing image: {args.input}")
        print(f"Image size: {image.shape[1]}x{image.shape[0]}")
        print(f"Detecting colors: {', '.join(args.colors)}")
        
        # Detect colors
        results = detector.identify_colors(image, args.colors)
        
        # Print results
        detector.print_results(results)
        
        # Save or display
        if args.output:
            cv2.imwrite(args.output, results['annotated_image'])
            print(f"\nAnnotated image saved to: {args.output}")
        else:
            cv2.imshow('Color Detection', results['annotated_image'])
            print("\nPress any key to close...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    else:
        parser.print_help()
        print("\nError: Must specify either --input or --camera")


if __name__ == "__main__":
    main()
