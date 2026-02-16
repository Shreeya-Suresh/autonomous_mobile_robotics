#!/usr/bin/env python3
"""
Color and Depth Differentiation Script for LidarBot Ascamera RGB-D
Visualizes objects using both color and depth information

Usage:
    python3 color_depth_visualizer.py --rgb color.jpg --depth depth.png
    # Or for ROS2 integration (subscribes to camera topics)
"""

import cv2
import numpy as np
import argparse
from typing import Dict, List, Tuple, Optional


class ColorDepthVisualizer:
    """Combines color detection with depth information for enhanced visualization"""
    
    # HSV color ranges (same as color_identifier.py)
    COLOR_RANGES = {
        'red': [((0, 100, 100), (10, 255, 255)), ((170, 100, 100), (180, 255, 255))],
        'blue': [((100, 100, 100), (130, 255, 255))],
        'green': [((40, 50, 50), (80, 255, 255))],
        'yellow': [((20, 100, 100), (30, 255, 255))],
        'orange': [((10, 100, 100), (20, 255, 255))],
        'purple': [((130, 50, 50), (170, 255, 255))],
        'cyan': [((80, 100, 100), (100, 255, 255))],
    }
    
    def __init__(self, min_area: int = 500, background_margin_mm: int = 50):
        """
        Initialize the visualizer
        
        Args:
            min_area: Minimum contour area to consider
            background_margin_mm: Objects must be this much closer than background
        """
        self.min_area = min_area
        self.background_margin_mm = background_margin_mm
        
    def process_rgbd(self, rgb_image: np.ndarray, 
                     depth_image: np.ndarray,
                     colors_to_detect: List[str] = None) -> Dict:
        """
        Process RGB-D data to identify colored objects with depth
        
        Args:
            rgb_image: BGR color image
            depth_image: 16UC1 depth image (millimeters)
            colors_to_detect: List of colors to detect
            
        Returns:
            Dictionary with detection results and visualizations
        """
        if colors_to_detect is None:
            colors_to_detect = list(self.COLOR_RANGES.keys())
        
        # Estimate background depth
        background_depth = self._estimate_background_depth(depth_image)
        
        # Create depth mask (objects closer than background)
        depth_threshold = background_depth - self.background_margin_mm
        depth_mask = (depth_image > 0) & (depth_image < depth_threshold)
        
        # Convert RGB to HSV
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        
        results = {
            'detections': [],
            'annotated_rgb': rgb_image.copy(),
            'depth_colormap': self._create_depth_colormap(depth_image),
            'combined_viz': None,
            'background_depth_mm': background_depth
        }
        
        # Detect each color
        for color_name in colors_to_detect:
            if color_name not in self.COLOR_RANGES:
                continue
            
            # Create color mask
            color_mask = self._create_color_mask(hsv, color_name)
            
            # Combine with depth mask
            combined_mask = cv2.bitwise_and(color_mask, color_mask, 
                                           mask=depth_mask.astype(np.uint8) * 255)
            
            # Find contours
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2
                
                # Get depth statistics for this region
                roi_depth = depth_image[y:y+h, x:x+w]
                roi_mask = combined_mask[y:y+h, x:x+w]
                
                depth_values = roi_depth[roi_mask > 0]
                if len(depth_values) == 0:
                    continue
                
                median_depth = np.median(depth_values)
                min_depth = np.min(depth_values)
                max_depth = np.max(depth_values)
                
                detection = {
                    'color': color_name,
                    'center': (cx, cy),
                    'bbox': (x, y, w, h),
                    'area': area,
                    'depth_mm': int(median_depth),
                    'depth_min_mm': int(min_depth),
                    'depth_max_mm': int(max_depth),
                    'distance_from_bg_mm': int(background_depth - median_depth)
                }
                results['detections'].append(detection)
                
                # Draw on annotated RGB
                self._draw_detection_with_depth(results['annotated_rgb'], 
                                               detection)
        
        # Create combined visualization
        results['combined_viz'] = self._create_combined_visualization(
            results['annotated_rgb'], 
            results['depth_colormap'],
            results['detections']
        )
        
        return results
    
    def _estimate_background_depth(self, depth_image: np.ndarray) -> float:
        """Estimate background depth by sampling image edges"""
        h, w = depth_image.shape
        edge_size = min(50, h // 10)
        
        # Sample top, bottom, left, right edges
        edges = [
            depth_image[:edge_size, :],  # top
            depth_image[-edge_size:, :],  # bottom
            depth_image[:, :edge_size],  # left
            depth_image[:, -edge_size:]  # right
        ]
        
        edge_depths = []
        for edge in edges:
            valid_depths = edge[edge > 0]
            if len(valid_depths) > 0:
                edge_depths.extend(valid_depths.flatten())
        
        if len(edge_depths) == 0:
            return 3000  # Default 3 meters
        
        return np.median(edge_depths)
    
    def _create_color_mask(self, hsv_image: np.ndarray, 
                          color_name: str) -> np.ndarray:
        """Create binary mask for a color"""
        masks = []
        for lower, upper in self.COLOR_RANGES[color_name]:
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            masks.append(mask)
        
        combined_mask = masks[0]
        for mask in masks[1:]:
            combined_mask = cv2.bitwise_or(combined_mask, mask)
        
        # Noise reduction
        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        
        return combined_mask
    
    def _create_depth_colormap(self, depth_image: np.ndarray, 
                              min_range: int = 200, 
                              max_range: int = 4000) -> np.ndarray:
        """Convert depth image to color visualization"""
        # Normalize depth to 0-255
        depth_normalized = np.copy(depth_image).astype(np.float32)
        depth_normalized[depth_normalized == 0] = max_range  # Set invalid to max
        
        depth_normalized = np.clip(depth_normalized, min_range, max_range)
        depth_normalized = ((depth_normalized - min_range) / 
                           (max_range - min_range) * 255).astype(np.uint8)
        
        # Apply colormap (closer = red/yellow, farther = blue/purple)
        colormap = cv2.applyColorMap(255 - depth_normalized, cv2.COLORMAP_JET)
        
        return colormap
    
    def _draw_detection_with_depth(self, image: np.ndarray, detection: Dict):
        """Draw detection with depth information"""
        color_name = detection['color']
        x, y, w, h = detection['bbox']
        cx, cy = detection['center']
        depth_mm = detection['depth_mm']
        
        # Drawing color
        draw_colors = {
            'red': (0, 0, 255), 'blue': (255, 0, 0), 'green': (0, 255, 0),
            'yellow': (0, 255, 255), 'orange': (0, 165, 255),
            'purple': (255, 0, 255), 'cyan': (255, 255, 0)
        }
        draw_color = draw_colors.get(color_name, (255, 255, 255))
        
        # Draw bounding box
        cv2.rectangle(image, (x, y), (x + w, y + h), draw_color, 2)
        
        # Draw center with distance
        cv2.circle(image, (cx, cy), 5, draw_color, -1)
        cv2.line(image, (cx, cy), (cx, cy - 20), draw_color, 2)
        
        # Labels
        label1 = f"{color_name.upper()}"
        label2 = f"{depth_mm}mm ({depth_mm/10:.1f}cm)"
        
        # Draw labels with background
        y_offset = y - 10
        for label in [label1, label2]:
            (text_w, text_h), _ = cv2.getTextSize(label, 
                                                   cv2.FONT_HERSHEY_SIMPLEX, 
                                                   0.5, 2)
            cv2.rectangle(image, (x, y_offset - text_h - 5), 
                         (x + text_w + 5, y_offset), draw_color, -1)
            cv2.putText(image, label, (x + 2, y_offset - 3), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            y_offset -= text_h + 8
    
    def _create_combined_visualization(self, annotated_rgb: np.ndarray,
                                      depth_colormap: np.ndarray,
                                      detections: List[Dict]) -> np.ndarray:
        """Create side-by-side visualization with statistics"""
        h, w = annotated_rgb.shape[:2]
        
        # Create canvas
        combined = np.zeros((h + 100, w * 2, 3), dtype=np.uint8)
        
        # Place RGB and depth side by side
        combined[:h, :w] = annotated_rgb
        combined[:h, w:] = depth_colormap
        
        # Add labels
        cv2.putText(combined, "COLOR + DEPTH", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined, "DEPTH MAP", (w + 10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Add statistics panel at bottom
        stats_y = h + 20
        cv2.putText(combined, f"Detected Objects: {len(detections)}", 
                   (10, stats_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                   (255, 255, 255), 2)
        
        # List detections
        for i, det in enumerate(detections[:5]):  # Show first 5
            text = (f"{i+1}. {det['color']}: {det['depth_mm']}mm "
                   f"({det['area']} px)")
            cv2.putText(combined, text, (10, stats_y + 25 + i * 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return combined
    
    def print_results(self, results: Dict):
        """Print detailed results"""
        detections = results['detections']
        bg_depth = results['background_depth_mm']
        
        print(f"\n{'='*70}")
        print(f"COLOR + DEPTH ANALYSIS")
        print(f"{'='*70}")
        print(f"Background depth: {bg_depth:.0f}mm ({bg_depth/10:.1f}cm)")
        print(f"Detected objects: {len(detections)}")
        print(f"{'='*70}")
        
        if not detections:
            print("No colored objects detected in foreground.")
            return
        
        # Sort by distance (closest first)
        sorted_detections = sorted(detections, key=lambda d: d['depth_mm'])
        
        for i, det in enumerate(sorted_detections, 1):
            print(f"\n{i}. {det['color'].upper()}")
            print(f"   Distance: {det['depth_mm']}mm ({det['depth_mm']/10:.1f}cm)")
            print(f"   From background: {det['distance_from_bg_mm']}mm closer")
            print(f"   Position: ({det['center'][0]}, {det['center'][1]})")
            print(f"   Area: {det['area']} pixels")
            print(f"   Depth range: {det['depth_min_mm']}-{det['depth_max_mm']}mm")


def main():
    parser = argparse.ArgumentParser(
        description='Visualize color and depth data from LidarBot RGB-D camera'
    )
    parser.add_argument('--rgb', '-r', 
                       help='RGB image path')
    parser.add_argument('--depth', '-d', 
                       help='Depth image path (16-bit PNG)')
    parser.add_argument('--colors', '-c', nargs='+',
                       default=['red', 'blue', 'green', 'yellow'],
                       help='Colors to detect')
    parser.add_argument('--min-area', '-a', type=int, default=500,
                       help='Minimum area threshold')
    parser.add_argument('--background-margin', '-b', type=int, default=50,
                       help='Background margin in mm (default: 50)')
    parser.add_argument('--output', '-o',
                       help='Save combined visualization to this path')
    
    args = parser.parse_args()
    
    if not args.rgb or not args.depth:
        parser.print_help()
        print("\nError: Both --rgb and --depth images are required")
        return
    
    # Load images
    print(f"Loading RGB image: {args.rgb}")
    rgb_image = cv2.imread(args.rgb)
    if rgb_image is None:
        print(f"Error: Could not load RGB image from {args.rgb}")
        return
    
    print(f"Loading depth image: {args.depth}")
    depth_image = cv2.imread(args.depth, cv2.IMREAD_ANYDEPTH)
    if depth_image is None:
        print(f"Error: Could not load depth image from {args.depth}")
        return
    
    # Verify depth image is 16-bit
    if depth_image.dtype != np.uint16:
        print(f"Warning: Depth image is {depth_image.dtype}, expected uint16")
        depth_image = depth_image.astype(np.uint16)
    
    print(f"RGB size: {rgb_image.shape}")
    print(f"Depth size: {depth_image.shape}")
    
    # Initialize visualizer
    visualizer = ColorDepthVisualizer(
        min_area=args.min_area,
        background_margin_mm=args.background_margin
    )
    
    # Process
    print(f"\nProcessing... Detecting: {', '.join(args.colors)}")
    results = visualizer.process_rgbd(rgb_image, depth_image, args.colors)
    
    # Print results
    visualizer.print_results(results)
    
    # Save or display
    if args.output:
        cv2.imwrite(args.output, results['combined_viz'])
        print(f"\nVisualization saved to: {args.output}")
    else:
        cv2.imshow('Color + Depth Visualization', results['combined_viz'])
        print("\nPress any key to close...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
