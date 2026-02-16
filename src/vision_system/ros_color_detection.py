#!/usr/bin/env python3
"""
ROS-Compatible Color Detection Node
Subscribes to ROS camera topics and performs HSV color filtering.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('h_min', 35)
        self.declare_parameter('h_max', 85)
        self.declare_parameter('s_min', 50)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 50)
        self.declare_parameter('v_max', 255)
        self.declare_parameter('show_windows', True)
        
        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            10)
            
        # Publisher for processed image
        self.publisher_ = self.create_publisher(Image, '/vision/color_mask', 10)
        
        self.get_logger().info(f'Color Detection Node started on topic: {self.get_parameter("camera_topic").value}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Get current parameter values
            lower = np.array([
                self.get_parameter('h_min').value,
                self.get_parameter('s_min').value,
                self.get_parameter('v_min').value
            ])
            upper = np.array([
                self.get_parameter('h_max').value,
                self.get_parameter('s_max').value,
                self.get_parameter('v_max').value
            ])
            
            mask = cv2.inRange(hsv, lower, upper)
            
            # Noise removal
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(frame, "Detected", (x, y-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Publish mask
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            self.publisher_.publish(mask_msg)
            
            # Show processed frame if enabled
            if self.get_parameter('show_windows').value:
                cv2.imshow("Detection Result", frame)
                cv2.imshow("Mask", mask)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
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
