#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RGBSubscriber(Node):
    def __init__(self):
        super().__init__('rgb_subscriber')
        
        # Create subscriptions for camera 1 and camera 2
        self.subscription1 = self.create_subscription(
            Image,
            'realsense1/rgb',
            self.rgb_callback1,
            10
        )
        self.subscription2 = self.create_subscription(
            Image,
            'realsense2/rgb',
            self.rgb_callback2,
            10
        )

        self.bridge = CvBridge()

    def rgb_callback1(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("RGB Image Camera 1", cv_image)
        cv2.waitKey(1)  # Keep the window open until a key is pressed

    def rgb_callback2(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("RGB Image Camera 2", cv_image)
        cv2.waitKey(1)  # Keep the window open until a key is pressed

def main(args=None):
    rclpy.init(args=args)
    node = RGBSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()