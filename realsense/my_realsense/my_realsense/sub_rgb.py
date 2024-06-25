#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RGBSubscriber(Node):
    def __init__(self):
        super().__init__('rgb_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'realsense/rgb',
            self.rgb_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def rgb_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("RGB Image", cv_image)
        cv2.waitKey(1)  # Keep the window open until a key is pressed

def main(args=None):
    rclpy.init(args=args)
    node = RGBSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
