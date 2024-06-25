#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'realsense/depth',
            self.depth_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imshow("Depth Image", cv_image)
        cv2.waitKey(1)  # Keep the window open until a key is pressed

def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
