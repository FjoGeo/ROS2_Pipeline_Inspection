#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription1 = self.create_subscription(
            Image,
            'realsense1/depth',
            self.depth_callback1,
            10
        )

        self.subscription2 = self.create_subscription(
            Image,
            'realsense2/depth',
            self.depth_callback2,
            10
        )
        

        self.bridge = CvBridge()

    def depth_callback1(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imshow("Depth Image 1", cv_image)
        cv2.waitKey(1)  # Keep the window open until a key is pressed
    
    def depth_callback2(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imshow("Depth Image 2", cv_image)
        cv2.waitKey(1)  # Keep the window open until a key is pressed

def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
