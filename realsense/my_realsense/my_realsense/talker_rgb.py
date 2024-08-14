#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        
        self.rgb_publisher1 = self.create_publisher(Image, 'realsense1/rgb', 10)
        self.rgb_publisher2 = self.create_publisher(Image, 'realsense2/rgb', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10Hz
        self.bridge = CvBridge()

        self.devices = rs.context().query_devices()
        self.serial_numbers = [dev.get_info(rs.camera_info.serial_number) for dev in self.devices]

        self.pipeline1, self.config1 = self.configure_camera(self.serial_numbers[0])
        self.pipeline2, self.config2 = self.configure_camera(self.serial_numbers[1])

        self.pipeline1.start(self.config1)
        self.pipeline2.start(self.config2)



    def configure_camera(self, serial_number):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_device(serial_number)
        return pipeline, config


    def timer_callback(self):
        frames1 = self.pipeline1.wait_for_frames()
        color_frame1 = frames1.get_color_frame()
     
        frames2 = self.pipeline2.wait_for_frames()
        color_frame2 = frames2.get_color_frame()
    
        if color_frame1:
            self.publish_data(color_frame1, self.rgb_publisher1,"camera1")
        if color_frame2:
            self.publish_data(color_frame2, self.rgb_publisher2,"camera2")
       

    def publish_data(self,         
                     color_frame, 
                     rgb_publisher, 
                     frame_id):

        color_image = np.asanyarray(color_frame.get_data())
        rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        rgb_publisher.publish(rgb_msg)

        self.get_logger().info(f'Published RGB  for {frame_id}')


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
