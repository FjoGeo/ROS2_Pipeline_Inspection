#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('depth_topic1', 'realsense1/depth'),
                ('depth_topic2', 'realsense2/depth'),
                ('resolution', (640, 480)),
                ('frame_rate', 30),
                ('publish_rate', 30.0)
            ]
        )

        self.depth_topic1 = self.get_parameter('depth_topic1').get_parameter_value().string_value
        self.depth_topic2 = self.get_parameter('depth_topic2').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().integer_array_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.depth_publisher1 = self.create_publisher(Image, self.depth_topic1, 10)
        self.depth_publisher2 = self.create_publisher(Image, self.depth_topic2, 10)

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.bridge = CvBridge()

        try:
            self.devices = rs.context().query_devices()
            self.serial_numbers = [dev.get_info(rs.camera_info.serial_number) for dev in self.devices]
            self.pipeline1, self.config1 = self.configure_camera(self.serial_numbers[0])
            self.pipeline2, self.config2 = self.configure_camera(self.serial_numbers[1])

            self.pipeline1.start(self.config1)
            self.pipeline2.start(self.config2)
        except Exception as e:
            self.get_logger().error(f'Failed to start RealSense cameras: {e}')
            self.destroy_node()

    def configure_camera(self, serial_number):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.resolution[0], self.resolution[1], rs.format.z16, self.frame_rate)
        config.enable_device(serial_number)
        return pipeline, config

    def timer_callback(self):
        self.publish_camera_data(self.pipeline1, self.depth_publisher1, "camera1")
        self.publish_camera_data(self.pipeline2, self.depth_publisher2, "camera2")

    def publish_camera_data(self, pipeline, depth_publisher, frame_id):
        try:
            frames = pipeline.wait_for_frames(timeout_ms=500)
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                self.get_logger().warn(f'No depth frame received for {frame_id}')
                return

            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding="bgr8")
            depth_publisher.publish(depth_msg)

            self.get_logger().info(f'Published Depth for {frame_id}')
        except Exception as e:
            self.get_logger().error(f'Error publishing data for {frame_id}: {e}')

    def destroy(self):
        self.pipeline1.stop()
        self.pipeline2.stop()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
