#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealSenseIRPublisher(Node):
    def __init__(self):
        super().__init__('realsense_IR_publisher')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('IR_topic1', 'realsense1/IR'),
                ('IR_topic2', 'realsense2/IR'),
                ('resolution', (640, 480)),
                ('frame_rate', 30),
                ('publish_rate', 10.0)
            ]
        )

        self.IR_topic1 = self.get_parameter('IR_topic1').get_parameter_value().string_value
        self.IR_topic2 = self.get_parameter('IR_topic2').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().integer_array_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.IR_publisher1 = self.create_publisher(Image, self.IR_topic1, 10)
        self.IR_publisher2 = self.create_publisher(Image, self.IR_topic2, 10)

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
        points = rs.points()
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.infrared, 1, self.resolution[0], self.resolution[1], rs.format.y8, self.frame_rate)
        config.enable_stream(rs.stream.infrared, 2, self.resolution[0], self.resolution[1], rs.format.y8, self.frame_rate)
        config.enable_device(serial_number)
        return pipeline, config

    def timer_callback(self):
        self.publish_camera_data(self.pipeline1, self.IR_publisher1, "camera1")
        self.publish_camera_data(self.pipeline2, self.IR_publisher2, "camera2")

    def publish_camera_data(self, pipeline, IR_publisher, frame_id):
        try:
            frames = pipeline.wait_for_frames(timeout_ms=500)
            nir_lf_frame = frames.get_infrared_frame(1)
            nir_rg_frame = frames.get_infrared_frame(2)
            if not nir_lf_frame or not nir_rg_frame:
                self.get_logger().warn(f'No IR frame received for {frame_id}')
                return

            nir_lf_image = np.asanyarray(nir_lf_frame.get_data())
            nir_rg_image = np.asanyarray(nir_rg_frame.get_data())
            IR_image = np.hstack((nir_lf_image, nir_rg_image))

            IR_msg = self.bridge.cv2_to_imgmsg(IR_image, encoding="mono8")
            IR_publisher.publish(IR_msg)

            self.get_logger().info(f'Published IR image for {frame_id}')
        except Exception as e:
            self.get_logger().error(f'Error publishing data for {frame_id}: {e}')

    def destroy(self):
        self.pipeline1.stop()
        self.pipeline2.stop()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseIRPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
