#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class RealSensePointCloudPublisher(Node):
    def __init__(self):
        super().__init__('realsense_pointcloud_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pc_topic1', 'realsense1/pointcloud'),
                ('pc_topic2', 'realsense2/pointcloud'),
                ('resolution', (640, 480)),
                ('frame_rate', 30), # 30 standard
                ('publish_rate', 30.0) # 10
            ]
        )

        self.pc_topic1 = self.get_parameter('pc_topic1').get_parameter_value().string_value
        self.pc_topic2 = self.get_parameter('pc_topic2').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().integer_array_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.pc_publisher1 = self.create_publisher(PointCloud2, self.pc_topic1, 10)
        self.pc_publisher2 = self.create_publisher(PointCloud2, self.pc_topic2, 10)

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
        config.enable_stream(rs.stream.color, self.resolution[0], self.resolution[1], rs.format.bgr8, self.frame_rate)
        config.enable_device(serial_number)
        return pipeline, config

    def timer_callback(self):
        self.publish_camera_data(self.pipeline1, self.pc_publisher1, "camera1")
        self.publish_camera_data(self.pipeline2, self.pc_publisher2, "camera2")

    def publish_camera_data(self, pipeline, pc_publisher, frame_id):
        try:
            frames = pipeline.wait_for_frames(timeout_ms=500)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                self.get_logger().warn(f'No frames received for {frame_id}')
                return

            color_image = np.asanyarray(color_frame.get_data())
            point_cloud = self.create_pointcloud(depth_frame, color_image, color_frame.profile.as_video_stream_profile().intrinsics, frame_id)
            pc_publisher.publish(point_cloud)

            self.get_logger().info(f'Published point cloud data for {frame_id}')
        except Exception as e:
            self.get_logger().error(f'Error publishing data for {frame_id}: {e}')

    def create_pointcloud(self, depth_frame, color_image, color_intrin, frame_id):
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(depth_frame.profile)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        points = []
        for y in range(depth_intrin.height):
            for x in range(depth_intrin.width):
                depth = depth_frame.get_distance(x, y)
                if 0 < depth < 10:  # Ignore invalid depth values
                    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
                    color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
                    color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
                    color_pixel = [int(i) for i in color_pixel]
                    if 0 <= color_pixel[0] < color_intrin.width and 0 <= color_pixel[1] < color_intrin.height:
                        b, g, r = color_image[color_pixel[1], color_pixel[0]]
                        points.append([depth_point[0], depth_point[1], depth_point[2], r, g, b])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
        ]

        return pc2.create_cloud(header, fields, points)

    def destroy(self):
        self.pipeline1.stop()
        self.pipeline2.stop()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
