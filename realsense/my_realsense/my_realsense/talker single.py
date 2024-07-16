#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        self.rgb_publisher = self.create_publisher(Image, 'realsense/rgb', 10)
        self.depth_publisher = self.create_publisher(Image, 'realsense/depth', 10)
        self.pc_publisher = self.create_publisher(PointCloud2, 'realsense/pointcloud', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10Hz
        self.bridge = CvBridge()

        # Initialize RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline.start(self.config)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Publish RGB image
        rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.rgb_publisher.publish(rgb_msg)

        # Publish depth image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        self.depth_publisher.publish(depth_msg)

        # Publish point cloud
        points = self.create_pointcloud(depth_frame, color_image, color_intrin=color_frame.profile.as_video_stream_profile().intrinsics)
        self.pc_publisher.publish(points)

        self.get_logger().info('Published RGB, depth, and point cloud data')

    def create_pointcloud(self, depth_frame, color_image, color_intrin):
        # Get intrinsics
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(depth_frame.profile)

        # PointCloud2 setup
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"

        points = []
        for y in range(depth_intrin.height):
            for x in range(depth_intrin.width):
                depth = depth_frame.get_distance(x, y)
                if 0 < depth < 10:  # ignore no data
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

def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
