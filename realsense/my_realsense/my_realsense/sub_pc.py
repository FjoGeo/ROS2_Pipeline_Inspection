#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'realsense/pointcloud',
            self.pointcloud_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def pointcloud_callback(self, msg):
        # Decode PointCloud2 message
        points = self.decode_pointcloud2(msg)

        # Process or print the point cloud data
        for i, point in enumerate(points):
            x, y, z, r, g, b = point
            print(f"Point {i}: ({x:.2f}, {y:.2f}, {z:.2f}), RGB: ({r}, {g}, {b})")

    def decode_pointcloud2(self, msg):
        # Extract point cloud data
        points = []
        data = msg.data
        offset = msg.fields[0].offset  # Offset of the first field
        point_step = msg.point_step  # Length of a point in bytes

        for i in range(msg.width * msg.height):
            # Unpack x, y, z, rgb from binary data
            x, y, z = struct.unpack_from('fff', data, offset)
            rgb = struct.unpack_from('B', data, offset + 12)[0]
            r = (rgb >> 16) & 0x0000ff
            g = (rgb >> 8) & 0x0000ff
            b = rgb & 0x0000ff
            points.append((x, y, z, r, g, b))
            offset += point_step

        return points

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
