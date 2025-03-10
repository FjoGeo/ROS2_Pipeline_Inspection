import time
import ctypes as ct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import sys 
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "pylinllt"))
import pylinllt as llt

class LaserScannerNode(Node):
    def __init__(self):
        super().__init__('laser_scanner_node')
        self.publisher_ = self.create_publisher(PointCloud2, 'laser_scan', 10)
        self.timer = self.create_timer(0.04, self.scan_and_publish)

        self.init_scanner()
        self.get_logger().info("Laser Scanner Node Started")

    def init_scanner(self):
        self.scanner_type = ct.c_int(0)
        self.timestamp = (ct.c_ubyte * 16)()
        self.available_resolutions = (ct.c_uint * 4)()
        self.available_interfaces = [ct.create_string_buffer(8) for _ in range(6)]
        self.available_interfaces_p = (ct.c_char_p * 6)(*map(ct.addressof, self.available_interfaces))
        self.lost_profiles = ct.c_uint()
        self.profile_count = ct.c_uint()
        self.null_ptr_short = ct.POINTER(ct.c_ushort)()
        self.null_ptr_int = ct.POINTER(ct.c_uint)()

        self.hLLT = llt.create_llt_device()

        if llt.get_device_interfaces(self.available_interfaces_p, len(self.available_interfaces)) < 1:
            raise ValueError("Error getting interfaces")
        if llt.set_device_interface(self.hLLT, self.available_interfaces[0]) < 1:
            raise ValueError("Error setting device interface")
        if llt.connect(self.hLLT) < 1:
            raise ConnectionError("Error connecting")
        if llt.get_resolutions(self.hLLT, self.available_resolutions, len(self.available_resolutions)) < 1:
            raise ValueError("Error getting resolutions")

        self.resolution = self.available_resolutions[0]
        if llt.set_resolution(self.hLLT, self.resolution) < 1:
            raise ValueError("Error setting resolution")

        self.profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        self.x = (ct.c_double * self.resolution)()
        self.z = (ct.c_double * self.resolution)()
        self.intensities = (ct.c_ushort * self.resolution)()

        if llt.get_llt_type(self.hLLT, ct.byref(self.scanner_type)) < 1:
            raise ValueError("Error getting scanner type")
        if llt.set_profile_config(self.hLLT, llt.TProfileConfig.PROFILE) < 1:
            raise ValueError("Error setting profile config")
        if llt.set_feature(self.hLLT, llt.FEATURE_FUNCTION_TRIGGER, llt.TRIG_INTERNAL) < 1:
            raise ValueError("Error setting trigger")
        if llt.set_feature(self.hLLT, llt.FEATURE_FUNCTION_EXPOSURE_TIME, 100) < 1:
            raise ValueError("Error setting exposure time")
        if llt.set_feature(self.hLLT, llt.FEATURE_FUNCTION_IDLE_TIME, 3900) < 1:
            raise ValueError("Error setting idle time")
        if llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1) < 1:
            raise ValueError("Error starting transfer profiles")
        time.sleep(0.2)

    def scan_and_publish(self):
        ret = llt.get_actual_profile(self.hLLT, self.profile_buffer, len(self.profile_buffer), llt.TProfileConfig.PROFILE, ct.byref(self.lost_profiles))
        if ret != len(self.profile_buffer):
            self.get_logger().error(f"Error getting profile data: {ret}")
            return

        ret = llt.convert_profile_2_values(self.profile_buffer, len(self.profile_buffer), self.resolution, llt.TProfileConfig.PROFILE, self.scanner_type, 0,
                                           self.null_ptr_short, self.intensities, self.null_ptr_short, self.x, self.z, self.null_ptr_int, self.null_ptr_int)
        if ret & llt.CONVERT_X == 0 or ret & llt.CONVERT_Z == 0 or ret & llt.CONVERT_MAXIMUM == 0:
            self.get_logger().error("Error converting data")
            return

        point_cloud_msg = self.create_pointcloud_msg()
        self.publisher_.publish(point_cloud_msg)

    def create_pointcloud_msg(self):
        header = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        points = np.array([(self.x[i], self.z[i], float(self.intensities[i])) for i in range(self.resolution)], dtype=np.float32)
        data = struct.pack(f"{len(points) * 3}f", *points.flatten())

        point_cloud_msg = PointCloud2()
        point_cloud_msg.header.stamp = header
        point_cloud_msg.header.frame_id = "laser_frame"
        point_cloud_msg.height = 1
        point_cloud_msg.width = len(points)
        point_cloud_msg.fields = fields
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 12  # 3 float32 values (4 bytes each)
        point_cloud_msg.row_step = point_cloud_msg.point_step * len(points)
        point_cloud_msg.data = data
        point_cloud_msg.is_dense = True

        return point_cloud_msg

    def destroy_node(self):
        ret = llt.transfer_profiles(self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        if ret < 1:
            self.get_logger().error("Error stopping transfer profiles")
        ret = llt.disconnect(self.hLLT)
        if ret < 1:
            self.get_logger().error("Error disconnecting")
        ret = llt.del_device(self.hLLT)
        if ret < 1:
            self.get_logger().error("Error deleting device")
        self.get_logger().info("Scanner disconnected.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaserScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
