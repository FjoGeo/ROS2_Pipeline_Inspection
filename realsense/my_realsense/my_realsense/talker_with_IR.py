#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header, Float32, String, Float32MultiArray
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        
        # Create publishers for camera 1
        self.rgb_publisher1 = self.create_publisher(Image, 'realsense1/rgb', 10)
        self.depth_publisher1 = self.create_publisher(Image, 'realsense1/depth', 10)
        self.pc_publisher1 = self.create_publisher(PointCloud2, 'realsense1/pointcloud', 10)
        self.accel_publisher1 = self.create_publisher(Float32MultiArray, 'realsense1/accel', 10)
        self.gyro_publisher1 = self.create_publisher(Float32MultiArray, 'realsense1/gyro', 10)
        self.IR_publisher1 = self.create_publisher(Image, 'realsense1/IR', 10)

        # Create publishers for camera 2
        self.rgb_publisher2 = self.create_publisher(Image, 'realsense2/rgb', 10)
        self.depth_publisher2 = self.create_publisher(Image, 'realsense2/depth', 10)
        self.pc_publisher2 = self.create_publisher(PointCloud2, 'realsense2/pointcloud', 10)
        self.accel_publisher2 = self.create_publisher(Float32MultiArray, 'realsense2/accel', 10)
        self.gyro_publisher2 = self.create_publisher(Float32MultiArray, 'realsense2/gyro', 10)
        self.IR_publisher2 = self.create_publisher(Image, 'realsense2/IR', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10Hz
        self.bridge = CvBridge()

        self.devices = rs.context().query_devices()
        self.serial_numbers = [dev.get_info(rs.camera_info.serial_number) for dev in self.devices]

        # Initialize RealSense for two cameras
        self.pipeline1, self.config1 = self.configure_camera(self.serial_numbers[0])
        self.pipeline2, self.config2 = self.configure_camera(self.serial_numbers[1])

        self.pipeline1.start(self.config1)
        self.pipeline2.start(self.config2)


    @staticmethod
    def gyro_data(gyro):
        return np.array([gyro.x, gyro.y, gyro.z])
    

    @staticmethod
    def accel_data(accel):
        return np.array([accel.x, accel.y, accel.z])


    def configure_camera(self, serial_number):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

        config.enable_device(serial_number)
        return pipeline, config


    def timer_callback(self):
        # Get frames from camera 1
        frames1 = self.pipeline1.wait_for_frames()
        accel1 = self.accel_data(frames1[2].as_motion_frame().get_motion_data())
        gyro1 = self.gyro_data(frames1[3].as_motion_frame().get_motion_data())
        depth_frame1 = frames1.get_depth_frame()
        color_frame1 = frames1.get_color_frame()
        nir_lf_frame1 = frames1.get_infrared_frame(1)
        nir_rg_frame1 = frames1.get_infrared_frame(2)

        # Get frames from camera 2
        frames2 = self.pipeline2.wait_for_frames()
        accel2 = self.accel_data(frames2[2].as_motion_frame().get_motion_data())
        gyro2 = self.gyro_data(frames2[3].as_motion_frame().get_motion_data())
        depth_frame2 = frames2.get_depth_frame()
        color_frame2 = frames2.get_color_frame()
        nir_lf_frame2 = frames2.get_infrared_frame(1)
        nir_rg_frame2 = frames2.get_infrared_frame(2)

        if depth_frame1 and color_frame1:
            self.publish_data(depth_frame1, 
                                color_frame1,
                                nir_lf_frame1,
                                nir_rg_frame1, 
                                self.rgb_publisher1, 
                                self.depth_publisher1,
                                self.pc_publisher1, 
                                self.IR_publisher1,
                                "camera1")
        if depth_frame2 and color_frame2:
            self.publish_data(depth_frame2, 
                                color_frame2, 
                                nir_lf_frame2, 
                                nir_rg_frame2, 
                                self.rgb_publisher2, 
                                self.depth_publisher2, 
                                self.pc_publisher2,  
                                self.IR_publisher2,
                                "camera2")
        if accel1.any() and gyro1.any():
            self.publish_accel_gyro(accel1, gyro1, self.accel_publisher1, self.gyro_publisher1, "camera1")
        if accel2.any() and gyro2.any():
            self.publish_accel_gyro(accel2, gyro2, self.accel_publisher2, self.gyro_publisher2, "camera2")


    def publish_data(self, depth_frame, 
                     color_frame, 
                     nir_lf_frame, 
                     nir_rg_frame, 
                     rgb_publisher, 
                     depth_publisher, 
                     pc_publisher, 
                     IR_publisher,
                     frame_id):
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        nir_lf_image = np.asanyarray(nir_lf_frame.get_data())
        nir_rg_image = np.asanyarray(nir_rg_frame.get_data())
        IR_image = np.hstack((nir_lf_image, nir_rg_image)) 

        # Publish RGB image
        rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        rgb_publisher.publish(rgb_msg)

        # Publish depth image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        depth_publisher.publish(depth_msg)

        # Publish IR image
        IR_msg = self.bridge.cv2_to_imgmsg(IR_image, encoding="mono8")
        IR_publisher.publish(IR_msg)

        # Publish point cloud
        points = self.create_pointcloud(depth_frame, color_image, color_frame.profile.as_video_stream_profile().intrinsics, frame_id)
        pc_publisher.publish(points)


        self.get_logger().info(f'Published RGB, depth, IR and point cloud data for {frame_id}')


    def publish_accel_gyro(self, accel, gyro, accel_publisher, gyro_publisher, frame_id):
        accel_msg = Float32MultiArray()
        accel_msg.data = [accel[0], accel[1], accel[2]]
        accel_publisher.publish(accel_msg)

        gyro_msg = Float32MultiArray()
        gyro_msg.data = [gyro[0], gyro[1], gyro[2]]
        gyro_publisher.publish(gyro_msg)

        self.get_logger().info(f'{frame_id} Accelerometer: {accel_msg.data}, Gyroscope: {gyro_msg.data}')



    def create_pointcloud(self, depth_frame, color_image, color_intrin, frame_id):
        # Get intrinsics
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(depth_frame.profile)

        # PointCloud2 setup
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

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
