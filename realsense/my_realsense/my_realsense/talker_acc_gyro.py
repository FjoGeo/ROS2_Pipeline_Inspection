#!/usr/bin/env python3



import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import pyrealsense2 as rs

import numpy as np



class RealSenseIMUPublisher(Node):

    def __init__(self):

        super().__init__('realsense_imu_publisher')

        

        self.declare_parameters(

            namespace='',

            parameters=[

                ('accel_topic1', 'realsense1/accel'),

                ('gyro_topic1', 'realsense1/gyro'),

                ('accel_topic2', 'realsense2/accel'),

                ('gyro_topic2', 'realsense2/gyro'),

                ('publish_rate', 200.0) # 10

            ]

        )



        self.accel_topic1 = self.get_parameter('accel_topic1').get_parameter_value().string_value

        self.gyro_topic1 = self.get_parameter('gyro_topic1').get_parameter_value().string_value

        self.accel_topic2 = self.get_parameter('accel_topic2').get_parameter_value().string_value

        self.gyro_topic2 = self.get_parameter('gyro_topic2').get_parameter_value().string_value

        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value



        self.accel_publisher1 = self.create_publisher(Float32MultiArray, self.accel_topic1, 10)

        self.gyro_publisher1 = self.create_publisher(Float32MultiArray, self.gyro_topic1, 10)

        self.accel_publisher2 = self.create_publisher(Float32MultiArray, self.accel_topic2, 10)

        self.gyro_publisher2 = self.create_publisher(Float32MultiArray, self.gyro_topic2, 10)



        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)



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

        config.enable_stream(rs.stream.accel)

        config.enable_stream(rs.stream.gyro)

        config.enable_device(serial_number)

        return pipeline, config



    @staticmethod

    def extract_motion_data(frame):

        motion_data = frame.as_motion_frame().get_motion_data()

        return np.array([motion_data.x, motion_data.y, motion_data.z])



    def timer_callback(self):

        self.publish_camera_data(self.pipeline1, self.accel_publisher1, self.gyro_publisher1, "camera1")

        self.publish_camera_data(self.pipeline2, self.accel_publisher2, self.gyro_publisher2, "camera2")



    def publish_camera_data(self, pipeline, accel_publisher, gyro_publisher, frame_id):

        try:

            frames = pipeline.wait_for_frames(timeout_ms=500)



            accel_frame = frames.first_or_default(rs.stream.accel)

            gyro_frame = frames.first_or_default(rs.stream.gyro)



            if accel_frame and gyro_frame:

                accel_data = self.extract_motion_data(accel_frame)

                gyro_data = self.extract_motion_data(gyro_frame)



                self.publish_accel_gyro(accel_data, gyro_data, accel_publisher, gyro_publisher, frame_id)

            else:

                self.get_logger().warn(f'No IMU frames received for {frame_id}')

        except Exception as e:

            self.get_logger().error(f'Error processing frames for {frame_id}: {e}')



    def publish_accel_gyro(self, accel, gyro, accel_publisher, gyro_publisher, frame_id):

        try:

            accel_msg = Float32MultiArray(data=accel.tolist())

            gyro_msg = Float32MultiArray(data=gyro.tolist())



            accel_publisher.publish(accel_msg)

            gyro_publisher.publish(gyro_msg)



            self.get_logger().info(f'{frame_id} Accelerometer: {accel.tolist()}, Gyroscope: {gyro.tolist()}')

        except Exception as e:

            self.get_logger().error(f'Error publishing data for {frame_id}: {e}')



    def destroy(self):

        self.pipeline1.stop()

        self.pipeline2.stop()

        super().destroy()



def main(args=None):

    rclpy.init(args=args)

    node = RealSenseIMUPublisher()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy()

        rclpy.shutdown()



if __name__ == '__main__':

    main()
