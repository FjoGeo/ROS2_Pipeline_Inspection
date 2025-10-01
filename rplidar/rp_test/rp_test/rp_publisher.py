import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from rplidar import RPLidar
import time

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_quality1 = self.create_publisher(Int32, 'lidar/quality1', 10)
        self.publisher_angle1 = self.create_publisher(Float32, 'lidar/angle1', 10)
        self.publisher_distance1 = self.create_publisher(Float32, 'lidar/distance1', 10)
        self.publisher_quality2 = self.create_publisher(Int32, 'lidar/quality2', 10)
        self.publisher_angle2 = self.create_publisher(Float32, 'lidar/angle2', 10)
        self.publisher_distance2 = self.create_publisher(Float32, 'lidar/distance2', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_scan)
        
        try:
            self.lidar1 = RPLidar(port="/dev/ttyUSB0", baudrate=1000000, timeout=10)
            self.lidar1.connect()
            self.lidar2 = RPLidar(port="/dev/ttyUSB1", baudrate=1000000, timeout=10)
            self.lidar2.connect()
        except:
            self.lidar1 = RPLidar(port="/dev/ttyUSB1", baudrate=1000000, timeout=10)
            self.lidar1.connect()
            self.lidar2 = RPLidar(port="/dev/ttyUSB2", baudrate=1000000, timeout=10)
            self.lidar2.connect()

        time.sleep(2)
        self.scan_generator1 = self.lidar1.iter_measurments(max_buf_meas=500)
        self.scan_generator2 = self.lidar2.iter_measurments(max_buf_meas=500)

    def publish_scan(self):
        try:
            
            msg_quality1 = Int32()
            msg_quality2 = Int32()
            msg_angle1 = Float32()
            msg_angle2 = Float32()
            msg_distance1 = Float32()
            msg_distance2 = Float32()


            for new_scan1, quality1, angle1, distance1, new_scan2, quality2, angle2, distance2 in zip(self.scan_generator1, self.scan_generator2):

                msg_quality1.data = quality1
                self.publisher_quality1.publish(msg_quality1)
                
                msg_quality2.data = quality2
                self.publisher_quality2.publish(msg_quality2)

                msg_angle1.data = angle1
                self.publisher_angle1.publish(msg_angle1)
                
                msg_angle2.data = angle2
                self.publisher_angle2.publish(msg_angle2)

                msg_distance1.data = distance1
                self.publisher_distance1.publish(msg_distance1)

                msg_distance2.data = distance2
                self.publisher_distance2.publish(msg_distance2)

                output = f'LiDAR1: quality={msg_quality1.data}, angle={msg_angle1.data}, distance={msg_distance1.data}' \
                        f'\nLiDAR2: quality={msg_quality2.data}, angle={msg_angle2.data}, distance={msg_distance2.data}'
            
                self.get_logger().info(output)
        except StopIteration:
            self.get_logger().info('StopIteration: No more scans')
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def destroy_node(self):
        self.lidar1.set_motor_pwm(0)
        self.lidar1.stop()
        self.lidar1.disconnect()

        self.lidar2.set_motor_pwm(0)
        self.lidar2.stop()
        self.lidar2.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
