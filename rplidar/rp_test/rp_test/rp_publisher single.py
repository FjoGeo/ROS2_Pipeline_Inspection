import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from rplidar import RPLidar
import time

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_quality = self.create_publisher(Int32, 'lidar_quality', 10)
        self.publisher_angle = self.create_publisher(Float32, 'lidar_angle', 10)
        self.publisher_distance = self.create_publisher(Float32, 'lidar_distance', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_scan)
        
        try:
            self.lidar = RPLidar(port="/dev/ttyUSB0", baudrate=1000000, timeout=10)
            self.lidar.connect()
        except:
            self.lidar = RPLidar(port="/dev/ttyUSB1", baudrate=1000000, timeout=10)
            self.lidar.connect()

        time.sleep(2)
        self.scan_generator = self.lidar.iter_measurments(max_buf_meas=500)

    def publish_scan(self):
        try:

            msg_quality = Int32()
            msg_angle = Float32()
            msg_distance = Float32()

            for new_scan, quality, angle, distance in self.scan_generator:
                msg_quality.data = quality
                msg_angle.data = angle
                msg_distance.data = distance

                self.publisher_quality.publish(msg_quality)
                self.publisher_angle.publish(msg_angle)
                self.publisher_distance.publish(msg_distance)

                self.get_logger().info(f'Publishing: quality={msg_quality.data}, angle={msg_angle.data}, distance={msg_distance.data}')



        except StopIteration:
            self.get_logger().info('StopIteration: No more scans')
            self.cleanup_lidar()
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            self.cleanup_lidar()

    def destroy_node(self):
        self.cleanup_lidar()
        super().destroy_node()

    def cleanup_lidar(self):
        try:
            self.lidar.stop()
            self.lidar.disconnect()
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")

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
