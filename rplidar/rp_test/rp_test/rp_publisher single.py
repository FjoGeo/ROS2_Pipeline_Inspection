import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from pyrplidar import PyRPlidar
import time

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_quality = self.create_publisher(Int32, 'lidar_quality', 10)
        self.publisher_angle = self.create_publisher(Float32, 'lidar_angle', 10)
        self.publisher_distance = self.create_publisher(Float32, 'lidar_distance', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_scan)
        self.lidar = PyRPlidar()
        self.lidar.connect(port="/dev/ttyUSB0", baudrate=1000000, timeout=10)
        self.lidar.set_motor_pwm(700)
        time.sleep(2)
        self.scan_generator = self.lidar.start_scan_express(0)()

    def publish_scan(self):
        try:
            scan = next(self.scan_generator)

            msg_quality = Int32()
            msg_quality.data = scan.quality
            self.publisher_quality.publish(msg_quality)

            msg_angle = Float32()
            msg_angle.data = scan.angle
            self.publisher_angle.publish(msg_angle)

            msg_distance = Float32()
            msg_distance.data = scan.distance
            self.publisher_distance.publish(msg_distance)

            self.get_logger().info(f'Publishing: quality={msg_quality.data}, angle={msg_angle.data}, distance={msg_distance.data}')
        except StopIteration:
            self.get_logger().info('StopIteration: No more scans')
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def destroy_node(self):
        self.lidar.set_motor_pwm(0)
        self.lidar.stop()
        self.lidar.disconnect()
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
