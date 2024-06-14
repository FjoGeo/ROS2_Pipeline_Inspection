import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pyrplidar import PyRPlidar
import time

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(String, 'lidar_scan', 10)
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
            msg = String()
            msg.data = f'{scan.quality}, {scan.angle}, {scan.distance}'
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
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
