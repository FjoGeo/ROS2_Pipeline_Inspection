import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SerialSubscriber(Node):

    def __init__(self):
        super().__init__('serial_subscriber')
        self.subscription = self.create_subscription(
            String,
            'serial_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Process or print the received serial data
        # For example, printing the received data
        self.get_logger().info('Received serial data: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    serial_subscriber = SerialSubscriber()
    rclpy.spin(serial_subscriber)
    serial_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()