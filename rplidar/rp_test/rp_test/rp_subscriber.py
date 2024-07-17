import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription_quality1 = self.create_subscription(
            Int32,
            'lidar/quality1',
            self.listener_callback_quality,
            10)
        self.subscription_quality1  # prevent unused variable warning

        self.subscription_angle1 = self.create_subscription(
            Float32,
            'lidar/angle1',
            self.listener_callback_angle,
            10)
        self.subscription_angle1  # prevent unused variable warning

        self.subscription_distance1 = self.create_subscription(
            Float32,
            'lidar/distance1',
            self.listener_callback_distance,
            10)
        self.subscription_distance1  # prevent unused variable warning



        self.subscription_quality2 = self.create_subscription(
            Int32,
            'lidar/quality2',
            self.listener_callback_quality,
            10)
        self.subscription_quality2  # prevent unused variable warning

        self.subscription_angle2 = self.create_subscription(
            Float32,
            'lidar/angle2',
            self.listener_callback_angle,
            10)
        self.subscription_angle2  # prevent unused variable warning

        self.subscription_distance2 = self.create_subscription(
            Float32,
            'lidar/distance2',
            self.listener_callback_distance,
            10)
        self.subscription_distance2  # prevent unused variable warning

    def listener_callback_quality(self, msg):
        self.get_logger().info(f'Received quality: {msg.data}')

    def listener_callback_angle(self, msg):
        self.get_logger().info(f'Received angle: {msg.data}')

    def listener_callback_distance(self, msg):
        self.get_logger().info(f'Received distance: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
