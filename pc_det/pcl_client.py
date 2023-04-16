import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('consumer')
        self.subscription1 = self.create_subscription(
            PointCloud2,
            '/point_cloud',
            self.listener_callback,
            10)
        self.subscription2 = self.create_subscription(
            MarkerArray,
            '/detect_bbox3d',
            self.listener_callback,
            10)
        self.subscription1  # prevent unused variable warning
        self.subscription2  # prevent unused variable warning
    def listener_callback(self, msg):
        pass
        # self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()