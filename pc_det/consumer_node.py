from rclpy.node import Node
import rclpy
from vision_msgs.msg import Detection3DArray
from sensor_msgs.msg import PointCloud2

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('synchronizer')
        self.pc_sub = self.create_subscription(PointCloud2, '/pub_pc2', self.listener_callback, 10)
        self.bbox_sub = self.create_subscription(Detection3DArray, '/pub_bbox3d', self.listener_callback, 10)

    def listener_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()