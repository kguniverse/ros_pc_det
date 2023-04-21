import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray

from message_filters import Subscriber, TimeSynchronizer

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('synchronizer')
        self.pc_sub = Subscriber(self, PointCloud2, '/point_cloud')
        self.bbox_sub = Subscriber(self, Detection3DArray, '/detect_bbox3d')
        self.ts = TimeSynchronizer([self.pc_sub, self.bbox_sub], 10)
        self.ts.registerCallback(self.listener_callback)
        self.pc_pub = self.create_publisher(PointCloud2, '/pub_pc2', 10)
        self.bbox_pub = self.create_publisher(Detection3DArray, '/pub_bbox3d', 10)

    def listener_callback(self, pc2, bbox3d):
        pc2.header.frame_id = 'pub'
        bbox3d.header.frame_id = 'pub'
        self.pc_pub.publish(pc2)
        self.bbox_pub.publish(bbox3d)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()