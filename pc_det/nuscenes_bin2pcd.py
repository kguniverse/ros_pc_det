import rclpy
import mmcv
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

from nuscenes.utils.data_classes import LidarPointCloud

class TransNode(Node):
    def __init__(self):
        super().__init__('trans_node')
        self.subscription = self.create_subscription(
            String,
            'bin_file_dir',
            self.listener_callback,
            1)
        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)
    
    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        pcd_bin_file = msg.data
        pc = LidarPointCloud.from_file(pcd_bin_file)
        bin_pcd = pc.points.T
        point_data = bin_pcd.reshape((-1, 4))
        # point_data = np.fromfile(bin_file, dtype=np.float32, count=-1).reshape([-1, 4])
        header = Header()
        header.frame_id = msg.data
        header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        pcl2_msg = pc2.create_cloud(header, fields, point_data)
        self.publisher.publish(pcl2_msg)
        # self.get_logger().info('I published: timestamp:"%s"' % header.stamp)

def main(args=None):
    rclpy.init(args=args)
    transnode = TransNode()
    rclpy.spin(transnode)
    transnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()