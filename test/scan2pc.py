import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, LaserScan, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

def deg2rad(deg):
    return deg * np.pi / 180

def rad2deg(rad):
    return rad * 180 / np.pi

class Scan2Pointcloud(Node):
    def __init__(self):
        super().__init__('scan2pointcloud')
        self.subscription = self.create_subscription(
            LaserScan,
            'laser',
            self.listener_callback,
            1)
        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)
    
    def listener_callback(self, msg: LaserScan):

        header = Header()
        header.frame_id = msg.header.frame_id
        header.stamp = msg.header.stamp
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        points = []
        height = 16
        count = msg.scan_time / msg.time_increment
        for j in range(height):
            for i in range(j * count, (j + 1) * count):
                degree = rad2deg(msg.angle_min + i * msg.angle_increment * (i - count * j))
                k = j
                if msg.ranges[i - count * j] != 0:
                    x = msg.ranges[i - count * j] * np.cos(deg2rad(degree))
                    y = msg.ranges[i - count * j] * np.sin(deg2rad(degree))
                    z = k/10 - 0.8
                    points.append([x, y, z, 255])
        pcl2_msg = pc2.create_cloud(header, fields, points)
        self.publisher.publish(pcl2_msg)

def main(args=None):
    rclpy.init(args=args)
    scan2pointcloud = Scan2Pointcloud()
    rclpy.spin(scan2pointcloud)
    scan2pointcloud.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()