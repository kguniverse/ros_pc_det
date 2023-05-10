import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.qos_event import SubscriptionEventCallbacks
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import numpy as np
class DepthCamTalker(Node):

    def __init__(self):
        super().__init__('depth_cam_talker')
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        custom_callback = lambda event: print('Custom Incompatible QoS callback')
        callbacks = SubscriptionEventCallbacks()
        callbacks.incompatible_qos = custom_callback
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.listener_callback,
            qos,
            event_callbacks=callbacks)
        self.declare_parameter('time_period', 0.5)
        timer_period = self.get_parameter('time_period').get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pub = self.create_publisher(PointCloud2, '/point_cloud', 10)
        self.massage_recieved = False
        self.msg = None

    def listener_callback(self, msg):
        if len(msg.data) == 0 :
            return
        # change from camera to lidar
        points = pc2.read_points_numpy(msg)
        tmp_dim = points[:, 0].copy()
        points[:, 0] = points[:, 2]
        points[:, 2] = -points[:, 1]
        points[:, 1] = -tmp_dim
        msg = pc2.create_cloud_xyz32(msg.header, points)
        self.msg = msg
        self.massage_recieved = True
    
    def timer_callback(self):
        if self.massage_recieved:
            self.pub.publish(self.msg)
            self.massage_recieved = False

def main(args=None):
    rclpy.init(args=args)

    depth_cam_talker = DepthCamTalker()

    rclpy.spin(depth_cam_talker)

    depth_cam_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()