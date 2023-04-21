import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

import numpy as np
from sensor_msgs.msg import PointField
import sensor_msgs_py.point_cloud2 as pc2
import os

count = 0



class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'bin_file_dir', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.filenames = self.getBinDir()
        self.count = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = self.filenames[self.count]
        self.count += 1
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

    def getBinDir(self):
        main_folder = 'data/nuscenes/samples/LIDAR_TOP'
        bin_files = os.listdir(main_folder)
        bin_files.sort()
        bin_files = [main_folder + '/' + x for x in bin_files]
        return bin_files

class MinimalSubscriberV2(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        self.declare_parameter('output_path')
        self.declare_parameter('intensity_range')

        global param_output_path
        param_output_path = self.get_parameter('output_path').value
        global param_intensity_range
        param_intensity_range = self.get_parameter('intensity_range').value

        self.subscription = self.create_subscription(
            PointCloud2,
            '/point_cloud',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        cloud_points = list(pc2.read_points(msg, skip_nans=True, field_names = ("x", "y", "z", "intensity")))
        
        pc_arr = np.array(cloud_points).flatten()
        # dividing by param_intensity_range to get intensity in desired range
        pc_arr[3::4] /= param_intensity_range
        global pc_num
        pc_num += 1
        print(pc_num)
        output_file = param_output_path + str(pc_num) + '.bin'
        pc_arr.astype('float32').tofile(output_file)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()