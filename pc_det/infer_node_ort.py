import os
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray
import sensor_msgs_py.point_cloud2 as pc2

from .utils import PointPillarsOrt, draw_bbox

class InferNode(Node):
    def __init__(self):
        super().__init__('infer_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/point_cloud',
            self.listener_callback,
            10)
        
        self.declare_parameter('mmdet3d_path', '')
        self.declare_parameter('config_file', '')
        self.declare_parameter('score_threshold', 0.3)
        self.declare_parameter('onnx_model', '')

        mmdet3d_path = self.get_parameter('mmdet3d_path').get_parameter_value().string_value
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        full_config_file = os.path.join(mmdet3d_path, config_file)        
        onnx_model = self.get_parameter('onnx_model').get_parameter_value().string_value
        self.get_logger().info('full_config_file: "%s"' % full_config_file)
        self.score_thr = self.get_parameter('score_threshold').get_parameter_value().double_value
        self.count = 0
        self.model = PointPillarsOrt(onnx_model, mmdet3d_path, config_file)
        self.marker_pub = self.create_publisher(Detection3DArray, '/detect_bbox3d', 10)
    
    def listener_callback(self, msg):
        points = pc2.read_points_numpy(msg)
        points_tensor = torch.tensor(points, dtype=torch.float32).unsqueeze(0)
        result = self.model.run_onnx(points_tensor)
        bboxes= result[0]['boxes_3d']
        label = result[0]['labels_3d'].numpy()
        score = result[0]['scores_3d'].numpy()

        timestamp = msg.header.stamp
        det3d_array = draw_bbox(bboxes, label, score, self.score_thr, timestamp)
        self.marker_pub.publish(det3d_array)

def main(args=None):
    rclpy.init(args=args)

    infer_node = InferNode()

    rclpy.spin(infer_node)

    infer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
