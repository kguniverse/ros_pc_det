import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String
import mmcv
from mmdet3d.apis import inference_detector, init_model
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

lines = [[0,2],[0,1],[0,4],[1,3],[1,5],[2,3],[2,6],[3,7],[4,5],[4,6],[5,7],[6,7]]

def run_infer():
    mmdet3d_path = '/home/wangqiankai/openmmlab/mmdetection3d'
    config_file = os.path.join(mmdet3d_path, 'configs/pointpillars/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d.py')
    checkpoint_file = 'checkpoint/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d_20210826_104936-fca299c1.pth'
    model = init_model(config_file, checkpoint_file, device='cuda:0')

class InferNode(Node):
    def __init__(self):
        super().__init__('infer_node')
        self.subscription = self.create_subscription(
            String,
            '/bin_file_dir',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        mmdet3d_path = '/home/wangqiankai/openmmlab/mmdetection3d'
        config_file = os.path.join(mmdet3d_path, 'configs/pointpillars/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d.py')
        checkpoint_file = 'checkpoint/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d_20210826_104936-fca299c1.pth'
        self.model = init_model(config_file, checkpoint_file, device='cuda:0')
        self.count = 0
        self.marker_array = MarkerArray()
        self.marker_pub = self.create_publisher(MarkerArray, '/detect_bbox3d', 10)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.header.frame_id)
        points_mmdet3d=msg.data
        result, data = inference_detector(self.model, points_mmdet3d)
        corners = result[0]['pts_bbox']['boxes_3d'].corners.numpy()
        label = result[0]['pts_bbox']['labels_3d'].numpy()
        score = result[0]['pts_bbox']['scores_3d'].numpy()
        self.draw_bbox(corners, label, score)
        # mmcv.dump((boxes, label, score), f'results/pc_det_result_{self.count}.pkl')
        self.count += 1
    
    def draw_bbox(self, corners_bboxes, labels, scores, score_thrs=0.3):
        # draw bbox on rviz2
        self.marker_array.markers.clear()
        
        for corners, label, score in zip(corners_bboxes, labels, scores):
            if score < score_thrs:
                continue
            marker = Marker()
            marker.header.frame_id = 'velodyne_lidar'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'bbox3d'
            marker.id = self.count

            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.points = []

            for line in lines:
                marker.points.append(Point(x=corners[line[0], 0].item(), y=corners[line[0], 1].item(), z=corners[line[0], 2].item()))
                marker.points.append(Point(x=corners[line[1], 0].item(), y=corners[line[1], 1].item(), z=corners[line[1], 2].item()))
            self.marker_array.markers.append(marker)
        self.marker_pub.publish(self.marker_array)
        


def main(args=None):
    rclpy.init(args=args)
    infer_node = InferNode()
    rclpy.spin(infer_node)
    infer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()