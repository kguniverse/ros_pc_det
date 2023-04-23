import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, LaserScan
import mmcv
from mmdet3d.apis import inference_detector, init_model
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import tf_transformations

from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose



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
        self.declare_parameter('checkpoint_file', '')
        self.declare_parameter('score_threshold', 0.3)

        mmdet3d_path = self.get_parameter('mmdet3d_path').get_parameter_value().string_value
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        full_config_file = os.path.join(mmdet3d_path, config_file)
        checkpoint_file = self.get_parameter('checkpoint_file').get_parameter_value().string_value
        self.get_logger().info('full_config_file: "%s"' % full_config_file)
        self.get_logger().info('checkpoint_file: "%s"' % checkpoint_file)
        self.model = init_model(full_config_file, checkpoint_file, device='cuda:0')
        self.count = 0
        self.marker_pub = self.create_publisher(Detection3DArray, '/detect_bbox3d', 10)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.header.frame_id)
        points_mmdet3d=msg.header.frame_id # load the bin file name
        result, data = inference_detector(self.model, points_mmdet3d)
        bboxes= result[0]['pts_bbox']['boxes_3d']
        label = result[0]['pts_bbox']['labels_3d'].numpy()
        score = result[0]['pts_bbox']['scores_3d'].numpy()
        self.draw_bbox(bboxes, label, score, timestamp=msg.header.stamp)
        # mmcv.dump((boxes, label, score), f'results/pc_det_result_{self.count}.pkl')
        self.count += 1
    
    def draw_bbox(self, bboxes, labels, scores, timestamp=None):
        # draw bbox on rviz2

        score_thrs = self.get_parameter('score_threshold').get_parameter_value().double_value
        det3d_array = Detection3DArray()
        det3d_array.header.frame_id = 'velodyne_lidar'
        if timestamp is None:
            det3d_array.header.stamp = self.get_clock().now().to_msg()
        det3d_array.header.stamp = timestamp
        
        for ind in range(len(bboxes)):
            bbox = bboxes[ind]
            label = labels[ind]
            score = scores[ind]

            if score < score_thrs:
                continue
            
            det3d = Detection3D()
            det3d.header.frame_id = 'velodyne_lidar'
            det3d.header.stamp = timestamp

            pose = Pose()
            pose.position.x = bbox.center[0][0].item()
            pose.position.y = bbox.center[0][1].item()
            pose.position.z = bbox.center[0][2].item()

            quat = Quaternion()
            q = tf_transformations.quaternion_from_euler(0, 0, bbox.yaw)
            quat.x = q[0]
            quat.y = q[1]
            quat.z = q[2]
            quat.w = q[3]
            pose.orientation = quat

            dimensions = Vector3()
            dimensions.x = bbox.dims[0][0].item()
            dimensions.y = bbox.dims[0][1].item()
            dimensions.z = bbox.dims[0][2].item()

            det3d.bbox.center = pose
            det3d.bbox.size = dimensions

            object_hypothesis = ObjectHypothesisWithPose()
            object_hypothesis.hypothesis.class_id = str(label)
            object_hypothesis.hypothesis.score = score.item()
            det3d.results.append(object_hypothesis)
            
            det3d_array.detections.append(det3d)
        
        self.marker_pub.publish(det3d_array)
        
        


def main(args=None):
    rclpy.init(args=args)
    infer_node = InferNode()
    rclpy.spin(infer_node)
    infer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()