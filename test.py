import os
from mmdet3d.apis import init_model, inference_detector
import mmcv

from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Quaternion, Vector3

import tf_transformations

def draw_bbox(bboxes, labels, scores, score_thrs=0.6, timestamp=None):
    # draw bbox on rviz2
    
    for ind in range(len(bboxes)):
        bbox = bboxes[ind]
        label = labels[ind]
        score = scores[ind]
        if score < score_thrs:
            continue

        # breakpoint()
        pose = Pose()
        pose.position.x = bbox.center[0][0].item()
        pose.position.y = bbox.center[0][1].item()
        pose.position.z = bbox.center[0][2].item()

        breakpoint()
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


        object_hypothesis = ObjectHypothesisWithPose()
        object_hypothesis.hypothesis.class_id = str(label)
        breakpoint()
        object_hypothesis.hypothesis.score = score.item()
        det3d.results.append(object_hypothesis)

mmdet3d_path = '/home/wangqiankai/openmmlab/mmdetection3d'
config_file = os.path.join(mmdet3d_path, 'configs/pointpillars/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d.py')
checkpoint_file = 'checkpoint/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d_20210826_104936-fca299c1.pth'
model = init_model(config_file, checkpoint_file, device='cuda:0')
points_mmdet3d = 'data/nuscenes/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin'
result, data = inference_detector(model, points_mmdet3d)

boxes=result[0]['pts_bbox']['boxes_3d']
label=result[0]['pts_bbox']['labels_3d'].numpy()
score = result[0]['pts_bbox']['scores_3d'].numpy()

draw_bbox(boxes, label, score)

breakpoint()
mmcv.dump((boxes, label, score), f'results/pc_det_result.pkl')