from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Quaternion, Vector3
import tf_transformations

def draw_bbox(bboxes, labels, scores, score_thrs, timestamp):
    # draw bbox on rviz2
    det3d_array = Detection3DArray()
    det3d_array.header.frame_id = 'velodyne_lidar'
    # TODO: timestamp make sure is not none
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
    
    return det3d_array