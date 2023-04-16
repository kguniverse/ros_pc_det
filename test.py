import os
from mmdet3d.apis import init_model, inference_detector
import mmcv


mmdet3d_path = '/home/wangqiankai/openmmlab/mmdetection3d'
config_file = os.path.join(mmdet3d_path, 'configs/pointpillars/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d.py')
checkpoint_file = 'checkpoint/hv_pointpillars_fpn_sbn-all_4x8_2x_nus-3d_20210826_104936-fca299c1.pth'
model = init_model(config_file, checkpoint_file, device='cuda:0')
points_mmdet3d = 'data/nuscenes/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin'
result, data = inference_detector(model, points_mmdet3d)

boxes=result[0]['pts_bbox']['boxes_3d']
label=result[0]['pts_bbox']['labels_3d'].numpy()
score = result[0]['pts_bbox']['scores_3d'].numpy()

breakpoint()
mmcv.dump((boxes, label, score), f'results/pc_det_result.pkl')