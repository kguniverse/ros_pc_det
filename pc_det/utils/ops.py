import os
import mmcv
import torch
import numpy as np
from typing import Union, Dict
import torch.nn.functional as F
import onnxruntime as ort
from mmdet3d.core import bbox3d2result 
from mmdet3d.models.builder import build_head 

class PointPillarsOrt:

    def __init__(
            self,
            onnx_model: str,
            mmdet3d_path: str,
            config_file: str
            ):
        self.full_config_file = os.path.join(mmdet3d_path, config_file)
        providers = ['CUDAExecutionProvider']
        self.ort_session = ort.InferenceSession(onnx_model, providers=providers)

        model_cfg = mmcv.Config.fromfile(self.full_config_file)
        head_cfg = dict(**model_cfg.model['pts_bbox_head']) 
        head_cfg['train_cfg'] = None 
        head_cfg['test_cfg'] = model_cfg.model['test_cfg']['pts']
        self.head = build_head(head_cfg) 
        self.model_cfg = model_cfg
        

    def voxelize(self, model_cfg: Union[str, mmcv.Config], points: torch.Tensor): 
        from mmcv.ops import Voxelization 
        model_cfg = mmcv.Config.fromfile(model_cfg)
        voxel_layer = model_cfg.model['pts_voxel_layer'] 
        voxel_layer = Voxelization(**voxel_layer) 
        voxels, coors, num_points = [], [], [] 
        for res in points: 
            res_voxels, res_coors, res_num_points = voxel_layer(res) 
            voxels.append(res_voxels) 
            coors.append(res_coors) 
            num_points.append(res_num_points) 
        voxels = torch.cat(voxels, dim=0) 
        num_points = torch.cat(num_points, dim=0) 
        coors_batch = [] 
        for i, coor in enumerate(coors): 
            coor_pad = F.pad(coor, (1, 0), mode='constant', value=i) 
            coors_batch.append(coor_pad) 
        coors_batch = torch.cat(coors_batch, dim=0) 
        return voxels, num_points, coors_batch 

    def generate_metas(self, cfg):
        from mmdet3d.core.bbox import get_box_type
        box_type_3d, box_mode_3d = get_box_type(cfg.data.test.box_type_3d)
        img_metas = dict(
                pts_filename='',
                box_type_3d=box_type_3d,
                box_mode_3d=box_mode_3d,
                # for ScanNet demo we need axis_align_matrix
                ann_info=dict(axis_align_matrix=np.eye(4)),
                sweeps=[],
                # set timestamp = 0
                timestamp=[0],
                img_fields=[],
                bbox3d_fields=[],
                pts_mask_fields=[],
                pts_seg_fields=[],
                bbox_fields=[],
                mask_fields=[],
                seg_fields=[])
        
        return [img_metas]

    def post_process(self,
                    outs: torch.Tensor, 
                    device: str, 
                    rescale=False): 

        cls_scores = [outs['scores'].to(device)] 
        bbox_preds = [outs['bbox_preds'].to(device)]
        dir_scores = [outs['dir_scores'].to(device)]
        img_metas = self.generate_metas(self.model_cfg)  
        bbox_list = self.head.get_bboxes( 
            cls_scores, bbox_preds, dir_scores, img_metas, rescale=False) 
        bbox_results = [ 
            bbox3d2result(bboxes, scores, labels) 
            for bboxes, scores, labels in bbox_list 
        ] 
        return bbox_results



    def run_onnx(
            self,
            points: torch.Tensor,
            ):
        assert points.dim() == 3 and points.shape[0] == 1
        voxels, num_points, coors = self.voxelize(self.full_config_file, points)
        voxels = voxels.numpy()
        num_points = num_points.numpy()
        coors = coors.numpy()
        voxels_ort = ort.OrtValue.ortvalue_from_numpy(voxels, 'cuda', 0)
        num_points_ort = ort.OrtValue.ortvalue_from_numpy(num_points, 'cuda', 0)
        coors_ort = ort.OrtValue.ortvalue_from_numpy(coors, 'cuda', 0)
        io_binding = self.ort_session.io_binding()
        io_binding.bind_input(
            name=self.ort_session.get_inputs()[0].name,
            device_type='cuda',
            device_id=0,
            element_type=np.float32,
            shape=voxels_ort.shape(),
            buffer_ptr=voxels_ort.data_ptr()
        )
        io_binding.bind_input(
            name=self.ort_session.get_inputs()[1].name,
            device_type='cuda',
            device_id=0,
            element_type=np.int32,
            shape=num_points_ort.shape(),
            buffer_ptr=num_points_ort.data_ptr()
        )
        io_binding.bind_input(
            name=self.ort_session.get_inputs()[2].name,
            device_type='cuda',
            device_id=0,
            element_type=np.int32,
            shape=coors_ort.shape(),
            buffer_ptr=coors_ort.data_ptr()
        )
        io_binding.bind_output('cls_score')
        io_binding.bind_output('bbox_pred')
        io_binding.bind_output('dir_cls_pred')
        self.ort_session.run_with_iobinding(io_binding)
        cls_score = torch.from_numpy(io_binding.copy_outputs_to_cpu()[0])
        bbox_pred = torch.from_numpy(io_binding.copy_outputs_to_cpu()[1])
        dir_cls_pred = torch.from_numpy(io_binding.copy_outputs_to_cpu()[2])
        # ort_inputs = {
        #     self.ort_session.get_inputs()[0].name: voxels,
        #     self.ort_session.get_inputs()[1].name: num_points, 
        #     self.ort_session.get_inputs()[2].name: coors
        #     }
        # ort_outs = self.ort_session.run(None, ort_inputs)
        # cls_score = torch.from_numpy(ort_outs[0])
        # bbox_pred = torch.from_numpy(ort_outs[1])
        # dir_cls_pred = torch.from_numpy(ort_outs[2])
        outs = dict(
            scores=cls_score,
            bbox_preds=bbox_pred,
            dir_scores=dir_cls_pred
        )
        bbox_results = self.post_process(outs, 'cuda:0')
        return bbox_results