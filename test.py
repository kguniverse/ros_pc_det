from nuscenes.utils.data_classes import LidarPointCloud
from std_msgs.msg import Header
from sensor_msgs.msg import PointField, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import torch

pcd_bin_file = 'data/nuscenes/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin'
pc = LidarPointCloud.from_file(pcd_bin_file)
bin_pcd = pc.points.T
point_data = bin_pcd.reshape((-1, 4))
# point_data = np.fromfile(bin_file, dtype=np.float32, count=-1).reshape([-1, 4])
header = Header()
header.frame_id = '1'
fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1)
]
msg = pc2.create_cloud(header, fields, point_data)

points = pc2.read_points_numpy(msg)
breakpoint()
points_tensor = torch.tensor(points, dtype=torch.float32).unsqueeze(0)