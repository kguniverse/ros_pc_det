from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'pc_det'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.[pxyr][ymav]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangqiankai',
    maintainer_email='kkwang@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker=pc_det.nuscenes_talker:main',
            'depthcam_talker=pc_det.depthcam_talker:main',
            'listener=pc_det.nuscenes_bin2pcd:main',
            'consumer_node=pc_det.consumer_node:main',
            'sync_node=pc_det.sync_node:main',
            'dummy_scan2pc=pc_det.scan2pc:main',
            'infer_node=pc_det.infer_node:main',
            'infer_node_ort=pc_det.infer_node_ort:main',
            'infer_node_point=pc_det.infer_node_point:main',
        ],
    },
)
