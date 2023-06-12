from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import yaml

import os

dirname = os.path.dirname(__file__)
rviz_config = 'default.rviz'
rviz_config_dir = os.path.join(get_package_share_directory('pc_det'), 'rviz', rviz_config)

config_file = 'params.yaml'
config_file_dir = os.path.join(get_package_share_directory('pc_det'), 'config', config_file)

with open(rviz_config_dir, 'r') as f:
    rviz_config = yaml.load(f, Loader=yaml.FullLoader)

def generate_launch_description():
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('astra_camera'), '/launch/astra_mini.launch.py'])
        ),
        Node(
            package='pc_det',
            executable='depthcam_talker',
            name='depthcam_pub',
            parameters=[
                config_file_dir
            ]
        ),
        Node(
            package='pc_det',
            executable='consumer_node',
            name='consumer_node',
            parameters=[]
        ),
        Node(
            package='pc_det',
            executable='infer_node_point',
            name='infer_node',
            parameters=[
                # {'mmdet3d_path', config.get('mmdet3d_path', '')},
                # {'config_file', config.get('config_file', '')},
                # {'checkpoint_file', config.get('checkpoint_file', '')},
                config_file_dir
            ]
        ),
        Node(
            package='pc_det',
            executable='sync_node',
            name='sync_node',
            parameters=[]
        ),
        rviz,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    ])