from launch import LaunchDescription
from launch_ros.actions import Node

import os

rviz_config_dir = os.path.join('rviz', 'default.rviz')
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pc_det',
            executable='talker',
            name='talker',
            parameters=[]
        ),
        Node(
            package='pc_det',
            executable='listener',
            name='listener',
            parameters=[]
        ),
        Node(
            package='pc_det',
            executable='pcl_client',
            name='pcl_client',
            parameters=[]
        ),
        Node(
            package='pc_det',
            executable='infer_node',
            name='infer_node',
            parameters=[]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen',
        )
    ])