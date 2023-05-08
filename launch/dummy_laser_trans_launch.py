from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'view_sllidar_launch.py'
            ])
        ])
    ),
    trans_node = Node(
        package='pc_det',
        executable='dummy_scan2pc',
        name='scan2pc',
        output='screen',
    )
    return LaunchDescription([
        included_launch,
        trans_node
    ])