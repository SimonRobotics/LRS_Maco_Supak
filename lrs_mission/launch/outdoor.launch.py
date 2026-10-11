import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lrs_mission')
    return LaunchDescription([
        Node(
            package='lrs_mission', executable='lrs_mission_node', name='lrs_mission', output='screen',
            parameters=[os.path.join(share, 'config', 'outdoor.yaml')],
        ),
    ])
