import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lrs_localization')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'localization.yaml')),
        Node(
            package='lrs_localization', executable='localization_node', name='lrs_localization', output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
