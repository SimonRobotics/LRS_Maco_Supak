import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lrs_mission')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file', default_value=os.path.join(share, 'maps', 'FEI_LRS_PCD', 'map.pcd')),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'voxel_map.yaml')),
        DeclareLaunchArgument('inflation_radius', default_value='0.5'),
        Node(
            package='lrs_mission',
            executable='voxel_map_node',
            name='voxel_map',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'map_file': LaunchConfiguration('map_file'),
                    'inflation_radius': LaunchConfiguration('inflation_radius'),
                },
            ],
        ),
    ])
