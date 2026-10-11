import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lrs_mission')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', os.path.join(share, 'rviz', 'lrs_mission.rviz')],
        ),
        DeclareLaunchArgument('mission_file', default_value=os.path.join(share, 'missions', 'exam_example.csv')),
        DeclareLaunchArgument('map_file', default_value=os.path.join(share, 'maps', 'FEI_LRS_PCD', 'map.pcd')),
        DeclareLaunchArgument('plan_only', default_value='false'),
        Node(
            package='lrs_mission', executable='lrs_mission_node', name='lrs_mission', output='screen',
            parameters=[
                os.path.join(share, 'config', 'indoor.yaml'),
                {
                    'mission_file': LaunchConfiguration('mission_file'),
                    'map_file': LaunchConfiguration('map_file'),
                    'plan_only': LaunchConfiguration('plan_only'),
                },
            ],
        ),
    ])
