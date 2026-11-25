"""A2 §4: the estimator (lrs_localization) and the controller flying on its estimate.

  ros2 launch lrs_mission estimated.launch.py                    # both
  ros2 launch lrs_mission estimated.launch.py estimator:=false   # controller only
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lrs_mission')
    loc = get_package_share_directory('lrs_localization')
    return LaunchDescription([
        DeclareLaunchArgument('estimator', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc, 'launch', 'localization.launch.py')),
            condition=IfCondition(LaunchConfiguration('estimator'))),
        Node(
            package='lrs_mission', executable='estimated_flight_node', name='lrs_estimated_flight',
            output='screen', parameters=[os.path.join(share, 'config', 'estimated.yaml')]),
    ])
