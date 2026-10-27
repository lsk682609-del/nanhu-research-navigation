"""Road-network navigation for the 30 x 30 m square bridge course."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('robot_launch')
    navigation_launch = os.path.join(share, 'launch', 'gazebo_net_nav.launch.py')
    topology = os.path.join(share, 'maps', 'topo_map_square_bridges_30m.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car1'),
        DeclareLaunchArgument('topo_map_path', default_value=topology),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'gazebo_net_nav.yaml')),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            share, 'rviz', 'navigation.rviz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'params_file': LaunchConfiguration('params_file'),
                'topo_map_path': LaunchConfiguration('topo_map_path'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items(),
        ),
    ])
