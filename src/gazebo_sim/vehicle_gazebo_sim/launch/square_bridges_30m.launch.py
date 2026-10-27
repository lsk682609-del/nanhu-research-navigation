"""Launch the 30 x 30 m square bridge-and-obstacle Gazebo course."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('vehicle_gazebo_sim')
    simulation_launch = os.path.join(share, 'launch', 'simulation.launch.py')
    world = os.path.join(share, 'worlds', 'square_bridges_30m.world')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('x', default_value='-10.0'),
        DeclareLaunchArgument('y', default_value='-10.0'),
        DeclareLaunchArgument('z', default_value='0.15'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
            }.items(),
        ),
    ])
