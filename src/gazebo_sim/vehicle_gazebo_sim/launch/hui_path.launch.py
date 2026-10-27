"""Launch a 3 m-wide square ring with 10 m centerline sides."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('vehicle_gazebo_sim')
    simulation_launch = os.path.join(
        package_share, 'launch', 'simulation.launch.py')
    world = os.path.join(package_share, 'worlds', 'hui_path.world')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-5.0'),
        DeclareLaunchArgument('z', default_value='0.15'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments={
                'world': world,
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
