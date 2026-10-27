"""Launch only Gazebo, the vehicle model, and its sensor/control plugins."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('vehicle_gazebo_sim')
    gazebo_share = get_package_share_directory('gazebo_ros')
    default_world = os.path.join(package_share, 'worlds', 'test_world.world')
    urdf = os.path.join(package_share, 'urdf', 'simple_car.urdf.xacro')
    rviz_config = os.path.join(package_share, 'rviz', 'sensors.rviz')

    robot_description = xacro.process_file(urdf).toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Absolute path to the Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo client GUI'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Start RViz with sensor displays'),
        # Default spawn = topology node 17 (9, 8), the centre of the road
        # network; spawning on a node keeps every goal within max_snap_distance.
        DeclareLaunchArgument('x', default_value='9.0'),
        DeclareLaunchArgument('y', default_value='8.0'),
        DeclareLaunchArgument('z', default_value='0.15'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world, 'gui': gui}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='vehicle_spawner',
            output='screen',
            arguments=[
                '-entity', 'simple_car',
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
