"""Gazebo navigation baseline: feed cur_lid directly to ObstacleLayer.

The active terrain-analysis YAML is used as the source of every other parameter.
Run this launch instead of gazebo_net_nav.launch.py, not alongside it.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _start_baseline(context):
    share = get_package_share_directory('robot_launch')
    source_path = LaunchConfiguration('params_file').perform(context)
    with open(source_path, 'r', encoding='utf-8') as source:
        params = yaml.safe_load(source)

    costmap = params['/**/local_costmap']['local_costmap']['ros__parameters']
    plugins = costmap['plugins']
    if plugins.count('slope_traversability_layer') != 1 or 'obstacle_layer' in plugins:
        raise RuntimeError(
            'Expected the LiDAR-only Gazebo costmap with one slope_traversability_layer')

    costmap['plugins'] = [
        'obstacle_layer' if name == 'slope_traversability_layer' else name
        for name in plugins
    ]
    costmap.pop('slope_traversability_layer')
    costmap['obstacle_layer'] = {
        'plugin': 'nav2_costmap_2d::ObstacleLayer',
        'enabled': True,
        'clear_before_update': True,
        'observation_sources': 'pointcloud',
        'min_obstacle_height': -100.0,
        'max_obstacle_height': 100.0,
        'pointcloud': {
            'topic': 'cur_lid',
            'data_type': 'PointCloud2',
            'obstacle_max_range': 3.0,
            'obstacle_min_range': 0.0,
            'raytrace_max_range': 3.5,
            'raytrace_min_range': 0.0,
            'min_obstacle_height': 0.2,
            'max_obstacle_height': 0.65,
            'clearing': True,
            'marking': True,
            'inf_is_valid': True,
            'observation_persistence': 0.0,
        },
    }

    fd, baseline_path = tempfile.mkstemp(prefix='gazebo_net_nav_raw_lidar_', suffix='.yaml')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as output:
            yaml.safe_dump(params, output, sort_keys=False, allow_unicode=True)
    except Exception:
        os.unlink(baseline_path)
        raise

    def _remove_baseline(_context):
        try:
            os.unlink(baseline_path)
        except FileNotFoundError:
            pass
        return []

    original_launch = os.path.join(share, 'launch', 'gazebo_net_nav.launch.py')
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_remove_baseline)])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(original_launch),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'params_file': baseline_path,
                'topo_map_path': LaunchConfiguration('topo_map_path'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items(),
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory('robot_launch')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car1'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'gazebo_net_nav.yaml')),
        DeclareLaunchArgument('topo_map_path', default_value=os.path.join(
            share, 'maps', 'topo_map_gazebo.yaml')),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            share, 'rviz', 'navigation.rviz')),
        OpaqueFunction(function=_start_baseline),
    ])
