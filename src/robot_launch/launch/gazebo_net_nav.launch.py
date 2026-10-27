"""LiDAR-only road-network navigation for vehicle_gazebo_sim/test_world.world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_launch')
    namespace = LaunchConfiguration('namespace')
    params = LaunchConfiguration('params_file')
    topology = LaunchConfiguration('topo_map_path')
    nodes = [
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
        # Gazebo publishes the model tree on global /tf and /tf_static.
        # Only input_bridge publishes map -> base_link; odom TF stays disabled.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='navigation_base_to_footprint',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'base_footprint'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': True}], output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]
    specifications = [
        ('nav2_input_bridge', 'input_bridge_node', 'nav2_input_bridge'),
        ('nav2_net_planner', 'net_planner_action_server', 'net_planner_action_server'),
        ('nav2_net_system', 'net_navigator_node', 'net_navigator'),
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_log_record', 'log_server_node', 'nav2_log_record'),
        ('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation'),
    ]
    for package, executable, name in specifications:
        overrides = {'use_sim_time': True}
        remappings = []
        if name == 'net_planner_action_server':
            overrides['topo_map_path'] = topology
        if name == 'net_navigator':
            remappings.append(('goal_pose', LaunchConfiguration('goal_topic')))
        if name == 'controller_server':
            remappings.extend([
                ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                # Also matches the internal local_costmap's relative subscription.
                ('cur_lid', PathJoinSubstitution(['/', namespace, 'cur_lid'])),
            ])
        if name == 'nav2_input_bridge':
            # GPS input is unused; isolate its periodic origin request from MAVROS.
            remappings.extend([
                ('/mavros/global_position/set_gp_origin', 'unused_gps/set_gp_origin'),
                ('/mavros/global_position/gp_origin', 'unused_gps/gp_origin'),
            ])
        nodes.append(Node(
            package=package, executable=executable, name=name, namespace=namespace,
            parameters=[params, overrides], remappings=remappings, output='screen'))
    return LaunchDescription(nodes)
