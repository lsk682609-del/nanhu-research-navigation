from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("plan_env")) / "config" / "camera_calibration.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("imu_topic", default_value="/camera/gyro_accel/sample"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("output_yaml", default_value="/tmp/gemini330l_extrinsic.yaml"),
        Node(
            package="plan_env",
            executable="camera_extrinsic_calibrator",
            name="camera_extrinsic_calibrator",
            output="screen",
            parameters=[LaunchConfiguration("config"), {
                "calibration.output_yaml": LaunchConfiguration("output_yaml"),
            }],
            remappings=[
                ("imu", LaunchConfiguration("imu_topic")),
                ("depth", LaunchConfiguration("depth_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),
            ],
        ),
    ])
