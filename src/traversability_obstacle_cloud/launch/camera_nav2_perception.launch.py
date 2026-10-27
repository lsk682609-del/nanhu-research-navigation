import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("traversability_obstacle_cloud")
    orbbec_package_share = FindPackageShare("orbbec_camera")
    default_params = PathJoinSubstitution(
        [package_share, "config", "camera_ground_extraction.yaml"]
    )
    default_pothole_detection = (
        "true" if platform.machine().lower() in ("aarch64", "arm64") else "false"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_orbbec_camera = LaunchConfiguration("use_orbbec_camera")
    enable_sync_output_accel_gyro = LaunchConfiguration(
        "enable_sync_output_accel_gyro"
    )
    use_depth_projector = LaunchConfiguration("use_depth_projector")
    depth_input_mode = LaunchConfiguration("depth_input_mode")
    use_rgb_decoder = LaunchConfiguration("use_rgb_decoder")
    use_pothole_detection = LaunchConfiguration("use_pothole_detection")
    params_file = LaunchConfiguration("params_file")

    orbbec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [orbbec_package_share, "launch", "gemini_330_series.launch.py"]
            )
        ),
        condition=IfCondition(use_orbbec_camera),
        launch_arguments={
            "enable_sync_output_accel_gyro": enable_sync_output_accel_gyro,
        }.items(),
    )

    traversability = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "camera_single_frame_ground.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_depth_projector": use_depth_projector,
            "depth_input_mode": depth_input_mode,
            "use_rgb_decoder": use_rgb_decoder,
            "params_file": params_file,
        }.items(),
    )

    costmap_adapter = Node(
        package="traversability_obstacle_cloud",
        executable="camera_costmap_cloud_node",
        name="camera_costmap_cloud_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    pothole_detector = Node(
        package="traversability_obstacle_cloud",
        executable="pothole_detector_node",
        name="pothole_detector_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_pothole_detection),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "use_orbbec_camera",
                default_value="true",
                description="Launch the Orbbec Gemini 330 camera driver.",
            ),
            DeclareLaunchArgument(
                "enable_sync_output_accel_gyro",
                default_value="true",
                description="Enable the Orbbec synchronized accel/gyro IMU output.",
            ),
            DeclareLaunchArgument("use_depth_projector", default_value="true"),
            DeclareLaunchArgument(
                "depth_input_mode",
                default_value="config",
                choices=["config", "raw", "compressed_depth"],
            ),
            DeclareLaunchArgument("use_rgb_decoder", default_value="true"),
            DeclareLaunchArgument(
                "use_pothole_detection",
                default_value=default_pothole_detection,
                description="Run the RK3588 RKNN pothole detector and geometry fusion.",
            ),
            DeclareLaunchArgument("params_file", default_value=default_params),
            orbbec_camera,
            traversability,
            pothole_detector,
            costmap_adapter,
        ]
    )
