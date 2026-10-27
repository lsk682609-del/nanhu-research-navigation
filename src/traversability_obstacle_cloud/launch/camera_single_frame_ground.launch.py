from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_depth_projector = LaunchConfiguration("use_depth_projector")
    depth_input_mode = LaunchConfiguration("depth_input_mode")
    use_rgb_decoder = LaunchConfiguration("use_rgb_decoder")
    params_file = LaunchConfiguration("params_file")
    default_params = PathJoinSubstitution(
        [
            FindPackageShare("traversability_obstacle_cloud"),
            "config",
            "camera_ground_extraction.yaml",
        ]
    )

    common_parameters = [params_file, {"use_sim_time": use_sim_time}]

    def make_depth_projector(context):
        # "config" leaves input_mode entirely to the YAML file. A concrete launch
        # value remains available as an explicit command-line override.
        selected_mode = depth_input_mode.perform(context)
        projector_parameters = list(common_parameters)
        if selected_mode != "config":
            projector_parameters.append({"input_mode": selected_mode})
        return [
            Node(
                package="traversability_obstacle_cloud",
                executable="depth_cloud_projector_node",
                name="depth_cloud_projector_node",
                output="screen",
                parameters=projector_parameters,
                condition=IfCondition(use_depth_projector),
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "use_depth_projector",
                default_value="true",
                description="Project /camera/depth/image_raw to /depth/points",
            ),
            DeclareLaunchArgument(
                "depth_input_mode",
                default_value="config",
                choices=["config", "raw", "compressed_depth"],
                description=(
                    "Depth input mode override. 'config' uses input_mode from the "
                    "parameter YAML file"
                ),
            ),
            DeclareLaunchArgument(
                "use_rgb_decoder",
                default_value="true",
                description="Decode the Orbbec compressed JPEG RGB topic",
            ),
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="traversability_obstacle_cloud",
                executable="traversability_obstacle_cloud_node",
                name="traversability_obstacle_cloud_node",
                output="screen",
                parameters=common_parameters,
            ),
            OpaqueFunction(function=make_depth_projector),
            Node(
                package="traversability_obstacle_cloud",
                executable="rgb_image_decoder_node",
                name="rgb_image_decoder_node",
                output="screen",
                parameters=common_parameters,
                condition=IfCondition(use_rgb_decoder),
            ),
        ]
    )
