from pathlib import Path
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_inference(context, inference_script):
    config_path = LaunchConfiguration("grid_config").perform(context)
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    global_parameters = config.get("/**", {}).get("ros__parameters", {})
    node_parameters = config.get(
        "plan_env_drivable_inference", {}).get("ros__parameters", {})
    perception_mode = str(node_parameters.get(
        "perception.mode", global_parameters.get("perception.mode", "fused"))).lower()
    if perception_mode == "depth_only":
        return [LogInfo(msg=(
            "[plan_env] perception.mode=depth_only; Jetson inference is not started"))]

    return [
        LogInfo(msg=[
            "[plan_env] Jetson Orin inference: python=",
            LaunchConfiguration("venv_python"),
            " device=", LaunchConfiguration("device"),
        ]),
        ExecuteProcess(
            cmd=[
                LaunchConfiguration("venv_python"), "-u", inference_script,
                "--model-config", LaunchConfiguration("model_config"),
                "--weight", LaunchConfiguration("weight"),
                "--image-size", LaunchConfiguration("image_size"),
                "--device", LaunchConfiguration("device"),
                "--ros-args", "--params-file", LaunchConfiguration("grid_config"),
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("plan_env"))
    package_prefix = Path(get_package_prefix("plan_env"))
    inference_script = str(
        package_prefix / "lib" / "plan_env" / "plan_env_drivable_inference"
    )
    default_venv_python = str(
        Path.home() / "venvs" / "twinlite" / "bin" / "python"
    )

    return LaunchDescription([
        DeclareLaunchArgument("venv_python", default_value=default_venv_python),
        DeclareLaunchArgument("model_config", default_value="medium"),
        DeclareLaunchArgument(
            "weight", default_value=str(package_share / "models" / "medium.pth")),
        DeclareLaunchArgument("image_size", default_value="640"),
        DeclareLaunchArgument("device", default_value="cuda:0"),
        DeclareLaunchArgument(
            "grid_config",
            default_value=str(
                package_share / "config" /
                "depth_grid_map_jetson_orin_realsense_d435.yaml")),
        DeclareLaunchArgument(
            "extrinsic_config",
            default_value=str(
                package_share / "config" /
                "depth_grid_map_jetson_orin_realsense_d435.yaml")),
        OpaqueFunction(function=_launch_inference, args=[inference_script]),
        Node(
            package="plan_env",
            executable="grid_map_node",
            name="grid_map_node",
            output="screen",
            parameters=[
                LaunchConfiguration("grid_config"),
                LaunchConfiguration("extrinsic_config"),
            ],
        ),
    ])
