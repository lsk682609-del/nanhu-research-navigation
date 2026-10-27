from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = Path(get_package_share_directory("plan_env"))
    package_prefix = Path(get_package_prefix("plan_env"))
    decoder_script = str(
        package_prefix / "lib" / "plan_env" / "plan_env_compressed_decoder")
    config = str(
        package_share / "config" /
        "depth_grid_map_gemini335l_compressed.yaml")
    perception_launch = str(package_share / "launch" / "perception.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("inference_runtime", default_value="auto"),
        DeclareLaunchArgument("conda_env", default_value="twinLiteNetPlus"),
        DeclareLaunchArgument("system_python", default_value=sys.executable),
        DeclareLaunchArgument("decoder_python", default_value=sys.executable),
        DeclareLaunchArgument("model_config", default_value="large"),
        DeclareLaunchArgument(
            "weight",
            default_value=str(package_share / "models" / "large.pth")),
        DeclareLaunchArgument("image_size", default_value="640"),
        DeclareLaunchArgument("device", default_value="auto"),
        DeclareLaunchArgument("grid_config", default_value=config),
        DeclareLaunchArgument("extrinsic_config", default_value=config),
        LogInfo(msg=(
            "[plan_env] Gemini 335L compressed input enabled; aligned "
            "depth-to-color is required for fused mapping")),
        ExecuteProcess(
            cmd=[
                LaunchConfiguration("decoder_python"), "-u", decoder_script,
                "--ros-args", "--params-file", LaunchConfiguration("grid_config"),
            ],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                "inference_runtime": LaunchConfiguration("inference_runtime"),
                "conda_env": LaunchConfiguration("conda_env"),
                "system_python": LaunchConfiguration("system_python"),
                "model_config": LaunchConfiguration("model_config"),
                "weight": LaunchConfiguration("weight"),
                "image_size": LaunchConfiguration("image_size"),
                "device": LaunchConfiguration("device"),
                "grid_config": LaunchConfiguration("grid_config"),
                "extrinsic_config": LaunchConfiguration("extrinsic_config"),
            }.items(),
        ),
    ])
