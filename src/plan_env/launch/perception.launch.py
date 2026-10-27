from pathlib import Path
import shutil
import subprocess
import sys
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _python_has_inference_dependencies(command_prefix):
    try:
        result = subprocess.run(
            command_prefix + [
                "-c", "import cv2, numpy, rclpy, torch",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=15,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as error:
        return False, str(error)
    if result.returncode == 0:
        return True, ""
    detail = result.stderr.strip().splitlines()
    return False, detail[-1] if detail else "dependency check failed"


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
            "[plan_env] perception.mode=depth_only; inference process is not started"))]

    runtime = LaunchConfiguration("inference_runtime").perform(context).lower()
    conda_env = LaunchConfiguration("conda_env").perform(context)
    system_python = LaunchConfiguration("system_python").perform(context)
    requested_device = LaunchConfiguration("device").perform(context)
    notices = []
    if runtime not in ("auto", "conda", "system"):
        notices.append(LogInfo(msg=(
            "[plan_env][WARN] inference_runtime must be auto, conda, or system; "
            "falling back to system CPU")))
        runtime = "system"

    common_arguments = [
        "-u", inference_script,
        "--model-config", LaunchConfiguration("model_config").perform(context),
        "--weight", LaunchConfiguration("weight").perform(context),
        "--image-size", LaunchConfiguration("image_size").perform(context),
    ]
    ros_arguments = [
        "--ros-args", "--params-file", LaunchConfiguration("grid_config").perform(context),
    ]

    conda_path = shutil.which("conda")
    conda_reason = (
        "system runtime was requested"
        if runtime == "system" else "conda executable was not found"
    )
    if runtime in ("auto", "conda") and conda_path:
        conda_python = [
            conda_path, "run", "--no-capture-output", "-n", conda_env, "python",
        ]
        conda_ready, conda_reason = _python_has_inference_dependencies(conda_python)
        if conda_ready:
            command = conda_python + common_arguments + [
                "--device", requested_device,
            ] + ros_arguments
            return notices + [
                LogInfo(msg=(
                    f"[plan_env] inference runtime: conda environment '{conda_env}', "
                    f"device={requested_device}")),
                ExecuteProcess(cmd=command, output="screen"),
            ]

    system_python_path = shutil.which(system_python) if not Path(system_python).is_absolute() \
        else system_python
    if not system_python_path or not Path(system_python_path).is_file():
        return notices + [LogInfo(msg=(
            f"[plan_env][WARN] {conda_reason}; system Python '{system_python}' was not "
            "found. The inference node was not started."))]

    system_ready, system_reason = _python_has_inference_dependencies([system_python_path])
    if not system_ready:
        return notices + [LogInfo(msg=(
            f"[plan_env][WARN] {conda_reason}; system Python cannot run inference: "
            f"{system_reason}. Install torch, numpy, OpenCV and rclpy for "
            f"'{system_python_path}'. The inference node was not started."))]

    command = [system_python_path] + common_arguments + [
        "--device", "cpu",
    ] + ros_arguments
    return notices + [
        LogInfo(msg=(
            f"[plan_env][WARN] {conda_reason}; falling back to system Python "
            f"'{system_python_path}' with CPU inference")),
        ExecuteProcess(cmd=command, output="screen"),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("plan_env"))
    package_prefix = Path(get_package_prefix("plan_env"))
    inference_script = str(
        package_prefix / "lib" / "plan_env" / "plan_env_drivable_inference"
    )
    return LaunchDescription([
        DeclareLaunchArgument("inference_runtime", default_value="auto"),
        DeclareLaunchArgument("conda_env", default_value="twinLiteNetPlus"),
        DeclareLaunchArgument("system_python", default_value=sys.executable),
        DeclareLaunchArgument("model_config", default_value="medium"),
        DeclareLaunchArgument(
            "weight", default_value=str(package_share / "models" / "medium.pth")),
        DeclareLaunchArgument("image_size", default_value="640"),
        DeclareLaunchArgument("device", default_value="auto"),
        DeclareLaunchArgument(
            "grid_config", default_value=str(package_share / "config" / "depth_grid_map.yaml")),
        DeclareLaunchArgument(
            "extrinsic_config", default_value=str(package_share / "config" / "depth_grid_map.yaml")),
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
