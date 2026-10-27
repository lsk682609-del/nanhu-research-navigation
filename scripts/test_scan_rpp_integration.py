#!/usr/bin/env python3
"""Isolated ROS2 smoke test: real controller_server, no robot/driver, synthetic inputs."""
import os
os.environ["ROS_DOMAIN_ID"] = "187"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["ROS_LOG_DIR"] = "/tmp/scan_rpp_test_logs"
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import FollowPath
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


def main():
    root = Path(__file__).resolve().parents[1]
    log = tempfile.NamedTemporaryFile(prefix="scan_controller_", suffix=".log", delete=False)
    process = subprocess.Popen([
        str(root / "install/nav2_controller/lib/nav2_controller/controller_server"),
        "--ros-args", "--params-file", str(root / "src/robot_launch/config/scan_rpp.yaml"),
        "-r", "__ns:=/scan_test", "-r", "cmd_vel:=test_cmd_vel",
    ], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node("scan_integration", namespace="/scan_test")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    transforms = node.create_publisher(TFMessage, "/tf_static", qos)
    cloud_pub = node.create_publisher(point_cloud2.PointCloud2, "cur_lid", 1)
    commands = []
    node.create_subscription(Twist, "test_cmd_vel",
                             lambda m: commands.append((time.monotonic(), m.linear.x, m.angular.z)), 10)

    def spin_until(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if process.poll() is not None:
                raise RuntimeError("controller_server exited; see " + log.name)
            if predicate():
                return
        raise TimeoutError("ROS integration condition timed out; see " + log.name)

    def transition(identifier):
        client = node.create_client(ChangeState, "controller_server/change_state")
        if not client.wait_for_service(timeout_sec=10):
            raise TimeoutError("No lifecycle service; see " + log.name)
        request = ChangeState.Request()
        request.transition.id = identifier
        future = client.call_async(request)
        spin_until(future.done, 15)
        assert future.result().success, "Lifecycle transition failed; see " + log.name
        node.destroy_client(client)

    def cloud():
        header = Header(frame_id="lidar", stamp=node.get_clock().now().to_msg())
        # Actual returns well away from the straight tracking corridor.
        points = [(4.0, math.sin(i * 0.1) * 2.0, 0.05) for i in range(30)]
        cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    try:
        message = TFMessage()
        for child, height in (("base_link", 0.0), ("lidar", 0.3)):
            t = TransformStamped()
            t.header.frame_id = "map"
            t.child_frame_id = child
            t.transform.translation.z = height
            t.transform.rotation.w = 1.0
            message.transforms.append(t)
        transforms.publish(message)
        transition(1)
        transition(3)
        timer = node.create_timer(0.05, cloud)
        start = time.monotonic()
        spin_until(lambda: time.monotonic() - start > 0.8)
        action = ActionClient(node, FollowPath, "follow_path")
        assert action.wait_for_server(timeout_sec=5)
        goal = FollowPath.Goal()
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"
        goal.path.header.frame_id = "map"
        for i in range(31):
            p = PoseStamped()
            p.header.frame_id = "map"
            p.pose.position.x = i * 0.1
            p.pose.orientation.w = 1.0
            goal.path.poses.append(p)
        future = action.send_goal_async(goal)
        spin_until(future.done)
        handle = future.result()
        assert handle.accepted
        spin_until(lambda: any(v > 0.01 for _, v, _ in commands), 8)
        timer.cancel()
        cutoff = time.monotonic()
        spin_until(lambda: any(t > cutoff + 0.65 and abs(v) < 1e-9 and abs(w) < 1e-9
                              for t, v, w in commands), 4)
        assert not any(t > cutoff + 0.7 and (abs(v) > 1e-9 or abs(w) > 1e-9)
                       for t, v, w in commands), "Motion after sensor timeout"
        cancel = handle.cancel_goal_async()
        spin_until(cancel.done)
        transition(4)
        transition(2)
        print("PASS: empty Costmap shell, SCAN/GridMap perception, RPP tracking, stale-input stop, lifecycle cleanup")
        print("Controller log:", log.name)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()


if __name__ == "__main__":
    main()
