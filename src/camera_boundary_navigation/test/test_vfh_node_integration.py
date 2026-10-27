#!/usr/bin/env python3
"""Synthetic ROS integration check; isolated domain and test-only velocity topic.

Usage: python3 test_vfh_node_integration.py /path/to/camera_boundary_navigation_node
"""
import os
import struct
import subprocess
import sys
import time
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = "173"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["ROS_LOG_DIR"] = "/tmp/camera_vfh_test_ros_logs"

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import qos_profile_sensor_data


def main():
    params = Path(__file__).resolve().parents[1] / "config/boundary_follower.yaml"
    log = open("/tmp/camera_vfh_node_integration.log", "w")
    process = subprocess.Popen(
        [sys.argv[1], "--ros-args", "--params-file", str(params),
         "-p", "motion.enabled_on_startup:=true", "-p", "cmd_vel_topic:=/camera_vfh_test/cmd_vel"],
        stdout=log, stderr=subprocess.STDOUT,
    )
    rclpy.init()
    node = rclpy.create_node("camera_vfh_integration_fixture")
    publishers = {name: node.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
                  for name, topic in [("boundary", "/traversability/costmap/boundary_constraints"),
                                      ("grid", "/traversability/grid"),
                                      ("obstacle", "/traversability/costmap/depth_obstacles")]}
    samples = []
    velocities = []
    node.create_subscription(DiagnosticArray, "/camera_navigation/status",
                             lambda msg: samples.append(msg.status[0]), 10)
    node.create_subscription(Twist, "/camera_vfh_test/cmd_vel",
                             lambda msg: velocities.append(msg), 10)

    def cloud(points, stamp, frame="gravity_aligned_base"):
        msg = PointCloud2()
        msg.header.stamp.sec = stamp.sec
        msg.header.stamp.nanosec = stamp.nanosec
        msg.header.frame_id = frame
        msg.height = 1
        msg.width = len(points)
        msg.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(["x", "y", "z", "intensity"])]
        msg.point_step = 16
        msg.row_step = 16 * len(points)
        msg.data = b"".join(struct.pack("ffff", *p) for p in points)
        return msg

    boundary = [(0.6 + i * .075, -.65, 0., 2.) for i in range(20)]
    grid = [((x+.5)*.15, (y+.5)*.15, 0., 1.) for x in range(20) for y in range(-15, 15)]

    def run_scene(obstacles=(), seconds=1.0, fault=None, boundary_points=None, ground_points=None):
        samples.clear()
        velocities.clear()
        end = time.monotonic() + seconds
        next_frame = 0.
        while time.monotonic() < end:
            assert process.poll() is None, Path(log.name).read_text()
            if time.monotonic() >= next_frame:
                stamp = node.get_clock().now().to_msg()
                if fault == "stale":
                    stamp.sec -= 2
                if fault == "future":
                    stamp.sec += 2
                messages = {"boundary": cloud(boundary if boundary_points is None else boundary_points, stamp),
                            "grid": cloud(grid if ground_points is None else ground_points, stamp),
                            "obstacle": cloud(obstacles, stamp)}
                if fault == "wrong_frame":
                    messages["obstacle"].header.frame_id = "camera_optical_frame"
                if fault == "malformed":
                    messages["obstacle"].fields = []
                if fault == "mismatch":
                    messages["obstacle"].header.stamp.nanosec = max(0, stamp.nanosec - 1)
                for name, msg in messages.items():
                    if fault == "dropout" and name == "obstacle":
                        continue
                    publishers[name].publish(msg)
                next_frame = time.monotonic() + .065
            rclpy.spin_once(node, timeout_sec=.01)
        assert samples and velocities, "No diagnostics or commands received"
        return samples[-1], velocities[-1]

    def values(status):
        return {entry.key: entry.value for entry in status.values}

    try:
        # Allow discovery before making assertions.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        status, cmd = run_scene()
        assert status.message == "TRACKING" and cmd.linear.x > 0., (status, cmd)
        print("PASS clear scene tracks", flush=True)
        obstacle = [(1.35 + i*.005, .025, .3, 0.) for i in range(3)]
        status, cmd = run_scene(obstacle)
        assert values(status)["avoidance_mode"] == "AVOID", status
        assert cmd.linear.x > 0. and abs(cmd.angular.z) > 0., (status, cmd)
        print("PASS central obstacle bypass", flush=True)
        # This wall is close enough that even the configured 1 m minimum safe
        # prefix is unavailable.  A farther wall may now permit a short,
        # low-speed prefix by design and is covered by the C++ adaptive tests.
        wall = [(.75+i*.005, y*.15+.025, .3, 0.)
                for y in range(-15, 15) for i in range(3)]
        status, cmd = run_scene(wall)
        assert values(status)["avoidance_mode"] == "BLOCKED" and cmd.linear.x == 0., (status, cmd)
        print("PASS wall stops", flush=True)
        status, cmd = run_scene(seconds=2.0)
        assert values(status)["avoidance_mode"] == "FOLLOW" and cmd.linear.x > 0., (status, cmd)
        print("PASS clear scene recovers", flush=True)
        for missing in [[], [(1., -.65, 0., 2.)],
                        [(p[0], p[1], p[2], -2.) for p in boundary]]:
            status, cmd = run_scene(seconds=2., boundary_points=missing)
            assert status.message == "TRACKING" and cmd.linear.x > 0., (status, cmd)
            assert values(status)["navigation_mode"] == "BOUNDARYLESS_FORWARD", status
            assert values(status)["boundary_valid"] == "false", status
            assert abs(cmd.angular.z) < 1e-4, cmd
        print("PASS empty/sparse/FOV-only boundary moves forward", flush=True)
        status, cmd = run_scene(obstacle, boundary_points=[])
        assert values(status)["avoidance_mode"] == "AVOID", status
        assert cmd.linear.x > 0. and abs(cmd.angular.z) > 0., (status, cmd)
        print("PASS boundaryless obstacle bypass", flush=True)
        status, cmd = run_scene(wall, boundary_points=[])
        assert cmd.linear.x == 0. and cmd.angular.z == 0., (status, cmd)
        status, cmd = run_scene(boundary_points=[], ground_points=[])
        assert cmd.linear.x == 0. and cmd.angular.z == 0., (status, cmd)
        print("PASS boundaryless wall/unknown ground stops", flush=True)
        status, cmd = run_scene(seconds=2.)
        assert values(status)["navigation_mode"] == "BOUNDARY_FOLLOW", status
        assert status.message == "TRACKING" and cmd.linear.x > 0., (status, cmd)
        print("PASS boundary recovery resumes following", flush=True)
        corridor = boundary + [(p[0], 1.0, p[2], 1.) for p in boundary]
        status, cmd = run_scene(seconds=2., boundary_points=corridor)
        assert values(status)["navigation_mode"] == "CORRIDOR_CENTER", status
        assert status.message == "TRACKING" and cmd.angular.z > 0., (status, cmd)
        print("PASS asymmetric two-sided corridor steers toward center", flush=True)
        narrow = [(p[0], .45, 0., 1.) for p in boundary] + [
            (p[0], -.45, 0., 2.) for p in boundary]
        status, cmd = run_scene(boundary_points=narrow)
        assert values(status)["avoidance_reason"] == "corridor_too_narrow", status
        assert cmd.linear.x == 0. and cmd.angular.z == 0., (status, cmd)
        print("PASS corridor too narrow stops without obstacle points", flush=True)
        left_only = [(p[0], .65, 0., 1.) for p in boundary]
        status, cmd = run_scene(seconds=2., boundary_points=left_only)
        assert values(status)["navigation_mode"] == "BOUNDARY_FOLLOW", status
        assert status.message == "TRACKING" and cmd.linear.x > 0., (status, cmd)
        print("PASS opposite single boundary remains usable", flush=True)
        assert values(status)["active_reference_side"] == "left", status
        assert abs(cmd.angular.z) < 1e-4, cmd
        disconnected = [(x, -.65, 0., 2.) for x in
                        [.6, .675, .75, .825, 1.6, 1.675, 1.75, 1.825]]
        status, cmd = run_scene(seconds=2., boundary_points=disconnected)
        assert values(status)["navigation_mode"] == "BOUNDARYLESS_FORWARD", status
        assert values(status)["boundary_valid"] == "false", status
        print("PASS boundary fitting does not bridge visibility gap", flush=True)
        close_right = [(1.0+i*.05, -.525, .3, 0.) for i in range(21)]
        status, cmd = run_scene(close_right, seconds=2.)
        assert values(status)["avoidance_mode"] == "AVOID", status
        assert cmd.linear.x > 0. and cmd.angular.z > 0., (status, cmd)
        print("PASS low-clearance straight actively steers inward", flush=True)
        for fault in ["dropout", "wrong_frame", "malformed", "mismatch", "stale", "future"]:
            status, cmd = run_scene(fault=fault)
            assert cmd.linear.x == 0. and cmd.angular.z == 0., (fault, status, cmd)
            assert status.message == "WAITING_FOR_INPUT", (fault, status)
            print(f"PASS {fault} stops", flush=True)
            status, cmd = run_scene()
            assert status.message == "TRACKING" and cmd.linear.x > 0., (fault, status, cmd)
        print("All ROS integration scenarios passed.", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()


if __name__ == "__main__":
    main()
