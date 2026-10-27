"""Decode ROS compressed color and compressedDepth images for plan_env."""

from array import array
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image


PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def image_message(header, image, encoding):
    message = Image()
    message.header = header
    message.height, message.width = image.shape[:2]
    message.encoding = encoding
    message.is_bigendian = 0
    message.step = message.width * image.dtype.itemsize
    if image.ndim == 3:
        message.step *= image.shape[2]
    message.data = array("B", np.ascontiguousarray(image).tobytes())
    return message


def decode_color(message):
    payload = np.frombuffer(message.data, dtype=np.uint8)
    image = cv2.imdecode(payload, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("OpenCV could not decode the compressed color image")
    return image_message(message.header, image, "bgr8")


def decode_depth(message):
    source_encoding = message.format.split(";", 1)[0].strip()
    payload = bytes(message.data)
    png_offset = payload.find(PNG_SIGNATURE)
    if png_offset < 0:
        raise ValueError("compressedDepth payload does not contain a PNG stream")
    encoded = np.frombuffer(payload, dtype=np.uint8, offset=png_offset)
    image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("OpenCV could not decode the compressedDepth PNG stream")
    if source_encoding != "16UC1" or image.dtype != np.uint16:
        raise ValueError(
            "only 16UC1 compressedDepth is supported; got format=%r dtype=%s" %
            (message.format, image.dtype))
    return image_message(message.header, image, "16UC1")


class CompressedImageDecoder(Node):
    def __init__(self):
        super().__init__("plan_env_compressed_decoder")
        self.enabled = bool(self.declare_parameter(
            "compressed_input.enabled", True).value)
        self.color_enabled = bool(self.declare_parameter(
            "compressed_input.color_enabled", True).value)
        self.depth_enabled = bool(self.declare_parameter(
            "compressed_input.depth_enabled", True).value)
        self.require_aligned_depth = bool(self.declare_parameter(
            "compressed_input.require_aligned_depth", True).value)
        self.color_topic = self.declare_parameter(
            "compressed_input.color_topic",
            "/camera/color/image_raw/compressed").value
        self.depth_topic = self.declare_parameter(
            "compressed_input.depth_topic",
            "/camera/depth/image_raw/compressedDepth").value
        self.color_output_topic = self.declare_parameter(
            "compressed_input.color_output_topic",
            "/plan_env/camera/color/image_raw").value
        self.depth_output_topic = self.declare_parameter(
            "compressed_input.depth_output_topic",
            "/plan_env/camera/depth/image_raw").value

        reliability_name = str(self.declare_parameter(
            "compressed_input.qos.reliability", "best_effort").value).lower()
        qos_depth = int(self.declare_parameter(
            "compressed_input.qos.depth", 1).value)
        if reliability_name == "reliable":
            reliability = ReliabilityPolicy.RELIABLE
        elif reliability_name == "best_effort":
            reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            raise ValueError(
                "compressed_input.qos.reliability must be reliable or best_effort")
        if qos_depth <= 0:
            raise ValueError("compressed_input.qos.depth must be positive")
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.color_shape = None
        self.color_frame_id = None
        self.last_alignment_warning = 0.0
        self.last_decode_warning = {"color": 0.0, "depth": 0.0}
        self.callback_group = ReentrantCallbackGroup()
        self.color_publisher = None
        self.depth_publisher = None
        self.color_subscription = None
        self.depth_subscription = None

        if not self.enabled:
            self.get_logger().info("compressed image decoding is disabled")
            return
        if self.color_enabled:
            self.color_publisher = self.create_publisher(
                Image, self.color_output_topic, qos_profile_sensor_data)
            self.color_subscription = self.create_subscription(
                CompressedImage, self.color_topic, self.on_color, input_qos,
                callback_group=self.callback_group)
        if self.depth_enabled:
            self.depth_publisher = self.create_publisher(
                Image, self.depth_output_topic, qos_profile_sensor_data)
            self.depth_subscription = self.create_subscription(
                CompressedImage, self.depth_topic, self.on_depth, input_qos,
                callback_group=self.callback_group)
        self.get_logger().info(
            "compressed decoder ready: color=%s -> %s, depth=%s -> %s, "
            "require_aligned_depth=%s, input_qos=%s(depth=%d)" % (
                self.color_topic if self.color_enabled else "disabled",
                self.color_output_topic,
                self.depth_topic if self.depth_enabled else "disabled",
                self.depth_output_topic,
                self.require_aligned_depth, reliability_name, qos_depth))

    def warn_decode(self, stream, error):
        now = time.monotonic()
        if now - self.last_decode_warning[stream] >= 2.0:
            self.get_logger().error(
                "%s decompression failed: %s: %s" %
                (stream, type(error).__name__, error))
            self.last_decode_warning[stream] = now

    def on_color(self, message):
        try:
            decoded = decode_color(message)
            self.color_shape = (decoded.width, decoded.height)
            self.color_frame_id = decoded.header.frame_id
            self.color_publisher.publish(decoded)
        except Exception as error:
            self.warn_decode("color", error)

    def on_depth(self, message):
        try:
            decoded = decode_depth(message)
            if self.require_aligned_depth:
                aligned = (
                    self.color_shape is not None and
                    self.color_shape == (decoded.width, decoded.height) and
                    self.color_frame_id == decoded.header.frame_id)
                if not aligned:
                    now = time.monotonic()
                    if now - self.last_alignment_warning >= 2.0:
                        self.get_logger().error(
                            "depth is not aligned to color: color=%s frame=%s, "
                            "depth=%dx%d frame=%s; suppressing decoded depth. "
                            "Enable depth-to-color registration in the camera driver, "
                            "or set compressed_input.require_aligned_depth=false only "
                            "for depth-only diagnostics" % (
                                self.color_shape, self.color_frame_id,
                                decoded.width, decoded.height,
                                decoded.header.frame_id))
                        self.last_alignment_warning = now
                    return
            self.depth_publisher.publish(decoded)
        except Exception as error:
            self.warn_decode("depth", error)


def main():
    rclpy.init()
    node = CompressedImageDecoder()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
