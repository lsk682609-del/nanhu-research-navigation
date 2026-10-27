"""ROS 2 node for plan_env's embedded TwinLiteNetPlus inference runtime."""

import argparse
from array import array
import queue
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from .inference import InferenceEngine, image_message_to_bgr


class DrivableAreaNode(Node):
    def __init__(self, args):
        super().__init__("plan_env_drivable_inference")
        self.args = args
        self.perception_mode = str(self.declare_parameter(
            "perception.mode", "fused").value).lower()
        if self.perception_mode not in ("semantic_only", "depth_only", "fused"):
            raise ValueError(
                "perception.mode must be semantic_only, depth_only, or fused")
        if self.perception_mode == "depth_only":
            self.get_logger().info(
                "perception.mode=depth_only; model loading and image inference are disabled")
            return

        self.input_topic = self.declare_parameter(
            "topics.color_image", args.input_topic).value
        self.probability_topic = self.declare_parameter(
            "topics.drivable_probability", args.probability_topic).value
        self.mask_topic = self.declare_parameter(
            "topics.drivable_mask", args.mask_topic).value
        self.lane_enabled = bool(self.declare_parameter(
            "lane.enabled", False).value)
        self.lane_probability_topic = self.declare_parameter(
            "topics.lane_probability", "/twinlitenet/lane_probability").value
        self.lane_mask_topic = self.declare_parameter(
            "topics.lane_mask", "/twinlitenet/lane_mask").value
        self.lane_publish_probability = bool(self.declare_parameter(
            "lane.publish_probability", False).value)
        self.lane_publish_mask = bool(self.declare_parameter(
            "lane.publish_mask", True).value)
        self.lane_mask_threshold = float(self.declare_parameter(
            "lane.mask_threshold", 0.50).value)
        self.publish_probability = bool(self.declare_parameter(
            "output.publish_probability", False).value)
        self.publish_mask = bool(self.declare_parameter(
            "output.publish_mask", True).value)
        self.mask_threshold = float(self.declare_parameter(
            "output.mask_threshold", args.mask_threshold).value)
        self.debug_enabled = self.declare_parameter(
            "debug.enabled", True).value
        self.debug_image_topic = self.declare_parameter(
            "debug.image_topic", args.overlay_topic).value
        if not 0.0 <= self.mask_threshold <= 1.0:
            raise ValueError("output.mask_threshold must be in [0, 1]")
        if not 0.0 <= self.lane_mask_threshold <= 1.0:
            raise ValueError("lane.mask_threshold must be in [0, 1]")

        input_reliability = str(self.declare_parameter(
            "input_qos.reliability", "best_effort").value).lower()
        input_qos_depth = int(self.declare_parameter(
            "input_qos.depth", 1).value)
        if input_reliability == "reliable":
            reliability = ReliabilityPolicy.RELIABLE
        elif input_reliability == "best_effort":
            reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            raise ValueError(
                "input_qos.reliability must be 'reliable' or 'best_effort'")
        if input_qos_depth <= 0:
            raise ValueError("input_qos.depth must be positive")
        self.input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=input_qos_depth,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.input_qos_name = input_reliability

        self.watchdog_enabled = bool(self.declare_parameter(
            "watchdog.enabled", True).value)
        self.input_timeout_sec = float(self.declare_parameter(
            "watchdog.input_timeout_sec", 3.0).value)
        self.processing_timeout_sec = float(self.declare_parameter(
            "watchdog.processing_timeout_sec", 10.0).value)
        self.watchdog_check_period_sec = float(self.declare_parameter(
            "watchdog.check_period_sec", 1.0).value)
        if (self.input_timeout_sec <= 0.0 or self.processing_timeout_sec <= 0.0 or
                self.watchdog_check_period_sec <= 0.0):
            raise ValueError("watchdog timeout and check-period values must be positive")

        self.engine = InferenceEngine(
            args.model_config, args.weight, args.image_size, args.device, args.half,
            enable_lane=self.lane_enabled)
        self.frames = queue.Queue(maxsize=1)
        self.probability_publisher = None
        if self.publish_probability:
            self.probability_publisher = self.create_publisher(
                Image, self.probability_topic, qos_profile_sensor_data)
        self.mask_publisher = None
        if self.publish_mask:
            self.mask_publisher = self.create_publisher(
                Image, self.mask_topic, qos_profile_sensor_data)
        self.lane_probability_publisher = None
        if self.lane_enabled and self.lane_publish_probability:
            self.lane_probability_publisher = self.create_publisher(
                Image, self.lane_probability_topic, qos_profile_sensor_data)
        self.lane_mask_publisher = None
        if self.lane_enabled and self.lane_publish_mask:
            self.lane_mask_publisher = self.create_publisher(
                Image, self.lane_mask_topic, qos_profile_sensor_data)
        self.overlay_publisher = None
        if self.debug_enabled:
            self.overlay_publisher = self.create_publisher(
                Image, self.debug_image_topic, qos_profile_sensor_data)
        self.inference_time_publisher = self.create_publisher(
            Float32, args.timing_topic, 10)
        self.total_time_publisher = self.create_publisher(
            Float32, args.total_timing_topic, 10)
        now = time.monotonic()
        self.last_input_time = now
        self.last_output_time = now
        self.processing_started_at = None
        self.last_processing_warning = 0.0
        self.subscription = None
        self.create_input_subscription()
        self.last_status_time = 0.0
        self.processing_thread = threading.Thread(
            target=self.process_frames, name="plan_env_inference", daemon=True)
        self.processing_thread.start()
        self.watchdog_timer = None
        if self.watchdog_enabled:
            self.watchdog_timer = self.create_timer(
                self.watchdog_check_period_sec, self.watchdog_callback)
        self.get_logger().info(
            "drivable inference ready: model=%s device=%s half=%s "
            "input=%s input_qos=%s(depth=%d) probability=%s mask=%s "
            "threshold=%.2f lane=%s lane_probability=%s lane_mask=%s "
            "lane_threshold=%.2f debug=%s "
            "watchdog=%s weight=%s" % (
                args.model_config, self.engine.device, self.engine.use_half,
                self.input_topic, self.input_qos_name, self.input_qos.depth,
                self.publish_probability, self.publish_mask,
                self.mask_threshold, self.lane_enabled,
                self.lane_publish_probability, self.lane_publish_mask,
                self.lane_mask_threshold, self.debug_enabled,
                self.watchdog_enabled, args.weight))

    def create_input_subscription(self):
        self.subscription = self.create_subscription(
            Image, self.input_topic, self.on_image, self.input_qos)

    def recreate_input_subscription(self, silence_sec):
        old_subscription = self.subscription
        self.subscription = None
        if old_subscription is not None:
            self.destroy_subscription(old_subscription)
        self.create_input_subscription()
        self.last_input_time = time.monotonic()
        self.get_logger().warning(
            "no input image for %.2f s on %s; recreated subscription with "
            "%s QoS (depth=%d)" % (
                silence_sec, self.input_topic, self.input_qos_name,
                self.input_qos.depth))

    def watchdog_callback(self):
        now = time.monotonic()
        silence_sec = now - self.last_input_time
        if silence_sec >= self.input_timeout_sec:
            self.recreate_input_subscription(silence_sec)

        processing_started_at = self.processing_started_at
        if processing_started_at is None:
            return
        processing_sec = now - processing_started_at
        if (processing_sec >= self.processing_timeout_sec and
                now - self.last_processing_warning >= self.processing_timeout_sec):
            self.get_logger().error(
                "inference has not completed for %.2f s; input subscription is alive, "
                "check CUDA/runtime health" % processing_sec)
            self.last_processing_warning = now

    def on_image(self, message):
        self.last_input_time = time.monotonic()
        try:
            self.frames.put_nowait(message)
        except queue.Full:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass
            try:
                self.frames.put_nowait(message)
            except queue.Full:
                pass

    @staticmethod
    def publish_timing(publisher, value):
        message = Float32()
        message.data = float(value)
        publisher.publish(message)

    @staticmethod
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

    def process_frames(self):
        while rclpy.ok():
            try:
                source = self.frames.get(timeout=0.2)
            except queue.Empty:
                continue
            started = time.perf_counter()
            self.processing_started_at = time.monotonic()
            try:
                bgr = image_message_to_bgr(source)
                probability, lane_probability, inference_ms = self.engine.infer(bgr)
                if self.probability_publisher is not None:
                    self.probability_publisher.publish(
                        self.image_message(source.header, probability, "32FC1"))
                if self.lane_probability_publisher is not None:
                    self.lane_probability_publisher.publish(
                        self.image_message(
                            source.header, lane_probability, "32FC1"))

                publish_mask = (
                    self.mask_publisher is not None
                    and self.mask_publisher.get_subscription_count() > 0)
                publish_overlay = (
                    self.overlay_publisher is not None
                    and self.overlay_publisher.get_subscription_count() > 0)
                publish_lane_mask = (
                    self.lane_mask_publisher is not None
                    and self.lane_mask_publisher.get_subscription_count() > 0)
                if publish_mask or publish_overlay:
                    mask = (probability >= self.mask_threshold).astype(np.uint8) * 255
                    if publish_mask:
                        self.mask_publisher.publish(
                            self.image_message(source.header, mask, "mono8"))
                    if publish_overlay:
                        overlay = bgr.copy()
                        active = mask != 0
                        if active.any():
                            green = np.zeros_like(bgr[active])
                            green[:, 1] = 255
                            overlay[active] = cv2.addWeighted(
                                bgr[active], 0.5, green, 0.5, 0)
                else:
                    overlay = None

                if lane_probability is not None and (publish_lane_mask or publish_overlay):
                    lane_mask = (
                        lane_probability >= self.lane_mask_threshold).astype(np.uint8) * 255
                    if publish_lane_mask:
                        self.lane_mask_publisher.publish(
                            self.image_message(source.header, lane_mask, "mono8"))
                    if publish_overlay:
                        lane_active = lane_mask != 0
                        if lane_active.any():
                            red = np.zeros_like(bgr[lane_active])
                            red[:, 2] = 255
                            overlay[lane_active] = cv2.addWeighted(
                                bgr[lane_active], 0.35, red, 0.65, 0)

                if publish_overlay:
                    overlay_rgb = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)
                    self.overlay_publisher.publish(
                        self.image_message(source.header, overlay_rgb, "rgb8"))

                total_ms = (time.perf_counter() - started) * 1000.0
                self.last_output_time = time.monotonic()
                self.publish_timing(self.inference_time_publisher, inference_ms)
                self.publish_timing(self.total_time_publisher, total_ms)
                now = time.monotonic()
                if now - self.last_status_time >= 2.0:
                    self.get_logger().info(
                        "inference %.2f ms, total %.2f ms" % (inference_ms, total_ms))
                    self.last_status_time = now
            except Exception as error:
                self.get_logger().error(
                    "frame processing failed: %s: %s" %
                    (type(error).__name__, error))
            finally:
                self.processing_started_at = None


def create_parser(default_weight):
    parser = argparse.ArgumentParser(
        description="plan_env TwinLiteNetPlus drivable-area inference")
    parser.add_argument("--input-topic", default="/camera/color/image_raw")
    parser.add_argument(
        "--probability-topic", default="/twinlitenet/drivable_probability")
    parser.add_argument("--mask-topic", default="/twinlitenet/drivable_mask")
    parser.add_argument("--overlay-topic", default="/twinlitenet/segmentation")
    parser.add_argument("--timing-topic", default="/twinlitenet/inference_time_ms")
    parser.add_argument(
        "--total-timing-topic", default="/twinlitenet/total_processing_time_ms")
    parser.add_argument(
        "--model-config", choices=("nano", "small", "medium", "large"),
        default="medium")
    parser.add_argument("--weight", default=default_weight)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--mask-threshold", type=float, default=0.5)
    parser.add_argument("--half", dest="half", action="store_true")
    parser.add_argument("--no-half", dest="half", action="store_false")
    parser.set_defaults(half=True)
    return parser


def main(default_weight):
    args, ros_arguments = create_parser(default_weight).parse_known_args()
    if not 0.0 <= args.mask_threshold <= 1.0:
        raise ValueError("mask-threshold must be in [0, 1]")
    rclpy.init(args=ros_arguments)
    node = DrivableAreaNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
