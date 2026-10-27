#!/usr/bin/env python3
"""Export every color image from a ROS 2 bag as a JPEG file."""

import argparse
from pathlib import Path
import re
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


RAW_IMAGE_TYPE = "sensor_msgs/msg/Image"
COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
DEFAULT_TOPICS = (
    "/camera/color/image_raw",
    "/camera/color/image_raw/compressed",
)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="将ROS 2 bag中的每一帧彩色图像导出为JPG。"
    )
    parser.add_argument("bag", type=Path, help="rosbag目录或MCAP/DB3文件")
    parser.add_argument("output", type=Path, help="JPG输出目录")
    parser.add_argument(
        "--topic",
        help="彩色图像话题；省略时自动选择常见Orbbec彩色话题",
    )
    parser.add_argument(
        "--quality", type=int, default=95, help="JPEG质量，范围1-100（默认95）"
    )
    parser.add_argument(
        "--prefix", default="color", help="输出文件名前缀（默认color）"
    )
    parser.add_argument(
        "--storage-id",
        choices=("sqlite3", "mcap"),
        help="存储插件；通常可从metadata.yaml自动判断",
    )
    args = parser.parse_args()
    if not 1 <= args.quality <= 100:
        parser.error("--quality必须在1到100之间")
    return args


def detect_storage_id(bag_path, requested):
    if requested:
        return requested
    if bag_path.is_file():
        if bag_path.suffix.lower() == ".mcap":
            return "mcap"
        if bag_path.suffix.lower() == ".db3":
            return "sqlite3"
    metadata = bag_path / "metadata.yaml"
    if metadata.is_file():
        match = re.search(
            r"^\s*storage_identifier:\s*([^\s#]+)",
            metadata.read_text(encoding="utf-8"),
            flags=re.MULTILINE,
        )
        if match:
            return match.group(1)
    return "sqlite3"


def open_reader(bag_path, storage_id):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def choose_topic(topic_types, requested):
    supported = {
        name: msg_type
        for name, msg_type in topic_types.items()
        if msg_type in (RAW_IMAGE_TYPE, COMPRESSED_IMAGE_TYPE)
    }
    if requested:
        if requested not in topic_types:
            raise RuntimeError(f"bag中不存在话题: {requested}")
        if requested not in supported:
            raise RuntimeError(
                f"话题{requested}的类型为{topic_types[requested]}，不是图像消息"
            )
        return requested, supported[requested]

    for topic in DEFAULT_TOPICS:
        if topic in supported:
            return topic, supported[topic]
    candidates = [name for name in supported if "color" in name.lower()]
    if len(candidates) == 1:
        topic = candidates[0]
        return topic, supported[topic]
    available = ", ".join(sorted(supported)) or "无"
    raise RuntimeError(f"无法自动确定彩色话题，请使用--topic指定。图像话题: {available}")


def raw_image_to_bgr(message):
    encoding = message.encoding.lower()
    formats = {
        "bgr8": (3, None),
        "8uc3": (3, None),
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
        "8uc1": (1, cv2.COLOR_GRAY2BGR),
    }
    if encoding not in formats:
        raise ValueError(f"不支持的原始图像编码: {message.encoding}")
    channels, conversion = formats[encoding]
    row_bytes = int(message.width) * channels
    if message.step < row_bytes or len(message.data) < message.step * message.height:
        raise ValueError("图像step无效或数据不完整")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        int(message.height), int(message.step)
    )
    pixels = rows[:, :row_bytes]
    if channels == 1:
        image = pixels.reshape(int(message.height), int(message.width))
    else:
        image = pixels.reshape(int(message.height), int(message.width), channels)
    if conversion is not None:
        return cv2.cvtColor(image, conversion)
    return np.ascontiguousarray(image)


def message_to_bgr(message, message_type):
    if message_type == COMPRESSED_IMAGE_TYPE:
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("压缩图像解码失败")
        return image
    return raw_image_to_bgr(message)


def message_timestamp_ns(message, bag_timestamp_ns):
    stamp = message.header.stamp
    timestamp = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return timestamp if timestamp > 0 else int(bag_timestamp_ns)


def main():
    args = parse_arguments()
    if not args.bag.exists():
        print(f"错误: bag路径不存在: {args.bag}", file=sys.stderr)
        return 2

    try:
        storage_id = detect_storage_id(args.bag, args.storage_id)
        reader = open_reader(args.bag, storage_id)
        topic_types = {
            item.name: item.type for item in reader.get_all_topics_and_types()
        }
        topic, message_type = choose_topic(topic_types, args.topic)
        message_class = get_message(message_type)
    except Exception as error:
        print(f"错误: {error}", file=sys.stderr)
        return 2

    args.output.mkdir(parents=True, exist_ok=True)
    print(f"bag: {args.bag}")
    print(f"storage: {storage_id}")
    print(f"topic: {topic} [{message_type}]")
    print(f"output: {args.output}")

    saved = 0
    failed = 0
    while reader.has_next():
        current_topic, serialized, bag_timestamp = reader.read_next()
        if current_topic != topic:
            continue
        try:
            message = deserialize_message(serialized, message_class)
            image = message_to_bgr(message, message_type)
            timestamp_ns = message_timestamp_ns(message, bag_timestamp)
            filename = f"{args.prefix}_{saved:06d}_{timestamp_ns}.jpg"
            output_path = args.output / filename
            if not cv2.imwrite(
                str(output_path), image, [cv2.IMWRITE_JPEG_QUALITY, args.quality]
            ):
                raise RuntimeError("OpenCV写入JPG失败")
            saved += 1
            if saved % 100 == 0:
                print(f"已保存 {saved} 帧")
        except Exception as error:
            failed += 1
            print(f"警告: 第{saved + failed}帧导出失败: {error}", file=sys.stderr)

    print(f"完成: 保存{saved}帧，失败{failed}帧")
    return 0 if saved > 0 and failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
