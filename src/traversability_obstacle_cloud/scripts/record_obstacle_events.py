#!/usr/bin/env python3
"""Record suspected interior-obstacle events; does not label them as false positives."""
import argparse
from collections import deque
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from rosidl_runtime_py.utilities import get_message

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_BAG = '/home/huibo/tdr/navigation/nanhu-research-navigation/rosbag/rosbag2_2026_08_26-16_25_06'
BOUNDARY = '/traversability/costmap/boundary_constraints'
DEPTH = '/traversability/costmap/depth_obstacles'
CLASSIFIED = '/traversability/classified_boundary_points'
TOPICS = {t: 'sensor_msgs/msg/PointCloud2' for t in [
    '/depth/points', '/ground_points', '/obstacle_points', '/unknown_points',
    '/traversability/ground_grid_points', '/traversability/grid',
    '/traversability/ground_boundary_points', CLASSIFIED, BOUNDARY, DEPTH,
    '/traversability/costmap/depth_noise',
    '/traversability/costmap/depth_ground_rejected']}
TOPICS.update({
    '/camera/gyro_accel/sample': 'sensor_msgs/msg/Imu',
    '/camera/depth/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/camera/color/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/camera/color/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
    '/camera/depth/image_raw/compressedDepth': 'sensor_msgs/msg/CompressedImage'})


def stamp(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


def points(msg):
    fields = {f.name: f for f in msg.fields}
    names = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in fields else [])
    if any(fields[n].datatype != 7 for n in names):
        raise ValueError('Expected FLOAT32 XYZ/intensity fields')
    dtype = np.dtype({'names': names, 'formats': [('>' if msg.is_bigendian else '<') + 'f4'] * len(names),
                      'offsets': [fields[n].offset for n in names], 'itemsize': msg.point_step})
    rows = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step))
    out = np.column_stack([rows[n].reshape(-1) for n in names])
    return out[np.isfinite(out).all(axis=1)]


def interpolate(side, xs, gap):
    result = np.full(len(xs), np.nan)
    if not len(side):
        return result
    side = side[np.argsort(side[:, 0])]
    for x, y in side[:, :2]:
        result[np.abs(xs - x) < 1e-5] = y
    for a, b in zip(side[:-1], side[1:]):
        dx = b[0] - a[0]
        if 0 < dx <= gap:
            mask = (xs >= a[0]) & (xs <= b[0])
            result[mask] = a[1] + (xs[mask] - a[0]) / dx * (b[1] - a[1])
    return result


def detect(obstacles, boundary, vetoes, args):
    p = obstacles[:, :3]
    left = interpolate(boundary[boundary[:, 3] == 1], p[:, 0], args.max_gap)
    right = interpolate(boundary[boundary[:, 3] == 2], p[:, 0], args.max_gap)
    covered = np.isfinite(left) & np.isfinite(right) & (left > right)
    for v in vetoes:
        if v[3] in (-1, -2):
            covered[np.abs(p[:, 0] - v[0]) <= args.cell_size / 2] = False
    inside = covered & (p[:, 1] > right + args.margin) & (p[:, 1] < left - args.margin)
    fallback = (~covered & (p[:, 0] >= args.x_min) & (p[:, 0] <= args.x_max) &
                (np.abs(p[:, 1]) < args.half_width)) if args.fallback else np.zeros(len(p), bool)
    selected = p[inside | fallback]
    return selected, int(inside.sum()), int(fallback.sum())


class Recorder(Node):
    def __init__(self, args):
        super().__init__('obstacle_event_recorder')
        self.args = args
        self.directory = Path(args.output) / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True)
        metadata = Path(args.source_bag) / 'metadata.yaml'
        if not metadata.exists():
            raise ValueError(f'Missing source bag metadata: {metadata}')
        shutil.copy2(metadata, self.directory / 'source_metadata.yaml')
        manifest = dict(vars(args), metadata_sha256=hashlib.sha256(metadata.read_bytes()).hexdigest(),
                        classification='suspected_event_not_confirmed_false_positive',
                        limitations=['No classification reason or filtered IMU rotation is published by current nodes',
                                     'No odometry in supplied bag; no motion compensation',
                                     'Pre-window can be shortened by memory limit',
                                     'Metadata hash identifies metadata, not a full bag content checksum'])
        try:
            manifest['git_revision'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
            manifest['git_diff'] = subprocess.check_output(['git', 'diff'], text=True)
        except (OSError, subprocess.CalledProcessError):
            pass
        if args.params:
            shutil.copy2(args.params, self.directory / 'perception_params.yaml')
        (self.directory / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
        self.ring = deque()
        self.bytes = 0
        self.latest = 0
        self.cache = {BOUNDARY: {}, DEPTH: {}, CLASSIFIED: {}}
        self.writer = None
        self.event = None
        self.count = 0
        self.last_trigger = 0
        self.last_detection = 0
        self.consecutive = 0
        self.stream = None
        self.subs = [self.create_subscription(get_message(kind), topic,
                    lambda msg, t=topic: self.receive(t, msg), qos_profile_sensor_data)
                    for topic, kind in TOPICS.items()]
        self.get_logger().info(f'Recording suspected events to {self.directory}')

    def finish(self):
        if self.writer is not None:
            self.writer = None  # SequentialWriter destructor closes and flushes bag metadata.
            self.stream.close()
            self.get_logger().info(f'Closed event {self.count}')

    def receive(self, topic, msg):
        t = stamp(msg)
        if t < self.latest - 2 * 10**9:
            self.finish()
            self.ring.clear()
            self.bytes = 0
            for c in self.cache.values():
                c.clear()
            self.consecutive = 0
            self.latest = t
        self.latest = max(t, self.latest)
        data = serialize_message(msg)
        if self.writer is not None and t > self.last_trigger + int(self.args.post * 10**9):
            self.finish()
        if self.writer is not None:
            self.writer.write(topic, data, t)
        self.ring.append((topic, data, t))
        self.bytes += len(data)
        while self.ring and (self.ring[0][2] < self.latest - int(self.args.pre * 10**9) or
                             self.bytes > self.args.memory_mb * 1024**2):
            self.bytes -= len(self.ring.popleft()[1])
        if topic not in self.cache:
            return
        self.cache[topic][t] = msg
        for c in self.cache.values():
            for old in list(c):
                if old < self.latest - 10**9:
                    del c[old]
        if not all(t in c for c in self.cache.values()):
            return
        b, d, v = (self.cache[k].pop(t) for k in (BOUNDARY, DEPTH, CLASSIFIED))
        # classified boundaries use gravity_aligned_base, adapter relabels to base_link.
        if b.header.frame_id != d.header.frame_id:
            self.get_logger().warning('Boundary/depth frame mismatch; skipped')
            return
        selected, inside, fallback = detect(points(d), points(b), points(v), self.args)
        if t <= self.last_detection:
            return
        if t - self.last_detection > 500000000:
            self.consecutive = 0
        self.last_detection = t
        self.consecutive = self.consecutive + 1 if len(selected) >= self.args.min_points else 0
        if self.consecutive < self.args.frames:
            return
        self.last_trigger = t
        if self.writer is None:
            self.count += 1
            self.event = self.directory / f'event_{self.count:05d}'
            self.event.mkdir()
            self.writer = rosbag2_py.SequentialWriter()
            self.writer.open(rosbag2_py.StorageOptions(uri=str(self.event / 'bag'), storage_id='sqlite3'),
                             rosbag2_py.ConverterOptions('', ''))
            for name, kind in TOPICS.items():
                self.writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format='cdr'))
            for name, payload, ts in self.ring:
                self.writer.write(name, payload, ts)
            self.stream = (self.event / 'detections.jsonl').open('w')
            self.get_logger().info(f'Event {self.count}: inside={inside}, fallback={fallback}')
        record = dict(stamp_ns=t, match_delta_ns=0, inside_points=inside, fallback_points=fallback,
                      bbox_min=selected.min(axis=0).tolist(), bbox_max=selected.max(axis=0).tolist(),
                      retained_pre_start_ns=min(r[2] for r in self.ring))
        self.stream.write(json.dumps(record) + '\n')
        self.stream.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-bag', default=DEFAULT_BAG)
    parser.add_argument('--output', default='obstacle_events')
    parser.add_argument('--params', default='')
    parser.add_argument('--pre', type=float, default=2.0)
    parser.add_argument('--post', type=float, default=3.0)
    parser.add_argument('--memory-mb', type=int, default=512)
    parser.add_argument('--margin', type=float, default=0.10)
    parser.add_argument('--max-gap', type=float, default=0.10)
    parser.add_argument('--cell-size', type=float, default=0.15)
    parser.add_argument('--min-points', type=int, default=5)
    parser.add_argument('--frames', type=int, default=3)
    parser.add_argument('--fallback', action='store_true', help='Also check fixed corridor where paired boundaries are absent')
    parser.add_argument('--x-min', type=float, default=0.30)
    parser.add_argument('--x-max', type=float, default=4.0)
    parser.add_argument('--half-width', type=float, default=0.30)
    args = parser.parse_args()
    if min(args.pre, args.post, args.margin) < 0 or min(args.memory_mb, args.frames, args.min_points,
            args.max_gap, args.cell_size, args.half_width) <= 0 or args.x_min >= args.x_max:
        parser.error('Invalid time, memory, detection threshold or corridor bounds')
    rclpy.init(args=[])
    node = Recorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
