#!/usr/bin/env python3
"""Perform a lightweight sanity check of live Azure Kinect ROS 2 streams."""

import argparse
import json
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2


class StreamValidator(Node):
    """Inspect one message from each high-value sensor stream."""

    def __init__(self, namespace):
        super().__init__("k4a_stream_validator")
        prefix = namespace.rstrip("/")
        self.results = {}
        self.subscriptions_by_name = {}
        self._subscribe("color", Image, f"{prefix}/color/image_raw", self._color)
        self._subscribe("depth", Image, f"{prefix}/depth/image_raw", self._depth)
        self._subscribe("ir", Image, f"{prefix}/ir/image_raw", self._ir)
        self._subscribe("points", PointCloud2, f"{prefix}/points", self._points)
        self._subscribe("imu", Imu, f"{prefix}/imu", self._imu)

    def _subscribe(self, name, message_type, topic, callback):
        self.subscriptions_by_name[name] = self.create_subscription(
            message_type, topic, callback, qos_profile_sensor_data
        )

    def _complete(self, name, result):
        self.results[name] = result
        subscription = self.subscriptions_by_name.pop(name, None)
        if subscription is not None:
            self.destroy_subscription(subscription)

    @staticmethod
    def _sample_bytes(data, maximum=8192):
        if not data:
            return []
        step = max(1, len(data) // maximum)
        return data[::step]

    @staticmethod
    def _sample_u16(data, maximum=20000):
        view = memoryview(data).cast("H")
        step = max(1, len(view) // maximum)
        return view[::step]

    def _color(self, message):
        samples = self._sample_bytes(message.data)
        self._complete(
            "color",
            {
                "encoding": message.encoding,
                "size": [message.width, message.height],
                "sample_min": min(samples),
                "sample_max": max(samples),
            },
        )

    def _depth(self, message):
        samples = self._sample_u16(message.data)
        valid = [value for value in samples if value > 0]
        self._complete(
            "depth",
            {
                "encoding": message.encoding,
                "size": [message.width, message.height],
                "valid_sample_ratio": len(valid) / len(samples),
                "valid_min_mm": min(valid) if valid else None,
                "valid_max_mm": max(valid) if valid else None,
            },
        )

    def _ir(self, message):
        samples = self._sample_u16(message.data)
        self._complete(
            "ir",
            {
                "encoding": message.encoding,
                "size": [message.width, message.height],
                "sample_min": min(samples),
                "sample_max": max(samples),
            },
        )

    def _points(self, message):
        field_offsets = {field.name: field.offset for field in message.fields}
        finite_z = []
        sample_stride = max(1, (message.width * message.height) // 20000)
        for index in range(0, message.width * message.height, sample_stride):
            offset = index * message.point_step
            x = struct.unpack_from("<f", message.data, offset + field_offsets["x"])[0]
            y = struct.unpack_from("<f", message.data, offset + field_offsets["y"])[0]
            z = struct.unpack_from("<f", message.data, offset + field_offsets["z"])[0]
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                finite_z.append(z)
        self._complete(
            "points",
            {
                "organized_size": [message.width, message.height],
                "fields": sorted(field_offsets),
                "finite_sample_count": len(finite_z),
                "z_min_m": min(finite_z) if finite_z else None,
                "z_max_m": max(finite_z) if finite_z else None,
            },
        )

    def _imu(self, message):
        acceleration_norm = math.sqrt(
            message.linear_acceleration.x**2
            + message.linear_acceleration.y**2
            + message.linear_acceleration.z**2
        )
        self._complete(
            "imu",
            {
                "frame_id": message.header.frame_id,
                "acceleration_norm_m_s2": acceleration_norm,
            },
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/k4a")
    parser.add_argument("--timeout", type=float, default=15.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = StreamValidator(args.namespace)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and node.subscriptions_by_name and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    missing = sorted(node.subscriptions_by_name)
    print(json.dumps({"streams": node.results, "missing": missing}, indent=2))
    healthy = not missing
    healthy = healthy and node.results["color"]["sample_max"] > node.results["color"]["sample_min"]
    healthy = healthy and node.results["depth"]["valid_sample_ratio"] > 0.01
    healthy = healthy and node.results["ir"]["sample_max"] > node.results["ir"]["sample_min"]
    healthy = healthy and node.results["points"]["finite_sample_count"] > 0
    node.destroy_node()
    rclpy.shutdown()
    return 0 if healthy else 1


if __name__ == "__main__":
    raise SystemExit(main())
