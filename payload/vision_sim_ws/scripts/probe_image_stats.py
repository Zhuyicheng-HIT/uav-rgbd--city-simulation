#!/usr/bin/env python3
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class ImageStats(Node):
    def __init__(self):
        super().__init__("probe_image_stats")
        self.color = []
        self.depth = []
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, "/camera/camera/color/image_raw", self._color_cb, sensor_qos)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self._depth_cb, sensor_qos)

    def _color_cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        self.color.append((
            msg.width, msg.height, msg.encoding,
            float(arr.mean()) if arr.size else -1.0,
            float(arr.std()) if arr.size else -1.0,
            int(arr.min()) if arr.size else -1,
            int(arr.max()) if arr.size else -1,
        ))

    def _depth_cb(self, msg):
        dtype = np.float32 if msg.step == msg.width * 4 else np.uint8
        arr = np.frombuffer(msg.data, dtype=dtype)
        if dtype == np.float32:
            valid = np.isfinite(arr) & (arr > 0.05)
            vals = arr[valid]
            self.depth.append((
                msg.width, msg.height, msg.encoding, int(valid.sum()),
                float(vals.min()) if vals.size else -1.0,
                float(vals.max()) if vals.size else -1.0,
                float(vals.mean()) if vals.size else -1.0,
                float(vals.std()) if vals.size else -1.0,
            ))
        else:
            self.depth.append((
                msg.width, msg.height, msg.encoding, int(arr.size),
                float(arr.min()) if arr.size else -1.0,
                float(arr.max()) if arr.size else -1.0,
                float(arr.mean()) if arr.size else -1.0,
                float(arr.std()) if arr.size else -1.0,
            ))


def main():
    rclpy.init()
    node = ImageStats()
    end = time.monotonic() + 8.0
    while rclpy.ok() and time.monotonic() < end and (len(node.color) < 5 or len(node.depth) < 5):
        rclpy.spin_once(node, timeout_sec=0.2)
    print("color_samples", len(node.color))
    for item in node.color[-3:]:
        print("color", item)
    print("depth_samples", len(node.depth))
    for item in node.depth[-3:]:
        print("depth", item)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
