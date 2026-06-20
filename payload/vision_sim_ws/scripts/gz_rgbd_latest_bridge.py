#!/usr/bin/env python3
import os
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import copy
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node as RosNode
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from gz.transport13 import Node as GzNode
from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
from gz.msgs10.image_pb2 import Image as GzImage

PIXEL_ENCODINGS = {1: "mono8", 2: "mono16", 3: "rgb8", 4: "rgba8", 5: "bgra8", 8: "bgr8", 11: "16FC1", 13: "32FC1"}

class LatestGzRgbdBridge(RosNode):
    def __init__(self):
        super().__init__("gz_rgbd_latest_bridge")
        self.declare_parameter("gz_prefix", "/camera/camera")
        self.declare_parameter("ros_prefix", "/camera/camera")
        self.declare_parameter("publish_hz", 15.0)
        self.declare_parameter("color_frame_id", "camera_color_optical_frame")
        self.declare_parameter("depth_frame_id", "camera_depth_optical_frame")
        self.gz_prefix = self.get_parameter("gz_prefix").value.rstrip("/")
        self.ros_prefix = self.get_parameter("ros_prefix").value.rstrip("/")
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.color_frame_id = self.get_parameter("color_frame_id").value
        self.depth_frame_id = self.get_parameter("depth_frame_id").value
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.color_pub = self.create_publisher(Image, f"{self.ros_prefix}/color/image_raw", qos)
        self.depth_pub = self.create_publisher(Image, f"{self.ros_prefix}/depth/image_rect_raw", qos)
        self.color_info_pub = self.create_publisher(CameraInfo, f"{self.ros_prefix}/color/camera_info", qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, f"{self.ros_prefix}/depth/camera_info", qos)
        self.gz_node = GzNode()
        self.lock = threading.Lock()
        self.latest_color = None
        self.latest_depth = None
        self.latest_info = None
        self.gz_node.subscribe(GzImage, f"{self.gz_prefix}/image", self._color_cb)
        self.gz_node.subscribe(GzImage, f"{self.gz_prefix}/depth_image", self._depth_cb)
        self.gz_node.subscribe(GzCameraInfo, f"{self.gz_prefix}/camera_info", self._info_cb)
        self.create_timer(1.0 / max(self.publish_hz, 1.0), self._publish_latest)
        self.get_logger().info(f"Latest-frame RGBD bridge active: {self.gz_prefix} -> {self.ros_prefix}, publish_hz={self.publish_hz}")

    def _color_cb(self, msg):
        with self.lock:
            self.latest_color = msg

    def _depth_cb(self, msg):
        with self.lock:
            self.latest_depth = msg

    def _info_cb(self, msg):
        with self.lock:
            self.latest_info = msg

    def _stamp(self, msg):
        stamp = self.get_clock().now().to_msg()
        try:
            stamp.sec = msg.header.stamp.sec
            stamp.nanosec = msg.header.stamp.nsec
        except Exception:
            pass
        return stamp

    def _image_msg(self, msg, frame_id):
        out = Image()
        out.header.stamp = self._stamp(msg)
        out.header.frame_id = frame_id
        out.height = int(msg.height)
        out.width = int(msg.width)
        out.encoding = PIXEL_ENCODINGS.get(int(msg.pixel_format_type), "passthrough")
        out.is_bigendian = 0
        out.step = int(msg.step)
        out.data = bytes(msg.data)
        return out

    def _camera_info_msg(self, msg, frame_id):
        out = CameraInfo()
        out.header.stamp = self._stamp(msg)
        out.header.frame_id = frame_id
        out.width = int(msg.width)
        out.height = int(msg.height)
        try:
            out.k = list(msg.intrinsics.k)
            out.p = list(msg.projection.p)
            out.r = list(msg.rectification_matrix) if len(msg.rectification_matrix) == 9 else [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
        except Exception:
            out.r = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
        try:
            out.d = list(msg.distortion.k)
            out.distortion_model = msg.distortion.model or "plumb_bob"
        except Exception:
            out.distortion_model = "plumb_bob"
        return out

    def _publish_latest(self):
        with self.lock:
            color = self.latest_color
            depth = self.latest_depth
            info = self.latest_info
        if color is not None:
            self.color_pub.publish(self._image_msg(color, self.color_frame_id))
        if depth is not None:
            out = self._image_msg(depth, self.depth_frame_id)
            if out.encoding == "passthrough" and out.step == out.width * 4:
                out.encoding = "32FC1"
            self.depth_pub.publish(out)
        if info is not None:
            ci = self._camera_info_msg(info, self.color_frame_id)
            self.color_info_pub.publish(ci)
            di = copy.deepcopy(ci)
            di.header.frame_id = self.depth_frame_id
            self.depth_info_pub.publish(di)

def main():
    rclpy.init()
    node = LatestGzRgbdBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
