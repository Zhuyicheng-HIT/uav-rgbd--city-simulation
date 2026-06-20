#!/usr/bin/env python3
import argparse
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

try:
    import pyrealsense2 as rs
except Exception:
    rs = None

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None


class RosD435iViewer(Node):
    def __init__(self, color_topic, depth_topic, yolo_model=None, yolo_conf=0.35, yolo_iou=0.1, yolo_device='0', yolo_imgsz=640, yolo_hz=12.0):
        super().__init__('d435i_mavros_opencv_viewer')
        self.bridge = CvBridge()
        self.state = State()
        self.pose = None
        self.color = None
        self.depth = None
        self.yolo = None
        self.yolo_model_path = yolo_model
        self.yolo_conf = float(yolo_conf)
        self.yolo_iou = float(yolo_iou)
        self.yolo_device = str(yolo_device)
        self.yolo_imgsz = int(yolo_imgsz)
        self.yolo_min_period = 1.0 / max(float(yolo_hz), 0.1)
        self.last_yolo_time = 0.0
        self.last_yolo_frame = None
        self.last_yolo_count = 0
        if yolo_model:
            if YOLO is None:
                raise RuntimeError('ultralytics is not installed. Install with: python3 -m pip install --user -U ultralytics')
            self.get_logger().info(f'Loading YOLO model: {yolo_model}')
            self.yolo = YOLO(yolo_model)
            self.get_logger().info(f'YOLO enabled on RGB window: conf={self.yolo_conf}, iou={self.yolo_iou}, device={self.yolo_device}, imgsz={self.yolo_imgsz}')
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, 10)
        self.create_subscription(Image, color_topic, self._color_cb, sensor_qos)
        self.create_subscription(Image, depth_topic, self._depth_cb, sensor_qos)
        self.get_logger().info(f'Subscribing color={color_topic}, depth={depth_topic}, and MAVROS state/local pose')

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg

    def _color_cb(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def draw_status(self, img):
        cv2.putText(img, f'MAVROS connected={self.state.connected} armed={self.state.armed} mode={self.state.mode}',
                    (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        if self.pose is not None:
            p = self.pose.pose.position
            cv2.putText(img, f'local xyz=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})',
                        (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        if self.yolo is not None:
            cv2.putText(img, f'YOLO conf={self.yolo_conf:.2f} iou={self.yolo_iou:.2f} det={self.last_yolo_count}',
                        (12, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

    def apply_yolo(self, color):
        if self.yolo is None:
            return color
        now = time.monotonic()
        if self.last_yolo_frame is not None and now - self.last_yolo_time < self.yolo_min_period:
            return self.last_yolo_frame.copy()
        results = self.yolo.predict(
            source=color,
            conf=self.yolo_conf,
            iou=self.yolo_iou,
            imgsz=self.yolo_imgsz,
            device=self.yolo_device,
            verbose=False,
        )
        result = results[0]
        try:
            self.last_yolo_count = 0 if result.boxes is None else len(result.boxes)
        except Exception:
            self.last_yolo_count = 0
        annotated = result.plot(img=color.copy(), line_width=2, font_size=0.6)
        self.last_yolo_time = now
        self.last_yolo_frame = annotated.copy()
        return annotated

    def depth_to_colormap(self):
        depth = np.asarray(self.depth)
        if depth.dtype == np.float32 or depth.dtype == np.float64:
            depth32 = depth.astype(np.float32, copy=False)
            valid = np.isfinite(depth32) & (depth32 > 0.05)
            u8 = np.zeros(depth32.shape, dtype=np.uint8)
            u8[valid] = np.clip(depth32[valid] * (255.0 / 12.0), 0, 255).astype(np.uint8)
        else:
            u8 = cv2.convertScaleAbs(depth, alpha=0.03)
        return cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)

    def show(self):
        last_print = 0.0
        while rclpy.ok():
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.0)
            if self.color is not None:
                color = self.color.copy()
                color = self.apply_yolo(color)
                self.draw_status(color)
                cv2.imshow('D435i RGB - ROS2/MAVROS', color)
            if self.depth is not None:
                depth_color = self.depth_to_colormap()
                self.draw_status(depth_color)
                cv2.imshow('D435i Depth - ROS2/MAVROS', depth_color)
            now = time.monotonic()
            if now - last_print > 3.0:
                last_print = now
                depth_stats = ''
                if self.depth is not None:
                    depth = np.asarray(self.depth)
                    valid = np.isfinite(depth) & (depth > 0.05)
                    if np.count_nonzero(valid):
                        depth_stats = f' depth_min={float(np.min(depth[valid])):.2f} depth_max={float(np.max(depth[valid])):.2f}'
                yolo_stats = f' yolo_det={self.last_yolo_count}' if self.yolo is not None else ''
                print(f'mavros connected={self.state.connected} armed={self.state.armed} mode={self.state.mode} color={self.color is not None} depth={self.depth is not None}{depth_stats}{yolo_stats}')
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
        cv2.destroyAllWindows()


def run_realsense():
    if rs is None:
        raise RuntimeError('pyrealsense2 is not installed. Use --source ros for Gazebo simulated D435i topics.')
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(cfg)
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            depth_u8 = cv2.convertScaleAbs(depth, alpha=0.03)
            depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
            cv2.imshow('D435i RGB - pyrealsense2', color)
            cv2.imshow('D435i Depth - pyrealsense2', depth_color)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description='View Gazebo D435i ROS2 topics with MAVROS status overlay and optional YOLO overlay on RGB.')
    parser.add_argument('--source', choices=['ros', 'realsense'], default='ros')
    parser.add_argument('--color-topic', default='/camera/camera/color/image_raw')
    parser.add_argument('--depth-topic', default='/camera/camera/depth/image_rect_raw')
    parser.add_argument('--yolo-model', default='', help='Ultralytics .pt model path. If set, detections are drawn on the RGB window.')
    parser.add_argument('--yolo-conf', type=float, default=0.35)
    parser.add_argument('--yolo-iou', type=float, default=0.1)
    parser.add_argument('--yolo-device', default='0', help='Ultralytics device, e.g. 0, cuda:0, or cpu.')
    parser.add_argument('--yolo-imgsz', type=int, default=640)
    parser.add_argument('--yolo-hz', type=float, default=12.0, help='Maximum YOLO inference rate; old camera frames are dropped.')
    args = parser.parse_args()
    if args.source == 'realsense':
        run_realsense()
        return
    rclpy.init()
    node = RosD435iViewer(
        args.color_topic,
        args.depth_topic,
        yolo_model=args.yolo_model or None,
        yolo_conf=args.yolo_conf,
        yolo_iou=args.yolo_iou,
        yolo_device=args.yolo_device,
        yolo_imgsz=args.yolo_imgsz,
        yolo_hz=args.yolo_hz,
    )
    try:
        node.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
