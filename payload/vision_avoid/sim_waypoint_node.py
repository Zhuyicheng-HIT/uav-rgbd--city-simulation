# -*- coding: utf-8 -*-
"""
视觉避障航点生成节点
======================

本文件是从原始的 “D435i + YOLO + 卡尔曼轨迹避障系统” 重构而来。
重构后的定位非常明确：

    本程序只做视觉感知、目标跟踪、轨迹预测、安全航点生成。
    本程序不直接输出 vx/vy/vz，也不直接控制 APM 飞控。

真实飞行时，本程序发布一个 ROS2 航点话题，另外的状态机程序订阅该航点，
再决定是否发送给 APM 飞控。这样可以让视觉算法和飞行状态机解耦，降低风险。

两种运行模式：

1. 调试模式 DEBUG
   - 不启动 ROS2 节点
   - 不订阅位姿、不发布航点
   - 使用带深度的离线包进行调试
   - 支持 RealSense .bag，Windows / Ubuntu 都比较容易使用
   - 支持 ROS2 .db3 离线包，但需要当前 Python 环境有 ROS2 的 rosbag2_py 等依赖

2. 真实飞行模式 FLIGHT
   - 启动 ROS2 节点
   - 实时读取 D435i
   - 订阅无人机位姿和 IMU
   - 发布 /vision/avoidance_waypoint 航点
   - 发布 /vision/avoidance_status 调试状态

坐标约定：

- 视觉规划内部统一使用“机体系水平坐标”：
    right_m   : 目标在无人机右侧为正，单位米
    forward_m : 目标在无人机前方为正，单位米

- 安全点 safe_point_body = (right_m, forward_m)

- 真实飞行模式发布航点时，才把机体系安全点转换到 ROS local/world 坐标系。

避障逻辑：

- 不再计算速度指令
- 只搜索安全航点
- 安全点会尽量靠近上一帧安全点，避免航点抖动
- 对每个目标，不只避开离散预测点，还会避开“当前点到未来预测轨迹折线”的走廊
- 默认轨迹走廊左右各 1m 范围不可进入

运行方式：

    直接修改本文件顶部“用户可调参数区”，然后执行：

        python vision_waypoint_node.py

不再使用 argparse，不需要命令行参数。
"""

from __future__ import annotations

import json
import math
import os
import csv
import sqlite3
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import cv2
import numpy as np

try:
    import torch
except ImportError:
    torch = None

try:
    import pyrealsense2 as rs
except ImportError:  # 允许只调试 ROS2 .db3 时没有 pyrealsense2，但 RealSense .bag/相机会不可用
    rs = None

from ultralytics import YOLO



class StageProfiler:
    def __init__(self, interval_s: float = 2.0, window: int = 120) -> None:
        self.interval_s = interval_s
        self.samples = defaultdict(lambda: deque(maxlen=window))
        self.last_print = time.monotonic()

    def add(self, **seconds: float) -> None:
        for name, value in seconds.items():
            self.samples[name].append(max(0.0, float(value)))

    def maybe_print(self, prefix: str = "PROFILE") -> None:
        now = time.monotonic()
        if now - self.last_print < self.interval_s:
            return
        self.last_print = now
        order = ["pose", "stabilize", "yolo", "track", "plan_publish", "display", "total"]
        parts = []
        total_avg = None
        for name in order:
            vals = self.samples.get(name)
            if not vals:
                continue
            avg_ms = sum(vals) / len(vals) * 1000.0
            if name == "total":
                total_avg = sum(vals) / len(vals)
            parts.append(f"{name}={avg_ms:.1f}ms")
        if total_avg and total_avg > 0:
            parts.append(f"fps={1.0 / total_avg:.1f}")
        print(prefix + " " + " ".join(parts), flush=True)


# =========================================================
# 用户可调参数区
# =========================================================
# 说明：
#   你后续主要改这里即可，不需要改 argparse，也不需要在命令行后面拼参数。

# -------------------------
# 1. 运行模式
# -------------------------
# 可选值：
#   "DEBUG"  : 离线调试模式，不启用 ROS2 节点，不发布航点
#   "FLIGHT" : 真实飞行模式，启用 ROS2，发布航点给状态机
RUN_MODE = os.environ.get("VISION_AVOID_RUN_MODE", "FLIGHT")

# -------------------------
# 2. 调试模式输入
# -------------------------
# 调试模式只使用“有深度”的离线数据，不使用普通 RGB 视频。
# 可选值：
#   "realsense_bag" : RealSense Viewer / pyrealsense2 录制的 .bag
#   "ros2_db3"      : ROS2 rosbag2 录制的 .db3，通常是一个包含 metadata.yaml 的目录
DEBUG_INPUT_TYPE = "ros2_db3" 

# RealSense .bag 路径。
# Windows 可写 r"D:\\data\\xxx.bag"，Ubuntu 可写 "/home/xxx/data/xxx.bag"。
DEBUG_REALSENSE_BAG_PATH = r"d435i_bag_records\record_20260615_185230.bag"
DEBUG_BAG_LOOP_PLAY = False

# ROS2 .db3 路径。
# 注意：rosbag2_py 通常要求传入 bag 目录，而不是单独的 .db3 文件。
# 如果你传入的是 .db3 文件，本程序会自动改用它的父目录。
DEBUG_ROS2_DB3_PATH = r"/home/zyc/vision_avoid/record_20260618_172548"
DEBUG_DB3_LOOP_PLAY = False

# ROS2 .db3 中的图像话题名，需要按你实际 rosbag 修改。
# 建议录制“彩色图、对齐到彩色图的深度图、相机内参”。
DEBUG_DB3_COLOR_TOPIC = "/device_0/sensor_1/Color_0/image/data"
DEBUG_DB3_DEPTH_TOPIC = "/device_0/sensor_0/Depth_0/image/data"
DEBUG_DB3_CAMERA_INFO_TOPIC = "/device_0/sensor_1/Color_0/camera_info"

# 16UC1 深度图通常单位是毫米，因此乘 0.001 变成米。
# 如果你的 .db3 深度图已经是 32FC1 米单位，这个参数不会再额外使用。
DEBUG_DB3_DEPTH_SCALE_M = 0.001

# 如果 .db3 没有录制 CameraInfo，则使用实测 D435i 640x480 RGB 内参兜底。
# 在线运行仍优先读取 /camera_info；深度内参为 fx=fy=385.78912354, ppx=325.4546814, ppy=237.49458313。
DEBUG_DB3_MANUAL_FX = 606.35186768
DEBUG_DB3_MANUAL_FY = 605.97052002
DEBUG_DB3_MANUAL_PPX = 328.86410522
DEBUG_DB3_MANUAL_PPY = 247.47494507
DEBUG_DB3_MANUAL_WIDTH = 640
DEBUG_DB3_MANUAL_HEIGHT = 480

# 调试模式最多处理多少帧。0 表示不限。
DEBUG_MAX_FRAMES = int(os.environ.get("VISION_AVOID_MAX_FRAMES", "0"))
DEBUG_START_FRAME = max(0, int(os.environ.get("VISION_AVOID_START_FRAME", "0")))

# -------------------------
# 3. 真实飞行 ROS2 设置
# -------------------------
FLIGHT_NODE_NAME = os.environ.get("VISION_AVOID_NODE_NAME", "vision_waypoint_node")
FLIGHT_POSE_TOPIC = os.environ.get("VISION_AVOID_POSE_TOPIC", "/mavros/local_position/pose")
FLIGHT_IMU_TOPIC = os.environ.get("VISION_AVOID_IMU_TOPIC", "/mavros/imu/data")
RGBD_COLOR_TOPIC = os.environ.get("VISION_AVOID_COLOR_TOPIC", "/camera/camera/color/image_raw")
RGBD_DEPTH_TOPIC = os.environ.get("VISION_AVOID_DEPTH_TOPIC", "/camera/camera/depth/image_rect_raw")
RGBD_CAMERA_INFO_TOPIC = os.environ.get("VISION_AVOID_CAMERA_INFO_TOPIC", "/camera/camera/color/camera_info")
ENABLE_GAZEBO_POSE_FALLBACK = os.environ.get("VISION_AVOID_GZ_POSE_FALLBACK", "1") != "0"
GAZEBO_WORLD_NAME = os.environ.get("VISION_AVOID_GZ_WORLD", "city_apm_rgbd")
GAZEBO_MODEL_NAME = os.environ.get("VISION_AVOID_GZ_MODEL", "apm_iris")

# 发布给状态机的航点话题。
WAYPOINT_TOPIC = "/vision/avoidance_waypoint"
STATUS_TOPIC = "/vision/avoidance_status"
WAYPOINT_FRAME_ID = "map"

# 航点高度策略：
#   "KEEP_CURRENT" : 航点 z 保持当前无人机 z
#   "FIXED"        : 航点 z 固定为 FIXED_WAYPOINT_Z_M
WAYPOINT_Z_MODE = "KEEP_CURRENT"
FIXED_WAYPOINT_Z_M = 1.5

# ENU yaw 约定。
#   "ROS_ENU_X_FORWARD" : 标准 ROS ENU，yaw=0 时机头朝 world +X，yaw=pi/2 朝 world +Y
#   "ENU_Y_FORWARD"     : 兼容某些旧代码/工程，yaw=0 时机头朝 world +Y，右侧朝 world +X
# 如果真实飞行时航点方向偏 90 度，优先检查这个参数。
WAYPOINT_YAW_CONVENTION = "ROS_ENU_X_FORWARD"

# -------------------------
# 4. YOLO 检测参数
# -------------------------
DEFAULT_SIM_MODEL = (
    "/home/zyc/vision_avoid/irreality.engine"
    if Path("/home/zyc/vision_avoid/irreality.engine").exists()
    else "/home/zyc/vision_avoid/irreality.pt"
)
MODEL_PATH = os.environ.get("VISION_AVOID_MODEL", DEFAULT_SIM_MODEL)
YOLO_CONF = float(os.environ.get("VISION_AVOID_YOLO_CONF", "0.60"))
YOLO_IOU = float(os.environ.get("VISION_AVOID_YOLO_IOU", "0.05"))  # NMS IoU 阈值。默认很低，用于尽量压掉同一目标上的重复框。
YOLO_IMGSZ = int(os.environ.get("VISION_AVOID_YOLO_IMGSZ", "640"))
YOLO_DEVICE = os.environ.get("VISION_AVOID_YOLO_DEVICE", "auto")
YOLO_CLASSES: Optional[List[str]] = None  # 例如 ["person"]，None 表示不过滤类别

# -------------------------
# 5. 相机与深度参数
# -------------------------
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
MIN_DEPTH_M = 0.2
MAX_DEPTH_M = 15.0
DEPTH_CROP_MARGIN_RATIO = 0.2

# -------------------------
# 6. 卡尔曼跟踪参数
# -------------------------
DEFAULT_PROCESS_VAR = 1.0
DEFAULT_MEASUREMENT_VAR = 0.08
INIT_POS_COV = 0.2
INIT_VEL_COV = 4.0

PREDICTION_HORIZON_S = 3.0
PREDICTION_DT_S = 0.2
# 仿真 YOLO 识别稳定，但 RGB-D + Gazebo pose 会有世界坐标抖动；默认比真实飞行略紧。
# 真实飞行中 D435i 深度噪声、漏检和机体扰动更明显，建议 MATCH_DISTANCE_M=2.2, DUPLICATE_SPAWN_DISTANCE_M=1.2。
MATCH_DISTANCE_M = float(os.environ.get("VISION_AVOID_MATCH_DISTANCE_M", "1.8"))
MAX_MISSED_FRAMES = 20
DUPLICATE_SPAWN_DISTANCE_M = float(os.environ.get("VISION_AVOID_DUPLICATE_SPAWN_DISTANCE_M", "1.4"))

# -------------------------
# 7. 安全航点搜索参数
# -------------------------
# 安全点前向默认距离。没有障碍时，安全点会逐步回到 (0, WAYPOINT_LOOKAHEAD_M)。
WAYPOINT_LOOKAHEAD_M = 0.0

# 目标当前点和预测点周围的基础安全半径。
SAFETY_RADIUS_M = 1.2

# 新增逻辑：目标当前点到预测轨迹折线的左右走廊半径。
# 候选安全点如果落在任一目标轨迹线左右 1m 范围内，会被直接淘汰。
TRAJECTORY_CORRIDOR_RADIUS_M = 1.0

# 上一安全点保持裕度。上一帧安全点如果加上裕度后仍安全，就继续保持，减少抖动。
SAFE_POINT_HOLD_MARGIN_M = 0.15

# 安全点单帧最大移动步长。越小越平滑，但绕障响应会更慢。
SAFE_POINT_MAX_STEP_M = 0.25

# 候选点横向搜索范围与分辨率。
SAFE_POINT_LATERAL_MAX_M = 3.0
SAFE_POINT_LATERAL_STEP_M = float(os.environ.get("VISION_AVOID_SAFE_STEP_M", "0.4"))
SAFE_POINT_FORWARD_MAX_M = 3.0
SAFE_POINT_FORWARD_STEP_M = float(os.environ.get("VISION_AVOID_SAFE_STEP_M", "0.4"))

# 候选点前向搜索比例。
# 例如 1.0 表示 WAYPOINT_LOOKAHEAD_M，0.5 表示一半前向距离。
SAFE_POINT_FORWARD_RATIOS = [0.0]
MAX_PLANNING_TRACKS = int(os.environ.get("VISION_AVOID_MAX_PLANNING_TRACKS", "8"))

# 局部真实坐标系：视觉程序激活时，以无人机地面投影为原点建立固定小地图。
# tracker、预测、安全点和 Top View 都在这个局部 map 坐标系里运行。
LOCAL_MAP_HALF_RANGE_M = float(os.environ.get("VISION_AVOID_LOCAL_MAP_HALF_RANGE_M", "10.0"))
LOCAL_MAP_REBASE_FRACTION = float(os.environ.get("VISION_AVOID_LOCAL_MAP_REBASE_FRACTION", "0.5"))
LOCAL_MAP_REBASE_MIN_SHIFT_M = float(os.environ.get("VISION_AVOID_LOCAL_MAP_REBASE_MIN_SHIFT_M", "0.25"))
LOCAL_MAP_SEARCH_RADIUS_M = float(os.environ.get("VISION_AVOID_LOCAL_MAP_SEARCH_RADIUS_M", "4.0"))

# 代价权重。
# CONTINUITY 权重加大后，安全点会更倾向于离上一帧安全点近，减少左右跳变。
POINT_CLEARANCE_COST_WEIGHT = 5.0
CORRIDOR_CLEARANCE_COST_WEIGHT = 8.0
CONTINUITY_COST_WEIGHT = 10.0
FORWARD_COST_WEIGHT = 1.0
LATERAL_COST_WEIGHT = 0.25

# -------------------------
# 8. IMU 图像防抖参数
# -------------------------
# 调试模式不启用 IMU 防抖。
# 真实飞行模式下，如果 IMU 话题有效，可以启用。
ENABLE_IMU_STABILIZATION = True
IMU_FILTER_ALPHA = 0.94
IMU_MAX_AGE_S = 0.35
IMU_SHAKE_RATE_THRESHOLD_RAD_S = 2.5
IMU_CORRECTION_LIMIT_DEG = 8.0
IMU_ROLL_GAIN = 1.0
IMU_PITCH_GAIN = 1.0
IMU_YAW_GAIN = 1.0

# -------------------------
# 9. 可视化与输出
# -------------------------
DISPLAY_ENABLE = os.environ.get("VISION_AVOID_DISPLAY", "1") != "0"
OUTPUT_VIDEO_PATH: Optional[str] = os.environ.get("VISION_AVOID_OUTPUT_VIDEO") or None
PRINT_INTERVAL_S = 0.2
PROFILE_ENABLE = os.environ.get("VISION_AVOID_PROFILE", "0") == "1"
PROFILE_INTERVAL_S = float(os.environ.get("VISION_AVOID_PROFILE_INTERVAL", "1.0"))

# 可视化颜色。
PANEL_BG = (28, 28, 28)
GRID_COLOR = (80, 80, 80)
DRONE_MARKER_COLOR = (255, 255, 255)
SAFE_POINT_COLOR = (0, 255, 0)
NORMAL_TRACK_COLOR = (0, 180, 255)
PREDICTED_TRACK_COLOR = (0, 165, 255)
CORRIDOR_COLOR = (55, 55, 95)
STATUS_TEXT_COLOR = (0, 255, 255)
GROUND_VIEW_SCALE = 55.0
FONT = cv2.FONT_HERSHEY_SIMPLEX

# Ultralytics 配置目录固定到项目目录下，避免 Windows/Ubuntu 用户目录权限导致的问题。
PROJECT_DIR = Path(__file__).resolve().parent
ULTRALYTICS_CONFIG_ROOT = PROJECT_DIR / ".ultralytics_config"
ULTRALYTICS_CONFIG_ROOT.mkdir(exist_ok=True)
os.environ.setdefault("YOLO_CONFIG_DIR", str(ULTRALYTICS_CONFIG_ROOT))
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")


# =========================================================
# 基础数据结构
# =========================================================
@dataclass
class CameraIntrinsics:
    """相机针孔模型内参。"""

    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float


@dataclass
class FramePacket:
    """输入源输出的一帧数据。"""

    color_bgr: np.ndarray
    depth_m: np.ndarray
    intrinsics: CameraIntrinsics
    stamp: float
    drone_pose: Optional["DronePose"] = None
    imu: Optional["ImuSnapshot"] = None
    flight_state: Optional[dict] = None


@dataclass
class Detection:
    """单帧目标检测结果。

    bbox / center 是图像坐标。
    right_m / forward_m 是机体系水平坐标。
    """

    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]
    center: Tuple[int, int]
    depth_m: float
    right_m: float
    forward_m: float


@dataclass
class TrackView:
    """跟踪器对外输出的只读视图。

    missed_frames > 0 表示该目标当前处于 YOLO 漏检后的卡尔曼预测状态。
    在 FLIGHT 模式且无人机位姿有效时，right_m/forward_m 存放世界坐标 x/y。
    DEBUG 或无位姿兜底时仍存放机体系 right/forward。
    """

    track_id: int
    label: str
    confidence: float
    bbox: Optional[Tuple[int, int, int, int]]
    center: Optional[Tuple[int, int]]
    right_m: float
    forward_m: float
    v_right_mps: float
    v_forward_mps: float
    missed_frames: int
    trajectory: List[Tuple[float, float]]


@dataclass
class CandidateCost:
    """候选安全点的代价分量，主要用于调试和可视化。"""

    total: float
    point_clearance_cost: float
    corridor_clearance_cost: float
    continuity_cost: float
    forward_cost: float
    lateral_cost: float
    min_point_clearance_m: float
    min_corridor_clearance_m: float


@dataclass
class WaypointTarget:
    """视觉程序最终输出的航点目标。

    DEBUG 模式：
        x = safe_point_body.right_m
        y = safe_point_body.forward_m
        z = 0
        frame_id = "base_link"

    FLIGHT 模式：
        x/y/z 是转换到 ROS local/world 坐标系后的航点。
        frame_id 通常是 map 或 odom。
    """

    valid: bool
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    frame_id: str = WAYPOINT_FRAME_ID
    reason: str = ""
    safe_point_body: Optional[Tuple[float, float]] = None
    candidate_cost: Optional[CandidateCost] = None


@dataclass
class ImuStabilizationStatus:
    """IMU 防抖状态。"""

    active: bool = False
    shaken: bool = False
    roll_delta: float = 0.0
    pitch_delta: float = 0.0
    yaw_delta: float = 0.0
    angular_rate: float = 0.0
    age: float = 0.0
    reason: str = "imu disabled"


@dataclass
class DronePose:
    """无人机当前位姿，仅真实飞行模式使用。"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    valid: bool = False


@dataclass
class ImuSnapshot:
    """IMU 快照，仅真实飞行模式使用。"""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    stamp: float = 0.0
    valid: bool = False


@dataclass
class LocalMapFrame:
    """视觉内部局部 map 坐标系。

    origin_x/origin_y 是 MAVROS/Gazebo 原始世界坐标里的地面投影点。
    这个类只做视觉内部坐标转换，不改变无人机自己的世界坐标源。
    """

    origin_x: float = 0.0
    origin_y: float = 0.0
    valid: bool = False
    rebase_count: int = 0

    def set_from_pose(self, pose: DronePose) -> None:
        self.origin_x = float(pose.x)
        self.origin_y = float(pose.y)
        self.valid = True

    def world_to_local(self, world_xy: Tuple[float, float]) -> Tuple[float, float]:
        return float(world_xy[0] - self.origin_x), float(world_xy[1] - self.origin_y)

    def local_to_world(self, local_xy: Tuple[float, float]) -> Tuple[float, float]:
        return float(local_xy[0] + self.origin_x), float(local_xy[1] + self.origin_y)

    def drone_local(self, pose: DronePose) -> Tuple[float, float]:
        return self.world_to_local((pose.x, pose.y))


# =========================================================
# 通用数学工具
# =========================================================
def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]。"""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """四元数转 roll/pitch/yaw，单位弧度。"""

    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return roll, pitch, yaw


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """只包含 yaw 的水平姿态四元数。"""

    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def manhattan_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def world_point_to_body(
    world_xy: Tuple[float, float],
    drone_pose: DronePose,
) -> Tuple[float, float]:
    dx = world_xy[0] - drone_pose.x
    dy = world_xy[1] - drone_pose.y
    cos_y = math.cos(drone_pose.yaw)
    sin_y = math.sin(drone_pose.yaw)

    if WAYPOINT_YAW_CONVENTION == "ENU_Y_FORWARD":
        forward_m = dx * sin_y + dy * cos_y
        right_m = dx * cos_y - dy * sin_y
    else:
        forward_m = dx * cos_y + dy * sin_y
        right_m = dx * sin_y - dy * cos_y
    return right_m, forward_m


def body_point_to_world_xy(
    body_point: Tuple[float, float],
    drone_pose: DronePose,
) -> Tuple[float, float]:
    wx, wy, _ = body_safe_point_to_world(body_point, drone_pose)
    return wx, wy


def body_safe_point_to_world(
    safe_point_body: Tuple[float, float],
    drone_pose: DronePose,
) -> Tuple[float, float, float]:
    """把机体系安全点转换成 ROS local/world 航点。

    safe_point_body = (right_m, forward_m)

    标准 ROS ENU 约定下：
        yaw=0 时，机头朝 world +X
        yaw=pi/2 时，机头朝 world +Y
        right_m 为无人机右侧，等价于机体 FRD 的 +Y

    如果你的工程里 yaw=0 表示朝 world +Y，可把 WAYPOINT_YAW_CONVENTION 改成
    "ENU_Y_FORWARD"，兼容原始代码中的转换习惯。
    """

    right_m, forward_m = safe_point_body
    cos_y = math.cos(drone_pose.yaw)
    sin_y = math.sin(drone_pose.yaw)

    if WAYPOINT_YAW_CONVENTION == "ENU_Y_FORWARD":
        # 兼容旧工程：yaw=0 时，前方映射到 world +Y，右方映射到 world +X。
        world_x = drone_pose.x + forward_m * sin_y + right_m * cos_y
        world_y = drone_pose.y + forward_m * cos_y - right_m * sin_y
    else:
        # 标准 ROS ENU：yaw=0 时，前方映射到 world +X，右方映射到 world -Y。
        world_x = drone_pose.x + forward_m * cos_y + right_m * sin_y
        world_y = drone_pose.y + forward_m * sin_y - right_m * cos_y

    if WAYPOINT_Z_MODE == "FIXED":
        world_z = FIXED_WAYPOINT_Z_M
    else:
        world_z = drone_pose.z

    return world_x, world_y, world_z


# =========================================================
# 输入源：RealSense 实时相机 / RealSense .bag
# =========================================================
class RealSenseSource:
    """RealSense 输入源。

    bag_path 为 None 时读取实时相机；否则读取 RealSense .bag。
    调试模式建议使用 .bag，真实飞行模式使用实时相机。
    """

    def __init__(self, bag_path: Optional[str]) -> None:
        if rs is None:
            raise RuntimeError("未安装 pyrealsense2，无法读取 RealSense 相机或 .bag 文件。")

        self.bag_path = bag_path
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        if bag_path:
            bag_file = Path(bag_path)
            if not bag_file.exists():
                raise FileNotFoundError(f"RealSense .bag 文件不存在：{bag_file}")
            self.config.enable_device_from_file(str(bag_file), repeat_playback=DEBUG_BAG_LOOP_PLAY)
        else:
            self.config.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.bgr8, CAMERA_FPS)
            self.config.enable_stream(rs.stream.depth, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.z16, CAMERA_FPS)

        self.profile = self.pipeline.start(self.config)

        if bag_path:
            # 离线包不按真实时间播放，避免算法处理慢时丢帧。
            self.profile.get_device().as_playback().set_real_time(False)

        self.align = rs.align(rs.stream.color)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())

        color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        rs_intr = color_stream.get_intrinsics()
        self.intrinsics = CameraIntrinsics(
            width=int(rs_intr.width),
            height=int(rs_intr.height),
            fx=float(rs_intr.fx),
            fy=float(rs_intr.fy),
            ppx=float(rs_intr.ppx),
            ppy=float(rs_intr.ppy),
        )

    def read(self) -> Optional[FramePacket]:
        """读取一帧。返回 None 表示离线包结束。"""

        try:
            frames = self.pipeline.wait_for_frames()
        except RuntimeError:
            return None

        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None

        color_bgr = np.asanyarray(color_frame.get_data())
        depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self.depth_scale
        return FramePacket(color_bgr=color_bgr, depth_m=depth_m, intrinsics=self.intrinsics, stamp=time.time())

    def close(self) -> None:
        self.pipeline.stop()


# =========================================================
# 输入源：ROS2 .db3 离线包
# =========================================================
class Ros2Db3Source:
    """ROS2 rosbag2 .db3 离线输入源。

    重要说明：
    - 本类不会启动 ROS2 节点，也不会发布/订阅话题。
    - 但是读取 .db3 需要 ROS2 Python 依赖，例如 rosbag2_py、rosidl_runtime_py、rclpy。
    - Windows 如果没有完整 ROS2 Python 环境，建议用 RealSense .bag 调试。
    """

    def __init__(self, bag_path: str) -> None:
        self.bag_path = Path(bag_path)
        if self.bag_path.suffix == ".db3":
            # rosbag2_py 通常需要 bag 目录作为 uri。
            self.bag_uri = self.bag_path.parent
        else:
            self.bag_uri = self.bag_path

        if not self.bag_uri.exists():
            raise FileNotFoundError(f"ROS2 bag 路径不存在：{self.bag_uri}")

        try:
            import rosbag2_py
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
        except ImportError as exc:
            raise RuntimeError(
                "读取 ROS2 .db3 需要 ROS2 Python 环境。"
                "如果你在 Windows 上没有 ROS2，建议改用 RealSense .bag。"
            ) from exc

        self.rosbag2_py = rosbag2_py
        self.deserialize_message = deserialize_message
        self.get_message = get_message
        self.reader = None
        self.topic_types: dict[str, str] = {}
        self.last_color_bgr: Optional[np.ndarray] = None
        self.last_depth_m: Optional[np.ndarray] = None
        self.intrinsics: Optional[CameraIntrinsics] = None
        self._open_reader()

    def _open_reader(self) -> None:
        storage_options = self.rosbag2_py.StorageOptions(uri=str(self.bag_uri), storage_id="sqlite3")
        converter_options = self.rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
        self.reader = self.rosbag2_py.SequentialReader()
        self.reader.open(storage_options, converter_options)
        self.topic_types = {item.name: item.type for item in self.reader.get_all_topics_and_types()}

        missing = [
            topic
            for topic in [DEBUG_DB3_COLOR_TOPIC, DEBUG_DB3_DEPTH_TOPIC]
            if topic not in self.topic_types
        ]
        if missing:
            raise RuntimeError(
                "ROS2 .db3 中找不到必要图像话题："
                + ", ".join(missing)
                + "。请修改 DEBUG_DB3_COLOR_TOPIC / DEBUG_DB3_DEPTH_TOPIC。"
            )

        if DEBUG_DB3_CAMERA_INFO_TOPIC not in self.topic_types and DEBUG_DB3_MANUAL_FX <= 0:
            raise RuntimeError(
                "ROS2 .db3 中没有 CameraInfo，且没有手动填写内参。"
                "请修改 DEBUG_DB3_CAMERA_INFO_TOPIC 或填写 DEBUG_DB3_MANUAL_FX/FY/PPX/PPY。"
            )

        if DEBUG_DB3_MANUAL_FX > 0:
            self.intrinsics = CameraIntrinsics(
                width=DEBUG_DB3_MANUAL_WIDTH,
                height=DEBUG_DB3_MANUAL_HEIGHT,
                fx=DEBUG_DB3_MANUAL_FX,
                fy=DEBUG_DB3_MANUAL_FY,
                ppx=DEBUG_DB3_MANUAL_PPX,
                ppy=DEBUG_DB3_MANUAL_PPY,
            )

    def _deserialize(self, topic: str, data: bytes):
        msg_type_name = self.topic_types[topic]
        msg_type = self.get_message(msg_type_name)
        return self.deserialize_message(data, msg_type)

    @staticmethod
    def _image_to_bgr(msg) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding not in {"bgr8", "rgb8", "mono8"}:
            raise RuntimeError(f"暂不支持的彩色图编码：{msg.encoding}")

        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)

        if encoding == "mono8":
            gray = raw[:, : msg.width].copy()
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        color = raw[:, : msg.width * 3].reshape(msg.height, msg.width, 3).copy()
        if encoding == "rgb8":
            color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        return color

    @staticmethod
    def _image_to_depth_m(msg) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding in {"16uc1", "mono16"}:
            row_values = msg.step // np.dtype(np.uint16).itemsize
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, row_values)
            return raw[:, : msg.width].astype(np.float32) * DEBUG_DB3_DEPTH_SCALE_M

        if encoding == "32fc1":
            row_values = msg.step // np.dtype(np.float32).itemsize
            raw = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, row_values)
            return raw[:, : msg.width].astype(np.float32)

        raise RuntimeError(f"暂不支持的深度图编码：{msg.encoding}")

    @staticmethod
    def _camera_info_to_intrinsics(msg) -> CameraIntrinsics:
        return CameraIntrinsics(
            width=int(msg.width),
            height=int(msg.height),
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            ppx=float(msg.k[2]),
            ppy=float(msg.k[5]),
        )

    def read(self) -> Optional[FramePacket]:
        """从 .db3 中顺序读取，凑齐最近的一帧 color + depth 后返回。"""

        assert self.reader is not None

        while True:
            if not self.reader.has_next():
                if DEBUG_DB3_LOOP_PLAY:
                    self._open_reader()
                    continue
                return None

            topic, data, stamp_ns = self.reader.read_next()

            if topic not in {
                DEBUG_DB3_COLOR_TOPIC,
                DEBUG_DB3_DEPTH_TOPIC,
                DEBUG_DB3_CAMERA_INFO_TOPIC,
            }:
                continue

            msg = self._deserialize(topic, data)

            if topic == DEBUG_DB3_CAMERA_INFO_TOPIC:
                self.intrinsics = self._camera_info_to_intrinsics(msg)
                continue

            if topic == DEBUG_DB3_COLOR_TOPIC:
                self.last_color_bgr = self._image_to_bgr(msg)

            elif topic == DEBUG_DB3_DEPTH_TOPIC:
                self.last_depth_m = self._image_to_depth_m(msg)

            if self.last_color_bgr is not None and self.last_depth_m is not None and self.intrinsics is not None:
                return FramePacket(
                    color_bgr=self.last_color_bgr,
                    depth_m=self.last_depth_m,
                    intrinsics=self.intrinsics,
                    stamp=float(stamp_ns) * 1e-9,
                )

    def close(self) -> None:
        self.reader = None


class RecordedFlightDb3Source:
    """读取本项目记录器生成的 db3 + frames_sync.csv + metadata.json。

    这种记录目录不是标准 rosbag2 目录：没有 metadata.yaml，CameraInfo 也保存为
    std_msgs/String。因此不能直接交给 rosbag2_py，需要直接读 sqlite3 表，并用
    frames_sync.csv 把每个图像帧和飞控状态对齐。
    """

    COLOR_TOPIC = "/device_0/sensor_1/Color_0/image/data"
    DEPTH_TOPIC = "/device_0/sensor_0/Depth_0/image/data"

    def __init__(self, record_path: str) -> None:
        self.record_dir = Path(record_path)
        if self.record_dir.suffix == ".db3":
            self.record_dir = self.record_dir.parent
        if not self.record_dir.exists():
            raise FileNotFoundError(f"记录目录不存在：{self.record_dir}")

        self.metadata = self._load_metadata()
        db_name = self.metadata.get("db3_file") or f"{self.record_dir.name}.db3"
        self.db_path = self.record_dir / db_name
        if not self.db_path.exists():
            db_files = sorted(self.record_dir.glob("*.db3"))
            if not db_files:
                raise FileNotFoundError(f"记录目录中找不到 .db3：{self.record_dir}")
            self.db_path = db_files[0]

        self.depth_scale_m = float(
            self.metadata.get("realsense", {}).get("depth_scale_m_per_unit", DEBUG_DB3_DEPTH_SCALE_M)
        )
        color_intr = self.metadata.get("realsense", {}).get("color_intrinsics", {})
        self.intrinsics = CameraIntrinsics(
            width=int(color_intr.get("width", DEBUG_DB3_MANUAL_WIDTH)),
            height=int(color_intr.get("height", DEBUG_DB3_MANUAL_HEIGHT)),
            fx=float(color_intr.get("fx", DEBUG_DB3_MANUAL_FX or 1.0)),
            fy=float(color_intr.get("fy", DEBUG_DB3_MANUAL_FY or 1.0)),
            ppx=float(color_intr.get("ppx", DEBUG_DB3_MANUAL_PPX)),
            ppy=float(color_intr.get("ppy", DEBUG_DB3_MANUAL_PPY)),
        )

        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image

        self.deserialize_message = deserialize_message
        self.Image = Image
        self.conn = sqlite3.connect(str(self.db_path))
        self.topic_ids = self._load_topic_ids()
        self.color_rows = self._load_image_rows(self.COLOR_TOPIC)
        self.depth_rows = self._load_image_rows(self.DEPTH_TOPIC)
        self.frames = self._load_frames()
        self.index = min(DEBUG_START_FRAME, max(len(self.frames) - 1, 0))

        if not self.color_rows:
            raise RuntimeError(f"db3 中没有彩色图像：{self.COLOR_TOPIC}")
        if not self.depth_rows:
            raise RuntimeError(f"db3 中没有深度图像：{self.DEPTH_TOPIC}")
        if not self.frames:
            raise RuntimeError(f"frames_sync.csv 没有可用帧：{self.record_dir}")

        print(
            f"📼 离线记录：{self.record_dir.name}, "
            f"frames={len(self.frames)}, color={len(self.color_rows)}, depth={len(self.depth_rows)}, "
            f"start_frame={self.index}"
        )

    def _load_metadata(self) -> dict:
        path = self.record_dir / "metadata.json"
        if not path.exists():
            return {}
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)

    def _load_topic_ids(self) -> dict[str, int]:
        rows = self.conn.execute("select id, name from topics").fetchall()
        return {str(name): int(topic_id) for topic_id, name in rows}

    def _load_image_rows(self, topic: str) -> list[Tuple[int, int, int]]:
        topic_id = self.topic_ids.get(topic)
        if topic_id is None:
            return []
        return [
            (int(msg_id), int(ts), int(topic_id))
            for msg_id, ts in self.conn.execute(
            "select id, timestamp from messages where topic_id=? order by id",
            (topic_id,),
            ).fetchall()
        ]

    def _read_image_msg(self, rows: list[Tuple[int, int, int]], index: int):
        msg_id, _, _ = rows[index]
        data = self.conn.execute("select data from messages where id=?", (msg_id,)).fetchone()[0]
        return self.deserialize_message(data, self.Image)

    def _load_frames(self) -> list[dict]:
        path = self.record_dir / "frames_sync.csv"
        if not path.exists():
            return []
        text = path.read_text(encoding="utf-8", errors="replace").replace("\x00", "")
        return list(csv.DictReader(text.splitlines()))

    @staticmethod
    def _to_float(row: dict, key: str, default: float = 0.0) -> float:
        value = row.get(key, "")
        if value is None or value == "":
            return default
        try:
            return float(value)
        except ValueError:
            return default

    @staticmethod
    def _to_bool(row: dict, key: str) -> bool:
        return str(row.get(key, "")).strip().lower() in {"1", "true", "yes", "armed"}

    @staticmethod
    def _msg_to_bgr(msg) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding not in {"bgr8", "rgb8", "mono8"}:
            raise RuntimeError(f"暂不支持的彩色图编码：{msg.encoding}")
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        if encoding == "mono8":
            gray = raw[:, : msg.width].copy()
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        color = raw[:, : msg.width * 3].reshape(msg.height, msg.width, 3).copy()
        if encoding == "rgb8":
            color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
        return color

    def _msg_to_depth_m(self, msg) -> np.ndarray:
        encoding = msg.encoding.lower()
        if encoding in {"16uc1", "mono16"}:
            row_values = msg.step // np.dtype(np.uint16).itemsize
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, row_values)
            return raw[:, : msg.width].astype(np.float32) * self.depth_scale_m
        if encoding == "32fc1":
            row_values = msg.step // np.dtype(np.float32).itemsize
            raw = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, row_values)
            return raw[:, : msg.width].astype(np.float32)
        raise RuntimeError(f"暂不支持的深度图编码：{msg.encoding}")

    def _frame_state(self, row: dict) -> Tuple[DronePose, ImuSnapshot, dict]:
        north = self._to_float(row, "local_x_north_m")
        east = self._to_float(row, "local_y_east_m")
        down = self._to_float(row, "local_z_down_m")
        yaw = self._to_float(row, "yaw_rad")
        stamp = self._to_float(row, "frame_host_unix_ns") * 1e-9
        pose = DronePose(x=north, y=east, z=-down, yaw=yaw, valid=True)
        imu = ImuSnapshot(
            roll=self._to_float(row, "roll_rad"),
            pitch=self._to_float(row, "pitch_rad"),
            yaw=yaw,
            stamp=stamp,
            valid=True,
        )
        state = {
            "frame_idx": int(float(row.get("frame_idx", 0) or 0)),
            "mode": row.get("mode", ""),
            "armed": self._to_bool(row, "armed"),
            "system_status": row.get("system_status", ""),
            "lat_deg": self._to_float(row, "lat_deg"),
            "lon_deg": self._to_float(row, "lon_deg"),
            "relative_alt_m": self._to_float(row, "relative_alt_m"),
            "heading_deg": self._to_float(row, "heading_deg"),
            "gps_fix_name": row.get("gps_fix_name", ""),
            "satellites_visible": self._to_float(row, "satellites_visible"),
            "local_vx_north_mps": self._to_float(row, "local_vx_north_mps"),
            "local_vy_east_mps": self._to_float(row, "local_vy_east_mps"),
            "local_vz_down_mps": self._to_float(row, "local_vz_down_mps"),
        }
        return pose, imu, state

    def read(self) -> Optional[FramePacket]:
        if self.index >= len(self.frames):
            if DEBUG_DB3_LOOP_PLAY:
                self.index = 0
            else:
                return None

        row = self.frames[self.index]
        self.index += 1
        color_idx = max(0, int(float(row.get("color_frame_number", self.index) or self.index)) - 1)
        depth_idx = max(0, int(float(row.get("depth_frame_number", self.index) or self.index)) - 1)
        color_idx = min(color_idx, len(self.color_rows) - 1)
        depth_idx = min(depth_idx, len(self.depth_rows) - 1)

        color_msg = self._read_image_msg(self.color_rows, color_idx)
        depth_msg = self._read_image_msg(self.depth_rows, depth_idx)
        pose, imu, state = self._frame_state(row)
        stamp = self._to_float(row, "frame_host_unix_ns") * 1e-9
        if stamp <= 0:
            stamp = time.time()

        return FramePacket(
            color_bgr=self._msg_to_bgr(color_msg),
            depth_m=self._msg_to_depth_m(depth_msg),
            intrinsics=self.intrinsics,
            stamp=stamp,
            drone_pose=pose,
            imu=imu,
            flight_state=state,
        )

    def close(self) -> None:
        self.conn.close()


# =========================================================
# 真实飞行 ROS2 接口
# =========================================================
class FlightRosInterface:
    """真实飞行模式下的 ROS2 接口。

    负责：
    - 启动节点
    - 订阅无人机位姿
    - 订阅 IMU
    - 发布视觉航点
    - 发布调试状态

    注意：调试模式不会实例化这个类，因此不会启用 ROS。
    """

    def __init__(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg import Imu
            from std_msgs.msg import String
            from rclpy.qos import qos_profile_sensor_data
        except ImportError as exc:
            raise RuntimeError("真实飞行模式需要 ROS2 Python 环境：rclpy / geometry_msgs / sensor_msgs / std_msgs。") from exc

        self.rclpy = rclpy
        self.PoseStamped = PoseStamped
        self.String = String
        self.drone_pose = DronePose(valid=False)
        self.imu = ImuSnapshot(valid=False)
        self.gz_pose = DronePose(valid=False)

        self.rclpy.init()
        self.node = self.rclpy.create_node(FLIGHT_NODE_NAME)

        self.node.create_subscription(PoseStamped, FLIGHT_POSE_TOPIC, self._pose_callback, qos_profile_sensor_data)
        self.node.create_subscription(Imu, FLIGHT_IMU_TOPIC, self._imu_callback, qos_profile_sensor_data)
        self.waypoint_pub = self.node.create_publisher(PoseStamped, WAYPOINT_TOPIC, 10)
        self.status_pub = self.node.create_publisher(String, STATUS_TOPIC, 10)
        self.gz_node = None
        if ENABLE_GAZEBO_POSE_FALLBACK:
            self._init_gazebo_pose_fallback()

        print(f"✅ ROS2 节点已启动：{FLIGHT_NODE_NAME}")
        print(f"   订阅位姿：{FLIGHT_POSE_TOPIC}")
        print(f"   订阅 IMU ：{FLIGHT_IMU_TOPIC}")
        print(f"   发布航点：{WAYPOINT_TOPIC}")
        print(f"   发布状态：{STATUS_TOPIC}")

    def _init_gazebo_pose_fallback(self) -> None:
        try:
            from gz.msgs10.pose_v_pb2 import Pose_V
            from gz.transport13 import Node as GzNode
        except Exception as exc:
            self.node.get_logger().warning(f"Gazebo pose fallback unavailable: {exc}")
            return
        self.gz_node = GzNode()
        topic = f"/world/{GAZEBO_WORLD_NAME}/pose/info"
        self.gz_node.subscribe(Pose_V, topic, self._gz_pose_callback)
        self.node.get_logger().info(f"Gazebo pose fallback subscribed: {topic} model={GAZEBO_MODEL_NAME}")

    def _gz_pose_callback(self, msg) -> None:
        for pose in msg.pose:
            if pose.name != GAZEBO_MODEL_NAME:
                continue
            q = pose.orientation
            roll, pitch, yaw = quat_to_euler(float(q.x), float(q.y), float(q.z), float(q.w))
            self.gz_pose = DronePose(
                x=float(pose.position.x),
                y=float(pose.position.y),
                z=float(pose.position.z),
                roll=float(roll),
                pitch=float(pitch),
                yaw=float(yaw),
                valid=True,
            )
            return

    def _pose_callback(self, msg) -> None:
        q = msg.pose.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        self.drone_pose = DronePose(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(msg.pose.position.z),
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
            valid=True,
        )

    def _imu_callback(self, msg) -> None:
        q = msg.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        self.imu = ImuSnapshot(
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            wx=float(msg.angular_velocity.x),
            wy=float(msg.angular_velocity.y),
            wz=float(msg.angular_velocity.z),
            ax=float(msg.linear_acceleration.x),
            ay=float(msg.linear_acceleration.y),
            az=float(msg.linear_acceleration.z),
            stamp=time.time(),
            valid=True,
        )

    def spin_once(self) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=0.001)

    def get_imu_snapshot(self) -> Optional[ImuSnapshot]:
        return self.imu if self.imu.valid else None

    def get_drone_pose(self) -> Optional[DronePose]:
        if self.drone_pose.valid:
            return self.drone_pose
        if self.gz_pose.valid:
            return self.gz_pose
        return None

    def publish_waypoint(self, waypoint: WaypointTarget) -> None:
        """发布航点和状态。

        valid=False 时只发布状态，不发布 PoseStamped 航点。
        这样状态机可以知道视觉暂时找不到安全航点，但不会收到坏航点。
        """

        status = {
            "valid": waypoint.valid,
            "frame_id": waypoint.frame_id,
            "x": waypoint.x,
            "y": waypoint.y,
            "z": waypoint.z,
            "reason": waypoint.reason,
        }

        if waypoint.safe_point_body is not None:
            status["safe_local_x_m"] = waypoint.safe_point_body[0]
            status["safe_local_y_m"] = waypoint.safe_point_body[1]
            # 兼容旧状态机/日志字段名；仿真局部 map 模式下这里实际表示 safe_local。
            status["safe_body_right_m"] = waypoint.safe_point_body[0]
            status["safe_body_forward_m"] = waypoint.safe_point_body[1]

        if waypoint.candidate_cost is not None:
            cost = waypoint.candidate_cost
            status["cost_total"] = cost.total
            status["min_point_clearance_m"] = cost.min_point_clearance_m
            status["min_corridor_clearance_m"] = cost.min_corridor_clearance_m

        status_msg = self.String()
        status_msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(status_msg)

        if not waypoint.valid:
            return

        current_pose = self.get_drone_pose()
        msg = self.PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = waypoint.frame_id
        msg.pose.position.x = float(waypoint.x)
        msg.pose.position.y = float(waypoint.y)
        msg.pose.position.z = float(waypoint.z)

        # 航点姿态默认保持当前 yaw。状态机如不需要姿态，可以忽略 orientation。
        yaw = current_pose.yaw if current_pose is not None and current_pose.valid else 0.0
        _, _, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.waypoint_pub.publish(msg)

    def close(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class RosRgbdSource:
    """D435i 风格 RGB-D 输入源。

    真实机载电脑和仿真都应提供同一组 ROS2 话题：
    color image、depth image、color camera_info。
    """

    def __init__(self, ros: FlightRosInterface) -> None:
        from cv_bridge import CvBridge
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image

        self.ros = ros
        self.bridge = CvBridge()
        self.color_bgr: Optional[np.ndarray] = None
        self.depth_m: Optional[np.ndarray] = None
        self.intrinsics: Optional[CameraIntrinsics] = None
        self.stamp = time.time()
        self.last_wait_log = 0.0
        self.color_seq = 0
        self.depth_seq = 0
        self.last_returned_pair: Optional[Tuple[int, int]] = None

        ros.node.create_subscription(Image, RGBD_COLOR_TOPIC, self._color_cb, qos_profile_sensor_data)
        ros.node.create_subscription(Image, RGBD_DEPTH_TOPIC, self._depth_cb, qos_profile_sensor_data)
        ros.node.create_subscription(CameraInfo, RGBD_CAMERA_INFO_TOPIC, self._info_cb, qos_profile_sensor_data)
        print(f"✅ RGB-D 订阅：{RGBD_COLOR_TOPIC}, {RGBD_DEPTH_TOPIC}, {RGBD_CAMERA_INFO_TOPIC}")

    def _color_cb(self, msg) -> None:
        if msg.encoding.lower() == "rgb8":
            self.color_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        else:
            self.color_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.stamp = time.time()
        self.color_seq += 1

    def _depth_cb(self, msg) -> None:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth.dtype == np.uint16:
            self.depth_m = depth.astype(np.float32) * DEBUG_DB3_DEPTH_SCALE_M
        else:
            self.depth_m = depth.astype(np.float32)
        self.stamp = time.time()
        self.depth_seq += 1

    def _info_cb(self, msg) -> None:
        self.intrinsics = CameraIntrinsics(
            width=int(msg.width),
            height=int(msg.height),
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            ppx=float(msg.k[2]),
            ppy=float(msg.k[5]),
        )

    def read(self) -> Optional[FramePacket]:
        while (
            self.color_bgr is None
            or self.depth_m is None
            or self.intrinsics is None
        ) and self.ros.rclpy.ok():
            now = time.monotonic()
            if now - self.last_wait_log >= 2.0:
                waiting = []
                if self.color_bgr is None:
                    waiting.append("color")
                if self.depth_m is None:
                    waiting.append("depth")
                if self.intrinsics is None:
                    waiting.append("camera_info")
                self.ros.node.get_logger().info(f"Waiting for RGB-D: {', '.join(waiting)}")
                self.last_wait_log = now
            self.ros.spin_once()
            time.sleep(0.005)

        while self.ros.rclpy.ok():
            self.ros.spin_once()
            pair = (self.color_seq, self.depth_seq)
            if pair != self.last_returned_pair:
                self.last_returned_pair = pair
                break
            time.sleep(0.001)

        pose = self.ros.drone_pose if self.ros.drone_pose.valid else None
        imu = self.ros.imu if self.ros.imu.valid else None
        return FramePacket(
            color_bgr=self.color_bgr.copy(),
            depth_m=self.depth_m.copy(),
            intrinsics=self.intrinsics,
            stamp=time.time(),
            drone_pose=pose,
            imu=imu,
            flight_state=None,
        )

    def close(self) -> None:
        pass


# =========================================================
# IMU 图像防抖
# =========================================================
class ImuFrameStabilizer:
    """基于 IMU 姿态低通参考的图像/深度防抖器。

    调试模式默认关闭。
    真实飞行模式下，当 IMU 数据有效时，对 RGB 和深度图做同一个单应变换。
    如果角速度过大，会设置 shaken=True，主循环可以跳过本帧 YOLO 更新，让卡尔曼继续预测。
    """

    def __init__(self, intrinsics: CameraIntrinsics) -> None:
        self.enabled = RUN_MODE == "FLIGHT" and ENABLE_IMU_STABILIZATION
        self.alpha = max(0.0, min(0.999, IMU_FILTER_ALPHA))
        self.max_age = max(0.01, IMU_MAX_AGE_S)
        self.shake_rate_threshold = max(0.0, IMU_SHAKE_RATE_THRESHOLD_RAD_S)
        self.correction_limit_rad = math.radians(max(0.0, IMU_CORRECTION_LIMIT_DEG))
        self._ref_roll: Optional[float] = None
        self._ref_pitch: Optional[float] = None
        self._ref_yaw: Optional[float] = None

        self._k = np.array(
            [
                [intrinsics.fx, 0.0, intrinsics.ppx],
                [0.0, intrinsics.fy, intrinsics.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self._k_inv = np.linalg.inv(self._k)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    @staticmethod
    def _rotation_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)
        rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
        rot_y = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
        rot_z = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        return rot_z @ rot_y @ rot_x

    def _update_reference(self, current: float, reference: Optional[float]) -> float:
        if reference is None:
            return current
        delta = normalize_angle(current - reference)
        return normalize_angle(reference + (1.0 - self.alpha) * delta)

    def update(self, imu: Optional[ImuSnapshot], now: float) -> ImuStabilizationStatus:
        if not self.enabled:
            return ImuStabilizationStatus(reason="imu disabled")
        if imu is None or not imu.valid:
            return ImuStabilizationStatus(reason="waiting imu")

        age = max(0.0, now - float(imu.stamp))
        if age > self.max_age:
            return ImuStabilizationStatus(age=age, reason="imu stale")

        if self._ref_roll is None or self._ref_pitch is None or self._ref_yaw is None:
            self._ref_roll, self._ref_pitch, self._ref_yaw = imu.roll, imu.pitch, imu.yaw
            return ImuStabilizationStatus(active=True, age=age, reason="imu reference init")

        self._ref_roll = self._update_reference(imu.roll, self._ref_roll)
        self._ref_pitch = self._update_reference(imu.pitch, self._ref_pitch)
        self._ref_yaw = self._update_reference(imu.yaw, self._ref_yaw)

        roll_delta = self._clamp(normalize_angle(imu.roll - self._ref_roll), self.correction_limit_rad)
        pitch_delta = self._clamp(normalize_angle(imu.pitch - self._ref_pitch), self.correction_limit_rad)
        yaw_delta = self._clamp(normalize_angle(imu.yaw - self._ref_yaw), self.correction_limit_rad)
        angular_rate = float(np.linalg.norm([imu.wx, imu.wy, imu.wz]))
        shaken = angular_rate >= self.shake_rate_threshold if self.shake_rate_threshold > 0 else False

        return ImuStabilizationStatus(
            active=True,
            shaken=shaken,
            roll_delta=roll_delta,
            pitch_delta=pitch_delta,
            yaw_delta=yaw_delta,
            angular_rate=angular_rate,
            age=age,
            reason="imu shake gate" if shaken else "imu stable",
        )

    def homography(self, status: ImuStabilizationStatus) -> Optional[np.ndarray]:
        if not status.active:
            return None

        # D435i 前视安装的近似约定：
        # pitch 主要影响图像上下，yaw 主要影响图像左右，roll 主要是图像旋转。
        rx = -status.pitch_delta * IMU_PITCH_GAIN
        ry = -status.yaw_delta * IMU_YAW_GAIN
        rz = -status.roll_delta * IMU_ROLL_GAIN
        if max(abs(rx), abs(ry), abs(rz)) < 1e-5:
            return None

        rotation = self._rotation_matrix(rx, ry, rz)
        h = self._k @ rotation @ self._k_inv
        return h / h[2, 2]

    def stabilize(
        self,
        color_bgr: np.ndarray,
        depth_m: np.ndarray,
        imu: Optional[ImuSnapshot],
        now: float,
    ) -> Tuple[np.ndarray, np.ndarray, ImuStabilizationStatus]:
        status = self.update(imu, now)
        h = self.homography(status)
        if h is None:
            return color_bgr, depth_m, status

        height, width = color_bgr.shape[:2]
        stable_color = cv2.warpPerspective(
            color_bgr,
            h,
            (width, height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        )
        stable_depth = cv2.warpPerspective(
            depth_m,
            h,
            (width, height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        return stable_color, stable_depth, status


# =========================================================
# 深度与 YOLO 检测
# =========================================================
def normalize_class_filter(classes: Optional[Iterable[str]]) -> Optional[set[str]]:
    """把类别过滤列表标准化成小写字符串集合。"""

    if not classes:
        return None
    return {str(item).strip().lower() for item in classes if str(item).strip()}


def clamp_bbox(
    bbox: Tuple[int, int, int, int],
    width: int,
    height: int,
) -> Optional[Tuple[int, int, int, int]]:
    """把 YOLO bbox 裁剪到图像范围内，非法框返回 None。"""

    x1, y1, x2, y2 = bbox
    x1 = max(0, min(width - 1, x1))
    x2 = max(0, min(width, x2))
    y1 = max(0, min(height - 1, y1))
    y2 = max(0, min(height, y2))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def median_depth_in_bbox(
    depth_m: np.ndarray,
    bbox: Tuple[int, int, int, int],
) -> Optional[float]:
    """计算检测框内目标深度。

    优先取 bbox 中心区域，减少背景深度干扰。
    如果中心区域没有有效深度，再退回使用整个 bbox。
    """

    x1, y1, x2, y2 = bbox
    box_w, box_h = x2 - x1, y2 - y1
    margin_x = int(box_w * DEPTH_CROP_MARGIN_RATIO)
    margin_y = int(box_h * DEPTH_CROP_MARGIN_RATIO)

    roi_list = [
        depth_m[y1 + margin_y : y2 - margin_y, x1 + margin_x : x2 - margin_x],
        depth_m[y1:y2, x1:x2],
    ]

    for roi in roi_list:
        if roi.size == 0:
            continue
        valid = roi[np.isfinite(roi) & (roi >= MIN_DEPTH_M) & (roi <= MAX_DEPTH_M)]
        if valid.size:
            return float(np.median(valid))
    return None


def deproject_pixel_to_body(
    intrinsics: CameraIntrinsics,
    center: Tuple[int, int],
    depth_m: float,
) -> Tuple[float, float]:
    """像素 + 深度反投影到机体系水平坐标。

    本数据集里 D435i 朝向正下方，且相机安装在无人机中部：
        图像中心 = 无人机正下方
        相机水平坐标相对无人机机体系逆时针偏航 90 度
        图像上方 = 机体前方

    OpenCV 图像坐标先得到：
        x_img_right =  (u - ppx) / fx * depth
        y_img_down  =  (v - ppy) / fy * depth

    下视安装修正后：
        body_forward = -y_img_down
        body_right   =  x_img_right
    """

    u, v = center
    x_img_right = (float(u) - intrinsics.ppx) / intrinsics.fx * depth_m
    y_img_down = (float(v) - intrinsics.ppy) / intrinsics.fy * depth_m
    right_m = x_img_right
    forward_m = -y_img_down
    return float(right_m), float(forward_m)


def run_detection(
    model: YOLO,
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    class_filter: Optional[set[str]],
    device: Optional[str] = None,
) -> List[Detection]:
    """执行 YOLO 检测，并给每个目标关联深度和空间坐标。"""

    height, width = color_bgr.shape[:2]
    results = model.predict(
        source=color_bgr,
        imgsz=YOLO_IMGSZ,
        conf=YOLO_CONF,
        iou=YOLO_IOU,
        device=device,
        verbose=False,
    )

    detections: List[Detection] = []
    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            label = str(model.names.get(cls_id, cls_id))

            if class_filter is not None:
                if label.lower() not in class_filter and str(cls_id) not in class_filter:
                    continue

            raw_bbox = tuple(map(int, box.xyxy[0].tolist()))
            bbox = clamp_bbox(raw_bbox, width, height)
            if bbox is None:
                continue

            depth = median_depth_in_bbox(depth_m, bbox)
            if depth is None:
                # 深度无效时先不送入 3D 跟踪器。
                # 后续如果需要更保守，可以扩展成 2D-only 障碍告警。
                continue

            x1, y1, x2, y2 = bbox
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            right_m, forward_m = deproject_pixel_to_body(intrinsics, center, depth)

            detections.append(
                Detection(
                    label=label,
                    confidence=float(box.conf[0]),
                    bbox=bbox,
                    center=center,
                    depth_m=depth,
                    right_m=right_m,
                    forward_m=forward_m,
                )
            )
    return detections


def detections_to_world(
    detections: List[Detection],
    drone_pose: Optional[DronePose],
) -> List[Detection]:
    """把当前帧机体系检测点转换成世界坐标，供跟踪和规划使用。"""

    if RUN_MODE != "FLIGHT" or drone_pose is None or not drone_pose.valid:
        return detections

    world_detections: List[Detection] = []
    for det in detections:
        wx, wy = body_point_to_world_xy((det.right_m, det.forward_m), drone_pose)
        world_detections.append(
            Detection(
                label=det.label,
                confidence=det.confidence,
                bbox=det.bbox,
                center=det.center,
                depth_m=det.depth_m,
                right_m=wx,
                forward_m=wy,
            )
        )
    return world_detections


def detections_to_local_map(
    detections: List[Detection],
    drone_pose: Optional[DronePose],
    local_map: Optional[LocalMapFrame],
) -> List[Detection]:
    """把当前帧机体系检测点转换成视觉内部局部 map 坐标。"""

    if (
        RUN_MODE != "FLIGHT"
        or drone_pose is None
        or not drone_pose.valid
        or local_map is None
        or not local_map.valid
    ):
        return detections

    local_detections: List[Detection] = []
    for det in detections:
        wx, wy = body_point_to_world_xy((det.right_m, det.forward_m), drone_pose)
        lx, ly = local_map.world_to_local((wx, wy))
        local_detections.append(
            Detection(
                label=det.label,
                confidence=det.confidence,
                bbox=det.bbox,
                center=det.center,
                depth_m=det.depth_m,
                right_m=lx,
                forward_m=ly,
            )
        )
    return local_detections


# =========================================================
# 卡尔曼单目标跟踪
# =========================================================
@dataclass
class KalmanTrack:
    """单目标卡尔曼跟踪器。

    状态向量：
        [right位置, forward位置, right速度, forward速度]

    特性：
    - 每帧都会 predict
    - YOLO 匹配成功才 update
    - YOLO 漏检时不删除，继续使用预测位置
    - 连续漏检超过 MAX_MISSED_FRAMES 才删除
    """

    track_id: int
    detection: Detection
    now: float
    process_var: float = DEFAULT_PROCESS_VAR
    measurement_var: float = DEFAULT_MEASUREMENT_VAR

    state: np.ndarray = field(init=False)
    covariance: np.ndarray = field(init=False)
    label: str = field(init=False)
    confidence: float = field(init=False)
    bbox: Optional[Tuple[int, int, int, int]] = field(init=False)
    center: Optional[Tuple[int, int]] = field(init=False)
    last_update: float = field(init=False)
    missed_frames: int = field(default=0, init=False)
    track_age: int = field(default=1, init=False)
    hit_count: int = field(default=1, init=False)

    def __post_init__(self) -> None:
        self.state = np.array(
            [[self.detection.right_m], [self.detection.forward_m], [0.0], [0.0]],
            dtype=float,
        )
        self.covariance = np.diag([INIT_POS_COV, INIT_POS_COV, INIT_VEL_COV, INIT_VEL_COV]).astype(float)
        self.label = self.detection.label
        self.confidence = self.detection.confidence
        self.bbox = self.detection.bbox
        self.center = self.detection.center
        self.last_update = self.now

    @staticmethod
    def transition_matrix(dt: float) -> np.ndarray:
        return np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    def process_noise_matrix(self, dt: float) -> np.ndarray:
        q = self.process_var
        dt_sq = dt * dt
        dt_cu = dt_sq * dt
        dt_qu = dt_sq * dt_sq
        return q * np.array(
            [
                [dt_qu / 4.0, 0.0, dt_cu / 2.0, 0.0],
                [0.0, dt_qu / 4.0, 0.0, dt_cu / 2.0],
                [dt_cu / 2.0, 0.0, dt_sq, 0.0],
                [0.0, dt_cu / 2.0, 0.0, dt_sq],
            ],
            dtype=float,
        )

    def predict_to(self, now: float) -> None:
        dt = max(now - self.last_update, 1e-3)
        f = self.transition_matrix(dt)
        self.state = f @ self.state
        self.covariance = f @ self.covariance @ f.T + self.process_noise_matrix(dt)
        self.last_update = now
        self.track_age += 1

    def update(self, detection: Detection) -> None:
        z = np.array([[detection.right_m], [detection.forward_m]], dtype=float)
        h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=float)
        r = np.eye(2, dtype=float) * self.measurement_var

        residual = z - h @ self.state
        s = h @ self.covariance @ h.T + r
        gain = self.covariance @ h.T @ np.linalg.inv(s)

        self.state = self.state + gain @ residual
        self.covariance = (np.eye(4, dtype=float) - gain @ h) @ self.covariance

        self.label = detection.label
        self.confidence = detection.confidence
        self.bbox = detection.bbox
        self.center = detection.center
        self.missed_frames = 0
        self.hit_count += 1

    def mark_missed(self) -> None:
        self.missed_frames += 1
        self.bbox = None
        self.center = None

    def shift_position(self, dx: float, dy: float) -> None:
        """局部 map 原点变化时平移轨迹状态，速度不变。"""

        self.state[0, 0] += dx
        self.state[1, 0] += dy

    def predict_trajectory(self, horizon_s: float, dt_s: float) -> List[Tuple[float, float]]:
        steps = max(1, int(round(horizon_s / max(dt_s, 1e-3))))
        state = self.state.copy()
        f = self.transition_matrix(dt_s)
        trajectory: List[Tuple[float, float]] = []
        for _ in range(steps):
            state = f @ state
            trajectory.append((float(state[0, 0]), float(state[1, 0])))
        return trajectory

    def as_view(self) -> TrackView:
        return TrackView(
            track_id=self.track_id,
            label=self.label,
            confidence=self.confidence,
            bbox=self.bbox,
            center=self.center,
            right_m=float(self.state[0, 0]),
            forward_m=float(self.state[1, 0]),
            v_right_mps=float(self.state[2, 0]),
            v_forward_mps=float(self.state[3, 0]),
            missed_frames=self.missed_frames,
            trajectory=self.predict_trajectory(PREDICTION_HORIZON_S, PREDICTION_DT_S),
        )


class MultiObjectTracker:
    """多目标跟踪器。

    当前使用简单、稳定、容易调试的匹配策略：
        同标签 + 距离阈值 + 贪心最近邻

    对于无人机避障，稳定和可解释性优先。
    后续如果目标密集、ID 交换严重，可以再升级匈牙利匹配。
    """

    def __init__(self) -> None:
        self.match_threshold = MATCH_DISTANCE_M
        self.max_missed_frames = MAX_MISSED_FRAMES
        self.tracks: dict[int, KalmanTrack] = {}
        self._next_id = 1

    def update(self, detections: List[Detection], now: float) -> List[KalmanTrack]:
        # 1. 所有旧轨迹先预测到当前时刻。
        for track in self.tracks.values():
            track.predict_to(now)

        unmatched_track_ids = set(self.tracks)
        unmatched_det_ids = set(range(len(detections)))
        candidates: List[Tuple[float, int, int]] = []

        # 2. 生成匹配候选。
        for track_id, track in self.tracks.items():
            tr = float(track.state[0, 0])
            tf = float(track.state[1, 0])
            for det_id, det in enumerate(detections):
                # 同一条轨迹只允许继续匹配相同 YOLO 类别的检测结果。
                if track.label != det.label:
                    continue
                dist = manhattan_distance((tr, tf), (det.right_m, det.forward_m))
                if dist <= self.match_threshold:
                    pixel_cost = 0.0
                    if track.center is not None and det.center is not None:
                        pixel_dist = abs(track.center[0] - det.center[0]) + abs(track.center[1] - det.center[1])
                        pixel_cost = min(pixel_dist / 180.0, 1.5)
                    candidates.append((dist + 0.35 * pixel_cost, track_id, det_id))

        # 3. 最近邻贪心匹配。
        for _, track_id, det_id in sorted(candidates, key=lambda x: x[0]):
            if track_id not in unmatched_track_ids or det_id not in unmatched_det_ids:
                continue
            self.tracks[track_id].update(detections[det_id])
            unmatched_track_ids.remove(track_id)
            unmatched_det_ids.remove(det_id)

        # 4. 未匹配轨迹标记为漏检。
        for track_id in unmatched_track_ids:
            self.tracks[track_id].mark_missed()

        # 5. 未匹配检测生成新轨迹。
        for det_id in unmatched_det_ids:
            det = detections[det_id]
            duplicate_existing = False
            for track in self.tracks.values():
                if track.label != det.label:
                    continue
                tr = float(track.state[0, 0])
                tf = float(track.state[1, 0])
                if manhattan_distance((tr, tf), (det.right_m, det.forward_m)) <= DUPLICATE_SPAWN_DISTANCE_M:
                    duplicate_existing = True
                    break
            if duplicate_existing:
                continue
            self.tracks[self._next_id] = KalmanTrack(self._next_id, detections[det_id], now)
            self._next_id += 1

        # 6. 删除过期轨迹。
        stale_ids = [tid for tid, track in self.tracks.items() if track.missed_frames > self.max_missed_frames]
        for tid in stale_ids:
            del self.tracks[tid]

        return list(self.tracks.values())

    def shift_all(self, dx: float, dy: float) -> None:
        """局部 map 重建原点后，把所有轨迹平移到新局部坐标系。"""

        for track in self.tracks.values():
            track.shift_position(dx, dy)


# =========================================================
# 安全航点规划
# =========================================================
def distance_point_to_segment(
    point: Tuple[float, float],
    seg_a: Tuple[float, float],
    seg_b: Tuple[float, float],
) -> float:
    """计算点到线段的最短距离。"""

    p = np.array(point, dtype=float)
    a = np.array(seg_a, dtype=float)
    b = np.array(seg_b, dtype=float)
    ab = b - a
    ab_len_sq = float(np.dot(ab, ab))
    if ab_len_sq <= 1e-9:
        return float(np.linalg.norm(p - a))
    t = float(np.dot(p - a, ab) / ab_len_sq)
    t = max(0.0, min(1.0, t))
    closest = a + t * ab
    return float(np.linalg.norm(p - closest))


def track_polyline(track: TrackView) -> List[Tuple[float, float]]:
    """目标当前点 + 未来预测点组成的轨迹折线。"""

    return [(track.right_m, track.forward_m), *track.trajectory]


def min_distance_to_track_points(point: Tuple[float, float], tracks: List[TrackView]) -> float:
    """候选点到所有目标当前点/预测点的最小距离。"""

    min_dist = float("inf")
    for track in tracks:
        for pt in track_polyline(track):
            min_dist = min(min_dist, manhattan_distance(point, pt))
    return min_dist


def min_distance_to_track_corridors(point: Tuple[float, float], tracks: List[TrackView]) -> float:
    """候选点到所有目标轨迹折线的最小距离。

    这就是“从目标当前点到目标预测点的直线左右 1m 范围都应该避开”的核心。
    只看离散预测点不够安全，因为两个预测点之间的连线区域也可能被目标经过。
    """

    min_dist = float("inf")
    for track in tracks:
        poly = track_polyline(track)
        if len(poly) == 1:
            min_dist = min(min_dist, manhattan_distance(point, poly[0]))
            continue
        for a, b in zip(poly, poly[1:]):
            min_dist = min(min_dist, distance_point_to_segment(point, a, b))
    return min_dist


def is_point_safe(point: Tuple[float, float], tracks: List[TrackView], extra_margin: float = 0.0) -> bool:
    """判断候选安全点是否安全。

    同时满足：
    1. 离所有目标当前点/预测点 >= SAFETY_RADIUS_M
    2. 离所有目标轨迹线段 >= TRAJECTORY_CORRIDOR_RADIUS_M
    """

    if not tracks:
        return True

    point_clearance = min_distance_to_track_points(point, tracks)
    corridor_clearance = min_distance_to_track_corridors(point, tracks)
    return (
        point_clearance >= SAFETY_RADIUS_M + extra_margin
        and corridor_clearance >= TRAJECTORY_CORRIDOR_RADIUS_M + extra_margin
    )


def compute_candidate_cost(
    point: Tuple[float, float],
    tracks: List[TrackView],
    reference: Tuple[float, float],
) -> Optional[CandidateCost]:
    """计算候选安全点代价。

    返回 None 表示候选点不安全，直接淘汰。
    代价越小越好。
    """

    if tracks:
        min_point_clearance = min_distance_to_track_points(point, tracks)
        min_corridor_clearance = min_distance_to_track_corridors(point, tracks)
    else:
        min_point_clearance = WAYPOINT_LOOKAHEAD_M
        min_corridor_clearance = WAYPOINT_LOOKAHEAD_M

    if min_point_clearance < SAFETY_RADIUS_M:
        return None
    if min_corridor_clearance < TRAJECTORY_CORRIDOR_RADIUS_M:
        return None

    search_scale = max(SAFE_POINT_LATERAL_MAX_M, SAFE_POINT_FORWARD_MAX_M, 1e-6)

    # 间距越接近安全边界，代价越大。
    # 加 0.05 是为了避免除零，也让刚刚越过边界的点仍然代价很高。
    point_margin = max(min_point_clearance - SAFETY_RADIUS_M, 0.05)
    corridor_margin = max(min_corridor_clearance - TRAJECTORY_CORRIDOR_RADIUS_M, 0.05)

    point_clearance_cost = POINT_CLEARANCE_COST_WEIGHT / point_margin
    corridor_clearance_cost = CORRIDOR_CLEARANCE_COST_WEIGHT / corridor_margin

    # 让新安全点尽量靠近上一帧安全点，减少航点左右来回跳。
    continuity_cost = CONTINUITY_COST_WEIGHT * manhattan_distance(point, reference) / search_scale

    # 下视降落场景默认优先靠近无人机正下方，而不是沿机头方向前探。
    forward_cost = FORWARD_COST_WEIGHT * abs(point[1] - WAYPOINT_LOOKAHEAD_M) / max(SAFE_POINT_FORWARD_MAX_M, 1e-6)
    lateral_cost = LATERAL_COST_WEIGHT * abs(point[0]) / max(SAFE_POINT_LATERAL_MAX_M, 1e-6)

    total = point_clearance_cost + corridor_clearance_cost + continuity_cost + forward_cost + lateral_cost

    return CandidateCost(
        total=total,
        point_clearance_cost=point_clearance_cost,
        corridor_clearance_cost=corridor_clearance_cost,
        continuity_cost=continuity_cost,
        forward_cost=forward_cost,
        lateral_cost=lateral_cost,
        min_point_clearance_m=min_point_clearance,
        min_corridor_clearance_m=min_corridor_clearance,
    )


def move_toward_point(
    current: Tuple[float, float],
    target: Tuple[float, float],
    max_step: float,
) -> Tuple[float, float]:
    """把安全点从 current 朝 target 平滑移动一小步。"""

    cur = np.array(current, dtype=float)
    tgt = np.array(target, dtype=float)
    delta = tgt - cur
    dist = float(abs(delta[0]) + abs(delta[1]))

    if dist <= max_step or dist <= 1e-6:
        return float(tgt[0]), float(tgt[1])

    next_pt = cur + delta / dist * max_step
    return float(next_pt[0]), float(next_pt[1])


def generate_safe_candidates() -> List[Tuple[float, float]]:
    """生成安全点候选网格。

    候选点格式：
        (right_m, forward_m)

    下视相机用于找降落点时，候选点从图像/无人机正下方 (0, 0) 开始，
    再按半径向外扩展。这样无障碍时初始点就在屏幕正中心。
    """

    cache_key = (
        SAFE_POINT_LATERAL_MAX_M,
        SAFE_POINT_FORWARD_MAX_M,
        SAFE_POINT_LATERAL_STEP_M,
        SAFE_POINT_FORWARD_STEP_M,
    )
    if getattr(generate_safe_candidates, "_cache_key", None) == cache_key:
        return getattr(generate_safe_candidates, "_cache")

    candidates: List[Tuple[float, float]] = [(0.0, 0.0)]
    right_values = np.arange(-SAFE_POINT_LATERAL_MAX_M, SAFE_POINT_LATERAL_MAX_M + 1e-6, SAFE_POINT_LATERAL_STEP_M)
    forward_values = np.arange(-SAFE_POINT_FORWARD_MAX_M, SAFE_POINT_FORWARD_MAX_M + 1e-6, SAFE_POINT_FORWARD_STEP_M)

    for right in right_values:
        for forward in forward_values:
            point = (float(right), float(forward))
            if abs(point[0]) < 1e-9 and abs(point[1]) < 1e-9:
                continue
            candidates.append(point)

    sorted_candidates = sorted(candidates, key=lambda p: (abs(p[0]) + abs(p[1]), abs(p[1]), abs(p[0])))
    generate_safe_candidates._cache_key = cache_key
    generate_safe_candidates._cache = sorted_candidates
    return sorted_candidates


def find_safe_point(
    tracks: List[TrackView],
    last_safe: Optional[Tuple[float, float]],
) -> Tuple[Optional[Tuple[float, float]], str, Optional[CandidateCost]]:
    """搜索当前帧最合适的安全点。

    设计原则：
    1. 没有目标时，安全点逐步回到正前方。
    2. 上一帧安全点如果仍安全，继续保持。
    3. 如果必须换点，优先选择离上一帧安全点近、又避开目标轨迹走廊的点。
    4. 输出安全点再做步长限制，避免航点突然跳变。
    """

    default_pt = (0.0, WAYPOINT_LOOKAHEAD_M)

    if not tracks:
        if last_safe is None:
            return default_pt, "no tracks", None
        return move_toward_point(last_safe, default_pt, SAFE_POINT_MAX_STEP_M), "no tracks, return to default", None

    # 上一安全点加一点裕度后仍安全，就保持，减少航点抖动。
    if last_safe is not None and is_point_safe(last_safe, tracks, SAFE_POINT_HOLD_MARGIN_M):
        cost = compute_candidate_cost(last_safe, tracks, last_safe)
        return last_safe, "keep previous safe point", cost

    reference = last_safe if last_safe is not None else default_pt
    scored_candidates: List[Tuple[CandidateCost, Tuple[float, float]]] = []
    for pt in generate_safe_candidates():
        cost = compute_candidate_cost(pt, tracks, reference)
        if cost is not None:
            scored_candidates.append((cost, pt))

    if not scored_candidates:
        return None, "no safe candidate", None

    best_cost, target = min(scored_candidates, key=lambda item: item[0].total)

    if last_safe is None:
        return target, "safe candidate found by cost", best_cost

    # 先尝试平滑移动。如果平滑移动后的点仍安全，就输出平滑点。
    smooth_pt = move_toward_point(last_safe, target, SAFE_POINT_MAX_STEP_M)
    smooth_cost = compute_candidate_cost(smooth_pt, tracks, reference)
    if smooth_cost is not None:
        return smooth_pt, "move to min-cost safe candidate", smooth_cost

    # 如果平滑点落入障碍轨迹走廊，只能跳到安全候选点。
    return target, "jump to safe candidate because smooth point is unsafe", best_cost


def compute_waypoint_target(
    tracks: List[TrackView],
    last_safe: Optional[Tuple[float, float]],
    drone_pose: Optional[DronePose],
) -> Tuple[WaypointTarget, Optional[Tuple[float, float]]]:
    """由目标轨迹生成视觉航点。"""

    safe_point, reason, candidate_cost = find_safe_point(tracks, last_safe)

    if safe_point is None:
        return (
            WaypointTarget(
                valid=False,
                frame_id=WAYPOINT_FRAME_ID if RUN_MODE == "FLIGHT" else "base_link",
                reason=reason,
                safe_point_body=None,
                candidate_cost=candidate_cost,
            ),
            None,
        )

    if RUN_MODE == "FLIGHT":
        if drone_pose is None or not drone_pose.valid:
            return (
                WaypointTarget(
                    valid=True,
                    x=safe_point[0],
                    y=safe_point[1],
                    z=0.0,
                    frame_id=WAYPOINT_FRAME_ID,
                    reason=f"{reason}; body-frame only, waiting drone pose",
                    safe_point_body=safe_point,
                    candidate_cost=candidate_cost,
                ),
                safe_point,
            )

        wx, wy, wz = body_safe_point_to_world(safe_point, drone_pose)
        return (
            WaypointTarget(
                valid=True,
                x=wx,
                y=wy,
                z=wz,
                frame_id=WAYPOINT_FRAME_ID,
                reason=reason,
                safe_point_body=safe_point,
                candidate_cost=candidate_cost,
            ),
            safe_point,
        )

    # 调试模式不启用 ROS，因此航点直接以机体系输出，便于观察。
    return (
        WaypointTarget(
            valid=True,
            x=safe_point[0],
            y=safe_point[1],
            z=0.0,
            frame_id="base_link",
            reason=reason,
            safe_point_body=safe_point,
            candidate_cost=candidate_cost,
        ),
        safe_point,
    )


def generate_local_map_candidates() -> List[Tuple[float, float]]:
    """生成以 (0,0) 为中心的局部搜索偏移网格。"""

    half_range = max(
        LOCAL_MAP_SEARCH_RADIUS_M,
        SAFE_POINT_LATERAL_MAX_M,
        SAFE_POINT_FORWARD_MAX_M,
        1.0,
    )
    step = max(min(SAFE_POINT_LATERAL_STEP_M, SAFE_POINT_FORWARD_STEP_M), 0.05)
    cache_key = (half_range, step)
    if getattr(generate_local_map_candidates, "_cache_key", None) == cache_key:
        return getattr(generate_local_map_candidates, "_cache")

    values = np.arange(-half_range, half_range + 1e-6, step)
    candidates: List[Tuple[float, float]] = []
    for x in values:
        for y in values:
            candidates.append((float(x), float(y)))

    candidates.sort(key=lambda p: (abs(p[0]) + abs(p[1]), abs(p[1]), abs(p[0])))
    generate_local_map_candidates._cache_key = cache_key
    generate_local_map_candidates._cache = candidates
    return candidates


def compute_local_map_waypoint(
    tracks: List[TrackView],
    last_safe_local: Optional[Tuple[float, float]],
    drone_pose: DronePose,
    local_map: LocalMapFrame,
) -> Tuple[WaypointTarget, Optional[Tuple[float, float]]]:
    """在视觉内部局部真实坐标系中选择安全点。

    无人机位置来自 MAVROS/Gazebo 原始世界坐标，只通过 local_map 投影到
    局部 map 中；不会重写或重建无人机自己的坐标源。
    """

    drone_local = local_map.drone_local(drone_pose)
    if not tracks:
        safe_local = drone_local
        wx, wy = local_map.local_to_world(safe_local)
        return (
            WaypointTarget(
                valid=True,
                x=wx,
                y=wy,
                z=drone_pose.z if WAYPOINT_Z_MODE != "FIXED" else FIXED_WAYPOINT_Z_M,
                frame_id=WAYPOINT_FRAME_ID,
                reason="no tracks, use drone ground projection in local map",
                safe_point_body=safe_local,
                candidate_cost=None,
            ),
            safe_local,
        )

    search_scale = max(LOCAL_MAP_HALF_RANGE_M, 1e-6)
    search_radius = LOCAL_MAP_HALF_RANGE_M + SAFETY_RADIUS_M + 2.0
    relevant: List[Tuple[float, List[Tuple[float, float]]]] = []
    for track in tracks:
        poly = track_polyline(track)
        nearest = min(manhattan_distance(pt, drone_local) for pt in poly)
        if nearest <= search_radius:
            relevant.append((nearest, poly))
    relevant_polys = [poly for _, poly in sorted(relevant, key=lambda item: item[0])[:MAX_PLANNING_TRACKS]]

    def fast_clearance(local_pt: Tuple[float, float]) -> Tuple[Optional[float], Optional[float]]:
        min_point_clearance = float("inf")
        min_corridor_clearance = float("inf")
        for poly in relevant_polys:
            prev = None
            for pt in poly:
                point_d = manhattan_distance(local_pt, pt)
                if point_d < min_point_clearance:
                    min_point_clearance = point_d
                    if min_point_clearance < SAFETY_RADIUS_M:
                        return None, None
                if prev is not None:
                    corridor_d = distance_point_to_segment(local_pt, prev, pt)
                    if corridor_d < min_corridor_clearance:
                        min_corridor_clearance = corridor_d
                        if min_corridor_clearance < TRAJECTORY_CORRIDOR_RADIUS_M:
                            return None, None
                prev = pt
            if len(poly) == 1:
                min_corridor_clearance = min(min_corridor_clearance, min_point_clearance)
        if not relevant_polys:
            min_point_clearance = LOCAL_MAP_HALF_RANGE_M
            min_corridor_clearance = LOCAL_MAP_HALF_RANGE_M
        return min_point_clearance, min_corridor_clearance

    reference = last_safe_local if last_safe_local is not None else drone_local

    if last_safe_local is not None:
        min_point_clearance, min_corridor_clearance = fast_clearance(last_safe_local)
        if (
            min_point_clearance is not None
            and min_corridor_clearance is not None
            and min_point_clearance >= SAFETY_RADIUS_M + SAFE_POINT_HOLD_MARGIN_M
            and min_corridor_clearance >= TRAJECTORY_CORRIDOR_RADIUS_M + SAFE_POINT_HOLD_MARGIN_M
        ):
            cost = compute_candidate_cost(last_safe_local, tracks, reference)
            wx, wy = local_map.local_to_world(last_safe_local)
            return (
                WaypointTarget(
                    valid=True,
                    x=wx,
                    y=wy,
                    z=drone_pose.z if WAYPOINT_Z_MODE != "FIXED" else FIXED_WAYPOINT_Z_M,
                    frame_id=WAYPOINT_FRAME_ID,
                    reason="keep previous safe point in local map",
                    safe_point_body=last_safe_local,
                    candidate_cost=cost,
                ),
                last_safe_local,
            )

    scored: List[Tuple[CandidateCost, Tuple[float, float]]] = []
    local_candidates = [drone_local]
    if last_safe_local is not None:
        local_candidates.append(last_safe_local)
    for offset in generate_local_map_candidates():
        local_candidates.append((drone_local[0] + offset[0], drone_local[1] + offset[1]))

    seen_candidate_keys = set()
    for local_pt in local_candidates:
        key = (round(local_pt[0], 3), round(local_pt[1], 3))
        if key in seen_candidate_keys:
            continue
        seen_candidate_keys.add(key)
        if abs(local_pt[0]) > LOCAL_MAP_HALF_RANGE_M or abs(local_pt[1]) > LOCAL_MAP_HALF_RANGE_M:
            continue

        min_point_clearance, min_corridor_clearance = fast_clearance(local_pt)
        if min_point_clearance is None or min_corridor_clearance is None:
            continue

        point_margin = max(min_point_clearance - SAFETY_RADIUS_M, 0.05)
        corridor_margin = max(min_corridor_clearance - TRAJECTORY_CORRIDOR_RADIUS_M, 0.05)
        point_clearance_cost = POINT_CLEARANCE_COST_WEIGHT / point_margin
        corridor_clearance_cost = CORRIDOR_CLEARANCE_COST_WEIGHT / corridor_margin
        continuity_cost = CONTINUITY_COST_WEIGHT * manhattan_distance(local_pt, reference) / search_scale
        forward_cost = FORWARD_COST_WEIGHT * manhattan_distance(local_pt, drone_local) / search_scale
        lateral_cost = LATERAL_COST_WEIGHT * manhattan_distance(local_pt, (0.0, 0.0)) / search_scale

        cost = CandidateCost(
            total=point_clearance_cost + corridor_clearance_cost + continuity_cost + forward_cost + lateral_cost,
            point_clearance_cost=point_clearance_cost,
            corridor_clearance_cost=corridor_clearance_cost,
            continuity_cost=continuity_cost,
            forward_cost=forward_cost,
            lateral_cost=lateral_cost,
            min_point_clearance_m=min_point_clearance,
            min_corridor_clearance_m=min_corridor_clearance,
        )
        scored.append((cost, local_pt))

    if not scored:
        return (
            WaypointTarget(
                valid=False,
                frame_id=WAYPOINT_FRAME_ID,
                reason="no safe candidate in local map",
                safe_point_body=None,
                candidate_cost=None,
            ),
            None,
        )

    best_cost, target = min(scored, key=lambda item: item[0].total)
    safe_local = target
    reason = "safe candidate found in local map"
    if last_safe_local is not None:
        smooth_local = move_toward_point(last_safe_local, target, SAFE_POINT_MAX_STEP_M)
        smooth_cost = compute_candidate_cost(smooth_local, tracks, reference)
        if smooth_cost is not None:
            safe_local = smooth_local
            best_cost = smooth_cost
            reason = "move to local-map safe candidate"

    wx, wy = local_map.local_to_world(safe_local)
    return (
        WaypointTarget(
            valid=True,
            x=wx,
            y=wy,
            z=drone_pose.z if WAYPOINT_Z_MODE != "FIXED" else FIXED_WAYPOINT_Z_M,
            frame_id=WAYPOINT_FRAME_ID,
            reason=reason,
            safe_point_body=safe_local,
            candidate_cost=best_cost,
        ),
        safe_local,
    )


# =========================================================
# 可视化
# =========================================================
def ground_to_pixel(
    point: Tuple[float, float],
    origin: Tuple[int, int],
    scale: float,
) -> Tuple[int, int]:
    """局部地面坐标转俯视图像素坐标，仅用于显示。

    Top View 按用户要求相对数学坐标逆时针旋转 90 度显示；
    规划、跟踪和发布航点仍使用未旋转的真实局部坐标。
    """

    x_m, y_m = point
    display_x_m = -y_m
    display_y_m = x_m
    return int(origin[0] + display_x_m * scale), int(origin[1] - display_y_m * scale)


def draw_ground_view(
    tracks: List[TrackView],
    waypoint: WaypointTarget,
    width: int,
    height: int,
    drone_pose: Optional[DronePose] = None,
    local_map: Optional[LocalMapFrame] = None,
) -> np.ndarray:
    """绘制俯视图。

    俯视图用于调试避障逻辑：
    - 白点：无人机
    - 蓝/橙点：检测中/预测中的目标
    - 暗色粗线：目标预测轨迹走廊
    - 绿色点：当前安全点
    """

    panel = np.zeros((height, width, 3), dtype=np.uint8)
    panel[:] = PANEL_BG
    origin = (width // 2, height // 2)
    visible_range_m = max(
        LOCAL_MAP_HALF_RANGE_M if local_map is not None and local_map.valid else 0.0,
        SAFE_POINT_LATERAL_MAX_M,
        SAFE_POINT_FORWARD_MAX_M,
        WAYPOINT_LOOKAHEAD_M,
        1.0,
    ) + max(SAFETY_RADIUS_M, TRAJECTORY_CORRIDOR_RADIUS_M) + 0.5
    scale = 0.46 * min(width, height) / visible_range_m

    cv2.line(panel, (origin[0], 0), (origin[0], height), GRID_COLOR, 1)
    cv2.line(panel, (0, origin[1]), (width, origin[1]), GRID_COLOR, 1)
    cv2.putText(panel, "local map view: rotated CCW 90deg", (12, 24), FONT, 0.55, (220, 220, 220), 1)
    if local_map is not None and local_map.valid:
        cv2.putText(
            panel,
            f"origin=({local_map.origin_x:.1f},{local_map.origin_y:.1f}) rebase={local_map.rebase_count}",
            (12, 48),
            FONT,
            0.48,
            (180, 180, 180),
            1,
        )
    else:
        cv2.putText(panel, "center = current camera nadir", (12, 48), FONT, 0.48, (180, 180, 180), 1)

    for meter in range(1, int(math.ceil(visible_range_m)) + 1):
        r = int(meter * scale)
        cv2.circle(panel, origin, r, (48, 48, 48), 1)

    corridor_thickness = max(1, int(TRAJECTORY_CORRIDOR_RADIUS_M * scale * 2))

    for track in tracks:
        color = PREDICTED_TRACK_COLOR if track.missed_frames > 0 else NORMAL_TRACK_COLOR
        poly = track_polyline(track)
        if local_map is None or not local_map.valid:
            if RUN_MODE == "FLIGHT" and drone_pose is not None and drone_pose.valid:
                poly = [world_point_to_body(p, drone_pose) for p in poly]
        pixels = [ground_to_pixel(p, origin, scale) for p in poly]

        # 先画轨迹走廊，再画中心轨迹线。
        if len(pixels) > 1:
            for p1, p2 in zip(pixels, pixels[1:]):
                cv2.line(panel, p1, p2, CORRIDOR_COLOR, corridor_thickness)
            for p1, p2 in zip(pixels, pixels[1:]):
                cv2.line(panel, p1, p2, color, 2)

        cur_pixel = pixels[0]
        cv2.circle(panel, cur_pixel, int(SAFETY_RADIUS_M * scale), (60, 60, 90), 1)
        cv2.circle(panel, cur_pixel, 5, color, -1)
        label_suffix = f" P{track.missed_frames}" if track.missed_frames > 0 else ""
        cv2.putText(panel, f"ID {track.track_id}{label_suffix}", (cur_pixel[0] + 7, cur_pixel[1] - 7), FONT, 0.45, color, 1)

    display_safe_body = waypoint.safe_point_body
    if (
        waypoint.valid
        and drone_pose is not None
        and drone_pose.valid
        and waypoint.frame_id == WAYPOINT_FRAME_ID
        and (local_map is None or not local_map.valid)
    ):
        display_safe_body = world_point_to_body((waypoint.x, waypoint.y), drone_pose)

    if display_safe_body is not None:
        safe_pixel = ground_to_pixel(display_safe_body, origin, scale)
        cv2.circle(panel, safe_pixel, 8, SAFE_POINT_COLOR, -1)
        cv2.putText(panel, "SAFE", (safe_pixel[0] + 8, safe_pixel[1] - 8), FONT, 0.5, SAFE_POINT_COLOR, 2)

    if local_map is not None and local_map.valid and drone_pose is not None and drone_pose.valid:
        drone_local = local_map.drone_local(drone_pose)
        drone_pixel = ground_to_pixel(drone_local, origin, scale)
        cv2.circle(panel, drone_pixel, 7, DRONE_MARKER_COLOR, -1)
        cv2.putText(panel, "UAV", (drone_pixel[0] + 9, drone_pixel[1] - 9), FONT, 0.45, DRONE_MARKER_COLOR, 1)
        threshold = LOCAL_MAP_HALF_RANGE_M * LOCAL_MAP_REBASE_FRACTION
        cv2.rectangle(
            panel,
            ground_to_pixel((-threshold, threshold), origin, scale),
            ground_to_pixel((threshold, -threshold), origin, scale),
            (70, 110, 70),
            1,
        )
    else:
        cv2.circle(panel, origin, 5, DRONE_MARKER_COLOR, -1)

    return panel


def draw_visualization(
    color_bgr: np.ndarray,
    depth_m: np.ndarray,
    detections: List[Detection],
    tracks: List[TrackView],
    waypoint: WaypointTarget,
    fps: float,
    imu_status: Optional[ImuStabilizationStatus],
    drone_pose: Optional[DronePose] = None,
    flight_state: Optional[dict] = None,
    local_map: Optional[LocalMapFrame] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """绘制 RGB 检测图、深度伪彩色图、俯视图。"""

    display = color_bgr.copy()
    track_map = {track.bbox: track for track in tracks if track.bbox is not None}

    for det in detections:
        track = track_map.get(det.bbox)
        color = SAFE_POINT_COLOR
        x1, y1, x2, y2 = det.bbox
        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
        cv2.circle(display, det.center, 4, color, -1)

        if track is None:
            text = f"{det.label} {det.confidence:.2f} {det.depth_m:.2f}m"
        else:
            text = (
                f"ID {track.track_id} {det.label} {det.depth_m:.2f}m "
                f"v=({track.v_right_mps:.2f},{track.v_forward_mps:.2f})"
            )
        cv2.putText(display, text, (x1, max(20, y1 - 8)), FONT, 0.5, color, 2)

    if waypoint.valid:
        if waypoint.safe_point_body is None:
            status_line1 = f"WP valid frame={waypoint.frame_id} FPS:{fps:.1f}"
        else:
            sr, sf = waypoint.safe_point_body
            status_line1 = (
                f"WP valid safe=({sr:.2f},{sf:.2f}) "
                f"out=({waypoint.x:.2f},{waypoint.y:.2f},{waypoint.z:.2f}) FPS:{fps:.1f}"
            )
    else:
        status_line1 = f"WP invalid reason={waypoint.reason} FPS:{fps:.1f}"

    cv2.putText(display, status_line1, (15, 30), FONT, 0.55, STATUS_TEXT_COLOR, 2)
    cv2.putText(display, waypoint.reason, (15, 58), FONT, 0.50, STATUS_TEXT_COLOR, 2)

    text_y = 86
    if waypoint.candidate_cost is not None:
        cost = waypoint.candidate_cost
        cost_text = (
            f"cost:{cost.total:.2f} point_clear:{cost.min_point_clearance_m:.2f}m "
            f"corridor_clear:{cost.min_corridor_clearance_m:.2f}m"
        )
        cv2.putText(display, cost_text, (15, text_y), FONT, 0.46, STATUS_TEXT_COLOR, 1)
        text_y += 24

    if imu_status is not None:
        imu_text = (
            f"IMU {imu_status.reason} rate:{imu_status.angular_rate:.2f}rad/s "
            f"d=({math.degrees(imu_status.roll_delta):.1f},"
            f"{math.degrees(imu_status.pitch_delta):.1f},"
            f"{math.degrees(imu_status.yaw_delta):.1f})deg"
        )
        cv2.putText(display, imu_text, (15, text_y), FONT, 0.46, STATUS_TEXT_COLOR, 1)

    depth_show = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
    depth_show = np.clip(depth_show / MAX_DEPTH_M * 255.0, 0, 255).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_show, cv2.COLORMAP_JET)
    for det in detections:
        cx, cy = det.center
        if 0 <= cx < depth_color.shape[1] and 0 <= cy < depth_color.shape[0]:
            # Body FRD convention: x forward, y right, z down.
            body_x = det.forward_m
            body_y = det.right_m
            body_z = det.depth_m
            marker_color = (255, 255, 255)
            cv2.drawMarker(
                depth_color,
                (cx, cy),
                marker_color,
                markerType=cv2.MARKER_CROSS,
                markerSize=14,
                thickness=2,
                line_type=cv2.LINE_AA,
            )
            cv2.circle(depth_color, (cx, cy), 5, marker_color, 1, lineType=cv2.LINE_AA)
            text = f"x={body_x:.2f} y={body_y:.2f} z={body_z:.2f}m"
            text_x = min(cx + 10, max(0, depth_color.shape[1] - 250))
            text_y = max(18, cy - 10)
            cv2.putText(depth_color, text, (text_x, text_y), FONT, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(depth_color, text, (text_x, text_y), FONT, 0.48, marker_color, 1, cv2.LINE_AA)

    ground_panel = draw_ground_view(tracks, waypoint, display.shape[1], display.shape[0], drone_pose, local_map)
    attitude_panel = draw_attitude_view(drone_pose, flight_state, display.shape[1], display.shape[0])
    return display, depth_color, ground_panel, attitude_panel


def draw_attitude_view(
    drone_pose: Optional[DronePose],
    flight_state: Optional[dict],
    width: int,
    height: int,
) -> np.ndarray:
    """绘制三维无人机姿态和相机指向向量。"""

    panel = np.zeros((height, width, 3), dtype=np.uint8)
    panel[:] = PANEL_BG
    cx, cy = width // 2, height // 2
    scale = min(width, height) * 0.23

    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    rel_alt = 0.0
    mode = ""
    armed = False
    if flight_state:
        roll = math.radians(float(flight_state.get("roll_deg", 0.0) or 0.0)) if "roll_deg" in flight_state else 0.0
        pitch = math.radians(float(flight_state.get("pitch_deg", 0.0) or 0.0)) if "pitch_deg" in flight_state else 0.0
        yaw = math.radians(float(flight_state.get("heading_deg", 0.0) or 0.0)) if "heading_deg" in flight_state else 0.0
        rel_alt = float(flight_state.get("relative_alt_m", 0.0) or 0.0)
        mode = str(flight_state.get("mode", ""))
        armed = bool(flight_state.get("armed", False))
    elif drone_pose is not None:
        roll = drone_pose.roll
        pitch = drone_pose.pitch
        yaw = drone_pose.yaw
        rel_alt = drone_pose.z

    cv2.putText(panel, "3D drone + camera vector", (16, 28), FONT, 0.65, STATUS_TEXT_COLOR, 2)
    cv2.putText(panel, f"mode: {mode}  armed: {armed}", (16, 58), FONT, 0.5, (220, 220, 220), 1)
    cv2.putText(panel, f"alt: {rel_alt:.2f}m", (16, 82), FONT, 0.5, (220, 220, 220), 1)
    cv2.putText(
        panel,
        f"roll {math.degrees(roll):+.1f}  pitch {math.degrees(pitch):+.1f}  yaw {math.degrees(yaw):.1f}",
        (16, 106),
        FONT,
        0.48,
        (220, 220, 220),
        1,
    )

    def rot_matrix(r, p, y):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy_, sy_ = math.cos(y), math.sin(y)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
        rz = np.array([[cy_, -sy_, 0], [sy_, cy_, 0], [0, 0, 1]], dtype=float)
        return rz @ ry @ rx

    body = rot_matrix(roll, pitch, yaw)

    camera_pos = body @ np.array([-3.0, 0.0, 1.55], dtype=float)
    camera_target = body @ np.array([0.35, 0.0, -0.08], dtype=float)
    forward_view = camera_target - camera_pos
    forward_view = forward_view / max(float(np.linalg.norm(forward_view)), 1e-6)
    world_up = np.array([0.0, 0.0, 1.0], dtype=float)
    right_view = np.cross(forward_view, world_up)
    if float(np.linalg.norm(right_view)) < 1e-6:
        right_view = body @ np.array([0.0, 1.0, 0.0], dtype=float)
    right_view = right_view / max(float(np.linalg.norm(right_view)), 1e-6)
    up_view = np.cross(right_view, forward_view)
    up_view = up_view / max(float(np.linalg.norm(up_view)), 1e-6)

    def project(v):
        rel = v - camera_target
        x = float(np.dot(rel, right_view))
        y = float(np.dot(rel, up_view))
        z = float(np.dot(rel, forward_view)) + 4.0
        perspective = 4.0 / max(z, 0.8)
        return int(cx + x * scale * perspective), int(cy - y * scale * perspective)

    origin3 = np.array([0.0, 0.0, 0.0])
    front = body @ np.array([1.25, 0.0, 0.0])
    right = body @ np.array([0.0, 0.9, 0.0])
    left = body @ np.array([0.0, -0.9, 0.0])
    rear = body @ np.array([-0.85, 0.0, 0.0])
    up = body @ np.array([0.0, 0.0, 0.9])
    camera_mount = body @ np.array([0.24, 0.0, -0.35])
    camera_axis = camera_mount + body @ np.array([0.0, 0.0, -1.2])

    hull = [front, right, rear, left]
    hull_pts = [project(p) for p in hull]
    cv2.polylines(panel, [np.array(hull_pts, dtype=np.int32)], True, (90, 130, 170), 2)
    cv2.circle(panel, project(origin3), 5, (255, 255, 255), -1)
    cv2.arrowedLine(panel, project(origin3), project(front), (0, 255, 0), 3, tipLength=0.18)
    cv2.arrowedLine(panel, project(origin3), project(right), (255, 80, 80), 3, tipLength=0.18)
    cv2.arrowedLine(panel, project(origin3), project(up), (80, 160, 255), 3, tipLength=0.18)
    cv2.circle(panel, project(camera_mount), 5, (0, 255, 255), -1)
    cv2.arrowedLine(panel, project(camera_mount), project(camera_axis), (0, 255, 255), 3, tipLength=0.20)
    cv2.putText(panel, "front", project(front), FONT, 0.45, (0, 255, 0), 1)
    cv2.putText(panel, "right", project(right), FONT, 0.45, (255, 80, 80), 1)
    cv2.putText(panel, "up", project(up), FONT, 0.45, (80, 160, 255), 1)
    cv2.putText(panel, "camera optical axis", (16, height - 22), FONT, 0.55, (0, 255, 255), 2)

    return panel


def combine_demo_frame(
    display: np.ndarray,
    depth_color: np.ndarray,
    ground_panel: np.ndarray,
    attitude_panel: np.ndarray,
) -> np.ndarray:
    """拼接四宫格演示画面，方便实时显示和保存视频。"""

    def add_panel_title(image: np.ndarray, title: str) -> np.ndarray:
        title_bar_h = 24
        title_bar = np.full((title_bar_h, image.shape[1], 3), (24, 24, 24), dtype=np.uint8)
        cv2.putText(title_bar, title, (10, 17), FONT, 0.46, (230, 230, 230), 1, cv2.LINE_AA)
        return np.vstack([title_bar, image])

    top_row = np.hstack([
        add_panel_title(display, "RGB Detection"),
        add_panel_title(depth_color, "Depth"),
    ])
    bottom_row = np.hstack([
        add_panel_title(ground_panel, "Top View"),
        add_panel_title(attitude_panel, "Attitude"),
    ])
    return np.vstack([top_row, bottom_row])


# =========================================================
# 系统主类
# =========================================================
class VisionWaypointSystem:
    """视觉避障航点生成系统。"""

    def __init__(self) -> None:
        self.ros: Optional[FlightRosInterface] = None
        self.source = None
        self.model: Optional[YOLO] = None
        self.yolo_device: Optional[str] = None
        self.tracker = MultiObjectTracker()
        self.class_filter = normalize_class_filter(YOLO_CLASSES)
        self.imu_stabilizer: Optional[ImuFrameStabilizer] = None
        self.last_safe_point: Optional[Tuple[float, float]] = None
        self.last_safe_world: Optional[Tuple[float, float]] = None
        self.local_map = LocalMapFrame()
        self.last_print_time = 0.0
        self.last_profile_time = 0.0
        self.last_frame_time = time.time()
        self.profiler = StageProfiler(interval_s=PROFILE_INTERVAL_S)
        self.frame_count = 0
        self.video_writer = None
        self.output_path: Optional[Path] = None

    def _ensure_local_map(self, drone_pose: Optional[DronePose]) -> None:
        if RUN_MODE != "FLIGHT" or drone_pose is None or not drone_pose.valid:
            return
        if not self.local_map.valid:
            self.local_map.set_from_pose(drone_pose)

    def _shift_internal_local_state(self, dx: float, dy: float) -> None:
        self.tracker.shift_all(dx, dy)
        if self.last_safe_point is not None:
            self.last_safe_point = (self.last_safe_point[0] + dx, self.last_safe_point[1] + dy)
        if self.last_safe_world is not None:
            self.last_safe_world = (self.last_safe_world[0] + dx, self.last_safe_world[1] + dy)

    def _maybe_rebase_local_map(self, drone_pose: Optional[DronePose], track_count: int) -> None:
        if RUN_MODE != "FLIGHT" or drone_pose is None or not drone_pose.valid or not self.local_map.valid:
            return
        drone_local = self.local_map.drone_local(drone_pose)
        threshold = LOCAL_MAP_HALF_RANGE_M * LOCAL_MAP_REBASE_FRACTION
        no_tracks = track_count == 0
        near_boundary = max(abs(drone_local[0]), abs(drone_local[1])) >= threshold
        if not no_tracks and not near_boundary:
            return

        old_origin = (self.local_map.origin_x, self.local_map.origin_y)
        new_origin = (float(drone_pose.x), float(drone_pose.y))
        shift = (old_origin[0] - new_origin[0], old_origin[1] - new_origin[1])
        if manhattan_distance((0.0, 0.0), shift) < LOCAL_MAP_REBASE_MIN_SHIFT_M:
            return

        self._shift_internal_local_state(shift[0], shift[1])
        self.local_map.origin_x = new_origin[0]
        self.local_map.origin_y = new_origin[1]
        self.local_map.rebase_count += 1

    def _init_source(self) -> None:
        if RUN_MODE == "FLIGHT":
            if self.ros is None:
                raise RuntimeError("FLIGHT 模式需要先初始化 ROS 接口。")
            self.source = RosRgbdSource(self.ros)
            return

        if DEBUG_INPUT_TYPE == "realsense_bag":
            self.source = RealSenseSource(bag_path=DEBUG_REALSENSE_BAG_PATH)
        elif DEBUG_INPUT_TYPE == "ros2_db3":
            debug_path = Path(DEBUG_ROS2_DB3_PATH)
            record_dir = debug_path.parent if debug_path.suffix == ".db3" else debug_path
            if (
                (record_dir / "metadata.json").exists()
                and (record_dir / "frames_sync.csv").exists()
                and any(record_dir.glob("*.db3"))
            ):
                self.source = RecordedFlightDb3Source(str(record_dir))
            else:
                self.source = Ros2Db3Source(DEBUG_ROS2_DB3_PATH)
        else:
            raise ValueError(f"未知 DEBUG_INPUT_TYPE：{DEBUG_INPUT_TYPE}")

    def _init_video_writer(self, demo_frame: np.ndarray) -> None:
        if OUTPUT_VIDEO_PATH is None or self.video_writer is not None:
            return

        self.output_path = Path(OUTPUT_VIDEO_PATH)
        if self.output_path.is_dir():
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_path = self.output_path / f"vision_waypoint_demo_{stamp}.mp4"
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer = cv2.VideoWriter(
            str(self.output_path),
            fourcc,
            float(CAMERA_FPS),
            (demo_frame.shape[1], demo_frame.shape[0]),
        )

    def _select_yolo_device(self) -> Optional[str]:
        if YOLO_DEVICE and YOLO_DEVICE.lower() not in ("auto", ""):
            return YOLO_DEVICE
        if torch is not None and torch.cuda.is_available():
            return "0"
        return "cpu"

    def start(self) -> None:
        cv2.setNumThreads(max(1, int(os.environ.get("VISION_AVOID_CV_THREADS", "1"))))
        if torch is not None:
            try:
                torch.set_num_threads(max(1, int(os.environ.get("VISION_AVOID_TORCH_THREADS", "2"))))
            except Exception:
                pass

        print("⏳ 初始化视觉航点生成系统...")
        print(f"📌 运行模式：{RUN_MODE}")

        if RUN_MODE == "FLIGHT":
            self.ros = FlightRosInterface()
        elif RUN_MODE != "DEBUG":
            raise ValueError("RUN_MODE 只能是 'DEBUG' 或 'FLIGHT'")

        self._init_source()
        first_frame = self.source.read()
        if first_frame is None:
            raise RuntimeError("无法读取第一帧，请检查相机/离线包路径。")

        # 先用第一帧内参初始化 IMU 防抖器。
        self.imu_stabilizer = ImuFrameStabilizer(first_frame.intrinsics)

        print("⏳ 加载 YOLO 模型...")
        self.model = YOLO(MODEL_PATH, task="detect")
        self.yolo_device = self._select_yolo_device()

        # 第一帧已经被读出来，先处理它，再进入循环继续读。
        print("🚀 系统启动，按 q / ESC 退出")
        cuda_status = "unavailable"
        if torch is not None:
            cuda_status = f"available={torch.cuda.is_available()}"
            if torch.cuda.is_available():
                cuda_status += f", name={torch.cuda.get_device_name(0)}"
        print(
            f"🔍 YOLO: model={MODEL_PATH}, conf={YOLO_CONF}, iou={YOLO_IOU}, "
            f"imgsz={YOLO_IMGSZ}, device={self.yolo_device}, cuda={cuda_status}"
        )
        print(f"📏 跟踪：match={MATCH_DISTANCE_M}m, missed>{MAX_MISSED_FRAMES} 删除")
        print(
            f"🧭 规划：lookahead={WAYPOINT_LOOKAHEAD_M}m, "
            f"safety={SAFETY_RADIUS_M}m, corridor={TRAJECTORY_CORRIDOR_RADIUS_M}m"
        )

        self._process_frame(first_frame)

        while True:
            if RUN_MODE == "DEBUG" and DEBUG_MAX_FRAMES and self.frame_count >= DEBUG_MAX_FRAMES:
                break

            if self.ros is not None:
                self.ros.spin_once()

            frame = self.source.read()
            if frame is None:
                print("⏹️ 输入数据结束")
                break

            should_continue = self._process_frame(frame)
            if not should_continue:
                break

    def _process_frame(self, frame: FramePacket) -> bool:
        assert self.model is not None
        assert self.imu_stabilizer is not None

        t0 = time.perf_counter()
        current_time = frame.stamp if RUN_MODE == "DEBUG" else time.time()
        imu_snapshot = self.ros.get_imu_snapshot() if self.ros is not None else frame.imu
        drone_pose = self.ros.get_drone_pose() if self.ros is not None else frame.drone_pose
        self._ensure_local_map(drone_pose)
        t_pose = time.perf_counter()

        stable_color, stable_depth, imu_status = self.imu_stabilizer.stabilize(
            frame.color_bgr,
            frame.depth_m,
            imu_snapshot,
            current_time,
        )
        t_stabilize = time.perf_counter()

        if imu_status.shaken:
            detections: List[Detection] = []
        else:
            detections = run_detection(
                self.model,
                stable_color,
                stable_depth,
                frame.intrinsics,
                self.class_filter,
                self.yolo_device,
            )
        t_detect = time.perf_counter()

        if RUN_MODE == "FLIGHT" and self.local_map.valid:
            tracker_detections = detections_to_local_map(detections, drone_pose, self.local_map)
        else:
            tracker_detections = detections_to_world(detections, drone_pose)
        tracks = self.tracker.update(tracker_detections, current_time)
        track_views = [track.as_view() for track in tracks]
        t_track = time.perf_counter()

        if RUN_MODE == "FLIGHT" and drone_pose is not None and drone_pose.valid and self.local_map.valid:
            self._maybe_rebase_local_map(drone_pose, len(track_views))
            if self.local_map.valid:
                track_views = [track.as_view() for track in self.tracker.tracks.values()]
            waypoint, self.last_safe_point = compute_local_map_waypoint(
                track_views,
                self.last_safe_point,
                drone_pose,
                self.local_map,
            )
        else:
            waypoint, self.last_safe_point = compute_waypoint_target(track_views, self.last_safe_point, drone_pose)

        if self.ros is not None:
            self.ros.publish_waypoint(waypoint)
        t_plan = time.perf_counter()

        fps = 1.0 / max(current_time - self.last_frame_time, 1e-6)
        self.last_frame_time = current_time

        if current_time - self.last_print_time >= PRINT_INTERVAL_S:
            self._print_status(waypoint, len(track_views), fps, imu_status, frame.flight_state)
            self.last_print_time = current_time

        if DISPLAY_ENABLE or OUTPUT_VIDEO_PATH is not None:
            display, depth_color, ground_panel, attitude_panel = draw_visualization(
                stable_color,
                stable_depth,
                detections,
                track_views,
                waypoint,
                fps,
                imu_status,
                drone_pose,
                frame.flight_state,
                self.local_map if self.local_map.valid else None,
            )

            demo_frame = combine_demo_frame(display, depth_color, ground_panel, attitude_panel)
            if OUTPUT_VIDEO_PATH is not None:
                self._init_video_writer(demo_frame)
                self.video_writer.write(demo_frame)

            if DISPLAY_ENABLE:
                cv2.imshow("Vision Waypoint", demo_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    return False
        t_display = time.perf_counter()

        if PROFILE_ENABLE:
            self.profiler.add(
                pose=t_pose - t0,
                stabilize=t_stabilize - t_pose,
                yolo=t_detect - t_stabilize,
                track=t_track - t_detect,
                plan_publish=t_plan - t_track,
                display=t_display - t_plan,
                total=t_display - t0,
            )
            self.profiler.maybe_print(f"SIM_PROFILE device={self.yolo_device}")

        self.frame_count += 1
        return True

    @staticmethod
    def _print_status(
        waypoint: WaypointTarget,
        track_count: int,
        fps: float,
        imu_status: ImuStabilizationStatus,
        flight_state: Optional[dict] = None,
    ) -> None:
        if waypoint.safe_point_body is None:
            safe_str = "safe=None"
        else:
            safe_str = f"safe_local=({waypoint.safe_point_body[0]:.2f},{waypoint.safe_point_body[1]:.2f})m"

        if waypoint.valid:
            wp_str = f"wp=({waypoint.x:.2f},{waypoint.y:.2f},{waypoint.z:.2f}) frame={waypoint.frame_id}"
        else:
            wp_str = "wp=None"

        if waypoint.candidate_cost is None:
            cost_str = "cost=None"
        else:
            cost = waypoint.candidate_cost
            cost_str = (
                f"cost={cost.total:.2f} "
                f"point_clear={cost.min_point_clearance_m:.2f}m "
                f"corridor_clear={cost.min_corridor_clearance_m:.2f}m"
            )

        flight_str = ""
        if flight_state:
            flight_str = (
                f" flight_frame={flight_state.get('frame_idx')} "
                f"mode={flight_state.get('mode')} armed={flight_state.get('armed')} "
                f"rel_alt={flight_state.get('relative_alt_m'):.2f}m "
                f"yaw={flight_state.get('heading_deg'):.1f}deg"
            )

        print(
            f"[{time.strftime('%H:%M:%S')}] "
            f"tracks={track_count} valid={waypoint.valid} {safe_str} {wp_str} "
            f"{cost_str} reason={waypoint.reason} "
            f"fps={fps:.1f} imu={imu_status.reason}{flight_str}"
        )

    def close(self) -> None:
        if self.video_writer is not None:
            self.video_writer.release()
        if self.source is not None:
            self.source.close()
        if self.ros is not None:
            self.ros.close()
        cv2.destroyAllWindows()
        if self.output_path is not None:
            print(f"🎞️ 演示视频已保存：{self.output_path}")
        print("👋 程序正常退出")


# =========================================================
# 程序入口
# =========================================================
def main() -> int:
    system = VisionWaypointSystem()
    try:
        system.start()
    finally:
        system.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
