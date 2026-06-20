#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real-flight lightweight D435i + MAVLink vision waypoint node.

No replay, no Gazebo, no OpenCV windows. The node reads a downward D435i,
uses MAVLink LOCAL_POSITION_NED + ATTITUDE as the aircraft state, publishes a
world-frame safe waypoint for the landing state machine, and records compact
JSONL/CSV logs for post-flight review.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO


WAYPOINT_TOPIC = "/vision/avoidance_waypoint"
STATUS_TOPIC = "/vision/avoidance_status"
WAYPOINT_FRAME_ID = "map"

DEFAULT_MODEL = "/home/zyc/vision_avoid/irreality.engine"
YOLO_CONF = 0.60
YOLO_IOU = 0.05
YOLO_IMGSZ = 640
MIN_DEPTH_M = 0.2
MAX_DEPTH_M = 15.0
DEPTH_CROP_MARGIN = 0.2

MATCH_DISTANCE_M = 2.2
DUPLICATE_SPAWN_DISTANCE_M = 1.2
MAX_MISSED_FRAMES = 20
PREDICTION_HORIZON_S = 3.0
PREDICTION_DT_S = 0.2

SAFETY_RADIUS_M = 1.2
CORRIDOR_RADIUS_M = 1.0
SAFE_HOLD_MARGIN_M = 0.15
SAFE_MAX_STEP_M = 0.25
LOCAL_MAP_HALF_RANGE_M = 10.0
LOCAL_MAP_SEARCH_RADIUS_M = 4.0
LOCAL_MAP_REBASE_FRACTION = 0.5
SAFE_STEP_M = 0.4
MAX_PLANNING_TRACKS = 8

POINT_CLEARANCE_WEIGHT = 5.0
CORRIDOR_CLEARANCE_WEIGHT = 8.0
CONTINUITY_WEIGHT = 10.0
DRONE_DISTANCE_WEIGHT = 1.0
ORIGIN_DISTANCE_WEIGHT = 0.25


@dataclass
class FcPose:
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    mode: str = ""
    armed: bool = False
    valid_pos: bool = False
    valid_att: bool = False

    @property
    def z_up(self) -> float:
        return -self.down

    @property
    def valid(self) -> bool:
        return self.valid_pos and self.valid_att


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: Tuple[int, int, int, int]
    center: Tuple[int, int]
    depth_m: float
    body_right_m: float
    body_forward_m: float
    local_x: float = 0.0
    local_y: float = 0.0


@dataclass
class TrackView:
    track_id: int
    label: str
    confidence: float
    local_x: float
    local_y: float
    vx: float
    vy: float
    missed_frames: int
    trajectory: List[Tuple[float, float]]


@dataclass
class CandidateCost:
    total: float
    point_clearance: float
    corridor_clearance: float


@dataclass
class LocalMap:
    origin_north: float = 0.0
    origin_east: float = 0.0
    valid: bool = False
    rebase_count: int = 0

    def ensure(self, pose: FcPose) -> None:
        if not self.valid:
            self.origin_north = pose.north
            self.origin_east = pose.east
            self.valid = True

    def world_to_local(self, north: float, east: float) -> Tuple[float, float]:
        return north - self.origin_north, east - self.origin_east

    def local_to_world(self, x: float, y: float) -> Tuple[float, float]:
        return x + self.origin_north, y + self.origin_east

    def drone_local(self, pose: FcPose) -> Tuple[float, float]:
        return self.world_to_local(pose.north, pose.east)


class MavlinkState:
    def __init__(self, conn: str, baud: int) -> None:
        self.conn = conn
        self.baud = baud
        self.pose = FcPose()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def latest(self) -> FcPose:
        with self.lock:
            return FcPose(**self.pose.__dict__)

    def _request_rate(self, master, msg_id: int, hz: float) -> None:
        try:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                int(1_000_000 / hz),
                0,
                0,
                0,
                0,
                0,
            )
        except Exception:
            pass

    def _run(self) -> None:
        master = mavutil.mavlink_connection(self.conn, baud=self.baud, autoreconnect=True)
        master.wait_heartbeat()
        self._request_rate(master, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1)
        self._request_rate(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30)
        self._request_rate(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)
        while not self.stop.is_set():
            msg = master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            msg_type = msg.get_type()
            with self.lock:
                if msg_type == "HEARTBEAT":
                    try:
                        self.pose.mode = mavutil.mode_string_v10(msg)
                    except Exception:
                        self.pose.mode = ""
                    self.pose.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif msg_type == "LOCAL_POSITION_NED":
                    self.pose.north = float(msg.x)
                    self.pose.east = float(msg.y)
                    self.pose.down = float(msg.z)
                    self.pose.valid_pos = True
                elif msg_type == "ATTITUDE":
                    self.pose.roll = float(msg.roll)
                    self.pose.pitch = float(msg.pitch)
                    self.pose.yaw = float(msg.yaw)
                    self.pose.valid_att = True


class KalmanTrack:
    def __init__(self, track_id: int, det: Detection, now: float) -> None:
        self.track_id = track_id
        self.label = det.label
        self.confidence = det.confidence
        self.state = np.array([[det.local_x], [det.local_y], [0.0], [0.0]], dtype=float)
        self.cov = np.diag([0.2, 0.2, 4.0, 4.0]).astype(float)
        self.last_update = now
        self.missed_frames = 0

    def predict_to(self, now: float) -> None:
        dt = max(now - self.last_update, 1e-3)
        f = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        q = 1.0
        noise = q * np.array(
            [[dt**4 / 4, 0, dt**3 / 2, 0], [0, dt**4 / 4, 0, dt**3 / 2], [dt**3 / 2, 0, dt**2, 0], [0, dt**3 / 2, 0, dt**2]],
            dtype=float,
        )
        self.state = f @ self.state
        self.cov = f @ self.cov @ f.T + noise
        self.last_update = now

    def update(self, det: Detection) -> None:
        z = np.array([[det.local_x], [det.local_y]], dtype=float)
        h = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        r = np.eye(2) * 0.08
        residual = z - h @ self.state
        s = h @ self.cov @ h.T + r
        gain = self.cov @ h.T @ np.linalg.inv(s)
        self.state = self.state + gain @ residual
        self.cov = (np.eye(4) - gain @ h) @ self.cov
        self.label = det.label
        self.confidence = det.confidence
        self.missed_frames = 0

    def miss(self) -> None:
        self.missed_frames += 1

    def shift(self, dx: float, dy: float) -> None:
        self.state[0, 0] += dx
        self.state[1, 0] += dy

    def trajectory(self) -> List[Tuple[float, float]]:
        f = np.array([[1, 0, PREDICTION_DT_S, 0], [0, 1, 0, PREDICTION_DT_S], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        state = self.state.copy()
        out = []
        for _ in range(max(1, int(PREDICTION_HORIZON_S / PREDICTION_DT_S))):
            state = f @ state
            out.append((float(state[0, 0]), float(state[1, 0])))
        return out

    def view(self) -> TrackView:
        return TrackView(
            self.track_id,
            self.label,
            self.confidence,
            float(self.state[0, 0]),
            float(self.state[1, 0]),
            float(self.state[2, 0]),
            float(self.state[3, 0]),
            self.missed_frames,
            self.trajectory(),
        )


class Tracker:
    def __init__(self) -> None:
        self.tracks: Dict[int, KalmanTrack] = {}
        self.next_id = 1

    def update(self, detections: List[Detection], now: float) -> List[TrackView]:
        for track in self.tracks.values():
            track.predict_to(now)
        unmatched_tracks = set(self.tracks)
        unmatched_dets = set(range(len(detections)))
        pairs = []
        for tid, track in self.tracks.items():
            tx, ty = float(track.state[0, 0]), float(track.state[1, 0])
            for did, det in enumerate(detections):
                if track.label != det.label:
                    continue
                d = abs(tx - det.local_x) + abs(ty - det.local_y)
                if d <= MATCH_DISTANCE_M:
                    pairs.append((d, tid, did))
        for _, tid, did in sorted(pairs):
            if tid not in unmatched_tracks or did not in unmatched_dets:
                continue
            self.tracks[tid].update(detections[did])
            unmatched_tracks.remove(tid)
            unmatched_dets.remove(did)
        for tid in unmatched_tracks:
            self.tracks[tid].miss()
        for did in unmatched_dets:
            det = detections[did]
            duplicate = False
            for track in self.tracks.values():
                if track.label == det.label:
                    d = abs(float(track.state[0, 0]) - det.local_x) + abs(float(track.state[1, 0]) - det.local_y)
                    if d <= DUPLICATE_SPAWN_DISTANCE_M:
                        duplicate = True
                        break
            if not duplicate:
                self.tracks[self.next_id] = KalmanTrack(self.next_id, det, now)
                self.next_id += 1
        for tid in [tid for tid, tr in self.tracks.items() if tr.missed_frames > MAX_MISSED_FRAMES]:
            del self.tracks[tid]
        return [track.view() for track in self.tracks.values()]

    def shift(self, dx: float, dy: float) -> None:
        for track in self.tracks.values():
            track.shift(dx, dy)


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def body_to_world(right_m: float, forward_m: float, pose: FcPose) -> Tuple[float, float]:
    north = pose.north + forward_m * math.cos(pose.yaw) + right_m * math.sin(pose.yaw)
    east = pose.east + forward_m * math.sin(pose.yaw) - right_m * math.cos(pose.yaw)
    return north, east


def bbox_depth(depth: np.ndarray, bbox: Tuple[int, int, int, int]) -> Optional[float]:
    x1, y1, x2, y2 = bbox
    mx = int((x2 - x1) * DEPTH_CROP_MARGIN)
    my = int((y2 - y1) * DEPTH_CROP_MARGIN)
    for roi in (depth[y1 + my : y2 - my, x1 + mx : x2 - mx], depth[y1:y2, x1:x2]):
        valid = roi[np.isfinite(roi) & (roi >= MIN_DEPTH_M) & (roi <= MAX_DEPTH_M)]
        if valid.size:
            return float(np.median(valid))
    return None


def distance_segment(p, a, b) -> float:
    p, a, b = np.array(p), np.array(a), np.array(b)
    ab = b - a
    den = float(np.dot(ab, ab))
    if den <= 1e-9:
        return float(np.linalg.norm(p - a))
    t = max(0.0, min(1.0, float(np.dot(p - a, ab) / den)))
    return float(np.linalg.norm(p - (a + t * ab)))


def polyline(track: TrackView) -> List[Tuple[float, float]]:
    return [(track.local_x, track.local_y), *track.trajectory]


def move_toward(cur, tgt, step):
    dx, dy = tgt[0] - cur[0], tgt[1] - cur[1]
    dist = abs(dx) + abs(dy)
    if dist <= step or dist <= 1e-6:
        return tgt
    return cur[0] + dx / dist * step, cur[1] + dy / dist * step


def plan_safe(tracks: List[TrackView], last_safe: Optional[Tuple[float, float]], pose: FcPose, local_map: LocalMap):
    drone_local = local_map.drone_local(pose)
    if not tracks:
        return drone_local, None, "no tracks"

    relevant = []
    radius = LOCAL_MAP_HALF_RANGE_M + SAFETY_RADIUS_M + 2.0
    for tr in tracks:
        poly = polyline(tr)
        nearest = min(abs(x - drone_local[0]) + abs(y - drone_local[1]) for x, y in poly)
        if nearest <= radius:
            relevant.append((nearest, poly))
    polys = [poly for _, poly in sorted(relevant)[:MAX_PLANNING_TRACKS]]

    def clearance(pt):
        point_clear = float("inf")
        corridor_clear = float("inf")
        for poly in polys:
            prev = None
            for q in poly:
                point_clear = min(point_clear, abs(pt[0] - q[0]) + abs(pt[1] - q[1]))
                if point_clear < SAFETY_RADIUS_M:
                    return None
                if prev is not None:
                    corridor_clear = min(corridor_clear, distance_segment(pt, prev, q))
                    if corridor_clear < CORRIDOR_RADIUS_M:
                        return None
                prev = q
        return point_clear, corridor_clear

    reference = last_safe or drone_local
    if last_safe is not None:
        c = clearance(last_safe)
        if c and c[0] >= SAFETY_RADIUS_M + SAFE_HOLD_MARGIN_M and c[1] >= CORRIDOR_RADIUS_M + SAFE_HOLD_MARGIN_M:
            return last_safe, CandidateCost(0.0, c[0], c[1]), "keep previous safe point"

    values = np.arange(-LOCAL_MAP_SEARCH_RADIUS_M, LOCAL_MAP_SEARCH_RADIUS_M + 1e-6, SAFE_STEP_M)
    candidates = [drone_local]
    if last_safe:
        candidates.append(last_safe)
    candidates += [(drone_local[0] + float(x), drone_local[1] + float(y)) for x in values for y in values]

    scored = []
    seen = set()
    for pt in candidates:
        key = (round(pt[0], 3), round(pt[1], 3))
        if key in seen or abs(pt[0]) > LOCAL_MAP_HALF_RANGE_M or abs(pt[1]) > LOCAL_MAP_HALF_RANGE_M:
            continue
        seen.add(key)
        c = clearance(pt)
        if not c:
            continue
        pc, cc = c
        total = (
            POINT_CLEARANCE_WEIGHT / max(pc - SAFETY_RADIUS_M, 0.05)
            + CORRIDOR_CLEARANCE_WEIGHT / max(cc - CORRIDOR_RADIUS_M, 0.05)
            + CONTINUITY_WEIGHT * (abs(pt[0] - reference[0]) + abs(pt[1] - reference[1])) / LOCAL_MAP_HALF_RANGE_M
            + DRONE_DISTANCE_WEIGHT * (abs(pt[0] - drone_local[0]) + abs(pt[1] - drone_local[1])) / LOCAL_MAP_HALF_RANGE_M
            + ORIGIN_DISTANCE_WEIGHT * (abs(pt[0]) + abs(pt[1])) / LOCAL_MAP_HALF_RANGE_M
        )
        scored.append((total, pt, pc, cc))
    if not scored:
        return None, None, "no safe candidate"
    _, target, pc, cc = min(scored, key=lambda item: item[0])
    safe = move_toward(last_safe, target, SAFE_MAX_STEP_M) if last_safe else target
    return safe, CandidateCost(float(_), pc, cc), "safe candidate found"


class RealVisionNode(Node):
    def __init__(self, args) -> None:
        super().__init__("real_vision_waypoint_node")
        self.args = args
        self.wp_pub = self.create_publisher(PoseStamped, WAYPOINT_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.model = YOLO(args.model, task="detect")
        self.mav = MavlinkState(args.mavlink, args.baud)
        self.mav.start()
        self.tracker = Tracker()
        self.local_map = LocalMap()
        self.last_safe: Optional[Tuple[float, float]] = None
        self.run_dir = Path(args.log_dir) / f"vision_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.events = (self.run_dir / "vision_events.jsonl").open("w", encoding="utf-8")
        self.summary = (self.run_dir / "vision_summary.csv").open("w", newline="", encoding="utf-8")
        self.summary_writer = csv.DictWriter(
            self.summary,
            fieldnames=[
                "host_ns",
                "frame",
                "tracks",
                "valid",
                "safe_north_m",
                "safe_east_m",
                "uav_north_m",
                "uav_east_m",
                "uav_z_up_m",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
                "reason",
            ],
        )
        self.summary_writer.writeheader()

    def close(self) -> None:
        self.mav.stop.set()
        self.events.close()
        self.summary.close()

    def detect(self, color, depth, intr) -> List[Detection]:
        h, w = color.shape[:2]
        results = self.model.predict(color, imgsz=YOLO_IMGSZ, conf=self.args.conf, iou=YOLO_IOU, device=self.args.device, verbose=False)
        detections = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                x1, x2 = max(0, min(w - 1, x1)), max(0, min(w, x2))
                y1, y2 = max(0, min(h - 1, y1)), max(0, min(h, y2))
                if x2 <= x1 or y2 <= y1:
                    continue
                d = bbox_depth(depth, (x1, y1, x2, y2))
                if d is None:
                    continue
                u, v = (x1 + x2) // 2, (y1 + y2) // 2
                x_img = (u - intr.ppx) / intr.fx * d
                y_img = (v - intr.ppy) / intr.fy * d
                detections.append(Detection(str(self.model.names[int(box.cls[0])]), float(box.conf[0]), (x1, y1, x2, y2), (u, v), d, x_img, -y_img))
        return detections

    def run(self) -> None:
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, self.args.width, self.args.height, rs.format.bgr8, self.args.fps)
        cfg.enable_stream(rs.stream.depth, self.args.width, self.args.height, rs.format.z16, self.args.fps)
        profile = pipeline.start(cfg)
        align = rs.align(rs.stream.color)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        frame_idx = 0
        try:
            while rclpy.ok():
                frames = align.process(pipeline.wait_for_frames())
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                pose = self.mav.latest()
                if not pose.valid:
                    rclpy.spin_once(self, timeout_sec=0.0)
                    continue
                self.local_map.ensure(pose)
                color = np.asanyarray(color_frame.get_data())
                depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
                detections = self.detect(color, depth, intr)
                local_dets = []
                for det in detections:
                    north, east = body_to_world(det.body_right_m, det.body_forward_m, pose)
                    det.local_x, det.local_y = self.local_map.world_to_local(north, east)
                    local_dets.append(det)
                now = time.time()
                tracks = self.tracker.update(local_dets, now)
                drone_local = self.local_map.drone_local(pose)
                if max(abs(drone_local[0]), abs(drone_local[1])) >= LOCAL_MAP_HALF_RANGE_M * LOCAL_MAP_REBASE_FRACTION:
                    old = (self.local_map.origin_north, self.local_map.origin_east)
                    new = (pose.north, pose.east)
                    shift = (old[0] - new[0], old[1] - new[1])
                    self.tracker.shift(*shift)
                    if self.last_safe:
                        self.last_safe = (self.last_safe[0] + shift[0], self.last_safe[1] + shift[1])
                    self.local_map.origin_north, self.local_map.origin_east = new
                    self.local_map.rebase_count += 1
                    tracks = self.tracker.update([], now)
                safe, cost, reason = plan_safe(tracks, self.last_safe, pose, self.local_map)
                self.last_safe = safe
                valid = safe is not None
                if valid:
                    safe_n, safe_e = self.local_map.local_to_world(*safe)
                    self.publish_waypoint(safe_n, safe_e, pose.z_up, pose.yaw)
                else:
                    safe_n = safe_e = None
                self.publish_status(valid, safe_n, safe_e, safe, cost, reason, tracks)
                self.log_frame(frame_idx, pose, valid, safe_n, safe_e, safe, reason, detections, tracks)
                frame_idx += 1
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            pipeline.stop()

    def publish_waypoint(self, north: float, east: float, z_up: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = WAYPOINT_FRAME_ID
        # MAVROS local_position/setpoint_position uses ENU, while raw APM
        # LOCAL_POSITION_NED is north/east/down. Keep logs in APM NED terms,
        # but publish waypoint coordinates in MAVROS ENU.
        msg.pose.position.x = float(east)
        msg.pose.position.y = float(north)
        msg.pose.position.z = float(z_up)
        _, _, qz, qw = yaw_to_quat(yaw)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.wp_pub.publish(msg)

    def publish_status(self, valid, safe_n, safe_e, safe_local, cost, reason, tracks) -> None:
        data = {
            "valid": valid,
            "frame_id": WAYPOINT_FRAME_ID,
            "x": None if safe_e is None else safe_e,
            "y": None if safe_n is None else safe_n,
            "safe_north_m": safe_n,
            "safe_east_m": safe_e,
            "mavros_x_east_m": safe_e,
            "mavros_y_north_m": safe_n,
            "reason": reason,
            "safe_local_x_m": None if safe_local is None else safe_local[0],
            "safe_local_y_m": None if safe_local is None else safe_local[1],
            "tracks": len(tracks),
        }
        if cost:
            data["min_point_clearance_m"] = cost.point_clearance
            data["min_corridor_clearance_m"] = cost.corridor_clearance
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.status_pub.publish(msg)

    def log_frame(self, frame_idx, pose, valid, safe_n, safe_e, safe_local, reason, detections, tracks) -> None:
        host_ns = time.time_ns()
        self.summary_writer.writerow(
            {
                "host_ns": host_ns,
                "frame": frame_idx,
                "tracks": len(tracks),
                "valid": valid,
                "safe_north_m": safe_n,
                "safe_east_m": safe_e,
                "uav_north_m": pose.north,
                "uav_east_m": pose.east,
                "uav_z_up_m": pose.z_up,
                "roll_rad": pose.roll,
                "pitch_rad": pose.pitch,
                "yaw_rad": pose.yaw,
                "reason": reason,
            }
        )
        event = {
            "host_ns": host_ns,
            "frame": frame_idx,
            "uav": pose.__dict__,
            "safe": {"valid": valid, "north_m": safe_n, "east_m": safe_e, "local": safe_local, "reason": reason},
            "detections": [det.__dict__ for det in detections],
            "tracks": [tr.__dict__ for tr in tracks],
            "local_map": self.local_map.__dict__,
        }
        self.events.write(json.dumps(event, ensure_ascii=False) + "\n")
        if frame_idx % 10 == 0:
            self.events.flush()
            self.summary.flush()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--conf", type=float, default=YOLO_CONF)
    parser.add_argument("--device", default="0")
    parser.add_argument("--mavlink", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--log-dir", default="/home/zyc/real_drone/vision_logs")
    return parser.parse_args()


def main() -> int:
    rclpy.init()
    node = RealVisionNode(parse_args())
    try:
        node.run()
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
