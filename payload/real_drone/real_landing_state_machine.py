#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real-flight APM landing state machine driven by /vision/avoidance_waypoint."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class RealLandingStateMachine(Node):
    def __init__(self, args) -> None:
        super().__init__("real_vision_landing_state_machine")
        self.args = args
        args.log_dir = str(Path(args.log_dir).expanduser())
        self.state = State()
        self.pose: Optional[PoseStamped] = None
        self.safe_world: Optional[Tuple[float, float, float]] = None
        self.safe_stamp = 0.0
        self.vision_status = {}
        self.cmd_x = self.cmd_y = self.cmd_z = self.cmd_yaw = 0.0

        self.create_subscription(State, args.state_topic, self._state_cb, 10)
        self.create_subscription(PoseStamped, args.local_pose_topic, self._pose_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, args.vision_waypoint_topic, self._waypoint_cb, 10)
        self.create_subscription(String, args.vision_status_topic, self._vision_status_cb, 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, args.setpoint_topic, 10)
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_cli = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_cli = self.create_client(CommandTOL, "/mavros/cmd/land")

        run_dir = Path(args.log_dir) / f"state_machine_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        run_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = (run_dir / "state_machine.csv").open("w", newline="", encoding="utf-8")
        self.log = csv.DictWriter(
            self.log_file,
            fieldnames=[
                "host_ns",
                "phase",
                "mode",
                "armed",
                "uav_x",
                "uav_y",
                "uav_z",
                "yaw",
                "safe_x",
                "safe_y",
                "safe_z",
                "cmd_x",
                "cmd_y",
                "cmd_z",
                "vision_status",
            ],
        )
        self.log.writeheader()

    def close(self):
        self.log_file.close()

    def _state_cb(self, msg: State) -> None:
        self.state = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.pose = msg

    def _waypoint_cb(self, msg: PoseStamped) -> None:
        self.safe_world = (float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z))
        self.safe_stamp = time.monotonic()

    def _vision_status_cb(self, msg: String) -> None:
        try:
            self.vision_status = json.loads(msg.data)
        except Exception:
            self.vision_status = {"raw": msg.data}

    def current_xyz_yaw(self):
        if self.pose is None:
            return self.cmd_x, self.cmd_y, self.cmd_z, self.cmd_yaw
        p = self.pose.pose.position
        q = self.pose.pose.orientation
        return float(p.x), float(p.y), float(p.z), yaw_from_quat(q.x, q.y, q.z, q.w)

    def write_log(self, phase: str):
        x, y, z, yaw = self.current_xyz_yaw()
        sx = sy = sz = None
        if self.safe_world is not None:
            sx, sy, sz = self.safe_world
        self.log.writerow(
            {
                "host_ns": time.time_ns(),
                "phase": phase,
                "mode": self.state.mode,
                "armed": self.state.armed,
                "uav_x": x,
                "uav_y": y,
                "uav_z": z,
                "yaw": yaw,
                "safe_x": sx,
                "safe_y": sy,
                "safe_z": sz,
                "cmd_x": self.cmd_x,
                "cmd_y": self.cmd_y,
                "cmd_z": self.cmd_z,
                "vision_status": json.dumps(self.vision_status, ensure_ascii=False),
            }
        )
        self.log_file.flush()

    def call(self, cli, req, label: str):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        self.get_logger().info(f"{label}: {res}")
        return res

    def wait_ready(self) -> None:
        self.get_logger().info("Waiting for MAVROS connection and local pose...")
        last_log = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state.connected and self.pose is not None:
                break
            now = time.monotonic()
            if now - last_log >= 2.0:
                last_log = now
                self.get_logger().info(
                    f"Still waiting: state.connected={self.state.connected}, pose_seen={self.pose is not None}"
                )
        for cli in (self.mode_cli, self.arm_cli, self.takeoff_cli):
            cli.wait_for_service(timeout_sec=20.0)

    def set_guided_arm_takeoff(self) -> None:
        end = time.monotonic() + self.args.command_retry_seconds
        while rclpy.ok() and time.monotonic() < end:
            req = SetMode.Request()
            req.custom_mode = self.args.guided_mode
            self.call(self.mode_cli, req, "GUIDED")
            time.sleep(0.5)
            arm = CommandBool.Request()
            arm.value = True
            self.call(self.arm_cli, arm, "ARM")
            time.sleep(0.5)
            takeoff = CommandTOL.Request()
            takeoff.altitude = float(self.args.takeoff_alt_m)
            if self.call(self.takeoff_cli, takeoff, "TAKEOFF"):
                return
        raise RuntimeError("Failed to arm/takeoff")

    def publish_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        self.cmd_x, self.cmd_y, self.cmd_z, self.cmd_yaw = x, y, z, yaw
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        _, _, qz, qw = yaw_to_quat(yaw)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.setpoint_pub.publish(msg)

    def hold(self, seconds: float, phase: str) -> None:
        x, y, _, yaw = self.current_xyz_yaw()
        period = 1.0 / self.args.setpoint_rate_hz
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.publish_setpoint(x, y, self.args.takeoff_alt_m, yaw)
            self.write_log(phase)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def wait_vision(self) -> None:
        end = time.monotonic() + self.args.vision_wait_timeout_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.safe_world is not None and time.monotonic() - self.safe_stamp <= 1.5:
                return
            self.write_log("wait_vision")
        raise RuntimeError("Timed out waiting for vision waypoint")

    def step_xy(self, tx: float, ty: float) -> Tuple[float, float]:
        x, y, _, _ = self.current_xyz_yaw()
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if dist <= self.args.xy_step_m:
            return tx, ty
        return x + dx / dist * self.args.xy_step_m, y + dy / dist * self.args.xy_step_m

    def follow_xy(self, seconds: float) -> None:
        period = 1.0 / self.args.setpoint_rate_hz
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            if self.safe_world is not None and time.monotonic() - self.safe_stamp <= 1.5:
                sx, sy = self.step_xy(self.safe_world[0], self.safe_world[1])
            else:
                sx, sy, _, _ = self.current_xyz_yaw()
            _, _, _, yaw = self.current_xyz_yaw()
            self.publish_setpoint(sx, sy, self.args.takeoff_alt_m, yaw)
            self.write_log("follow_xy")
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def descend(self) -> None:
        period = 1.0 / self.args.setpoint_rate_hz
        z_cmd = self.args.takeoff_alt_m
        while rclpy.ok():
            if self.safe_world is not None and time.monotonic() - self.safe_stamp <= 1.5:
                sx, sy = self.step_xy(self.safe_world[0], self.safe_world[1])
            else:
                sx, sy, _, _ = self.current_xyz_yaw()
            _, _, current_z, yaw = self.current_xyz_yaw()
            z_cmd = max(self.args.land_alt_m, min(z_cmd, current_z) - self.args.descent_rate_mps * period)
            self.publish_setpoint(sx, sy, z_cmd, yaw)
            self.write_log("descend")
            rclpy.spin_once(self, timeout_sec=0.0)
            if current_z <= self.args.land_alt_m + 0.1:
                break
            time.sleep(period)
        if self.land_cli.wait_for_service(timeout_sec=5.0):
            land = CommandTOL.Request()
            self.call(self.land_cli, land, "LAND")

    def run(self) -> None:
        self.wait_ready()
        time.sleep(self.args.preflight_wait_seconds)
        self.set_guided_arm_takeoff()
        time.sleep(self.args.takeoff_free_climb_seconds)
        self.hold(self.args.hover_seconds, "hover")
        self.wait_vision()
        self.follow_xy(self.args.safe_overhead_seconds)
        self.descend()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--takeoff-alt-m", type=float, default=12.0)
    p.add_argument("--hover-seconds", type=float, default=20.0)
    p.add_argument("--safe-overhead-seconds", type=float, default=60.0)
    p.add_argument("--setpoint-rate-hz", type=float, default=10.0)
    p.add_argument("--xy-step-m", type=float, default=0.18)
    p.add_argument("--descent-rate-mps", type=float, default=0.25)
    p.add_argument("--land-alt-m", type=float, default=0.35)
    p.add_argument("--vision-wait-timeout-s", type=float, default=120.0)
    p.add_argument("--preflight-wait-seconds", type=float, default=15.0)
    p.add_argument("--takeoff-free-climb-seconds", type=float, default=20.0)
    p.add_argument("--command-retry-seconds", type=float, default=45.0)
    p.add_argument("--guided-mode", default="GUIDED")
    p.add_argument("--state-topic", default="/mavros/state")
    p.add_argument("--local-pose-topic", default="/mavros/local_position/pose")
    p.add_argument("--setpoint-topic", default="/mavros/setpoint_position/local")
    p.add_argument("--vision-waypoint-topic", default="/vision/avoidance_waypoint")
    p.add_argument("--vision-status-topic", default="/vision/avoidance_status")
    p.add_argument("--log-dir", default="~/real_drone/vision_logs")
    return p.parse_args()


def main() -> int:
    rclpy.init()
    node = RealLandingStateMachine(parse_args())
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
