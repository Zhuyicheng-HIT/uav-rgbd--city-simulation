#!/usr/bin/env python3
import json
import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


DEFAULT_TAKEOFF_ALT_M = 12.0
DEFAULT_HOVER_SECONDS = 20.0
DEFAULT_SAFE_OVERHEAD_SECONDS = 60.0
DEFAULT_SETPOINT_RATE_HZ = 10.0
DEFAULT_XY_STEP_M = 0.18
DEFAULT_DESCENT_RATE_MPS = 0.25
DEFAULT_LAND_ALT_M = 0.35


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class SimLandingStateMachine(Node):
    def __init__(self) -> None:
        super().__init__("vision_landing_state_machine")
        self.declare_parameter("takeoff_alt_m", DEFAULT_TAKEOFF_ALT_M)
        self.declare_parameter("hover_seconds", DEFAULT_HOVER_SECONDS)
        self.declare_parameter("safe_overhead_seconds", DEFAULT_SAFE_OVERHEAD_SECONDS)
        self.declare_parameter("setpoint_rate_hz", DEFAULT_SETPOINT_RATE_HZ)
        self.declare_parameter("xy_step_m", DEFAULT_XY_STEP_M)
        self.declare_parameter("descent_rate_mps", DEFAULT_DESCENT_RATE_MPS)
        self.declare_parameter("land_alt_m", DEFAULT_LAND_ALT_M)
        self.declare_parameter("vision_wait_timeout_s", 120.0)
        self.declare_parameter("preflight_wait_seconds", 15.0)
        self.declare_parameter("command_retry_seconds", 45.0)
        self.declare_parameter("takeoff_free_climb_seconds", 20.0)
        self.declare_parameter("mavlink_takeoff_url", "tcp:127.0.0.1:5762")
        self.declare_parameter("mavlink_target_component", 1)
        self.declare_parameter("takeoff_param3", 1.0)
        self.declare_parameter("vision_status_topic", "/vision/avoidance_status")
        self.declare_parameter("vision_waypoint_topic", "/vision/avoidance_waypoint")
        self.declare_parameter("allow_legacy_safe_body", False)
        self.declare_parameter("local_pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("setpoint_topic", "/mavros/setpoint_position/local")
        self.declare_parameter("enable_gazebo_pose_fallback", True)
        self.declare_parameter("gazebo_world_name", "city_apm_rgbd")
        self.declare_parameter("gazebo_model_name", "apm_iris")

        self.takeoff_alt_m = float(self.get_parameter("takeoff_alt_m").value)
        self.hover_seconds = float(self.get_parameter("hover_seconds").value)
        self.safe_overhead_seconds = float(self.get_parameter("safe_overhead_seconds").value)
        self.setpoint_rate_hz = float(self.get_parameter("setpoint_rate_hz").value)
        self.xy_step_m = float(self.get_parameter("xy_step_m").value)
        self.descent_rate_mps = float(self.get_parameter("descent_rate_mps").value)
        self.land_alt_m = float(self.get_parameter("land_alt_m").value)
        self.vision_wait_timeout_s = float(self.get_parameter("vision_wait_timeout_s").value)
        self.preflight_wait_seconds = float(self.get_parameter("preflight_wait_seconds").value)
        self.command_retry_seconds = float(self.get_parameter("command_retry_seconds").value)
        self.takeoff_free_climb_seconds = float(self.get_parameter("takeoff_free_climb_seconds").value)
        self.mavlink_takeoff_url = str(self.get_parameter("mavlink_takeoff_url").value)
        self.mavlink_target_component = int(self.get_parameter("mavlink_target_component").value)
        self.takeoff_param3 = float(self.get_parameter("takeoff_param3").value)
        self.vision_status_topic = str(self.get_parameter("vision_status_topic").value)
        self.vision_waypoint_topic = str(self.get_parameter("vision_waypoint_topic").value)
        self.allow_legacy_safe_body = bool(self.get_parameter("allow_legacy_safe_body").value)
        self.local_pose_topic = str(self.get_parameter("local_pose_topic").value)
        self.setpoint_topic = str(self.get_parameter("setpoint_topic").value)
        self.enable_gazebo_pose_fallback = bool(self.get_parameter("enable_gazebo_pose_fallback").value)
        self.gazebo_world_name = str(self.get_parameter("gazebo_world_name").value)
        self.gazebo_model_name = str(self.get_parameter("gazebo_model_name").value)

        self.state = State()
        self.pose: Optional[PoseStamped] = None
        self.gz_pose: Optional[Tuple[float, float, float, float]] = None
        self.safe_body: Optional[Tuple[float, float]] = None
        self.safe_world: Optional[Tuple[float, float]] = None
        self.safe_body_stamp = 0.0
        self.safe_world_stamp = 0.0
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_z = 0.0
        self.cmd_yaw = 0.0
        self.pose_seen_once = False

        self.create_subscription(State, "/mavros/state", self._state_cb, 10)
        self.create_subscription(PoseStamped, self.local_pose_topic, self._pose_cb, qos_profile_sensor_data)
        self.create_subscription(String, self.vision_status_topic, self._vision_cb, 10)
        self.create_subscription(PoseStamped, self.vision_waypoint_topic, self._waypoint_cb, 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, self.setpoint_topic, 10)

        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_cli = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_cli = self.create_client(CommandTOL, "/mavros/cmd/land")
        self.gz_node = None
        if self.enable_gazebo_pose_fallback:
            self._init_gazebo_pose_fallback()

    def _init_gazebo_pose_fallback(self) -> None:
        try:
            from gz.msgs10.pose_v_pb2 import Pose_V
            from gz.transport13 import Node as GzNode
        except Exception as exc:
            self.get_logger().warning(f"Gazebo pose fallback unavailable: {exc}")
            return
        self.gz_node = GzNode()
        topic = f"/world/{self.gazebo_world_name}/pose/info"
        self.gz_node.subscribe(Pose_V, topic, self._gz_pose_cb)
        self.get_logger().info(f"Gazebo pose fallback subscribed: {topic} model={self.gazebo_model_name}")

    def _state_cb(self, msg: State) -> None:
        self.state = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.pose = msg
        self.pose_seen_once = True

    def _gz_pose_cb(self, msg) -> None:
        for pose in msg.pose:
            if pose.name != self.gazebo_model_name:
                continue
            q = pose.orientation
            yaw = quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
            self.gz_pose = (float(pose.position.x), float(pose.position.y), float(pose.position.z), yaw)
            return

    def _vision_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not data.get("valid", False):
            return
        if self.allow_legacy_safe_body and "safe_body_right_m" in data and "safe_body_forward_m" in data:
            self.safe_body = (float(data["safe_body_right_m"]), float(data["safe_body_forward_m"]))
            self.safe_body_stamp = time.monotonic()

    def _waypoint_cb(self, msg: PoseStamped) -> None:
        self.safe_world = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.safe_world_stamp = time.monotonic()

    def wait_ready(self) -> None:
        self.get_logger().info("Waiting for MAVROS connection...")
        last_log = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state.connected:
                break
            now = time.monotonic()
            if now - last_log >= 2.0:
                self.get_logger().info(f"Still waiting: state.connected={self.state.connected}")
                last_log = now
        for cli, name in [
            (self.mode_cli, "set_mode"),
            (self.arm_cli, "arming"),
            (self.takeoff_cli, "takeoff"),
        ]:
            if not cli.wait_for_service(timeout_sec=20.0):
                raise RuntimeError(f"MAVROS service unavailable: {name}")

    def wait_for_local_pose(self, timeout_s: float = 60.0) -> None:
        self.get_logger().info("Waiting for MAVROS local pose or Gazebo pose fallback after takeoff...")
        last_log = 0.0
        end = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None or self.gz_pose is not None:
                x, y, z, yaw = self.current_xyz_yaw()
                source = "MAVROS" if self.pose is not None else "Gazebo"
                self.get_logger().info(f"{source} pose ready: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={math.degrees(yaw):.1f}deg")
                return
            now = time.monotonic()
            if now - last_log >= 2.0:
                self.get_logger().info("Still waiting for /mavros/local_position/pose or Gazebo pose...")
                last_log = now
        raise RuntimeError("Timed out waiting for MAVROS local pose or Gazebo pose after takeoff.")

    def call(self, cli, req, label: str, timeout: float = 10.0):
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if fut.done():
                resp = fut.result()
                self.get_logger().info(f"{label}: {resp}")
                return resp
        raise RuntimeError(f"Timeout calling {label}")

    def publish_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
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
        self.cmd_x = float(x)
        self.cmd_y = float(y)
        self.cmd_z = float(z)
        self.cmd_yaw = float(yaw)

    def current_xyz_yaw(self) -> Tuple[float, float, float, float]:
        if self.pose is None:
            if self.gz_pose is not None:
                return self.gz_pose
            return self.cmd_x, self.cmd_y, self.cmd_z, self.cmd_yaw
        p = self.pose.pose.position
        q = self.pose.pose.orientation
        return float(p.x), float(p.y), float(p.z), quat_to_yaw(q.x, q.y, q.z, q.w)

    def hold(self, x: float, y: float, z: float, seconds: float, yaw: float = 0.0) -> None:
        period = 1.0 / max(self.setpoint_rate_hz, 1.0)
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.publish_setpoint(x, y, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def spin_without_setpoints(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_state(self, predicate, label: str, timeout: float = 12.0) -> bool:
        period = 1.0 / max(self.setpoint_rate_hz, 1.0)
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            self.publish_setpoint(0.0, 0.0, self.takeoff_alt_m, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                self.get_logger().info(
                    f"{label} confirmed: mode={self.state.mode}, armed={self.state.armed}, connected={self.state.connected}"
                )
                return True
            time.sleep(period)
        self.get_logger().warning(
            f"{label} not confirmed within {timeout:.1f}s: mode={self.state.mode}, armed={self.state.armed}, connected={self.state.connected}"
        )
        return False

    def send_takeoff_command_int(self) -> bool:
        self.get_logger().info(
            f"Sending MAV_CMD_NAV_TAKEOFF COMMAND_INT z={self.takeoff_alt_m:.1f}m via {self.mavlink_takeoff_url}"
        )
        mav = mavutil.mavlink_connection(self.mavlink_takeoff_url, source_system=252)
        mav.wait_heartbeat(timeout=10)
        target_system = mav.target_system or 1
        target_component = self.mavlink_target_component or mav.target_component or 1
        mav.mav.command_int_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0.0,
            0.0,
            self.takeoff_param3,
            0.0,
            0,
            0,
            self.takeoff_alt_m,
        )
        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            msg = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                ok = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                self.get_logger().info(f"takeoff COMMAND_INT ack: result={msg.result}, accepted={ok}")
                mav.close()
                return ok
        mav.close()
        self.get_logger().warning("Timed out waiting for takeoff COMMAND_INT ACK")
        return False

    def set_guided_arm_takeoff(self) -> None:
        self.get_logger().info("Priming origin setpoints...")
        self.hold(0.0, 0.0, self.takeoff_alt_m, self.preflight_wait_seconds)

        deadline = time.monotonic() + self.command_retry_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            if self.state.mode != "GUIDED":
                mode_req = SetMode.Request()
                mode_req.base_mode = 0
                mode_req.custom_mode = "GUIDED"
                mode_resp = self.call(self.mode_cli, mode_req, "set GUIDED")
                if not bool(getattr(mode_resp, "mode_sent", False)):
                    self.get_logger().warning("GUIDED mode command was not accepted; retrying.")
                    self.hold(0.0, 0.0, self.takeoff_alt_m, 2.0)
                    continue

            if not self.wait_for_state(lambda: self.state.mode == "GUIDED", "GUIDED mode", timeout=15.0):
                self.hold(0.0, 0.0, self.takeoff_alt_m, 2.0)
                continue

            if not self.state.armed:
                arm_req = CommandBool.Request()
                arm_req.value = True
                arm_resp = self.call(self.arm_cli, arm_req, "arm")
                if not bool(getattr(arm_resp, "success", False)):
                    self.get_logger().warning("Arm rejected, waiting and retrying.")
                    self.hold(0.0, 0.0, self.takeoff_alt_m, 3.0)
                    continue

            if not self.wait_for_state(lambda: self.state.armed, "armed state", timeout=15.0):
                self.hold(0.0, 0.0, self.takeoff_alt_m, 2.0)
                continue

            if self.state.mode != "GUIDED":
                self.get_logger().warning(f"Mode changed to {self.state.mode}; retrying GUIDED before takeoff.")
                continue

            if self.send_takeoff_command_int():
                self.get_logger().info("Takeoff command accepted after confirmed GUIDED + armed state.")
                return
            self.get_logger().warning("Takeoff rejected, waiting and retrying.")
            self.hold(0.0, 0.0, self.takeoff_alt_m, 3.0)

        raise RuntimeError("Failed to enter GUIDED, arm, and send accepted takeoff command.")

    def safe_body_to_world(self) -> Optional[Tuple[float, float]]:
        if self.safe_world is not None and time.monotonic() - self.safe_world_stamp <= 1.5:
            return self.safe_world
        # 仿真视觉节点现在直接发布真实世界坐标航点。若航点话题暂时不可用，
        # 才使用旧的机体系 safe_body 兼容路径。
        if not self.allow_legacy_safe_body:
            return None
        if self.pose is None and self.gz_pose is None:
            return None
        x, y, _, yaw = self.current_xyz_yaw()
        if self.safe_body is None or time.monotonic() - self.safe_body_stamp > 1.5:
            return x, y
        right_m, forward_m = self.safe_body
        world_x = x + forward_m * math.cos(yaw) + right_m * math.sin(yaw)
        world_y = y + forward_m * math.sin(yaw) - right_m * math.cos(yaw)
        return world_x, world_y

    def wait_for_vision_safe_point(self) -> None:
        self.get_logger().info("Waiting for valid visual safe point...")
        last_log = 0.0
        end = time.monotonic() + self.vision_wait_timeout_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.safe_world is not None and time.monotonic() - self.safe_world_stamp <= 1.5:
                self.get_logger().info(f"Visual safe waypoint ready: x={self.safe_world[0]:.2f}m, y={self.safe_world[1]:.2f}m")
                return
            if self.allow_legacy_safe_body and self.safe_body is not None and time.monotonic() - self.safe_body_stamp <= 1.5:
                self.get_logger().info(f"Visual safe point ready through legacy body path: right={self.safe_body[0]:.2f}m, forward={self.safe_body[1]:.2f}m")
                return
            now = time.monotonic()
            if now - last_log >= 2.0:
                self.get_logger().info("Still waiting for /vision/avoidance_waypoint valid world waypoint...")
                last_log = now
        raise RuntimeError("Timed out waiting for visual safe point. Start sim_waypoint_node.py and check RGB-D topics.")

    def step_xy_toward(self, target_x: float, target_y: float) -> Tuple[float, float]:
        x, y, _, _ = self.current_xyz_yaw()
        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)
        if dist <= self.xy_step_m:
            return target_x, target_y
        return x + dx / dist * self.xy_step_m, y + dy / dist * self.xy_step_m

    def follow_safe_xy(self, seconds: float, z: float) -> None:
        self.get_logger().info("平飞：跟随视觉安全点，到安全点上方但保持高度。")
        period = 1.0 / max(self.setpoint_rate_hz, 1.0)
        end = time.monotonic() + seconds
        last_pose_warn = 0.0
        while rclpy.ok() and time.monotonic() < end:
            target = self.safe_body_to_world()
            if target is None:
                now = time.monotonic()
                if now - last_pose_warn >= 2.0:
                    self.get_logger().warning("No fresh visual world waypoint; holding last setpoint and refusing blind horizontal motion.")
                    last_pose_warn = now
                self.publish_setpoint(self.cmd_x, self.cmd_y, z, self.cmd_yaw)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(period)
                continue
            tx, ty = target
            sx, sy = self.step_xy_toward(tx, ty)
            _, _, _, yaw = self.current_xyz_yaw()
            self.publish_setpoint(sx, sy, z, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def descend_following_safe(self) -> None:
        self.get_logger().info("跟随降落：持续跟随视觉安全点，同时逐步降低高度。")
        period = 1.0 / max(self.setpoint_rate_hz, 1.0)
        z_cmd = self.takeoff_alt_m
        last_pose_warn = 0.0
        while rclpy.ok():
            target = self.safe_body_to_world()
            if target is None:
                now = time.monotonic()
                if now - last_pose_warn >= 2.0:
                    self.get_logger().warning("No fresh visual world waypoint; holding altitude and refusing blind descent.")
                    last_pose_warn = now
                self.publish_setpoint(self.cmd_x, self.cmd_y, self.cmd_z, self.cmd_yaw)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(period)
                continue
            tx, ty = target
            sx, sy = self.step_xy_toward(tx, ty)
            _, _, current_z, yaw = self.current_xyz_yaw()
            z_cmd = max(self.land_alt_m, min(z_cmd, current_z) - self.descent_rate_mps * period)
            self.publish_setpoint(sx, sy, z_cmd, yaw)
            rclpy.spin_once(self, timeout_sec=0.0)
            if current_z <= self.land_alt_m + 0.1:
                break
            time.sleep(period)

        if self.land_cli.wait_for_service(timeout_sec=5.0):
            req = CommandTOL.Request()
            req.min_pitch = 0.0
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            req.altitude = 0.0
            self.call(self.land_cli, req, "LAND")

    def run(self) -> None:
        self.wait_ready()
        self.set_guided_arm_takeoff()
        self.get_logger().info(
            f"起飞命令已接受，先不发布位置 setpoint，让 ArduPilot 自主爬升 {self.takeoff_free_climb_seconds:.1f}s。"
        )
        self.spin_without_setpoints(self.takeoff_free_climb_seconds)
        self.wait_for_local_pose(timeout_s=60.0)
        self.get_logger().info(f"悬停：世界原点上方 {self.takeoff_alt_m:.1f}m，先稳定 {self.hover_seconds:.1f}s。")
        self.hold(0.0, 0.0, self.takeoff_alt_m, self.hover_seconds)
        self.wait_for_vision_safe_point()
        self.follow_safe_xy(self.safe_overhead_seconds, self.takeoff_alt_m)
        self.descend_following_safe()
        self.get_logger().info("流程完成：悬停 -> 平飞 -> 跟随降落。")


def main() -> int:
    rclpy.init()
    node = SimLandingStateMachine()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
