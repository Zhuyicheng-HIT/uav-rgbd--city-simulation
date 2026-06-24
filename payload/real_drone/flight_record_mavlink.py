#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Flight-mode MAVROS logger.

This recorder intentionally does not open the D435i or /dev/ttyACM0.
MAVROS owns the flight-controller serial port, and real_vision_node.py owns the
RealSense pipeline when RGB-D db3 recording is required.
"""

import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Float64


FC_FIELDS = [
    "host_ns",
    "host_unix_ns",
    "mode",
    "armed",
    "connected",
    "system_status",
    "local_x_north_m",
    "local_y_east_m",
    "local_z_down_m",
    "local_vx_north_mps",
    "local_vy_east_mps",
    "local_vz_down_mps",
    "lat_deg",
    "lon_deg",
    "alt_msl_m",
    "relative_alt_m",
    "heading_deg",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "battery_voltage_v",
    "battery_current_a",
    "battery_percentage",
]


NAVSAT_FIX_NAMES = {
    -1: "NO_GPS",
    0: "NO_FIX",
    1: "FIX",
    2: "SBAS_FIX",
    3: "GBAS_FIX",
}


def now_ns() -> int:
    return time.monotonic_ns()


def now_unix_ns() -> int:
    return time.time_ns()


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def roll_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def pitch_from_quat(x: float, y: float, z: float, w: float) -> float:
    value = 2.0 * (w * y - z * x)
    return math.asin(max(-1.0, min(1.0, value)))


class MavrosFlightLogger(Node):
    def __init__(self, args) -> None:
        super().__init__("flight_record_mavlink")
        self.args = args
        self.latest = {k: None for k in FC_FIELDS}
        self.latest.update({"host_ns": now_ns(), "host_unix_ns": now_unix_ns()})

        self.raw_file = Path(args.raw_csv).open("w", newline="", encoding="utf-8")
        self.raw_writer = csv.writer(self.raw_file)
        self.raw_writer.writerow(["host_ns", "host_unix_ns", "topic", "msg_json"])

        self.snapshot_file = Path(args.snapshot_csv).open("w", newline="", encoding="utf-8")
        self.snapshot_writer = csv.DictWriter(self.snapshot_file, fieldnames=FC_FIELDS)
        self.snapshot_writer.writeheader()

        self.create_subscription(State, args.state_topic, self._state_cb, 10)
        self.create_subscription(PoseStamped, args.local_pose_topic, self._local_pose_cb, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, args.local_velocity_topic, self._local_velocity_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, args.imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, args.global_position_topic, self._global_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, args.rel_alt_topic, self._rel_alt_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, args.heading_topic, self._heading_cb, qos_profile_sensor_data)
        self.create_subscription(BatteryState, args.battery_topic, self._battery_cb, qos_profile_sensor_data)

    def close(self) -> None:
        self.raw_file.flush()
        self.snapshot_file.flush()
        self.raw_file.close()
        self.snapshot_file.close()

    def _stamp(self) -> None:
        self.latest["host_ns"] = now_ns()
        self.latest["host_unix_ns"] = now_unix_ns()

    def _write_raw(self, topic: str, data: dict) -> None:
        self._stamp()
        self.raw_writer.writerow([self.latest["host_ns"], self.latest["host_unix_ns"], topic, json.dumps(data, ensure_ascii=False)])

    def write_snapshot(self) -> None:
        self._stamp()
        self.snapshot_writer.writerow(dict(self.latest))
        self.snapshot_file.flush()
        self.raw_file.flush()

    def _state_cb(self, msg: State) -> None:
        data = {
            "connected": bool(msg.connected),
            "armed": bool(msg.armed),
            "guided": bool(msg.guided),
            "manual_input": bool(msg.manual_input),
            "mode": msg.mode,
            "system_status": int(msg.system_status),
        }
        self._write_raw(self.args.state_topic, data)
        self.latest.update({
            "mode": msg.mode,
            "armed": bool(msg.armed),
            "connected": bool(msg.connected),
            "system_status": int(msg.system_status),
        })

    def _local_pose_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        roll = roll_from_quat(q.x, q.y, q.z, q.w)
        pitch = pitch_from_quat(q.x, q.y, q.z, q.w)
        yaw_enu = yaw_from_quat(q.x, q.y, q.z, q.w)
        yaw_ned = math.atan2(math.sin(math.pi * 0.5 - yaw_enu), math.cos(math.pi * 0.5 - yaw_enu))
        self._write_raw(self.args.local_pose_topic, {
            "x_east_m": float(p.x),
            "y_north_m": float(p.y),
            "z_up_m": float(p.z),
            "qx": float(q.x),
            "qy": float(q.y),
            "qz": float(q.z),
            "qw": float(q.w),
        })
        self.latest.update({
            "local_x_north_m": float(p.y),
            "local_y_east_m": float(p.x),
            "local_z_down_m": -float(p.z),
            "roll_rad": roll,
            "pitch_rad": pitch,
            "yaw_rad": yaw_ned,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw_ned),
        })

    def _local_velocity_cb(self, msg: TwistStamped) -> None:
        v = msg.twist.linear
        self._write_raw(self.args.local_velocity_topic, {
            "vx_east_mps": float(v.x),
            "vy_north_mps": float(v.y),
            "vz_up_mps": float(v.z),
        })
        self.latest.update({
            "local_vx_north_mps": float(v.y),
            "local_vy_east_mps": float(v.x),
            "local_vz_down_mps": -float(v.z),
        })

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        roll = roll_from_quat(q.x, q.y, q.z, q.w)
        pitch = pitch_from_quat(q.x, q.y, q.z, q.w)
        yaw_enu = yaw_from_quat(q.x, q.y, q.z, q.w)
        yaw_ned = math.atan2(math.sin(math.pi * 0.5 - yaw_enu), math.cos(math.pi * 0.5 - yaw_enu))
        self._write_raw(self.args.imu_topic, {
            "qx": float(q.x),
            "qy": float(q.y),
            "qz": float(q.z),
            "qw": float(q.w),
            "angular_velocity": {
                "x": float(msg.angular_velocity.x),
                "y": float(msg.angular_velocity.y),
                "z": float(msg.angular_velocity.z),
            },
            "linear_acceleration": {
                "x": float(msg.linear_acceleration.x),
                "y": float(msg.linear_acceleration.y),
                "z": float(msg.linear_acceleration.z),
            },
        })
        self.latest.update({
            "roll_rad": roll,
            "pitch_rad": pitch,
            "yaw_rad": yaw_ned,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw_ned),
        })

    def _global_cb(self, msg: NavSatFix) -> None:
        fix_type = int(msg.status.status)
        self._write_raw(self.args.global_position_topic, {
            "lat_deg": float(msg.latitude),
            "lon_deg": float(msg.longitude),
            "alt_msl_m": float(msg.altitude),
            "status": fix_type,
            "fix_name": NAVSAT_FIX_NAMES.get(fix_type, f"UNKNOWN_{fix_type}"),
        })
        self.latest.update({
            "lat_deg": float(msg.latitude),
            "lon_deg": float(msg.longitude),
            "alt_msl_m": float(msg.altitude),
        })

    def _rel_alt_cb(self, msg: Float64) -> None:
        self._write_raw(self.args.rel_alt_topic, {"relative_alt_m": float(msg.data)})
        self.latest.update({"relative_alt_m": float(msg.data)})

    def _heading_cb(self, msg: Float64) -> None:
        self._write_raw(self.args.heading_topic, {"heading_deg": float(msg.data)})
        self.latest.update({"heading_deg": float(msg.data)})

    def _battery_cb(self, msg: BatteryState) -> None:
        self._write_raw(self.args.battery_topic, {
            "voltage_v": float(msg.voltage),
            "current_a": float(msg.current),
            "percentage": float(msg.percentage),
        })
        self.latest.update({
            "battery_voltage_v": float(msg.voltage),
            "battery_current_a": float(msg.current),
            "battery_percentage": float(msg.percentage),
        })


def write_metadata(path: Path, args, run_dir: Path) -> None:
    metadata = {
        "created_time_local": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "program": "flight_record_mavlink.py",
        "purpose": "Record MAVROS flight-controller topics only.",
        "note": "This program does not open D435i and does not open /dev/ttyACM0. It can run together with real_vision_node.py.",
        "outputs": {
            "raw_csv": Path(args.raw_csv).name,
            "snapshot_csv": Path(args.snapshot_csv).name,
        },
        "mavros_topics": {
            "state": args.state_topic,
            "local_pose": args.local_pose_topic,
            "local_velocity": args.local_velocity_topic,
            "imu": args.imu_topic,
            "global_position": args.global_position_topic,
            "relative_altitude": args.rel_alt_topic,
            "heading": args.heading_topic,
            "battery": args.battery_topic,
        },
        "timestamp_policy": {
            "clock": "host monotonic_ns and host unix_ns",
            "snapshot_interval_s": args.snapshot_interval,
        },
        "run_dir": str(run_dir),
    }
    with path.open("w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)


def parse_args():
    parser = argparse.ArgumentParser(description="Flight-mode logger: synchronized MAVROS FC state only.")
    parser.add_argument("--save-dir", default="~/real_drone/flight_records")
    parser.add_argument("--snapshot-interval", type=float, default=0.1)
    parser.add_argument("--print-interval", type=float, default=5.0)
    parser.add_argument("--state-topic", default="/mavros/state")
    parser.add_argument("--local-pose-topic", default="/mavros/local_position/pose")
    parser.add_argument("--local-velocity-topic", default="/mavros/local_position/velocity_local")
    parser.add_argument("--imu-topic", default="/mavros/imu/data")
    parser.add_argument("--global-position-topic", default="/mavros/global_position/global")
    parser.add_argument("--rel-alt-topic", default="/mavros/global_position/rel_alt")
    parser.add_argument("--heading-topic", default="/mavros/global_position/compass_hdg")
    parser.add_argument("--battery-topic", default="/mavros/battery")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.save_dir = str(Path(args.save_dir).expanduser())
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.save_dir) / f"mavros_record_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    args.raw_csv = str(run_dir / "fc_mavros_raw.csv")
    args.snapshot_csv = str(run_dir / "fc_state_snapshots.csv")
    metadata_path = run_dir / "metadata.json"
    write_metadata(metadata_path, args, run_dir)

    print(f"[INFO] MAVROS日志目录: {run_dir}")
    print("[INFO] 本程序只订阅MAVROS话题，不打开D435i，不打开/dev/ttyACM0，不向飞控发送控制指令")
    print("[INFO] Ctrl+C停止记录\n")

    rclpy.init()
    node = MavrosFlightLogger(args)
    last_snapshot = 0.0
    last_print = time.monotonic()
    start = last_print
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_snapshot >= args.snapshot_interval:
                node.write_snapshot()
                last_snapshot = now
            if now - last_print >= args.print_interval:
                elapsed = int(now - start)
                latest = node.latest
                print(
                    f"[记录中] {elapsed // 60}分{elapsed % 60}秒 | "
                    f"mode={latest.get('mode')} armed={latest.get('armed')} "
                    f"connected={latest.get('connected')} | "
                    f"NED=({latest.get('local_x_north_m')}, "
                    f"{latest.get('local_y_east_m')}, "
                    f"{latest.get('local_z_down_m')}) | yaw={latest.get('yaw_deg')}"
                )
                last_print = now
    except KeyboardInterrupt:
        print("\n[INFO] 收到Ctrl+C，正在停止记录...")
    finally:
        node.write_snapshot()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"[SUCCESS] MAVROS原始日志: {args.raw_csv}")
        print(f"[SUCCESS] 状态快照: {args.snapshot_csv}")
        print(f"[SUCCESS] 元数据: {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
