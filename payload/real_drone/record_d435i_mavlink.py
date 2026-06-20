#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import math
import os
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import pyrealsense2 as rs
from pymavlink import mavutil


GPS_FIX_NAMES = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
}


FC_FIELDS = [
    "fc_host_ns",
    "fc_host_unix_ns",

    "mode",
    "armed",
    "system_status",

    "time_boot_ms",

    # ArduPilot / MAVLink 常用本地坐标：LOCAL_NED
    "local_x_north_m",
    "local_y_east_m",
    "local_z_down_m",
    "local_vx_north_mps",
    "local_vy_east_mps",
    "local_vz_down_mps",

    # 全局坐标
    "lat_deg",
    "lon_deg",
    "alt_msl_m",
    "relative_alt_m",
    "global_vx_north_mps",
    "global_vy_east_mps",
    "global_vz_down_mps",
    "heading_deg",

    # 姿态
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",

    # 四元数，MAVLink ATTITUDE_QUATERNION: q1 q2 q3 q4
    "q1",
    "q2",
    "q3",
    "q4",

    # GPS / RTK 状态
    "gps_fix_type",
    "gps_fix_name",
    "satellites_visible",
    "h_acc_m",
    "v_acc_m",

    # 电池状态
    "battery_voltage_v",
    "battery_current_a",
]


NUMERIC_INTERP_FIELDS = {
    "time_boot_ms",

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
    "global_vx_north_mps",
    "global_vy_east_mps",
    "global_vz_down_mps",
    "heading_deg",

    "roll_rad",
    "pitch_rad",
    "roll_deg",
    "pitch_deg",

    "q1",
    "q2",
    "q3",
    "q4",

    "h_acc_m",
    "v_acc_m",

    "battery_voltage_v",
    "battery_current_a",
}


ANGLE_FIELDS_RAD = {"yaw_rad"}
ANGLE_FIELDS_DEG = {"yaw_deg"}


def now_ns() -> int:
    return time.monotonic_ns()


def now_unix_ns() -> int:
    return time.time_ns()


def shortest_angle_interp(a, b, alpha, period):
    if a is None or b is None:
        return None
    diff = (b - a + period / 2.0) % period - period / 2.0
    return a + alpha * diff


def safe_getattr(obj, name, default=None):
    return getattr(obj, name, default)


class FCStateBuffer:
    def __init__(self, maxlen=10000):
        self.lock = threading.Lock()
        self.buf = deque(maxlen=maxlen)
        self.latest = {k: None for k in FC_FIELDS}

    def update(self, updates: dict):
        t_ns = now_ns()
        t_unix_ns = now_unix_ns()

        with self.lock:
            self.latest["fc_host_ns"] = t_ns
            self.latest["fc_host_unix_ns"] = t_unix_ns
            self.latest.update(updates)
            self.buf.append(dict(self.latest))

    def latest_copy(self) -> dict:
        with self.lock:
            return dict(self.latest)

    def get_at(self, t_ns: int) -> dict:
        """
        按主机 monotonic_ns 对齐。
        优先使用前后两条飞控状态插值；没有前后状态时退化为最近邻。
        """
        with self.lock:
            states = list(self.buf)

        if not states:
            empty = {k: None for k in FC_FIELDS}
            empty["fc_match_mode"] = "none"
            empty["fc_time_error_ms"] = None
            return empty

        before = None
        after = None

        for s in reversed(states):
            if s.get("fc_host_ns") is not None and s["fc_host_ns"] <= t_ns:
                before = s
                break

        for s in states:
            if s.get("fc_host_ns") is not None and s["fc_host_ns"] >= t_ns:
                after = s
                break

        if before is None:
            nearest = states[0]
            out = dict(nearest)
            out["fc_match_mode"] = "nearest_after"
            out["fc_time_error_ms"] = (nearest["fc_host_ns"] - t_ns) / 1e6
            return out

        if after is None:
            nearest = states[-1]
            out = dict(nearest)
            out["fc_match_mode"] = "nearest_before"
            out["fc_time_error_ms"] = (nearest["fc_host_ns"] - t_ns) / 1e6
            return out

        tb = before["fc_host_ns"]
        ta = after["fc_host_ns"]

        if ta == tb:
            out = dict(before)
            out["fc_match_mode"] = "exact"
            out["fc_time_error_ms"] = 0.0
            return out

        alpha = (t_ns - tb) / (ta - tb)
        nearest = before if abs(t_ns - tb) <= abs(ta - t_ns) else after

        out = {}
        for k in FC_FIELDS:
            vb = before.get(k)
            va = after.get(k)

            if k in ANGLE_FIELDS_RAD and vb is not None and va is not None:
                out[k] = shortest_angle_interp(float(vb), float(va), alpha, 2.0 * math.pi)
            elif k in ANGLE_FIELDS_DEG and vb is not None and va is not None:
                out[k] = shortest_angle_interp(float(vb), float(va), alpha, 360.0)
            elif k in NUMERIC_INTERP_FIELDS and vb is not None and va is not None:
                try:
                    out[k] = float(vb) + alpha * (float(va) - float(vb))
                except Exception:
                    out[k] = nearest.get(k)
            else:
                out[k] = nearest.get(k)

        out["fc_match_mode"] = "interp"
        out["fc_time_error_ms"] = 0.0
        return out


def request_message_interval(master, msg_id: int, hz: float):
    if hz <= 0:
        return

    interval_us = int(1_000_000 / hz)

    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )
    except Exception as e:
        print(f"[WARN] 请求 MAVLink 消息频率失败 msg_id={msg_id}: {e}")


def mavlink_reader(conn_str: str, baud: int, state_buf: FCStateBuffer,
                   raw_csv_path: Path, stop_event: threading.Event):
    print(f"[MAVLink] 连接飞控: {conn_str}, baud={baud}")

    try:
        master = mavutil.mavlink_connection(conn_str, baud=baud, autoreconnect=True)
        print("[MAVLink] 等待 heartbeat...")
        master.wait_heartbeat()
        print(f"[MAVLink] heartbeat OK: system={master.target_system}, component={master.target_component}")
    except Exception as e:
        print(f"[ERROR] MAVLink 连接失败: {e}")
        return

    # 请求常用状态频率。不响应也没关系，飞控原本发什么就记录什么。
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 50)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2)

    with raw_csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["host_ns", "host_unix_ns", "msg_type", "msg_json"])

        while not stop_event.is_set():
            msg = master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue

            msg_type = msg.get_type()
            host_ns = now_ns()
            host_unix_ns = now_unix_ns()

            try:
                msg_json = json.dumps(msg.to_dict(), ensure_ascii=False)
            except Exception:
                msg_json = repr(msg)

            writer.writerow([host_ns, host_unix_ns, msg_type, msg_json])

            updates = {}

            if msg_type == "HEARTBEAT":
                try:
                    mode = mavutil.mode_string_v10(msg)
                except Exception:
                    mode = None

                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                updates.update({
                    "mode": mode,
                    "armed": armed,
                    "system_status": safe_getattr(msg, "system_status"),
                })

            elif msg_type == "LOCAL_POSITION_NED":
                updates.update({
                    "time_boot_ms": safe_getattr(msg, "time_boot_ms"),
                    "local_x_north_m": msg.x,
                    "local_y_east_m": msg.y,
                    "local_z_down_m": msg.z,
                    "local_vx_north_mps": msg.vx,
                    "local_vy_east_mps": msg.vy,
                    "local_vz_down_mps": msg.vz,
                })

            elif msg_type == "GLOBAL_POSITION_INT":
                hdg = None if msg.hdg == 65535 else msg.hdg / 100.0

                updates.update({
                    "time_boot_ms": safe_getattr(msg, "time_boot_ms"),
                    "lat_deg": msg.lat / 1e7,
                    "lon_deg": msg.lon / 1e7,
                    "alt_msl_m": msg.alt / 1000.0,
                    "relative_alt_m": msg.relative_alt / 1000.0,
                    "global_vx_north_mps": msg.vx / 100.0,
                    "global_vy_east_mps": msg.vy / 100.0,
                    "global_vz_down_mps": msg.vz / 100.0,
                    "heading_deg": hdg,
                })

            elif msg_type == "ATTITUDE":
                updates.update({
                    "time_boot_ms": safe_getattr(msg, "time_boot_ms"),
                    "roll_rad": msg.roll,
                    "pitch_rad": msg.pitch,
                    "yaw_rad": msg.yaw,
                    "roll_deg": math.degrees(msg.roll),
                    "pitch_deg": math.degrees(msg.pitch),
                    "yaw_deg": math.degrees(msg.yaw),
                })

            elif msg_type == "ATTITUDE_QUATERNION":
                updates.update({
                    "time_boot_ms": safe_getattr(msg, "time_boot_ms"),
                    "q1": msg.q1,
                    "q2": msg.q2,
                    "q3": msg.q3,
                    "q4": msg.q4,
                })

            elif msg_type == "GPS_RAW_INT":
                fix_type = int(msg.fix_type)
                h_acc_raw = safe_getattr(msg, "h_acc", None)
                v_acc_raw = safe_getattr(msg, "v_acc", None)

                h_acc_m = None
                v_acc_m = None

                if h_acc_raw is not None and h_acc_raw != 0xFFFFFFFF:
                    h_acc_m = h_acc_raw / 1000.0

                if v_acc_raw is not None and v_acc_raw != 0xFFFFFFFF:
                    v_acc_m = v_acc_raw / 1000.0

                updates.update({
                    "gps_fix_type": fix_type,
                    "gps_fix_name": GPS_FIX_NAMES.get(fix_type, f"UNKNOWN_{fix_type}"),
                    "satellites_visible": safe_getattr(msg, "satellites_visible"),
                    "h_acc_m": h_acc_m,
                    "v_acc_m": v_acc_m,
                })

            elif msg_type == "SYS_STATUS":
                voltage_raw = safe_getattr(msg, "voltage_battery", None)
                current_raw = safe_getattr(msg, "current_battery", None)

                voltage_v = None
                current_a = None

                if voltage_raw is not None and voltage_raw != 65535:
                    voltage_v = voltage_raw / 1000.0

                if current_raw is not None and current_raw != -1:
                    current_a = current_raw / 100.0

                updates.update({
                    "battery_voltage_v": voltage_v,
                    "battery_current_a": current_a,
                })

            if updates:
                state_buf.update(updates)


def intrinsics_to_dict(intr):
    return {
        "width": intr.width,
        "height": intr.height,
        "ppx": intr.ppx,
        "ppy": intr.ppy,
        "fx": intr.fx,
        "fy": intr.fy,
        "model": str(intr.model),
        "coeffs": list(intr.coeffs),
    }


def write_metadata(path: Path, args, db3_path: Path, profile, depth_scale):
    device = profile.get_device()

    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()

    color_intr = color_profile.get_intrinsics()
    depth_intr = depth_profile.get_intrinsics()

    device_info = {}
    for info in [
        rs.camera_info.name,
        rs.camera_info.serial_number,
        rs.camera_info.firmware_version,
        rs.camera_info.product_id,
    ]:
        try:
            device_info[str(info)] = device.get_info(info)
        except Exception:
            pass

    metadata = {
        "created_time_local": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "db3_file": str(db3_path.name),

        "realsense": {
            "device_info": device_info,
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
            "color_format": "bgr8",
            "depth_format": "z16",
            "depth_scale_m_per_unit": depth_scale,
            "color_intrinsics": intrinsics_to_dict(color_intr),
            "depth_intrinsics": intrinsics_to_dict(depth_intr),
        },

        "mavlink": {
            "connection": args.mavlink,
            "baud": args.baud,
        },

        "coordinate_frames": {
            "local_ned": {
                "x": "North, meter",
                "y": "East, meter",
                "z": "Down, meter",
            },
            "body_frd": {
                "x_b": "aircraft forward",
                "y_b": "aircraft right",
                "z_b": "aircraft down",
            },
            "camera_optical": {
                "x_c": "image right",
                "y_c": "image down",
                "z_c": "camera optical axis / depth direction",
            },
            "camera_mount": {
                "direction": "downward",
                "image_top": "aircraft forward",
                "image_right": "aircraft right",
                "camera_to_body_formula": {
                    "x_b": "-y_c",
                    "y_b": "x_c",
                    "z_b": "z_c",
                },
                "R_body_camera": [
                    [0, -1, 0],
                    [1, 0, 0],
                    [0, 0, 1],
                ],
                "t_body_camera_m": [
                    args.cam_x,
                    args.cam_y,
                    args.cam_z,
                ],
                "formula": "P_body = R_body_camera @ P_camera + t_body_camera",
            },
        },

        "timestamp_policy": {
            "main_sync_clock": "host monotonic_ns",
            "frame_host_ns": "recorded immediately after pipeline.wait_for_frames() returns",
            "fc_host_ns": "recorded immediately when each MAVLink message is received",
            "frame_sync": "frames_sync.csv uses interpolated or nearest FC state by host monotonic_ns",
            "sync_delay_ms": args.sync_delay_ms,
            "note": "RealSense timestamp and MAVLink time_boot_ms are saved, but direct subtraction is not used.",
        },
    }

    with path.open("w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)


def main():
    parser = argparse.ArgumentParser(description="Record D435i RGBD db3 and synchronized MAVLink FC state.")

    parser.add_argument("--save-dir", default="d435i_flight_records")
    parser.add_argument("--mavlink", default="/dev/ttyACM)",
                        help="飞控连接，例如 /dev/ttyUSB0 或 udp:127.0.0.1:14550")
    parser.add_argument("--baud", type=int, default=57600)

    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)

    parser.add_argument("--sync-delay-ms", type=float, default=100.0,
                        help="为了拿到飞控前后两侧数据，延迟写入每帧同步行，默认 100ms")
    parser.add_argument("--print-interval", type=float, default=5.0)

    # 相机相对飞控/重心的安装偏移，机体系 FRD：前右下为正
    parser.add_argument("--cam-x", type=float, default=0.0, help="相机相对飞控/重心前后偏移，向前为正，单位 m")
    parser.add_argument("--cam-y", type=float, default=0.0, help="相机相对飞控/重心左右偏移，向右为正，单位 m")
    parser.add_argument("--cam-z", type=float, default=0.0, help="相机相对飞控/重心上下偏移，向下为正，单位 m")

    args = parser.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.save_dir) / f"record_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=True)

    db3_path = run_dir / f"record_{timestamp}.db3"
    frames_csv_path = run_dir / "frames_sync.csv"
    fc_raw_csv_path = run_dir / "fc_raw.csv"
    metadata_path = run_dir / "metadata.json"

    print(f"[INFO] 录制目录: {run_dir}")
    print(f"[INFO] RealSense db3: {db3_path}")
    print("[INFO] Ctrl+C 停止录制")
    print("[INFO] 本程序只读取飞控状态，不向飞控发送控制指令\n")

    state_buf = FCStateBuffer()
    stop_event = threading.Event()

    mav_thread = threading.Thread(
        target=mavlink_reader,
        args=(args.mavlink, args.baud, state_buf, fc_raw_csv_path, stop_event),
        daemon=True,
    )
    mav_thread.start()

    pipeline = rs.pipeline()
    cfg = rs.config()

    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    # 保留你原来程序的做法：直接让 RealSense SDK 录制原始 RGB + Depth 到 db3
    cfg.enable_record_to_file(str(db3_path))

    started = False
    frame_count = 0
    start_time = time.time()
    last_print = start_time
    pending_frames = deque()
    sync_delay_ns = int(args.sync_delay_ms * 1e6)

    frame_fields = [
        "frame_idx",
        "frame_host_ns",
        "frame_host_unix_ns",

        "color_frame_number",
        "depth_frame_number",
        "color_rs_timestamp_ms",
        "depth_rs_timestamp_ms",
        "color_timestamp_domain",
        "depth_timestamp_domain",

        "db3_file",

        "fc_match_mode",
        "fc_time_error_ms",
    ] + FC_FIELDS

    try:
        profile = pipeline.start(cfg)
        started = True

        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        write_metadata(metadata_path, args, db3_path, profile, depth_scale)

        print("[INFO] 相机启动成功，开始录制")
        print(f"[INFO] 深度尺度 depth_scale = {depth_scale} m/unit")
        print(f"[INFO] 帧同步表: {frames_csv_path}")
        print(f"[INFO] 飞控原始日志: {fc_raw_csv_path}\n")

        with frames_csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=frame_fields)
            writer.writeheader()

            while True:
                frames = pipeline.wait_for_frames()

                frame_host_ns = now_ns()
                frame_host_unix_ns = now_unix_ns()

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    continue

                row = {
                    "frame_idx": frame_count,
                    "frame_host_ns": frame_host_ns,
                    "frame_host_unix_ns": frame_host_unix_ns,

                    "color_frame_number": color_frame.get_frame_number(),
                    "depth_frame_number": depth_frame.get_frame_number(),
                    "color_rs_timestamp_ms": color_frame.get_timestamp(),
                    "depth_rs_timestamp_ms": depth_frame.get_timestamp(),
                    "color_timestamp_domain": str(color_frame.get_frame_timestamp_domain()),
                    "depth_timestamp_domain": str(depth_frame.get_frame_timestamp_domain()),

                    "db3_file": db3_path.name,
                }

                pending_frames.append(row)
                frame_count += 1

                # 延迟写入，给飞控线程留出“后一个状态点”，这样可以做插值
                flush_deadline_ns = now_ns() - sync_delay_ns

                while pending_frames and pending_frames[0]["frame_host_ns"] <= flush_deadline_ns:
                    frame_row = pending_frames.popleft()
                    fc_state = state_buf.get_at(frame_row["frame_host_ns"])

                    out_row = dict(frame_row)
                    out_row["fc_match_mode"] = fc_state.get("fc_match_mode")
                    out_row["fc_time_error_ms"] = fc_state.get("fc_time_error_ms")

                    for k in FC_FIELDS:
                        out_row[k] = fc_state.get(k)

                    writer.writerow(out_row)

                now = time.time()
                if now - last_print >= args.print_interval:
                    cost = int(now - start_time)
                    m = cost // 60
                    s = cost % 60

                    latest = state_buf.latest_copy()

                    fix = latest.get("gps_fix_name")
                    mode = latest.get("mode")
                    armed = latest.get("armed")

                    lx = latest.get("local_x_north_m")
                    ly = latest.get("local_y_east_m")
                    lz = latest.get("local_z_down_m")
                    yaw = latest.get("yaw_deg")

                    print(
                        f"[录制中] {m}分{s}秒 | 相机帧 {frame_count} | "
                        f"mode={mode} armed={armed} fix={fix} | "
                        f"NED=({lx}, {ly}, {lz}) | yaw={yaw}"
                    )

                    last_print = now

    except KeyboardInterrupt:
        print("\n[INFO] 收到 Ctrl+C，正在停止录制...")

    except Exception as e:
        print(f"[ERROR] 录制异常: {e}")

    finally:
        stop_event.set()

        if started:
            # 停止相机前，把 pending 的帧也尽量写完
            # 这里重新打开 append 写入，避免主循环异常时丢尾部索引
            try:
                existing = frames_csv_path.exists() and frames_csv_path.stat().st_size > 0
                with frames_csv_path.open("a", newline="", encoding="utf-8") as f:
                    writer = csv.DictWriter(f, fieldnames=frame_fields)
                    if not existing:
                        writer.writeheader()

                    while pending_frames:
                        frame_row = pending_frames.popleft()
                        fc_state = state_buf.get_at(frame_row["frame_host_ns"])

                        out_row = dict(frame_row)
                        out_row["fc_match_mode"] = fc_state.get("fc_match_mode")
                        out_row["fc_time_error_ms"] = fc_state.get("fc_time_error_ms")

                        for k in FC_FIELDS:
                            out_row[k] = fc_state.get(k)

                        writer.writerow(out_row)
            except Exception as e:
                print(f"[WARN] 写入尾部帧同步表失败: {e}")

            try:
                pipeline.stop()
            except Exception:
                pass

            print(f"\n[SUCCESS] db3录制完成: {db3_path}")
            print(f"[SUCCESS] 帧同步表: {frames_csv_path}")
            print(f"[SUCCESS] 飞控原始日志: {fc_raw_csv_path}")
            print(f"[SUCCESS] 元数据: {metadata_path}")
            print("Windows读取前转换命令示例：rs-convert -i xxx.db3 -B win.bag")
        else:
            print("\n[ERROR] 相机启动失败，无 db3 文件生成")


if __name__ == "__main__":
    main()