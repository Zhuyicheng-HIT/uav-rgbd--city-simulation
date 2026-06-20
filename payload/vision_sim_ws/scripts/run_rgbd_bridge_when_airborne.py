#!/usr/bin/env python3
import os
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
import time
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V

def main():
    world = os.environ.get("RGBD_WORLD", "city_apm_rgbd")
    camera_model = os.environ.get("RGBD_CAMERA_MODEL", "apm_iris_d435i")
    min_z = float(os.environ.get("RGBD_BRIDGE_MIN_Z", "1.0"))
    timeout = float(os.environ.get("RGBD_BRIDGE_WAIT_TIMEOUT", "180.0"))
    node = Node()
    seen_z = None

    def cb(msg):
        nonlocal seen_z
        for pose in msg.pose:
            if pose.name == camera_model:
                seen_z = pose.position.z
                return

    node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", cb)
    start = time.monotonic()
    print(f"Waiting for {camera_model} z >= {min_z:.2f} before starting gz_rgbd_bridge...", flush=True)
    while time.monotonic() - start < timeout:
        if seen_z is not None and seen_z >= min_z:
            print(f"Starting gz_rgbd_bridge at {camera_model} z={seen_z:.2f}", flush=True)
            os.execvp("python3", ["python3", "/home/zyc/vision_sim_ws/scripts/gz_rgbd_latest_bridge.py", "--ros-args", "-p", "publish_hz:=15.0"])
        time.sleep(0.1)
    print(f"Timed out waiting for {camera_model}; last z={seen_z}. Starting bridge anyway.", flush=True)
    os.execvp("python3", ["python3", "/home/zyc/vision_sim_ws/scripts/gz_rgbd_latest_bridge.py", "--ros-args", "-p", "publish_hz:=15.0"])

if __name__ == "__main__":
    main()
