#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/zyc/vision_sim_ws/install/setup.bash
printf '\n--- MAVROS + depth probe ---\n'
timeout 20s python3 /home/zyc/vision_sim_ws/scripts/probe_stack.py || true
printf '\n--- camera topics ---\n'
ros2 topic list --no-daemon | grep -E '(^/camera/camera|^/mavros)' | sort | sed -n '1,120p'
printf '\n--- depth rate quick check ---\n'
timeout 8s ros2 topic hz /camera/camera/depth/image_rect_raw || true
