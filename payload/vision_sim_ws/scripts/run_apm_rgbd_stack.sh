#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/zyc/vision_sim_ws/install/setup.bash

export GZ_SIM_RESOURCE_PATH=/home/zyc/vision_sim_ws/install/vision_people_sim/share/vision_people_sim/models:/home/zyc/ardupilot_gazebo/models:/home/zyc/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/zyc/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
export GZ_RENDER_ENGINE=ogre2
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

WORLD=/home/zyc/vision_sim_ws/install/vision_people_sim/share/vision_people_sim/worlds/city_apm_rgbd.sdf
LOG_DIR=/home/zyc/vision_sim_ws/logs/apm_rgbd_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG_DIR"

pids=()
cleanup() {
  printf '\nStopping APM RGB-D stack...\n'
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
    kill -- "-$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]:-}"; do
    kill -TERM "$pid" 2>/dev/null || true
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]:-}"; do
    kill -9 "$pid" 2>/dev/null || true
    kill -9 -- "-$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

printf 'Logs: %s\n' "$LOG_DIR"
printf 'Starting Gazebo city + APM Iris + downward RGB-D camera...\n'
if [[ "${HEADLESS:-0}" == "1" ]]; then
  setsid gz sim -s -r --headless-rendering -v 2 "$WORLD" >"$LOG_DIR/gazebo.log" 2>&1 &
else
  setsid gz sim -r -v 2 --render-engine-gui ogre2 "$WORLD" >"$LOG_DIR/gazebo.log" 2>&1 &
fi
pids+=("$!")
sleep 6

printf 'Starting ArduPilot SITL gazebo-iris model...\n'
setsid bash -lc 'cd /home/zyc/ardupilot && build/sitl/bin/arducopter -S --model JSON --speedup 1 --slave 0 --defaults Tools/autotest/default_params/copter.parm,Tools/autotest/default_params/gazebo-iris.parm,/home/zyc/vision_sim_ws/vision_mavros_guided.parm --sim-address=127.0.0.1 -I0' >"$LOG_DIR/sitl.log" 2>&1 &
pids+=("$!")
sleep 10

printf 'Starting MAVROS on tcp://127.0.0.1:5760...\n'
setsid ros2 run mavros mavros_node --ros-args \
  --params-file /opt/ros/humble/share/mavros/launch/apm_config.yaml \
  --params-file /opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml \
  --params-file /home/zyc/vision_sim_ws/install/vision_people_sim/share/vision_people_sim/config/mavros_apm_rgbd.yaml \
  >"$LOG_DIR/mavros.log" 2>&1 &
pids+=("$!")
sleep 4

printf 'Starting pedestrians and vehicles motion...\n'
setsid ros2 run vision_people_sim people_motion --ros-args --params-file /home/zyc/vision_sim_ws/city_apm_motion_params.yaml >"$LOG_DIR/people_motion.log" 2>&1 &
pids+=("$!")

printf 'Starting Gazebo RGB-D camera mount follower...\n'
setsid ros2 run vision_people_sim rgbd_camera_follow --ros-args \
  -p offset_xyz:="[0.24, 0.0, -0.35]" \
  -p follow_attitude:=false \
  -p camera_rpy:="[0.0, 0.0, 0.0]" \
  -p rate_hz:=30.0 \
  -p timeout_ms:=2 \
  >"$LOG_DIR/rgbd_camera_follow.log" 2>&1 &
pids+=("$!")

printf 'Starting native Gazebo RGB-D -> ROS2/D435i bridge...\n'
setsid python3 /home/zyc/vision_sim_ws/scripts/run_rgbd_bridge_when_airborne.py >"$LOG_DIR/gz_rgbd_bridge.log" 2>&1 &
pids+=("$!")

cat <<EOF

APM + Gazebo + MAVROS + downward D435i-style RGB-D stack is running.

Useful checks in another terminal:
  source /opt/ros/humble/setup.bash && source /home/zyc/vision_sim_ws/install/setup.bash
  python3 /home/zyc/vision_sim_ws/scripts/probe_stack.py
  ros2 topic list --no-daemon | grep camera
  ros2 topic hz /camera/camera/depth/image_rect_raw
  ros2 run vision_people_sim guided_flight --ros-args -p takeoff_alt:=4.0 -p side_length:=4.0

D435i-style ROS topics:
  /camera/camera/color/image_raw
  /camera/camera/color/camera_info
  /camera/camera/depth/image_rect_raw
  /camera/camera/depth/camera_info
  /camera/camera/aligned_depth_to_color/image_raw

Raw bridged Gazebo topics remain available under:
  /camera/camera/image
  /camera/camera/depth_image
  /camera/camera/camera_info

Press Ctrl+C in this terminal to stop the stack.
EOF

wait "${pids[0]}"
