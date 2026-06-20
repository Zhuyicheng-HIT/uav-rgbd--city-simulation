#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/zyc/vision_sim_ws/install/setup.bash

LOG_DIR=/home/zyc/vision_sim_ws/logs/white_city_10m_demo_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG_DIR"

echo "Logs: $LOG_DIR"

setsid /home/zyc/vision_sim_ws/scripts/run_apm_rgbd_stack.sh >"$LOG_DIR/stack.log" 2>&1 &
stack_pid=$!
echo "$stack_pid" > "$LOG_DIR/stack.pid"

echo "Waiting for MAVROS and RGB-D frames..."
sleep 65
python3 /home/zyc/vision_sim_ws/scripts/probe_stack.py | tee "$LOG_DIR/probe_before_takeoff.log"

echo "Moving APM Iris model to 10m for downward camera capture..."
setsid python3 /home/zyc/vision_sim_ws/scripts/lift_apm_iris_to_10m.py city_apm_rgbd apm_iris 10 8 3600 >"$LOG_DIR/lift_10m.log" 2>&1 &
lift_pid=$!
echo "$lift_pid" > "$LOG_DIR/lift.pid"

sleep 12
python3 /home/zyc/vision_sim_ws/scripts/probe_gz_pose.py city_apm_rgbd apm_iris 5 | tee "$LOG_DIR/pose_after_lift.log"

echo "Opening RGB image window..."
setsid ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw >"$LOG_DIR/rqt_color.log" 2>&1 &
echo "$!" > "$LOG_DIR/rqt_color.pid"

sleep 2

echo "Opening depth image window..."
setsid ros2 run rqt_image_view rqt_image_view /camera/camera/depth/image_rect_raw >"$LOG_DIR/rqt_depth.log" 2>&1 &
echo "$!" > "$LOG_DIR/rqt_depth.pid"

echo
cat <<EOF
Demo is running.
Gazebo GUI: white buildings + moving pedestrians/vehicles + APM Iris
UAV command: Gazebo smooth lift/hold target 10m for camera capture
Image windows:
  RGB   /camera/camera/color/image_raw
  Depth /camera/camera/depth/image_rect_raw

To check status:
  source /opt/ros/humble/setup.bash && source /home/zyc/vision_sim_ws/install/setup.bash
  python3 /home/zyc/vision_sim_ws/scripts/probe_stack.py
  python3 /home/zyc/vision_sim_ws/scripts/probe_gz_pose.py city_apm_rgbd apm_iris 5

To stop this demo, close the windows and Ctrl+C the stack terminal if it is foreground, or kill PIDs in:
  $LOG_DIR
EOF
