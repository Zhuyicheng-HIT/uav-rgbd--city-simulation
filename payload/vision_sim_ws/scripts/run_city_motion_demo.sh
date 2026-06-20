#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/zyc/vision_sim_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/zyc/vision_sim_ws/install/vision_people_sim/share/vision_people_sim/models:/home/zyc/ardupilot_gazebo/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/zyc/ardupilot_gazebo/build

gz sim -r -v 3 --render-engine-gui ogre /home/zyc/vision_sim_ws/install/vision_people_sim/share/vision_people_sim/worlds/city_people.sdf &
gz_pid=
cleanup() {
  kill "" "" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
sleep 6
ros2 run vision_people_sim people_motion --ros-args --params-file /home/zyc/vision_sim_ws/city_full_demo_params.yaml &
motion_pid=
wait ""
