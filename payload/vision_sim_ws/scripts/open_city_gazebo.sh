#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/zyc/vision_sim_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/zyc/vision_sim_ws/install/vision_people_sim/share/vision_people_sim/models:/home/zyc/ardupilot_gazebo/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/zyc/ardupilot_gazebo/build
exec gz sim -r -v 3 --render-engine-gui ogre /home/zyc/vision_sim_ws/install/vision_people_sim/share/vision_people_sim/worlds/city_people.sdf
