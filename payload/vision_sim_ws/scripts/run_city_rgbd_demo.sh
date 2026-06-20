#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/zyc/vision_sim_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/zyc/vision_sim_ws/install/vision_people_sim/share/vision_people_sim/models:/home/zyc/ardupilot_gazebo/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/zyc/ardupilot_gazebo/build

pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

gz sim -r -v 3 --render-engine-gui ogre /home/zyc/vision_sim_ws/install/vision_people_sim/share/vision_people_sim/worlds/city_people.sdf &
pids+=("$!")
sleep 6

ros2 run vision_people_sim people_motion --ros-args --params-file /home/zyc/vision_sim_ws/city_full_demo_params.yaml &
pids+=("$!")

ros2 run ros_gz_bridge parameter_bridge   /camera/camera/image@sensor_msgs/msg/Image@gz.msgs.Image   /camera/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image   /camera/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo   /camera/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked &
pids+=("$!")

printf '
RGB-D demo running. ROS2 topics bridged from Gazebo:
'
printf '  /camera/camera/image        # RGB image, sensor_msgs/msg/Image
'
printf '  /camera/camera/depth_image  # depth image, sensor_msgs/msg/Image
'
printf '  /camera/camera/camera_info  # intrinsics, sensor_msgs/msg/CameraInfo
'
printf '  /camera/camera/points       # point cloud, sensor_msgs/msg/PointCloud2
'
printf '
Gazebo source topics have the same names. Press Ctrl+C here to stop everything.
'
wait "${pids[0]}"
