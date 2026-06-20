#!/usr/bin/env bash
set +e

source /opt/ros/humble/setup.bash
source /home/zyc/vision_sim_ws/install/setup.bash

BASE=/home/zyc/vision_sim_ws/debug_logs/orientation_probe
SRC=/home/zyc/vision_sim_ws/src/vision_people_sim/models/d435i_downward_rgbd
mkdir -p "$BASE"

variants=(
  "p_plus|0 0 -0.08 0 1.57079632679 0"
  "p_minus|0 0 -0.08 0 -1.57079632679 0"
  "roll_plus|0 0 -0.08 1.57079632679 0 0"
  "roll_minus|0 0 -0.08 -1.57079632679 0 0"
  "identity|0 0 -0.08 0 0 0"
  "yaw_pi|0 0 -0.08 0 0 3.14159265359"
)

for item in "${variants[@]}"; do
  name="${item%%|*}"
  pose="${item#*|}"
  run_dir="$BASE/$name"
  rm -rf "$run_dir"
  mkdir -p "$run_dir/models/d435i_downward_rgbd" "$run_dir/logs"
  cp -a "$SRC"/. "$run_dir/models/d435i_downward_rgbd/"
  python3 - "$run_dir/models/d435i_downward_rgbd/model.sdf" "$pose" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
pose = sys.argv[2]
text = path.read_text()
text = re.sub(
    r'(<sensor name="d435i_rgbd_down" type="rgbd_camera">\s*<pose>)[^<]+(</pose>)',
    r'\1' + pose + r'\2',
    text,
    count=1,
)
path.write_text(text)
PY
  cat > "$run_dir/world.sdf" <<EOF
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="orientation_probe">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene><ambient>1 1 1 1</ambient><background>0.8 0.9 1 1</background><shadows>false</shadows></scene>
    <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>1 1 1 1</diffuse><direction>-0.3 0.2 -1</direction></light>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <visual name="v"><geometry><box><size>20 20 0.02</size></box></geometry><material><diffuse>0.2 0.6 0.2 1</diffuse></material></visual>
        <collision name="c"><geometry><box><size>20 20 0.02</size></box></geometry></collision>
      </link>
    </model>
    <include><uri>model://d435i_downward_rgbd</uri><name>cam</name><pose>0 0 5 0 0 0</pose></include>
  </world>
</sdf>
EOF
  export GZ_RENDER_ENGINE=ogre2
  export GZ_SIM_RESOURCE_PATH="$run_dir/models:/home/zyc/vision_sim_ws/install/vision_people_sim/share/vision_people_sim/models:${GZ_SIM_RESOURCE_PATH:-}"
  timeout 18s gz sim -s -r --headless-rendering -v 1 "$run_dir/world.sdf" > "$run_dir/logs/gz.log" 2>&1 &
  gz_pid=$!
  sleep 5
  ros2 run vision_people_sim gz_rgbd_bridge > "$run_dir/logs/bridge.log" 2>&1 &
  bridge_pid=$!
  sleep 4
  python3 /home/zyc/vision_sim_ws/scripts/probe_image_stats.py > "$run_dir/logs/stats.log" 2>&1
  kill "$bridge_pid" "$gz_pid" 2>/dev/null || true
  sleep 1
  echo "=== $name ==="
  cat "$run_dir/logs/stats.log"
done
