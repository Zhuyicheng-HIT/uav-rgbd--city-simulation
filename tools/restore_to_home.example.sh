#!/usr/bin/env bash
set -eo pipefail

# Run from extracted package root.
# Recommended Ubuntu/WSL username: zyc.

PKG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$PKG_ROOT/payload"

if [[ ! -d "$SRC" ]]; then
  echo "payload/ not found. Run this script from package root."
  exit 1
fi

copy_dir() {
  local name="$1"
  if [[ -d "$SRC/$name" ]]; then
    mkdir -p "$HOME/$name"
    cp -a "$SRC/$name/." "$HOME/$name/"
    echo "Restored $name -> $HOME/$name"
  fi
}

copy_dir vision_sim_ws
copy_dir vision_avoid
copy_dir real_drone
copy_dir ardupilot_gazebo

if [[ -d "$SRC/vision_sim_yolo_dataset" ]]; then
  cp -a "$SRC/vision_sim_yolo_dataset" "$HOME/"
fi

for dir in "$HOME/vision_sim_ws" "$HOME/vision_avoid" "$HOME/real_drone"; do
  [[ -d "$dir" ]] || continue
  find "$dir" -type f \
    \( -name "*.py" -o -name "*.sh" -o -name "*.md" -o -name "*.yaml" -o -name "*.yml" -o -name "*.launch.py" -o -name "*.sdf" -o -name "*.world" \) \
    -exec perl -pi -e 's#/home/zyc#$ENV{HOME}#g; s#/home/ld666#$ENV{HOME}#g' {} \;
done

for dir in "$HOME/vision_sim_ws/scripts" "$HOME/vision_avoid" "$HOME/real_drone"; do
  [[ -d "$dir" ]] || continue
  find "$dir" -type f \( -name "*.sh" -o -name "*.py" \) -exec chmod +x {} \;
done

if [[ -d "$HOME/vision_sim_ws/src" ]]; then
  source /opt/ros/humble/setup.bash
  cd "$HOME/vision_sim_ws"
  colcon build --symlink-install
fi

echo "Restore complete."

