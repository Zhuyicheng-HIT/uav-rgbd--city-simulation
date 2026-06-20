#!/usr/bin/env bash
set -euo pipefail

export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION="${PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION:-python}"
export GZ_RENDER_ENGINE="${GZ_RENDER_ENGINE:-ogre2}"

source /opt/ros/humble/setup.bash
source "$HOME/vision_sim_ws/install/setup.bash"

cd "$HOME/vision_sim_ws"
./scripts/run_apm_rgbd_stack.sh

