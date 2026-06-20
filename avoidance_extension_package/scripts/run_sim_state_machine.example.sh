#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/vision_sim_ws/install/setup.bash"

cd "$HOME/vision_avoid"
python3 sim_landing_state_machine.py

