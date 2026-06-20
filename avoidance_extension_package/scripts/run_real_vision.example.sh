#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

cd "$HOME/real_drone"
python3 real_vision_node.py

