#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

cd "$HOME/real_drone"
python3 real_landing_state_machine.py

