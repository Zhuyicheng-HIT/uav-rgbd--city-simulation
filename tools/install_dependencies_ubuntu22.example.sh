#!/usr/bin/env bash
set -eo pipefail

# Optional helper for Ubuntu 22.04 / WSL2.
# Review before running. ROS/Gazebo/PyTorch versions may need adjustment.

sudo apt update
sudo apt install -y curl gnupg lsb-release git build-essential cmake wget unzip \
  python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool

sudo apt install -y ros-humble-desktop ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-cv-bridge ros-humble-image-transport

sudo rosdep init 2>/dev/null || true
rosdep update

python3 -m pip install --user -U pip -i https://pypi.tuna.tsinghua.edu.cn/simple
python3 -m pip install --user ultralytics opencv-python==4.10.0.84 \
  numpy==1.26.4 protobuf==3.20.3 PyYAML \
  -i https://pypi.tuna.tsinghua.edu.cn/simple

cat <<'NOTE'

Manual steps:
1. Install Gazebo Sim so `gz sim` works.
2. Install CUDA-compatible PyTorch for your GPU.
3. Build ArduPilot SITL from official GitHub:
   cd ~
   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
   cd ardupilot
   Tools/environment_install/install-prereqs-ubuntu.sh -y
   . ~/.profile
   ./waf configure --board sitl
   ./waf copter
4. Restore this package:
   bash restore_to_home.sh

NOTE

