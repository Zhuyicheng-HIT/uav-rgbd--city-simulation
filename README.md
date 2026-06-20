# 无人机 RGB-D 视觉避障仿真系统：快速安装版

本页适合直接复制命令。零基础解释、每个软件的作用、版本原因、排错方法见：

```text
README_详细配置教程.md
```

本项目分两个包：

1. `uav_rgbd_base_sim_package.zip`：基础仿真包，包含 Gazebo 城市场景、ArduPilot/MAVROS、D435i RGB-D 相机、基础脚本。
2. `uav_rgbd_avoidance_extension_package.zip`：避障拓展包，包含 YOLO 视觉避障、仿真状态机、真实飞控轻量程序、录像/回放程序。

安装顺序必须是：先基础包，再拓展包。

## 0. 强烈建议

如果你是 Windows 用户，先用管理员 PowerShell 安装 WSL 和 Ubuntu-22.04：

```powershell
wsl --install -d Ubuntu-22.04
```

安装完成后重启电脑。第一次打开 Ubuntu 时，用户名建议输入：

```text
zyc
```

之后在 Windows PowerShell 里进入 Ubuntu：

```powershell
wsl -d Ubuntu-22.04
```

进入后看到类似 `zyc@电脑名:~$` 的提示符，后面的命令都在这个 Ubuntu 终端里执行。

Ubuntu/WSL 用户名建议使用：

```text
zyc
```

本项目默认验证路径为：

```text
/home/zyc/vision_sim_ws
/home/zyc/vision_avoid
/home/zyc/real_drone
```

如果用户名不是 `zyc`，恢复脚本会尝试替换路径，但不如直接使用 `zyc` 稳定。

## 1. 安装 ROS2 Humble

推荐使用鱼香 ROS 一键安装：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

进入菜单后选择：

```text
安装 ROS
ROS2
humble
桌面版/desktop
```

安装完成后执行：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

## 2. 安装常用依赖

```bash
sudo apt update
sudo apt install -y git wget curl unzip build-essential cmake python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-vcstool

sudo apt install -y ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-cv-bridge ros-humble-image-transport

sudo rosdep init 2>/dev/null || true
rosdep update

python3 -m pip install --user -U pip -i https://pypi.tuna.tsinghua.edu.cn/simple
python3 -m pip install --user ultralytics opencv-python==4.10.0.84 \
  numpy==1.26.4 protobuf==3.20.3 PyYAML \
  -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## 3. 安装 Gazebo Sim

先检查是否已有 `gz`：

```bash
gz sim --help
```

如果没有，执行：

```bash
sudo apt update
sudo apt install -y lsb-release wget gnupg
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install -y gz-harmonic
```

安装后再次确认：

```bash
gz sim --help
```

## 4. 下载并编译 ArduPilot

ArduPilot 必须使用 GitHub 官方地址：

```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

## 5. 解压两个包

如果 zip 在 Windows 下载目录，先在 Ubuntu 终端里复制到 Linux 下载目录：

```bash
mkdir -p ~/Downloads
cp /mnt/c/Users/$USER/Downloads/uav_rgbd_base_sim_package.zip ~/Downloads/ 2>/dev/null || true
cp /mnt/c/Users/$USER/Downloads/uav_rgbd_avoidance_extension_package.zip ~/Downloads/ 2>/dev/null || true
```

如果上面命令找不到文件，请把 `$USER` 换成你的 Windows 用户名，例如：

```bash
cp /mnt/c/Users/你的Windows用户名/Downloads/uav_rgbd_base_sim_package.zip ~/Downloads/
cp /mnt/c/Users/你的Windows用户名/Downloads/uav_rgbd_avoidance_extension_package.zip ~/Downloads/
```

然后执行：

```bash
cd ~/Downloads
unzip uav_rgbd_base_sim_package.zip
unzip uav_rgbd_avoidance_extension_package.zip
```

## 6. 恢复基础包

```bash
cd ~/Downloads/uav_rgbd_base_sim_package
bash tools/restore_to_home.example.sh
```

编译基础仿真工作空间：

```bash
cd ~/vision_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

编译 ArduPilot Gazebo 插件：

```bash
cd ~/ardupilot_gazebo
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

## 7. 恢复拓展包

```bash
cd ~/Downloads/uav_rgbd_avoidance_extension_package
bash tools/restore_to_home.example.sh
```

检查 Python 文件：

```bash
cd ~/vision_avoid
python3 -m py_compile sim_waypoint_node.py sim_landing_state_machine.py waypoint_node.py

cd ~/real_drone
python3 -m py_compile real_vision_node.py real_landing_state_machine.py record_d435i_mavlink.py
```

## 8. 添加环境变量

把下面内容加入 `~/.bashrc`：

```bash
cat >> ~/.bashrc <<'EOF'

# UAV RGB-D simulation environment
source /opt/ros/humble/setup.bash
if [ -f "$HOME/vision_sim_ws/install/setup.bash" ]; then
  source "$HOME/vision_sim_ws/install/setup.bash"
fi
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export GZ_RENDER_ENGINE=ogre2
export GZ_SIM_RESOURCE_PATH="$HOME/vision_sim_ws/models:$HOME/vision_sim_ws/src/vision_people_sim/models:$HOME/ardupilot_gazebo/models:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
EOF

source ~/.bashrc
```

## 9. 快速检查

```bash
python3 - <<'PY'
import cv2, numpy, google.protobuf
print("cv2", cv2.__version__)
print("numpy", numpy.__version__)
print("protobuf", google.protobuf.__version__)
PY

ros2 --help
gz sim --help
test -f ~/vision_avoid/irreality.engine && echo "irreality.engine OK"
test -f ~/vision_avoid/irreality.pt && echo "irreality.pt OK"
```

## 10. 启动仿真

终端 1：

```bash
cd ~/vision_sim_ws
./scripts/run_apm_rgbd_stack.sh
```

终端 2：

```bash
cd ~/vision_avoid
source /opt/ros/humble/setup.bash
source ~/vision_sim_ws/install/setup.bash
python3 sim_landing_state_machine.py
```

终端 3：无人机起飞后启动。

```bash
cd ~/vision_avoid
source /opt/ros/humble/setup.bash
source ~/vision_sim_ws/install/setup.bash
python3 sim_waypoint_node.py
```

