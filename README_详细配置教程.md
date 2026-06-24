# 无人机 RGB-D 视觉避障仿真系统：详细配置教程

本文面向零基础学生，目标是从一台新的 Ubuntu 22.04 / WSL2 机器开始，完整配置无人机城市仿真、D435i RGB-D 深度相机、YOLO 识别、ArduPilot/MAVROS 飞控接口、视觉避障状态机和真实飞控轻量程序。

如果你只想复制命令快速安装，请先看：

```text
README.md
```

## 1. 项目是什么

本项目模拟一架搭载 Intel RealSense D435i 深度相机的无人机，在城市道路环境中识别行人和车辆，预测目标轨迹，寻找安全降落点，并通过飞控状态机执行“起飞、悬停、平飞、跟随安全点降落”流程。

项目分为两个压缩包：

### 1.1 基础仿真包

文件名：

```text
uav_rgbd_base_sim_package.zip
```

功能：

- 提供 Gazebo 城市场景。
- 提供无人机、道路、建筑、行人、车辆、D435i 深度相机模型。
- 连接 ArduPilot SITL 和 MAVROS。
- 发布 RGB 图像、深度图、相机内参和飞控话题。
- 提供基础启动脚本和检查脚本。

### 1.2 避障拓展包

文件名：

```text
uav_rgbd_avoidance_extension_package.zip
```

功能：

- YOLO 检测行人/车辆。
- 读取深度图计算目标三维坐标。
- 根据无人机位置和姿态，把目标转换到局部真实坐标系。
- 预测目标轨迹，生成安全降落点。
- 仿真状态机控制无人机起飞、悬停、平飞、降落。
- 提供真实飞控轻量程序。
- 提供 D435i 录像、MAVROS 飞控话题记录和离线回放程序。

## 2. 软件之间的关系

### Ubuntu 22.04

操作系统。ROS2 Humble、Gazebo、ArduPilot、MAVROS 都在这里运行。

### ROS2 Humble

机器人通信框架。图像、深度、IMU、无人机位姿、安全点都通过 ROS2 topic 传递。

### Gazebo Sim

三维仿真器。负责显示城市环境、无人机、车辆、行人和相机画面。

### ArduPilot SITL

飞控软件的仿真版本。它模拟真实 APM 飞控，让我们不用真实无人机也能测试起飞、悬停和降落。

### MAVROS

连接 ArduPilot 和 ROS2 的桥。视觉程序和状态机通过 MAVROS 获取无人机状态，也通过 MAVROS 控制无人机。

### D435i RGB-D 相机

RGB-D 表示既有彩色图，也有深度图。YOLO 在彩色图上识别目标，深度图用于计算目标相对无人机的三维位置。

### YOLO

目标检测模型。仿真中使用专门适配场景的 `irreality.pt` 和 `irreality.engine`。

### TensorRT engine

YOLO 的 GPU 加速格式。`irreality.engine` 比 `.pt` 更适合实时推理。如果电脑不支持，也可以回退到 `.pt`。

## 3. Windows 用户：安装 WSL2 和 Ubuntu-22.04

如果你已经在 Ubuntu 22.04 里，可以跳过本节。如果你是 Windows 用户，从这里开始。

### 3.1 什么是 WSL

WSL 是 Windows Subsystem for Linux，意思是在 Windows 里运行 Linux。我们用 Windows 打开仿真图形界面，同时用 Ubuntu 运行 ROS2、Gazebo、ArduPilot 和 Python 程序。

### 3.2 打开 PowerShell

1. 点击 Windows 开始菜单。
2. 搜索 `PowerShell`。
3. 右键 `Windows PowerShell`。
4. 选择“以管理员身份运行”。

后面标注为 `powershell` 的命令，都在 PowerShell 里执行。

### 3.3 安装 WSL 和 Ubuntu-22.04

在管理员 PowerShell 中执行：

```powershell
wsl --install -d Ubuntu-22.04
```

安装完成后，按提示重启电脑。

如果提示 WSL 已安装，可以检查：

```powershell
wsl --list --online
wsl --list --verbose
```

如果列表里没有 Ubuntu-22.04，可以执行：

```powershell
wsl --install -d Ubuntu-22.04
```

### 3.4 第一次启动 Ubuntu

在 PowerShell 中执行：

```powershell
wsl -d Ubuntu-22.04
```

第一次启动会要求创建 Linux 用户名和密码。

推荐用户名：

```text
zyc
```

密码输入时屏幕不会显示星号，也不会显示字符，这是 Linux 正常现象。输入完按回车即可。

以后看到类似下面的提示符，就说明你已经进入 Ubuntu：

```text
zyc@电脑名:~$
```

本文后面所有 `bash` 命令，都在这个 Ubuntu 终端里执行，不是在 Windows PowerShell 里执行。

### 3.5 Windows 和 Ubuntu 文件路径关系

Windows 的 C 盘在 Ubuntu 里对应：

```text
/mnt/c
```

例如 Windows 下载目录：

```text
C:\Users\你的Windows用户名\Downloads
```

在 Ubuntu 里通常是：

```text
/mnt/c/Users/你的Windows用户名/Downloads
```

Ubuntu 自己的用户目录是：

```text
~
```

如果用户名是 `zyc`，`~` 等价于：

```text
/home/zyc
```

所以：

```text
~/Downloads
```

等价于：

```text
/home/zyc/Downloads
```

### 3.6 把 zip 从 Windows 复制到 Ubuntu

假设两个 zip 已经下载到 Windows 下载目录，先在 Ubuntu 里创建下载目录：

```bash
mkdir -p ~/Downloads
```
手动复制：把两个压缩包拖过来


自动复制（不推荐）：

```bash
cp /mnt/c/Users/$USER/Downloads/uav_rgbd_base_sim_package.zip ~/Downloads/ 2>/dev/null || true
cp /mnt/c/Users/$USER/Downloads/uav_rgbd_avoidance_extension_package.zip ~/Downloads/ 2>/dev/null || true
```

如果没有复制成功，说明 Windows 用户名和 Ubuntu 用户名不同。手动把 `你的Windows用户名` 替换成真实 Windows 用户名：

```bash
cp /mnt/c/Users/你的Windows用户名/Downloads/uav_rgbd_base_sim_package.zip ~/Downloads/
cp /mnt/c/Users/你的Windows用户名/Downloads/uav_rgbd_avoidance_extension_package.zip ~/Downloads/
```

检查：

```bash
ls -lh ~/Downloads/*.zip
```

### 3.7 常见 Windows/WSL 问题

如果 PowerShell 提示找不到 `wsl`，请升级 Windows 或启用“适用于 Linux 的 Windows 子系统”。

如果 WSL 图形界面打不开 Gazebo，请确认：

- Windows 版本支持 WSLg。
- 显卡驱动较新。
- 终端中能执行 `echo $DISPLAY`，并有输出。

如果 Ubuntu 忘记密码，新手最简单的处理方式通常是重装 Ubuntu-22.04；但这会删除 Ubuntu 内的数据，重装前先备份重要文件。

## 4. 推荐环境和路径

推荐系统：

```text
Ubuntu 22.04 / WSL2
ROS2 Humble
Python 3.10
Gazebo Sim
ArduPilot Copter SITL
```

推荐用户名：

```text
zyc
```

推荐路径：

```text
/home/zyc/vision_sim_ws
/home/zyc/vision_avoid
/home/zyc/real_drone
/home/zyc/ardupilot
/home/zyc/ardupilot_gazebo
```

为什么建议用户名是 `zyc`：

- 当前工程就是在 `/home/zyc` 下验证的。
- Gazebo、脚本、模型路径和日志路径曾经因为用户名不同出过问题。
- 恢复脚本可以替换路径，但对零基础学生来说，直接使用 `zyc` 最稳。

如果你已经使用其他用户名，也可以继续安装，但遇到路径问题时先检查：

```bash
grep -R "/home/zyc" -n ~/vision_sim_ws ~/vision_avoid ~/real_drone 2>/dev/null
```

## 5. Linux 基础概念

### 5.1 终端

终端就是输入命令的窗口。Windows 里有 PowerShell，Ubuntu 里有 Linux shell。本文大部分命令都在 Ubuntu shell 中执行。

### 5.2 sudo

`sudo` 表示用管理员权限执行命令。第一次执行会要求输入 Ubuntu 用户密码。输入密码时屏幕不显示字符，这是正常现象。

### 5.3 apt

`apt` 是 Ubuntu 的软件安装工具，例如：

```bash
sudo apt install git
```

### 5.4 pip

`pip` 是 Python 的软件包安装工具，例如安装 OpenCV、YOLO。

### 5.5 source

`source` 用来加载环境。ROS2 和本项目工作空间都需要 source：

```bash
source /opt/ros/humble/setup.bash
source ~/vision_sim_ws/install/setup.bash
```

### 5.6 三个终端

仿真运行时一般要开三个 Ubuntu 终端：

- 终端 1：Gazebo、ArduPilot、MAVROS、相机桥接。
- 终端 2：飞控状态机。
- 终端 3：视觉避障程序。

在 Windows 中可以打开多个 PowerShell 窗口，每个窗口执行：

```powershell
wsl -d Ubuntu-22.04
```

这样就得到多个 Ubuntu 终端。

## 6. 安装 ROS2 Humble

推荐使用鱼香 ROS 一键安装。它适合新手，会自动处理很多源和依赖问题。

执行：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

菜单中选择：

```text
安装 ROS
ROS2
humble
桌面版/desktop
```

安装后检查：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

如果 `ros2 --help` 能输出帮助信息，说明 ROS2 安装成功。

## 7. 安装 ROS2 常用工具和 MAVROS

执行：

```bash
sudo apt update
sudo apt install -y git wget curl unzip build-essential cmake python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-vcstool

sudo apt install -y ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-cv-bridge ros-humble-image-transport
```

说明：

- `colcon` 用来编译 ROS2 工作空间。
- `rosdep` 用来安装 ROS 包依赖。
- `mavros` 用来连接 ArduPilot 飞控。
- `cv_bridge` 用来在 ROS 图像消息和 OpenCV 图像之间转换。

初始化 `rosdep`：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

## 8. 安装 Python 视觉依赖

执行：

```bash
python3 -m pip install --user -U pip -i https://pypi.tuna.tsinghua.edu.cn/simple
python3 -m pip install --user ultralytics opencv-python==4.10.0.84 \
  numpy==1.26.4 protobuf==3.20.3 PyYAML \
  -i https://pypi.tuna.tsinghua.edu.cn/simple
```

为什么固定版本：

- `numpy==1.26.4`：避免升级到 2.x 后 OpenCV、ROS Python 扩展、TensorRT 周边库不兼容。
- `protobuf==3.20.3`：避免 Gazebo Python 绑定出现 protobuf 解析问题。
- `opencv-python==4.10.0.84`：用于显示图像和处理深度图。
- `ultralytics`：YOLO 推理框架。

检查：

```bash
python3 - <<'PY'
import cv2, numpy, google.protobuf
print("cv2", cv2.__version__)
print("numpy", numpy.__version__)
print("protobuf", google.protobuf.__version__)
PY
```

## 9. 安装 PyTorch

YOLO 需要 PyTorch。PyTorch 是否使用 GPU，取决于显卡和 CUDA。

先检查显卡：

```bash
nvidia-smi
```

如果能看到 NVIDIA 显卡信息，说明有 GPU 环境。PyTorch 建议到官网选择与你显卡匹配的安装命令：

```text
https://pytorch.org/get-started/locally/
```

如果你不确定怎么选，可以先使用 CUDA 12.1 版本。多数较新的 NVIDIA 驱动都能支持：

```bash
python3 -m pip install --user torch torchvision torchaudio \
  --index-url https://download.pytorch.org/whl/cu121
```

如果没有 NVIDIA GPU，使用 CPU 版本：

```bash
python3 -m pip install --user torch torchvision torchaudio \
  --index-url https://download.pytorch.org/whl/cpu
```

安装后检查：

```bash
python3 - <<'PY'
import torch
print("torch", torch.__version__)
print("cuda", torch.version.cuda)
print("cuda available", torch.cuda.is_available())
PY
```

如果 `torch.cuda.is_available()` 是 `True`，说明 YOLO 可以使用 GPU。

如果是 `False`，程序仍可运行，但 YOLO 会慢，实时性可能不足。

## 10. 安装 Gazebo Sim

Gazebo 用来运行三维仿真世界。

先检查：

```bash
gz sim --help
```

如果有帮助信息，说明已安装。

如果没有，请按 Gazebo 官方教程安装 Ubuntu 22.04 支持的版本。安装完成后再次检查：

```bash
sudo apt update
sudo apt install -y lsb-release wget gnupg
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install -y gz-harmonic
```

检查：

```bash
gz sim --help
```

注意：

- 不同电脑 Gazebo 版本可能不同。
- 本项目脚本使用 `gz sim` 命令。
- 如果 Gazebo 图形界面卡顿，先确认显卡驱动和 WSL 图形支持正常。

## 11. 下载并编译 ArduPilot

ArduPilot 是飞控程序。它很大，不放在 zip 包里，需要单独下载。

GitHub 地址必须使用官方裸连路径：

```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

说明：

- `--recurse-submodules` 会下载子模块。
- `install-prereqs-ubuntu.sh` 会安装 ArduPilot 编译依赖。
- `./waf copter` 编译多旋翼无人机 SITL。

如果下载很慢，可以多试几次；不建议把 ArduPilot 改成来历不明的镜像地址。

## 12. 解压两个发布包

如果你按 Windows/WSL 章节操作，两个文件已经在 `~/Downloads`。如果没有，请先把两个文件放到 Ubuntu 的 `~/Downloads`：

```text
uav_rgbd_base_sim_package.zip
uav_rgbd_avoidance_extension_package.zip
```

检查：

```bash
ls -lh ~/Downloads/uav_rgbd_base_sim_package.zip
ls -lh ~/Downloads/uav_rgbd_avoidance_extension_package.zip
```

解压：

```bash
cd ~/Downloads
unzip uav_rgbd_base_sim_package.zip
unzip uav_rgbd_avoidance_extension_package.zip
```

解压后应该看到：

```text
~/Downloads/uav_rgbd_base_sim_package
~/Downloads/uav_rgbd_avoidance_extension_package
```

## 13. 恢复基础仿真包

执行：

```bash
cd ~/Downloads/uav_rgbd_base_sim_package
bash tools/restore_to_home.example.sh
```

恢复后应出现：

```text
~/vision_sim_ws
~/ardupilot_gazebo
```

检查：

```bash
ls ~/vision_sim_ws
ls ~/ardupilot_gazebo
```

编译 ROS2 工作空间：

```bash
cd ~/vision_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 14. 编译 ArduPilot Gazebo 插件

执行：

```bash
cd ~/ardupilot_gazebo
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

插件的作用是让 Gazebo 中的无人机模型和 ArduPilot SITL 通信。

## 15. 恢复避障拓展包

执行：

```bash
cd ~/Downloads/uav_rgbd_avoidance_extension_package
bash tools/restore_to_home.example.sh
```

恢复后应出现：

```text
~/vision_avoid
~/real_drone
```

检查：

```bash
ls ~/vision_avoid
ls ~/real_drone
```

确认仿真模型存在：

```bash
test -f ~/vision_avoid/irreality.pt && echo "irreality.pt OK"
test -f ~/vision_avoid/irreality.engine && echo "irreality.engine OK"
```

## 16. 配置环境变量

环境变量告诉系统去哪里找 ROS 包、Gazebo 模型和插件。

执行：

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

变量说明：

- `source /opt/ros/humble/setup.bash`：启用 ROS2 Humble。
- `source ~/vision_sim_ws/install/setup.bash`：启用本项目 ROS2 工作空间。
- `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python`：避开 protobuf/Gazebo Python 兼容问题。
- `GZ_RENDER_ENGINE=ogre2`：指定 Gazebo 渲染引擎。
- `GZ_SIM_RESOURCE_PATH`：告诉 Gazebo 去哪里找模型。
- `GZ_SIM_SYSTEM_PLUGIN_PATH`：告诉 Gazebo 去哪里找插件。

## 17. 编译和语法检查

基础工作空间：

```bash
cd ~/vision_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

避障程序：

```bash
cd ~/vision_avoid
python3 -m py_compile sim_waypoint_node.py sim_landing_state_machine.py waypoint_node.py
```

实机程序：

```bash
cd ~/real_drone
python3 -m py_compile real_vision_node.py real_landing_state_machine.py flight_record_mavlink.py record_d435i_mavlink.py
```

## 18. 运行前检查

检查 ROS2：

```bash
ros2 --help
```

检查 Gazebo：

```bash
gz sim --help
```

检查 Python：

```bash
python3 - <<'PY'
import cv2, numpy, google.protobuf
print("cv2", cv2.__version__)
print("numpy", numpy.__version__)
print("protobuf", google.protobuf.__version__)
PY
```

检查模型：

```bash
ls -lh ~/vision_avoid/irreality.pt ~/vision_avoid/irreality.engine
```

检查工作空间：

```bash
ls ~/vision_sim_ws/install/setup.bash
```

## 19. 启动仿真

建议每次重新运行前先关闭旧 Gazebo、MAVROS、ArduPilot 进程，避免内存占用累积。

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

## 20. ROS 话题检查

查看话题：

```bash
ros2 topic list --no-daemon | grep -E 'camera/camera|mavros|vision'
```

检查图像帧率：

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

检查 MAVROS：

```bash
ros2 topic echo /mavros/state --once
```

注意：RGB-D bridge 可能会等无人机离地后才发布相机话题。刚启动时没有相机话题，不一定是错误。

## 21. 常见问题

### 21.1 用户名不是 zyc

现象：

- 找不到 `/home/zyc/...`。
- Gazebo 模型路径错误。
- 脚本找不到工作空间。

检查：

```bash
grep -R "/home/zyc" -n ~/vision_sim_ws ~/vision_avoid ~/real_drone 2>/dev/null
```

替换：

```bash
OLD_USER=zyc
NEW_HOME="$HOME"
grep -RIl "/home/${OLD_USER}" ~/vision_sim_ws ~/vision_avoid ~/real_drone 2>/dev/null \
  | xargs sed -i "s#/home/${OLD_USER}#${NEW_HOME}#g"
```

### 21.2 MAVROS QoS 不兼容

现象：

```text
New publisher discovered ... incompatible QoS
```

处理：

- 图像、IMU、位姿等高频话题使用 `BEST_EFFORT`。
- 视频宁可缺帧，也不要堆积旧帧造成延迟。

### 21.3 起飞失败

可能原因：

- MAVROS 未连接。
- GPS/EKF/姿态估计未稳定。
- 没有等待足够时间就 ARM。

处理：

- 启动后等待约 15 s。
- 确认：

```bash
ros2 topic echo /mavros/state --once
```

### 21.4 没有 RGB-D 图像

检查：

```bash
ros2 topic list --no-daemon | grep camera
```

说明：

- 仿真中相机桥接可能等待无人机离地后才开始发布。
- 如果起飞后仍没有，检查 Gazebo 是否启动、模型名称是否正确、protobuf 是否兼容。

### 21.5 numpy 或 OpenCV 报错

恢复版本：

```bash
python3 -m pip install --user "numpy==1.26.4" "opencv-python==4.10.0.84" \
  -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### 21.6 protobuf 报错

执行：

```bash
python3 -m pip install --user "protobuf==3.20.3" \
  -i https://pypi.tuna.tsinghua.edu.cn/simple
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
```

并确认 `~/.bashrc` 中已经写入该环境变量。

### 21.7 YOLO 很慢

检查 GPU：

```bash
nvidia-smi
python3 - <<'PY'
import torch
print(torch.cuda.is_available())
PY
```

如果没有 GPU，可以降低 YOLO 频率或使用 `.engine` 加速模型。

### 21.8 不知道命令应该在哪里执行

看代码块右上角或上下文：

- `powershell`：在 Windows PowerShell 执行。
- `bash`：在 Ubuntu 终端执行。

如果你已经在 Ubuntu 终端里，提示符一般长这样：

```text
zyc@电脑名:~$
```

如果你在 PowerShell 里，提示符一般长这样：

```text
PS C:\Users\你的用户名>
```

### 21.9 找不到下载的 zip

先在 Windows 下载目录确认文件存在。然后在 Ubuntu 中检查：

```bash
ls /mnt/c/Users
```

找到你的 Windows 用户名后再复制：

```bash
cp /mnt/c/Users/你的Windows用户名/Downloads/uav_rgbd_base_sim_package.zip ~/Downloads/
cp /mnt/c/Users/你的Windows用户名/Downloads/uav_rgbd_avoidance_extension_package.zip ~/Downloads/
```
