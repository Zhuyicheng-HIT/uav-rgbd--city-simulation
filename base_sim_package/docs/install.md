# 基础仿真包安装说明

## 推荐环境

- Ubuntu 22.04。
- ROS2 Humble。
- Gazebo Garden / Harmonic 相关运行环境以实际工程为准。
- Python 3.10。
- ArduPilot SITL。
- MAVROS / mavros_msgs。

## 用户名与路径

强烈建议 Ubuntu/WSL 用户名直接创建为：

```text
zyc
```

原因：

- 已验证工程默认工作区为 `/home/zyc/vision_sim_ws`。
- 避障程序默认位于 `/home/zyc/vision_avoid`。
- 实机程序默认位于 `/home/zyc/real_drone`。
- 部分脚本、模型路径、日志路径和文档命令以 `~/...` 或 `/home/zyc/...` 为基础验证。

如果用户名不是 `zyc`，需要统一替换路径。可以使用脚本扫描后替换，但不推荐初学者这么做，因为 Gazebo 世界文件、启动脚本、Python 配置和 README 中都可能存在路径引用。

示例检查命令：

```bash
grep -R "/home/zyc" -n ~/vision_sim_ws ~/vision_avoid ~/real_drone
```

示例替换思路：

```bash
OLD_USER=zyc
NEW_USER="$USER"
grep -RIl "/home/${OLD_USER}" ~/vision_sim_ws ~/vision_avoid ~/real_drone \
  | xargs sed -i "s#/home/${OLD_USER}#/home/${NEW_USER}#g"
```

执行替换前建议先提交 git 或备份工程。

## apt 源优化

网络较慢时可以切换 Ubuntu 和 ROS apt 源到清华源。该步骤依赖用户本机环境，不建议脚本强制覆盖。

参考：

- `https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu/`
- `https://mirrors.tuna.tsinghua.edu.cn/help/ros2/`

## Python 依赖

建议建立虚拟环境或在固定 Python 环境中安装：

```bash
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

已踩坑版本建议：

```bash
pip install "numpy==1.26.4" "protobuf==3.20.3" -i https://pypi.tuna.tsinghua.edu.cn/simple
```

说明：

- `numpy` 升级到 2.x 可能导致 OpenCV、ROS Python 扩展或 TensorRT 相关包 ABI 不兼容。
- Gazebo Python 绑定遇到 protobuf 解析问题时，可固定 `protobuf==3.20.3`，并设置：

```bash
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
```

## ArduPilot 源码

ArduPilot 没有可靠的清华源镜像，文档和脚本中应使用 GitHub 官方裸连路径：

```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

不要把 ArduPilot 源码打进本仓库。

## 工作空间构建

```bash
cd ~/vision_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
