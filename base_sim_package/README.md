# 基础仿真包

本包提供基础的 Gazebo + ArduPilot/MAVROS + D435i RGB-D 相机 + 城市移动目标仿真环境。它的目标是先让无人机、飞控、深度相机和移动障碍物稳定跑起来，为后续避障算法提供可复现输入。

## 功能范围

- Ubuntu 22.04 + ROS2 Humble。
- Gazebo 城市场景、道路、建筑、车辆和行人。
- ArduPilot SITL + MAVROS 连接。
- 无人机挂载 D435i RGB-D 相机。
- RGB、深度图、CameraInfo 发布到 ROS2。
- 移动目标轨迹脚本，避免前 300 s 内明显穿模。
- 一键启动脚本和检查脚本。

## 不包含内容

- 不包含 ArduPilot 完整源码。
- 不包含 ROS/Gazebo 二进制包。
- 不包含 YOLO 权重和 TensorRT engine。
- 不包含构建产物、rosbag、飞行录像。

## 文档入口

- [安装说明](docs/install.md)
- [运行说明](docs/run.md)
- [故障排查](docs/troubleshooting.md)

