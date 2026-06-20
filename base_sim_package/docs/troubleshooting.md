# 基础仿真包故障排查

## 用户名不是 zyc 导致路径错误

现象：

- 脚本找不到 `~/vision_sim_ws`、`~/vision_avoid`。
- 模型、世界文件或日志路径不存在。
- Gazebo 能启动，但桥接脚本或视觉程序找不到资源。

建议：

- 新装环境优先把 Ubuntu/WSL 用户名建为 `zyc`。
- 已经使用其他用户名时，统一替换 `/home/zyc`。
- 替换前先扫描：

```bash
grep -R "/home/zyc" -n ~/vision_sim_ws ~/vision_avoid ~/real_drone
```

该问题非常常见，属于迁移工程时最容易忽略的路径坑。

## MAVROS 话题 QoS 不兼容

现象：

```text
New publisher discovered ... offering incompatible QoS
Last incompatible policy: RELIABILITY
```

处理：

- 图像、IMU、位姿等高频数据优先使用 `BEST_EFFORT`。
- 对视频流宁可缺帧，也不要堆积旧帧造成延迟。
- ROS2 Python 可使用 `qos_profile_sensor_data` 或显式设置 `ReliabilityPolicy.BEST_EFFORT`。

## 起飞后没有 RGB-D 图像

检查：

```bash
ros2 topic list --no-daemon | grep camera
ros2 topic hz /camera/camera/color/image_raw
```

可能原因：

- RGB-D bridge 设计为等待无人机或相机实体出现后才发布。
- Gazebo 中相机模型名称变化，桥接脚本找不到实体。
- protobuf/numpy 版本错误导致 bridge 静默异常。

处理：

- 先确认 Gazebo 中无人机已经离地。
- 检查 `run_apm_rgbd_stack.sh` 日志。
- 固定：

```bash
pip install "numpy==1.26.4" "protobuf==3.20.3" -i https://pypi.tuna.tsinghua.edu.cn/simple
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
```

## 无法 ARM 或 TAKEOFF

现象：

```text
arm: success=False
takeoff: success=False
```

处理：

- MAVROS 连接后等待约 15 s，让 GPS、姿态估计和 EKF 状态稳定。
- 先发送 setpoint 预热，再切 GUIDED，再 arm。
- 确认 `/mavros/state` 中 `connected=True`。

## `/mavros/local_position/pose` 不发布

仿真中可能出现 MAVROS 已连接但 local pose 不来的情况。基础仿真可以使用 Gazebo pose 作为备用位姿源；真实飞行程序不要使用 Gazebo fallback。

## Gazebo Python 绑定 protobuf 报错

处理：

```bash
pip install "protobuf==3.20.3" -i https://pypi.tuna.tsinghua.edu.cn/simple
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
```

## TensorRT 后 Python 环境损坏

TensorRT/Ultralytics 导出 engine 时可能升级 `numpy`，导致 OpenCV 或 ROS Python 扩展失效。

处理：

```bash
pip install "numpy==1.26.4" -i https://pypi.tuna.tsinghua.edu.cn/simple
python3 -c "import cv2, numpy; print(cv2.__version__, numpy.__version__)"
```
