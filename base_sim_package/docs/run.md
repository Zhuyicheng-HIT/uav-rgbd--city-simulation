# 基础仿真包运行说明

## 一键启动

终端 1：

```bash
cd ~/vision_sim_ws
./scripts/run_apm_rgbd_stack.sh
```

该脚本负责启动：

- Gazebo 城市场景。
- ArduPilot SITL。
- MAVROS。
- RGB-D 相机桥接。
- 移动车辆/行人目标。

## 检查话题

```bash
source /opt/ros/humble/setup.bash
source ~/vision_sim_ws/install/setup.bash
ros2 topic list --no-daemon | grep -E 'camera/camera|mavros/local_position|mavros/state'
```

常用检查：

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic echo /mavros/state --once
```

注意：当前 RGB-D 桥接会等无人机起飞或相机模型有效后再发布话题。如果刚启动时没有 `/camera/camera/...`，先确认无人机是否已经离地。

## 推荐关闭顺序

Gazebo、ArduPilot、MAVROS 和图像桥接比较占资源。每次完整运行后建议关闭旧进程，再启动下一轮测试，避免内存持续上涨。

```bash
pkill -f sim_vehicle.py
pkill -f mavros
pkill -f gz
```

如果还有残留进程，再用 `ps aux | grep` 检查。

