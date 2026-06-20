# 仿真飞行说明

## 启动顺序

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

## 状态机流程

1. 等待 MAVROS 连接。
2. 发送 setpoint 预热，等待 GPS/EKF/姿态估计稳定。
3. 切换 GUIDED。
4. ARM。
5. 起飞到 12 m。
6. 在世界原点上方悬停 20 s。
7. 启动/等待视觉安全点。
8. 平飞到安全点上方，保持高度不下降。
9. 在安全点上方保持。
10. 跟随安全点降落。

## 仿真专用 fallback

仿真中如果 `/mavros/local_position/pose` 不稳定，可从 Gazebo 获取无人机位姿作为 fallback。该逻辑只允许出现在仿真程序中，实机程序必须使用 APM/MAVROS 或真实传感器输入。

## YOLO 模型

仿真模型优先使用：

```text
/home/zyc/vision_avoid/irreality.engine
```

不存在时 fallback：

```text
/home/zyc/vision_avoid/irreality.pt
```

默认置信度：

```text
0.60
```

## 性能建议

- 图像订阅使用 `BEST_EFFORT`。
- YOLO 支持 TensorRT engine 加速。
- 不需要调试时可降低可视化刷新率。
- 每次仿真结束后关闭旧进程，避免内存累积。

