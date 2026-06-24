# 实机飞行说明

实机程序面向 APM 4.6 飞控和 Intel RealSense D435i 深度相机。与仿真程序相比，实机版本应更轻量、更保守。

## 实机程序原则

- 不读取 Gazebo。
- 不读取回放视频。
- 不打开可视化窗口。
- 不使用仿真 pose fallback。
- 不直接打开飞控串口，飞控通信统一交给 MAVROS。
- 保留安全点计算、目标跟踪、飞控状态读取和日志记录。
- 保留人工接管能力。

## 输入

- D435i RGB 图像。
- D435i 深度图。
- D435i CameraInfo。
- APM/MAVROS 本地位置。
- APM/MAVROS IMU 或姿态。
- MAVROS state。

## 输出

- 视觉节点输出 `/vision/avoidance_waypoint` 安全点。
- 视觉节点输出 `/vision/avoidance_status` 状态。
- 状态机向 `/mavros/setpoint_position/local` 发布位置 setpoint。
- 视觉日志、状态机日志和 MAVROS 飞控话题日志。

建议日志：

```text
logs/
├── vision_events.jsonl
├── vision_summary.csv
├── state_machine.csv
└── fc_state_snapshots.csv
```

每条日志至少包含：

- 时间戳。
- 当前状态机阶段。
- 无人机位置。
- 无人机姿态。
- tracker 目标点。
- 安全点。
- YOLO 置信度。
- 算法耗时和 FPS。

## 实机测试顺序

1. 桌面静态测试：确认 D435i 图像、深度、CameraInfo 正常。
2. 地面联调：确认 MAVROS state、pose、IMU 正常。
3. 不解锁测试：运行视觉节点，只记录日志，不发送控制。
4. 旁路记录测试：运行 `flight_record_mavlink.py`，确认它只订阅 MAVROS 话题。
5. 低高度悬停测试：只观察安全点是否稳定。
6. 人工保护下半自动测试：状态机每个阶段单独验证。
7. 全流程测试。

## 风险提示

真实飞行中安全点只应作为辅助决策。第一次测试必须在空旷环境进行，并保留遥控器人工接管。
