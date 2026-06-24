# payload

`payload/` 只用于制作离线迁移交付包。GitHub 日常源码开发不一定需要填充该目录。

推荐放入：

- `payload/vision_sim_ws/`
- `payload/vision_avoid/`
- `payload/real_drone/`
- `payload/ardupilot_gazebo/`
- `payload/vision_sim_yolo_dataset/`

不要放入：

- `build/`
- `install/`
- `log/`
- `.git/`
- `.db3`
- 视频文件
- 实机模型权重
- 非必要 TensorRT engine
- ArduPilot 完整源码

实机程序当前采用 MAVROS 中心结构：

- `real_vision_node.py` 读取 D435i，并订阅 MAVROS 位姿/状态生成安全点。
- `real_landing_state_machine.py` 订阅安全点，并向 MAVROS 发布位置 setpoint。
- `flight_record_mavlink.py` 只记录 MAVROS 飞控话题，不直接打开飞控串口。

本迁移包允许包含仿真专用模型：

- `vision_avoid/irreality.pt`
- `vision_avoid/irreality.engine`
- `vision_sim_ws/models/irreality.pt`
