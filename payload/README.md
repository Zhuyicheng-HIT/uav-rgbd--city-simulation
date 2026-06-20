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

本迁移包允许包含仿真专用模型：

- `vision_avoid/irreality.pt`
- `vision_avoid/irreality.engine`
- `vision_sim_ws/models/irreality.pt`
