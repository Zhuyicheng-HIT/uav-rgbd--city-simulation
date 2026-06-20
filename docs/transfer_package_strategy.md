# 离线迁移交付包策略

本项目除了 GitHub 源码仓库，还可以制作一个“小于 1 GB 的离线迁移包”。该策略参考早期 `vision_sim_transfer_package`：包内有必要工程文件、恢复脚本、依赖提示和校验文件，但不塞入大型第三方源码和构建产物。

## 推荐结构

```text
uav_rgbd_transfer_package/
|-- README_快速开始.md
|-- INSTALL_零基础配置摘要.md
|-- PACKAGE_INFO.txt
|-- SHA256SUMS.txt
|-- install_dependencies_ubuntu22.sh
|-- restore_to_home.sh
|-- verify_and_zip.py
|-- docs/
|   |-- 技术文档.md
|   `-- ros2_node_graph.png
`-- payload/
    |-- vision_sim_ws/
    |-- vision_avoid/
    |-- real_drone/
    |-- ardupilot_gazebo/
    `-- vision_sim_yolo_dataset/
```

## payload 中建议包含

- `vision_sim_ws/src/`：基础仿真 ROS2 包、世界文件、模型文件、移动目标脚本。
- `vision_sim_ws/scripts/`：一键启动、检查、桥接、相机验证脚本。
- `vision_avoid/`：仿真视觉、状态机、回放程序。
- `real_drone/`：实机轻量视觉、实机状态机、录制程序。
- `ardupilot_gazebo/`：插件源码，可以包含源码，不包含 `build/` 和 `.git/`。
- 少量示例数据集：只保留小体积训练样例或截图，便于验证 YOLO 流程。

## payload 中不建议包含

- `/home/zyc/ardupilot` 完整源码。
- ArduPilot SITL build 产物。
- `vision_sim_ws/build`、`vision_sim_ws/install`、`vision_sim_ws/log`。
- rosbag、`.db3`、视频文件、大型飞行日志。
- 实机 YOLO 大权重、非必要 TensorRT engine；如果模型是仿真核心适配文件，可以随包发布。
- pip、apt、conda 缓存。

## 用户名策略

推荐下载者 Ubuntu/WSL 用户名使用 `zyc`。这样恢复后路径与已验证环境一致：

```text
/home/zyc/vision_sim_ws
/home/zyc/vision_avoid
/home/zyc/real_drone
```

如果不是 `zyc`，`restore_to_home.sh` 可以把 `/home/zyc` 或旧用户名路径替换成 `$HOME`，但该方式不如直接使用 `zyc` 稳定。

## 清华源策略

- `apt` 和 `pip` 可以提供清华源版本的命令。
- GitHub 源码必须使用官方裸连路径。
- ArduPilot 源码使用：

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
```

不要把 ArduPilot 改成非官方镜像地址写进文档。

## 校验

打包前生成校验文件：

```bash
find . -type f \
  -not -path './.git/*' \
  -not -name 'SHA256SUMS.txt' \
  -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS.txt
```

恢复后可以执行：

```bash
sha256sum -c SHA256SUMS.txt
```

如果包中包含会被恢复脚本修改的路径文件，应在修改前校验。
