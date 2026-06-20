# 开源打包策略

本项目建议采用“小仓库 + 命令下载 + 明确放置路径”的方式开源。这样仓库可长期维护，也不会因为第三方大文件或构建产物导致克隆困难。

## 应当放入仓库

- 自己编写或修改的 ROS2 节点、Python 程序、状态机、桥接脚本。
- 自己修改的 Gazebo 世界文件、无人机模型、相机挂载描述、移动目标轨迹脚本。
- `README.md`、安装说明、启动说明、故障排查、参数说明。
- 小型示例配置，例如 `.env.example`、topic 名称表、默认相机内参。
- 小型示意图、流程图、ROS 节点图。

## 不建议放入仓库

- ArduPilot 源码、Gazebo 源码、ROS 安装包。
- `build/`、`install/`、`log/`、`.colcon/`。
- YOLO 权重、TensorRT engine、ONNX 模型。
- rosbag、`.db3`、飞行视频、D435i 原始录像。
- 本机路径、密码、飞控串口号、遥控器参数等个人环境信息。

## 下载源选择

- GitHub 源码必须使用官方裸连地址，例如：

```bash
git clone https://github.com/ArduPilot/ardupilot.git
```

- `pip` 可以使用清华源：

```bash
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

- Ubuntu `apt` 可以使用清华源，但文档中建议写成“可选优化”，避免覆盖用户已有源配置。
- 没有可靠镜像的内容使用官方源，例如 ArduPilot、Gazebo 官方包、模型发布页。

## 大文件处理

对于模型和录像，建议只保留说明文件：

```text
models/
└── README.md

data/
└── README.md
```

说明文件中写清：

- 文件名。
- 来源。
- 放置路径。
- 校验方式。
- 程序找不到文件时的降级逻辑。

