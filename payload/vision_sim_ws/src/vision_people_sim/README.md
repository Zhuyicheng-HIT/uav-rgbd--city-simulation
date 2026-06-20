# Gazebo + ROS2 视觉人物仿真

目标：给无人机视觉采集使用的轻量人物 + 车辆运动环境。全工程不做碰撞设置，不处理人物、车辆与场景之间的穿插；人物和车辆模型都只有 visual，没有 collision。

## 构建

```bash
cd /home/ld666/vision_sim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vision_people_sim
source install/setup.bash
```

## 1. 基础阶段：写实纹理人物/车辆 + 直线/矩形运动

```bash
source /opt/ros/humble/setup.bash
source /home/ld666/vision_sim_ws/install/setup.bash
ros2 launch vision_people_sim basic_people.launch.py
```

校验：

```bash
gz service -l | grep /world/basic_people/set_pose
ros2 node list | grep people_motion
```

## 2. 进阶阶段：多人物 + 多车辆独立循环，可互相穿透

```bash
source /opt/ros/humble/setup.bash
source /home/ld666/vision_sim_ws/install/setup.bash
ros2 launch vision_people_sim multi_people.launch.py
```

校验：

```bash
gz service -l | grep /world/multi_people/set_pose
ros2 node info /people_motion
```

## 3. 最终阶段：轻量城市视觉环境 + 人物/车辆运动

```bash
source /opt/ros/humble/setup.bash
source /home/ld666/vision_sim_ws/install/setup.bash
ros2 launch vision_people_sim city_people.launch.py
```

校验：

```bash
gz service -l | grep /world/city_people/set_pose
ros2 node list | grep people_motion
```

## 城市场景资源策略

本机 `gz fuel list --type model --url https://fuel.gazebosim.org/openrobotics/models` 未返回可下载列表，因此默认交付的是无外部依赖的轻量 SDF 城市：道路、楼体、窗格纹理由本工程生成，只包含 visual，不包含 collision 或复杂插件。这样可以先保证无人机视觉采集链路可直接运行。

如果后续网络可用，推荐从 Gazebo Fuel 官方模型或 GitHub 开源城市/建筑 mesh 中选择低面数资源，并按下面方式接入：

1. 把资源放到 `/home/ld666/vision_sim_ws/src/vision_people_sim/models/<model_name>`。
2. 在 `worlds/city_people.sdf` 中用 `<include><uri>model://<model_name></uri></include>` 引入。
3. 人物模型名称保持与 launch 中 `name` 一致，例如 `person_north`、`person_south`、`person_block`。
4. 只保留 visual mesh 和 texture，不添加 collision。运动代码 `people_motion.py` 不需要修改。

## 车辆推进方式

车辆与人物共用同一个 ROS2 节点 `people_motion.py`：launch 文件把人物和车辆都作为 `agents_json` 传入，节点按模型名调用 Gazebo `/world/<world>/set_pose` 服务推进位姿。区别只在模型资源：人物使用 `model://textured_person`，车辆使用 `model://textured_vehicle`。

阶段对应关系：

- 基础：`vehicle_line` 在直线路径上往返，和人物基础运动一起验证。
- 进阶：`vehicle_a`、`vehicle_b`、`vehicle_c` 独立运动，和多人物一样允许穿透。
- 最终：`vehicle_main`、`vehicle_cross`、`vehicle_loop` 接入城市道路环境，与人物共同作为视觉目标。

校验车辆是否接入成功，可以在对应 world 启动后执行：

```bash
gz service -s /world/city_people/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req 'name: "vehicle_main" position { x: 0 y: 0.75 z: 0.05 } orientation { w: 1 }'
```

返回 `data: true` 表示车辆模型可被 Gazebo 位姿服务控制。

## 新版视觉修正

- 人物模型已从 2D billboard 改成 3D 低模人形，仍然只有 visual，没有 collision。
- ROS2 节点改为通过 `gz.transport13` 原生 API 每帧批量调用 `/world/<world>/set_pose_vector`，避免每帧启动 `gz service` 子进程造成的闪现感。
- 城市地图扩大到 34 x 26，包含主路、十字路和外圈路。
- 车辆路径全部位于黑色道路中心线附近；人物路径放在人行道/道路外侧区域。

## 比例查看更新

- 光照已提高，城市环境更亮。
- 人物模型整体约缩小 30%，并补齐胸腹/腰部体块，减少中间空洞。
- 城市 world 已加入 `iris_with_ardupilot` 静态比例参考模型，资源来自 `/home/ld666/ardupilot_gazebo/models`。
- Gazebo GUI 默认使用 `--render-engine-gui ogre`，避开当前 WSL 图形环境中 OGRE2/Mesa 的崩溃问题。

## 车辆上方无人机跟随

city_people.sdf 中新增 drone_above_vehicle_main，使用 ArduPilot Gazebo 的 iris_with_standoffs 模型。它位于 vehicle_main 上方约 1 m，并在 city_people.launch.py / city_full_demo_params.yaml 中使用与 vehicle_main 相同的直线路径和速度，因此会跟随该车辆移动。

## 无人机跟随修正

drone_above_vehicle_main 已恢复为 world 顶层的完整 ArduPilot 模型 `iris_with_ardupilot`，不再使用 static 外层包装。ROS 节点直接移动该一级模型，路径与 vehicle_main 相同，z=2.0，用于避免贴近或穿入车辆，同时保留后续接入 APM 飞行仿真的模型结构。

## 无人机工程说明

当前车辆跟随演示中的 drone_above_vehicle_main 是 static 外层模型，内部包含完整 `iris_with_ardupilot`，路径 z=3.0，用于保证视觉演示不受重力/插件扰动而掉落。后续真正 APM 飞行仿真应单独启动顶层动态 `iris_with_ardupilot` 并连接 SITL，而不是把飞行器挂在车辆跟随路径上。

## 跟随无人机稳定性修正

车辆上方跟随演示使用 `apm_iris_follow_static`，它复制 ArduPilot Iris 外形资源并设为 static，用于视觉比例和跟随展示，避免完整动态飞行器在未连接 SITL 时受重力/插件扰动掉落。真实飞行仿真仍使用 `/home/ld666/ardupilot_gazebo/models/iris_with_ardupilot`。

## D435i 风格 RGB-D 相机

跟随车辆的无人机展示模型 `apm_iris_follow_static` 内新增 `d435i_rgbd` 传感器，分辨率 640x480、30Hz、水平视场约 69.4 度、深度范围 0.105-10m。Gazebo 原生话题前缀为 `/camera/camera`，可用 `gz topic -l | grep camera` 查看实际 color/depth/camera_info 输出。ROS2 可通过 `ros_gz_bridge parameter_bridge` 桥接为 `sensor_msgs/msg/Image` 和 `sensor_msgs/msg/CameraInfo`。

## RGB-D 演示启动

运行：

```bash
/home/ld666/vision_sim_ws/scripts/run_city_rgbd_demo.sh
```

Gazebo 原生话题：

- `/camera/camera/image`
- `/camera/camera/depth_image`
- `/camera/camera/camera_info`
- `/camera/camera/points`

ROS2 bridge 后的 D435i 风格话题：

- `/camera/camera/color/image_raw`
- `/camera/camera/color/camera_info`
- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/depth/camera_info`
- `/camera/camera/depth/color/points`
