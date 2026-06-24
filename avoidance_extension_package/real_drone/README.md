# real_drone

这里放实机飞行轻量程序。

建议从已验证环境复制：

- `/home/zyc/real_drone/real_vision_node.py`
- `/home/zyc/real_drone/real_landing_state_machine.py`
- `/home/zyc/real_drone/flight_record_mavlink.py`
- `/home/zyc/real_drone/record_d435i_mavlink.py`（旧命令兼容入口）

实机程序要求：

- 无 Gazebo fallback。
- 无可视化窗口。
- 无回放读取逻辑。
- 保留日志。
- 参数可通过配置文件或命令行修改。

实机连接原则：

- MAVROS 独占飞控串口 `/dev/ttyACM0`，视觉程序和日志程序都不直接打开飞控串口。
- `real_vision_node.py` 独占 D435i，并可通过 `--record-db3` 同步保存 RGB-D 数据。
- `flight_record_mavlink.py` 只订阅 MAVROS 话题，作为飞控状态日志记录器。
