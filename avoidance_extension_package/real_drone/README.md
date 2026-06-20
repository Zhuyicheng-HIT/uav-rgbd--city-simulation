# real_drone

这里放实机飞行轻量程序。

建议从已验证环境复制：

- `/home/zyc/real_drone/real_vision_node.py`
- `/home/zyc/real_drone/real_landing_state_machine.py`
- `/home/zyc/real_drone/record_d435i_mavlink.py`

实机程序要求：

- 无 Gazebo fallback。
- 无可视化窗口。
- 无回放读取逻辑。
- 保留日志。
- 参数可通过配置文件或命令行修改。

