# sim

这里放仿真避障程序。

建议从已验证环境复制：

- `/home/zyc/vision_avoid/sim_waypoint_node.py`
- `/home/zyc/vision_avoid/sim_landing_state_machine.py`

仿真程序允许包含：

- Gazebo pose fallback。
- 四窗口或整合窗口可视化。
- 仿真模型 `irreality.pt` / `irreality.engine` 的加载逻辑。

仿真程序不应混入实机专用串口、真实飞控密码或个人路径。

