# scripts

这里放轻量启动脚本模板。模板只负责进入目录、source ROS 环境、启动对应程序。

正式开源时可以根据仓库实际目录修改：

- `run_sim_state_machine.example.sh`
- `run_sim_vision.example.sh`
- `run_real_vision.example.sh`
- `run_real_state_machine.example.sh`

如果程序已经安装成 ROS2 package，也可以把这里改成 `ros2 run ...`。

