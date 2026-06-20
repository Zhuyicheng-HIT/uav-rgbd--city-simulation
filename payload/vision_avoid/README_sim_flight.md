# Vision Landing: real-flight-compatible simulation test

## Interface contract

The same code should run on the onboard computer and in simulation when these ROS2 topics are present:

- RGB: `/camera/camera/color/image_raw`
- Depth: `/camera/camera/depth/image_rect_raw`
- CameraInfo: `/camera/camera/color/camera_info`
- MAVROS state: `/mavros/state`
- MAVROS local pose: `/mavros/local_position/pose`
- MAVROS IMU: `/mavros/imu/data`
- Position setpoint output: `/mavros/setpoint_position/local`

The vision node publishes:

- `/vision/avoidance_waypoint`
- `/vision/avoidance_status`

## Simulation flow

1. Start Gazebo + ArduPilot + MAVROS + RGB-D bridge.
2. Start `sim_waypoint_node.py`; it opens four windows and publishes visual safe-point status.
3. Start `sim_landing_state_machine.py`; it performs:
   - hover at world origin after takeoff to 12 m
   - horizontal flight over the visual safe point while holding altitude
   - descent while continuously following the visual safe point

## Commands

Terminal 1:

```bash
cd ~/vision_sim_ws
./scripts/run_apm_rgbd_stack.sh
```

Terminal 2:

```bash
cd ~/vision_avoid
source /opt/ros/humble/setup.bash
source ~/vision_sim_ws/install/setup.bash
python3 sim_waypoint_node.py
```

Terminal 3:

```bash
cd ~/vision_avoid
source /opt/ros/humble/setup.bash
source ~/vision_sim_ws/install/setup.bash
python3 sim_landing_state_machine.py
```

For quick testing, reduce the dwell times:

```bash
python3 sim_landing_state_machine.py --ros-args \
  -p hover_seconds:=10.0 \
  -p safe_overhead_seconds:=10.0
```
