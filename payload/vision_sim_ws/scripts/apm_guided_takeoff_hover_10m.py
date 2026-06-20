#!/usr/bin/env python3
import math
import time

from pymavlink import mavutil

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node


class ApmGuidedTakeoffHover(Node):
    def __init__(self):
        super().__init__('apm_guided_takeoff_hover_10m')
        self.declare_parameter('altitude', 10.0)
        self.declare_parameter('hold_seconds', 0.0)
        self.declare_parameter('setpoint_rate_hz', 10.0)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('preflight_wait_seconds', 15.0)
        self.declare_parameter('command_retry_seconds', 45.0)
        self.declare_parameter('enable_position_setpoint', False)
        self.declare_parameter('enable_rc_override', False)
        self.declare_parameter('rc_override_throttle', 1500)
        self.declare_parameter('mavlink_takeoff_url', 'tcp:127.0.0.1:5762')
        self.declare_parameter('mavlink_target_component', 1)
        self.declare_parameter('takeoff_param3', 1.0)
        self.altitude = float(self.get_parameter('altitude').value)
        self.hold_seconds = float(self.get_parameter('hold_seconds').value)
        self.rate_hz = float(self.get_parameter('setpoint_rate_hz').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.preflight_wait_seconds = float(self.get_parameter('preflight_wait_seconds').value)
        self.command_retry_seconds = float(self.get_parameter('command_retry_seconds').value)
        self.enable_position_setpoint = bool(self.get_parameter('enable_position_setpoint').value)
        self.enable_rc_override = bool(self.get_parameter('enable_rc_override').value)
        self.rc_override_throttle = int(self.get_parameter('rc_override_throttle').value)
        self.mavlink_takeoff_url = str(self.get_parameter('mavlink_takeoff_url').value)
        self.mavlink_target_component = int(self.get_parameter('mavlink_target_component').value)
        self.takeoff_param3 = float(self.get_parameter('takeoff_param3').value)
        self.state = State()
        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

    def _state_cb(self, msg):
        self.state = msg

    def wait_connected(self, timeout=60.0):
        self.get_logger().info('Waiting for MAVROS / FCU connection...')
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state.connected:
                self.get_logger().info('FCU connected')
                return
        raise RuntimeError('Timed out waiting for /mavros/state.connected')

    def wait_services(self, timeout=20.0):
        for cli, name in [(self.mode_cli, 'set_mode'), (self.arm_cli, 'arming')]:
            if not cli.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'MAVROS service missing: {name}')

    def call(self, cli, req, label, timeout=10.0):
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if fut.done():
                resp = fut.result()
                self.get_logger().info(f'{label}: {resp}')
                return resp
        raise RuntimeError(f'Timeout calling {label}')

    def publish_rc_override(self):
        if not self.enable_rc_override:
            return
        msg = OverrideRCIn()
        throttle = max(1000, min(2000, self.rc_override_throttle))
        msg.channels = [1500, 1500, throttle, 1500] + [OverrideRCIn.CHAN_RELEASE] * 14
        self.rc_override_pub.publish(msg)

    def publish_hover_setpoint(self):
        if not self.enable_position_setpoint:
            self.publish_rc_override()
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.origin_x
        msg.pose.position.y = self.origin_y
        msg.pose.position.z = self.altitude
        msg.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.orientation.w = math.cos(self.yaw * 0.5)
        self.setpoint_pub.publish(msg)
        self.publish_rc_override()

    def stream_setpoints(self, seconds):
        period = 1.0 / max(self.rate_hz, 1.0)
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.publish_hover_setpoint()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def wait_for_state(self, predicate, label, timeout=12.0):
        period = 1.0 / max(self.rate_hz, 1.0)
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            self.publish_hover_setpoint()
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                self.get_logger().info(
                    f'{label} confirmed: mode={self.state.mode}, armed={self.state.armed}, connected={self.state.connected}')
                return True
            time.sleep(period)
        self.get_logger().warning(
            f'{label} not confirmed within {timeout:.1f}s: mode={self.state.mode}, armed={self.state.armed}, connected={self.state.connected}')
        return False

    def send_takeoff_command_int(self):
        self.get_logger().info(
            f'Sending MAV_CMD_NAV_TAKEOFF as COMMAND_INT, frame=GLOBAL_RELATIVE_ALT, '
            f'z={self.altitude:.1f}m, param3={self.takeoff_param3:.1f} via {self.mavlink_takeoff_url}')
        mav = mavutil.mavlink_connection(self.mavlink_takeoff_url, source_system=252)
        mav.wait_heartbeat(timeout=10)
        target_system = mav.target_system or 1
        target_component = self.mavlink_target_component or mav.target_component or 1
        self.get_logger().info(f'MAVLink takeoff target: system={target_system}, component={target_component}')
        mav.mav.command_int_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0.0,
            0.0,
            self.takeoff_param3,
            self.yaw,
            0,
            0,
            self.altitude)
        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            msg = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                ok = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                self.get_logger().info(f'takeoff COMMAND_INT ack: result={msg.result}, accepted={ok}')
                mav.close()
                return ok
        mav.close()
        self.get_logger().warning('Timed out waiting for takeoff COMMAND_INT ACK')
        return False

    def run(self):
        self.wait_connected()
        self.wait_services()
        self.get_logger().info(f'Waiting {self.preflight_wait_seconds:.1f}s for EKF/GPS position estimate...')
        self.stream_setpoints(self.preflight_wait_seconds)

        deadline = time.monotonic() + self.command_retry_seconds
        guided_ok = False
        armed_ok = False
        takeoff_ok = False
        while rclpy.ok() and time.monotonic() < deadline:
            if self.state.mode != 'GUIDED':
                req = SetMode.Request()
                req.base_mode = 0
                req.custom_mode = 'GUIDED'
                mode_resp = self.call(self.mode_cli, req, 'set GUIDED')
                mode_sent = bool(getattr(mode_resp, 'mode_sent', False))
                if not mode_sent:
                    self.get_logger().warning('GUIDED mode command was not accepted for sending; retrying.')
                    self.stream_setpoints(2.0)
                    continue
            guided_ok = self.wait_for_state(lambda: self.state.mode == 'GUIDED', 'GUIDED mode', timeout=15.0)
            if not guided_ok:
                self.stream_setpoints(2.0)
                continue

            if not self.state.armed:
                req = CommandBool.Request()
                req.value = True
                arm_resp = self.call(self.arm_cli, req, 'arm')
                arm_sent = bool(getattr(arm_resp, 'success', False))
                if not arm_sent:
                    self.get_logger().warning('Arm rejected, waiting and retrying. Inspect /mavros/statustext/recv for details.')
                    self.stream_setpoints(3.0)
                    continue
            armed_ok = self.wait_for_state(lambda: self.state.armed, 'armed state', timeout=15.0)
            if not armed_ok:
                self.stream_setpoints(2.0)
                continue

            if self.state.mode != 'GUIDED':
                self.get_logger().warning(f'Mode changed to {self.state.mode} after arming; retrying GUIDED before takeoff.')
                self.stream_setpoints(1.0)
                continue

            takeoff_ok = self.send_takeoff_command_int()
            if takeoff_ok:
                self.get_logger().info('Takeoff command accepted after confirmed GUIDED + armed state.')
                break
            self.get_logger().warning('Takeoff rejected, waiting and retrying.')
            self.stream_setpoints(3.0)

        if not guided_ok or not armed_ok or not takeoff_ok:
            self.get_logger().warning(f'Command phase ended with guided={guided_ok}, armed={armed_ok}, takeoff={takeoff_ok}. Continuing setpoint stream for diagnosis.')
        if self.enable_position_setpoint:
            self.get_logger().info(f'Holding GUIDED origin setpoint at z={self.altitude:.1f}m. Ctrl+C to stop.')
        else:
            self.get_logger().info(f'GUIDED takeoff accepted; letting ArduPilot climb and hover at z={self.altitude:.1f}m. Ctrl+C to stop.')
        start = time.monotonic()
        while rclpy.ok():
            self.publish_hover_setpoint()
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.hold_seconds > 0.0 and time.monotonic() - start >= self.hold_seconds:
                break
            time.sleep(1.0 / max(self.rate_hz, 1.0))


def main():
    rclpy.init()
    node = ApmGuidedTakeoffHover()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
