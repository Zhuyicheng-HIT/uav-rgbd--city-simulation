import time
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import CommandBool, CommandTOL, ParamSet, SetMode

class Takeoff10(Node):
    def __init__(self):
        super().__init__('set_arming_check_and_takeoff')
        self.state = None
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.param = self.create_client(ParamSet, '/mavros/param/set')
        self.mode = self.create_client(SetMode, '/mavros/set_mode')
        self.arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
    def state_cb(self, msg):
        self.state = msg
    def wait_conn(self):
        end = time.time() + 20
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.state and self.state.connected:
                return True
        return False
    def call(self, cli, req, name, timeout=12):
        if not cli.wait_for_service(timeout_sec=timeout):
            print(name, 'service_missing')
            return None
        fut = cli.call_async(req)
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if fut.done():
                print(name, fut.result())
                return fut.result()
        print(name, 'timeout')
        return None

rclpy.init()
node = Takeoff10()
print('connected_wait', node.wait_conn())
req = ParamSet.Request()
req.param_id = 'ARMING_CHECK'
req.value = ParamValue(integer=0, real=0.0)
node.call(node.param, req, 'set_ARMING_CHECK')
req = SetMode.Request()
req.base_mode = 0
req.custom_mode = 'GUIDED'
node.call(node.mode, req, 'set_GUIDED')
req = CommandBool.Request()
req.value = True
node.call(node.arm, req, 'arm')
time.sleep(2)
req = CommandTOL.Request()
req.min_pitch = 0.0
req.yaw = 0.0
req.latitude = 0.0
req.longitude = 0.0
req.altitude = 10.0
node.call(node.takeoff, req, 'takeoff_10m')
end = time.time() + 10
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
print('state', None if node.state is None else (node.state.connected, node.state.armed, node.state.mode))
node.destroy_node()
rclpy.shutdown()
