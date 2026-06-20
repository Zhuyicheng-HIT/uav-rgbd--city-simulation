import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State
from sensor_msgs.msg import Image

class Probe(Node):
    def __init__(self):
        super().__init__('stack_probe')
        self.state = None
        self.depth = None
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_cb,
            qos_profile_sensor_data,
        )

    def state_cb(self, msg):
        self.state = msg

    def depth_cb(self, msg):
        self.depth = msg

rclpy.init()
node = Probe()
end = time.time() + 15
while time.time() < end and (node.state is None or node.depth is None):
    rclpy.spin_once(node, timeout_sec=0.2)
print('state_seen=', node.state is not None)
if node.state:
    print('connected=', node.state.connected, 'armed=', node.state.armed, 'mode=', node.state.mode)
print('depth_seen=', node.depth is not None)
if node.depth:
    print('depth=', node.depth.width, node.depth.height, node.depth.encoding, node.depth.step, len(node.depth.data))
node.destroy_node()
rclpy.shutdown()
