import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node as RosNode
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from gz.transport13 import Node as GzNode
from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
from gz.msgs10.image_pb2 import Image as GzImage


PIXEL_ENCODINGS = {
    1: 'mono8',
    2: 'mono16',
    3: 'rgb8',
    4: 'rgba8',
    5: 'bgra8',
    8: 'bgr8',
    11: '16FC1',
    13: '32FC1',
}


class GzRgbdBridge(RosNode):
    def __init__(self):
        super().__init__('gz_rgbd_bridge')
        self.declare_parameter('gz_prefix', '/camera/camera')
        self.declare_parameter('ros_prefix', '/camera/camera')
        self.declare_parameter('color_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('depth_frame_id', 'camera_depth_optical_frame')
        self.gz_prefix = self.get_parameter('gz_prefix').value.rstrip('/')
        self.ros_prefix = self.get_parameter('ros_prefix').value.rstrip('/')
        self.color_frame_id = self.get_parameter('color_frame_id').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value

        self._closing = False
        self.gz_node = GzNode()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.raw_color_pub = self.create_publisher(Image, f'{self.ros_prefix}/image', qos)
        self.raw_depth_pub = self.create_publisher(Image, f'{self.ros_prefix}/depth_image', qos)
        self.raw_info_pub = self.create_publisher(CameraInfo, f'{self.ros_prefix}/camera_info', qos)
        self.color_pub = self.create_publisher(Image, f'{self.ros_prefix}/color/image_raw', qos)
        self.color_info_pub = self.create_publisher(CameraInfo, f'{self.ros_prefix}/color/camera_info', qos)
        self.depth_pub = self.create_publisher(Image, f'{self.ros_prefix}/depth/image_rect_raw', qos)
        self.aligned_depth_pub = self.create_publisher(Image, f'{self.ros_prefix}/aligned_depth_to_color/image_raw', qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, f'{self.ros_prefix}/depth/camera_info', qos)

        self.gz_node.subscribe(GzImage, f'{self.gz_prefix}/image', self._color_cb)
        self.gz_node.subscribe(GzImage, f'{self.gz_prefix}/depth_image', self._depth_cb)
        self.gz_node.subscribe(GzCameraInfo, f'{self.gz_prefix}/camera_info', self._info_cb)
        self.get_logger().info(
            f'Gazebo RGB-D bridge active from {self.gz_prefix} to ROS/D435i topics under {self.ros_prefix}'
        )

    def _stamp(self, msg):
        stamp = self.get_clock().now().to_msg()
        try:
            stamp.sec = msg.header.stamp.sec
            stamp.nanosec = msg.header.stamp.nsec
        except Exception:
            pass
        return stamp

    def _image_msg(self, msg, frame_id):
        out = Image()
        out.header.stamp = self._stamp(msg)
        out.header.frame_id = frame_id
        out.height = int(msg.height)
        out.width = int(msg.width)
        out.encoding = PIXEL_ENCODINGS.get(int(msg.pixel_format_type), 'passthrough')
        out.is_bigendian = 0
        out.step = int(msg.step)
        out.data = bytes(msg.data)
        return out

    def _safe_publish(self, pub, msg):
        if self._closing or not rclpy.ok():
            return
        try:
            pub.publish(msg)
        except Exception:
            pass

    def _color_cb(self, msg):
        out = self._image_msg(msg, self.color_frame_id)
        self._safe_publish(self.raw_color_pub, out)
        self._safe_publish(self.color_pub, out)

    def _depth_cb(self, msg):
        out = self._image_msg(msg, self.depth_frame_id)
        if out.encoding == 'passthrough' and out.step == out.width * 4:
            out.encoding = '32FC1'
        self._safe_publish(self.raw_depth_pub, out)
        self._safe_publish(self.depth_pub, out)
        aligned = copy.deepcopy(out)
        aligned.header.frame_id = self.color_frame_id
        self._safe_publish(self.aligned_depth_pub, aligned)

    def _info_cb(self, msg):
        out = CameraInfo()
        out.header.stamp = self._stamp(msg)
        out.header.frame_id = self.color_frame_id
        out.width = int(msg.width)
        out.height = int(msg.height)
        try:
            out.k = list(msg.intrinsics.k)
            out.p = list(msg.projection.p)
            out.r = list(msg.rectification_matrix) if len(msg.rectification_matrix) == 9 else [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
        except Exception:
            out.r = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
        try:
            out.d = list(msg.distortion.k)
            out.distortion_model = msg.distortion.model or 'plumb_bob'
        except Exception:
            out.distortion_model = 'plumb_bob'
        self._safe_publish(self.raw_info_pub, out)
        self._safe_publish(self.color_info_pub, out)
        depth = copy.deepcopy(out)
        depth.header.frame_id = self.depth_frame_id
        self._safe_publish(self.depth_info_pub, depth)


def main(args=None):
    rclpy.init(args=args)
    node = GzRgbdBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._closing = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
