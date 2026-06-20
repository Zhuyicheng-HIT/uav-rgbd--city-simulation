import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class D435iTopicAdapter(Node):
    def __init__(self):
        super().__init__('d435i_topic_adapter')
        self.declare_parameter('source_prefix', '/camera/camera')
        self.declare_parameter('d435i_prefix', '/camera/camera')
        self.declare_parameter('color_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('depth_frame_id', 'camera_depth_optical_frame')
        src = self.get_parameter('source_prefix').value.rstrip('/')
        dst = self.get_parameter('d435i_prefix').value.rstrip('/')
        self.color_frame_id = self.get_parameter('color_frame_id').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value

        qos = 10
        self.color_pub = self.create_publisher(Image, f'{dst}/color/image_raw', qos)
        self.color_info_pub = self.create_publisher(CameraInfo, f'{dst}/color/camera_info', qos)
        self.depth_pub = self.create_publisher(Image, f'{dst}/depth/image_rect_raw', qos)
        self.aligned_depth_pub = self.create_publisher(Image, f'{dst}/aligned_depth_to_color/image_raw', qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, f'{dst}/depth/camera_info', qos)
        self.points_pub = self.create_publisher(PointCloud2, f'{dst}/depth/color/points', qos)

        self.create_subscription(Image, f'{src}/image', self._color_cb, qos)
        self.create_subscription(Image, f'{src}/depth_image', self._depth_cb, qos)
        self.create_subscription(CameraInfo, f'{src}/camera_info', self._camera_info_cb, qos)
        self.create_subscription(PointCloud2, f'{src}/points', self._points_cb, qos)
        self.get_logger().info(
            'D435i-style ROS topics active: '
            f'{dst}/color/image_raw, {dst}/depth/image_rect_raw, '
            f'{dst}/aligned_depth_to_color/image_raw, {dst}/depth/color/points'
        )

    def _color_cb(self, msg: Image):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.color_frame_id
        self.color_pub.publish(out)

    def _depth_cb(self, msg: Image):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.depth_frame_id
        self.depth_pub.publish(out)
        aligned = copy.deepcopy(msg)
        aligned.header.frame_id = self.color_frame_id
        self.aligned_depth_pub.publish(aligned)

    def _camera_info_cb(self, msg: CameraInfo):
        color = copy.deepcopy(msg)
        color.header.frame_id = self.color_frame_id
        self.color_info_pub.publish(color)
        depth = copy.deepcopy(msg)
        depth.header.frame_id = self.depth_frame_id
        self.depth_info_pub.publish(depth)

    def _points_cb(self, msg: PointCloud2):
        out = copy.deepcopy(msg)
        out.header.frame_id = self.depth_frame_id
        self.points_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = D435iTopicAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
