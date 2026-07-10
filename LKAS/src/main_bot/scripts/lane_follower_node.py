#!/usr/bin/env python3
"""lane_follower_node.py — camera -> vision pipeline -> /status_err bridge.

Runs LaneProcessor on each incoming frame and publishes (e_y, e_psi, kappa)
for lane_control_node, plus an annotated debug image.
"""

import os
import sys
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from vision.processor import LaneProcessor   # noqa: E402


# ──────────────────────────────────────────────────────────────────────────────
# Lightweight ROS Image <-> numpy converters (no cv_bridge / boost dependency)
# ──────────────────────────────────────────────────────────────────────────────

_ENCODING_CHANNELS = {
    'rgb8': 3, 'bgr8': 3,
    'rgba8': 4, 'bgra8': 4,
    'mono8': 1, '8UC1': 1,
    'R8G8B8': 3,    # Gazebo Harmonic raw format
}

def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to numpy BGR uint8 array without cv_bridge."""
    n_ch = _ENCODING_CHANNELS.get(msg.encoding, 3)
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, n_ch)

    enc = msg.encoding.lower()
    if enc in ('rgb8', 'r8g8b8'):
        return frame[:, :, ::-1].copy()
    if enc in ('rgba8',):
        return frame[:, :, 2::-1].copy()
    if enc in ('bgra8',):
        return frame[:, :, :3].copy()
    return frame.copy()


def bgr_to_imgmsg(frame: np.ndarray, stamp, frame_id: str) -> Image:
    """Convert numpy BGR uint8 to sensor_msgs/Image without cv_bridge."""
    msg = Image()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.height          = frame.shape[0]
    msg.width           = frame.shape[1]
    msg.encoding        = 'bgr8'
    msg.is_bigendian    = False
    msg.step            = frame.shape[1] * 3
    msg.data            = frame.tobytes()
    return msg


# ──────────────────────────────────────────────────────────────────────────────
class LaneFollowerNode(Node):

    def __init__(self):
        super().__init__('lane_follower_node')

        default_model = os.path.join(
            get_package_share_directory('main_bot'),
            'models', 'EgoLanes_Lite_FP32.onnx'
        )
        self.declare_parameter('model_path',   default_model)
        self.declare_parameter('cam_height',   0.134)
        self.declare_parameter('cam_pitch',    0.0)
        self.declare_parameter('cam_x_offset', 0.1485)
        self.declare_parameter('image_topic',  '/camera/image_raw')

        model_path   = self.get_parameter('model_path').value
        cam_height   = self.get_parameter('cam_height').value
        cam_pitch    = self.get_parameter('cam_pitch').value
        cam_x_offset = self.get_parameter('cam_x_offset').value
        image_topic  = self.get_parameter('image_topic').value

        self.get_logger().info(f'Model  : {model_path}')
        self.get_logger().info(
            f'Camera : height={cam_height:.3f}m  pitch={math.degrees(cam_pitch):.2f}deg'
        )

        try:
            self._proc = LaneProcessor(
                model_path=model_path,
                cam_height=cam_height,
                cam_pitch=cam_pitch,
                cam_x_offset=cam_x_offset,
            )
            self.get_logger().info('LaneProcessor initialised OK')
        except Exception as exc:
            self.get_logger().fatal(f'Failed to load model: {exc}')
            raise

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._sub = self.create_subscription(
            Image, image_topic,
            self._image_callback,
            sensor_qos,
        )

        self._pub_err = self.create_publisher(Vector3, '/status_err', 10)
        self._pub_img = self.create_publisher(Image,   '/processed_image', sensor_qos)

        self.get_logger().info(
            f'Listening on {image_topic} -> /status_err  /processed_image'
        )

        self._frame_count = 0

    def _image_callback(self, msg: Image):
        try:
            frame_bgr = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f'Image decode error: {exc}')
            return

        e_y, e_psi, kappa, valid, _, debug_frame = self._proc.process_frame(frame_bgr)

        now = self.get_clock().now().to_msg()

        if valid:
            err_msg   = Vector3()
            err_msg.x = float(e_y)    # cross-track error [m]
            err_msg.y = float(e_psi)  # heading error     [rad]
            err_msg.z = float(kappa)  # road curvature    [1/m]
            self._pub_err.publish(err_msg)

        img_msg = bgr_to_imgmsg(debug_frame, now, 'camera_link_optical')
        self._pub_img.publish(img_msg)

        # Throttled log: every 30 frames (~3s at 10Hz)
        self._frame_count += 1
        if self._frame_count % 30 == 0:
            if valid and (abs(e_y) > 1e-6 or abs(e_psi) > 1e-6):
                r_str = f'R={1/kappa:.1f}m' if abs(kappa) > 0.05 else 'straight'
                self.get_logger().info(
                    f'e_y={e_y:+.4f}m  e_psi={math.degrees(e_psi):+.2f}deg'
                    f'  kappa={kappa:+.3f} ({r_str})'
                )
            elif not valid:
                self.get_logger().warning('Lane LOST — waiting for re-detection')


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
