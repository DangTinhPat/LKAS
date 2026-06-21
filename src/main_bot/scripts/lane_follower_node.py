#!/usr/bin/env python3
"""
lane_follower_node.py — ROS 2 wrapper around LaneProcessor.

Subscribes  : /camera/image_raw  (sensor_msgs/msg/Image)
Publishes   : /status_err        (geometry_msgs/msg/Vector3)  x=e_y [m], y=e_psi [rad], z=valid
              /processed_image   (sensor_msgs/msg/Image)      annotated BGR frame

Parameters (settable via launch / --ros-args -p):
  model_path   : str   — path to EgoLanes_Lite_FP32.onnx
  cam_height   : float — camera height above ground [m]        (default 0.134)
  cam_pitch    : float — camera pitch angle [rad]              (default 0.0)
  cam_x_offset : float — camera forward offset from vehicle [m](default 0.1485)
  image_topic  : str   — input image topic                    (default /camera/image_raw)

Note: cv_bridge is intentionally NOT used to avoid numpy 1.x / 2.x ABI conflicts.
      Image conversion is done with plain numpy, which works regardless of numpy version.
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

# lane_processor.py sits in the same lib/main_bot/ directory → ensure importable
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from lane_processor import LaneProcessor, CameraConfig   # noqa: E402


# ──────────────────────────────────────────────────────────────────────────────
# Lightweight ROS Image ↔ numpy converters (no cv_bridge / boost dependency)
# ──────────────────────────────────────────────────────────────────────────────

_ENCODING_CHANNELS = {
    'rgb8': 3, 'bgr8': 3,
    'rgba8': 4, 'bgra8': 4,
    'mono8': 1, '8UC1': 1,
    'R8G8B8': 3,    # Gazebo Harmonic raw format
}

def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image → numpy BGR uint8 array without cv_bridge."""
    n_ch = _ENCODING_CHANNELS.get(msg.encoding, 3)
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, n_ch)

    enc = msg.encoding.lower()
    if enc in ('rgb8', 'r8g8b8'):
        return frame[:, :, ::-1].copy()   # RGB → BGR
    if enc in ('rgba8',):
        return frame[:, :, 2::-1].copy()  # RGBA → BGR
    if enc in ('bgra8',):
        return frame[:, :, :3].copy()     # BGRA → BGR
    return frame.copy()                   # already BGR / mono


def bgr_to_imgmsg(frame: np.ndarray, stamp, frame_id: str) -> Image:
    """Convert numpy BGR uint8 → sensor_msgs/Image without cv_bridge."""
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

        # ── Parameters ────────────────────────────────────────────────────
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

        # ── Lane processor ────────────────────────────────────────────────
        cfg = CameraConfig(
            cam_height=cam_height,
            cam_pitch=cam_pitch,
            cam_x_offset=cam_x_offset,
        )
        try:
            self._proc = LaneProcessor(model_path, cfg)
            self.get_logger().info('LaneProcessor initialised OK')
        except Exception as exc:
            self.get_logger().fatal(f'Failed to load model: {exc}')
            raise

        # ── QoS ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriber ────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Image, image_topic,
            self._image_callback,
            sensor_qos,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_err = self.create_publisher(Vector3, '/status_err', 10)
        self._pub_img = self.create_publisher(Image,   '/processed_image', sensor_qos)

        self.get_logger().info(
            f'Listening on {image_topic} → /status_err  /processed_image'
        )

        self._frame_count = 0

    # ──────────────────────────────────────────────────────────────────────
    def _image_callback(self, msg: Image):
        # ── ROS Image → numpy BGR ─────────────────────────────────────────
        try:
            frame_bgr = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f'Image decode error: {exc}')
            return

        # ── Lane processing ───────────────────────────────────────────────
        e_y, e_psi, valid, debug_frame = self._proc.process(frame_bgr)

        now = self.get_clock().now().to_msg()

        # ── Publish /status_err ───────────────────────────────────────────
        err_msg   = Vector3()
        err_msg.x = float(e_y)               # cross-track error [m]
        err_msg.y = float(e_psi)             # heading error     [rad]
        err_msg.z = 1.0 if valid else 0.0   # detection valid flag
        self._pub_err.publish(err_msg)

        # ── Publish /processed_image ──────────────────────────────────────
        img_msg = bgr_to_imgmsg(debug_frame, now, 'camera_link_optical')
        self._pub_img.publish(img_msg)

        # ── Throttled log (every 30 frames ≈ 3 s at 10 Hz) ──────────────
        self._frame_count += 1
        if self._frame_count % 30 == 0:
            if valid:
                self.get_logger().info(
                    f'e_y={e_y:+.4f}m  e_psi={math.degrees(e_psi):+.2f}deg'
                )
            else:
                self.get_logger().info('No lane detected')


# ──────────────────────────────────────────────────────────────────────────────
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
