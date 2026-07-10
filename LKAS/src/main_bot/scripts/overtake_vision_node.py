#!/usr/bin/env python3
"""
overtake_vision_node.py — camera-based NPC detection, supplementary to the
LiDAR-based overtake pipeline.

Detects the orange NPC box (RGB 1.0, 0.45, 0.0) via an HSV color filter and
classifies detections into:
  - Front sector: NPC ahead in the robot's own lane (triggers overtake)
  - Adj sector  : NPC ahead in the outer lane (blocks overtake if occupied)

Publishes:
  /overtake/vision_front_dist  (std_msgs/Float32)  — front NPC distance [m], -1 = none
  /overtake/vision_adj_clear   (std_msgs/Bool)     — outer lane clear, camera's view
  /overtake/vision_debug       (sensor_msgs/Image) — debug image with bbox + sector lines

Subscribes:
  /camera/image_raw  (sensor_msgs/Image)
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3

# ── Camera intrinsics (camera.xacro: FOV_H=120°, 640x480) ──────────────────
_IMG_W = 640
_IMG_H = 480
_CX    = _IMG_W / 2.0
_CY    = _IMG_H / 2.0
_FX    = _IMG_W / (2.0 * math.tan(math.radians(60.0)))
_FY    = _FX

# ── NPC box dimensions (must match npc_driver_node.cpp) ─────────────────────
_NPC_WID = 0.190   # m
_NPC_HGT = 0.167   # m

# ── HSV range for the NPC's orange material ──────────────────────────────────
# ambient/diffuse RGB(1.0, 0.45, 0.0) -> RGB(255,115,0) -> HSV(H~14°, S=100%, V=100%)
# OpenCV convention: H in 0-180, S/V in 0-255. Range widened for Gazebo lighting variance.
_HSV_LO = np.array([ 5, 110,  50], dtype=np.uint8)
_HSV_HI = np.array([25, 255, 255], dtype=np.uint8)

# ── ROI: excludes sky (top) and the road right in front of the robot (bottom) ─
_ROI_TOP = 80
_ROI_BOT = 420

# ── Contour size filter ───────────────────────────────────────────────────────
_MIN_AREA = 25     # px^2 — rejects small noise
_MAX_AREA = 9000   # px^2 — largest expected NPC blob is ~600px^2 at 2m

# ── Position classification in robot-frame coordinates ──────────────────────
# Outer lane offset from inner: 2.801 - 2.267 = 0.534m
_FRONT_Y_MAX = 0.40   # m — same-lane NPC (within +-0.40m of center)
_ADJ_Y_MIN   = 0.15   # m — start of the outer lane band
_ADJ_Y_MAX   = 1.10   # m — outer edge of the outer lane band

# ── EMA smoothing ──────────────────────────────────────────────────────────
_ALPHA_DIST  = 0.40
_ALPHA_ADJ   = 0.45
_ADJ_THRESH  = 0.5    # adj score above this -> outer lane considered clear

_IMG_TIMEOUT_S = 1.5   # no frames for this long -> reset to defaults

# ── Distance estimate sanity bounds ─────────────────────────────────────────
_DIST_MIN = 0.25   # m — closer than this, the estimate is unreliable
_DIST_MAX = 8.0    # m — farther than this, the bbox is too small (<4px)


class OvertakeVisionNode(Node):
    """
    Detects the NPC by camera and publishes front-NPC distance and outer-lane
    occupancy.

    Pipeline:
        0. HSV filter -> binary orange mask
        1. Morphology + contour detection -> bounding boxes
        2. Geometric classification -> front / adjacent lane
        3. Distance estimation from bbox size (known NPC dimensions)
        4. EMA smoothing to suppress oscillation
    """

    def __init__(self):
        super().__init__('overtake_vision_node')

        self.declare_parameter('image_topic',   '/camera/image_raw')
        self.declare_parameter('publish_debug', True)

        image_topic   = self.get_parameter('image_topic').value
        self._pub_dbg = self.get_parameter('publish_debug').value

        # Curvature from lane_follower (kappa = vyaw/vx), used to compensate
        # the camera's viewing angle on curves.
        self._kappa: float = 0.0

        self._dist_ema: float  = -1.0   # -1 = not detected
        self._adj_score: float =  1.0   # 1.0 = clear, 0.0 = occupied
        self._has_init: bool   = False
        self._last_img_t: float = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._sub = self.create_subscription(
            Image, image_topic, self._img_cb, sensor_qos)

        self.create_subscription(
            Vector3, '/status_err',
            lambda msg: setattr(self, '_kappa', float(msg.z)),
            10)

        self._pub_dist  = self.create_publisher(Float32, '/overtake/vision_front_dist', 10)
        self._pub_adj   = self.create_publisher(Bool,   '/overtake/vision_adj_clear',  10)
        self._pub_debug = self.create_publisher(Image,  '/overtake/vision_debug', 10)

        self.create_timer(0.10, self._timer_cb)

        self.get_logger().info(
            f'[overtake_vision] ready — topic={image_topic}'
            f'  FX={_FX:.1f}px  ADJ_offset=0.534m'
        )

    @staticmethod
    def _to_bgr(msg: Image) -> np.ndarray:
        enc = msg.encoding.lower()
        nc = {'rgb8': 3, 'bgr8': 3, 'r8g8b8': 3,
              'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(enc, 3)
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, nc)
        if enc in ('rgb8', 'r8g8b8'):
            return frame[:, :, ::-1].copy()
        if enc == 'rgba8':
            return frame[:, :, 2::-1].copy()
        if enc == 'bgra8':
            return frame[:, :, :3].copy()
        return frame.copy()

    @staticmethod
    def _estimate_dist(bbox_w: float, bbox_h: float) -> float:
        """Pinhole distance estimate D = f * L_real / L_px, weighted toward
        height since it's less prone to horizontal clipping than width."""
        d_w = _FX * _NPC_WID / max(bbox_w, 1.0)
        d_h = _FY * _NPC_HGT / max(bbox_h, 1.0)
        return (d_w + 1.5 * d_h) / 2.5

    @staticmethod
    def _lateral_y(col_px: float, dist_m: float, kappa: float = 0.0) -> float:
        """Column pixel -> lateral Y in the robot frame (Y>0 = left / outer
        lane direction), compensated for curvature (a curve shifts the NPC's
        apparent column by FX*D*kappa pixels)."""
        col_corrected = col_px - _FX * dist_m * kappa
        return (_CX - col_corrected) * dist_m / _FX

    def _img_cb(self, msg: Image):
        self._last_img_t = self.get_clock().now().nanoseconds * 1e-9

        try:
            bgr = self._to_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f'[overtake_vision] decode error: {exc}')
            return

        h_img, w_img = bgr.shape[:2]

        roi = bgr[_ROI_TOP:_ROI_BOT, :]
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, _HSV_LO, _HSV_HI)

        k    = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)

        front_dists: list[float] = []
        adj_occupied = False
        debug_rects  = []   # (x, y, w, h, color, label)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < _MIN_AREA or area > _MAX_AREA:
                continue

            bx, by, bw, bh = cv2.boundingRect(cnt)
            cx_box = bx + bw / 2.0
            col_img = cx_box
            row_img = by + _ROI_TOP + bh / 2.0

            d = self._estimate_dist(float(bw), float(bh))
            if not (_DIST_MIN < d < _DIST_MAX):
                continue

            Y = self._lateral_y(col_img, d, self._kappa)

            is_front = abs(Y) < _FRONT_Y_MAX
            is_adj   = _ADJ_Y_MIN < Y < _ADJ_Y_MAX

            if is_front:
                front_dists.append(d)
                debug_rects.append((
                    bx, by + _ROI_TOP, bw, bh,
                    (0, 80, 255),
                    f'F {d:.1f}m Y={Y:+.2f}',
                ))

            if is_adj:
                adj_occupied = True
                debug_rects.append((
                    bx, by + _ROI_TOP, bw, bh,
                    (255, 80, 0),
                    f'A {d:.1f}m Y={Y:+.2f}',
                ))

        # Front distance EMA
        raw_dist = min(front_dists) if front_dists else -1.0

        if raw_dist > 0:
            if not self._has_init:
                self._dist_ema = raw_dist
                self._has_init = True
            else:
                self._dist_ema = (_ALPHA_DIST * raw_dist
                                  + (1.0 - _ALPHA_DIST) * self._dist_ema)
        else:
            if self._dist_ema > 0:
                # Decay on a momentary loss of detection rather than snapping to -1
                self._dist_ema *= (1.0 - _ALPHA_DIST)
                if self._dist_ema < _DIST_MIN:
                    self._dist_ema = -1.0
                    self._has_init = False
            else:
                self._dist_ema = -1.0

        raw_adj = 0.0 if adj_occupied else 1.0
        self._adj_score = (_ALPHA_ADJ * raw_adj
                           + (1.0 - _ALPHA_ADJ) * self._adj_score)

        if self._pub_dbg:
            self._publish_debug(bgr, mask, debug_rects, msg.header.stamp)

    def _publish_debug(self, bgr, mask_roi, rects, stamp):
        dbg = bgr.copy()

        mask_full = np.zeros(bgr.shape[:2], dtype=np.uint8)
        mask_full[_ROI_TOP:_ROI_BOT, :] = mask_roi
        green_layer = np.zeros_like(dbg)
        green_layer[:, :, 1] = 100
        dbg = np.where(mask_full[:, :, None] > 0, green_layer, dbg).astype(np.uint8)

        cv2.rectangle(dbg,
                      (0, _ROI_TOP), (dbg.shape[1]-1, _ROI_BOT),
                      (150, 150, 150), 1)

        # Sector boundary lines projected at D=3m: col = CX - FX * Y / D
        col_f_l = int(_CX + _FX * _FRONT_Y_MAX / 3.0)
        col_f_r = int(_CX - _FX * _FRONT_Y_MAX / 3.0)
        col_a_l = int(_CX - _FX * _ADJ_Y_MIN  / 3.0)
        col_a_r = int(_CX - _FX * _ADJ_Y_MAX  / 3.0)

        for cx, clr, lbl in [
            (col_f_l, (0, 200, 80),   'F-R'),
            (col_f_r, (0, 200, 80),   'F-L'),
            (col_a_l, (200, 150, 0),  'A-R'),
            (col_a_r, (200, 150, 0),  'A-L'),
        ]:
            if 0 < cx < dbg.shape[1]:
                cv2.line(dbg, (cx, _ROI_TOP), (cx, _ROI_BOT), clr, 1)
                cv2.putText(dbg, lbl, (cx-10, _ROI_TOP+12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, clr, 1)

        for (bx, by, bw, bh, clr, lbl) in rects:
            cv2.rectangle(dbg, (bx, by), (bx+bw, by+bh), clr, 2)
            cv2.putText(dbg, lbl, (bx, max(by-4, 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, clr, 1)

        d_str  = f'{self._dist_ema:.2f}m' if self._dist_ema > 0 else 'none'
        adj_ok = self._adj_score > _ADJ_THRESH
        adj_str = f'CLR({self._adj_score:.2f})' if adj_ok else f'OCC({self._adj_score:.2f})'
        cv2.putText(dbg, f'front={d_str}  adj={adj_str}  k={self._kappa:+.2f}',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        out = Image()
        out.header.stamp    = stamp
        out.header.frame_id = 'camera_link_optical'
        out.height    = dbg.shape[0]
        out.width     = dbg.shape[1]
        out.encoding  = 'bgr8'
        out.is_bigendian = False
        out.step      = dbg.shape[1] * 3
        out.data      = dbg.tobytes()
        self._pub_debug.publish(out)

    def _timer_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._last_img_t > 0 and (now - self._last_img_t) > _IMG_TIMEOUT_S:
            if self._dist_ema > 0 or self._adj_score < 1.0:
                self.get_logger().warning('[overtake_vision] image timeout — reset')
            self._dist_ema  = -1.0
            self._adj_score =  1.0
            self._has_init  = False

        dist_msg      = Float32()
        dist_msg.data = float(self._dist_ema)
        self._pub_dist.publish(dist_msg)

        adj_msg      = Bool()
        adj_msg.data = bool(self._adj_score > _ADJ_THRESH)
        self._pub_adj.publish(adj_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OvertakeVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
