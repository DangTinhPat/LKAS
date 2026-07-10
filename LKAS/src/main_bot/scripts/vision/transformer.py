#!/usr/bin/env python3
"""
transformer.py — camera geometry and inverse perspective mapping (IPM).

Builds the pinhole intrinsic matrix K and projects between image pixels and
ground-plane world coordinates:
  - pixel_to_vehicle : image pixel -> ground coordinates (m, ROS frame)
  - vehicle_to_pixel : ground coordinates (m) -> image pixel (for debug draw)

No ROS 2 dependency.

Axis convention (REP-103): X = forward, Y = left, Z = up.

Camera parameters (from robot_core.xacro / camera.xacro):
  Resolution   : 640 x 480 px
  Horizontal FOV: 2.094 rad
  Lens height  : 0.134 m above ground
  X offset     : 0.1485 m (camera mounted at the front of the chassis)
  Pitch        : 0.0 rad (level, rpy="0 0 0")
"""

import math
from typing import Optional, Tuple

import numpy as np
import cv2

_MIN_X_M = 0.05   # m — ignore points directly under the camera (geometric noise)
_MAX_X_M = 4.0    # m — ignore far projections (numerically unstable)


class GeometryTransformer:
    """
    Analytic inverse perspective mapping under a flat-ground pinhole model.

    Intrinsics from resolution and horizontal FOV:
        fx = W / (2 * tan(h_fov / 2)),  fy = fx (square pixels)
        cx = W / 2,  cy = H / 2

    Forward projection (pixel -> ground) solves the ray/ground-plane
    intersection: unproject with K^-1, rotate into the vehicle frame by the
    camera pitch, then solve for the ray parameter t where z = 0.
    """

    def __init__(
        self,
        img_w:       int   = 640,
        img_h:       int   = 480,
        h_fov:       float = 2.094,
        cam_height:  float = 0.134,
        cam_pitch:   float = 0.0,
        cam_x_offset: float = 0.1485,
    ):
        self.img_w        = img_w
        self.img_h        = img_h
        self.cam_height   = cam_height
        self.cam_pitch    = cam_pitch
        self.cam_x_offset = cam_x_offset

        self.fx = img_w / (2.0 * math.tan(h_fov / 2.0))
        self.fy = self.fx
        self.cx = img_w / 2.0
        self.cy = img_h / 2.0

        # Per-row meters/pixel (lateral) lookup, used to estimate lane width
        # in pixels when only one lane marking is visible.
        self._row_lateral_scale: dict = {}
        self._precompute_row_scales()

    def _precompute_row_scales(self):
        """Precomputes, for each row below the horizon, |delta_Y| in meters
        per 1px horizontal step (lateral_scale[v])."""
        for v in range(int(self.cy) + 1, self.img_h):
            pt_center = self._project_raw(self.cx,        float(v))
            pt_side   = self._project_raw(self.cx + 1.0, float(v))
            if pt_center is None or pt_side is None:
                continue
            scale = abs(pt_side[1] - pt_center[1])
            if scale > 1e-7:
                self._row_lateral_scale[v] = scale

    def _project_raw(
        self, u: float, v: float
    ) -> Optional[Tuple[float, float]]:
        """
        Casts a ray from pixel (u, v) and intersects it with the ground
        plane (z=0), unbounded by the valid-range filter.

          du, dv = (u-cx)/fx, (v-cy)/fy         — normalised pixel offsets
          r_veh_z = -sin(p) - cos(p)*dv          — ray's vertical component
                                                    (must be < 0, i.e. looking down)
          t = h / (-r_veh_z)                     — distance along the ray to z=0
          r_veh_x = cos(p) - sin(p)*dv
          X = cam_x_offset + t*r_veh_x           — forward distance
          Y = t*(-du)                            — lateral distance (+left)

        r_y = -du because a pixel right of center (u>cx, du>0) projects to
        the vehicle's right (Y<0), and vice versa.
        """
        p  = self.cam_pitch
        h  = self.cam_height
        du = (u - self.cx) / self.fx
        dv = (v - self.cy) / self.fy

        r_veh_z = -math.sin(p) - math.cos(p) * dv
        if r_veh_z >= -1e-6:
            return None  # ray points up/parallel — no ground intersection

        t = h / (-r_veh_z)
        r_veh_x = math.cos(p) - math.sin(p) * dv

        X = self.cam_x_offset + t * r_veh_x
        Y = t * (-du)
        return X, Y

    def pixel_to_vehicle(
        self, px: float, py: float
    ) -> Optional[Tuple[float, float]]:
        """
        Original-resolution pixel (px, py) -> ground coordinates (X_m, Y_m)
        in the ROS vehicle frame, or None if the pixel is above the horizon
        or projects outside the valid [_MIN_X_M, _MAX_X_M] range.
        """
        pt = self._project_raw(px, py)
        if pt is None:
            return None
        X, Y = pt
        if not (_MIN_X_M < X < _MAX_X_M):
            return None
        return X, Y

    def vehicle_to_pixel(
        self, mx: float, my: float
    ) -> Optional[Tuple[int, int]]:
        """
        Ground coordinates (mx, my) in the vehicle frame -> pixel on the
        original 640x480 image, or None if outside the valid view. Used for
        drawing predicted lane centers / test points on the debug image.

        Algebraic inverse of _project_raw: solve for dv from X, recover v,
        then recover t and finally u from Y.
        """
        p      = self.cam_pitch
        h      = self.cam_height
        cam_x  = self.cam_x_offset

        if abs(mx - cam_x) < 1e-6:
            return None  # degenerate: point coincides with the camera position

        r    = (mx - cam_x) / h
        den  = r * math.cos(p) + math.sin(p)
        if abs(den) < 1e-9:
            return None
        dv = (math.cos(p) - r * math.sin(p)) / den

        v_f = self.cy + dv * self.fy
        if not (self.cy < v_f < self.img_h):
            return None

        r_veh_z = -math.sin(p) - math.cos(p) * dv
        if r_veh_z >= -1e-6:
            return None
        t = h / (-r_veh_z)

        u_f = self.cx - my * self.fx / t
        if not (0 <= u_f < self.img_w):
            return None

        return int(round(u_f)), int(round(v_f))

    def lateral_scale_at(self, row: int) -> Optional[float]:
        """Meters/pixel lateral scale at image `row`, or None if that row
        has no valid IPM data. Used to convert an assumed lane half-width
        from meters to pixels for single-marking rows."""
        return self._row_lateral_scale.get(row)
