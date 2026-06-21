#!/usr/bin/env python3
"""
lane_processor.py — Pure lane-detection pipeline (no ROS 2).

Camera geometry (derived from main_bot/description/camera.xacro & robot_core.xacro):
  - Resolution        : 640 × 480 px, horizontal FOV = 1.089 rad
  - chassis_height    : 0.062 m  |  ground_clearance : 0.022 m
  - chassis_z_offset  : 0.053 m  (height of base_link above ground)
  - cam_z_above_base  : 0.081 m  (chassis_height/2 + 0.05)
  - cam_height_ground : 0.134 m  (total above ground)
  - cam_x_offset      : 0.1485 m (chassis_length/2, camera is at front face)
  - default pitch     : 0.0 rad  (looking straight ahead, rpy="0 0 0")

Model: EgoLanes_Lite_FP32.onnx
  - Input  : [1, 3, 320, 640]  float32, ImageNet-normalised RGB
  - Output : [1, 3, 320, 640]  logits, multi-label per channel (NOT argmax)
              ch0 = left lane   →  left_mask  = (logits[0] > 0)
              ch1 = right lane  →  right_mask = (logits[1] > 0)
              ch2 = other lane  →  (ignored for ego-lane tracking)
              Background/sky logits are negative → thresholding naturally suppresses them
"""

import math
import numpy as np
import cv2
import onnxruntime as ort
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

# ──────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────
MODEL_H, MODEL_W = 320, 640          # model input/output height & width

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

CLS_BG    = 0   # background / off-road
CLS_LEFT  = 1   # left ego-lane marking
CLS_RIGHT = 2   # right ego-lane marking

LOOKAHEAD_M              = 0.8    # m ahead of vehicle center for e_y evaluation
ASSUMED_LANE_HALF_WIDTH_M = 0.22  # fallback half-lane-width when only one line visible
MIN_GROUND_PTS           = 8     # minimum points to accept a line fit
MAX_GROUND_X             = 4.0   # m — discard far projections (noise)
MIN_GROUND_X             = 0.05  # m — discard projections right under camera


# ──────────────────────────────────────────────
# Camera configuration
# ──────────────────────────────────────────────
@dataclass
class CameraConfig:
    img_w: int   = 640
    img_h: int   = 480
    h_fov: float = 1.089    # rad, horizontal FOV
    cam_height: float = 0.134   # m, camera lens above ground
    cam_pitch:  float = 0.0     # rad, positive = tilting nose down toward ground
    cam_x_offset: float = 0.1485  # m, camera forward offset from vehicle center


# ──────────────────────────────────────────────
# Main processor class
# ──────────────────────────────────────────────
class LaneProcessor:
    """
    Stateless (per-frame) lane processor.

    Call  process(frame_bgr)  to get (e_y, e_psi, valid, debug_frame).
    """

    def __init__(self, model_path: str, camera_cfg: CameraConfig = CameraConfig()):
        self.cfg = camera_cfg

        # ── ONNX runtime ──
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self._sess = ort.InferenceSession(model_path, providers=providers)
        self._inp_name = self._sess.get_inputs()[0].name

        # ── Camera intrinsics (original image) ──
        fx = camera_cfg.img_w / (2.0 * math.tan(camera_cfg.h_fov / 2.0))
        fy = fx                          # square-pixel assumption
        cx = camera_cfg.img_w / 2.0
        cy = camera_cfg.img_h / 2.0

        # Scale to model input size
        sx = MODEL_W / camera_cfg.img_w  # = 1.0 (width unchanged)
        sy = MODEL_H / camera_cfg.img_h  # = 320/480 ≈ 0.667

        self._fx = fx * sx   # ≈ 532.5 px
        self._fy = fy * sy   # ≈ 355.0 px
        self._cx = cx * sx   # = 320.0 px
        self._cy = cy * sy   # = 160.0 px

        # ── Precompute IPM helpers for each valid row ──
        self._row_meta: dict = {}   # v → (t_at_cx, m_per_px_y, X_at_cx)
        self._precompute_ipm_rows()

    # ──────────────────────────────────────
    # Internal helpers
    # ──────────────────────────────────────

    def _precompute_ipm_rows(self):
        """Cache IPM scale factors for rows below the horizon."""
        for v in range(int(self._cy) + 1, MODEL_H):
            pt = self._pixel_to_ground_raw(self._cx, v)
            if pt is None:
                continue
            X_center, _ = pt
            if not (MIN_GROUND_X < X_center < MAX_GROUND_X):
                continue
            # meters-per-pixel in the lateral (Y) direction at this row
            pt_side = self._pixel_to_ground_raw(self._cx + 1.0, v)
            if pt_side is None:
                continue
            m_per_px = abs(pt_side[1] - pt[1])
            if m_per_px < 1e-7:
                continue
            # t (depth parameter) for a pixel at the horizon column
            t = self._ray_t(v)
            self._row_meta[v] = dict(t=t, m_per_px=m_per_px, X=X_center)

    def _ray_t(self, v: float) -> Optional[float]:
        """Depth parameter t where the ray through (cx, v) hits the ground."""
        p = self.cfg.cam_pitch
        h = self.cfg.cam_height
        dv = (v - self._cy) / self._fy
        r_veh_z = -math.sin(p) - math.cos(p) * dv
        if r_veh_z >= -1e-6:
            return None
        return h / (-r_veh_z)

    def _pixel_to_ground_raw(
        self, u: float, v: float
    ) -> Optional[Tuple[float, float]]:
        """Map model-image pixel (u, v) → vehicle-frame ground (X, Y) meters.

        X : forward from vehicle center  (positive = ahead)
        Y : left from vehicle center     (positive = left, standard ROS convention)
        Returns None when pixel is above the horizon or behind camera.
        """
        p  = self.cfg.cam_pitch
        h  = self.cfg.cam_height
        du = (u - self._cx) / self._fx
        dv = (v - self._cy) / self._fy

        # Vehicle-frame Z component of ray (negative means going toward ground)
        r_veh_z = -math.sin(p) - math.cos(p) * dv
        if r_veh_z >= -1e-6:
            return None          # ray aims up or horizontal — no ground intersection

        t = h / (-r_veh_z)      # distance along ray to ground plane

        # Vehicle-frame X component of ray (forward direction)
        r_veh_x = math.cos(p) - math.sin(p) * dv

        X = self.cfg.cam_x_offset + t * r_veh_x
        Y = t * (-du)            # left is positive

        return X, Y

    def pixel_to_ground(
        self, u: float, v: float
    ) -> Optional[Tuple[float, float]]:
        """Public wrapper with range filtering."""
        pt = self._pixel_to_ground_raw(u, v)
        if pt is None:
            return None
        X, Y = pt
        if not (MIN_GROUND_X < X < MAX_GROUND_X):
            return None
        return X, Y

    def ground_to_pixel(
        self, X: float, Y: float
    ) -> Optional[Tuple[int, int]]:
        """Inverse projection: ground (X, Y) → model-image pixel (u, v).

        Solves the IPM forward equations analytically.
        """
        p = self.cfg.cam_pitch
        h = self.cfg.cam_height
        cam_x = self.cfg.cam_x_offset

        if abs(X - cam_x) < 1e-6:
            return None

        # Solve for dv given X (see pixel_to_ground derivation)
        # X = cam_x + h * (cos p − sin p · dv) / (sin p + cos p · dv)
        # Let r = (X − cam_x) / h
        # r · (sin p + cos p · dv) = cos p − sin p · dv
        # dv · (r · cos p + sin p) = cos p − r · sin p
        r    = (X - cam_x) / h
        den  = r * math.cos(p) + math.sin(p)
        if abs(den) < 1e-9:
            return None
        dv = (math.cos(p) - r * math.sin(p)) / den

        v = self._cy + dv * self._fy
        if not (self._cy < v < MODEL_H):
            return None

        # t at this row (recompute)
        r_veh_z = -math.sin(p) - math.cos(p) * dv
        if r_veh_z >= -1e-6:
            return None
        t = h / (-r_veh_z)

        # u from Y
        # Y = t · (-(u - cx) / fx)  →  u = cx - Y * fx / t
        u = self._cx - Y * self._fx / t
        if not (0 <= u < MODEL_W):
            return None

        # Scale back to original image coordinates
        u_orig = int(round(u * self.cfg.img_w / MODEL_W))
        v_orig = int(round(v * self.cfg.img_h / MODEL_H))
        return u_orig, v_orig

    # ──────────────────────────────────────
    # Pipeline steps
    # ──────────────────────────────────────

    def preprocess(self, frame_bgr: np.ndarray) -> np.ndarray:
        """BGR 640×480 → NCHW float32 tensor [1, 3, 320, 640]."""
        rgb    = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
        img    = resized.astype(np.float32) / 255.0
        img    = (img - IMAGENET_MEAN) / IMAGENET_STD
        tensor = img.transpose(2, 0, 1)[np.newaxis]  # [1, 3, H, W]
        return np.ascontiguousarray(tensor, dtype=np.float32)

    def infer(self, tensor: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Run model, return (left_mask, right_mask) binary uint8 [H, W].

        Each channel is thresholded independently at logit=0 (sigmoid=0.5).
        Sky/background has negative logits → automatically becomes 0.
        """
        logits = self._sess.run(None, {self._inp_name: tensor})[0]  # [1, 3, H, W]
        pred   = logits[0]                                           # [3, H, W]
        left_mask  = (pred[0] > 0).astype(np.uint8)                 # ch0: left lane
        right_mask = (pred[1] > 0).astype(np.uint8)                 # ch1: right lane
        return left_mask, right_mask

    def extract_lane_centers(
        self,
        left_mask:  np.ndarray,   # [MODEL_H, MODEL_W] binary uint8
        right_mask: np.ndarray,   # [MODEL_H, MODEL_W] binary uint8
    ) -> List[Tuple[float, int]]:
        """Row-by-row: find (u_center, v) for rows with detected lane pixels.

        Uses independent binary masks — no argmax ambiguity.
        When only one side is visible, estimates center from assumed lane width.
        """
        centers: List[Tuple[float, int]] = []

        for v, meta in self._row_meta.items():
            l_idx = np.where(left_mask[v]  > 0)[0]
            r_idx = np.where(right_mask[v] > 0)[0]

            if l_idx.size > 0 and r_idx.size > 0:
                u_center = 0.5 * (float(l_idx.mean()) + float(r_idx.mean()))
            elif l_idx.size > 0:
                half_px  = ASSUMED_LANE_HALF_WIDTH_M / meta['m_per_px']
                u_center = float(l_idx.mean()) + half_px
            elif r_idx.size > 0:
                half_px  = ASSUMED_LANE_HALF_WIDTH_M / meta['m_per_px']
                u_center = float(r_idx.mean()) - half_px
            else:
                continue

            centers.append((u_center, v))

        return centers

    def fit_lane_line(
        self, centers: List[Tuple[float, int]]
    ) -> Optional[Tuple[float, float, np.ndarray]]:
        """Project centers to ground, fit Y = a·X + b.

        Returns (a, b, ground_pts_array) or None when not enough points.
        """
        pts: List[Tuple[float, float]] = []
        for u, v in centers:
            gpt = self.pixel_to_ground(u, v)
            if gpt is not None:
                pts.append(gpt)

        if len(pts) < MIN_GROUND_PTS:
            return None

        arr = np.array(pts, dtype=np.float64)
        Xv, Yv = arr[:, 0], arr[:, 1]
        try:
            a, b = np.polyfit(Xv, Yv, 1)
        except np.linalg.LinAlgError:
            return None

        return float(a), float(b), arr

    def compute_errors(
        self, a: float, b: float
    ) -> Tuple[float, float]:
        """
        e_y  [m]   : lateral offset of lane center at look-ahead distance
                     (Y = a·X + b  evaluated at X = LOOKAHEAD_M)
        e_psi [rad]: heading error = angle of fitted lane w.r.t. forward axis
        """
        e_y   = a * LOOKAHEAD_M + b
        e_psi = math.atan(a)
        return e_y, e_psi

    # ──────────────────────────────────────
    # Debug visualisation
    # ──────────────────────────────────────

    def draw_debug(
        self,
        frame_bgr:  np.ndarray,
        left_mask:  np.ndarray,   # [MODEL_H, MODEL_W] binary uint8
        right_mask: np.ndarray,   # [MODEL_H, MODEL_W] binary uint8
        e_y:        float,
        e_psi:      float,
        valid:      bool,
        a: float = 0.0,
        b: float = 0.0,
    ) -> np.ndarray:
        """Return frame_bgr with semi-transparent lane overlay, fitted line, and text.

        Each binary mask is upscaled with INTER_LINEAR then re-thresholded so
        lane edges are anti-aliased. Sky/off-road pixels never receive colour.
        """
        H, W = frame_bgr.shape[:2]
        vis  = frame_bgr.copy()

        # ── Upscale each mask independently (smooth edges, no argmax artefacts) ──
        def _upscale(mask: np.ndarray) -> np.ndarray:
            up = cv2.resize(mask.astype(np.float32), (W, H),
                            interpolation=cv2.INTER_LINEAR)
            return up > 0.3    # re-threshold after bilinear interpolation

        lm_up = _upscale(left_mask)
        rm_up = _upscale(right_mask)

        # ── Semi-transparent colour only on detected lane pixels ──
        overlay = vis.copy()
        overlay[lm_up] = (255, 100,   0)   # blue-ish  = left lane
        overlay[rm_up] = (  0,  80, 255)   # red-ish   = right lane
        vis = cv2.addWeighted(vis, 0.55, overlay, 0.45, 0)

        # ── Projected fitted lane-center line ──
        if valid:
            prev_pt = None
            for X in np.linspace(MIN_GROUND_X + 0.05, MAX_GROUND_X - 0.5, 40):
                Y  = a * X + b
                px = self.ground_to_pixel(X, Y)
                if px is not None:
                    if prev_pt is not None:
                        cv2.line(vis, prev_pt, px, (0, 255, 0), 2)
                    prev_pt = px

        # ── Red dot: vehicle center projected to image bottom ──
        vc_u = W // 2
        vc_v = H - 12
        cv2.circle(vis, (vc_u, vc_v), 7, (0, 0, 255), -1)
        cv2.line(vis, (vc_u, vc_v - 14), (vc_u, vc_v + 14), (0, 0, 255), 2)

        # ── Text overlay ──
        status_text  = 'TRACKING' if valid else 'NO LANE'
        status_color = (0, 220, 0) if valid else (0, 50, 255)

        cv2.putText(vis, status_text,
                    (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)
        cv2.putText(vis, f'e_y  = {e_y:+.4f} m',
                    (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
        cv2.putText(vis, f'e_psi= {math.degrees(e_psi):+.2f} deg',
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)

        return vis

    # ──────────────────────────────────────
    # Full pipeline
    # ──────────────────────────────────────

    def process(
        self, frame_bgr: np.ndarray
    ) -> Tuple[float, float, bool, np.ndarray]:
        """
        End-to-end pipeline.

        Parameters
        ----------
        frame_bgr : np.ndarray  shape (H, W, 3) BGR uint8

        Returns
        -------
        e_y        : float  cross-track error [m],  +left / -right
        e_psi      : float  heading error     [rad], +left / -right
        valid      : bool   True when lane was detected
        debug_frame: np.ndarray  annotated BGR image (same size as input)
        """
        try:
            tensor               = self.preprocess(frame_bgr)
            left_mask, right_mask = self.infer(tensor)
            centers              = self.extract_lane_centers(left_mask, right_mask)
            result               = self.fit_lane_line(centers)

            if result is not None:
                a, b, _ = result
                e_y, e_psi = self.compute_errors(a, b)
                valid      = True
            else:
                a = b = e_y = e_psi = 0.0
                valid = False

            debug = self.draw_debug(
                frame_bgr, left_mask, right_mask, e_y, e_psi, valid, a, b
            )
            return e_y, e_psi, valid, debug

        except Exception as exc:
            debug = frame_bgr.copy()
            cv2.putText(
                debug, f'ERR: {exc}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1
            )
            return 0.0, 0.0, False, debug


# ──────────────────────────────────────────────────────────────────────────────
# Standalone test (no ROS 2)
#   python3 lane_processor.py                 ← synthetic image test
#   python3 lane_processor.py video.mp4       ← video file test
# ──────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import sys, os

    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, '..', 'models', 'EgoLanes_Lite_FP32.onnx')

    cfg  = CameraConfig()
    proc = LaneProcessor(model_path, cfg)

    # ── Print camera geometry ──
    print('=== Camera geometry ===')
    print(f'  fx_model = {proc._fx:.2f} px')
    print(f'  fy_model = {proc._fy:.2f} px')
    print(f'  cx_model = {proc._cx:.2f} px')
    print(f'  cy_model = {proc._cy:.2f} px')
    print(f'  cam_height = {cfg.cam_height:.4f} m')
    print(f'  cam_pitch  = {math.degrees(cfg.cam_pitch):.2f} deg')
    print(f'  valid IPM rows: {len(proc._row_meta)}')

    # ── IPM spot-checks ──
    print('\n=== IPM spot checks (model-image pixels → ground) ===')
    test_pixels = [
        (320, 200, 'center col, row 200 (below horizon)'),
        (320, 280, 'center col, row 280 (near bottom)'),
        (200, 250, 'left of center, row 250'),
        (440, 250, 'right of center, row 250'),
    ]
    for u, v, label in test_pixels:
        pt = proc.pixel_to_ground(u, v)
        if pt:
            print(f'  ({u:3d},{v:3d})  →  X={pt[0]:.3f}m  Y={pt[1]:+.3f}m  [{label}]')
        else:
            print(f'  ({u:3d},{v:3d})  →  above horizon  [{label}]')

    # ── Inverse spot-check ──
    print('\n=== Inverse IPM spot checks (ground → model-image pixels) ===')
    for X, Y, label in [(1.0, 0.0, 'center, 1m ahead'),
                         (1.0, 0.2, '0.2m left at 1m'),
                         (2.0, -0.15, '0.15m right at 2m')]:
        px = proc.ground_to_pixel(X, Y)
        print(f'  X={X:.1f}m  Y={Y:+.2f}m  →  pixel {px}  [{label}]')

    if len(sys.argv) > 1:
        # ── Video-file test ──
        cap = cv2.VideoCapture(sys.argv[1])
        print(f'\n=== Video test: {sys.argv[1]} ===')
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            e_y, e_psi, valid, debug = proc.process(frame)
            print(f'  e_y={e_y:+.4f}m  e_psi={math.degrees(e_psi):+.2f}°  valid={valid}')
            cv2.imshow('LaneProcessor debug', debug)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

    else:
        # ── Synthetic frame test ──
        print('\n=== Synthetic frame test ===')
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # Road surface (gray)
        frame[180:, :] = (60, 60, 60)
        # Left lane marking  — white vertical stripe
        frame[180:, 195:215] = (230, 230, 230)
        # Right lane marking — white vertical stripe
        frame[180:, 425:445] = (230, 230, 230)
        # Add some perspective fade near horizon
        for row in range(180, 230):
            alpha = (row - 180) / 50.0
            frame[row, :] = (frame[row] * alpha).astype(np.uint8)

        e_y, e_psi, valid, debug = proc.process(frame)
        print(f'  e_y   = {e_y:+.6f} m')
        print(f'  e_psi = {math.degrees(e_psi):+.4f} deg')
        print(f'  valid = {valid}')

        out_path = '/tmp/lane_debug.png'
        cv2.imwrite(out_path, debug)
        print(f'\nDebug image saved → {out_path}')
        print('Run:  eog /tmp/lane_debug.png   (or xdg-open /tmp/lane_debug.png)')
