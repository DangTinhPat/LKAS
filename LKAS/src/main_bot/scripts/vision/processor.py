#!/usr/bin/env python3
"""
processor.py — vision pipeline V2.

Layer 0: Raw ONNX logits — no sigmoid, no upsample, keeps full confidence signal.

Layer 1: Denoise + weighted centroid in model space (80x160).
  Morphology OPEN per channel, weight = relu(logit) for sub-pixel accuracy,
  ROI = bottom 45% of rows.

Layer 2: RANSAC quadratic fit in model space, rejecting outliers from
  dashed/partial lane markings.

Layer 2.5: Scale to world meters via GeometryTransformer (only here — the
  mask itself is never scaled).
  e_y   = Y_world at the eval row (lookahead)
  e_psi = atan2(delta_Y, delta_X) between a near and far row

Layer 3: Adaptive EMA — alpha scales with detection confidence (0 when
  confidence is low, so it predicts-only instead of chasing noise); reports
  invalid after _MAX_LOSS_FRAMES consecutive low-confidence frames.
"""

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np

try:
    from .ai_detector import AIDetector
    from .transformer import GeometryTransformer
except ImportError:
    from ai_detector import AIDetector
    from transformer import GeometryTransformer

# ── ROI ───────────────────────────────────────────────────────────────────────
_ROI_FRAC        = 0.45    # bottom fraction of model rows used (depth ~0.14-1.03m)

# ── Centroid per row ──────────────────────────────────────────────────────────
_MIN_ROW_SIGNAL  = 0.3     # minimum summed relu-weight for a row to count
_GOOD_ROW_SIGNAL = 5.0     # summed relu-weight considered full quality (1.0)

# ── RANSAC ────────────────────────────────────────────────────────────────────
_RANSAC_ITER     = 40
_RANSAC_THRESH   = 3.0     # inlier threshold, model pixels
_RANSAC_MIN_IN   = 4       # minimum inliers to accept a fit

# ── Adaptive EMA ──────────────────────────────────────────────────────────────
_EMA_CONF_LOW    = 0.30    # below this: alpha=0 (predict-only)
_EMA_CONF_HIGH   = 0.70    # above this: alpha = _EMA_ALPHA_MAX
_EMA_ALPHA_MAX   = 0.50
_MAX_LOSS_FRAMES = 25      # ~2.5s @ 10Hz before reporting invalid

# ── Row fractions for world-space evaluation ────────────────────────────────
_Y_EVAL_FRAC     = 0.60    # e_y lookahead row
_Y_PSI_NEAR_FRAC = 0.60    # e_psi near reference row
_Y_PSI_FAR_FRAC  = 0.55    # e_psi far reference row

_MORPH_KERNEL    = (2, 2)

# ── Lane geometry (race_way.world: road_top width=1.068m) ──────────────────
_HALF_LANE_W_M   = 0.267   # m, half lane width — used when only one marking is visible

# ── Debug drawing ─────────────────────────────────────────────────────────────
_DBG_MASK_ALPHA  = 0.30
_DBG_MASK_CLR    = (200, 80, 0)
_DBG_INLIER_CLR  = (0, 220, 0)
_DBG_OUTLIER_CLR = (100, 100, 255)
_DBG_POLY_CLR    = (0, 220, 0)
_DBG_DOT_R       = 3


class LaneProcessor:
    """Frame BGR -> (e_y, e_psi, kappa, valid, mask_display, debug_img)."""

    def __init__(
        self,
        model_path:   str,
        cam_height:   float = 0.134,
        cam_pitch:    float = 0.0,
        cam_x_offset: float = 0.1485,
    ):
        self._det = AIDetector(model_path)
        self._tf  = GeometryTransformer(
            cam_height=cam_height,
            cam_pitch=cam_pitch,
            cam_x_offset=cam_x_offset,
        )

        self._e_y_ema:    float = 0.0
        self._e_psi_ema:  float = 0.0
        self._kappa_ema:  float = 0.0   # road curvature [1/m], positive = left turn
        self._has_init:   bool  = False
        self._loss_frames: int  = 0

    def _half_lane_px(self, model_row: int, sc_row: float, W_m: int, orig_w: int) -> float:
        """Half lane width (0.267m) in model-space pixels at `model_row`,
        used to offset the centroid when only one lane marking is visible."""
        v_orig = min(int(model_row * sc_row), self._tf.img_h - 1)
        scale  = self._tf.lateral_scale_at(v_orig)
        if scale is None or scale < 1e-9:
            dv = (v_orig - self._tf.cy) / self._tf.fy
            if abs(dv) < 1e-9:
                return 0.0
            depth = self._tf.cam_height / (math.cos(self._tf.cam_pitch) * dv)
            scale = depth / self._tf.fx
        return (_HALF_LANE_W_M / scale) * (W_m / orig_w)

    def _extract_centroids(
        self,
        frame_bgr: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int, int, int, int]:
        """
        Returns:
          pts_row      : [N] model row index of each centroid
          pts_cx       : [N] lane-center column, sub-pixel, model space
          pts_quality  : [N] confidence in [0,1]
          combined_relu: [H_m, W_m] max(relu0, relu1), for debug display
          orig_h, orig_w, H_m, W_m : image/model dimensions

        Per-row lane-center strategy:
          - both channels have signal -> midpoint(cx0, cx1)
          - only left marking visible -> cx0 + half_lane_px
          - only right marking visible -> cx1 - half_lane_px
        Single-marking rows get half quality weight since the estimate is
        geometric rather than measured.
        """
        orig_h, orig_w = frame_bgr.shape[:2]
        sc_col = orig_w / 160.0
        sc_row = orig_h / 80.0

        logits = self._det.detect_raw(frame_bgr)   # [3, H_m, W_m]
        H_m, W_m = logits.shape[1], logits.shape[2]
        sc_col = orig_w / W_m
        sc_row = orig_h / H_m

        relu0 = np.maximum(logits[0], 0.0)   # left marking channel
        relu1 = np.maximum(logits[1], 0.0)   # right marking channel

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, _MORPH_KERNEL)

        binary0 = (logits[0] > 0).astype(np.uint8) * 255
        binary0 = cv2.morphologyEx(binary0, cv2.MORPH_OPEN, kernel)
        relu0   = relu0 * (binary0 > 0).astype(np.float32)

        binary1 = (logits[1] > 0).astype(np.uint8) * 255
        binary1 = cv2.morphologyEx(binary1, cv2.MORPH_OPEN, kernel)
        relu1   = relu1 * (binary1 > 0).astype(np.float32)

        combined_relu = np.maximum(relu0, relu1)

        roi_top = int((1.0 - _ROI_FRAC) * H_m)
        roi0    = relu0[roi_top:]
        roi1    = relu1[roi_top:]
        roi_h   = roi0.shape[0]

        col_idx = np.arange(W_m, dtype=np.float32)

        pts_row: List[float] = []
        pts_cx:  List[float] = []
        pts_q:   List[float] = []

        for r in range(roi_h):
            row0   = roi0[r]
            row1   = roi1[r]
            total0 = float(row0.sum())
            total1 = float(row1.sum())
            has0   = total0 >= _MIN_ROW_SIGNAL
            has1   = total1 >= _MIN_ROW_SIGNAL

            if not has0 and not has1:
                continue

            model_row = roi_top + r

            if has0 and has1:
                cx0 = float((row0 * col_idx).sum() / total0)
                cx1 = float((row1 * col_idx).sum() / total1)
                cx  = (cx0 + cx1) * 0.5
                q   = min(total0, total1) / _GOOD_ROW_SIGNAL
            elif has0:
                cx0       = float((row0 * col_idx).sum() / total0)
                half_px   = self._half_lane_px(model_row, sc_row, W_m, orig_w)
                cx        = cx0 + half_px
                q         = total0 / _GOOD_ROW_SIGNAL * 0.5
            else:
                cx1       = float((row1 * col_idx).sum() / total1)
                half_px   = self._half_lane_px(model_row, sc_row, W_m, orig_w)
                cx        = cx1 - half_px
                q         = total1 / _GOOD_ROW_SIGNAL * 0.5

            pts_row.append(float(model_row))
            pts_cx.append(cx)
            pts_q.append(min(1.0, q))

        return (
            np.array(pts_row, dtype=np.float32),
            np.array(pts_cx,  dtype=np.float32),
            np.array(pts_q,   dtype=np.float32),
            combined_relu,
            orig_h, orig_w,
            H_m, W_m,
        )

    @staticmethod
    def _ransac_polyfit(
        rows:    np.ndarray,
        cols:    np.ndarray,
        weights: np.ndarray,
    ) -> Tuple[Optional[np.ndarray], np.ndarray]:
        """Fit cx = A*row^2 + B*row + C via RANSAC in model space.
        Returns (coeffs, inlier_bool_mask) or (None, empty_mask)."""
        n = len(rows)
        if n < _RANSAC_MIN_IN:
            return None, np.zeros(n, dtype=bool)

        rng           = np.random.default_rng(seed=42)
        best_coeffs:  Optional[np.ndarray] = None
        best_inliers: np.ndarray           = np.zeros(n, dtype=bool)
        best_n_in:    int                  = 0

        for _ in range(_RANSAC_ITER):
            idx = rng.choice(n, 3, replace=False)
            try:
                c = np.polyfit(rows[idx], cols[idx], 2)
            except (np.linalg.LinAlgError, ValueError):
                continue

            res     = np.abs(cols - np.polyval(c, rows))
            inliers = res < _RANSAC_THRESH
            n_in    = int(inliers.sum())

            if n_in > best_n_in:
                best_n_in    = n_in
                best_inliers = inliers
                best_coeffs  = c

        if best_coeffs is None or best_n_in < _RANSAC_MIN_IN:
            return None, np.zeros(n, dtype=bool)

        try:
            coeffs = np.polyfit(
                rows[best_inliers], cols[best_inliers], 2,
                w=weights[best_inliers],
            )
        except (np.linalg.LinAlgError, ValueError):
            coeffs = best_coeffs

        return coeffs, best_inliers

    def _poly_to_errors(
        self,
        coeffs: np.ndarray,
        H_m: int, W_m: int,
        orig_h: int, orig_w: int,
    ) -> Tuple[Optional[float], Optional[float], float]:
        """
        Model-space polynomial (col = f(row)) -> (e_y, e_psi, kappa) in world
        meters, via transformer.pixel_to_vehicle at the eval/near/far rows.

          kappa = 2*(Y_f-Y_n)/(X_f^2-X_n^2) — curvature at the robot origin;
          the e_y offset term cancels out of this formula.
        """
        sc_col = orig_w / W_m
        sc_row = orig_h / H_m

        row_eval = _Y_EVAL_FRAC * H_m
        cx_eval  = float(np.polyval(coeffs, row_eval))
        pt_eval  = self._tf.pixel_to_vehicle(cx_eval * sc_col, row_eval * sc_row)
        if pt_eval is None:
            return None, None, 0.0
        _, e_y = pt_eval

        if abs(e_y) > 1.5:
            return None, None, 0.0

        row_near = _Y_PSI_NEAR_FRAC * H_m
        row_far  = _Y_PSI_FAR_FRAC  * H_m

        cx_near = float(np.polyval(coeffs, row_near))
        cx_far  = float(np.polyval(coeffs, row_far))

        pt_near = self._tf.pixel_to_vehicle(cx_near * sc_col, row_near * sc_row)
        pt_far  = self._tf.pixel_to_vehicle(cx_far  * sc_col, row_far  * sc_row)

        e_psi = 0.0
        kappa = 0.0

        if pt_near is not None and pt_far is not None:
            X_n, Y_n = pt_near
            X_f, Y_f = pt_far
            dX = X_f - X_n
            dY = Y_f - Y_n
            e_psi = math.atan2(dY, dX) if abs(dX) > 1e-6 else 0.0

            denom = X_f * X_f - X_n * X_n
            if abs(denom) > 1e-6:
                kappa = float(np.clip(2.0 * dY / denom, -2.0, 2.0))
        elif pt_near is not None:
            e_psi = 0.0

        return e_y, e_psi, kappa

    def _draw_debug(
        self,
        frame_bgr:     np.ndarray,
        combined_relu: np.ndarray,
        pts_row:       np.ndarray,
        pts_cx:        np.ndarray,
        inliers:       np.ndarray,
        coeffs:        Optional[np.ndarray],
        e_y:           float,
        e_psi:         float,
        kappa:         float,
        valid:         bool,
        H_m: int, W_m: int,
    ) -> np.ndarray:
        orig_h, orig_w = frame_bgr.shape[:2]
        sc_col = orig_w / W_m
        sc_row = orig_h / H_m

        vis = frame_bgr.copy()

        relu_max = combined_relu.max()
        if relu_max > 1e-6:
            mask_u8 = (combined_relu * (255.0 / relu_max)).astype(np.uint8)
        else:
            mask_u8 = np.zeros_like(combined_relu, dtype=np.uint8)
        mask_u8 = cv2.resize(mask_u8, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)
        overlay = vis.copy()
        overlay[mask_u8 > 20] = _DBG_MASK_CLR
        vis = cv2.addWeighted(vis, 1 - _DBG_MASK_ALPHA, overlay, _DBG_MASK_ALPHA, 0)

        for i in range(len(pts_row)):
            u = int(pts_cx[i] * sc_col)
            v = int(pts_row[i] * sc_row)
            is_in = bool(len(inliers) > i and inliers[i])
            cv2.circle(vis, (u, v), _DBG_DOT_R,
                       _DBG_INLIER_CLR if is_in else _DBG_OUTLIER_CLR, -1)

        if coeffs is not None:
            roi_top = int((1.0 - _ROI_FRAC) * H_m)
            prev = None
            for r in range(roi_top, H_m):
                cx_m = float(np.polyval(coeffs, r))
                p = (int(cx_m * sc_col), int(r * sc_row))
                if 0 <= p[0] < orig_w and 0 <= p[1] < orig_h:
                    if prev is not None:
                        cv2.line(vis, prev, p, _DBG_POLY_CLR, 2)
                    prev = p

        cx_veh = orig_w // 2
        cv2.circle(vis, (cx_veh, orig_h - 12), 7, (0, 0, 255), -1)
        cv2.line(vis,   (cx_veh, orig_h - 20), (cx_veh, orig_h - 4), (0, 0, 255), 2)

        st_clr = (0, 220, 0) if valid else (0, 50, 255)
        cv2.putText(vis, 'TRACKING' if valid else 'LOST',
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.70, st_clr, 2)
        cv2.putText(vis, f'e_y  = {e_y:+.4f} m',
                    (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(vis, f'e_psi= {math.degrees(e_psi):+.2f} deg',
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        r_str = f'{1.0/kappa:.1f}m' if abs(kappa) > 0.05 else 'straight'
        cv2.putText(vis, f'kappa= {kappa:+.3f} (R={r_str})',
                    (10, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (180, 220, 255), 2)
        cv2.putText(vis, f'pts={len(pts_row)}  loss={self._loss_frames}',
                    (10, 126), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 0), 1)
        return vis

    def process_frame(
        self,
        cv_image: np.ndarray,
    ) -> Tuple[float, float, float, bool, np.ndarray, np.ndarray]:
        """
        Frame BGR -> (e_y, e_psi, kappa, valid, mask_display, debug_img).

          e_y   [m]     : cross-track error (positive = lane center is left of robot)
          e_psi [rad]   : heading error (positive = lane curves left ahead)
          kappa [1/m]   : road curvature at the robot (positive = left turn)
          valid         : True if the EMA estimate is currently trustworthy
        """
        try:
            (pts_row, pts_cx, pts_q,
             combined_relu,
             orig_h, orig_w, H_m, W_m) = self._extract_centroids(cv_image)

            coeffs, inliers = self._ransac_polyfit(pts_row, pts_cx, pts_q)

            e_y_raw   = 0.0
            e_psi_raw = 0.0
            kappa_raw = 0.0
            detected  = False

            if coeffs is not None:
                result = self._poly_to_errors(coeffs, H_m, W_m, orig_h, orig_w)
                if result[0] is not None:
                    e_y_raw, e_psi_raw, kappa_raw = result
                    detected = True

            n_roi_rows = max(1, int(_ROI_FRAC * H_m))
            confidence = len(pts_row) / n_roi_rows

            if detected and confidence > _EMA_CONF_LOW:
                t     = min(1.0, (confidence - _EMA_CONF_LOW) /
                                 (_EMA_CONF_HIGH - _EMA_CONF_LOW))
                alpha = t * _EMA_ALPHA_MAX

                if not self._has_init:
                    self._e_y_ema   = e_y_raw
                    self._e_psi_ema = e_psi_raw
                    self._kappa_ema = kappa_raw
                    self._has_init  = True
                else:
                    self._e_y_ema   = alpha * e_y_raw   + (1 - alpha) * self._e_y_ema
                    self._e_psi_ema = alpha * e_psi_raw + (1 - alpha) * self._e_psi_ema
                    self._kappa_ema = alpha * kappa_raw + (1 - alpha) * self._kappa_ema

                self._loss_frames = 0
            else:
                self._loss_frames += 1

            valid = self._has_init and self._loss_frames <= _MAX_LOSS_FRAMES

            mask_model   = (combined_relu > 0).astype(np.uint8)
            mask_display = cv2.resize(mask_model, (orig_w, orig_h),
                                      interpolation=cv2.INTER_NEAREST)

            debug_img = self._draw_debug(
                cv_image, combined_relu,
                pts_row, pts_cx, inliers,
                coeffs if detected else None,
                self._e_y_ema, self._e_psi_ema, self._kappa_ema,
                valid, H_m, W_m,
            )
            return self._e_y_ema, self._e_psi_ema, self._kappa_ema, valid, mask_display, debug_img

        except Exception as exc:
            debug_img = cv_image.copy()
            cv2.putText(debug_img, f'ERR: {exc}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            return 0.0, 0.0, 0.0, False, np.zeros(cv_image.shape[:2], np.uint8), debug_img


if __name__ == '__main__':
    import sys, os

    if len(sys.argv) < 2:
        print('Usage: python3 processor.py <video_file>')
        sys.exit(1)

    model_path = os.path.join(
        os.path.dirname(__file__), '..', '..', 'models', 'EgoLanes_Lite_FP32.onnx'
    )
    proc = LaneProcessor(model_path)
    cap  = cv2.VideoCapture(sys.argv[1])

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        e_y, e_psi, valid, _, debug = proc.process_frame(frame)
        print(f'e_y={e_y:+.4f}m  e_psi={math.degrees(e_psi):+.2f}deg  valid={valid}')
        cv2.imshow('Lane V2', debug)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
