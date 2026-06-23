#!/usr/bin/env python3
"""
processor.py — Bộ Điều Phối Pipeline Thị Giác Chính (Vision Pipeline Orchestrator).

Khởi tạo và kết nối ba khối chức năng:
  AIDetector          → phát hiện vạch làn (AI Inference)
  GeometryTransformer → biến đổi hình học pixel ↔ mét
  LaneEstimator       → khớp đa thức và tính sai số điều khiển

Giao diện công khai duy nhất: process_frame(cv_image) → (e_y, e_psi, mask, debug_img)

KHÔNG phụ thuộc bất kỳ thư viện ROS 2 nào — có thể test offline bằng video.

Thuật toán phát hiện tâm làn: Sliding Window
─────────────────────────────────────────────
Thay vì quét từng hàng độc lập (dễ mất lane trên đường cong), dùng cửa sổ trượt:
  1. Histogram ngang trên 40% phần dưới → tìm vị trí khởi đầu trái/phải
  2. Trượt N cửa sổ từ dưới lên trên, mỗi cửa sổ tự căn chỉnh theo pixel tìm được
  3. Tính tâm từ cặp trái-phải; fallback offset khi chỉ thấy 1 vạch
"""

import math
from typing import List, Optional, Tuple

import numpy as np
import cv2

try:
    from .ai_detector  import AIDetector
    from .transformer  import GeometryTransformer
    from .estimator    import LaneEstimator
except ImportError:
    from ai_detector  import AIDetector
    from transformer  import GeometryTransformer
    from estimator    import LaneEstimator

# ── Tham số Sliding Window ────────────────────────────────────────────────────
_N_WINDOWS          = 10     # số cửa sổ trượt theo chiều cao
_WINDOW_MARGIN      = 50     # px — nửa chiều rộng mỗi cửa sổ
_MIN_WIN_PX         = 4      # pixel thực tối thiểu để dời tâm cửa sổ
_HIST_BOTTOM_FRAC   = 0.40   # phần dưới ảnh dùng để tính histogram khởi tạo
_MIN_HIST_PEAK      = 5      # số pixel tối thiểu trong cột histogram (mask 0/1)

# ── Tham số fallback 1 vạch ───────────────────────────────────────────────────
_ASSUMED_HALF_WIDTH_M = 0.22  # m — nửa chiều rộng làn giả định

# ── Tham số vẽ debug ──────────────────────────────────────────────────────────
_DEBUG_LINE_COLOR   = (0, 220, 0)
_DEBUG_DOT_RADIUS   = 4
_DEBUG_DOT_STEP     = 0.12
_DEBUG_MASK_COLOR   = (255, 100, 0)
_DEBUG_MASK_ALPHA   = 0.40
_DEBUG_WIN_COLOR    = (0, 180, 255)   # màu vẽ cửa sổ trượt


class LaneProcessor:
    """
    Bộ điều phối chính: nhận ảnh BGR thô → trả về sai số điều khiển + ảnh debug.
    """

    def __init__(
        self,
        model_path:   str,
        cam_height:   float = 0.134,
        cam_pitch:    float = 0.0,
        cam_x_offset: float = 0.1485,
    ):
        self._detector    = AIDetector(model_path)
        self._transformer = GeometryTransformer(
            cam_height=cam_height,
            cam_pitch=cam_pitch,
            cam_x_offset=cam_x_offset,
        )
        self._estimator = LaneEstimator()

        # Cache vị trí cửa sổ từ frame trước để khởi tạo nhanh hơn
        self._prev_left_x:  Optional[int] = None
        self._prev_right_x: Optional[int] = None

    # ──────────────────────────────────────────────────────────────────────────
    # BƯỚC TRUNG GIAN: Sliding Window → điểm tâm làn (mét)
    # ──────────────────────────────────────────────────────────────────────────
    def _extract_center_pts(
        self, mask: np.ndarray
    ) -> Tuple[List[Tuple[float, float]], list, list]:
        """
        Sliding window: trả về (pts_meters, left_windows, right_windows).
        left/right_windows là list [(x, y_lo, y_hi)] để vẽ debug.
        """
        H, W = mask.shape
        mid   = W // 2

        # ── 1. Histogram để tìm điểm khởi đầu ────────────────────────────
        hist_top  = int(H * (1.0 - _HIST_BOTTOM_FRAC))
        histogram = np.sum(mask[hist_top:, :], axis=0).astype(np.float32)

        # Làm mịn histogram để giảm nhiễu
        k = max(3, W // 32) | 1   # đảm bảo lẻ
        histogram = cv2.GaussianBlur(
            histogram.reshape(1, -1), (k, 1), 0
        ).flatten()

        # Ưu tiên vị trí từ frame trước (tracking liên tục)
        if self._prev_left_x is not None:
            search_range = _WINDOW_MARGIN * 2
            l0 = max(0,   self._prev_left_x  - search_range)
            l1 = min(mid, self._prev_left_x  + search_range)
            r0 = max(mid, self._prev_right_x - search_range)
            r1 = min(W,   self._prev_right_x + search_range)
            left_base  = l0 + int(np.argmax(histogram[l0:l1]))
            right_base = r0 + int(np.argmax(histogram[r0:r1]))
        else:
            left_base  = int(np.argmax(histogram[:mid]))
            right_base = int(np.argmax(histogram[mid:])) + mid

        has_left  = histogram[left_base]  >= _MIN_HIST_PEAK
        has_right = histogram[right_base] >= _MIN_HIST_PEAK

        if not has_left and not has_right:
            self._prev_left_x  = None
            self._prev_right_x = None
            return [], [], []

        # ── 2. Sliding windows từ dưới lên ────────────────────────────────
        road_top = H // 2          # chỉ xét nửa dưới ảnh (phần đường)
        road_h   = H - road_top
        win_h    = road_h // _N_WINDOWS

        lx = left_base
        rx = right_base

        pts_meters: List[Tuple[float, float]] = []
        left_wins:  list = []
        right_wins: list = []

        for i in range(_N_WINDOWS - 1, -1, -1):    # từ dưới (i=N-1) lên trên (i=0)
            y_lo = road_top + i * win_h
            y_hi = min(y_lo + win_h, H)
            v_c  = (y_lo + y_hi) // 2

            left_fresh  = False
            right_fresh = False

            if has_left:
                xl0   = max(0, lx - _WINDOW_MARGIN)
                xl1   = min(W, lx + _WINDOW_MARGIN)
                strip = mask[y_lo:y_hi, xl0:xl1]
                _, pix_cols = np.where(strip > 0)
                if len(pix_cols) >= _MIN_WIN_PX:
                    lx = int(pix_cols.mean()) + xl0
                    left_fresh = True
                left_wins.append((lx, y_lo, y_hi))

            if has_right:
                xr0   = max(0, rx - _WINDOW_MARGIN)
                xr1   = min(W, rx + _WINDOW_MARGIN)
                strip = mask[y_lo:y_hi, xr0:xr1]
                _, pix_cols = np.where(strip > 0)
                if len(pix_cols) >= _MIN_WIN_PX:
                    rx = int(pix_cols.mean()) + xr0
                    right_fresh = True
                right_wins.append((rx, y_lo, y_hi))

            # Bug cong phải: khi 1 vạch fresh + 1 vạch stale (đứng yên), midpoint bị lệch.
            # Giải pháp: nếu CẢ HAI stale → dùng midpoint stale (an toàn, cùng drift).
            # Chỉ bỏ midpoint khi MỘT fresh MỘT stale → dùng single-lane offset.
            if left_fresh and right_fresh:
                pt = self._transformer.pixel_to_vehicle((lx + rx) / 2.0, float(v_c))
                if pt is not None:
                    pts_meters.append(pt)
            elif right_fresh:
                # Chỉ right fresh: left stale có thể đứng yên sai → dùng right + offset
                scale = self._transformer.lateral_scale_at(v_c)
                if scale:
                    pt = self._transformer.pixel_to_vehicle(
                        rx - _ASSUMED_HALF_WIDTH_M / scale, float(v_c))
                    if pt is not None:
                        pts_meters.append(pt)
            elif left_fresh:
                # Chỉ left fresh: right stale có thể đứng yên sai → dùng left + offset
                scale = self._transformer.lateral_scale_at(v_c)
                if scale:
                    pt = self._transformer.pixel_to_vehicle(
                        lx + _ASSUMED_HALF_WIDTH_M / scale, float(v_c))
                    if pt is not None:
                        pts_meters.append(pt)
            else:
                # Cả hai stale (vạch đứt đoạn, tạm thời mất pixel):
                # Dùng midpoint stale — cả hai cùng frozen nên tỷ lệ tương đối vẫn đúng.
                # Đảm bảo đủ điểm cho polynomial fit thay vì bỏ qua hoàn toàn.
                if has_left and has_right:
                    pt = self._transformer.pixel_to_vehicle((lx + rx) / 2.0, float(v_c))
                    if pt is not None:
                        pts_meters.append(pt)
                elif has_right:
                    scale = self._transformer.lateral_scale_at(v_c)
                    if scale:
                        pt = self._transformer.pixel_to_vehicle(
                            rx - _ASSUMED_HALF_WIDTH_M / scale, float(v_c))
                        if pt is not None:
                            pts_meters.append(pt)
                elif has_left:
                    scale = self._transformer.lateral_scale_at(v_c)
                    if scale:
                        pt = self._transformer.pixel_to_vehicle(
                            lx + _ASSUMED_HALF_WIDTH_M / scale, float(v_c))
                        if pt is not None:
                            pts_meters.append(pt)

        # Cập nhật cache vị trí
        self._prev_left_x  = lx if has_left  else self._prev_left_x
        self._prev_right_x = rx if has_right else self._prev_right_x

        return pts_meters, left_wins, right_wins

    # ──────────────────────────────────────────────────────────────────────────
    # Vẽ debug
    # ──────────────────────────────────────────────────────────────────────────
    def _draw_debug(
        self,
        frame_bgr:   np.ndarray,
        mask:        np.ndarray,
        e_y:         float,
        e_psi:       float,
        valid:       bool,
        coeffs:      Optional[np.ndarray],
        left_wins:   list,
        right_wins:  list,
    ) -> np.ndarray:
        vis = frame_bgr.copy()

        # Overlay mask
        overlay = vis.copy()
        overlay[mask > 0] = _DEBUG_MASK_COLOR
        vis = cv2.addWeighted(vis, 1.0 - _DEBUG_MASK_ALPHA,
                              overlay, _DEBUG_MASK_ALPHA, 0)

        # Sliding windows
        for (x, y_lo, y_hi) in left_wins:
            cv2.rectangle(vis,
                          (x - _WINDOW_MARGIN, y_lo),
                          (x + _WINDOW_MARGIN, y_hi),
                          _DEBUG_WIN_COLOR, 1)
        for (x, y_lo, y_hi) in right_wins:
            cv2.rectangle(vis,
                          (x - _WINDOW_MARGIN, y_lo),
                          (x + _WINDOW_MARGIN, y_hi),
                          (255, 180, 0), 1)

        # Đường tâm làn dự báo
        if valid and coeffs is not None:
            A, B, C = float(coeffs[0]), float(coeffs[1]), float(coeffs[2])
            x_vals = np.arange(0.05, 3.5, _DEBUG_DOT_STEP)
            for x_m in x_vals:
                y_m = A * x_m**2 + B * x_m + C
                px  = self._transformer.vehicle_to_pixel(x_m, y_m)
                if px is not None:
                    cv2.circle(vis, px, _DEBUG_DOT_RADIUS, _DEBUG_LINE_COLOR, -1)

        # Chấm đỏ tâm xe
        H, W = vis.shape[:2]
        vc = (W // 2, H - 15)
        cv2.circle(vis, vc, 8, (0, 0, 255), -1)
        cv2.line(vis, (vc[0], vc[1] - 14), (vc[0], vc[1] + 14), (0, 0, 255), 2)

        # Text trạng thái
        status_txt   = 'TRACKING' if valid else 'NO LANE'
        status_color = (0, 220, 0) if valid else (0, 50, 255)
        cv2.putText(vis, status_txt,
                    (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)
        cv2.putText(vis, f'e_y  = {e_y:+.4f} m',
                    (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
        cv2.putText(vis, f'e_psi= {math.degrees(e_psi):+.2f} deg',
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
        if not valid and self._estimator.has_cache:
            cv2.putText(vis, f'CACHE [{self._estimator.loss_frames}]',
                        (10, 118), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 200, 255), 1)

        return vis

    # ──────────────────────────────────────────────────────────────────────────
    # API CÔNG KHAI
    # ──────────────────────────────────────────────────────────────────────────
    def process_frame(
        self, cv_image: np.ndarray
    ) -> Tuple[float, float, np.ndarray, np.ndarray]:
        try:
            mask = self._detector.detect(cv_image)
            pts_meters, left_wins, right_wins = self._extract_center_pts(mask)
            e_y, e_psi, valid = self._estimator.estimate(pts_meters)
            coeffs    = self._estimator._cached_coeffs if self._estimator.has_cache else None
            debug_img = self._draw_debug(
                cv_image, mask, e_y, e_psi, valid, coeffs, left_wins, right_wins
            )
            return e_y, e_psi, mask, debug_img

        except Exception as exc:
            debug_img = cv_image.copy()
            cv2.putText(debug_img, f'ERR: {exc}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            return 0.0, 0.0, np.zeros(cv_image.shape[:2], dtype=np.uint8), debug_img


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import sys, os

    if len(sys.argv) < 2:
        print('Cách dùng: python3 processor.py <video_file>')
        sys.exit(1)

    video_path = sys.argv[1]
    model_path = os.path.join(
        os.path.dirname(__file__), '..', '..', 'models', 'EgoLanes_Lite_FP32.onnx'
    )

    proc = LaneProcessor(model_path)
    cap  = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f'Không mở được video: {video_path}')
        sys.exit(1)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        e_y, e_psi, _, debug = proc.process_frame(frame)
        print(f'e_y={e_y:+.4f}m  e_psi={math.degrees(e_psi):+.2f}deg')
        cv2.imshow('Lane Debug', debug)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
