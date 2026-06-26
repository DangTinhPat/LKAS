#!/usr/bin/env python3
"""
estimator.py — Khớp Đa Thức + Kalman Filter trên Hệ Số Làn Đường.

Cải tiến so với phiên bản cũ:
  - Weighted polyfit: điểm từ cửa sổ có độ tin cậy cao được ưu tiên hơn.
  - LaneKalman: Kalman filter 3D trên hệ số đa thức [A, B, C] thay vì
    inertia cache cứng nhắc (giữ hệ số cũ nguyên xi).
    → e_y, e_psi mượt mà theo thời gian, không nhảy frame-to-frame.
    → Trong frame mất dấu làn: Kalman tự dự đoán (predict-only), hội tụ
      lại tự nhiên khi detection phục hồi.

Mô hình đa thức: Y = A·X² + B·X + C  (hệ xe ROS, X = tiến, Y = trái)
  e_y   = A·Ld² + B·Ld + C   (tại điểm nhìn trước Ld)
  e_psi = arctan(B)            (góc tiếp tuyến tại X=0)
"""

import math
from typing import List, Optional, Tuple

import numpy as np

_LOOKAHEAD_M     = 1.0    # m — khoảng cách nhìn trước để tính e_y
_MIN_PTS         = 4      # số điểm tối thiểu (4 > 3 hệ số → vẫn overdetermined)
_MAX_LOSS_FRAMES = 25     # frame mất dấu tối đa trước khi báo invalid (~2.5s @ 10Hz)


# ── Kalman Filter trên hệ số đa thức [A, B, C] ───────────────────────────────

class LaneKalman:
    """
    Bộ lọc Kalman 3D với mô hình trạng thái hằng số (constant model).

    State  x = [A, B, C]^T   — hệ số đa thức bậc 2
    Model  F = I              — đường cong thay đổi liên tục nên mô hình "giữ nguyên"
    Obs    H = I              — đo trực tiếp hệ số từ polyfit

    Phương trình:
      Predict:  x_k|k-1 = x_{k-1}         (hằng số)
                P_k|k-1 = P_{k-1} + Q     (độ bất định tăng mỗi bước)

      Update:   K = P · (P + R)⁻¹
                x = x + K · (z − x)
                P = (I − K) · P

    Q (process noise): tốc độ thay đổi của hệ số theo thời gian (~10 Hz):
      A (độ cong đường): rất ổn định → Q_A nhỏ
      B (góc hướng):     thay đổi ở góc cua → Q_B vừa
      C (lệch ngang):    thay đổi khi xe trôi → Q_C vừa

    R (measurement noise): tỉ lệ nghịch với chất lượng detection
      r_scale = 1 (detection tốt) → R nhỏ → tin measurement nhiều
      r_scale lớn (ít điểm / fit xấu) → R lớn → tin prediction nhiều
    """

    # Noise calibration: A (curvature), B (heading), C (lateral offset)
    _Q = np.diag([5e-7, 2e-3, 1e-3])          # Q_C tăng 4e-4→1e-3: thích nghi nhanh hơn
    _R_base = np.diag([2e-5, 8e-4, 2e-4])     # measurement noise (perfect detection)

    def __init__(self):
        self._x: Optional[np.ndarray] = None   # state [A, B, C]
        self._P: Optional[np.ndarray] = None   # covariance (3×3)

    @property
    def ready(self) -> bool:
        return self._x is not None

    def predict(self):
        """Bước predict: độ bất định P tăng thêm Q mỗi frame."""
        if self.ready:
            self._P = self._P + self._Q

    def update(self, z: np.ndarray, r_scale: float = 1.0):
        """
        Bước update với đo lường z = [A_meas, B_meas, C_meas].

        r_scale: hệ số nhân cho R — lớn = ít tin measurement, nhiều tin prediction.
        """
        R = self._R_base * max(0.5, r_scale)
        if not self.ready:
            self._x = z.copy()
            self._P = R.copy()
            return
        S = self._P + R
        K = self._P @ np.linalg.inv(S)     # Kalman gain (H=I)
        self._x = self._x + K @ (z - self._x)
        self._P = (np.eye(3) - K) @ self._P

    @property
    def state(self) -> Optional[np.ndarray]:
        return self._x.copy() if self.ready else None

    def reset(self):
        self._x = None
        self._P = None


# ── Bộ Ước Lượng Chính ───────────────────────────────────────────────────────

class LaneEstimator:
    """
    Ước lượng vị trí làn đường:
      1. Weighted polyfit bậc 2 từ điểm tâm làn (có trọng số confidence).
      2. Kalman filter làm mịn hệ số [A, B, C] theo thời gian.
      3. Tính e_y, e_psi từ hệ số Kalman đã lọc.
    """

    def __init__(
        self,
        lookahead_m:     float = _LOOKAHEAD_M,
        min_pts:         int   = _MIN_PTS,
        max_loss_frames: int   = _MAX_LOSS_FRAMES,
    ):
        self._ld              = lookahead_m
        self._min_pts         = min_pts
        self._max_loss_frames = max_loss_frames
        self._kf              = LaneKalman()
        self._loss_frames: int = 0

    # ── Weighted poly-fit với sigma clipping ─────────────────────────────────
    def _polyfit(
        self,
        pts:     List[Tuple[float, float]],
        weights: Optional[List[float]] = None,
    ) -> Tuple[Optional[np.ndarray], float, int]:
        """
        Khớp đa thức bậc 2 có trọng số.

        Returns: (coeffs, residual_sigma, n_inliers)
          residual_sigma: độ lệch chuẩn của residual sau sigma-clipping (mét)
          n_inliers: số điểm sau khi loại outlier
        """
        arr = np.array(pts, dtype=np.float64)
        Xv  = arr[:, 0]
        Yv  = arr[:, 1]
        w   = (np.ones(len(pts), dtype=np.float64)
               if weights is None
               else np.asarray(weights, dtype=np.float64))
        w   = np.clip(w, 1e-6, None)    # tránh weight = 0

        try:
            coeffs = np.polyfit(Xv, Yv, deg=2, w=w)
        except (np.linalg.LinAlgError, ValueError):
            return None, 1.0, 0

        # Sigma clipping: loại outlier > 2σ (dùng residual không trọng số)
        Y_pred   = np.polyval(coeffs, Xv)
        residual = np.abs(Yv - Y_pred)
        sigma    = float(residual.std())

        if sigma > 1e-6:
            inliers = residual < 2.0 * sigma
            n_in    = int(inliers.sum())
            if n_in >= self._min_pts:
                try:
                    coeffs = np.polyfit(Xv[inliers], Yv[inliers], deg=2, w=w[inliers])
                    Y2     = np.polyval(coeffs, Xv[inliers])
                    sigma  = float(np.abs(Yv[inliers] - Y2).std())
                except (np.linalg.LinAlgError, ValueError):
                    pass
                return coeffs, sigma, n_in

        return coeffs, sigma, len(pts)

    # ── Tính sai số điều khiển từ hệ số đa thức ──────────────────────────────
    def _compute_errors(self, coeffs: np.ndarray) -> Tuple[float, float]:
        A, B, C = float(coeffs[0]), float(coeffs[1]), float(coeffs[2])
        e_y   = A * self._ld**2 + B * self._ld + C
        e_psi = math.atan(B)
        return e_y, e_psi

    # ── API Công Khai ─────────────────────────────────────────────────────────
    def estimate(
        self,
        pts_meters: List[Tuple[float, float]],
        weights:    Optional[List[float]] = None,
    ) -> Tuple[float, float, bool]:
        """
        Ước lượng (e_y, e_psi) với Kalman-smoothed polynomial coefficients.

        Luồng xử lý mỗi frame:
          1. kf.predict()  — luôn gọi, tăng P thêm Q
          2. Nếu đủ điểm:
               - weighted_polyfit → coeffs_meas, sigma, n_inliers
               - tính r_scale từ chất lượng fit
               - kf.update(coeffs_meas, r_scale)
               - reset loss_frames
          3. Lấy hệ số smooth từ Kalman state
          4. Tính và trả về (e_y, e_psi, valid)
        """
        # Bước 1: Predict mỗi frame (luôn luôn)
        self._kf.predict()

        # Bước 2: Cố gắng fit từ frame hiện tại
        n_pts = len(pts_meters)
        if n_pts >= self._min_pts:
            coeffs_meas, sigma, n_inliers = self._polyfit(pts_meters, weights)
            if coeffs_meas is not None:
                # r_scale tỉ lệ nghịch chất lượng fit:
                #   fresh_ratio: tỉ lệ điểm còn lại sau sigma clip
                #   sigma_penalty: mức độ scatter của fit
                fresh_ratio   = n_inliers / max(1, n_pts)
                sigma_penalty = max(1.0, sigma / 0.008)  # chuẩn = 8mm
                r_scale       = sigma_penalty / max(0.1, fresh_ratio)
                self._kf.update(coeffs_meas, r_scale)
                self._loss_frames = 0

        else:
            self._loss_frames += 1

        # Bước 3: Lấy hệ số từ Kalman (smooth)
        smooth = self._kf.state
        # Sanity check: nếu C (lệch ngang) vô lý (>0.8m), reset Kalman
        # Tránh "lock-on" vào trạng thái sai khi detection ban đầu bị lỗi
        if smooth is not None and abs(smooth[2]) > 0.8:
            self._kf.reset()
            self._loss_frames = 0
            smooth = None
        if smooth is not None and self._loss_frames <= self._max_loss_frames:
            e_y, e_psi = self._compute_errors(smooth)
            return e_y, e_psi, True

        # Kalman chưa sẵn sàng hoặc mất dấu quá lâu
        return 0.0, 0.0, False

    def reset(self):
        self._kf.reset()
        self._loss_frames = 0

    @property
    def cached_coeffs(self) -> Optional[np.ndarray]:
        """Hệ số đa thức hiện tại (đã qua Kalman), dùng để vẽ debug."""
        return self._kf.state

    @property
    def loss_frames(self) -> int:
        return self._loss_frames

    @property
    def has_cache(self) -> bool:
        return self._kf.ready
