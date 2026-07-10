#!/usr/bin/env python3
"""
estimator.py — polynomial lane fit + Kalman filter over the fit coefficients.

Pipeline: weighted 2nd-degree polyfit of lane-center points (higher-confidence
points weighted more) -> LaneKalman smooths the [A, B, C] coefficients across
frames, so e_y/e_psi don't jump frame-to-frame and degrade gracefully
(predict-only) through brief detection loss.

Polynomial model: Y = A*X^2 + B*X + C  (ROS frame, X = forward, Y = left)
  e_y   = A*Ld^2 + B*Ld + C   (at look-ahead distance Ld)
  e_psi = arctan(B)           (tangent angle at X=0)
"""

import math
from typing import List, Optional, Tuple

import numpy as np

_LOOKAHEAD_M     = 1.0    # m — look-ahead distance for e_y
_MIN_PTS         = 4      # minimum points for an overdetermined quadratic fit
_MAX_LOSS_FRAMES = 25     # max frames without detection before reporting invalid (~2.5s @ 10Hz)


class LaneKalman:
    """
    3D Kalman filter with a constant-state model over polynomial coefficients.

    State  x = [A, B, C]^T   — quadratic coefficients
    Model  F = I              — lane geometry treated as slowly varying
    Obs    H = I              — coefficients measured directly from polyfit

    Q (process noise) is tuned per coefficient: A (curvature) changes slowly,
    B (heading) and C (lateral offset) change faster through turns/drift.
    R (measurement noise) scales inversely with fit quality via r_scale.
    """

    _Q = np.diag([5e-7, 2e-3, 1e-3])
    _R_base = np.diag([2e-5, 8e-4, 2e-4])

    def __init__(self):
        self._x: Optional[np.ndarray] = None   # state [A, B, C]
        self._P: Optional[np.ndarray] = None   # covariance (3x3)

    @property
    def ready(self) -> bool:
        return self._x is not None

    def predict(self):
        if self.ready:
            self._P = self._P + self._Q

    def update(self, z: np.ndarray, r_scale: float = 1.0):
        """r_scale > 1 trusts the prediction more than this measurement."""
        R = self._R_base * max(0.5, r_scale)
        if not self.ready:
            self._x = z.copy()
            self._P = R.copy()
            return
        S = self._P + R
        K = self._P @ np.linalg.inv(S)
        self._x = self._x + K @ (z - self._x)
        self._P = (np.eye(3) - K) @ self._P

    @property
    def state(self) -> Optional[np.ndarray]:
        return self._x.copy() if self.ready else None

    def reset(self):
        self._x = None
        self._P = None


class LaneEstimator:
    """Weighted polyfit -> Kalman-smoothed coefficients -> (e_y, e_psi)."""

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

    def _polyfit(
        self,
        pts:     List[Tuple[float, float]],
        weights: Optional[List[float]] = None,
    ) -> Tuple[Optional[np.ndarray], float, int]:
        """Weighted quadratic fit with one round of 2-sigma outlier clipping.
        Returns (coeffs, residual_sigma_m, n_inliers)."""
        arr = np.array(pts, dtype=np.float64)
        Xv  = arr[:, 0]
        Yv  = arr[:, 1]
        w   = (np.ones(len(pts), dtype=np.float64)
               if weights is None
               else np.asarray(weights, dtype=np.float64))
        w   = np.clip(w, 1e-6, None)

        try:
            coeffs = np.polyfit(Xv, Yv, deg=2, w=w)
        except (np.linalg.LinAlgError, ValueError):
            return None, 1.0, 0

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

    def _compute_errors(self, coeffs: np.ndarray) -> Tuple[float, float]:
        A, B, C = float(coeffs[0]), float(coeffs[1]), float(coeffs[2])
        e_y   = A * self._ld**2 + B * self._ld + C
        e_psi = math.atan(B)
        return e_y, e_psi

    def estimate(
        self,
        pts_meters: List[Tuple[float, float]],
        weights:    Optional[List[float]] = None,
    ) -> Tuple[float, float, bool]:
        """Per-frame: Kalman predict, fit+update if enough points, then
        derive (e_y, e_psi, valid) from the smoothed coefficients."""
        self._kf.predict()

        n_pts = len(pts_meters)
        if n_pts >= self._min_pts:
            coeffs_meas, sigma, n_inliers = self._polyfit(pts_meters, weights)
            if coeffs_meas is not None:
                fresh_ratio   = n_inliers / max(1, n_pts)
                sigma_penalty = max(1.0, sigma / 0.008)  # normalised to 8mm
                r_scale       = sigma_penalty / max(0.1, fresh_ratio)
                self._kf.update(coeffs_meas, r_scale)
                self._loss_frames = 0
        else:
            self._loss_frames += 1

        smooth = self._kf.state
        # Guards against locking onto a bad initial fit.
        if smooth is not None and abs(smooth[2]) > 0.8:
            self._kf.reset()
            self._loss_frames = 0
            smooth = None
        if smooth is not None and self._loss_frames <= self._max_loss_frames:
            e_y, e_psi = self._compute_errors(smooth)
            return e_y, e_psi, True

        return 0.0, 0.0, False

    def reset(self):
        self._kf.reset()
        self._loss_frames = 0

    @property
    def cached_coeffs(self) -> Optional[np.ndarray]:
        return self._kf.state

    @property
    def loss_frames(self) -> int:
        return self._loss_frames

    @property
    def has_cache(self) -> bool:
        return self._kf.ready
