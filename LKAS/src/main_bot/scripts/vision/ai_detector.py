#!/usr/bin/env python3
"""
ai_detector.py — ONNX inference wrapper for the lane segmentation model.

Model: EgoLanes_Lite_FP32.onnx
  Input  : [1, 3, H_model, W_model]  float32, ImageNet-normalised RGB
  Output : [1, 3, H_model, W_model]  logits (multi-label, not argmax)
             ch0 = left  ego-lane marking
             ch1 = right ego-lane marking
             ch2 = other lanes (ignored)

detect_raw() returns unprocessed logits so the caller can use relu(logit) as
a sub-pixel confidence weight (see processor.py) instead of thresholding to
a binary mask.
"""

from typing import Tuple

import cv2
import numpy as np
import onnxruntime as ort

_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

_CH_LEFT  = 0
_CH_RIGHT = 1


class AIDetector:
    def __init__(self, model_path: str):
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self._sess     = ort.InferenceSession(model_path, providers=providers)
        self._inp_name = self._sess.get_inputs()[0].name
        _, _, self._model_h, self._model_w = self._sess.get_inputs()[0].shape

    def _preprocess(self, frame_bgr: np.ndarray) -> np.ndarray:
        rgb     = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self._model_w, self._model_h),
                             interpolation=cv2.INTER_LINEAR)
        img    = resized.astype(np.float32) / 255.0
        img    = (img - _IMAGENET_MEAN) / _IMAGENET_STD
        tensor = img.transpose(2, 0, 1)[np.newaxis]
        return np.ascontiguousarray(tensor, dtype=np.float32)

    def detect_raw(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Raw logits [3, H_m, W_m] in model space — no sigmoid, no resize."""
        tensor = self._preprocess(frame_bgr)
        logits = self._sess.run(None, {self._inp_name: tensor})[0]
        return logits[0]  # [3, H_m, W_m]

    def detect(self, frame_bgr: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """(left_prob, right_prob) float32 [0,1] at original resolution.
        Kept for backward compatibility; unused by the V2 processor."""
        orig_h, orig_w = frame_bgr.shape[:2]

        tensor = self._preprocess(frame_bgr)
        logits = self._sess.run(None, {self._inp_name: tensor})[0]
        pred   = logits[0]  # [3, H_m, W_m]

        left_prob  = (1.0 / (1.0 + np.exp(-np.clip(pred[_CH_LEFT],  -15, 15)))).astype(np.float32)
        right_prob = (1.0 / (1.0 + np.exp(-np.clip(pred[_CH_RIGHT], -15, 15)))).astype(np.float32)

        left_prob  = cv2.resize(left_prob,  (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        right_prob = cv2.resize(right_prob, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)

        return left_prob, right_prob
