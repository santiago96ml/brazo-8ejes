"""Tests for the hybrid arm vision analyzer (S12)."""

import os
import sys

import cv2
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

try:
    from src.arm_vision_analyzer import ArmVisionAnalyzer, KeypointNeuralPredictor
except Exception:  # pragma: no cover
    from arm_vision_analyzer import ArmVisionAnalyzer, KeypointNeuralPredictor


def _blue_arm_frame():
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    cv2.line(img, (300, 600), (900, 200), (200, 40, 20), 14)
    cv2.rectangle(img, (580, 380), (620, 420), (0, 0, 0), -1)
    return img


def test_geometric_blue_arm():
    img = _blue_arm_frame()
    a = ArmVisionAnalyzer()
    r = a.analyze(img)
    assert r["mode_used"] == "geometric"
    assert r["thetas_deg"] is not None and len(r["thetas_deg"]) == 6
    assert len(r["skeleton"]) >= 5
    assert r["confidence"] > 0.3
    assert r["latency_ms"] > 0


def test_no_blue_returns_none():
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    a = ArmVisionAnalyzer()
    r = a.analyze(img)
    assert r["mode_used"] == "none"
    assert r["thetas_deg"] is None


def test_45_degree_segment():
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    cv2.line(img, (300, 600), (700, 200), (200, 40, 20), 14)
    a = ArmVisionAnalyzer()
    r = a.analyze(img)
    assert r["mode_used"] == "geometric"
    assert abs(r["thetas_deg"][1] - 45.0) <= 12.0


def test_neural_predictor_unavailable():
    p = KeypointNeuralPredictor()
    assert p.available is False
    assert p.predict(np.zeros((100, 100, 3), dtype=np.uint8)) is None
