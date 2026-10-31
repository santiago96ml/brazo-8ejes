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


def test_geometric_traces_folded_arm_from_base_to_gripper():
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    folded = np.array([[600, 640], [475, 470], [345, 210], [95, 565]], dtype=np.int32)
    cv2.polylines(img, [folded], False, (200, 40, 20), 34)

    result = ArmVisionAnalyzer(mode="geometric").analyze(img)

    assert result["mode_used"] == "geometric"
    keypoints = result["keypoints_px"]
    assert keypoints["base"][1] > 560
    assert keypoints["shoulder"][0] > keypoints["elbow"][0]
    assert keypoints["elbow"][1] < keypoints["shoulder"][1]
    assert keypoints["gripper_tip"][0] < keypoints["wrist_pitch"][0]
    assert keypoints["gripper_tip"][1] > keypoints["elbow"][1]


def test_geometric_arched_arm_uses_anatomical_moment_regions():
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    mask = np.zeros(img.shape[:2], dtype=np.uint8)
    cv2.line(mask, (760, 310), (950, 490), 255, 48)
    cv2.line(mask, (950, 490), (950, 640), 255, 48)
    cv2.line(mask, (760, 310), (625, 525), 255, 48)
    cv2.line(mask, (625, 525), (505, 625), 255, 48)
    cv2.line(mask, (505, 625), (380, 685), 255, 48)

    analyzer = ArmVisionAnalyzer(mode="geometric")
    analyzer._blue_mask = lambda frame: (mask, np.zeros_like(mask))
    result = analyzer._geometric(img)

    assert result is not None
    keypoints = result[1]
    assert 700 < keypoints["elbow"][0] < 850
    assert keypoints["elbow"][1] < 380
    assert keypoints["shoulder"][0] > keypoints["elbow"][0]
    assert keypoints["shoulder"][1] > keypoints["elbow"][1]
    assert keypoints["base"][0] == keypoints["shoulder"][0]
    assert keypoints["base"][1] > keypoints["shoulder"][1] + 40
    assert keypoints["wrist_pitch"][0] < keypoints["elbow"][0]
    assert keypoints["wrist_roll"][0] < keypoints["wrist_pitch"][0]
    assert keypoints["gripper_tip"][0] < keypoints["wrist_roll"][0]
    assert keypoints["gripper_tip"][1] > keypoints["wrist_roll"][1]


def test_geometric_keypoint_ema_uses_15_percent_median_when_mask_is_stable():
    analyzer = ArmVisionAnalyzer(mode="geometric")
    first = {
        name: (float(index * 10), float(index * 10), 0.8)
        for index, name in enumerate(analyzer.KEYPOINT_ORDER)
    }
    analyzer._smooth_keypoints(first, (100.0, 100.0))
    shifted = {
        name: (point[0] + 4.0, point[1] + 4.0, point[2])
        for name, point in first.items()
    }

    smoothed = analyzer._smooth_keypoints(shifted, (104.0, 104.0))

    assert smoothed["base"] == (0.3, 0.3, 0.8)


def test_local_mask_centroid_ignores_pixels_outside_radius():
    analyzer = ArmVisionAnalyzer(mode="geometric")
    mask = np.zeros((120, 120), dtype=np.uint8)
    mask[40:61, 40:61] = 255
    mask[45:56, 90:101] = 255

    centroid = analyzer._local_mask_centroid(mask, (50.0, 50.0))

    assert centroid == (50.0, 50.0)


def test_keypoint_smoothing_uses_ema_and_resets_large_jumps():
    analyzer = ArmVisionAnalyzer(mode="geometric")
    first = {
        name: (float(index * 10), float(index * 10), 0.8)
        for index, name in enumerate(analyzer.KEYPOINT_ORDER)
    }
    initial = analyzer._smooth_keypoints(first)
    shifted = {
        name: (point[0] + 4.0, point[1] + 4.0, point[2])
        for name, point in first.items()
    }

    smoothed = analyzer._smooth_keypoints(shifted)

    for name, point in first.items():
        assert smoothed[name] == (point[0] + 1.0, point[1] + 1.0, point[2])
    jumped = dict(shifted)
    jumped["base"] = (50.0, 50.0, 0.8)
    reset = analyzer._smooth_keypoints(jumped)
    assert reset["base"] == (50.0, 50.0, 0.8)
    assert initial["base"] == first["base"]


def test_neural_predictor_unavailable():
    p = KeypointNeuralPredictor()
    assert p.available is False
    assert p.predict(np.zeros((100, 100, 3), dtype=np.uint8)) is None


def test_wrist_roll_uses_signed_dark_servo_silhouette_asymmetry():
    hsv = np.zeros((100, 160, 3), dtype=np.uint8)
    hsv[:, :, 2] = 255
    hsv[55:76, 20:141] = (0, 0, 0)

    theta5 = ArmVisionAnalyzer._estimate_wrist_roll(
        hsv, (20.0, 50.0), (140.0, 50.0), (1.0, 0.0)
    )

    assert 0.0 < theta5 <= 90.0
