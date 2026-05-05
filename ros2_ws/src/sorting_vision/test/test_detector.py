"""Tests for the rule-based tray detector helpers."""

from sorting_interfaces.msg import TrayCell

from sorting_vision.algorithms.detector import (
    DetectionConfig,
    ImageView,
    RuleBasedTrayDetector,
    TrayRoi,
    classify_by_green_ratio,
    sample_depth_mm,
)


def test_sample_depth_mm_uses_median_16uc1_window():
    """Depth sampling should ignore zeros and return millimeters."""
    values = [
        0, 100, 110,
        90, 120, 130,
        0, 140, 150,
    ]
    data = b''.join(value.to_bytes(2, 'little') for value in values)
    image = ImageView(width=3, height=3, encoding='16UC1', step=6, data=data)

    assert sample_depth_mm(image, 1, 1, 3) == 120.0


def test_classify_by_green_ratio_thresholds():
    """Green area ratio should map to empty, weak, and good classes."""
    assert classify_by_green_ratio(0.0, 0.20, 0.03, 0.0)[0] == TrayCell.CLASS_EMPTY
    assert classify_by_green_ratio(0.05, 0.20, 0.03, 600.0)[0] == TrayCell.CLASS_WEAK
    assert classify_by_green_ratio(0.25, 0.20, 0.03, 600.0)[0] == TrayCell.CLASS_GOOD


def test_detector_generates_150_cells_for_three_rois():
    """Three configured tray rectangles should produce a full matrix."""
    color = ImageView(
        width=30,
        height=20,
        encoding='rgb8',
        step=90,
        data=bytes([0, 80, 0] * 30 * 20),
    )
    detector = RuleBasedTrayDetector(
        DetectionConfig(
            leaf_area_ratio_threshold=0.20,
            weak_area_ratio_threshold=0.03,
            color_sample_step_px=4,
        )
    )
    rois = [
        TrayRoi(1, 0.0, 0.0, 10.0, 20.0),
        TrayRoi(2, 10.0, 0.0, 10.0, 20.0),
        TrayRoi(3, 20.0, 0.0, 10.0, 20.0),
    ]

    detections = detector.detect(color, None, rois)

    assert len(detections) == 150
    assert detections[0].tray_id == 1
    assert detections[0].row == 1
    assert detections[0].col == 1
    assert detections[-1].tray_id == 3
    assert detections[-1].row == 10
    assert detections[-1].col == 5
