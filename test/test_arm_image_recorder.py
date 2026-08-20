from datetime import datetime, timedelta
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from vlm_nav.arm_image_recorder import ArmImageRecorder
from vlm_nav.models import FrontierDecision, Pixel, VLMResult


def target_snapshot(sequence=1):
    return SimpleNamespace(
        sequence=sequence,
        request_kind="target",
        rgb=np.zeros((160, 240, 3), dtype=np.uint8),
        auxiliary_rgb=None,
    )


def test_recorder_retains_only_latest_three_arm_sessions(tmp_path):
    recorder = ArmImageRecorder(tmp_path, keep_sessions=3)
    start = datetime(2026, 1, 1, 10, 0, 0)
    created = [
        recorder.start_arm(start + timedelta(seconds=index)) for index in range(4)
    ]

    assert not Path(created[0]).exists()
    assert len(list(tmp_path.glob("arm_*"))) == 3


def test_target_record_draws_only_target_and_semantic_evidence(tmp_path):
    recorder = ArmImageRecorder(tmp_path)
    recorder.start_arm(datetime(2026, 1, 1, 10, 0, 0))
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.9,
        target_pixel=Pixel(180, 40),
        evidence_pixel=Pixel(200, 30),
    )

    saved = recorder.record(target_snapshot(), result, "accepted_target")
    retried = recorder.record(target_snapshot(), result, "accepted_target")

    assert len(saved) == 1
    assert saved[0] != retried[0]
    image = cv2.imread(saved[0])
    assert image is not None
    assert np.count_nonzero(image) > 0


def test_failed_target_record_preserves_frame_without_coordinate_grid(tmp_path):
    recorder = ArmImageRecorder(tmp_path, jpeg_quality=100)
    recorder.start_arm(datetime(2026, 1, 1, 10, 0, 0))

    saved = recorder.record(target_snapshot(), None, "api_error")

    image = cv2.imread(saved[0])
    assert image is not None
    assert np.count_nonzero(image) == 0


def test_target_annotation_marks_semantic_evidence_pixel():
    image = np.zeros((160, 240, 3), dtype=np.uint8)
    result = VLMResult(
        target_visible=True,
        object_match=True,
        qualifier_match=True,
        relation_match=True,
        confidence=0.9,
        target_pixel=Pixel(180, 40),
        evidence_pixel=Pixel(35, 50),
    )

    ArmImageRecorder.draw_target_annotation(image, result)

    assert np.count_nonzero(image[40:61, 25:46]) > 0


def test_frontier_record_saves_both_inputs_and_marks_selected_path(tmp_path):
    recorder = ArmImageRecorder(tmp_path)
    recorder.start_arm(datetime(2026, 1, 1, 10, 0, 0))
    snapshot = SimpleNamespace(
        sequence=7,
        request_kind="frontier",
        rgb=np.zeros((100, 200, 3), dtype=np.uint8),
        auxiliary_rgb=np.zeros((200, 200, 3), dtype=np.uint8),
        frontier_robot_pixel=(30, 170),
        frontier_candidate_pixels=((2, (170, 30)),),
    )

    saved = recorder.record(
        snapshot, FrontierDecision(2, 0.8, "open space"), "accepted_frontier"
    )

    assert len(saved) == 2
    annotated_map = cv2.imread(saved[1])
    assert annotated_map is not None
    assert np.count_nonzero(annotated_map) > 0


def test_navigation_events_are_appended_to_arm_jsonl(tmp_path):
    recorder = ArmImageRecorder(tmp_path)
    session = Path(recorder.start_arm(datetime(2026, 1, 1, 10, 0, 0)))

    assert recorder.record_event({"event": "target_confirmed"})
    assert recorder.record_event(
        {"event": "direct_path_accepted", "deviation_m": 0.03}
    )

    records = [
        json.loads(line)
        for line in (session / "events.jsonl").read_text(
            encoding="utf-8"
        ).splitlines()
    ]
    assert [record["event"] for record in records] == [
        "target_confirmed",
        "direct_path_accepted",
    ]
    assert all("logged_at" in record for record in records)
