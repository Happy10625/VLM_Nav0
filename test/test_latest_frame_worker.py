import threading
import time

import numpy as np

from vlm_nav.latest_frame_worker import LatestFrameWorker
from vlm_nav.models import FrameSnapshot, VLMResult


def snapshot(sequence):
    return FrameSnapshot(
        sequence=sequence,
        task_epoch=1,
        target_description="chair",
        captured_monotonic=time.monotonic(),
        stamp=None,
        frame_id="camera",
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_m=np.ones((2, 2), dtype=np.float32),
        intrinsics=(1.0, 1.0, 0.0, 0.0),
        transform_matrix=np.eye(4),
    )


def test_worker_has_one_in_flight_and_replaces_pending_with_latest():
    release = threading.Event()
    started = threading.Event()
    completed = []

    def infer(_image, _target):
        started.set()
        release.wait(timeout=2.0)
        return VLMResult(
            target_visible=False,
            object_match=False,
            qualifier_match=True,
            relation_match=True,
            confidence=0.1,
            target_pixel=None,
            evidence_pixel=None,
        )

    worker = LatestFrameWorker(
        infer,
        completed.append,
        get_raw_response=lambda: '{"target_visible":false}',
    )
    try:
        worker.submit(snapshot(1))
        assert started.wait(timeout=1.0)
        assert worker.in_flight
        worker.submit(snapshot(2))
        worker.submit(snapshot(3))
        release.set()
        deadline = time.monotonic() + 2.0
        while len(completed) < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert [item.snapshot.sequence for item in completed] == [1, 3]
        assert all(
            item.raw_response == '{"target_visible":false}' for item in completed
        )
        assert worker.replaced_frames == 1
    finally:
        release.set()
        worker.stop()
