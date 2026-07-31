"""Single-in-flight worker that always replaces its pending frame with the latest."""

import threading
import time
from typing import Callable, Optional

from .models import FrameSnapshot, WorkerResult


class LatestFrameWorker:
    def __init__(
        self,
        infer: Callable,
        on_result: Callable[[WorkerResult], None],
        get_raw_response: Optional[Callable[[], Optional[str]]] = None,
        pass_snapshot: bool = False,
    ):
        self._infer = infer
        self._on_result = on_result
        self._get_raw_response = get_raw_response
        self._pass_snapshot = bool(pass_snapshot)
        self._condition = threading.Condition()
        self._pending: Optional[FrameSnapshot] = None
        self._stopping = False
        self.in_flight = False
        self.replaced_frames = 0
        self._thread = threading.Thread(target=self._run, name="vlm-latest-frame", daemon=True)
        self._thread.start()

    def submit(self, snapshot: FrameSnapshot):
        with self._condition:
            if self._pending is not None:
                self.replaced_frames += 1
            self._pending = snapshot
            self._condition.notify()

    def stop(self, timeout: float = 1.0):
        with self._condition:
            self._stopping = True
            self._pending = None
            self._condition.notify_all()
        self._thread.join(timeout=timeout)

    def _run(self):
        while True:
            with self._condition:
                while self._pending is None and not self._stopping:
                    self._condition.wait()
                if self._stopping:
                    return
                snapshot = self._pending
                self._pending = None
                self.in_flight = True
            started = time.monotonic()
            try:
                result = (
                    self._infer(snapshot)
                    if self._pass_snapshot
                    else self._infer(snapshot.rgb, snapshot.target_description)
                )
                raw_response = (
                    self._get_raw_response()
                    if self._get_raw_response is not None
                    else None
                )
                completed = WorkerResult(
                    snapshot,
                    result,
                    time.monotonic() - started,
                    raw_response=raw_response,
                )
            except Exception as error:  # the ROS thread receives a safe error value
                raw_response = (
                    self._get_raw_response()
                    if self._get_raw_response is not None
                    else None
                )
                completed = WorkerResult(
                    snapshot,
                    None,
                    time.monotonic() - started,
                    f"{type(error).__name__}: {error}",
                    raw_response,
                )
            finally:
                with self._condition:
                    self.in_flight = False
            self._on_result(completed)
