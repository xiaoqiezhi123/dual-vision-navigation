"""Bounded handoff of completed VIO results; camera tracking still runs every frame."""
from dataclasses import dataclass
import math
import queue


@dataclass(frozen=True)
class PoseResult:
    values: list
    received_monotonic: float
    processed_monotonic: float
    tracker_generation: int = 0

    def age(self, now):
        return now - self.received_monotonic

    def is_fresh(self, now, max_age_s):
        return (all(math.isfinite(v) for v in (now, self.received_monotonic, self.processed_monotonic))
                and self.received_monotonic <= self.processed_monotonic <= now
                and 0 <= self.age(now) <= max_age_s)


class LatestPoseQueue(queue.Queue):
    """Single producer, single consumer. Never replay a backlog of old poses."""
    def __init__(self):
        super().__init__(maxsize=1)
        self.dropped_results = 0

    def put_latest(self, result):
        # A consumer can race the replacement: retry if it emptied the queue.
        while True:
            try:
                self.put_nowait(result)
                return
            except queue.Full:
                try:
                    self.get_nowait()
                    self.task_done()
                    self.dropped_results += 1
                except queue.Empty:
                    pass

    def clear_pending(self):
        # Caller uses this while the producer is paused for localization.
        while True:
            try:
                self.get_nowait()
                self.task_done()
                self.dropped_results += 1
            except queue.Empty:
                return
