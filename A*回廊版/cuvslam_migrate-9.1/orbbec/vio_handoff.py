"""Camera-owned tracker lifecycle and ordered IMU buffers. No SDK imports."""
from collections import deque
import queue
import threading
import time


def wait_while_paused(pause, stop, acknowledgement=None):
    """An already-set pause Event cannot itself be used as a sleeping wait."""
    if not pause.is_set():
        return False
    try:
        while pause.is_set() and not stop.is_set():
            if acknowledgement is not None:
                acknowledgement.set()
            stop.wait(.02)
    finally:
        if acknowledgement is not None:
            acknowledgement.clear()
    return True


class AnchorHandoff:
    """One request at a time; ready means camera AND IMU are parked.

    Main calls start(), performs localization only after ready, then finish().
    Camera warms its temporary tracker, calls park(), replaces its VIO tracker
    after release, then complete(). A new request cannot overtake that reset.
    """
    def __init__(self, pause, stop):
        self.pause, self.stop = pause, stop
        self.requested = threading.Event()
        self.ready = threading.Event()
        self.release = threading.Event()
        self.finished = threading.Event()
        self.imu_parked = threading.Event()
        self.cancelled = threading.Event()
        self.generation = 0
        self.data = dict(images=None, timestamp_ns=None, slam_pose=None, odom_pose=None)

    def start(self, use_lastpose_guess=False):
        if self.requested.is_set() or self.stop.is_set():
            raise RuntimeError('上一轮 tracker 交接尚未完成或采集已停止')
        self.ready.clear()
        self.release.clear()
        self.finished.clear()
        self.cancelled.clear()
        self.imu_parked.clear()
        self.generation += 1
        self.data.update(images=None, timestamp_ns=None, anchor_tracker=None, warm_ok=False,
                         anchor_error=None, worker_error=None, use_lastpose_guess=use_lastpose_guess)
        # Publish the request before releasing manual pause: camera must enter warmup,
        # while IMU resumes. Downstream publication remains inhibited by main's busy flag.
        self.requested.set()
        self.pause.clear()

    def park(self, timeout_s=3.):
        self.pause.set()
        deadline = time.monotonic()+timeout_s
        while not self.imu_parked.wait(.02):
            if self.stop.is_set() or time.monotonic() >= deadline:
                raise RuntimeError('IMU 线程未确认挂起，禁止进入 localize')
        self.ready.set()
        while not self.release.wait(.02):
            if self.stop.is_set():
                return False
        self.release.clear()
        return not self.stop.is_set()

    def finish(self, timeout_s=15.):
        if not self.ready.is_set() or not self.imu_parked.is_set():
            raise RuntimeError('定位结束时采集握手状态不一致')
        self.release.set()
        deadline = time.monotonic()+timeout_s
        while not self.finished.wait(.02):
            if self.stop.is_set() or time.monotonic() >= deadline:
                raise RuntimeError('相机线程未完成新 VIO tracker 交接')
        if self.data.get('worker_error'):
            raise RuntimeError(self.data['worker_error'])
        self.ready.clear()

    def complete(self, tracker):
        self.data.update(vio_generation=self.generation, vio_tracker_id=id(tracker))
        self.requested.clear()
        self.finished.set()

    def fail(self, error):
        self.data['worker_error'] = str(error)
        self.stop.set()
        self.finished.set()


class TrackerImuBuffer:
    """State belongs to one tracker, never shared between VIO and localization."""
    def __init__(self):
        self.pending = deque()
        self.last_timestamp = None
        self.last_image = None
        self.discarded = 0
        self.reason = None

    def prepare(self, timestamp_ns, imu_queue, stop, cancelled=None, timeout_s=.15,
                max_lag_ns=20_000_000, max_ahead_ns=250_000_000):
        self.reason = None
        if stop.is_set() or (cancelled is not None and cancelled.is_set()):
            self.reason = 'cancelled'
            return False
        if self.last_image is not None and timestamp_ns <= self.last_image:
            self.reason = 'image_timestamp_not_increasing'
            return False
        # A fresh tracker must not ingest IMU left over from a long pause. Keep a
        # short current initialization window, then advance the bound on every Track.
        if self.last_image is None:
            self.last_timestamp = timestamp_ns-200_000_000
        deadline = time.monotonic()+timeout_s
        while True:
            while True:
                try:
                    self.pending.append(imu_queue.get_nowait())
                except queue.Empty:
                    break
            while self.pending and self.pending[0].timestamp_ns < self.last_timestamp:
                self.pending.popleft()
                self.discarded += 1
            before = [s.timestamp_ns for s in self.pending if s.timestamp_ns <= timestamp_ns]
            newest = self.pending[-1].timestamp_ns if self.pending else None
            if newest is not None and newest-timestamp_ns > max_ahead_ns:
                self.reason = 'image_behind_imu'
                return False
            if before and 0 <= timestamp_ns-before[-1] <= max_lag_ns:
                return True
            if stop.is_set() or (cancelled is not None and cancelled.is_set()):
                self.reason = 'cancelled'
                return False
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                self.reason = 'imu_not_fresh'
                return False
            try:
                self.pending.append(imu_queue.get(timeout=min(.01, remaining)))
            except queue.Empty:
                pass

    def tracked(self, timestamp_ns):
        # Even a Track with no valid pose consumed the image timestamp.
        self.last_timestamp = timestamp_ns
        self.last_image = timestamp_ns
