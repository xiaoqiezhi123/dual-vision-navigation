"""Offline tests: no SDK imports, camera streams, UDP or robot commands."""
import ast
from pathlib import Path
import queue
import sys
import threading
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'orbbec'))
from pose_result_queue import LatestPoseQueue, PoseResult


def result(seq, received=10., processed=10.05):
    return PoseResult([seq], received, processed)


class LatestPoseTests(unittest.TestCase):
    def test_stalled_consumer_gets_latest_result_without_replaying_backlog(self):
        handoff = LatestPoseQueue()
        for seq in range(18):
            handoff.put_latest(result(seq))
        self.assertEqual(handoff.qsize(), 1)
        self.assertEqual(handoff.get_nowait().values, [17])
        handoff.task_done()
        self.assertEqual(handoff.dropped_results, 17)
        self.assertEqual(handoff.unfinished_tasks, 0)
        with self.assertRaises(queue.Empty):
            handoff.get_nowait()

    def test_producer_keeps_tracking_when_consumer_is_blocked(self):
        handoff = LatestPoseQueue()
        tracked = []
        def produce():
            for seq in range(1000):
                tracked.append(seq)
                handoff.put_latest(result(seq))
        producer = threading.Thread(target=produce, daemon=True)
        producer.start()
        producer.join(timeout=2)
        self.assertFalse(producer.is_alive())
        self.assertEqual(len(tracked), 1000)
        self.assertEqual(handoff.get_nowait().values, [999])
        handoff.task_done()

    def test_long_tracking_or_queue_delay_cannot_be_sent_as_fresh(self):
        self.assertTrue(result(1).is_fresh(10.2, .5))
        self.assertFalse(result(1).is_fresh(11., .5))
        self.assertFalse(result(1, processed=11.).is_fresh(11.01, .5))

    def test_invalid_timing_rejected(self):
        for item, now in ((result(1, processed=9.), 10.), (result(1), 9.),
                          (result(1, received=float('nan')), 10.2), (result(1), float('inf'))):
            with self.subTest(item=item, now=now):
                self.assertFalse(item.is_fresh(now, .5))

    def test_anchor_clear_prevents_old_frame_from_becoming_new_baseline(self):
        handoff = LatestPoseQueue()
        handoff.put_latest(result(1))
        handoff.clear_pending()
        handoff.clear_pending()
        handoff.put_latest(result(2, received=20., processed=20.1))
        self.assertEqual(handoff.get_nowait().values, [2])
        handoff.task_done()
        self.assertEqual(handoff.unfinished_tasks, 0)

    def test_skipping_completed_results_preserves_full_odometry_motion(self):
        # Execute the production pure pose math without importing camera/GPU code.
        tree = ast.parse((ROOT/'orbbec/run_vio_tasknav.py').read_text())
        names = {'quat_mul', 'quat_rotate', 'pose_compose', 'pose_inverse'}
        selected = ast.Module([n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names], [])
        scope = {'np': np, 'vslam': SimpleNamespace(Pose=SimpleNamespace)}
        exec(compile(selected, 'production_pose_math', 'exec'), scope)
        compose, inverse = scope['pose_compose'], scope['pose_inverse']
        poses = [SimpleNamespace(translation=[np.sin(i*.1), i*.12, i*.03],
                                rotation=[0., np.sin(i*.025), 0., np.cos(i*.025)]) for i in range(18)]
        anchor = SimpleNamespace(translation=[10., .2, -4.], rotation=[.5, .5, .5, .5])
        every_frame = anchor
        for previous, current in zip(poses, poses[1:]):
            every_frame = compose(every_frame, compose(inverse(previous), current))
        latest_only = compose(anchor, compose(inverse(poses[0]), poses[-1]))
        np.testing.assert_allclose(latest_only.translation, every_frame.translation, atol=1e-10)
        np.testing.assert_allclose(latest_only.rotation, every_frame.rotation, atol=1e-10)


if __name__ == '__main__':
    unittest.main()
