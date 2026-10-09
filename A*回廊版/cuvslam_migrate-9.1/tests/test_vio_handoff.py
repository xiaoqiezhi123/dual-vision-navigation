"""Hardware-free lifecycle tests, including the actual camera worker body."""
import ast
from collections import deque
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace as NS
from typing import Deque, Optional
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'orbbec'))
from vio_handoff import AnchorHandoff, TrackerImuBuffer, wait_while_paused
from pose_result_queue import LatestPoseQueue, PoseResult


def sample(timestamp):
    return NS(timestamp_ns=timestamp, linear_accelerations=(0,0,9.8), angular_velocities=(0,0,0))


class HandoffTests(unittest.TestCase):
    def test_manual_pause_is_released_for_warmup_then_both_threads_park(self):
        pause, stop = threading.Event(), threading.Event()
        pause.set()
        h = AnchorHandoff(pause, stop)
        reads = []
        def imu():
            while not stop.is_set():
                if wait_while_paused(pause, stop, h.imu_parked):continue
                reads.append(time.monotonic())
                stop.wait(.001)
        worker = threading.Thread(target=imu,daemon=True);worker.start()
        self.addCleanup(stop.set)
        self.assertTrue(h.imu_parked.wait(1))
        h.start()
        deadline=time.monotonic()+1
        while not reads and time.monotonic()<deadline:time.sleep(.002)
        self.assertTrue(reads)
        new_tracker=object()
        def camera():
            if h.park():h.complete(new_tracker)
        camera_worker=threading.Thread(target=camera,daemon=True);camera_worker.start()
        self.assertTrue(h.ready.wait(1))
        self.assertTrue(h.imu_parked.is_set())
        n=len(reads);time.sleep(.025);self.assertEqual(len(reads),n)
        with self.assertRaises(RuntimeError):h.start()
        h.finish()
        self.assertEqual(h.data['vio_tracker_id'],id(new_tracker))
        self.assertEqual(h.data['vio_generation'],1)
        self.assertTrue(pause.is_set())  # No implicit motion/output resume.
        self.assertFalse(h.requested.is_set())
        stop.set();camera_worker.join(1);worker.join(1)
        self.assertFalse(camera_worker.is_alive());self.assertFalse(worker.is_alive())

    def test_imu_ack_is_required_before_localization(self):
        h=AnchorHandoff(threading.Event(),threading.Event());h.start()
        with self.assertRaisesRegex(RuntimeError,'IMU'):
            h.park(timeout_s=.02)
        self.assertFalse(h.ready.is_set())

    def test_stale_imu_and_buffered_images_are_not_tracked(self):
        state=TrackerImuBuffer();q=queue.Queue();stop=threading.Event()
        q.put(sample(1_000_000_000))
        self.assertFalse(state.prepare(74_000_000_000,q,stop,timeout_s=0))
        self.assertEqual(state.reason,'imu_not_fresh')
        self.assertFalse(state.pending)
        q.put(sample(73_995_000_000))
        self.assertTrue(state.prepare(74_000_000_000,q,stop,timeout_s=0))
        state.tracked(74_000_000_000)
        self.assertFalse(state.prepare(74_000_000_000,q,stop,timeout_s=0))
        self.assertEqual(state.reason,'image_timestamp_not_increasing')
        other=TrackerImuBuffer();q.put(sample(75_000_000_000))
        self.assertFalse(other.prepare(74_100_000_000,q,stop,timeout_s=0))
        self.assertEqual(other.reason,'image_behind_imu')

    def test_waits_for_current_imu_and_cancellation_has_priority(self):
        q=queue.Queue();stop=threading.Event();cancel=threading.Event();state=TrackerImuBuffer()
        producer=threading.Timer(.01,lambda:q.put(sample(995_000_000)));producer.start()
        self.assertTrue(state.prepare(1_000_000_000,q,stop,timeout_s=.1))
        producer.join()
        cancel.set()
        self.assertFalse(state.prepare(1_000_000_000,q,stop,cancel))

    def test_production_handoff_resets_baseline_also_without_successful_anchor(self):
        tree=ast.parse((ROOT/'orbbec/run_vio_tasknav.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        reset=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='complete_anchor_handoff')
        wrapper=ast.parse('''def scenario():
    odom_baseline_pending=False
    last_odom=object()
    last_pose_source_ns=123
    last_pose_generation=1
''').body[0]
        wrapper.body.append(reset)
        wrapper.body.extend(ast.parse('''complete_anchor_handoff()
return odom_baseline_pending, last_odom, last_pose_source_ns, last_pose_generation
''').body)
        q=LatestPoseQueue();q.put_latest(PoseResult([123],1.,1.01,1))
        h=NS(finish=Mock(),generation=2)
        scope=dict(handoff=h,q=q)
        exec(compile(ast.fix_missing_locations(ast.Module([wrapper],[])),'production_baseline_reset','exec'),scope)
        self.assertEqual(scope['scenario'](),(True,None,None,2))
        self.assertTrue(q.empty());h.finish.assert_called_once()

    def camera_scope(self):
        tree=ast.parse((ROOT/'orbbec/run_vio_tasknav.py').read_text())
        nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('camera_thread','register_imu_until')]
        trace=Mock(enabled=True)
        scope=dict(queue=queue,threading=threading,time=time,deque=deque,Deque=Deque,Optional=Optional,
                   vslam=NS(Tracker=NS(Internals=object),ImuMeasurement=NS),
                   ThreadWithTimestamp=object,Pipeline=object,DepthShmWriter=object,ImuSample=object,
                   AnchorHandoff=AnchorHandoff,TrackerImuBuffer=TrackerImuBuffer,wait_while_paused=wait_while_paused,
                   PoseResult=PoseResult,get_trace=lambda:trace,process_ir_frame=lambda f,target_size:object(),
                   SLAM_RESOLUTION=(10,10),ANCHOR_WARMUP_FRAMES=3,HEALTH_WINDOW_FRAMES=5,
                   HEALTH_PRINT_INTERVAL_S=10.,print_tracking_health=lambda *a:None,diag_emit=lambda *a:None,
                   ENABLE_MAPPING_VISUALIZATION=False,OBFrameType=NS(LEFT_IR_FRAME=1,RIGHT_IR_FRAME=2))
        exec(compile(ast.Module(nodes,[]),'actual_camera_worker','exec'),scope)
        return scope,trace

    def run_actual_camera(self,cancel=False):
        scope,trace=self.camera_scope()
        pause,stop=threading.Event(),threading.Event();pause.set()
        h=AnchorHandoff(pause,stop);q=queue.Queue();results=LatestPoseQueue()
        self.addCleanup(stop.set);self.addCleanup(h.release.set)
        class Tracker:
            def __init__(self):self.tracks=[];self.imu=[];self.last=-1
            def register_imu_measurement(self,idx,s):
                if s.timestamp_ns<self.last:raise AssertionError('IMU older than last Track')
                self.last=s.timestamp_ns;self.imu.append(s.timestamp_ns)
            def track(self,ts,images,internals):
                if ts<=self.last:raise AssertionError('Track not strictly newer')
                self.last=ts;self.tracks.append(ts)
                return NS(world_from_rig=NS(pose=NS(translation=[0,0,0],rotation=[0,0,0,1]))),None
            def get_last_observations(self,_):return []
        created=[]
        def factory(**kw):
            trk=Tracker();created.append((kw,trk));return trk
        class IR:
            timestamp=1_000_000_000
            def wait_for_frames(self,_):
                self.timestamp+=100_000_000
                for t in (self.timestamp-15_000_000,self.timestamp-5_000_000):q.put(sample(t))
                frame=NS(get_timestamp_us=lambda:self.timestamp/1000)
                return NS(get_frame=lambda _:frame)
        def imu_pause_loop():
            while not stop.is_set():
                if not wait_while_paused(pause,stop,h.imu_parked):stop.wait(.001)
        imu=threading.Thread(target=imu_pause_loop,daemon=True);imu.start()
        self.assertTrue(h.imu_parked.wait(1));h.start()
        if cancel:h.cancelled.set()
        timestamps=NS(prev_low_rate_timestamp=None,low_rate_threshold_ns=200_000_000)
        camera=threading.Thread(target=scope['camera_thread'],args=(None,results,q,timestamps,IR(),False,object(),
            stop,pause,h.requested,h.ready,h.release,h.data),kwargs=dict(handoff=h,tracker_factory=factory),daemon=True)
        camera.start()
        self.assertTrue(h.ready.wait(2),h.data)
        self.assertEqual(h.data['warm_ok'],not cancel)
        self.assertTrue(results.empty())
        h.finish()
        self.assertIsNone(h.data['anchor_tracker'])
        self.assertEqual(len(created),3)
        self.assertEqual(created[0][1].tracks,[])
        self.assertEqual(created[0][0],dict(slam=False))
        self.assertEqual(created[1][0],dict(slam=True,sync=True))
        self.assertEqual(created[2][0],dict(slam=False))
        # Main deliberately hasn't issued resume yet.
        self.assertTrue(results.empty())
        pause.clear()
        result=results.get(timeout=2)
        stop.set();camera.join(1);imu.join(1)
        self.assertFalse(camera.is_alive());self.assertFalse(imu.is_alive())
        self.assertEqual(result.tracker_generation,1)
        self.assertTrue(created[2][1].tracks)
        self.assertEqual(h.data['vio_tracker_id'],id(created[2][1]))
        self.assertTrue(any(c.args[0]=='tracker_switched' and c.kwargs['tracker_generation']==1
                            for c in trace.emit.call_args_list))
        self.assertIsNone(h.data.get('worker_error'))

    def test_actual_camera_switches_tracker_and_imu_after_localize(self):
        self.run_actual_camera()

    def test_actual_camera_cancellation_also_resets_and_keeps_manual_pause(self):
        self.run_actual_camera(cancel=True)


if __name__=='__main__':unittest.main()
