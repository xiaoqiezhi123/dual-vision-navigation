from __future__ import annotations

import copy
import math
import queue
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

import numpy as np


class NavMode(str, Enum):
    STANDBY = "STANDBY"
    LOW_SPEED = "LOW_SPEED"
    MEDIUM_SPEED = "MEDIUM_SPEED"
    EMERGENCY = "EMERGENCY"


@dataclass(frozen=True)
class ModeDecision:
    mode: NavMode
    run_policy: bool
    render_depth: bool
    force_zero: bool
    vx_max: float
    wz_max: float
    zero_reason_override: Optional[str] = None
    reset_adapter: bool = False
    zero_burst_count: int = 0
    preserve_policy_goal_dist: bool = False


@dataclass(frozen=True)
class LastEvent:
    key: str
    accepted: bool
    previous_mode: NavMode
    new_mode: NavMode
    message: str
    timestamp: float


_DECISIONS = {
    NavMode.STANDBY: ModeDecision(
        mode=NavMode.STANDBY,
        run_policy=False,
        render_depth=False,
        force_zero=True,
        vx_max=0.0,
        wz_max=0.0,
        zero_reason_override="standby",
        reset_adapter=True,
        zero_burst_count=3,
        preserve_policy_goal_dist=False,
    ),
    NavMode.LOW_SPEED: ModeDecision(
        mode=NavMode.LOW_SPEED,
        run_policy=True,
        render_depth=True,
        force_zero=False,
        vx_max=0.5,
        wz_max=0.35,
        zero_reason_override=None,
        reset_adapter=False,
        zero_burst_count=0,
        preserve_policy_goal_dist=False,
    ),
    NavMode.MEDIUM_SPEED: ModeDecision(
        mode=NavMode.MEDIUM_SPEED,
        run_policy=True,
        render_depth=True,
        force_zero=False,
        vx_max=0.8,
        wz_max=0.45,
        zero_reason_override=None,
        reset_adapter=False,
        zero_burst_count=0,
        preserve_policy_goal_dist=False,
    ),
    NavMode.EMERGENCY: ModeDecision(
        mode=NavMode.EMERGENCY,
        run_policy=True,
        render_depth=True,
        force_zero=True,
        vx_max=0.0,
        wz_max=0.0,
        zero_reason_override="emergency",
        reset_adapter=False,
        zero_burst_count=0,
        preserve_policy_goal_dist=True,
    ),
}

_KEY_TO_MODE = {
    "A": NavMode.STANDBY,
    "S": NavMode.LOW_SPEED,
    "D": NavMode.MEDIUM_SPEED,
    "F": NavMode.EMERGENCY,
    "G": NavMode.STANDBY,
}


class NavModeController:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._mode = NavMode.STANDBY
        self._status: dict[str, Any] = {}
        now = time.time()
        self._last_event = LastEvent(
            key="",
            accepted=True,
            previous_mode=NavMode.STANDBY,
            new_mode=NavMode.STANDBY,
            message="init",
            timestamp=now,
        )
        self._events: queue.Queue[LastEvent] = queue.Queue()
        self._input_stop = threading.Event()
        self._input_thread: Optional[threading.Thread] = None
        self._input_prompt = "NavSide> "

    def get_mode(self) -> NavMode:
        with self._lock:
            return self._mode

    def get_decision(self) -> ModeDecision:
        return _DECISIONS[self.get_mode()]

    def get_last_event(self) -> LastEvent:
        with self._lock:
            return self._last_event

    def update_status(self, **kwargs: Any) -> None:
        with self._lock:
            self._status.update(kwargs)

    def set_status(self, snapshot: dict[str, Any]) -> None:
        with self._lock:
            self._status = copy.deepcopy(snapshot) if snapshot is not None else {}

    def apply_line(self, line: str) -> LastEvent:
        text = "" if line is None else str(line).strip()
        key = text[:1].upper() if text else ""
        now = time.time()

        with self._lock:
            previous_mode = self._mode
            new_mode = previous_mode
            accepted = False
            message = "ignored"

            if key in _KEY_TO_MODE:
                new_mode = _KEY_TO_MODE[key]
                accepted = True
                if key == "G":
                    message = "quit_to_standby"
                elif key == "A":
                    message = "standby"
                elif key == "S":
                    message = "low_speed"
                elif key == "D":
                    message = "medium_speed"
                elif key == "F":
                    message = "emergency"
                self._mode = new_mode
            elif text == "":
                message = "empty_input"
            else:
                message = "invalid_key"

            event = LastEvent(
                key=key,
                accepted=accepted,
                previous_mode=previous_mode,
                new_mode=new_mode,
                message=message,
                timestamp=now,
            )
            self._last_event = event
            self._events.put(event)
            return event

    def poll_events(self) -> list[LastEvent]:
        events: list[LastEvent] = []
        while True:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                break
        return events

    def start_input_thread(self, prompt: str = "NavSide> ") -> threading.Thread:
        with self._lock:
            if self._input_thread is not None and self._input_thread.is_alive():
                return self._input_thread
            self._input_prompt = prompt
            self._input_stop.clear()
            thread = threading.Thread(
                target=self._input_loop,
                name="NavModeInputThread",
                daemon=True,
            )
            self._input_thread = thread

        thread.start()
        return thread

    def stop_input_thread(self) -> None:
        self._input_stop.set()
        thread = None
        with self._lock:
            thread = self._input_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=0.2)

    def render_panel(self) -> str:
        with self._lock:
            mode = self._mode
            status = copy.deepcopy(self._status)
            last_event = self._last_event

        decision = _DECISIONS[mode]
        final_cmd = self._format_vec(status.get("final_cmd"))
        policy_cmd = self._format_vec(status.get("policy_cmd", status.get("raw_action")))
        zero_reason = self._format_scalar(status.get("zero_reason"))
        goal_dist = self._format_scalar(status.get("policy_goal_dist", status.get("goal_dist")))
        state_source = self._format_scalar(status.get("state_source"))
        robot_pos_w = status.get("robot_pos_w")
        robot_quat_wxyz = status.get("robot_quat_wxyz")
        fps = self._format_scalar(status.get("fps"))
        obs_lin_vel = self._format_vec(status.get("obs_lin_vel"))
        obs_ang_vel = self._format_vec(status.get("obs_ang_vel"))
        obs_gravity = self._format_vec(status.get("obs_gravity"))
        obs_prev_act = self._format_vec(status.get("obs_prev_act"))
        obs_target = self._format_vec(status.get("obs_target"))
        front_dist = self._format_scalar(status.get("front_dist"))
        valid_pct = self._format_scalar(status.get("valid_pct"))
        depth_mean = self._format_scalar(status.get("depth_mean"))
        depth_std = self._format_scalar(status.get("depth_std"))
        depth_delta = self._format_scalar(status.get("depth_delta"))
        last_event_text = self._format_last_event(last_event)

        lines = [
            "=== NavSide Mode Panel ===",
            f"mode: {mode.value}",
            f"output: {'forced_zero' if decision.force_zero else 'normal'}  |  "
            f"limits: vx_max={decision.vx_max:.3f}  wz_max={decision.wz_max:.3f}",
            f"── Input ─────────────────────────────────────────────",
            f"lin_vel:{obs_lin_vel}  ang_vel:{obs_ang_vel}",
            f"gravity:{obs_gravity}  prev_act:{obs_prev_act}",
            f"target: {obs_target}",
            f"depth:  front={front_dist}m  valid={valid_pct}%  μ={depth_mean}  σ={depth_std}  Δ={depth_delta}",
            f"── Output ────────────────────────────────────────────",
            f"cmd: raw={policy_cmd}  final={final_cmd}  dist={goal_dist}",
            f"zero_reason: {zero_reason}",
            f"── System ────────────────────────────────────────────",
            f"fps: {fps}  source: {state_source}",
            f"pos_w: {self._format_world_pose(robot_pos_w, robot_quat_wxyz)}",
            f"last_event: {last_event_text}",
            "Keys: A->STANDBY  S->LOW  D->MED  F->EMERG  G->STANDBY",
        ]
        return "\033[H\033[J" + "\n".join(lines) + "\n"

    def _input_loop(self) -> None:
        while not self._input_stop.is_set():
            try:
                line = input(self._input_prompt)
            except (EOFError, KeyboardInterrupt, OSError):
                break
            self.apply_line(line)

    @staticmethod
    def _format_scalar(value: Any) -> str:
        if value is None:
            return "n/a"
        if isinstance(value, str):
            return value
        if isinstance(value, (bool, np.bool_)):
            return "true" if bool(value) else "false"
        if isinstance(value, (int, float, np.integer, np.floating)):
            return f"{float(value):.3f}"
        arr = np.asarray(value)
        if arr.size == 0:
            return "n/a"
        if arr.size == 1:
            return f"{float(arr.reshape(-1)[0]):.3f}"
        return np.array2string(arr, precision=3, separator=", ")

    @staticmethod
    def _format_vec(value: Any) -> str:
        if value is None:
            return "n/a"
        arr = np.asarray(value, dtype=np.float32).reshape(-1)
        if arr.size == 0:
            return "n/a"
        return np.array2string(arr, precision=3, separator=", ")

    @staticmethod
    def _format_robot_xy(value: Any) -> str:
        if value is None:
            return "n/a"
        arr = np.asarray(value, dtype=np.float32).reshape(-1)
        if arr.size < 2:
            return "n/a"
        return f"({arr[0]:.3f}, {arr[1]:.3f})"

    @staticmethod
    def _format_last_event(event: LastEvent) -> str:
        return (
            f"key={event.key or 'n/a'} accepted={str(event.accepted).lower()} "
            f"{event.previous_mode.value}->{event.new_mode.value} "
            f"message={event.message} ts={event.timestamp:.3f}"
        )

    @staticmethod
    def _format_world_pose(pos_w: Any, quat_wxyz: Any) -> str:
        """Format world pose as 'x=Y.YY  y=Y.YY  z=Z.ZZ  yaw=θ.θθ'."""
        pos = np.asarray(pos_w, dtype=np.float32).reshape(-1) if pos_w is not None else None
        quat = np.asarray(quat_wxyz, dtype=np.float32).reshape(-1) if quat_wxyz is not None else None

        if pos is None or pos.size < 3:
            return "n/a"

        yaw_str = "n/a"
        if quat is not None and quat.size >= 4:
            w, x, y, z = quat[0], quat[1], quat[2], quat[3]
            siny = 2.0 * (w * z + x * y)
            cosy = 1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(siny, cosy)
            yaw_str = f"{yaw:.3f}"

        return (
            f"x={pos[0]:.3f}  y={pos[1]:.3f}  z={pos[2]:.3f}  "
            f"yaw={yaw_str}rad"
        )


__all__ = [
    "LastEvent",
    "ModeDecision",
    "NavMode",
    "NavModeController",
]
