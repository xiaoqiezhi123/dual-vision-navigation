from __future__ import annotations

import copy
import math
import os
import queue
import sys
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
        self._goal: Optional[np.ndarray] = None
        # 用户终端按键接管标志：交互按键（A/S/D/F/G）置位；调度器的行进指令
        # （S/D）在置位期间被忽略，只有调度器停车指令（A）能清除。
        # 交互终端永远是最高优先级——随时可以停车。
        self._user_override = False
        # 调度器 quit 指令：主循环轮询后优雅退出（finally 发零）。
        self._quit_requested = False
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
        self._sched_stop = threading.Event()
        self._sched_thread: Optional[threading.Thread] = None
        self._input_prompt = "NavSide> "

    def get_mode(self) -> NavMode:
        with self._lock:
            return self._mode

    def get_goal(self) -> Optional[np.ndarray]:
        """返回调度器设定的目标点（Z-up 世界系）；未设定时返回 None。

        由输入线程经 ``apply_line("goal x y z")`` / ``apply_sched_line`` 更新
        （线程安全），主循环每 tick 读取，用于任务导航调度器切换各段目标。
        """
        with self._lock:
            return None if self._goal is None else self._goal.copy()

    @property
    def quit_requested(self) -> bool:
        """调度器 quit 指令是否已下达（主循环轮询，优雅退出）。"""
        with self._lock:
            return self._quit_requested

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

            if text.lower().startswith("goal"):
                # 调度器指令："goal x y z"（Z-up 世界系，z 取机器人高度）。
                # 必须放在单键判定之前：'g' 开头的 "goal" 会撞上 G 键（STANDBY）。
                parts = text.split()
                if len(parts) == 4:
                    try:
                        self._goal = np.asarray(
                            [float(parts[1]), float(parts[2]), float(parts[3])],
                            dtype=np.float32,
                        )
                        accepted = True
                        message = f"goal_set={np.array2string(self._goal, precision=3)}"
                    except ValueError:
                        message = "invalid_goal"
                else:
                    message = "invalid_goal"
            elif key in _KEY_TO_MODE:
                new_mode = _KEY_TO_MODE[key]
                accepted = True
                # 交互终端按键 = 用户接管控制（最高优先级）：此后调度器的行进
                # 指令（S/D）被忽略，直到调度器下发停车指令（A）交还控制权。
                self._user_override = True
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

    def apply_sched_line(self, line: str) -> LastEvent:
        """调度器经指令文件下达的命令（低优先级通道）。

        优先级规则（交互终端按键永远是最高优先级，随时可停车）：
        - ``goal x y z`` 总是生效（只更新目标，不动模式、不碰接管标志）；
        - ``A``（停车）总是生效，并清除用户接管标志（交还调度器控制权）；
        - ``S``/``D``（行进）仅在用户没有手动接管时生效；
        - ``F`` 保守起见直接应用（急停，调度器正常不会发）；
        - ``quit`` 置退出标志（主循环轮询后优雅退出）。
        """
        text = "" if line is None else str(line).strip()
        key = text[:1].upper() if text else ""
        now = time.time()

        with self._lock:
            previous_mode = self._mode
            new_mode = previous_mode
            accepted = False
            message = "ignored"

            if text.lower().startswith("goal"):
                parts = text.split()
                if len(parts) == 4:
                    try:
                        self._goal = np.asarray(
                            [float(parts[1]), float(parts[2]), float(parts[3])],
                            dtype=np.float32,
                        )
                        accepted = True
                        message = f"goal_set={np.array2string(self._goal, precision=3)}"
                    except ValueError:
                        message = "invalid_goal"
                else:
                    message = "invalid_goal"
            elif key == "A":
                new_mode = NavMode.STANDBY
                accepted = True
                message = "standby"
                self._user_override = False
                self._mode = new_mode  # 必须真正改模式（曾遗漏：事件里改了、实际模式没变）
            elif key in ("S", "D"):
                if self._user_override:
                    message = "ignored_user_override"
                else:
                    new_mode = _KEY_TO_MODE[key]
                    accepted = True
                    message = "low_speed" if key == "S" else "medium_speed"
                    self._mode = new_mode
            elif key == "F":
                new_mode = NavMode.EMERGENCY
                accepted = True
                message = "emergency"
                self._mode = new_mode
            elif text.lower() in ("quit", "q", "exit"):
                self._quit_requested = True
                accepted = True
                message = "quit_requested"
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
        # 关闭 stdin 让阻塞在 input() 的线程立即退出：否则进程收尾时
        # daemon 输入线程仍持有 stdin 缓冲锁，触发
        # "Fatal Python error: _enter_buffered_busy" 并可能挂住解释器退出。
        try:
            os.close(sys.stdin.fileno())
        except (OSError, ValueError):
            pass
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def start_sched_input(self, path: str) -> threading.Thread:
        """监听调度器指令文件（追加式）：新行 → apply_sched_line（低优先级）。"""
        with self._lock:
            if self._sched_thread is not None and self._sched_thread.is_alive():
                return self._sched_thread
            self._sched_stop.clear()
            thread = threading.Thread(
                target=self._sched_loop,
                args=(path,),
                name="NavSchedInputThread",
                daemon=True,
            )
            self._sched_thread = thread

        thread.start()
        return thread

    def stop_sched_input(self) -> None:
        self._sched_stop.set()
        thread = None
        with self._lock:
            thread = self._sched_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=0.5)

    def _sched_loop(self, path: str) -> None:
        """tail 指令文件：偏移量持续前移，读到一行应用一行（EOF 后轮询）。"""
        offset = 0
        while not self._sched_stop.is_set():
            try:
                with open(path, "r", encoding="utf-8") as f:
                    f.seek(offset)
                    while not self._sched_stop.is_set():
                        line = f.readline()
                        if line:
                            offset = f.tell()
                            self.apply_sched_line(line)
                        else:
                            time.sleep(0.05)
            except OSError:
                if self._sched_stop.is_set():
                    break
                time.sleep(0.2)

    def render_panel(self) -> str:
        with self._lock:
            mode = self._mode
            status = copy.deepcopy(self._status)
            last_event = self._last_event
            # 当前导航目标（调度器每段下发 goal，随任务点推进更新；未设置时 n/a）
            goal = None if self._goal is None else self._goal.copy()

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
        goal_w = self._format_vec(goal)
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
            f"goal_w: {goal_w}  ← 当前任务点目标（随任务点推进更新）",
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
            except (EOFError, KeyboardInterrupt, OSError, ValueError):
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
