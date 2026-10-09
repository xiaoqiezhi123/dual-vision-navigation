from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
import yaml

from .bridge import RobotComm
from .csv_logger import CsvTickLogger
from .depth import compute_depth_summary
from .mode import NavMode, NavModeController
from .runtime import NavSideApp, parse_goal
from perception import create_depth_perception


def run(args) -> None:
    """Headless real-robot navigation loop using a configurable depth camera."""
    # --- Init ---
    app = NavSideApp.from_config(args.config)
    app.adapter.reset_recurrent_state()

    # --- Goal ---
    if args.goal:
        goal = parse_goal(args.goal)
    else:
        goal = app.default_goal()
    print(f"[NavSide Real] fixed goal = {goal.tolist()}")

    # --- Depth camera ---
    depth_backend = app.config.depth_backend
    camera = create_depth_perception(
        depth_backend,
        realsense_config_path=app.config.realsense_config_path,
    )
    camera.start()
    print(f"[NavSide Real] depth camera started (backend={depth_backend})")

    viewer = None
    last_viewer_time = 0.0
    if getattr(args, "show_depth", False):
        try:
            from .depth_viewer import DepthViewer

            viewer = DepthViewer()
            print("[NavSide Real] depth viewer opened")
        except Exception as exc:
            print(f"[NavSide Real] depth viewer unavailable: {exc}")

    # --- RobotComm (Foxglove WS + UDP cmd) ---
    deploy_cfg_path = args.deploy_config or str(Path(args.config).parent / "nav_deploy.yaml")
    comm = RobotComm(config_path=deploy_cfg_path)
    comm.daemon = True
    comm.start()
    print(f"[NavSide Real] RobotComm started (config: {deploy_cfg_path})")

    # Handshake zeros
    for _ in range(5):
        comm.send_zero()
        time.sleep(0.1)

    # --- Load map + relocalize + task (blocking, ~10-30s) ---
    print("[NavSide Real] Loading map → relocalizing → loading task ...")
    if comm.load_task():
        print(f"[NavSide Real] Task loaded successfully")
        for i, g in enumerate(comm._task_loader.goals):
            print(f"  goal[{i}]: x={g.get('x',0):.2f} y={g.get('y',0):.2f} "
                  f"theta={g.get('theta',0):.2f} name={g.get('name','')}")
    else:
        print("[NavSide Real] Task loading FAILED — continuing anyway")

    # --- Logging / summary ---
    raw_cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    log_cfg = raw_cfg.get("logging", {})
    summary_hz = float(log_cfg.get("summary_hz", 1.0))
    summary_interval = 1.0 / max(summary_hz, 1e-6)
    csv_logger = CsvTickLogger(
        args.csv_dir or app.config.csv_dir,
        mode="real",
        enabled=app.config.csv_enabled,
    )

    zero_vec = np.zeros(3, dtype=np.float32)
    last_safe_cmd = zero_vec.copy()
    policy_cmd = zero_vec.copy()
    zero_reason: str = "standby"
    goal_dist: float | None = None
    last_output_time = 0.0
    output_interval = 1.0 / max(app.config.dry_run_hz, 1e-6)
    last_print_time = 0.0
    fps_frame_count = 0
    fps_t0 = time.perf_counter()
    fps = 0.0
    prev_raw_action = np.zeros(3, dtype=np.float32)

    # 调度器文件通道（任务导航调度器）：
    #   NAVSIDE_SCHED_FILE = [SCHED] 状态行写入该文件（调度器读取，不刷终端面板）
    #   NAVSIDE_CMD_FILE  = 调度器指令文件（mode_controller 低优先级应用；
    #                       交互终端按键永远是最高优先级，随时可停车）。
    # 面板照常打印到终端（调度器模式下 NavSide 跑在独立交互终端窗口里）。
    sched_file = os.environ.get("NAVSIDE_SCHED_FILE", "") or None
    cmd_file = os.environ.get("NAVSIDE_CMD_FILE", "") or None

    mode_controller = NavModeController()
    mode_controller.start_input_thread(prompt="NavSide> ")
    if cmd_file:
        mode_controller.start_sched_input(cmd_file)
    sched_last_sig = None
    sched_last_print = 0.0

    def _zero_burst(count: int = 3) -> None:
        for _ in range(max(int(count), 0)):
            comm.send_zero()

    def _emit_sched_status() -> None:
        """给任务导航调度器的状态行：状态变化即时输出，否则按 summary 节流。

        行格式：``[SCHED] mode=.. run_policy=.. zero_reason=.. goal_dist=.. final_cmd=..``
        调度器据此驱动分段导航状态机（如 zero_reason=goal_reached → 到点停车）。
        """
        nonlocal sched_last_sig, sched_last_print
        if not sched_file:
            return
        # goal_dist 四舍五入进签名，避免逐 tick 的浮点微变导致每帧都打状态行。
        sig = (
            decision.mode.value,
            zero_reason,
            None if goal_dist is None else round(goal_dist, 2),
        )
        now = time.monotonic()
        if sig == sched_last_sig and now - sched_last_print < summary_interval:
            return
        sched_last_sig = sig
        sched_last_print = now
        line = "[SCHED] mode={} run_policy={} zero_reason={} goal_dist={} final_cmd={}".format(
            decision.mode.value,
            int(decision.run_policy),
            zero_reason,
            "None" if goal_dist is None else f"{goal_dist:.4f}",
            np.array2string(last_safe_cmd, precision=4),
        )
        try:
            with open(sched_file, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except OSError:
            pass

    # --- Main loop ---
    try:
        while True:
            now = time.time()

            # -- 调度器 quit 指令（经 NAVSIDE_CMD_FILE 下达）→ 优雅退出 --
            if mode_controller.quit_requested:
                print("[NavSide Real] quit requested by scheduler")
                break

            # -- Mode events --
            for event in mode_controller.poll_events():
                if not event.accepted:
                    continue
                if event.key in ("A", "G"):
                    app.adapter.reset_recurrent_state()
                    last_safe_cmd = zero_vec.copy()
                    policy_cmd = zero_vec.copy()
                    goal_dist = None
                    zero_reason = "quit_to_standby" if event.key == "G" else "standby"
                    _zero_burst(3)
                    last_output_time = now
                elif event.key == "F":
                    last_safe_cmd = zero_vec.copy()
                    policy_cmd = zero_vec.copy()
                    zero_reason = "emergency"
                    _zero_burst(3)
                    last_output_time = now

            decision = mode_controller.get_decision()

            # -- 调度器目标点更新（stdin "goal x y z"，Z-up 世界系）--
            sched_goal = mode_controller.get_goal()
            if sched_goal is not None and not np.array_equal(sched_goal, goal):
                goal = sched_goal
                print(f"[NavSide Real] goal updated by scheduler: {goal.tolist()}")

            depth_output = None
            if viewer is not None:
                if not viewer.is_open():
                    viewer.close()
                    viewer = None
                else:
                    now_viewer = time.time()
                    if now_viewer - last_viewer_time >= 1.0 / 15.0:
                        depth_output = camera.read()
                        last_viewer_time = now_viewer
                        try:
                            if depth_output.success:
                                viewer_front, viewer_valid = compute_depth_summary(
                                    depth_output.depth_input
                                )
                                viewer.update(
                                    depth_output.depth_input,
                                    crop_width=app.adapter.depth_crop_width,
                                    crop_height=app.adapter.depth_crop_height,
                                    front_m=viewer_front,
                                    valid_pct=viewer_valid,
                                    backend=depth_backend,
                                )
                            else:
                                viewer.update_error(depth_output.error)
                        except Exception as exc:
                            print(f"[NavSide Real] depth viewer update failed: {exc}")
                            viewer.close()
                            viewer = None

            # -- Inactive mode: just send zeros --
            if not decision.run_policy:
                last_safe_cmd = zero_vec.copy()
                policy_cmd = zero_vec.copy()
                if zero_reason not in ("quit_to_standby", "emergency"):
                    zero_reason = "standby"
                if now - last_output_time >= output_interval:
                    comm.send_zero()
                    last_output_time = now
                _emit_sched_status()
                time.sleep(0.01)
                continue

            # -- Rate limiting --
            if not app.adapter.should_tick(now):
                time.sleep(0.001)
                continue

            # -- Get depth image --
            if depth_output is None:
                depth_output = camera.read()
            if not depth_output.success:
                print(f"[NavSide Real] depth read failed: {depth_output.error}")
                if now - last_output_time >= output_interval:
                    comm.send_zero()
                    last_output_time = now
                continue

            # -- Get robot state --
            state_packet = comm.get_latest_state()
            if state_packet is None:
                state = app.default_state()
            else:
                state = state_packet.to_sru_robot_state()
                state.projected_gravity_b = np.array([0.0, 0.0, -1.0], dtype=np.float32)

            # -- Run policy --
            prev_raw_action = app.adapter.last_action.copy()
            result = app.step(
                depth_img=depth_output.depth_input,
                state=state,
                target_pos_w=goal,
                timestamp=now,
                vx_max=decision.vx_max,
                wz_max=decision.wz_max,
                print_control=False,
            )
            if result is None:
                continue

            control_info = result["control"]
            diag = result["diag"]
            policy_cmd = np.asarray(control_info["raw_cmd"], dtype=np.float32).copy()
            last_safe_cmd = np.asarray(control_info["final_cmd"], dtype=np.float32).copy()
            zero_reason = control_info["zero_reason"]
            goal_dist = float(result["goal_dist"])
            state_source = f"real_{depth_backend}"
            current_raw_action = diag["raw_action"]
            prev_raw_action = current_raw_action.copy()
            # -- Depth physical stats --
            front_dist, valid_pct = compute_depth_summary(depth_output.depth_input)
            # -- FPS --
            fps_frame_count += 1
            fps_elapsed = time.perf_counter() - fps_t0
            if fps_elapsed >= 1.0:
                fps = fps_frame_count / fps_elapsed
                fps_frame_count = 0
                fps_t0 = time.perf_counter()

            if decision.mode == NavMode.EMERGENCY:
                last_safe_cmd = zero_vec.copy()
                zero_reason = "emergency"

            if not np.all(np.isfinite(last_safe_cmd)):
                last_safe_cmd = zero_vec.copy()
                zero_reason = "command_nan_or_inf"

            # -- Send command --
            comm.send_command(last_safe_cmd[0], last_safe_cmd[1], last_safe_cmd[2])
            last_output_time = now
            csv_logger.write_tick(
                timestamp=now,
                control_mode=decision.mode.value,
                lin_vel=diag["linear_vel_b"],
                ang_vel=diag["angular_vel_b"],
                gravity=diag["projected_gravity_b"],
                target_obs=diag["target_position"],
                depth_front_m=front_dist,
                depth_valid_pct=valid_pct,
                raw_cmd=policy_cmd,
                final_cmd=last_safe_cmd,
                goal_dist=goal_dist,
                zero_reason=zero_reason,
            )

            _emit_sched_status()

            # -- Update mode panel status --
            robot_xy = np.asarray([state.robot_pos_w[0], state.robot_pos_w[1]], dtype=np.float32)
            mode_controller.update_status(
                final_cmd=last_safe_cmd,
                policy_cmd=policy_cmd,
                goal_dist=goal_dist,
                zero_reason=zero_reason,
                state_source=state_source,
                robot_xy=robot_xy,
                fps=fps,
                obs_lin_vel=state.linear_vel_b,
                obs_ang_vel=state.angular_vel_b,
                obs_gravity=state.projected_gravity_b,
                obs_prev_act=prev_raw_action,
                obs_target=diag["target_position"],
                robot_pos_w=state.robot_pos_w,
                robot_quat_wxyz=state.robot_quat_wxyz,
                front_dist=front_dist,
                valid_pct=valid_pct,
                depth_mean=diag.get("depth_feature_mean", np.array([0.0]))[0],
                depth_std=diag.get("depth_feature_std", np.array([0.0]))[0],
                depth_delta=diag.get("depth_feature_delta", np.array([0.0]))[0],
            )

            # -- Print summary panel (overwrites, no scroll) --
            # 调度器模式下也照常打印：NavSide 跑在独立交互终端窗口，用户靠
            # 这个面板上的按键（A/S/D/F/G）随时接管/停车。
            if now - last_print_time >= summary_interval:
                print(mode_controller.render_panel(), end="", flush=True)
                last_print_time = now

    except KeyboardInterrupt:
        print("[NavSide Real] interrupted by user.")
    finally:
        csv_logger.close()
        try:
            mode_controller.stop_input_thread()
        except Exception:
            pass
        try:
            mode_controller.stop_sched_input()
        except Exception:
            pass
        if sched_file:
            # 通知调度器进程退出（关停流程据此确认 NavSide 已收尾）。
            try:
                with open(sched_file, "a", encoding="utf-8") as f:
                    f.write("[SCHED] process=exit\n")
            except OSError:
                pass
        try:
            comm.send_zero()
        except Exception:
            pass
        comm.stop()
        if viewer is not None:
            viewer.close()
        camera.close()
        print("[NavSide Real] shutdown complete.")
