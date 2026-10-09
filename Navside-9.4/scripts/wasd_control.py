#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘 WASD 控制机器人（Phybot C2）—— 纯 UDP 速度命令，绕过 SRU 推理。

读物理键盘 /dev/input/eventX（evdev input_event），把 WASD 映射为速度命令，
通过 UDP 发送到机器人端。机器人端 RL_deploy_cpg（PhybotSoftware_c2）监听
UDP 8080，按 `struct CommandPacket { float vx; float vy; float omega_z; }`
解析速度命令 —— 与本脚本打包格式 `struct.pack("3f", vx, vy, wz)` 完全一致。

速度映射（对齐 PhybotSoftware_c2/Joystick 的速度逻辑）:
    W      = 前进  vx      = +0.5
    S      = 后退  vx      = -0.3
    A      = 左转  omega_z = +0.4
    D      = 右转  omega_z = -0.4
    Q      = 左横移 vy      = +0.1（可选）
    E      = 右横移 vy      = -0.1（可选）
    松开所有键 = 停车（持续发 0 速度）

方向约定：若实测 A/D（或 W/S）反了，用 --invert-yaw（或 --invert-x）翻转。

权限：读 /dev/input/eventX 需要 input 组权限或 root。当前用户无权限时，
脚本会给出明确提示。解决方法任选其一：
    sudo chmod 666 /dev/input/event*            # 临时，重启失效
    sudo usermod -aG input $USER && 重新登录     # 持久
    sudo python3 wasd_control.py ...            # 用 root 运行

用法:
    python3 wasd_control.py                      # 自动发现键盘 + 读 config/nav_deploy.yaml
    python3 wasd_control.py --device /dev/input/event5
    python3 wasd_control.py --host 127.0.0.1 --port 8080
    python3 wasd_control.py --speed 0.5          # 半速（速度乘 0.5）
    python3 wasd_control.py --list               # 只列出键盘设备后退出

安全：Ctrl+C 或 Esc 退出前强制发送 0 速度停车；键盘读错误也会停车。
"""

from __future__ import annotations

import argparse
import fcntl
import os
import select
import signal
import socket
import struct
import sys
import threading
import time

try:
    import yaml
except ImportError:  # 允许在没有 yaml 时仍可用（走 --host/--port 或默认值）
    yaml = None

# ---------------------------------------------------------------------------
# 可调参数（与 PhybotSoftware_c2/Joystick 一致）
# ---------------------------------------------------------------------------
MAXSPEED_X = 0.5     # W 前进速度 (m/s)
MINSPEED_X = -0.3    # S 后退速度 (m/s)
MAXSPEED_YAW = 0.4   # A 左转角速度 (rad/s)
MINSPEED_YAW = -0.4  # D 右转角速度 (rad/s)
MAXSPEED_Y = 0.1     # Q 左横移速度 (m/s)
MINSPEED_Y = -0.1    # E 右横移速度 (m/s)

SEND_HZ = 50.0       # UDP 发送频率（Hz），持续发送保证命令新鲜
RAMP_STEP = 0.02     # 阶梯增长步长：每周期速度最大变化量（m/s 或 rad/s），避免起步冲击

# ---------------------------------------------------------------------------
# Linux input 常量（标准 keycode）
# ---------------------------------------------------------------------------
EV_KEY = 0x01
KEY_ESC = 1
KEY_Q = 16
KEY_W = 17
KEY_E = 18
KEY_A = 30
KEY_S = 31
KEY_D = 32
KEY_SPACE = 57
KEY_MAX = 0x2FF  # 767

# input_event 结构（64 位 Linux，无 padding，共 24 字节）
#   struct input_event { struct timeval time; __u16 type; __u16 code; __s32 value; }
EVENT_FMT = "llHHi"       # tv_sec(8) + tv_usec(8) + type(2) + code(2) + value(4)
EVENT_SIZE = struct.calcsize(EVENT_FMT)


def _eviocgbit(ev: int, size: int) -> int:
    """EVIOCGBIT(ev,len) = _IOC(_IOC_READ=2, 'E'=0x45, 0x20+ev, len)。"""
    return (2 << 30) | (0x45 << 8) | ((0x20 + ev) & 0xFF) | ((size & 0x3FFF) << 16)


def _has_key_fd(fd: int, keycode: int) -> bool:
    """用 ioctl EVIOCGBIT(EV_KEY) 检查已打开的 fd 是否支持某按键。"""
    nbytes = KEY_MAX // 8 + 1
    buf = bytearray(nbytes)
    try:
        fcntl.ioctl(fd, _eviocgbit(EV_KEY, nbytes), buf, True)
    except OSError:
        return False
    return bool(buf[keycode // 8] & (1 << (keycode % 8)))


def _device_name(path: str) -> str:
    dev = os.path.basename(path)
    try:
        with open(f"/sys/class/input/{dev}/device/name", "r", encoding="utf-8") as f:
            return f.read().strip()
    except OSError:
        return ""


def _event_paths() -> list[str]:
    if not os.path.isdir("/dev/input"):
        return []
    return sorted(f"/dev/input/{n}" for n in os.listdir("/dev/input") if n.startswith("event"))


def _permission_hint() -> str:
    return (
        "读 /dev/input/eventX 需要 input 组权限或 root。解决方法任选其一:\n"
        "  1) 临时:   sudo chmod 666 /dev/input/event*\n"
        "  2) 持久:   sudo usermod -aG input $USER   然后重新登录\n"
        "  3) 直接:   sudo python3 wasd_control.py ..."
    )


def _check_readable() -> tuple[int, int]:
    """返回 (可读的 event 设备数, event 设备总数)，用于诊断权限。"""
    paths = _event_paths()
    readable = 0
    for p in paths:
        try:
            fd = os.open(p, os.O_RDONLY | os.O_NONBLOCK)
            os.close(fd)
            readable += 1
        except OSError:
            pass
    return readable, len(paths)


def find_keyboard(device: str | None = None) -> str:
    """返回第一个支持 KEY_W 的键盘设备路径；找不到抛 RuntimeError。"""
    if device:
        try:
            fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError:
            raise RuntimeError(f"设备 {device} 无读取权限。\n{_permission_hint()}")
        except OSError as exc:
            raise RuntimeError(f"无法打开 {device}: {exc}")
        try:
            if _has_key_fd(fd, KEY_W):
                return device
        finally:
            os.close(fd)
        raise RuntimeError(f"设备 {device} 不支持 KEY_W（不是键盘）")

    paths = _event_paths()
    if not paths:
        raise RuntimeError("无 /dev/input/event* 设备，请插好键盘")

    for path in paths:
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            continue  # 无权限的跳过，最后统一诊断
        try:
            if _has_key_fd(fd, KEY_W):
                return path
        finally:
            os.close(fd)

    # 找不到键盘 —— 诊断权限
    readable, total = _check_readable()
    if readable == 0 and total > 0:
        raise RuntimeError(f"发现 {total} 个 input 设备，但全部无读取权限（当前用户不在 input 组）。\n{_permission_hint()}")
    raise RuntimeError("未找到键盘设备（/dev/input/event* 中无 KEY_W）。请插好键盘或用 --device 指定")


def list_keyboards() -> None:
    print("input 设备列表（含是否支持 W/A/S/D）:")
    paths = _event_paths()
    if not paths:
        print("  （无 /dev/input/event* 设备）")
        return
    shown = 0
    for path in paths:
        dname = _device_name(path)
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError:
            print(f"  {path}  [无权限]  {dname}")
            continue
        except OSError:
            continue
        try:
            w = _has_key_fd(fd, KEY_W)
            a = _has_key_fd(fd, KEY_A)
        finally:
            os.close(fd)
        shown += 1
        mark = "  <== 键盘" if (w and a) else ""
        print(f"  {path}  W={int(w)} A={int(a)}  {dname}{mark}")

    readable, total = _check_readable()
    if readable == 0 and total > 0:
        print("")
        print("⚠ 所有 input 设备均无读取权限。请先解决权限后重试：")
        print(_permission_hint())


# ---------------------------------------------------------------------------
# 键盘读取线程
# ---------------------------------------------------------------------------
class KeyboardReader(threading.Thread):
    def __init__(self, device: str):
        super().__init__(daemon=True, name="KeyboardReader")
        self.device = device
        self._lock = threading.Lock()
        self._pressed: set[int] = set()
        self._error = False
        self._stop = threading.Event()

    def run(self) -> None:
        try:
            fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            with self._lock:
                self._error = True
            print(f"[键盘] 无法打开 {self.device}: {exc}", file=sys.stderr)
            return

        try:
            while not self._stop.is_set():
                r, _, _ = select.select([fd], [], [], 0.02)
                if not r:
                    continue
                try:
                    data = os.read(fd, EVENT_SIZE)
                except OSError:
                    continue
                if len(data) < EVENT_SIZE:
                    break
                _, _, etype, ecode, evalue = struct.unpack(EVENT_FMT, data)
                if etype != EV_KEY:
                    continue
                if ecode not in (KEY_W, KEY_S, KEY_A, KEY_D, KEY_Q, KEY_E, KEY_SPACE, KEY_ESC):
                    continue
                with self._lock:
                    if evalue:  # 1=按下 2=自动重复；非 0 视为按住
                        self._pressed.add(ecode)
                    else:  # 0=松开
                        self._pressed.discard(ecode)
        except Exception as exc:
            with self._lock:
                self._error = True
            print(f"[键盘] 读取错误: {exc}", file=sys.stderr)
        finally:
            try:
                os.close(fd)
            except OSError:
                pass

    def pressed(self) -> set[int]:
        with self._lock:
            return set(self._pressed)

    def error(self) -> bool:
        with self._lock:
            return self._error

    def stop(self) -> None:
        self._stop.set()


# ---------------------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------------------
def _ramp(cur: float, target: float, step: float) -> float:
    """阶梯增长：当前速度向目标速度逼近，每步最多变化 step（正值）。"""
    if target > cur:
        return min(cur + step, target)
    if target < cur:
        return max(cur - step, target)
    return cur


def load_remote_from_config(config_path: str | None):
    """从 nav_deploy.yaml 读 cmd_remote_ip/cmd_remote_port。失败返回 None。"""
    if config_path is None or yaml is None:
        return None
    if not os.path.isfile(config_path):
        return None
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        ip = cfg.get("cmd_remote_ip")
        port = cfg.get("cmd_remote_port")
        if ip is None or port is None:
            return None
        return str(ip), int(port)
    except Exception:
        return None


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="键盘 WASD 控制机器人（UDP 速度命令，绕过 SRU）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--device", default=None, help="键盘设备路径，如 /dev/input/event5（默认自动发现）")
    ap.add_argument("--host", default=None, help="UDP 目标 IP（默认读 config/nav_deploy.yaml，再默认 127.0.0.1）")
    ap.add_argument("--port", type=int, default=None, help="UDP 目标端口（默认读 config/nav_deploy.yaml，再默认 8080）")
    ap.add_argument("--config", default=None, help="nav_deploy.yaml 路径（默认 <脚本目录>/../config/nav_deploy.yaml）")
    ap.add_argument("--speed", type=float, default=1.0, help="速度系数 0.0~1.0（默认 1.0）")
    ap.add_argument("--invert-x", action="store_true", help="翻转 W/S 前进后退方向")
    ap.add_argument("--invert-yaw", action="store_true", help="翻转 A/D 左右转方向")
    ap.add_argument("--hz", type=float, default=SEND_HZ, help="UDP 发送频率 Hz（默认 50）")
    ap.add_argument("--ramp-step", type=float, default=RAMP_STEP,
                    help="阶梯增长步长：每周期速度最大变化量，越小起步越缓（默认 0.02）")
    ap.add_argument("--list", action="store_true", help="只列出 input 设备后退出")
    return ap


def main() -> None:
    args = build_parser().parse_args()

    if args.list:
        list_keyboards()
        return

    # --- 发现键盘 ---
    device = find_keyboard(args.device)
    print(f"[键盘] {device}  {_device_name(device)}")

    # --- UDP 目标 ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_config = os.path.join(script_dir, "..", "config", "nav_deploy.yaml")
    config_path = args.config or default_config
    host, port = args.host, args.port
    if host is None or port is None:
        from_cfg = load_remote_from_config(config_path)
        if from_cfg is not None:
            cfg_ip, cfg_port = from_cfg
            host = host or cfg_ip
            port = port or cfg_port
            print(f"[配置] 从 {config_path} 读取目标 {cfg_ip}:{cfg_port}")
        else:
            print(f"[配置] 未找到 {config_path}，使用默认目标")
    host = host or "127.0.0.1"
    port = port if port is not None else 8080

    speed = max(0.0, min(1.0, args.speed))
    sign_x = -1.0 if args.invert_x else 1.0
    sign_yaw = -1.0 if args.invert_yaw else 1.0
    hz = max(1.0, min(500.0, args.hz))
    interval = 1.0 / hz
    ramp_step = max(0.0, args.ramp_step)

    # --- UDP socket ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    remote = (host, port)
    print(f"[UDP] 发送速度命令到 {host}:{port}（3×float32: vx,vy,omega_z）")

    # --- 键盘读取线程 ---
    reader = KeyboardReader(device)
    reader.start()

    def _send(vx: float, vy: float, wz: float) -> None:
        sock.sendto(struct.pack("3f", float(vx), float(vy), float(wz)), remote)

    def _stop_robot() -> None:
        try:
            _send(0.0, 0.0, 0.0)
        except OSError:
            pass

    running = True

    def _on_exit(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _on_exit)
    signal.signal(signal.SIGTERM, _on_exit)

    print("[按键] W前进 S后退 A左转 D右转  Q/E横移  松开=停车  Esc/Ctrl+C=退出")

    cur_vx = cur_vy = cur_wz = 0.0
    last_vx = last_vy = last_wz = 0.0
    try:
        while running:
            pressed = reader.pressed()
            if reader.error():
                print("[键盘] 键盘读取失败，停车退出", file=sys.stderr)
                break

            if KEY_ESC in pressed:
                print("[键盘] 收到 Esc，退出")
                break

            # --- 目标速度映射（按键） ---
            target_vx = 0.0
            if KEY_W in pressed:
                target_vx = MAXSPEED_X
            elif KEY_S in pressed:
                target_vx = MINSPEED_X

            target_vy = 0.0
            if KEY_Q in pressed:
                target_vy = MAXSPEED_Y
            elif KEY_E in pressed:
                target_vy = MINSPEED_Y

            target_wz = 0.0
            if KEY_A in pressed:
                target_wz = MAXSPEED_YAW
            elif KEY_D in pressed:
                target_wz = MINSPEED_YAW

            target_vx *= speed * sign_x
            target_vy *= speed
            target_wz *= speed * sign_yaw

            # --- 阶梯增长：当前速度向目标逐步逼近，避免起步/急停冲击 ---
            cur_vx = _ramp(cur_vx, target_vx, ramp_step)
            cur_vy = _ramp(cur_vy, target_vy, ramp_step)
            cur_wz = _ramp(cur_wz, target_wz, ramp_step)

            if (cur_vx, cur_vy, cur_wz) != (last_vx, last_vy, last_wz):
                print(f"\r[vx={cur_vx:+.2f} vy={cur_vy:+.2f} wz={cur_wz:+.2f}]", end="", flush=True)
                last_vx, last_vy, last_wz = cur_vx, cur_vy, cur_wz

            _send(cur_vx, cur_vy, cur_wz)
            time.sleep(interval)

    finally:
        reader.stop()
        print("\n[退出] 发送 0 速度停车 ...")
        for _ in range(5):
            _stop_robot()
            time.sleep(0.02)
        sock.close()
        print("[退出] 完成")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[退出] 已停止")
    except RuntimeError as exc:
        print(f"\n[错误] {exc}", file=sys.stderr)
        sys.exit(1)
