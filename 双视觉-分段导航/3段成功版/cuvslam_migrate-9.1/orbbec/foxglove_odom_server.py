# Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA software released under the NVIDIA Community License is intended to be used to enable
# the further development of AI and robotics technologies. Such software has been designed, tested,
# and optimized for use with NVIDIA hardware, and this License grants permission to use the software
# solely with such hardware.
# Subject to the terms of this License, NVIDIA confirms that you are free to commercially use,
# modify, and distribute the software with NVIDIA hardware. NVIDIA does not claim ownership of any
# outputs generated using the software or derivative works thereof. Any code contributions that you
# share with NVIDIA are licensed to NVIDIA as feedback under this License and may be incorporated
# in future releases without notice or attribution.
# By using, reproducing, modifying, distributing, performing, or displaying any portion or element
# of the software or derivative works thereof, you agree to be bound by this License.

"""Foxglove WebSocket 服务端：把 cuVSLAM 全局位姿发布为 nav_msgs/Odometry。

下游 bridge.py（/home/amov/Desktop/bridge.py）通过 Foxglove WebSocket 客户端
（foxglove_ws_client.py）订阅 odom 话题，只读取 position + orientation（twist 被忽略，
速度由下游自己用位姿差分计算）。本模块扮演「LiDAR 导航系统」原来那个服务端角色，
让 cuVSLAM 无缝替换 LiDAR，下游 bridge.py 无需任何改动（只需把 websocket_uri 指到本机）。

坐标系：
  cuVSLAM 世界系 = OpenCV（+X 右、+Y 下、+Z 前，重力 ≈ +Y）。
  下游世界系 = Z-up / REP-103（+X 前、+Y 左、+Z 上，重力 = (0,0,-9.81)，bridge.py 硬编码）。
  发布前把「相机位姿（OpenCV 世界）」变换成「机体位姿 base_link（Z-up 世界）」。
  因为 OpenCV 约定与 REP-103 约定之间的 120° 旋转，恰好等于「相机系→机体系」的安装旋转，
  所以一次共轭同时完成两件事。若相机非正向朝前安装，请改 _OPENCV_TO_BODY_R。

传输协议（Foxglove WebSocket v1，子协议 foxglove.websocket.v1）：
  连接后服务端发 serverInfo → advertise → （收 subscribe）→ 二进制 MESSAGE_DATA。
  二进制帧载荷：1 字节 opcode(0x01) + 4 字节小端 sub_id + 8 字节小端时间戳(ns) + ROS1 载荷。
  ROS1 载荷为 nav_msgs/Odometry（小端、无对齐填充），与下游反序列化一一对应。

仅用标准库（socket + RFC 6455 帧）+ numpy 实现，无第三方依赖。
"""

import base64
import hashlib
import json
import socket
import struct
import threading
import time

import numpy as np

_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

# OpenCV（相机系）→ 机体系 base_link（REP-103）的固定旋转矩阵：
#   x_body =  z_cam（前）
#   y_body = -x_cam（右）
#   z_body = -y_cam（下）
_OPENCV_TO_BODY_R = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64
)


# ---------------------------------------------------------------------------
# 位姿变换 OpenCV（相机系）→ Z-up（机体系）
# ---------------------------------------------------------------------------

def quaternion_xyzw_to_matrix(q) -> np.ndarray:
    """单位四元数 (x, y, z, w) → 旋转矩阵（Hamilton 约定，v' = R v）。"""
    x, y, z, w = (float(v) for v in q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → 单位四元数 (x, y, z, w)。"""
    R = np.asarray(R, dtype=np.float64)
    trace = float(np.trace(R))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def opencv_pose_to_zup(position_xyz, quaternion_xyzw):
    """cuVSLAM 相机位姿（OpenCV 世界）→ 下游机体位姿（Z-up 世界）。

    Args:
        position_xyz: (3,) 相机在 OpenCV 世界下的位置 (x右, y下, z前)。
        quaternion_xyzw: (4,) 相机朝向 (x, y, z, w)。

    Returns:
        (position_zup (3,), quaternion_zup_xyzw (4,))
    """
    p = np.asarray(position_xyz, dtype=np.float64).reshape(3)
    R_cw = quaternion_xyzw_to_matrix(quaternion_xyzw)
    # 位置：向量随世界系约定变化，直接左乘（共轭只作用于旋转）。
    p_zup = _OPENCV_TO_BODY_R @ p
    # 朝向：同时改变世界系约定与机体系约定 → 旋转矩阵做共轭 R_BC · R_cw · R_BC^T。
    R_bw_z = _OPENCV_TO_BODY_R @ R_cw @ _OPENCV_TO_BODY_R.T
    q_zup = matrix_to_quaternion_xyzw(R_bw_z)
    return p_zup, q_zup


# ---------------------------------------------------------------------------
# ROS1 nav_msgs/Odometry 二进制序列化
# ---------------------------------------------------------------------------

def serialize_odometry(position_xyz, quaternion_xyzw, seq: int = 0, timestamp_sec=None) -> bytes:
    """nav_msgs/Odometry → ROS1 二进制（小端、无对齐填充）。

    布局与下游 foxglove_ws_client.deserialize_odometry() 一一对应：
      Header(uint32 seq, uint32 secs, uint32 nsecs, string frame_id)
      string child_frame_id
      Point(float64 x,y,z) + Quaternion(float64 x,y,z,w) + float64[36] 协方差
      Twist(Vector3 linear + Vector3 angular) + float64[36] 协方差
    """
    if timestamp_sec is None:
        timestamp_sec = time.time()
    secs = int(timestamp_sec)
    nsecs = int((timestamp_sec - secs) * 1e9)

    frame_id = b"odom"
    child_frame_id = b"base_link"
    zero_cov = struct.pack("<36d", *([0.0] * 36))  # 288 字节零协方差

    buf = bytearray()
    buf += struct.pack("<III", seq & 0xFFFFFFFF, secs, nsecs)
    buf += struct.pack("<I", len(frame_id)) + frame_id
    buf += struct.pack("<I", len(child_frame_id)) + child_frame_id
    buf += struct.pack("<3d", *[float(v) for v in position_xyz])
    buf += struct.pack("<4d", *[float(v) for v in quaternion_xyzw])
    buf += zero_cov
    buf += struct.pack("<3d", 0.0, 0.0, 0.0)  # twist.linear（下游自己差分算速度）
    buf += struct.pack("<3d", 0.0, 0.0, 0.0)  # twist.angular
    buf += zero_cov
    return bytes(buf)


# ---------------------------------------------------------------------------
# Foxglove WebSocket 服务端（stdlib 实现）
# ---------------------------------------------------------------------------

class _ClientConnection:
    """单个下游客户端的 WebSocket 连接（每个连接一个线程）。"""

    def __init__(self, server, conn, addr):
        self.server = server
        self.conn = conn
        self.addr = addr
        self.sub_ids = set()  # 本连接订阅 odom 频道的 sub_id 集合
        self.lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.alive = True

    def _send_frame(self, payload: bytes, opcode: int) -> None:
        header = bytearray([0x80 | opcode])  # FIN + opcode，服务端帧不掩码
        n = len(payload)
        if n <= 125:
            header.append(n)
        elif n <= 0xFFFF:
            header.append(126)
            header += struct.pack(">H", n)
        else:
            header.append(127)
            header += struct.pack(">Q", n)
        with self.send_lock:
            self.conn.sendall(bytes(header) + payload)

    def _recv_exact(self, n: int):
        buf = b""
        while len(buf) < n:
            chunk = self.conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _recv_frame(self):
        hdr = self._recv_exact(2)
        if hdr is None:
            return None
        b0, b1 = hdr[0], hdr[1]
        opcode = b0 & 0x0F
        masked = (b1 >> 7) & 1
        length = b1 & 0x7F
        if length == 126:
            ext = self._recv_exact(2)
            if ext is None:
                return None
            length = struct.unpack(">H", ext)[0]
        elif length == 127:
            ext = self._recv_exact(8)
            if ext is None:
                return None
            length = struct.unpack(">Q", ext)[0]
        mask = self._recv_exact(4) if masked else None
        payload = self._recv_exact(length)
        if payload is None:
            return None
        if masked:
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        return opcode, payload

    def handshake(self) -> bool:
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = self.conn.recv(4096)
            if not chunk:
                return False
            data += chunk
            if len(data) > 65536:
                return False
        headers = {}
        for line in data.decode("latin-1").split("\r\n")[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()
        key = headers.get("sec-websocket-key", "")
        accept = base64.b64encode(hashlib.sha1((key + _WS_GUID).encode()).digest()).decode()
        resp = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n"
            "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n"
            "\r\n"
        )
        self.conn.sendall(resp.encode())
        return True

    def run(self) -> None:
        try:
            if not self.handshake():
                return
            print(f"[FoxgloveOdomServer] client connected: {self.addr[0]}:{self.addr[1]}")
            # 1) serverInfo
            self._send_frame(
                json.dumps({
                    "op": "serverInfo",
                    "name": "cuVSLAM",
                    "capabilities": [],
                    "supportedEncodings": ["ros1", "cdr"],
                }).encode(),
                0x1,
            )
            # 2) advertise odom 频道
            self._send_frame(
                json.dumps({
                    "op": "advertise",
                    "channels": [{
                        "id": self.server.channel_id,
                        "topic": self.server.odom_topic,
                        "encoding": "ros1",
                        "schemaName": "nav_msgs/Odometry",
                        "schemaEncoding": "ros1msg",
                    }],
                }).encode(),
                0x1,
            )

            while self.alive and self.server.running:
                frame = self._recv_frame()
                if frame is None:
                    break
                opcode, payload = frame
                if opcode == 0x8:  # close
                    break
                if opcode == 0x9:  # ping → pong（下游 websockets 客户端会定时 ping）
                    self._send_frame(payload, 0xA)
                elif opcode == 0x1:  # text（JSON：subscribe/unsubscribe）
                    self._handle_json(payload.decode("utf-8", "replace"))
                # 0x2 binary / 0xA pong / 0x0 continuation：忽略
        except (OSError, ConnectionError):
            pass
        finally:
            self.alive = False
            with self.server.lock:
                self.server.clients.discard(self)
            try:
                self.conn.close()
            except OSError:
                pass
            print(f"[FoxgloveOdomServer] client disconnected: {self.addr[0]}:{self.addr[1]}")

    def _handle_json(self, text: str) -> None:
        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            return
        op = msg.get("op")
        if op == "subscribe":
            subs = msg.get("subscriptions", [])
            with self.lock:
                for sub in subs:
                    if sub.get("channelId") == self.server.channel_id and sub.get("id") is not None:
                        self.sub_ids.add(sub["id"])
            self._send_sub_status(subs)
        elif op == "unsubscribe":
            with self.lock:
                for sid in msg.get("subscriptionIds", []):
                    self.sub_ids.discard(sid)

    def _send_sub_status(self, subs) -> None:
        self._send_frame(
            json.dumps({
                "op": "subscriptionStatus",
                "subscriptions": [{"id": s.get("id"), "status": "OK"} for s in subs],
            }).encode(),
            0x1,
        )

    def send_odometry(self, payload: bytes, ts_ns: int) -> None:
        """向本连接广播一条 odometry 二进制 MESSAGE_DATA。"""
        with self.lock:
            sub_ids = list(self.sub_ids)
        for sid in sub_ids:
            frame = b"\x01" + struct.pack("<I", sid) + struct.pack("<Q", ts_ns) + payload
            try:
                self._send_frame(frame, 0x2)
            except (OSError, ConnectionError):
                self.alive = False
                break


class FoxgloveOdomServer:
    """Foxglove WebSocket 服务端，把 cuVSLAM 位姿发布为 nav_msgs/Odometry。

    后台线程运行；`publish_pose()` 可从任意线程调用（线程安全）。
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8765,
                 odom_topic: str = "/high_frequency_odometry_baselink", channel_id: int = 0):
        self.host = host
        self.port = port
        self.odom_topic = odom_topic
        self.channel_id = channel_id
        self.clients = set()
        self.lock = threading.Lock()
        self.running = False
        self._seq = 0
        self._thread = None
        self._sock = None

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(8)
        self._sock.settimeout(0.5)
        self.running = True
        self._thread = threading.Thread(target=self._accept_loop, daemon=True,
                                        name="foxglove-odom-server")
        self._thread.start()
        print(f"[FoxgloveOdomServer] listening ws://{self.host}:{self.port} "
              f"(topic={self.odom_topic})")

    def _accept_loop(self) -> None:
        while self.running:
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            client = _ClientConnection(self, conn, addr)
            with self.lock:
                self.clients.add(client)
            threading.Thread(target=client.run, daemon=True).start()

    def stop(self) -> None:
        self.running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass

    def publish_pose(self, position_xyz, quaternion_xyzw, timestamp_sec=None) -> None:
        """变换 + 序列化 + 广播一条 cuVSLAM 位姿。"""
        if not self.running:
            return
        p_zup, q_zup = opencv_pose_to_zup(position_xyz, quaternion_xyzw)
        self._seq += 1
        payload = serialize_odometry(p_zup, q_zup, seq=self._seq, timestamp_sec=timestamp_sec)
        ts_ns = int((timestamp_sec if timestamp_sec is not None else time.time()) * 1e9)
        with self.lock:
            clients = list(self.clients)
        for client in clients:
            client.send_odometry(payload, ts_ns)
