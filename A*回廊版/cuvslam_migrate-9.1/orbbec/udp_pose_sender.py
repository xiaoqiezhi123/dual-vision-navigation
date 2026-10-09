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

"""UDP 位姿发送器：把 cuVSLAM 全局位姿以位姿包发给同机下游 RobotComm。

数据流（替代原 LiDAR→Foxglove WebSocket 方案）：
  cuVSLAM slam_pose → 变换 Z-up → UDP（只传位姿）→ RobotComm 自己差分算速度/重力、z=0.695 → SRU

协议（下游 bridge.py RobotComm 按此解析）：
  struct "<7d" = [px, py, pz, qw, qx, qy, qz]（7×float64 = 56 字节，小端）
    - 位置为 Z-up 世界系（+X 前、+Y 左、+Z 上）
    - 姿态为 Z-up 世界系单位四元数（wxyz 顺序）
  只传位姿，不含速度/重力 —— 下游 RobotComm 复用原有 _compute_velocities /
  _compute_projected_gravity / get_latest_state(z=0.695) 逻辑自行计算。

坐标系变换复用 foxglove_odom_server.opencv_pose_to_zup（OpenCV→Z-up 共轭变换）。
"""

import socket
import struct

from foxglove_odom_server import opencv_pose_to_zup
from pose_trace_hook import get_trace


class UdpPoseSender:
    """把 cuVSLAM 相机位姿变换为 Z-up 机体位姿后经 UDP 发送。

    UDP 的 sendto 非阻塞，`send_pose()` 可直接在主线程逐帧调用，无需额外线程。
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 8082):
        self.addr = (host, port)
        self._packer = struct.Struct("<7d")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.trace = get_trace()

    def send_pose(self, position_xyz, quaternion_xyzw, **trace_context) -> None:
        """变换 OpenCV→Z-up 并发送一帧位姿。

        Args:
            position_xyz: (3,) 相机在 OpenCV 世界下的位置 (x右, y下, z前)。
            quaternion_xyzw: (4,) 相机朝向 (x, y, z, w)。
        """
        p_zup, q_zup = opencv_pose_to_zup(position_xyz, quaternion_xyzw)
        qx, qy, qz, qw = q_zup  # opencv_pose_to_zup 返回 xyzw，报文用 wxyz
        packet = self._packer.pack(
            float(p_zup[0]), float(p_zup[1]), float(p_zup[2]),
            float(qw), float(qx), float(qy), float(qz),
        )
        packet, metadata = self.trace.encode(packet, **trace_context)
        try:
            self._sock.sendto(packet, self.addr)
        except OSError as exc:
            self.trace.emit('tx_error', **metadata, error=str(exc))
            raise
        if self.trace.enabled:
            self.trace.emit('tx', **metadata, wire_pose=list(self._packer.unpack(packet[:56])),
                map_position=list(map(float, position_xyz)), map_quaternion=list(map(float, quaternion_xyzw)))

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass
