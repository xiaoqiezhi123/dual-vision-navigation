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

import argparse
import os
import queue
import shutil
import signal
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional

import numpy as np

from pyorbbecsdk import (
    Config,
    Context,
    OBFormat,
    OBFrameType,
    OBPermissionType,
    OBPropertyID,
    OBSensorType,
    Pipeline,
)

import cuvslam as vslam
from camera_utils import get_orbbec_stereo_rig, get_stereo_calibration, process_ir_frame
from depth_shm import DepthShmWriter
from udp_pose_sender import UdpPoseSender
from visualizer_mapping import MappingVisualizer

# Gemini 336L 支持的 IR 分辨率与帧率（LEFT_IR 与 RIGHT_IR 一致）。
# 由 query_device_info.py 枚举得到；除注明外均为 Y8 格式：
#     1280x800: 5/10/15/30 fps    （另有 25 fps，格式为 Y12/Y16）
#     1280x720: 5/10/15/30 fps
#      848x480: 5/10/15/30/60 fps
#      640x480: 5/10/15/30/60/90 fps
#      640x400: 5/10/15/30 fps    （另有 25 fps，格式为 Y12/Y16）
#      640x360: 5/10/15/30/60/90 fps
#      480x270: 5/10/15/30/60/90 fps
#      424x266: 5/10/15/30 fps
#      424x240: 5/10/15/30/60/90 fps
#      848x100: 100 fps           （高速条带模式）
RESOLUTION = (1280, 720)
FPS = 15  #15

# SLAM 输入分辨率：把 IR 流从 RESOLUTION 软件降采样到该尺寸再喂 tracker。
# 设备强制 IR 与 depth 同分辨率（R0 实测），而 NavSide 要 1280x720 深度，于是 IR 也被
# 抬到 1280x720 —— Slam::Track() 计算量变成 848x480 的 2.26 倍，是「运行一段时间后卡」
# 的一个主因。这里在软件层把 IR 降回上游已验证不卡的 848x480 喂 SLAM，深度仍按
# RESOLUTION 输出 1280x720。改此值后需重新建图（地图建在 SLAM 输入分辨率下，
# 旧 1280x720 地图与 848x480 输入不兼容）。设成与 RESOLUTION 相同即关闭降采样。
SLAM_RESOLUTION = (848, 480)

# 深度转发给 NavSide（VAE/SRU 消费）：R0 实测 depth 与 IR 强制同分辨率同帧率，
# 故 depth 与 IR 都跑 1280x720@15。填洞后写共享内存，NavSide 读（见 depth_shm.py）。
# 双相机方案：深度改由「相机 B」（NavSide 原生 2.1.2 SDK，激光开）直接出，不再从相机 A 转发。
# 相机 A 只做被动双目 SLAM，故这里关闭深度转发与深度流。
DEPTH_OUTPUT_ENABLED = False
DEPTH_SHM_NAME = "cuvslam_depth"

# IR 激光散斑发射器开关（实验 A 的 A/B 开关）。
# Gemini 336L 是主动双目：深度靠激光散斑做立体匹配，激光一关深度就跳变/空洞；
# 而被动双目 SLAM 需要关激光（散斑相对相机静止、几乎无视差，会骗 SLAM 以为没动）。
# 二者共享同一 device 级激光，无法同时满足。
#   True  = 关激光跑 SLAM（默认，SLAM 稳、深度差）
#   False = 开激光跑 SLAM（实验 A：深度好，但 SLAM 跟踪可能退化/漂移，需实机验证）
DISABLE_IR_EMITTER = True  # 双相机：相机 A 固定关激光跑被动 SLAM

# 双相机方案：按序列号绑定「相机 A」（SLAM 被动双目，激光关），避免与相机 B 抢设备。
# 留空 "" = 打开枚举到的第一台（单相机测试 / 未接第二台时用）。
# 接上两台后用 enumerate_devices.py 读出 SN，把 SLAM 相机填到这里；或用环境变量覆盖：
#   CUVSLAM_CAMERA_SERIAL="<SN>" ./run_orbbec.sh ...
CAMERA_A_SERIAL = os.environ.get("CUVSLAM_CAMERA_SERIAL", "")

# IMU 采样频率（Gemini 336L 内置 IMU 约以 200 Hz 输出）
IMU_FREQUENCY = 200

# IMU 噪声参数：在 RealSense BMI055 原版值基础上 × IMU_NOISE_INFLATION 放大。
# 方向与标定相反——标定把值调小（更信 IMU），这里故意调大（更不信 IMU），
# 用于验证「实时漂移是否来自 IMU 被过度信任」这一假设。
# 注：标定真值比 RealSense 原版低 11~31 倍，原版本身已经「轻 IMU」。
IMU_NOISE_INFLATION = 1.0
IMU_GYROSCOPE_NOISE_DENSITY = 6.0673370376614875e-03 * IMU_NOISE_INFLATION
IMU_GYROSCOPE_RANDOM_WALK = 3.6211951458325785e-05 * IMU_NOISE_INFLATION
IMU_ACCELEROMETER_NOISE_DENSITY = 3.3621979208052800e-02 * IMU_NOISE_INFLATION
IMU_ACCELEROMETER_RANDOM_WALK = 9.8256589971851467e-04 * IMU_NOISE_INFLATION

FRAME_PERIOD_MS = 1000 / FPS
IMAGE_JITTER_THRESHOLD_NS = (FRAME_PERIOD_MS + 5) * 1e6  # 相机帧间隔容差
IMU_JITTER_THRESHOLD_NS = 20 * 1e6  # IMU 采样间隔容差（~200 Hz -> ~5 ms）
IMU_QUEUE_MAX_SIZE = IMU_FREQUENCY * 5  # 最多缓冲约 5 s 的 IMU 数据

SHOW_GRAVITY = False  # 可视化估计的重力向量

USE_SLAM_MODE = True  # 启用回环检测 + 位姿图优化
SLAM_SYNC_MODE = False  # False = SLAM 在后台线程运行（推荐）
PLANAR_CONSTRAINTS = False  # 关闭每帧平面 PGO（隔离测试：定位漂移是否来自 SLAM 后端纠正）
LOOP_CLOSURE_THROTTLING_MS = 10**9  # 关闭回环（隔离测试：定位漂移是否来自回环纠正）

# ---- 全局建图 / 重定位（给下游 VAE/SRU 提供全局地图与位姿）----
# "map"      = 建图模式：边跑边建 SLAM 地图，退出时 save_map 落盘。
# "localize" = 重定位模式：从所选地图载入已有地图，用 localize_in_map 找到当前位姿后继续跟踪。
#              定位成功后，track() 返回的 slam_pose 即相机在全局地图系下的位姿。
MODE = "map"  # 「map 模式发位姿」测试版：固定 map，位姿出口 + 终端打印（slam_pose -> UDP）

# ---- 多地图管理 ----
# 每张地图就是脚本目录下的一个子文件夹（如 orbbec_map），内部是 cuVSLAM 的 LMDB 数据库（data.mdb）。
# 每张地图对应一个 last-pose 文件（<name>_last_pose.txt，如 orbbec_map_last_pose.txt），
# 记录上次退出时的全局位姿。这与历史单地图版本的存放位置完全一致，旧地图无需迁移。
# map 模式：交互式新建 / 覆盖地图（或 --map <name> 直接指定）。
# localize 模式：交互式从已有地图中选择（或 --map <name> 直接指定）。
# --delete：进入删除地图菜单后退出（不启动相机）。
MAPS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MAP_NAME = "orbbec_map"  # 交互输入为空时使用的地图名
MAX_MAP_SIZE = 0  # SLAM 位姿图节点上限；0 = 不限（做全局大图，覆盖大范围场景）

# localize_in_map 是在 guess_pose 附近做 4D 网格搜索（x/y/z + 绕竖直轴航向角），
# 航向角永远搜满 360°。半径小则快但覆盖不到远点，半径大则候选位姿爆炸。
# 因此用「由粗到细三级定位」：大半径+粗步长 → 中半径+中步长 → 小半径+细步长，
# 每级都以上一级结果为 guess，逐级收敛到精确位姿。
# 若存在该地图上次保存的位姿（last-pose 文件），则用它作初始 guess；否则用下方默认值。
LOCALIZE_GUESS_TRANSLATION = (0.0, 0.0, 0.0)   # 冷启动默认先验平移（米），世界系下
LOCALIZE_GUESS_ROTATION = (0.0, 0.0, 0.0, 1.0)  # 冷启动默认先验朝向四元数 (x, y, z, w)

# 第一层（粗）：大半径覆盖整张地图 / 大范围，粗步长快速锁定大致位置。
LOCALIZE_COARSE_H_RADIUS = 25.0   # 水平搜索半径（米），覆盖真值可能出现的最远距离
LOCALIZE_COARSE_V_RADIUS = 0.2    # 竖直搜索半径（米）；平面运动 y 几乎不动，保持小
LOCALIZE_COARSE_H_STEP = 1.0      # 水平步长（米）
LOCALIZE_COARSE_V_STEP = 0.2      # 竖直步长（米）
LOCALIZE_COARSE_A_STEP = 0.15     # 航向角步长（弧度，~8.6°）

# 第二层（中）：围绕粗结果进一步收敛，半径 / 步长居中。
LOCALIZE_MID_H_RADIUS = 4.0
LOCALIZE_MID_V_RADIUS = 0.1
LOCALIZE_MID_H_STEP = 0.5
LOCALIZE_MID_V_STEP = 0.1
LOCALIZE_MID_A_STEP = 0.06        # 航向角步长（弧度，~3.4°）

# 第三层（精）：围绕中结果，半径小、步长细，得到精确位姿。
LOCALIZE_FINE_H_RADIUS = 1.0
LOCALIZE_FINE_V_RADIUS = 0.1
LOCALIZE_FINE_H_STEP = 0.2
LOCALIZE_FINE_V_STEP = 0.1
LOCALIZE_FINE_A_STEP = 0.03       # 航向角步长（弧度，~1.7°）

# 建图结束时的「原地环绕采样」：保存地图前提示用户原地旋转相机，
# 让 SLAM 从多个朝向记录关键帧，提升后续重定位对航向角的鲁棒性。
ROTATIONAL_SURVEY_ENABLED = True

# 是否在 Rerun 里实时显示建图内容（稀疏地图点 / 位姿图 / 回环位置）。
# True  = 显示建图；False = 仅显示轨迹与 2D 特征（与 run_vio_slam.py 一致），并跳过建图数据读取。
ENABLE_MAPPING_VISUALIZATION = True
# 建图数据（地图点 / 位姿图）的刷新节流间隔（ms），避免每帧读取全量 landmarks。
MAP_UPDATE_INTERVAL_MS = 300

NUM_DESIRED_TRACKS = 800  #800  # 每帧期望的特征跟踪数量

# 跟踪健康度提示：滑动窗口内统计特征数与失败率，在卡住 / 漂移之前预警。
HEALTH_PRINT_INTERVAL_S = 1.0      # 健康提示打印节流（秒）
ENABLE_HEALTH_PRINT = False        # 是否打印 [health] 跟踪健康度（False = 关闭）
HEALTH_WINDOW_FRAMES = 30          # 滑动窗口帧数（15fps 下约 2s）
FEATURE_COUNT_WARN_THRESHOLD = 100  # 单帧特征数低于此值判 WARN
FEATURE_COUNT_LOST_THRESHOLD = 30   # 单帧特征数低于此值判 LOST（即将丢跟踪）

POSE_PRINT_INTERVAL_S = 0.1  # 重定位完成后实时坐标打印节流（秒）；0.1=10Hz, 0.067≈15Hz

# ---- 位姿输出（给下游 VAE/SRU）：以 UDP 发送位姿包给同机下游 RobotComm ----
# 下游 bridge.py（/home/amov/Desktop/bridge.py）RobotComm 用 pose_transport=udp 接收，
# 收到位姿后自己差分算速度/重力、z=0.695。本模块只传位姿，不做速度/重力。
# 位姿包：struct "<7d" = [px,py,pz, qw,qx,qy,qz]（Z-up 世界系，四元数 wxyz）。
POSE_UDP_OUTPUT_ENABLED = True
POSE_UDP_HOST = "127.0.0.1"  # 同机 loopback，发给下游 RobotComm
POSE_UDP_PORT = 8082

STEREO_SENSORS = {OBSensorType.LEFT_IR_SENSOR, OBSensorType.RIGHT_IR_SENSOR}


@dataclass
class ImuSample:
    """注册进 cuVSLAM 之前缓存的原始 IMU 采样。"""

    timestamp_ns: int
    linear_accelerations: tuple
    angular_velocities: tuple


class ThreadWithTimestamp:
    """用于在相机线程与 IMU 线程之间共享时间戳的辅助类。"""

    def __init__(self, low_rate_threshold_ns: int, high_rate_threshold_ns: int) -> None:
        self.prev_low_rate_timestamp: Optional[int] = None
        self.prev_high_rate_timestamp: Optional[int] = None
        self.low_rate_threshold_ns = low_rate_threshold_ns
        self.high_rate_threshold_ns = high_rate_threshold_ns
        self.last_low_rate_timestamp: Optional[int] = None


def disable_ir_emitter(pipeline: Pipeline) -> Optional[int]:
    """关闭 IR 激光发射器，用于被动双目跟踪。

    IR 发射器会投射散斑图案，干扰立体匹配。
    被动双目（以及双目惯性）跟踪必须将其关闭。

    Returns:
        关闭前的 OB_PROP_LASER_CONTROL_INT 值，供退出时恢复；若未通过该属性关闭则返回 None。

    Args:
        pipeline: Orbbec pipeline
    """
    device = pipeline.get_device()
    emitter_disabled = False
    original_laser_control: Optional[int] = None

    # 尝试 OB_PROP_LASER_BOOL（最常见）
    try:
        if device.is_property_supported(
            OBPropertyID.OB_PROP_LASER_BOOL, OBPermissionType.PERMISSION_READ_WRITE
        ):
            device.set_bool_property(OBPropertyID.OB_PROP_LASER_BOOL, False)
            current = device.get_bool_property(OBPropertyID.OB_PROP_LASER_BOOL)
            if not current:
                print("IR emitter disabled (OB_PROP_LASER_BOOL)")
                emitter_disabled = True
            else:
                print("Warning: Failed to disable laser via OB_PROP_LASER_BOOL")
    except Exception as e:
        print(f"OB_PROP_LASER_BOOL not available: {e}")

    # 尝试 OB_PROP_LASER_CONTROL_INT（0=关，1=开，2=自动）
    if not emitter_disabled:
        try:
            if device.is_property_supported(
                OBPropertyID.OB_PROP_LASER_CONTROL_INT, OBPermissionType.PERMISSION_READ_WRITE
            ):
                original_laser_control = device.get_int_property(OBPropertyID.OB_PROP_LASER_CONTROL_INT)
                device.set_int_property(OBPropertyID.OB_PROP_LASER_CONTROL_INT, 0)
                print("IR emitter disabled (OB_PROP_LASER_CONTROL_INT)")
                emitter_disabled = True
        except Exception as e:
            print(f"OB_PROP_LASER_CONTROL_INT not available: {e}")

    # 尝试 OB_PROP_LDP_BOOL（激光二极管功率）
    if not emitter_disabled:
        try:
            if device.is_property_supported(
                OBPropertyID.OB_PROP_LDP_BOOL, OBPermissionType.PERMISSION_READ_WRITE
            ):
                device.set_bool_property(OBPropertyID.OB_PROP_LDP_BOOL, False)
                print("IR emitter disabled (OB_PROP_LDP_BOOL)")
                emitter_disabled = True
        except Exception as e:
            print(f"OB_PROP_LDP_BOOL not available: {e}")

    if not emitter_disabled:
        print("Warning: Could not disable IR emitter - device may not support this feature")
    return original_laser_control


def restore_ir_emitter(pipeline: Pipeline, original_laser_control: Optional[int]) -> None:
    """退出时恢复 IR 激光发射器。

    Gemini 336L 的深度依赖激光散斑图案；cuVSLAM 为被动双目跟踪关闭了它（见
    ``disable_ir_emitter``）。若退出时不恢复，相机会停留在「激光关闭」状态，导致
    后续打开相机（如原生 NavSide 深度后端）拿到的深度明显跳变/闪动，直到拔插相机。
    """
    if original_laser_control is None:
        return
    try:
        device = pipeline.get_device()
        if device.is_property_supported(
            OBPropertyID.OB_PROP_LASER_CONTROL_INT, OBPermissionType.PERMISSION_READ_WRITE
        ):
            device.set_int_property(OBPropertyID.OB_PROP_LASER_CONTROL_INT, original_laser_control)
            print(f"IR emitter restored (OB_PROP_LASER_CONTROL_INT={original_laser_control})")
    except Exception as e:  # noqa: BLE001
        print(f"Warning: could not restore IR emitter: {e}")


def open_camera_pipeline(serial: str = "") -> Pipeline:
    """按序列号打开相机；空串 = 枚举到的第一台（单相机兼容）。

    双相机方案里 cuVSLAM 只应打开「相机 A」（SLAM 被动双目，激光关），
    不能抢相机 B（NavSide 原生深度用）。串号用 enumerate_devices.py 读出后填入
    CAMERA_A_SERIAL，或用环境变量 CUVSLAM_CAMERA_SERIAL 覆盖。
    """
    if not serial:
        return Pipeline()
    ctx = Context()  # 保持引用，防止临时 Context 被 GC 后 deviceMgr 悬空（与 NavSide 同款坑）
    devices = ctx.query_devices()
    device = devices.get_device_by_serial_number(serial)
    if device is None:
        raise RuntimeError(
            f"未找到序列号 '{serial}' 的相机（当前已连接 {devices.get_count()} 台）"
        )
    return Pipeline(device)


def get_orbbec_imu(
    frequency: int = IMU_FREQUENCY,
    gyroscope_noise_density: float = IMU_GYROSCOPE_NOISE_DENSITY,
    gyroscope_random_walk: float = IMU_GYROSCOPE_RANDOM_WALK,
    accelerometer_noise_density: float = IMU_ACCELEROMETER_NOISE_DENSITY,
    accelerometer_random_walk: float = IMU_ACCELEROMETER_RANDOM_WALK,
) -> vslam.ImuCalibration:
    """为 Orbbec 内置 IMU 构建 IMU 标定对象。

    Orbbec SDK 未暴露 IMU<->相机外参和噪声模型，因此：
    - rig_from_imu 为占位值（单位旋转、零平移）；请用 kalibr 标定。
    - 噪声密度 / 随机游走沿用 RealSense BMI055 的默认值。
    """
    imu = vslam.ImuCalibration()
    # 单位四元数（x, y, z, w）+ 零平移。
    imu.rig_from_imu = vslam.Pose(rotation=[0.0, 0.0, 0.0, 1.0], translation=[0.0, 0.0, 0.0])
    imu.gyroscope_noise_density = gyroscope_noise_density
    imu.gyroscope_random_walk = gyroscope_random_walk
    imu.accelerometer_noise_density = accelerometer_noise_density
    imu.accelerometer_random_walk = accelerometer_random_walk
    imu.frequency = frequency
    return imu


def get_orbbec_vio_rig(
    stereo_params: dict,
    gyroscope_noise_density: float = IMU_GYROSCOPE_NOISE_DENSITY,
    gyroscope_random_walk: float = IMU_GYROSCOPE_RANDOM_WALK,
    accelerometer_noise_density: float = IMU_ACCELEROMETER_NOISE_DENSITY,
    accelerometer_random_walk: float = IMU_ACCELEROMETER_RANDOM_WALK,
    target_size: Optional[tuple] = None,
) -> vslam.Rig:
    """根据 Orbbec 双目参数构建 VIO 配置（2 个相机 + 1 个 IMU）。"""
    rig = get_orbbec_stereo_rig(stereo_params, target_size=target_size)
    rig.imus = [
        get_orbbec_imu(
            gyroscope_noise_density=gyroscope_noise_density,
            gyroscope_random_walk=gyroscope_random_walk,
            accelerometer_noise_density=accelerometer_noise_density,
            accelerometer_random_walk=accelerometer_random_walk,
        )
    ]
    return rig


def imu_thread(
    imu_queue: queue.Queue,
    thread_with_timestamp: ThreadWithTimestamp,
    imu_pipe: Pipeline,
    stop_event: threading.Event,
) -> None:
    """IMU 采集线程：读取加速度计 + 陀螺仪并写入 ImuSample 队列。

    Orbbec IMU 将加速度计和陀螺仪暴露为两个独立传感器，但由同一个物理 IMU 采样。
    SDK 可能把二者放在同一个 FrameSet 中，也可能分开返回，因此我们保留最新的陀螺仪值，
    每来一帧新的加速度计数据就输出一个 ImuSample（以加速度计作为时钟）。
    """
    high_rate_threshold = thread_with_timestamp.high_rate_threshold_ns
    prev_timestamp = None
    drop_count = 0
    queue_drop_count = 0
    last_gyro = None  # 最新的 (gx, gy, gz)

    try:
        while not stop_event.is_set():
            frames = imu_pipe.wait_for_frames(100)
            if frames is None:
                continue

            gyro_frame = frames.get_frame(OBFrameType.GYRO_FRAME)
            if gyro_frame is not None:
                gyro_frame = gyro_frame.as_gyro_frame()
                if gyro_frame is not None:
                    last_gyro = (gyro_frame.get_x(), gyro_frame.get_y(), gyro_frame.get_z())

            accel_frame = frames.get_frame(OBFrameType.ACCEL_FRAME)
            if accel_frame is not None:
                accel_frame = accel_frame.as_accel_frame()
            if accel_frame is None or last_gyro is None:
                continue

            current_timestamp = int(accel_frame.get_timestamp_us() * 1000)

            # 时间戳间隔检查
            if prev_timestamp is not None:
                timestamp_diff = current_timestamp - prev_timestamp
                if timestamp_diff < 0:
                    continue
                if timestamp_diff > high_rate_threshold:
                    drop_count += 1
                    if drop_count % 100 == 1:
                        print(
                            f"Warning: IMU drops detected ({drop_count} total, "
                            f"last gap: {timestamp_diff/1e6:.2f} ms)"
                        )
            prev_timestamp = current_timestamp
            thread_with_timestamp.prev_high_rate_timestamp = current_timestamp

            sample = ImuSample(
                timestamp_ns=current_timestamp,
                linear_accelerations=(accel_frame.get_x(), accel_frame.get_y(), accel_frame.get_z()),
                angular_velocities=last_gyro,
            )
            try:
                imu_queue.put_nowait(sample)
            except queue.Full:
                queue_drop_count += 1
                # 丢弃最旧的采样，避免采集线程被阻塞。
                try:
                    imu_queue.get_nowait()
                except queue.Empty:
                    pass
                imu_queue.put_nowait(sample)
                if queue_drop_count % 100 == 1:
                    print(f"Warning: IMU queue overflow ({queue_drop_count} dropped samples)")
    except Exception as e:
        print(f"IMU thread error: {e}")


def register_imu_until(
    tracker: vslam.Tracker,
    imu_queue: queue.Queue,
    pending_imu: Deque[ImuSample],
    timestamp_ns: int,
    last_tracker_timestamp_ns: Optional[int],
) -> Optional[int]:
    """把缓存的 IMU 采样注册到当前图像时间戳为止。

    cuVSLAM 要求 Track 与 register_imu_measurement 的调用按时间戳顺序进行。
    IMU 采集在独立线程中运行；所有 tracker 调用都由相机线程汇总到这里。
    """
    while True:
        try:
            pending_imu.append(imu_queue.get_nowait())
        except queue.Empty:
            break

    while pending_imu and pending_imu[0].timestamp_ns <= timestamp_ns:
        sample = pending_imu.popleft()
        if last_tracker_timestamp_ns is not None and sample.timestamp_ns < last_tracker_timestamp_ns:
            continue
        imu_measurement = vslam.ImuMeasurement()
        imu_measurement.timestamp_ns = sample.timestamp_ns
        imu_measurement.linear_accelerations = sample.linear_accelerations
        imu_measurement.angular_velocities = sample.angular_velocities
        tracker.register_imu_measurement(0, imu_measurement)
        last_tracker_timestamp_ns = sample.timestamp_ns
    return last_tracker_timestamp_ns


def quaternion_to_euler_deg(rotation) -> tuple:
    """四元数 (x, y, z, w) -> 欧拉角 (roll, pitch, yaw)，单位度（ZYX 内旋）。

    注意 cuVSLAM 世界系是 OpenCV 约定（+X 右、+Y 下、+Z 前），竖直轴是 Y（向下），
    所以三个角对应的物理含义与常见 Z-up 车辆系不同：
      - roll   = 绕 X 轴（右轴）  -> 俯仰（前后倾斜）
      - pitch  = 绕 Y 轴（下轴）  -> 航向角 heading（左右转向）
      - yaw    = 绕 Z 轴（前轴）  -> 横滚（绕光轴自转）
    即「航向角 = pitch」。
    """
    x, y, z, w = rotation
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw))


def format_pose(pose) -> str:
    """把 Pose 格式化成「xyz 位置 + 三个角度（度）」的可读字符串。"""
    x, y, z = pose.translation
    roll, pitch, yaw = quaternion_to_euler_deg(pose.rotation)
    # 航向（pitch）接近 ±90° 时 ZYX 欧拉角进入万向锁，俯仰/横滚数值失去意义，仅作参考。
    warn = " ⚠️俯仰/横滚近万向锁(仅供参考)" if abs(pitch) > 85.0 else ""
    return (
        f"xyz=({x:.3f}, {y:.3f}, {z:.3f}) m | "
        f"heading(航向)={pitch:.2f}° 俯仰={roll:.2f}° 横滚={yaw:.2f}°{warn}"
    )


def map_dir_path(name: str) -> str:
    """地图名 → 地图文件夹路径（脚本目录/<name>）。"""
    return os.path.join(MAPS_DIR, name)


def last_pose_path(name: str) -> str:
    """地图名 → 该地图的 last-pose 文件路径（脚本目录/<name>_last_pose.txt）。"""
    return os.path.join(MAPS_DIR, name + "_last_pose.txt")


def list_maps() -> List[str]:
    """列出脚本目录下已存在的地图名（含 data.mdb 的 LMDB 地图目录，按名称排序）。"""
    if not os.path.isdir(MAPS_DIR):
        return []
    return sorted(
        name for name in os.listdir(MAPS_DIR)
        if os.path.isfile(os.path.join(MAPS_DIR, name, "data.mdb"))
    )


def delete_map(name: str) -> bool:
    """删除指定地图（地图文件夹 + 其 last-pose 文件）。"""
    removed = False
    p = map_dir_path(name)
    if os.path.isdir(p):
        shutil.rmtree(p)
        print(f"已删除地图文件夹: {p}")
        removed = True
    lp = last_pose_path(name)
    if os.path.isfile(lp):
        os.remove(lp)
        print(f"已删除位姿文件: {lp}")
        removed = True
    if not removed:
        print(f"地图 '{name}' 不存在。")
        return False
    return True


def select_map_for_save() -> str:
    """交互式选择「保存到哪张地图」：新名=新建，已有名=覆盖（需确认）。"""
    os.makedirs(MAPS_DIR, exist_ok=True)
    existing = list_maps()
    print("\n" + "=" * 62)
    print("建图模式：选择保存到哪张地图")
    if existing:
        print("已有地图：" + ", ".join(existing))
    else:
        print("（暂无已有地图）")
    print("输入新名字 = 新建；输入已有名字 = 覆盖（会先确认）。")
    print(f"直接回车 = 使用默认名 '{DEFAULT_MAP_NAME}'。")
    print("=" * 62)

    while True:
        name = input("地图名: ").strip() or DEFAULT_MAP_NAME
        if not name or "/" in name or "\\" in name or name in (".", ".."):
            print("名称不能为空或包含路径分隔符，请重试。")
            continue
        if name in existing:
            confirm = input(f"地图 '{name}' 已存在，覆盖将丢失旧数据。确认覆盖？[y/N]: ").strip().lower()
            if confirm != "y":
                continue
        return name


def select_map_for_load() -> Optional[str]:
    """交互式从已有地图中选择要载入的地图；无地图时返回 None。"""
    existing = list_maps()
    if not existing:
        print("未找到任何已保存地图，请先以 --mode map 建图。")
        return None
    print("\n" + "=" * 62)
    print("重定位模式：选择要载入的地图")
    for i, name in enumerate(existing, 1):
        print(f"  {i}. {name}")
    print("=" * 62)
    while True:
        choice = input("请输入编号: ").strip()
        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(existing):
                return existing[idx - 1]
        print(f"请输入 1~{len(existing)} 之间的编号。")


def delete_map_menu() -> None:
    """交互式删除地图：列出已有地图，选择其一删除（需确认）。"""
    existing = list_maps()
    if not existing:
        print("没有可删除的地图。")
        return
    print("\n" + "=" * 62)
    print("删除地图")
    for i, name in enumerate(existing, 1):
        print(f"  {i}. {name}")
    print("  0. 取消")
    print("=" * 62)
    while True:
        choice = input("请输入要删除的地图编号: ").strip()
        if choice == "0":
            print("已取消。")
            return
        if choice.isdigit():
            idx = int(choice)
            if 1 <= idx <= len(existing):
                name = existing[idx - 1]
                confirm = input(f"确认删除地图 '{name}'？此操作不可恢复。[y/N]: ").strip().lower()
                if confirm == "y":
                    delete_map(name)
                else:
                    print("已取消。")
                return
        print("输入无效，请重试。")


def load_last_pose(name: str = DEFAULT_MAP_NAME) -> Optional[vslam.Pose]:
    """读取指定地图上次退出时保存的全局位姿（世界系），用作重定位的初始 guess。

    文件不存在或损坏时返回 None（走冷启动 guess）。
    """
    path = last_pose_path(name)
    if not os.path.isfile(path):
        return None
    try:
        with open(path) as f:
            vals = f.read().split()
        tx, ty, tz, qx, qy, qz, qw = (float(v) for v in vals)
        return vslam.Pose(translation=[tx, ty, tz], rotation=[qx, qy, qz, qw])
    except Exception as e:
        print(f"Warning: failed to load last pose from {path}: {e}")
        return None


def save_last_pose(pose: vslam.Pose, name: str = DEFAULT_MAP_NAME) -> None:
    """把当前全局位姿落盘到指定地图，供下次 localize 作初始 guess。

    连续定位时用上次位姿作 guess，粗定位搜索框可锁在当前位置附近，大幅提速。
    """
    path = last_pose_path(name)
    os.makedirs(MAPS_DIR, exist_ok=True)
    try:
        t, r = pose.translation, pose.rotation
        with open(path, "w") as f:
            f.write(f"{t[0]:.6f} {t[1]:.6f} {t[2]:.6f} {r[0]:.6f} {r[1]:.6f} {r[2]:.6f} {r[3]:.6f}\n")
        print(f"Saved last pose to {path}: {format_pose(pose)}")
    except Exception as e:
        print(f"Warning: failed to save last pose: {e}")


def start_localization(
    tracker: vslam.Tracker,
    timestamp_ns: int,
    images: tuple,
    done_event: threading.Event,
    result_holder: List[Optional[vslam.Pose]],
    map_name: str = DEFAULT_MAP_NAME,
) -> None:
    """由粗到细三级重定位（localize_in_map）。

    localize 模式使用同步 SLAM（``sync_mode=True``），每次 ``localize_in_map``
    在调用内同步完成、回调同步执行（无后台线程），因此可以顺序做三遍：
      1) 粗定位：大半径 + 粗步长，覆盖整张地图，锁定大致位置；
      2) 中定位：以粗结果为 guess，中等半径 + 中步长，进一步收敛；
      3) 精定位：以中结果为 guess，小半径 + 细步长，得到精确位姿。

    初始 guess 优先取该地图上次保存的位姿（last-pose 文件），无则用冷启动默认值。
    任一层失败都会回退到上一层的成功结果（不丢弃已得到的较粗位姿）。

    Args:
        tracker: cuVSLAM tracker（需已开启 SLAM）
        timestamp_ns: 定位帧时间戳
        images: 当前双目图像（左、右）
        done_event: 定位完成事件（整个三级流程结束后 set）
        result_holder: 长度为 1 的容器，定位成功时 result_holder[0] 存最终位姿
        map_name: 要载入的地图名（对应 maps/<name> 子文件夹）
    """
    # 绑定侧接收 std::vector<nb::ndarray>；显式转成 list 以避免 tuple 转换差异。
    images = list(images)

    def run_pass(guess_pose, h_radius, v_radius, h_step, v_step, a_step, label):
        """执行一次 localize_in_map 并同步返回 (pose, error_message)。"""
        settings = vslam.Tracker.SlamLocalizationSettings(
            horizontal_search_radius=h_radius,
            vertical_search_radius=v_radius,
            horizontal_step=h_step,
            vertical_step=v_step,
            angular_step_rads=a_step,
        )
        out = {"pose": None, "error": None}
        inner_done = threading.Event()

        def finish_cb(pose: Optional[vslam.Pose], error_message: str) -> None:
            out["pose"] = pose
            out["error"] = error_message
            inner_done.set()

        print(f"[localize] {label}: H_radius={h_radius}m, h_step={h_step}m, a_step={a_step}rad ...")
        tracker.localize_in_map(map_dir_path(map_name), timestamp_ns, guess_pose, images, settings, lambda: None, finish_cb)
        inner_done.wait(timeout=60.0)  # sync 模式下已同步完成，这里仅作兜底
        return out["pose"], out["error"]

    # 初始 guess：优先该地图上次保存的位姿，否则冷启动默认值。
    guess = load_last_pose(map_name)
    if guess is not None:
        print(f"[localize] using saved last pose as initial guess: {format_pose(guess)}")
    else:
        guess = vslam.Pose(
            rotation=list(LOCALIZE_GUESS_ROTATION), translation=list(LOCALIZE_GUESS_TRANSLATION)
        )
        print("[localize] no saved last pose; using cold-start origin guess")

    # 三级由粗到细定位，每级以上一级结果为 guess。
    stages = [
        ("coarse", LOCALIZE_COARSE_H_RADIUS, LOCALIZE_COARSE_V_RADIUS,
         LOCALIZE_COARSE_H_STEP, LOCALIZE_COARSE_V_STEP, LOCALIZE_COARSE_A_STEP),
        ("mid", LOCALIZE_MID_H_RADIUS, LOCALIZE_MID_V_RADIUS,
         LOCALIZE_MID_H_STEP, LOCALIZE_MID_V_STEP, LOCALIZE_MID_A_STEP),
        ("fine", LOCALIZE_FINE_H_RADIUS, LOCALIZE_FINE_V_RADIUS,
         LOCALIZE_FINE_H_STEP, LOCALIZE_FINE_V_STEP, LOCALIZE_FINE_A_STEP),
    ]

    current = guess
    last_pose: Optional[vslam.Pose] = None
    for label, h_radius, v_radius, h_step, v_step, a_step in stages:
        pose, err = run_pass(current, h_radius, v_radius, h_step, v_step, a_step, label)
        if pose is None:
            # 本层失败：若有上一层结果则回退用之，否则整体失败。
            if last_pose is not None:
                result_holder[0] = last_pose
                print(f"[localize] {label} failed ({err}); using previous-stage pose: {format_pose(last_pose)}")
            else:
                print(f"Localization failed ({label}): {err}")
            done_event.set()
            return
        last_pose = pose
        current = pose

    result_holder[0] = last_pose
    print(f"Localized pose: {format_pose(last_pose)}")
    done_event.set()


def rotational_survey() -> None:
    """建图结束时的原地环绕采样引导。

    相机线程仍在后台继续 track、SLAM 持续插入关键帧；本函数阻塞主线程，
    等待用户完成原地旋转后按 Enter，再返回保存地图。旋转会让 SLAM 从多个
    朝向观察同一批地图点，从而提升后续重定位对航向角的鲁棒性。
    """
    print()
    print("=" * 62)
    print("原地环绕采样（提升重定位航向鲁棒性）")
    print("请把相机举在当前位置，缓慢原地旋转一整圈（360°）。")
    print("建议旋转时小幅前后 / 左右平移，给 SLAM 提供三角化基线，")
    print("确保旋转过程中会插入新关键帧（纯绕光轴自转可能无基线）。")
    print("保持画面内有纹理特征，约 10~15 秒。")
    print("旋转完成后按 Enter 保存地图（直接按 Enter 可跳过本步骤）。")
    print("=" * 62)
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass
    # 给后台异步 SLAM 线程一点时间消化刚插入的关键帧，再落盘。
    time.sleep(1.0)


def save_map_and_wait(tracker: vslam.Tracker, path: str, timeout_s: float = 15.0) -> bool:
    """保存 SLAM 地图并阻塞等待完成。"""
    done_event = threading.Event()
    result_holder = [False]

    def callback(success: bool) -> None:
        result_holder[0] = success
        done_event.set()

    print(f"Saving map to {path} ...")
    tracker.save_map(path, callback)
    done_event.wait(timeout=timeout_s)
    if result_holder[0]:
        print("Map saved successfully.")
    else:
        print("WARNING: map saving may not have completed (timeout or failure).")
    return result_holder[0]


def print_tracking_health(obs_history: Deque[int], fail_history: Deque[int]) -> None:
    """打印跟踪健康度：滑动窗口内的失败率 + 特征数（当前/均值/最低）。

    用于在跟踪卡住 / 漂移之前预警：特征数持续走低或失败率升高时，应放慢移动、
    增加纹理或避免原地自转等退化运动。
    """
    if not ENABLE_HEALTH_PRINT:
        return
    n = len(fail_history)
    if n == 0:
        return
    obs = list(obs_history)
    fails = sum(fail_history)
    cur = obs[-1]
    avg = sum(obs) / n
    mn = min(obs)
    fail_rate = 100.0 * fails / n
    if cur <= FEATURE_COUNT_LOST_THRESHOLD or fail_rate >= 50.0:
        status = "LOST"
    elif cur <= FEATURE_COUNT_WARN_THRESHOLD or fail_rate >= 10.0:
        status = "WARN"
    else:
        status = "OK"
    print(
        f"[health] {status} | 特征数 当前={cur} 均值={avg:.0f} 最低={mn} | "
        f"失败率 {fail_rate:.0f}% ({fails}/{n}帧)"
    )


def camera_thread(
    tracker: vslam.Tracker,
    result_queue: queue.Queue,
    imu_queue: queue.Queue,
    thread_with_timestamp: ThreadWithTimestamp,
    ir_pipe: Pipeline,
    show_gravity: bool,
    internals: vslam.Tracker.Internals,
    mode: str,
    map_name: str,
    stop_event: threading.Event,
    depth_hole_filter=None,
    depth_writer: Optional[DepthShmWriter] = None,
) -> None:
    """相机处理线程：读取双目 IR，注册 IMU，并进行跟踪。

    在 ``localize`` 模式下，首帧成功后发起 localize_in_map，随后继续正常跟踪；
    定位结果由后台线程回调写入 ``localized_pose_holder``，并随结果队列带给主线程。
    """
    pending_imu: Deque[ImuSample] = deque()
    last_tracker_timestamp: Optional[int] = None
    last_map_update_ns = 0
    last_trajectory_update_ns = 0
    camera_drop_count = 0

    # 重定位状态（localize 模式）
    localization_done = threading.Event()
    localized_pose_holder: List[Optional[vslam.Pose]] = [None]
    localize_requested = False

    # 跟踪健康度统计（滑动窗口）
    health_obs_history: Deque[int] = deque(maxlen=HEALTH_WINDOW_FRAMES)
    health_fail_history: Deque[int] = deque(maxlen=HEALTH_WINDOW_FRAMES)
    last_health_print = 0.0

    def emit_health() -> None:
        nonlocal last_health_print
        now = time.monotonic()
        if now - last_health_print >= HEALTH_PRINT_INTERVAL_S:
            print_tracking_health(health_obs_history, health_fail_history)
            last_health_print = now

    try:
        while not stop_event.is_set():
            frames = ir_pipe.wait_for_frames(100)
            if frames is None:
                continue

            left_frame = frames.get_frame(OBFrameType.LEFT_IR_FRAME)
            right_frame = frames.get_frame(OBFrameType.RIGHT_IR_FRAME)
            if left_frame is None or right_frame is None:
                continue

            current_timestamp = int(left_frame.get_timestamp_us() * 1000)

            # 深度转发：填洞后写共享内存给 NavSide（VAE/SRU）。depth 与 IR 同一 FrameSet，帧同步。
            if depth_writer is not None:
                depth_frame = frames.get_depth_frame()
                if depth_frame is not None:
                    try:
                        depth_ts = int(depth_frame.get_timestamp_us() * 1000)
                        if depth_hole_filter is not None:
                            depth_frame = depth_hole_filter.process(depth_frame)
                        depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                        depth_data = depth_data[: RESOLUTION[1] * RESOLUTION[0]].reshape(
                            (RESOLUTION[1], RESOLUTION[0])
                        )
                        depth_writer.write(depth_data, depth_ts)
                    except Exception as e:  # noqa: BLE001
                        print(f"Warning: depth forward failed: {e}")

            # 相机流间隔检查
            if thread_with_timestamp.prev_low_rate_timestamp is not None:
                timestamp_diff = current_timestamp - thread_with_timestamp.prev_low_rate_timestamp
                if timestamp_diff > thread_with_timestamp.low_rate_threshold_ns:
                    camera_drop_count += 1
                    if camera_drop_count % 100 == 1:
                        print(
                            f"Warning: Camera stream message drop: timestamp gap "
                            f"({timestamp_diff/1e6:.2f} ms) exceeds threshold "
                            f"{thread_with_timestamp.low_rate_threshold_ns/1e6:.2f} ms "
                            f"({camera_drop_count} total)"
                        )
            thread_with_timestamp.prev_low_rate_timestamp = current_timestamp

            left_img = process_ir_frame(left_frame, target_size=SLAM_RESOLUTION)
            right_img = process_ir_frame(right_frame, target_size=SLAM_RESOLUTION)
            if left_img is None or right_img is None:
                print("Warning: Failed to convert IR frames; skipping frame pair")
                continue

            images = (left_img, right_img)

            last_tracker_timestamp = register_imu_until(
                tracker, imu_queue, pending_imu, current_timestamp, last_tracker_timestamp
            )
            odom_pose_estimate, slam_pose = tracker.track(current_timestamp, images, internals=internals)
            last_tracker_timestamp = current_timestamp

            odom_pose_with_cov = odom_pose_estimate.world_from_rig
            if odom_pose_with_cov is None:
                health_fail_history.append(1)
                health_obs_history.append(0)
                print(f"Tracking failed at frame {current_timestamp}")
                emit_health()
                continue
            health_fail_history.append(0)

            # 首帧跟踪成功后发起重定位（仅一次）。发起后本循环继续调用 track()，
            # 满足 async 模式下「定位期间持续喂图」的要求；结果由 finish_cb 异步写入。
            if mode == "localize" and not localize_requested:
                start_localization(
                    tracker, current_timestamp, images, localization_done, localized_pose_holder, map_name
                )
                localize_requested = True

            observations = tracker.get_last_observations(0)
            health_obs_history.append(len(observations))
            emit_health()
            gravity = tracker.get_last_gravity() if show_gravity else None

            # 读取 SLAM 平滑轨迹（回环后回溯矫正历史位姿），按节流间隔刷新。
            # 需与 track 同线程调用（满足 Slam 单线程要求）。
            slam_poses = None
            if slam_pose is not None and (
                current_timestamp - last_trajectory_update_ns
            ) >= MAP_UPDATE_INTERVAL_MS * 1e6:
                slam_poses = tracker.get_all_slam_poses()
                last_trajectory_update_ns = current_timestamp

            # 读取建图数据（与 track 同线程，满足 Slam 单线程要求），按节流间隔刷新。
            map_landmarks = None
            loop_closure_poses = None
            if ENABLE_MAPPING_VISUALIZATION and (
                current_timestamp - last_map_update_ns
            ) >= MAP_UPDATE_INTERVAL_MS * 1e6:
                map_landmarks = tracker.get_slam_landmarks(vslam.Tracker.SlamDataLayer.Map)
                loop_closure_poses = tracker.get_loop_closure_poses()
                last_map_update_ns = current_timestamp

            result_queue.put(
                [
                    current_timestamp,
                    odom_pose_with_cov.pose,
                    images,
                    observations,
                    gravity,
                    slam_pose,
                    slam_poses,
                    map_landmarks,
                    loop_closure_poses,
                    localized_pose_holder[0],
                ]
            )
            thread_with_timestamp.last_low_rate_timestamp = current_timestamp
    except Exception as e:
        print(f"Camera thread error: {e}")


def configure_ir_streams(pipeline: Pipeline, config: Config) -> None:
    """在 pipeline 的配置中启用 LEFT_IR + RIGHT_IR 流。"""
    for sensor_type in [OBSensorType.LEFT_IR_SENSOR, OBSensorType.RIGHT_IR_SENSOR]:
        profile_list = pipeline.get_stream_profile_list(sensor_type)
        profile = None
        for fmt in [OBFormat.Y8, OBFormat.Y16]:
            try:
                profile = profile_list.get_video_stream_profile(RESOLUTION[0], RESOLUTION[1], fmt, FPS)
                break
            except Exception as e:
                print(f"Warning: Failed to get stream profile for {sensor_type}: {e}")
                continue
        if profile is None:
            profile = profile_list.get_default_video_stream_profile()
            print(
                f"Using default profile for {sensor_type}: "
                f"{profile.get_width()}x{profile.get_height()} @ {profile.get_fps()} FPS"
            )
        config.enable_stream(profile)


def configure_depth_stream(pipeline: Pipeline, config: Config) -> None:
    """在 pipeline 配置中启用 DEPTH 流（与 IR 同分辨率同帧率，R0 实测强制）。"""
    profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
    profile = profile_list.get_video_stream_profile(RESOLUTION[0], RESOLUTION[1], OBFormat.Y16, FPS)
    if profile is None:
        profile = profile_list.get_default_video_stream_profile()
        print(
            f"Using default depth profile: "
            f"{profile.get_width()}x{profile.get_height()} @ {profile.get_fps()} FPS"
        )
    config.enable_stream(profile)


def get_hole_filling_filter(pipeline: Pipeline):
    """获取 Depth 传感器的 HoleFillingFilter。

    2.0.10 无 FilterFactory，只能从 get_recommended_filters() 里按类型挑；且该版本
    的 Filter 没有 set_config_value，只能用 SDK 默认填洞模式（与 NavSide 2.1.2 的
    mode=2 是否一致，联调时对拍确认）。
    """
    try:
        device = pipeline.get_device()
        sensor = device.get_sensor(OBSensorType.DEPTH_SENSOR)
        for f in (sensor.get_recommended_filters() or []):
            if f is None:
                continue
            if f.is_hole_filling_filter():
                f.enable(True)
                return f
    except Exception as e:  # noqa: BLE001
        print(f"Warning: could not get HoleFillingFilter: {e}")
    return None


def main() -> None:
    """使用 Orbbec Gemini 336L 进行双目惯性 VIO + SLAM 跟踪，可实时可视化建图。"""
    parser = argparse.ArgumentParser(description="Orbbec Gemini 336L VIO + SLAM (mapping / localization)")
    parser.add_argument(
        "--mode",
        choices=["map"],
        default="map",
        help="map = 建图并保存；localize = 载入地图重定位后继续跟踪",
    )
    parser.add_argument(
        "--map",
        dest="map_name",
        default=None,
        help="地图名（脚本目录下的 <name> 子文件夹）；不指定则在交互式菜单中选择",
    )
    parser.add_argument(
        "--delete",
        action="store_true",
        help="进入删除地图菜单后退出（不启动相机）",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="不启动 Rerun Viewer GUI（无显示器 / SSH 无 X 转发时用），仅落盘 .rrd 供离线查看",
    )
    args = parser.parse_args()
    mode: str = args.mode

    # --delete：仅删除地图，不启动相机。
    if args.delete:
        delete_map_menu()
        return

    # 解析地图名：命令行 --map 优先，否则按模式交互式选择。
    if args.map_name:
        map_name = args.map_name
        # map 模式下直接覆盖已有地图需确认（与交互式保存一致，防误删）。
        if mode == "map" and map_name in list_maps():
            confirm = input(f"地图 '{map_name}' 已存在，覆盖将丢失旧数据。确认覆盖？[y/N]: ").strip().lower()
            if confirm != "y":
                print("已取消。")
                return
    elif mode == "localize":
        selected = select_map_for_load()
        if selected is None:
            return
        map_name = selected
    else:
        map_name = select_map_for_save()

    map_path = map_dir_path(map_name)

    if mode == "localize" and not USE_SLAM_MODE:
        raise RuntimeError("localize 模式需要启用 SLAM（USE_SLAM_MODE=True）")
    if mode == "localize" and not os.path.isdir(map_path):
        raise RuntimeError(f"地图文件夹不存在: {map_path}，请先以 --mode map 建图")

    # --- IR 流水线（LEFT_IR + RIGHT_IR）---
    ir_pipeline = open_camera_pipeline(CAMERA_A_SERIAL)
    ir_config = Config()

    sensor_types = {
        ir_pipeline.get_device().get_sensor_list().get_type_by_index(i)
        for i in range(ir_pipeline.get_device().get_sensor_list().get_count())
    }
    if not STEREO_SENSORS.issubset(sensor_types):
        raise RuntimeError("Device does not support dual IR sensors (LEFT_IR + RIGHT_IR required)")

    original_laser_control = None
    if DISABLE_IR_EMITTER:
        original_laser_control = disable_ir_emitter(ir_pipeline)
    configure_ir_streams(ir_pipeline, ir_config)
    if DEPTH_OUTPUT_ENABLED:
        configure_depth_stream(ir_pipeline, ir_config)
    try:
        ir_pipeline.enable_frame_sync()
    except Exception as e:
        print(f"Warning: Could not enable frame sync: {e}")

    ir_pipeline.start(ir_config)

    # 深度转发：填洞滤波器 + 共享内存写方（在流水线启动后取，确保传感器可用）。
    depth_hole_filter = None
    depth_writer = None
    if DEPTH_OUTPUT_ENABLED:
        depth_hole_filter = get_hole_filling_filter(ir_pipeline)
        depth_writer = DepthShmWriter(DEPTH_SHM_NAME, RESOLUTION[0], RESOLUTION[1])
        print(
            f"Depth forwarding enabled -> shared memory '{DEPTH_SHM_NAME}' "
            f"({RESOLUTION[0]}x{RESOLUTION[1]})"
        )

    # 双目标定（从已在运行的 IR 流水线读取）。
    print("Getting stereo calibration...")
    stereo_params = get_stereo_calibration(ir_pipeline)

    # 构建 VIO 配置（2 个相机 + 1 个 IMU）。SLAM 输入降采样到 SLAM_RESOLUTION，内参同步缩放。
    rig = get_orbbec_vio_rig(stereo_params, target_size=SLAM_RESOLUTION)

    # 配置 tracker 用于双目惯性 VIO。
    cfg = vslam.Tracker.OdometryConfig(
        async_sba=True,
        enable_final_landmarks_export=True,
        enable_observations_export=True,
        debug_imu_mode=False,
        odometry_mode=vslam.Tracker.OdometryMode.Inertial,
        rectified_stereo_camera=True,
        use_denoising=True,
        use_gpu=True,
    )

    # localize 模式强制同步 SLAM：localize_in_map 及其回调都在相机线程内同步完成，
    # 避免后台 SLAM 线程回调（nb::gil_scoped_acquire）与相机线程的 pyorbbecsdk
    # wait_for_frames 并发抢 GIL 导致 "PyThreadState_Get ... GIL is released" 崩溃。
    # 代价是 SLAM 在 track 线程内做位姿图优化（会阻塞 track），但定位时相机静止，可接受。
    slam_sync_mode = SLAM_SYNC_MODE or (mode == "localize")
    slam_cfg = (
        vslam.Tracker.SlamConfig(
            sync_mode=slam_sync_mode,
            planar_constraints=PLANAR_CONSTRAINTS,
            throttling_time_ms=LOOP_CLOSURE_THROTTLING_MS,
            use_gpu=True,
            max_map_size=MAX_MAP_SIZE,
        )
        if USE_SLAM_MODE
        else None
    )

    tracker = vslam.Tracker(rig, cfg, slam_cfg)

    internals = vslam.Tracker.Internals()
    internals.num_desired_tracks = NUM_DESIRED_TRACKS

    # --- IMU 流水线（ACCEL + GYRO），按 SDK 建议与视频分开 ---
    imu_pipeline = open_camera_pipeline(CAMERA_A_SERIAL)
    imu_config = Config()
    # 使用设备默认的量程 / 采样率（与 SDK 的 07_imu.py 示例一致）。
    imu_config.enable_accel_stream()
    imu_config.enable_gyro_stream()

    # 队列与可视化器（在流水线之前启动 Rerun，避免 fd 继承）。
    q = queue.Queue()
    imu_queue = queue.Queue(maxsize=IMU_QUEUE_MAX_SIZE)
    visualizer = MappingVisualizer(
        image_size=SLAM_RESOLUTION,
        show_gravity=SHOW_GRAVITY,
        show_mapping=ENABLE_MAPPING_VISUALIZATION,
        spawn=not args.no_viz,  # 默认启动 Rerun Viewer（wheel 内置 viewer 二进制）；--no-viz 时仅落盘
        save_path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "vio_mapping_tracking.rrd"),
    )
    thread_with_timestamp = ThreadWithTimestamp(IMAGE_JITTER_THRESHOLD_NS, IMU_JITTER_THRESHOLD_NS)

    # 位姿出口：UDP 位姿发送器，把全局位姿发给同机下游 RobotComm（VAE/SRU）。
    pose_sender = None
    if POSE_UDP_OUTPUT_ENABLED:
        pose_sender = UdpPoseSender(host=POSE_UDP_HOST, port=POSE_UDP_PORT)

    imu_pipeline.start(imu_config)

    # 采集线程退出标志：主线程在 finally 里置位它，让 IMU/相机线程干净退出，
    # 避免进程退出 / C++ 对象析构时这些线程仍在访问 Pipeline/Tracker 导致 SIGABRT。
    stop_event = threading.Event()
    imu_thread_obj = threading.Thread(
        target=imu_thread, args=(imu_queue, thread_with_timestamp, imu_pipeline, stop_event), daemon=True
    )
    camera_thread_obj = threading.Thread(
        target=camera_thread,
        args=(
            tracker, q, imu_queue, thread_with_timestamp, ir_pipeline, SHOW_GRAVITY, internals,
            mode, map_name, stop_event, depth_hole_filter, depth_writer,
        ),
        daemon=True,
    )
    imu_thread_obj.start()
    camera_thread_obj.start()

    frame_id = 0
    trajectory_slam: List[np.ndarray] = []
    localized_printed = False
    last_global_pose: Optional[vslam.Pose] = None  # 最近一次全局位姿（地图系），退出时保存
    last_pose_print = 0.0  # 实时坐标打印节流时间戳（秒，time.monotonic）

    print("Starting VIO+SLAM tracking with cuvslam...")
    print(f"Mode: {mode}  |  Map: {map_name}  ({map_path})")
    print(f"Mapping visualization: {'ON' if ENABLE_MAPPING_VISUALIZATION else 'OFF'}")
    print("Press Ctrl+C to stop")

    try:
        while True:
            try:
                (
                    timestamp,
                    odom_pose,
                    images,
                    observations,
                    gravity,
                    slam_pose,
                    slam_poses,
                    map_landmarks,
                    loop_closure_poses,
                    localized_pose,
                ) = q.get(timeout=1.0)
            except queue.Empty:
                continue

            if odom_pose is None:
                continue

            # 重定位成功后打印一次当前全局位姿（后续 slam_pose 即该全局系下的位姿）。
            if localized_pose is not None and not localized_printed:
                localized_printed = True
                print(f"[localize] done. Global pose: {format_pose(localized_pose)}")

            # 记录最近一次全局位姿（地图系），退出时保存供下次 localize 作 guess。
            if slam_pose is not None:
                last_global_pose = slam_pose
            elif localized_pose is not None:
                last_global_pose = localized_pose

            # 重定位完成后，按节流间隔实时打印当前全局位姿（xyz + 航向），
            # 便于人工核对实时定位；也是后续给 VAE/SRU 的位姿出口的直接参考。
            if localized_printed or mode == "map":
                pose_to_print = slam_pose if slam_pose is not None else localized_pose
                if pose_to_print is not None:
                    now = time.monotonic()
                    if now - last_pose_print >= POSE_PRINT_INTERVAL_S:
                        last_pose_print = now
                        print(f"[global] {format_pose(pose_to_print)}")

            # 位姿出口：把全局位姿经 UDP 发给下游 VAE/SRU（下游自己算速度/重力）。
            # map 模式 slam_pose 从首帧起即全局系；localize 模式需定位完成后才是全局系。
            if pose_sender is not None:
                pub_pose = None
                if mode == "map":
                    pub_pose = slam_pose
                elif localized_printed:
                    pub_pose = slam_pose if slam_pose is not None else localized_pose
                if pub_pose is not None:
                    pose_sender.send_pose(pub_pose.translation, pub_pose.rotation)

            frame_id += 1
            # 用 SLAM 平滑轨迹（回环后回溯矫正）替换逐帧追加，避免回环跳变。
            if slam_poses is not None:
                trajectory_slam = [p.pose.translation for p in slam_poses]

            # 一旦 SLAM 优化结果可用，优先使用它。
            viz_pose = slam_pose if slam_pose is not None else odom_pose

            visualizer.visualize_frame(
                frame_id=frame_id,
                images=[images[0]],
                pose=viz_pose,
                observations_main_cam=[observations],
                trajectory=trajectory_slam,
                timestamp=timestamp,
                gravity=gravity,
                map_landmarks=map_landmarks,
                loop_closure_poses=loop_closure_poses,
            )

    except KeyboardInterrupt:
        print("Stopping VIO+SLAM tracking...")
        # 建图模式：保存地图前引导用户原地环绕采样，提升重定位航向鲁棒性。
        if mode == "map" and ROTATIONAL_SURVEY_ENABLED:
            rotational_survey()
    finally:
        # 清理期间忽略 SIGINT，避免第二次 Ctrl+C 中断流水线关闭导致 std::terminate。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        # 先让采集线程退出：置位停止标志 + 停止流水线（解除 wait_for_frames 阻塞），
        # 再 join 等线程真正结束。否则线程在进程退出 / C++ 对象析构时仍在访问
        # Pipeline/Tracker，会触发 "terminate called without an active exception"（SIGABRT）。
        stop_event.set()
        imu_pipeline.stop()
        ir_pipeline.stop()
        restore_ir_emitter(ir_pipeline, original_laser_control)
        imu_thread_obj.join(timeout=5.0)
        camera_thread_obj.join(timeout=5.0)
        if imu_thread_obj.is_alive() or camera_thread_obj.is_alive():
            print("WARNING: 采集线程未在 5s 内退出（仍为 daemon，将随进程结束）")
        if pose_sender is not None:
            pose_sender.close()
        if depth_writer is not None:
            depth_writer.close()
        print("Pipelines stopped.")
        # 建图模式下退出前保存全局地图；重定位模式不覆盖参考地图。
        if mode == "map":
            save_map_and_wait(tracker, map_path)
        # 两种模式都保存最后全局位姿，供下次 localize 作初始 guess。
        if last_global_pose is not None:
            save_last_pose(last_global_pose, map_name)
        visualizer.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), "vio_mapping_tracking.rrd"))
        print("Clean shutdown complete.")


if __name__ == "__main__":
    main()
