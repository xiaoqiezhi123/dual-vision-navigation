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

# =============================================================================
# run_vio_tasknav.py —— 任务导航版：全程纯 map 跑 + 任务点 localize 锚定
#
# 与 run_vio.py（map/localize 两模式）的关键区别：完全不用 localize 的实时定位。
# 行进全程都是 map 模式（异步 SLAM、回环、平面约束、发位姿，与 run_vio_mapnav.py
# 一致）；参考地图只用于「任务点」的一次性重定位锚定：
#
#   1. 用 run_vio.py --mode map 建好参考地图（如 --map 12）。
#   2. 本脚本启动后（AUTO_ANCHOR_AT_START=True）首帧成功即自动执行一次 localize，
#      把坐标系对齐到参考地图全局系；之后纯 map 行进并发位姿（全局系）。
#   3. 机器人到达任务点、静止下来后：终端提示到达并暂停位姿发送，输入 localize 回车：
#      - 相机线程/IMU 线程暂停（规避后台 SLAM 回调与 pyorbbecsdk 并发抢 GIL 崩溃）
#      - 用参考地图执行由粗到细三级 localize_in_map（异步排队，safe）
#      - 成功后 SLAM 内部地图被参考地图替换，恢复 track 后 slam_pose 即全局系位姿
#   4. 该全局位姿即下一段子任务的起点，继续纯 map 行进并发位姿（全局系）。
#   5. 每到一个任务点重复输入 localize 重新锚定（漂移被每段清零）。
#   6. 退出时不保存地图（不污染参考地图）；仅锚定成功后保存 last-pose 供下次 guess。
#
# 终端指令（随时可输入）：
#   localize / l  在当前位置（需静止）用参考地图重定位锚定
#   pause / p     挂起相机/IMU 采集并停发位姿（调度器模式专用）
#   resume / r    恢复采集与位姿输出
#   help / h      打印指令帮助
#   quit / q      优雅退出（等价 Ctrl+C，不保存地图）
#
# 用法：
#   CUVSLAM_CAMERA_SERIAL=CPC8763000J0 ./run_orbbec_tasknav.sh --ref-map 12
#   （run_orbbec_tasknav.sh = run_orbbec.sh 的脚本名替换版，见同目录）
# =============================================================================

import argparse
import os
import queue
import shutil
import signal
import sys
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
from foxglove_odom_server import opencv_pose_to_zup
from udp_pose_sender import UdpPoseSender
from pose_result_queue import LatestPoseQueue, PoseResult
from vio_handoff import AnchorHandoff, TrackerImuBuffer, wait_while_paused
from pose_trace_hook import get_trace
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
FPS = 10  # run_vio_mapnav.py 实测值（人形机器人）；run_vio.py 用 15

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
# run_vio_mapnav.py 实测用 3.0（更不信 IMU，抑制人形机器人步行冲击下的漂移）。
IMU_NOISE_INFLATION = 3.0
IMU_GYROSCOPE_NOISE_DENSITY = 6.0673370376614875e-03 * IMU_NOISE_INFLATION
IMU_GYROSCOPE_RANDOM_WALK = 3.6211951458325785e-05 * IMU_NOISE_INFLATION
IMU_ACCELEROMETER_NOISE_DENSITY = 3.3621979208052800e-02 * IMU_NOISE_INFLATION
IMU_ACCELEROMETER_RANDOM_WALK = 9.8256589971851467e-04 * IMU_NOISE_INFLATION

FRAME_PERIOD_MS = 1000 / FPS
IMAGE_JITTER_THRESHOLD_NS = (FRAME_PERIOD_MS + 5) * 1e6  # 相机帧间隔容差
IMU_JITTER_THRESHOLD_NS = 20 * 1e6  # IMU 采样间隔容差（~200 Hz -> ~5 ms）
IMU_QUEUE_MAX_SIZE = IMU_FREQUENCY * 5  # 最多缓冲约 5 s 的 IMU 数据

SHOW_GRAVITY = False  # 可视化估计的重力向量

# 方案一：SLAM 开关不再用全局常量控制——由 make_tracker(slam, sync) 按场景决定：
#   行进 = make_tracker(slam=False)   纯 VIO，不建图（无 landmarks/位姿图/回环）
#   定位 = make_tracker(slam=True, sync=True)  锚定瞬间临时启用 SLAM（同步，防 GIL 崩溃）
SLAM_SYNC_MODE = False  # 行进默认异步：SLAM 后端在后台线程跑，不阻塞采集
# 定位不用这个常量——锚定流程会临时重建一个 sync_mode=True 的 tracker
# （见锚定状态机 make_tracker）。两难实测（2026-09-14/15）：
#   全程 sync：localize 从不崩 GIL，但行进中后端内联工作饿死采集线程，
#     位姿流卡顿 0.5s~125s（IMU/相机大量掉帧）；
#   全程 async：行进流畅，但 localize 回调在后台线程触发，绑定层 GIL
#     保护缺失 → 随机硬崩溃。
# 折中：async 行进 + sync 定位（锚定前后各重建一次 tracker）。
PLANAR_CONSTRAINTS = True  # 将 SLAM 位姿约束到平面（平面运动时设为 True）
LOOP_CLOSURE_THROTTLING_MS = 3600000  # 回环节流间隔（ms）；0 = 不限制
# 任务导航禁用回环（1h 间隔 = 会话内不触发）。原因（2026-09-15 实测）：
# SLAM_SYNC_MODE=True 下回环检测/PGO 在 track() 内联执行，主线程持 GIL
# 数秒~数十秒（实测单次 21~27s），相机/IMU 采集线程拿不到 GIL 无法消费
# 帧流 → SDK 缓冲溢出（实测 IMU 掉 39.8s、相机掉 3.4s）→ 行进中位姿流
# 卡住，停车后回环结束才恢复。本脚本每段靠任务点锚定重置全局系、
# 段内位姿用 odom 帧间增量累积，本就不依赖回环。

# ---- 任务导航模式（本脚本专用）：全程纯 map + 任务点 localize 锚定 ----
# 本脚本固定 map 模式行进（异步 SLAM、回环、平面约束、发位姿）；「任务点」静止后
# 由终端指令触发一次 localize_in_map，把当前 SLAM 地图替换为参考地图（全局系）。
# 锚定成功后继续 map 行进，slam_pose 即全局系位姿；每段漂移在下一个任务点重新清零。
# 完全不用 run_vio.py 的 --mode localize 持续实时定位。
MODE = "map"

# 参考地图名：任务点 localize 锚定的目标地图（只读，本脚本退出时不写回）。
# 用 run_vio.py --mode map --map <同名> 建图。可用 --ref-map 命令行覆盖。
REF_MAP_NAME = os.environ.get("CUVSLAM_REF_MAP", "orbbec_map")

# 锚定前（本地系）是否也经 UDP 发位姿。
# True  = 启动即发（本地系位姿，与 run_vio_mapnav.py 一致）；False = 只在锚定后发全局系位姿。
SEND_POSE_BEFORE_ANCHOR = True

# 位姿出口模式（锚定后生效）：任务点之间用纯 VIO 里程计，不用 SLAM 后端位姿。
# True  = 段内发/打「odom 帧间增量累积」位姿（纯 VIO 前端，回环/PGO/平面约束都不干预；
#         漂移随段长累积，每个任务点锚定后重置为零——锚定只做坐标系对齐与漂移清零）。
#         对 odom 坐标系在 localize 后的可能跳变免疫（只依赖帧间增量）。
# False = 段内发 slam_pose（SLAM 后端优化结果，回环后历史轨迹会被回溯矫正）。
USE_ODOM_FOR_POSE_OUTPUT = True

# 任务点锚定的由粗到细三级搜索半径：
#   首次锚定（本地系与全局系无关）：用下方大半径覆盖全图；
#   已锚定后再锚定（漂移小）：用 ANCHORED 小半径，速度更快。
LOCALIZE_COARSE_H_RADIUS_FIRST = 25.0   # 首次锚定水平半径（米）
LOCALIZE_COARSE_H_RADIUS_ANCHORED = 25.0  # 已锚定后的水平半径（米）
# 曾用 5.0m（假设锚定后漂移小）：实测台架/真机上「到达后推车到下一任务点」
# 的位移可达任务点间距（5~11m），5m 盒会漏掉真值导致锚定失败。
# 与 run_vio.py --mode localize（始终 25m、从不漏定位）对齐。

# ---- 任务点列表（数量与位置都在这里配置）----
# 每个任务点 = 参考地图全局系下的坐标 (x, y, z)（米），即 run_vio.py --mode map
# 建图时的世界系。注意 cuVSLAM 世界系是 OpenCV 约定（+X 右、+Y 下、+Z 前），
# 竖直轴是 Y（向下），水平面是 XZ——所以任务点写 (x, 高度, z)，到达判定用 XZ 平面距离。
# 获取方法：把机器人推到目标位置静止，手动 localize 一次，把终端打印的 [global]
# 坐标抄进来；或从建图录制的 .rrd 轨迹读数；或用 map2d_picker.py 在 2D 地图上选点。
# y（高度）一般与建图时的地面高度一致（抄同一次运行的任一 [global] 打印值即可）。
# 到达检测：锚定后位姿即全局系，直接与任务点坐标比较。启动自动锚定
# （AUTO_ANCHOR_AT_START）在首帧后自动对齐全局系，第一个任务点也能正常检测。
# 空列表 [] = 关闭自动任务点，回到纯手动 localize 模式。
TASK_POINTS = [
     (0, 0, 2.0),        # 任务点 1（手动锚定；此坐标用于完成后校验/推进）
    (0.0, 0.0, 5.0),  # 任务点 2（自动：到达+静止后锚定）
    # (6.0, 0.0, 0.0),  # 任务点 3
    # (6.0, 2.0, 0.0),  # 任务点 4 …… 想加几个加几行
]
TASK_ARRIVE_RADIUS_M = 0.5   # 距任务点小于该距离判定「到达」
TASK_STATIC_WINDOW_S = 2.0   # 静止判定时间窗（秒）
TASK_STATIC_DISP_M = 0.05    # 该时间窗内总位移小于该值判定「静止」（米）
TASK_AUTO_ANCHOR = True     # True = 到达+静止自动锚定；False = 仅提示，等待手动输入 localize
TASK_ANCHOR_TOLERANCE_M = 1.0  # 锚定结果与任务点坐标偏差大于该值判为异常并警告（米）
# 到达任务点后暂停 UDP 位姿发送（等待锚定确认），锚定完成推进下一个任务点后恢复。
TASK_PAUSE_POSE_ON_ARRIVE = True
# 未锚定（本地系）时是否也做任务点到达检测。
# 默认 False：启动后由 AUTO_ANCHOR_AT_START 自动锚定对齐全局系，所有任务点都在
# 全局系下检测，不需要本地系近似。若关闭启动自动锚定，可改回 True 作回退
# （前提：机器人每次从建图原点、以建图时相同朝向启动，本地系≈全局系）。
TASK_DETECT_IN_LOCAL_FRAME = False
# 启动自动锚定：首帧跟踪成功后自动执行一次 localize_in_map，把坐标系对齐到参考
# 地图全局系（机器人需静止在起点）。对齐后所有任务点（含第一个）都能按全局坐标
# 检测到达。锚定失败不重试，继续本地系行进，可随时手动输入 localize。
AUTO_ANCHOR_AT_START = True
STARTUP_LOCALIZE_H_RADIUS = 25.0  # 启动自动锚定的 coarse 搜索半径（米）
# 曾用 10.0m（假设「起点位置已知」）：实测机器人常被推到离 last-pose 猜测点
# 10m 以外，10m 盒漏掉真值 → 启动锚定失败；25m 与 run_vio.py localize 模式
# 一致（全图搜索），代价是 coarse 阶段多几秒，仍在 180s 超时内。

ANCHOR_WARMUP_FRAMES = 10  # 锚定前预热帧数（≈1s @10fps）
# 预热让锚定 tracker 初始化重力/IMU 姿态后再 localize：全新 tracker 直接
# localize_in_map 在部分位置匹配失败（实测 z=7 连败）；单帧不一定够重力
# 收敛，多帧对齐小推车稳定版「先跟踪后定位」的行为。

ANCHOR_RETRY_COOLDOWN_S = 10.0  # 锚定成功后再次 localize 的最小间隔（秒）
# 锚定刚完成时 SLAM 后台线程仍在收尾（地图替换/异步队列排空），立即再次
# localize_in_map 会撞上绑定层的 GIL 竞态崩溃（实测：Fatal Python error:
# PyThreadState_Get ... GIL is released）。任务导航流程中相邻两次锚定间隔
# 远超 10s，此冷却对正常使用无感，只挡住会触发崩溃的背靠背 localize。

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

NUM_DESIRED_TRACKS = 800  # 每帧期望的特征跟踪数量（run_vio_mapnav.py 实测值）

# 跟踪健康度提示：滑动窗口内统计特征数与失败率，在卡住 / 漂移之前预警。
HEALTH_PRINT_INTERVAL_S = 1.0      # 健康提示打印节流（秒）
ENABLE_HEALTH_PRINT = False        # 是否打印 [health] 跟踪健康度（False = 关闭）
HEALTH_WINDOW_FRAMES = 30          # 滑动窗口帧数（15fps 下约 2s）
FEATURE_COUNT_WARN_THRESHOLD = 100  # 单帧特征数低于此值判 WARN
FEATURE_COUNT_LOST_THRESHOLD = 30   # 单帧特征数低于此值判 LOST（即将丢跟踪）

POSE_PRINT_INTERVAL_S = 0.1  # 重定位完成后实时坐标打印节流（秒）；0.1=10Hz, 0.067≈15Hz
POSE_MAX_RESULT_AGE_S = 0.5  # 从 SDK 取到帧到主循环消费超过此时限，不把旧结果当新位姿发送

# ---- 位姿输出（给下游 VAE/SRU）：以 UDP 发送位姿包给同机下游 RobotComm ----
# 下游 bridge.py（/home/amov/Desktop/bridge.py）RobotComm 用 pose_transport=udp 接收，
# 收到位姿后自己差分算速度/重力、z=0.695。本模块只传位姿，不做速度/重力。
# 位姿包：struct "<7d" = [px,py,pz, qw,qx,qy,qz]（Z-up 世界系，四元数 wxyz）。
POSE_UDP_OUTPUT_ENABLED = True
POSE_UDP_HOST = "127.0.0.1"  # 同机 loopback，发给下游 RobotComm
POSE_UDP_PORT = 8082

# 调度器事件文件通道（task_nav_scheduler.py 注入 SLAM_SCHED_FILE 环境变量）。
# [SCHED] 行双写：stdout（管道，兼容旧路径）+ 文件（调度器 tail 解析的主通道）。
# 文件留痕便于排查（管道若丢行，文件里能看到 cuvslam 到底输出了什么）。
SLAM_SCHED_FILE = os.environ.get("SLAM_SCHED_FILE", "")


def sched_emit(text: str) -> None:
    """[SCHED] 事件双通道输出：stdout + SLAM_SCHED_FILE（追加写，立即落盘）。"""
    print(text, flush=True)
    if SLAM_SCHED_FILE:
        try:
            with open(SLAM_SCHED_FILE, "a", encoding="utf-8") as f:
                f.write(text + "\n")
        except OSError:
            pass


def diag_emit(text: str) -> None:
    """诊断信息双出口：直落 slam 日志 + 经 [SCHED] note= 显示在调度器终端。

    调度器把 note 行解析为 [TASK] SLAM: <文本> 打印——用户盯着调度器终端
    就能看到定位失败原因与埋点计时（不必去翻 slam 日志）。
    """
    print(text, flush=True)
    sched_emit(f"[SCHED] note={text}")

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

    串号字符串描述符偶发读取超时（UsbEnumeratorLibusb: Operation timed out）
    会导致 get_device_by_serial_number 抛 OBError——加重试 + 按索引遍历兜底
    （enumerate_devices.py 证实索引路径能稳定读到串号）。
    """
    if not serial:
        return Pipeline()
    ctx = Context()  # 保持引用，防止临时 Context 被 GC 后 deviceMgr 悬空（与 NavSide 同款坑）
    device = None
    last_err: Exception | None = None
    # 重试 15 次 × 1s：调度器崩溃重启会重开相机，Orbbec USB 栈释放/重新枚举
    # 需要时间，立即重开常出现 Device not found / 标定取帧失败（实测）。
    for _attempt in range(15):
        devices = ctx.query_devices()
        try:
            device = devices.get_device_by_serial_number(serial)
        except Exception as exc:  # OBError: Device not found by serial number
            last_err = exc
            device = None
        if device is None:
            try:
                for i in range(devices.get_count()):
                    if devices.get_device_serial_number_by_index(i) == serial:
                        device = devices.get_device_by_index(i)
                        break
            except Exception as exc:
                last_err = exc
        if device is not None:
            break
        time.sleep(1.0)  # 等 USB 枚举稳定后重试
    if device is None:
        raise RuntimeError(
            f"未找到序列号 '{serial}' 的相机（当前已连接 {devices.get_count()} 台；"
            f"最后错误: {last_err}）。请重插该相机 USB（或重启）后重试。"
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
    pause_event: threading.Event,
    handoff: AnchorHandoff,
) -> None:
    """IMU 采集线程：读取加速度计 + 陀螺仪并写入 ImuSample 队列。

    Orbbec IMU 将加速度计和陀螺仪暴露为两个独立传感器，但由同一个物理 IMU 采样。
    SDK 可能把二者放在同一个 FrameSet 中，也可能分开返回，因此我们保留最新的陀螺仪值，
    每来一帧新的加速度计数据就输出一个 ImuSample（以加速度计作为时钟）。

    任务点 localize 锚定期间 pause_event 置位：线程挂起不取帧，避免后台 SLAM 线程
    回调（nb::gil_scoped_acquire）与 pyorbbecsdk wait_for_frames 并发抢 GIL 崩溃。
    """
    high_rate_threshold = thread_with_timestamp.high_rate_threshold_ns
    pose_trace = get_trace()
    prev_timestamp = None
    drop_count = 0
    queue_drop_count = 0
    last_gyro = None  # 最新的 (gx, gy, gz)
    last_gyro_timestamp = None

    try:
        while not stop_event.is_set():
            if wait_while_paused(pause_event, stop_event, handoff.imu_parked):
                last_gyro = None  # Do not pair a new accelerometer frame with pre-pause gyro data.
                last_gyro_timestamp = None
                continue
            imu_wait_started_ns = time.monotonic_ns()
            frames = imu_pipe.wait_for_frames(100)
            imu_received_ns = time.monotonic_ns()
            if frames is None:
                continue

            gyro_frame = frames.get_frame(OBFrameType.GYRO_FRAME)
            if gyro_frame is not None:
                gyro_frame = gyro_frame.as_gyro_frame()
                if gyro_frame is not None:
                    last_gyro = (gyro_frame.get_x(), gyro_frame.get_y(), gyro_frame.get_z())
                    if pose_trace.enabled:
                        last_gyro_timestamp = int(gyro_frame.get_timestamp_us()*1000)

            accel_frame = frames.get_frame(OBFrameType.ACCEL_FRAME)
            if accel_frame is not None:
                accel_frame = accel_frame.as_accel_frame()
            if accel_frame is None or last_gyro is None:
                continue

            current_timestamp = int(accel_frame.get_timestamp_us() * 1000)
            if pose_trace.enabled:
                pose_trace.emit('imu', source_timestamp_ns=current_timestamp, gyro_timestamp_ns=last_gyro_timestamp,
                    accel=[accel_frame.get_x(), accel_frame.get_y(), accel_frame.get_z()], gyro=list(last_gyro),
                    source_dt_s=(current_timestamp-prev_timestamp)/1e9 if prev_timestamp is not None else None,
                    queue_size=imu_queue.qsize(), queue_dropped=queue_drop_count)
                pose_trace.emit('imu_capture', source_timestamp_ns=current_timestamp,
                                wait_started_ns=imu_wait_started_ns, received_ns=imu_received_ns,
                                sdk_wait_s=(imu_received_ns-imu_wait_started_ns)/1e9)

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
        if not stop_event.is_set():
            handoff.fail(f'IMU thread error: {e}')


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
    pose_trace = get_trace()
    registered, skipped, first_imu, last_imu = 0, 0, None, None
    while True:
        try:
            pending_imu.append(imu_queue.get_nowait())
        except queue.Empty:
            break

    while pending_imu and pending_imu[0].timestamp_ns <= timestamp_ns:
        sample = pending_imu.popleft()
        if last_tracker_timestamp_ns is not None and sample.timestamp_ns < last_tracker_timestamp_ns:
            skipped += 1
            continue
        imu_measurement = vslam.ImuMeasurement()
        imu_measurement.timestamp_ns = sample.timestamp_ns
        imu_measurement.linear_accelerations = sample.linear_accelerations
        imu_measurement.angular_velocities = sample.angular_velocities
        tracker.register_imu_measurement(0, imu_measurement)
        registered += 1
        first_imu = sample.timestamp_ns if first_imu is None else first_imu
        last_imu = sample.timestamp_ns
        last_tracker_timestamp_ns = sample.timestamp_ns
    pose_trace.emit('imu_register', source_timestamp_ns=timestamp_ns, registered=registered, skipped_old=skipped,
        tracker_id=id(tracker),
        first_imu_ns=first_imu, last_imu_ns=last_imu, pending=len(pending_imu),
        image_to_last_imu_s=(timestamp_ns-last_imu)/1e9 if last_imu is not None else None)
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


def pose_panel_text(anchored: bool, localize_busy: bool, paused_by_cmd: bool,
                    slam_pose, panel_msg: str = "", out_pose=None) -> str:
    """调度器模式位姿面板：原地刷新（清屏重绘），终端只更新数字不刷屏。

    调度器模式下 stdout 走日志文件 + tail 观察窗口，ANSI 清屏码经终端解释后
    即「原地更新」效果；机器行 [SCHED] 由调度器读取且不写日志，面板保持干净。

    out_pose：UDP 实际发送的出口位姿（与发给 NavSide 的同源）。面板同时显示
    它的 Z-up 位置，方便与 NavSide 面板的 pos_w 直接比对：
    pos_w.x=Z-up.x（前）、pos_w.y=Z-up.y（左）、pos_w.z 恒为 0.695。
    """
    if localize_busy:
        state = "锚定中（localize 搜索）..."
    elif paused_by_cmd:
        state = "已暂停（等待调度器 resume）"
    elif anchored:
        state = "全局系（已锚定参考地图）"
    else:
        state = "本地系（未锚定）"
    pose_text = format_pose(slam_pose) if slam_pose is not None else "等待首帧位姿..."
    lines = [
        "=== cuVSLAM 位姿（原地刷新） ===",
        f"状态: {state}",
        f"位姿: {pose_text}",
    ]
    if out_pose is not None:
        try:
            p_zup, _ = opencv_pose_to_zup(out_pose.translation, out_pose.rotation)
            lines.append(f"Z-up: x={p_zup[0]:.3f}(前)  y={p_zup[1]:.3f}(左)  ← 发 NavSide ≈ pos_w")
        except Exception:
            pass
    if panel_msg:
        lines.append(f"消息: {panel_msg}")
    return "\033[H\033[J" + "\n".join(lines)


def quat_mul(q1, q2) -> np.ndarray:
    """四元数乘法（x, y, z, w 顺序，Hamilton 约定）。"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ], dtype=float)


def quat_rotate(q, v) -> np.ndarray:
    """用四元数 q 旋转向量 v（q 为单位四元数）。"""
    qv = np.asarray(q[:3], dtype=float)
    qw = float(q[3])
    t = 2.0 * np.cross(qv, v)
    return np.asarray(v, dtype=float) + qw * t + np.cross(qv, t)


def pose_compose(p1: vslam.Pose, p2: vslam.Pose) -> vslam.Pose:
    """位姿合成 p1 ⊗ p2（先 p2 后 p1）。"""
    q = quat_mul(p1.rotation, p2.rotation)
    t = np.asarray(p1.translation, dtype=float) + quat_rotate(p1.rotation, p2.translation)
    return vslam.Pose(rotation=q.tolist(), translation=t.tolist())


def pose_inverse(p: vslam.Pose) -> vslam.Pose:
    """位姿逆。"""
    q = np.asarray(p.rotation, dtype=float)
    q_inv = np.array([-q[0], -q[1], -q[2], q[3]])
    t = np.asarray(p.translation, dtype=float)
    return vslam.Pose(rotation=q_inv.tolist(), translation=(-quat_rotate(q_inv, t)).tolist())


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


def localize_anchor(
    tracker: vslam.Tracker,
    timestamp_ns: int,
    images: tuple,
    guess_slam_pose: Optional[vslam.Pose],
    anchored: bool,
    map_name: str,
    coarse_h_radius_override: Optional[float] = None,
    progress_cb=None,
) -> Optional[vslam.Pose]:
    """任务点锚定：用临时 SLAM tracker 由粗到细搜索参考地图的全局位姿。

    **必须**在主线程调用，且相机/IMU 线程已挂起（pause 协议已生效）：
      - async SLAM（sync_mode=False）下 localize_in_map 排队到后台 SLAM 线程执行，
        finish_cb 在后台线程回调（nb::gil_scoped_acquire）；
      - 本流程使用 sync_mode=True，并等待两个采集线程确认挂起，减少 SDK 调用
        与定位回调并发；不能据此保证原生库内部不会发生 GIL/线程问题。
      - 挂起期间没有新 track，SLAM 时间冻结在锚定帧，timestamp_ns 始终「当前」，
        不存在 retention_time_ms 过期问题。

    每级之间串行等待回调完成。guess 优先级：
      已锚定（anchored=True）→ 调用方提供的全局 XZ、Y=0、单位姿态猜测；
      首次锚定 → last-pose 文件 → 冷启动默认（大半径全图搜索）。
    任一级失败回退上一级结果；全部失败返回 None。

    返回位姿作为下一段的全局起点。临时定位 tracker 随后释放，相机线程创建
    新的纯 VIO tracker；新 VIO 首个有效帧建立局部基线，再累积到此全局起点。

    Returns:
        锚定成功返回全局位姿；失败返回 None。

    progress_cb：进度消息出口（默认 print）。调度器模式传入 panel_print，
    三级搜索提示（coarse/mid/fine 半径与步长）显示在面板消息行里。
    """
    if progress_cb is None:
        progress_cb = print
    # 绑定侧接收 std::vector<nb::ndarray>；显式转成 list 以避免 tuple 转换差异。
    images = list(images)

    def run_pass(guess_pose, h_radius, v_radius, h_step, v_step, a_step, label):
        """执行一次 localize_in_map 并等待回调返回 (pose, error_message)。"""
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

        progress_cb(f"[localize] {label}: H_radius={h_radius}m, h_step={h_step}m, a_step={a_step}rad ...")
        # start_cb 绑定要求 Callable（传 None 会 TypeError）。localize 偶发的
        # GIL 崩溃（Fatal Python error: PyThreadState_Get）由调度器的
        # 「崩溃自动重启 + 启动自动锚定兜底」机制覆盖。
        tracker.localize_in_map(map_dir_path(map_name), timestamp_ns, guess_pose, images, settings, lambda: None, finish_cb)
        inner_done.wait(timeout=180.0)  # async 排队 + GPU 网格搜索，首次全图搜索可能较慢
        if out["pose"] is None and out["error"] is None:
            progress_cb(f"[localize] {label}: timeout waiting for result")
        return out["pose"], out["error"]

    # 初始 guess：已锚定 → 当前 slam_pose（全局系）；否则 last-pose 文件；再否则冷启动默认。
    if anchored and guess_slam_pose is not None:
        guess = guess_slam_pose
        progress_cb(f"[localize] using current slam pose as initial guess: {format_pose(guess)}")
    else:
        guess = load_last_pose(map_name)
        if guess is not None:
            progress_cb(f"[localize] using saved last pose as initial guess: {format_pose(guess)}")
        else:
            guess = vslam.Pose(
                rotation=list(LOCALIZE_GUESS_ROTATION), translation=list(LOCALIZE_GUESS_TRANSLATION)
            )
            progress_cb("[localize] no saved last pose; using cold-start origin guess")

    # 首次锚定本地系与全局系无关，用大半径全图搜索；已锚定后漂移小，小半径快搜。
    # 调用方可覆盖（如启动自动锚定：起点位置已知，用较小半径）。
    if coarse_h_radius_override is not None:
        coarse_h_radius = coarse_h_radius_override
    else:
        coarse_h_radius = LOCALIZE_COARSE_H_RADIUS_ANCHORED if anchored else LOCALIZE_COARSE_H_RADIUS_FIRST

    # 三级由粗到细定位，每级以上一级结果为 guess。
    stages = [
        ("coarse", coarse_h_radius, LOCALIZE_COARSE_V_RADIUS,
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
                progress_cb(f"[localize] {label} failed ({err}); using previous-stage pose: {format_pose(last_pose)}")
                return last_pose
            progress_cb(f"Localization failed ({label}): {err}")
            # 双出口：日志留痕 + 调度器终端显示
            diag_emit(f"[localize] FAILED ({label}): {err}")
            return None
        last_pose = pose
        current = pose

    progress_cb(f"Localized pose: {format_pose(last_pose)}")
    return last_pose


def stdin_reader(cmd_queue: queue.Queue, stop_event: threading.Event) -> None:
    """终端指令读取线程：非阻塞读 stdin，把指令放入命令队列。

    主循环不阻塞在 input() 上（需要持续消费 result_queue 驱动可视化与 UDP），
    因此用独立 daemon 线程读行；主循环轮询 cmd_queue。
    """
    while not stop_event.is_set():
        try:
            line = sys.stdin.readline()
        except Exception:
            break
        if line:
            cmd = line.strip().lower()
            if cmd:
                cmd_queue.put(cmd)
        else:  # EOF（如 stdin 被重定向）
            break


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
    stop_event: threading.Event,
    pause_event: threading.Event,
    localize_req_event: threading.Event,
    localize_paused_event: threading.Event,
    localize_resume_event: threading.Event,
    localize_frame_holder: dict,
    depth_hole_filter=None,
    depth_writer: Optional[DepthShmWriter] = None,
    handoff: Optional[AnchorHandoff] = None,
    tracker_factory=None,
) -> None:
    """相机线程持有行进 VIO 和临时定位 tracker，串行注册 IMU/Track。

    定位预热期间 IMU 正常采集；两采集线程确认挂起后主线程执行同步
    localize。结束后本线程销毁临时对象、创建新 VIO 和独立 IMU 时间线，
    主线程收到完成确认才发布 anchor 结果。全局位姿由 anchor 与新 VIO
    的局部增量合成；采集恢复仍受调度器 pause/resume 控制。
    """
    pose_trace = get_trace()
    imu_state = TrackerImuBuffer()
    tracker_generation = 0
    last_map_update_ns = 0
    last_trajectory_update_ns = 0
    camera_drop_count = 0

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
        if tracker is None:
            tracker = tracker_factory(slam=False)
        pose_trace.emit('tracker_switched', tracker_id=id(tracker), tracker_generation=tracker_generation)
        while not stop_event.is_set():
            # ---- 任务点锚定暂停协议 ----
            if localize_req_event.is_set():
                # The camera owns tracker creation/reset. The main thread accesses
                # only the warmed anchor tracker, while BOTH capture threads are parked.
                warm = 0
                anchor_trk = tracker_factory(slam=True, sync=True)
                localize_frame_holder['anchor_tracker'] = anchor_trk
                anchor_imu = TrackerImuBuffer()
                warm_deadline = time.monotonic()+30.
                while (warm < ANCHOR_WARMUP_FRAMES and not stop_event.is_set()
                       and not handoff.cancelled.is_set() and time.monotonic() < warm_deadline):
                    frames = ir_pipe.wait_for_frames(100)
                    if frames is None:
                        continue
                    left_frame = frames.get_frame(OBFrameType.LEFT_IR_FRAME)
                    right_frame = frames.get_frame(OBFrameType.RIGHT_IR_FRAME)
                    if left_frame is None or right_frame is None:
                        continue
                    ts = int(left_frame.get_timestamp_us() * 1000)
                    left_img = process_ir_frame(left_frame, target_size=SLAM_RESOLUTION)
                    right_img = process_ir_frame(right_frame, target_size=SLAM_RESOLUTION)
                    if left_img is None or right_img is None:
                        continue
                    if not anchor_imu.prepare(ts, imu_queue, stop_event, handoff.cancelled):
                        pose_trace.emit('imu_frame_rejected', source_timestamp_ns=ts,
                                        reason=anchor_imu.reason, tracker_role='anchor',
                                        tracker_generation=handoff.generation)
                        continue
                    images_w = (left_img, right_img)
                    anchor_imu.last_timestamp = register_imu_until(
                        anchor_trk, imu_queue, anchor_imu.pending, ts, anchor_imu.last_timestamp,
                    )
                    odom_est, _sl = anchor_trk.track(ts, images_w, internals=internals)
                    anchor_imu.tracked(ts)
                    if odom_est is not None and odom_est.world_from_rig is not None:
                        # 仅跟踪成功的帧计入预热并作为交付帧
                        localize_frame_holder["images"] = images_w
                        localize_frame_holder["timestamp_ns"] = ts
                        localize_frame_holder['n_obs'] = len(anchor_trk.get_last_observations(0))
                        warm += 1
                localize_frame_holder['warm_ok'] = warm >= ANCHOR_WARMUP_FRAMES and not handoff.cancelled.is_set()
                if not localize_frame_holder['warm_ok']:
                    localize_frame_holder['anchor_error'] = '定位预热取消/超时，未凑齐带新鲜 IMU 的有效帧'
                pose_trace.emit('anchor_warmup', tracker_id=id(anchor_trk), valid_frames=warm,
                                warm_ok=localize_frame_holder['warm_ok'], tracker_generation=handoff.generation)
                if stop_event.is_set() or not handoff.park():
                    break
                # Release every reference to the temporary SLAM tracker before building
                # the new active VIO. Main waits for complete() before emitting anchor=ok.
                localize_frame_holder['anchor_tracker'] = None
                anchor_trk = None
                anchor_imu = None
                tracker = None
                tracker = tracker_factory(slam=False)
                imu_state = TrackerImuBuffer()
                tracker_generation = handoff.generation
                thread_with_timestamp.prev_low_rate_timestamp = None
                health_obs_history.clear()
                health_fail_history.clear()
                pose_trace.emit('tracker_switched', tracker_id=id(tracker), tracker_generation=tracker_generation)
                diag_emit(f'[diag] VIO tracker 已由相机线程切换：generation={tracker_generation} id={id(tracker)}；IMU 时间线已重置')
                handoff.complete(tracker)
                continue
            if wait_while_paused(pause_event, stop_event):
                continue
            frame_wait_started_ns = time.monotonic_ns()
            frames = ir_pipe.wait_for_frames(100)
            if frames is None:
                continue

            frame_received_monotonic = time.monotonic()

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

            imu_wait_started = time.monotonic()
            if not imu_state.prepare(current_timestamp, imu_queue, stop_event, localize_req_event):
                pose_trace.emit('imu_frame_rejected', source_timestamp_ns=current_timestamp,
                                reason=imu_state.reason, tracker_role='vio', tracker_generation=tracker_generation)
                continue
            imu_state.last_timestamp = register_imu_until(
                tracker, imu_queue, imu_state.pending, current_timestamp, imu_state.last_timestamp
            )
            _t_track = time.monotonic()
            track_cpu_started_ns = time.thread_time_ns()
            odom_pose_estimate, slam_pose = tracker.track(current_timestamp, images, internals=internals)
            track_done_ns = time.monotonic_ns()
            track_cpu_s = (time.thread_time_ns()-track_cpu_started_ns)/1e9
            _dt_track = track_done_ns/1e9 - _t_track
            if _dt_track > .3:
                diag_emit(f"[diag] track() 耗时 {_dt_track:.3f}s；当前线程 CPU={track_cpu_s:.3f}s")
            imu_state.tracked(current_timestamp)

            odom_pose_with_cov = odom_pose_estimate.world_from_rig if odom_pose_estimate is not None else None
            if odom_pose_with_cov is None:
                pose_trace.emit('vio_failed', source_timestamp_ns=current_timestamp,
                                frame_received_ns=int(frame_received_monotonic*1e9), track_done_ns=track_done_ns)
                health_fail_history.append(1)
                health_obs_history.append(0)
                print(f"Tracking failed at frame {current_timestamp}")
                emit_health()
                continue
            health_fail_history.append(0)

            # 供任务点锚定使用：保留最近一次成功帧 + 当前位姿。
            # localize_in_map 需要图像/时间戳；slam_pose 用作下次锚定的 guess。
            localize_frame_holder["images"] = (left_img, right_img)
            localize_frame_holder["timestamp_ns"] = current_timestamp
            localize_frame_holder["slam_pose"] = slam_pose
            localize_frame_holder["odom_pose"] = odom_pose_with_cov.pose

            observations = tracker.get_last_observations(0)
            if pose_trace.enabled:
                pose_trace.emit('vio', source_timestamp_ns=current_timestamp,
                    tracker_id=id(tracker), tracker_generation=tracker_generation,
                    track_cpu_s=track_cpu_s, imu_wait_and_register_s=_t_track-imu_wait_started,
                    right_source_timestamp_ns=int(right_frame.get_timestamp_us()*1000),
                    frame_wait_started_ns=frame_wait_started_ns,
                    frame_received_ns=int(frame_received_monotonic*1e9),
                    track_started_ns=int(_t_track*1e9), track_done_ns=track_done_ns,
                    local_position=list(map(float, odom_pose_with_cov.pose.translation)),
                    local_quaternion=list(map(float, odom_pose_with_cov.pose.rotation)), observations=len(observations))
            localize_frame_holder["n_obs"] = len(observations)  # 锚定帧特征数（失败诊断用）
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
            if ENABLE_MAPPING_VISUALIZATION and slam_pose is not None and (
                current_timestamp - last_map_update_ns
            ) >= MAP_UPDATE_INTERVAL_MS * 1e6:
                # 纯 VIO 行进（SLAM 关闭）时 slam_pose 为 None，跳过取图调用
                # （get_slam_landmarks 在 SLAM 关闭时会报错/无数据）。
                map_landmarks = tracker.get_slam_landmarks(vslam.Tracker.SlamDataLayer.Map)
                loop_closure_poses = tracker.get_loop_closure_poses()
                last_map_update_ns = current_timestamp

            result_queue.put_latest(PoseResult(
                values=[
                    current_timestamp,
                    odom_pose_with_cov.pose,
                    images,
                    observations,
                    gravity,
                    slam_pose,
                    slam_poses,
                    map_landmarks,
                    loop_closure_poses,
                ],
                received_monotonic=frame_received_monotonic,
                processed_monotonic=time.monotonic(),
                tracker_generation=tracker_generation,
            ))
            thread_with_timestamp.last_low_rate_timestamp = current_timestamp
    except Exception as e:
        print(f"Camera thread error: {e}")
        if not stop_event.is_set():
            handoff.fail(f'Camera thread error: {e}')


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
    """任务导航模式：全程纯 map 行进 + 任务点 localize 锚定（详见文件头说明）。"""
    pose_trace = get_trace()
    parser = argparse.ArgumentParser(description="Orbbec Gemini 336L 任务导航：纯 map 行进 + 任务点 localize 锚定")
    parser.add_argument(
        "--ref-map",
        dest="ref_map",
        default=REF_MAP_NAME,
        help="参考地图名（任务点锚定目标，只读；脚本目录下的 <name> 子文件夹）",
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="不启动 Rerun Viewer GUI（无显示器 / SSH 无 X 转发时用），仅落盘 .rrd 供离线查看",
    )
    parser.add_argument(
        "--scheduler",
        action="store_true",
        help="调度器模式：关闭内部任务点状态机（任务表由外部调度器持有），"
             "锚定完成后保持采集暂停，由调度器显式发 resume 恢复 VIO 与位姿输出",
    )
    args = parser.parse_args()
    map_name: str = args.ref_map

    # 调度器模式下任务表为空：到达/推进/自动锚定全由外部调度器编排，本进程只做
    # 采集 + track + 手动 localize 锚定 + pause/resume 位姿输出。
    points = [] if args.scheduler else TASK_POINTS

    map_path = map_dir_path(map_name)
    if not os.path.isdir(map_path):
        raise RuntimeError(f"参考地图文件夹不存在: {map_path}，请先用 run_vio.py --mode map --map {map_name} 建图")
    print(f"参考地图: {map_name} ({map_path}) —— 仅用于任务点锚定，退出时不写回。")

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
    odometry_settings = dict(
        async_sba=True,
        enable_final_landmarks_export=True,
        enable_observations_export=True,
        debug_imu_mode=False,
        odometry_mode=vslam.Tracker.OdometryMode.Inertial,
        rectified_stereo_camera=True,
        use_gpu=True,
        use_denoising=True,  # run_vio_mapnav.py 实测配置
    )

    def make_tracker(slam: bool, sync: bool = False) -> vslam.Tracker:
        """创建 tracker（方案一：分段行进不建图）。

        行进：slam=False —— 纯 VIO，关闭 SLAM 地图/回环后端；
        惯性估计自身的优化仍可能产生长调用，需要由诊断记录继续核对。
        定位：slam=True, sync=True —— localize_in_map 要求 SLAM 开启；
        同步定位时两采集线程已确认挂起，避免并发访问 SDK/tracker。
        """
        # Tracker with SLAM mutates its OdometryConfig export flags. Never carry
        # that mutable config into a later VIO-only tracker.
        cfg = vslam.Tracker.OdometryConfig(**odometry_settings)
        scfg = (
            vslam.Tracker.SlamConfig(
                sync_mode=sync,
                planar_constraints=PLANAR_CONSTRAINTS,
                throttling_time_ms=LOOP_CLOSURE_THROTTLING_MS,
                use_gpu=True,
                max_map_size=MAX_MAP_SIZE,
            )
            if slam
            else None
        )
        print(f"[tracker] 创建 tracker：SLAM={'ON(同步定位)' if slam else 'OFF(纯VIO行进)'}")
        return vslam.Tracker(rig, cfg, scfg)

    tracker = None  # Created by the camera; Thread._args must not retain an old native tracker.

    internals = vslam.Tracker.Internals()
    internals.num_desired_tracks = NUM_DESIRED_TRACKS

    # --- IMU 流水线（ACCEL + GYRO），按 SDK 建议与视频分开 ---
    imu_pipeline = open_camera_pipeline(CAMERA_A_SERIAL)
    imu_config = Config()
    # 使用设备默认的量程 / 采样率（与 SDK 的 07_imu.py 示例一致）。
    imu_config.enable_accel_stream()
    imu_config.enable_gyro_stream()

    # 队列与可视化器（在流水线之前启动 Rerun，避免 fd 继承）。
    q = LatestPoseQueue()
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
    # 任务点锚定协议事件：
    #   pause_event      主线程置位 → IMU 线程挂起
    #   localize_req_event  主线程置位 → 相机线程下一循环入口挂起并交付锚定帧
    #   localize_paused_event  相机线程置位 → 主线程得知可安全调用 localize_in_map
    #   localize_resume_event  主线程置位 → 相机线程恢复取帧 track
    pause_event = threading.Event()
    handoff = AnchorHandoff(pause_event, stop_event)
    localize_req_event = handoff.requested
    localize_paused_event = handoff.ready
    localize_resume_event = handoff.release
    # 相机线程持续更新的「最近成功帧」容器：主线程在锚定时取用。
    localize_frame_holder = handoff.data

    imu_thread_obj = threading.Thread(
        target=imu_thread,
        args=(imu_queue, thread_with_timestamp, imu_pipeline, stop_event, pause_event, handoff),
        daemon=True,
    )
    camera_thread_obj = threading.Thread(
        target=camera_thread,
        args=(
            tracker, q, imu_queue, thread_with_timestamp, ir_pipeline, SHOW_GRAVITY, internals,
            stop_event, pause_event,
            localize_req_event, localize_paused_event, localize_resume_event, localize_frame_holder,
            depth_hole_filter, depth_writer,
        ),
        kwargs=dict(handoff=handoff, tracker_factory=make_tracker),
        daemon=True,
    )
    imu_thread_obj.start()
    camera_thread_obj.start()

    # 终端指令线程（localize / help / quit）。
    cmd_queue: queue.Queue = queue.Queue()
    stdin_thread = threading.Thread(target=stdin_reader, args=(cmd_queue, stop_event), daemon=True)
    stdin_thread.start()

    frame_id = 0
    anchor_epoch = 0
    last_sched_pose_emit_m = 0.0
    last_pose_source_ns = None
    last_pose_generation = 0
    last_pose_result_drops = 0
    trajectory_slam: List[np.ndarray] = []
    anchored = False  # 是否已完成至少一次任务点锚定（锚定后 slam_pose 即全局系）
    last_global_pose: Optional[vslam.Pose] = None  # 最近一次位姿（锚定后为全局系），退出时保存
    slam_pose: Optional[vslam.Pose] = None  # 最近一次 SLAM 位姿（调度器模式面板与 [SCHED] pose 流用）
    odom_pose: Optional[vslam.Pose] = None  # 最近一次前端 VIO 位姿（循环顶面板回退显示用）
    last_pose_print = 0.0  # 实时坐标打印节流时间戳（秒，time.monotonic）
    localize_busy = False  # 锚定流程进行中（请求 → 挂起 → 搜索 → 恢复）
    localize_req_time = 0.0
    # ---- 自动任务点状态 ----
    task_index = 0            # 当前目标任务点下标（0 = 第一个，锚定后全局系下判定到达）
    task_arrived = False      # 已到达当前任务点（等待静止）
    task_prompt_printed = False  # 「请静止」提示只打印一次
    static_history: Deque = deque()  # 静止检测窗口：(monotonic_sec, x, y)
    anchor_source = "manual"  # 本次锚定触发来源："startup" / "manual" / "auto"
    startup_anchor_requested = False  # 启动自动锚定是否已发起（只一次）
    last_anchor_ok_time = 0.0  # 最近一次锚定成功时刻（request_anchor 冷却用）
    retry_pending = False  # 对照实验：定位失败后待自动重试（改用 last-pose guess）
    # 段内纯 VIO 出口位姿状态（USE_ODOM_FOR_POSE_OUTPUT）
    pub_pose: Optional[vslam.Pose] = None  # 当前出口位姿（全局系，odom 帧间增量累积）
    last_odom: Optional[vslam.Pose] = None  # 上一帧 odom（算帧间增量）
    odom_baseline_pending = False  # 锚定后首帧只重置 odom 基线，不累积增量
    paused_by_cmd = False  # 调度器 pause 命令：挂起采集 + 停发位姿，直到 resume 命令
    panel_msg = ""  # 调度器模式面板消息行（事件信息显示在面板里，避免被清屏刷掉）

    def panel_print(full_line: str) -> None:
        """事件/进度消息出口：调度器模式进面板消息行（去 [xx] 前缀），否则原样打印。

        三级搜索提示（[localize] coarse/mid/fine ...）也走这里：面板每 0.1s 重绘，
        普通 print 会被清屏刷掉，面板消息行则能持续显示。
        """
        nonlocal panel_msg
        if args.scheduler:
            for pfx in ("[localize] ", "[task] ", "[local ] "):
                if full_line.startswith(pfx):
                    full_line = full_line[len(pfx):]
                    break
            panel_msg = full_line
        else:
            print(full_line)

    def task_print(msg: str) -> None:
        panel_print(f"[task] {msg}")

    def request_anchor(source: str, reason: str, use_lastpose_guess: bool = False) -> None:
        """发起任务点锚定（手动指令与自动任务点共用入口）。"""
        nonlocal localize_busy, localize_req_time, anchor_source
        if localize_busy:
            return
        # 锚定结束（成功/失败/取消）后冷却：立即再次 localize 会撞上 SLAM
        # 后台线程收尾竞态（GIL 崩溃，见 ANCHOR_RETRY_COOLDOWN_S 注释）。
        if time.monotonic() - last_anchor_ok_time < ANCHOR_RETRY_COOLDOWN_S:
            task_print(f"距上次锚定结束不足 {ANCHOR_RETRY_COOLDOWN_S:.0f}s，"
                       "忽略再次 localize（防竞态崩溃）。")
            return
        localize_busy = True
        localize_req_time = time.monotonic()
        anchor_source = source
        handoff.start(use_lastpose_guess)
        task_print(reason)
        sched_emit(f"[SCHED] anchor=busy source={source}")
        # start resumes IMU capture even after manual pause, without authorizing output/motion.

    def complete_anchor_handoff():
        nonlocal odom_baseline_pending, last_odom, last_pose_source_ns, last_pose_generation
        handoff.finish()
        q.clear_pending()
        last_pose_source_ns = None
        last_pose_generation = handoff.generation
        # Failure/cancellation also resets the local frame. Never subtract poses
        # belonging to different tracker generations, even if no new anchor was found.
        odom_baseline_pending = True
        last_odom = None

    print("Starting task navigation (pure map + task-point localize anchoring)...")
    print(f"Reference map: {map_name}  ({map_path})")
    print(f"Mapping visualization: {'ON' if ENABLE_MAPPING_VISUALIZATION else 'OFF'}")
    if points:
        print(f"任务点列表（全局系，共 {len(points)} 个）：")
        for i, (tx, ty, tz) in enumerate(points):
            if i == 0 and not TASK_DETECT_IN_LOCAL_FRAME:
                tag = " ← 第一个任务点：请静止后手动输入 localize"
            elif TASK_AUTO_ANCHOR:
                tag = " ← 到达+静止后自动锚定"
            else:
                tag = " ← 到达+静止后提示手动锚定"
            print(f"  {i + 1}. ({tx:.2f}, {ty:.2f}, {tz:.2f}){tag}")
        print(f"  到达判定半径 {TASK_ARRIVE_RADIUS_M}m | 静止判定 {TASK_STATIC_WINDOW_S}s 内位移 < {TASK_STATIC_DISP_M}m"
              f" | 自动锚定: {'ON' if TASK_AUTO_ANCHOR else 'OFF'}"
              f" | 本地系检测: {'ON' if TASK_DETECT_IN_LOCAL_FRAME else 'OFF'}")
        if TASK_DETECT_IN_LOCAL_FRAME:
            print("  注意：本地系检测要求机器人从建图原点、以建图时相同朝向启动。")
    else:
        print("任务点列表为空：未启用自动任务点，全程手动输入 localize 锚定。")
    if AUTO_ANCHOR_AT_START:
        print(f"启动自动锚定：ON（首帧后自动 localize 对齐全局系，"
              f"coarse 半径 {STARTUP_LOCALIZE_H_RADIUS}m；请保持机器人静止）")
    else:
        print("启动自动锚定：OFF（需手动 localize 对齐全局系）")
    print("=" * 62)
    print("终端指令（在任务点静止后输入）：")
    print("  localize / l   用参考地图锚定全局位姿（作为下一段子任务的起点）")
    print("  help / h       打印指令帮助")
    print("  quit / q       优雅退出（不保存地图，等价 Ctrl+C）")
    print("=" * 62)
    print("Press Ctrl+C to stop")

    try:
        _t_loop_prev = None  # 主循环整轮耗时埋点（上一轮 >2s 时报告，抓卡顿位置）
        while True:
            _t_loop_now = time.monotonic()
            if _t_loop_prev is not None and _t_loop_now - _t_loop_prev > 2.0:
                diag_emit(f"[diag] 主循环上一轮耗时 {_t_loop_now - _t_loop_prev:.1f}s")
            _t_loop_prev = _t_loop_now
            # ---- 调度器模式位姿面板（可显示缓存；到达事件仅从下方新采样发送）----
            # 事件消息经 task_print 进面板「消息」行；机器行 [SCHED] 单独输出
            # （调度器读取并过滤，不进日志面板）。挂起/锚定期间没有新帧也照常
            # 刷新面板（显示当前状态与最后位姿）。
            if args.scheduler:
                now_m = time.monotonic()
                if now_m - last_pose_print >= POSE_PRINT_INTERVAL_S:
                    last_pose_print = now_m
                    # 出口位姿与 UDP 发送同源（odom 累积 pub_pose / slam_pose），
                    # 面板上显示其 Z-up 位置，与 NavSide pos_w 直接比对。
                    # 纯 VIO 行进（SLAM 关闭）时 slam_pose 恒为 None，
                    # 必须优先选 pub_pose（锚定后 odom 累积的全局位姿）。
                    out_pose = None
                    if USE_ODOM_FOR_POSE_OUTPUT and anchored and pub_pose is not None:
                        out_pose = pub_pose
                    elif slam_pose is not None:
                        out_pose = slam_pose
                    # 纯 VIO 行进（SLAM 关闭）时锚定前 slam_pose/pub_pose 均为
                    # None，面板回退显示本地 odom 位姿（直观确认 VIO 存活）。
                    display_pose = out_pose if out_pose is not None else (
                        slam_pose if slam_pose is not None else odom_pose)
                    print(pose_panel_text(anchored, localize_busy, paused_by_cmd,
                                          display_pose, panel_msg, out_pose), flush=True)

            # ---- 终端指令处理 ----
            while True:
                try:
                    cmd = cmd_queue.get_nowait()
                except queue.Empty:
                    break
                if cmd in ("localize", "l"):
                    if localize_busy:
                        task_print("锚定流程进行中，请稍候...")
                    else:
                        request_anchor("manual", "任务点锚定开始：请保持机器人静止。")
                elif cmd in ("pause", "p"):
                    if localize_busy:
                        task_print("锚定流程进行中，忽略 pause。")
                    else:
                        pose_trace.emit('control', command='pause')
                        paused_by_cmd = True
                        pause_event.set()
                        sched_emit("[SCHED] paused=1")
                elif cmd in ("resume", "r"):
                    if localize_busy:
                        task_print("锚定流程进行中，忽略 resume。")
                    else:
                        pose_trace.emit('control', command='resume')
                        paused_by_cmd = False
                        pause_event.clear()
                        sched_emit("[SCHED] paused=0")
                elif cmd.startswith("msg "):
                    # 调度器下发的面板提示（如「已到达任务点 X」）。
                    panel_print(cmd[4:])
                elif cmd in ("help", "h"):
                    print("指令: localize / l = 任务点锚定 | pause / p = 暂停采集与位姿输出 | "
                          "resume / r = 恢复 | msg <文本> = 面板提示 | help / h = 帮助 | quit / q = 退出")
                    if points and task_index < len(points):
                        tx, ty, tz = points[task_index]
                        print(f"[task] 当前目标：任务点 {task_index + 1}/{len(points)} ({tx:.2f}, {ty:.2f}, {tz:.2f})")
                elif cmd in ("quit", "q", "exit"):
                    raise KeyboardInterrupt

            # ---- 锚定流程状态机 ----
            if stop_event.is_set():
                raise RuntimeError(localize_frame_holder.get('worker_error') or '采集线程已停止')
            if localize_busy and not localize_paused_event.is_set():
                # 等待相机线程挂起并交付锚定帧（首帧未完成前会一直取帧）。
                if time.monotonic() - localize_req_time > 30.0 and not handoff.cancelled.is_set():
                    task_print('定位预热超时，正在取消；等待采集线程安全交接后再报告失败。')
                    handoff.cancelled.set()

            # 对照实验的自动重试（定位失败后改用 last-pose guess 再来一次）
            if retry_pending and not localize_busy:
                retry_pending = False
                last_anchor_ok_time = 0.0  # 内部对照重试不受冷却限制
                request_anchor("manual", "对照重试：改用 last-pose 文件 guess 定位",
                               use_lastpose_guess=True)

            if localize_busy and localize_paused_event.is_set():
                # 生产者已经挂起；锚定后的首帧不能来自锚定前的结果队列。
                q.clear_pending()
                last_pose_source_ns = None
                # 相机线程已挂起：此时调 localize_in_map，后台 SLAM 线程回调不会与
                # pyorbbecsdk wait_for_frames 并发抢 GIL（安全）。
                holder_images = localize_frame_holder["images"]
                holder_ts = localize_frame_holder["timestamp_ns"]
                if (holder_images is None or holder_ts is None or not localize_frame_holder.get('warm_ok')
                        or handoff.cancelled.is_set()):
                    task_print(localize_frame_holder.get('anchor_error') or '锚定帧不可用，已取消本次锚定。')
                    pose = None
                    complete_anchor_handoff()
                else:
                    # 使用已预热的同步 SLAM tracker 定位；交接后由相机线程
                    # 重建纯 VIO。关闭建图不代表惯性计算/GPU 同步不会出现长调用。
                    try:
                        # 猜测位姿（假设：累积姿态的滚转/俯仰漂移会把搜索网格
                        # 带斜——底层 T_candidate = T_guess × T_offset，Y 漂移
                        # 单独不足以解释失败，第一点成功时 Y 也已漂 0.51m；
                        # 待对照实验确认）：
                        #   位置 = pub_pose 的 XZ（实时位姿）、Y = 0（地图地面高度）；
                        #   姿态 = 单位四元数——航向由角向搜索 360° 覆盖，
                        #   单位姿态零损失且不注入漂移。
                        guess_pose = None
                        if (anchored and pub_pose is not None
                                and not localize_frame_holder.get("use_lastpose_guess")):
                            _gt = pub_pose.translation
                            guess_pose = vslam.Pose(
                                rotation=[0.0, 0.0, 0.0, 1.0],
                                translation=[float(_gt[0]), 0.0, float(_gt[2])],
                            )
                        # 用相机线程预热过的锚定 tracker（经 holder 传递）定位。
                        pose = localize_anchor(
                            localize_frame_holder.get("anchor_tracker"),
                            holder_ts, holder_images, guess_pose, anchored, map_name,
                            coarse_h_radius_override=(
                                STARTUP_LOCALIZE_H_RADIUS if anchor_source == "startup" else None
                            ),
                            progress_cb=panel_print,
                        )
                    finally:
                        complete_anchor_handoff()
                if pose is not None:
                    anchor_epoch += 1
                    pose_trace.emit('anchor', anchor_epoch=anchor_epoch,
                                    map_position=list(map(float, pose.translation)))
                    anchored = True
                    last_global_pose = pose
                    last_anchor_ok_time = time.monotonic()
                    localize_frame_holder["use_lastpose_guess"] = False
                    task_print("锚定成功：已取得参考地图全局位姿，相机已切换新 VIO 并重置局部基线；"
                               f"本段子任务起点位姿: {format_pose(pose)}")
                    _t, _r = pose.translation, pose.rotation
                    sched_emit(f"[SCHED] anchor=ok pose=({_t[0]:.4f},{_t[1]:.4f},{_t[2]:.4f},"
                               f"{_r[0]:.4f},{_r[1]:.4f},{_r[2]:.4f},{_r[3]:.4f})")
                    # 纯 VIO 出口：以锚定全局位姿为新起点；下一帧重置 odom 基线，
                    # 此后按 odom 帧间增量累积（对 localize 后 odom 坐标系跳变免疫）。
                    pub_pose = pose
                    odom_baseline_pending = True
                    last_odom = None
                    # 任务点校验与推进：锚定结果与当前任务点坐标比对。
                    # cuVSLAM 世界系竖直轴是 Y（向下），水平面是 XZ，故 2D 距离用 x-z。
                    # 启动自动锚定只对齐坐标系，不推进任务点。
                    if anchor_source != "startup" and points and task_index < len(points):
                        tx, tz = points[task_index][0], points[task_index][2]
                        gx, gz = pose.translation[0], pose.translation[2]
                        err = float(np.hypot(gx - tx, gz - tz))
                        if err <= TASK_ANCHOR_TOLERANCE_M:
                            print(f"[task] 任务点 {task_index + 1}/{len(TASK_POINTS)} 完成（锚定偏差 {err:.2f}m）。")
                            task_index += 1
                            task_arrived = False
                            task_prompt_printed = False
                            static_history.clear()
                            if TASK_PAUSE_POSE_ON_ARRIVE:
                                print("[task] 位姿发送恢复。")
                            if task_index < len(points):
                                nt = points[task_index]
                                print(f"[task] 下一任务点 {task_index + 1}: ({nt[0]:.2f}, {nt[1]:.2f}, {nt[2]:.2f})")
                            else:
                                print("[task] 所有任务点已完成，继续行进。")
                        elif anchor_source == "auto":
                            print(f"[task] 警告：任务点 {task_index + 1} 锚定偏差 {err:.2f}m > "
                                  f"{TASK_ANCHOR_TOLERANCE_M}m，不推进（检查地图与任务点坐标）。")
                        else:
                            print(f"[task] 提示：当前位姿距任务点 {task_index + 1} 还有 {err:.2f}m，未推进。")
                else:
                    # 失败诊断：错误文本已由 localize_anchor 直落日志（[localize] FAILED ...）；
                    # 补一行当前位姿 + 锚定帧特征数对照：
                    #   特征数过低 → 相机视图纹理不足/对着近墙，需调整机器人朝向；
                    #   特征正常但仍未命中 → 位置与地图不一致（地图覆盖/坐标系问题）。
                    _n = localize_frame_holder.get("n_obs")
                    _warm_ok = localize_frame_holder.get("warm_ok", False)
                    if pub_pose is not None:
                        _p = pub_pose.translation
                        diag_emit(f"[localize] 诊断: 当前位姿≈({_p[0]:.2f},{_p[1]:.2f},{_p[2]:.2f})，"
                                  f"guess=(x={_p[0]:.2f}, y=0, z={_p[2]:.2f}, q=identity)，"
                                  f"预热有效帧={'是' if _warm_ok else '否'}，"
                                  f"锚定帧特征数={_n}，25m 搜索盒未命中"
                                  f"{'——特征过少，疑似视图纹理不足/对着近墙，请调整机器人朝向' if _n is not None and _n < 30 else ''}"
                                  f"（若位姿可信且特征正常，检查地图覆盖/坐标系；"
                                  f"对照实验：同帧改用 last-pose 文件 guess 重试）")
                    sched_emit("[SCHED] anchor=fail")
                    last_anchor_ok_time = time.monotonic()  # 失败同样进入冷却（防背靠背重试竞态）
                    # 对照实验：手动定位失败且本次用的是实时 guess 时，自动改用
                    # last-pose 文件 guess 重试一次——同一位置两次结果直接比对，
                    # 即可定论「guess 是否是根因」。
                    if (anchor_source != "startup"
                            and not localize_frame_holder.get("use_lastpose_guess")):
                        retry_pending = True
                        task_print("对照实验：改用 last-pose 文件 guess 自动重试一次...")
                    else:
                        localize_frame_holder["use_lastpose_guess"] = False
                    if anchor_source == "startup":
                        task_print("启动自动锚定失败：请重新调整机器人位姿（移动/转向），"
                                   "然后在调度器终端输入 localize 重试。")
                    else:
                        task_print("锚定失败：请重新调整机器人位姿（移动/转向）后，"
                                   "再次输入 localize 重试。")
                # 恢复相机线程（解除 localize 挂起）。调度器模式下保持 pause_event
                # 置位（采集与位姿输出维持暂停），由外部调度器确认锚定结果后
                # 显式发 resume 恢复 VIO。例外：启动锚定失败且调度器没主动 pause 过
                # 时恢复取帧，否则相机线程停摆、可视化/本地系打印全部冻结。
                if not args.scheduler or (anchor_source == "startup" and not paused_by_cmd):
                    pause_event.clear()
                localize_busy = False
                anchor_source = "manual"

            # ---- 结果消费 ----
            try:
                result = q.get(timeout=0.5)
            except queue.Empty:
                continue
            q.task_done()
            if localize_busy:
                continue  # Do not consume a pre-anchor pose while preparing a new tracker.
            if result.tracker_generation != last_pose_generation:
                if result.tracker_generation < last_pose_generation:
                    continue
                last_pose_generation = result.tracker_generation
                last_pose_source_ns = None
                odom_baseline_pending = True
                last_odom = None
            consumed_m = time.monotonic()
            frame_age = result.age(consumed_m)
            queue_age = consumed_m - result.processed_monotonic
            drops = q.dropped_results - last_pose_result_drops
            last_pose_result_drops = q.dropped_results
            source_dt = ((result.values[0] - last_pose_source_ns) * 1e-9
                         if last_pose_source_ns is not None else 0.)
            fresh = result.is_fresh(consumed_m, POSE_MAX_RESULT_AGE_S)
            pose_trace.emit('consume', source_timestamp_ns=result.values[0], anchor_epoch=anchor_epoch,
                tracker_generation=result.tracker_generation,
                frame_received_ns=int(result.received_monotonic*1e9), processed_ns=int(result.processed_monotonic*1e9),
                accepted=fresh, dropped_results=drops, frame_age_s=frame_age, queue_age_s=queue_age)
            if drops or frame_age > 0.3 or source_dt > 0.3 or not fresh:
                diag_emit(f"[diag] pose_pipeline frame_age_s={frame_age:.4f} "
                          f"queue_age_s={queue_age:.4f} dropped_results={drops} "
                          f"source_dt_s={source_dt:.4f} accepted={int(fresh)}")
            if not fresh:
                continue
            (
                timestamp, odom_pose, images, observations, gravity, slam_pose,
                slam_poses, map_landmarks, loop_closure_poses,
            ) = result.values
            last_pose_source_ns = timestamp

            if odom_pose is None:
                continue

            # 记录最近一次位姿（锚定后为全局系），退出时保存供下次锚定作 guess。
            # 锚定后优先用出口位姿 pub_pose：tracker 在每次锚定时重建，
            # 重建后 slam_pose 回到本地系，不再代表全局位姿。
            if pub_pose is not None:
                last_global_pose = pub_pose
            elif slam_pose is not None:
                last_global_pose = slam_pose

            # ---- 段内纯 VIO 出口位姿（odom 帧间增量累积）----
            # 任务点之间完全不用 SLAM 后端位姿：回环/PGO/平面约束都不干预出口。
            # 只用 odom 帧间增量（对 localize 后 odom 坐标系可能跳变免疫）；
            # 漂移随段长累积，每个任务点锚定后清零（见锚定成功块）。
            if USE_ODOM_FOR_POSE_OUTPUT and anchored and odom_pose is not None:
                if odom_baseline_pending:
                    # 锚定后首帧：只重置基线，不累积增量。
                    last_odom = odom_pose
                    odom_baseline_pending = False
                elif last_odom is not None and pub_pose is not None:
                    delta = pose_compose(pose_inverse(last_odom), odom_pose)
                    pub_pose = pose_compose(pub_pose, delta)
                    last_odom = odom_pose

            # 启动自动锚定：首帧跟踪成功后自动执行一次 localize 对齐全局系。
            # 之后所有任务点（含第一个）都在全局系下检测到达。
            if (
                AUTO_ANCHOR_AT_START
                and not anchored
                and not startup_anchor_requested
                and not localize_busy
                and odom_pose is not None  # 纯 VIO 行进时无 slam_pose，首帧 odom 即触发
            ):
                startup_anchor_requested = True
                request_anchor(
                    "startup",
                    "启动自动锚定：请保持机器人静止（对齐参考地图全局系）...",
                )

            # ---- 任务点到达检测状态机 ----
            # 锚定后位姿为全局系，直接与 TASK_POINTS 比较；未锚定时按
            # TASK_DETECT_IN_LOCAL_FRAME 决定是否用本地系位姿检测（需从建图原点启动）。
            if (
                (anchored or TASK_DETECT_IN_LOCAL_FRAME)
                and points
                and task_index < len(points)
                and slam_pose is not None
                and not localize_busy
            ):
                # 水平距离用 x-z（竖直轴是 Y），与 TASK_POINTS 注释一致。
                gx, gz = slam_pose.translation[0], slam_pose.translation[2]
                tx, tz = points[task_index][0], points[task_index][2]
                dist = float(np.hypot(gx - tx, gz - tz))
                now_m = time.monotonic()
                static_history.append((now_m, gx, gz))
                while static_history and static_history[0][0] < now_m - TASK_STATIC_WINDOW_S:
                    static_history.popleft()

                if not task_arrived:
                    if dist <= TASK_ARRIVE_RADIUS_M:
                        task_arrived = True
                        if TASK_PAUSE_POSE_ON_ARRIVE:
                            print(f"[task] 已到达任务点 {task_index + 1}/{len(TASK_POINTS)} "
                                  f"({tx:.2f}, {tz:.2f})，请静止。位姿发送已暂停。")
                        else:
                            print(f"[task] 已到达任务点 {task_index + 1}/{len(TASK_POINTS)} "
                                  f"({tx:.2f}, {tz:.2f})，请静止。")
                elif dist > TASK_ARRIVE_RADIUS_M * 1.5:
                    # 到达后机器人又走开：复位到达状态，恢复位姿发送。
                    task_arrived = False
                    task_prompt_printed = False
                    if TASK_PAUSE_POSE_ON_ARRIVE:
                        print("[task] 已离开任务点范围，位姿发送恢复。")

                if task_arrived:
                    # 静止判定：时间窗基本填满且窗口内总位移 < 阈值。
                    if len(static_history) >= 2:
                        window_s = static_history[-1][0] - static_history[0][0]
                        disp = float(np.hypot(
                            static_history[-1][1] - static_history[0][1],
                            static_history[-1][2] - static_history[0][2],
                        ))
                        if window_s >= TASK_STATIC_WINDOW_S * 0.8 and disp <= TASK_STATIC_DISP_M:
                            if TASK_AUTO_ANCHOR:
                                request_anchor(
                                    "auto",
                                    f"任务点 {task_index + 1}：检测到静止，自动发起锚定...",
                                )
                            elif not task_prompt_printed:
                                task_prompt_printed = True
                                print("[task] 已静止。输入 localize 手动锚定"
                                      "（或设 TASK_AUTO_ANCHOR=True 自动）。")

            # 实时坐标打印（节流）。锚定前是本地系；锚定后 odom 模式打纯 VIO 累积位姿
            # （[odom ] 前缀），否则打 slam_pose（[global]）。调度器模式改为循环顶部的
            # 原地刷新面板（不刷屏），此处不再逐行打印。
            # 纯 VIO 行进（SLAM 关闭）时 slam_pose 恒为 None，用 pub_pose 判活。
            if (slam_pose is not None or (anchored and pub_pose is not None)) and not args.scheduler:
                now = time.monotonic()
                if now - last_pose_print >= POSE_PRINT_INTERVAL_S:
                    last_pose_print = now
                    if USE_ODOM_FOR_POSE_OUTPUT and anchored and pub_pose is not None:
                        print(f"[odom ] {format_pose(pub_pose)}")
                    else:
                        prefix = "[global]" if anchored else "[local ]"
                        print(f"{prefix} {format_pose(slam_pose)}")

            # 位姿出口：锚定后 odom 模式发纯 VIO 累积位姿（pub_pose），否则发 slam_pose；
            # 锚定前按 SEND_POSE_BEFORE_ANCHOR 决定是否发本地系位姿。
            # 到达任务点后（task_arrived）暂停发送，锚定完成推进后恢复（TASK_PAUSE_POSE_ON_ARRIVE）。
            # 纯 VIO 行进时 slam_pose 为 None，判活改用 pub_pose/slam_pose 任一。
            if slam_pose is not None or pub_pose is not None:
                if (anchored or SEND_POSE_BEFORE_ANCHOR) and not (
                    TASK_PAUSE_POSE_ON_ARRIVE and task_arrived
                ) and not paused_by_cmd and not localize_busy:
                    out_pose = None
                    if USE_ODOM_FOR_POSE_OUTPUT and anchored and pub_pose is not None:
                        out_pose = pub_pose
                    elif slam_pose is not None:
                        out_pose = slam_pose
                    output_m = time.monotonic()
                    if out_pose is not None and result.is_fresh(output_m, POSE_MAX_RESULT_AGE_S):
                        if pose_sender is not None:
                            pose_sender.send_pose(out_pose.translation, out_pose.rotation,
                                source_timestamp_ns=int(timestamp), anchor_epoch=anchor_epoch,
                                frame_received_ns=int(result.received_monotonic*1e9),
                                processed_ns=int(result.processed_monotonic*1e9))
                        # 与 UDP 同源的新采样；不能用缓存位姿加当前时间冒充更新。
                        # SRU 关闭时仍需此事件，所以不依赖 pose_sender 是否启用。
                        if args.scheduler and anchored and output_m-last_sched_pose_emit_m >= POSE_PRINT_INTERVAL_S:
                            last_sched_pose_emit_m = output_m
                            _t, _r = out_pose.translation, out_pose.rotation
                            sched_emit(f"[SCHED] pose=({_t[0]:.4f},{_t[1]:.4f},{_t[2]:.4f},"
                                       f"{_r[0]:.4f},{_r[1]:.4f},{_r[2]:.4f},{_r[3]:.4f}) "
                                       f"t={time.time():.3f} source_timestamp_ns={timestamp} "
                                       f"frame_age_s={result.age(output_m):.4f}")

            frame_id += 1
            # 用 SLAM 平滑轨迹（回环后回溯矫正）替换逐帧追加，避免回环跳变。
            if slam_poses is not None:
                trajectory_slam = [p.pose.translation for p in slam_poses]

            # 一旦 SLAM 优化结果可用，优先使用它。
            viz_pose = slam_pose if slam_pose is not None else odom_pose

            _t_viz = time.monotonic()
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
            _dt_viz = time.monotonic() - _t_viz
            pose_trace.emit('visualize', source_timestamp_ns=timestamp, duration_s=_dt_viz)
            if _dt_viz > 2.0:
                diag_emit(f"[diag] visualize_frame 阻塞 {_dt_viz:.1f}s")

    except KeyboardInterrupt:
        print("Stopping task navigation...")
    finally:
        # 清理期间忽略 SIGINT，避免第二次 Ctrl+C 中断流水线关闭导致 std::terminate。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        # 先让采集线程退出：置位停止标志 + 停止流水线（解除 wait_for_frames 阻塞），
        # 再 join 等线程真正结束。否则线程在进程退出 / C++ 对象析构时仍在访问
        # Pipeline/Tracker，会触发 "terminate called without an active exception"（SIGABRT）。
        stop_event.set()
        # 若采集线程正挂在锚定暂停中，先解除暂停，让它们能退出循环。
        localize_resume_event.set()
        pause_event.clear()
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
        # 不保存地图：本脚本对参考地图只读，退出不写回，避免污染下次锚定用的地图。
        # 仅锚定成功后保存最后全局位姿，供下次锚定作初始 guess。
        if anchored and last_global_pose is not None:
            save_last_pose(last_global_pose, map_name)
        visualizer.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), "vio_mapping_tracking.rrd"))
        pose_trace.close()
        print("Clean shutdown complete.")


if __name__ == "__main__":
    main()
