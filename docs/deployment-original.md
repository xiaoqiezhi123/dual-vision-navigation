AAA   推车版本
cuvslam_migrate-9.1   直道slam版本
cuvslsm -main-backup 源码版本


# 双视觉版本 — Phybot 双相机导航部署说明

本目录只保留「双视觉」主线交付的最小集合：**相机 A 的 cuVSLAM 被动双目惯性 SLAM（出位姿）**、**相机 B 的 NavSide 深度感知 + SRU 导航推理（出速度）**、**RobotSide CPG 行走控制**，以及一套绕过 SRU 的 **WASD 键盘直控**旁路。四者构成从「感知 → 定位 → 推理 → 速度 → 执行 → 状态回馈」的完整闭环。

## 当前交付范围

| 目录 | 角色 | 验证状态 |
|---|---|---|
| `cuvslam_migrate-9.1` | 相机 A：cuVSLAM SLAM，出全局位姿 | 真机已跑（被动双目惯性 SLAM，位姿经 UDP 8082 发 NavSide） |
| `Navside-9.1` | 主 NavSide：Orbbec 深度 + VAE/SRU 推理 | 真机导航主路径 |
| `Navside-9.4` | WASD 键盘直控（绕过 SRU 的轻量旁路） | 真机已跑，用于操控机器人建图（armS版本没有写joysick） |
| `PhybotSoftware_c2 arm` | 机器人侧 C++：CPG 策略 + 状态机 + 电机 | 真机运行（`realrobot`），`mujoco_sim` 仿真 |

> 与上游 `sru_mujoco_sim` 的差异：本目录是**双相机实机方案**。定位不再依赖郎毅雷达包，而是由相机 A 的 cuVSLAM 提供；深度由相机 B 提供。上游里 `RobotComm` 订阅 Foxglove/Langyi 的里程计来源，在本方案中被 cuVSLAM 的 UDP 位姿包替代。

## 主流水线图

```
相机 A (Orbbec 336L #1, IR 双目, 激光关)
   │  1280×720@15 IR → cuVSLAM (run_vio.py)
   │  slam_pose → OpenCV→Z-up 变换
   ▼  UDP 127.0.0.1:8082   struct "<7d" = [px,py,pz, qw,qx,qy,qz] (56B)
NavSide-9.1  RobotComm._pose_sock
   │  复用 _compute_velocities / _compute_projected_gravity，z=0.695
   │  → 组装 NavStatePacketV2 (84B <IHHId16f)
   │
相机 B (Orbbec 336L #2, 深度, 激光开)
   │  Y16 1280×720@30 → OrbbecDepthPerception → float32 米
   ▼
VAE 编码器 (vae_orbbec_mix_ep30_deploy.onnx) → 深度特征 2560维
   │  + NavStatePacketV2 状态 → obs 2576维
   ▼
SRU 策略 (nav_policy.onnx, 5Hz) → vx, vy, wz (3 floats, clamp)
   ▼  UDP 127.0.0.1:8080   struct.pack("3f", vx, vy, wz)
机器人侧 PhybotSoftware_c2 / RL_deploy_cpg
   │  InitUDP() 监听 8080 → CommandPacket {vx, vy, omega_z}
   ▼  CPG 策略 (phybot_cpg_policy.onnx) → 状态机 → MotorList → 电机
```

## 目录结构

```
双视觉版本/
├── cuvslam_migrate-9.1/          # 相机 A：cuVSLAM 被动 SLAM
│   ├── run_orbbec.sh             #   隔离 venv 启动脚本（本地化 / 建图）
│   ├── venv/                     #   隔离环境（pyorbbecsdk + cuvslam）
│   └── orbbec/
│       ├── run_vio.py            #   VIO 主循环（RESOLUTION 1280×720@15）
│       ├── camera_utils.py       #   双目标定 / 抓帧
│       ├── udp_pose_sender.py    #   位姿 → UDP 8082
│       ├── foxglove_odom_server.py  # opencv_pose_to_zup / Foxglove odom
│       ├── depth_shm.py          #   深度共享内存写
│       ├── feature_map.py        #   特征地图
│       ├── visualizer.py / visualizer_mapping.py
│       └── enumerate_devices.py
│
├── Navside-9.1/                  # 主 NavSide：SRU 导航侧
│   ├── scripts/run_nav.py        #   启动入口（--real / --sim-control）
│   ├── navside/
│   │   ├── runtime.py            #   顶层编排
│   │   ├── adapter.py            #   深度→特征→obs→ONNX→动作→clamp
│   │   ├── bridge.py             #   RobotComm：UDP 位姿(8082) + 命令(8080)
│   │   ├── depth.py / sim.py / real.py / state.py / mode.py / timing.py
│   ├── perception/
│   │   ├── orbbec_depth_perception.py   # 当前默认后端
│   │   ├── realsense_depth_perception.py
│   │   ├── zed_depth_perception.py
│   │   └── depth_shm.py
│   ├── config/nav.yaml           #   后端/模型/限幅/UDP 端口
│   ├── asset/models/             #   VAE / policy ONNX
│   ├── asset/robot/phybot_mini_mark2/  #  MuJoCo 场景与机器人 XML
│   ├── docs/                     #   navside_readme / baseline / orbbec 接入
│   └── .venv_navside/            #   正式运行环境
│
├── Navside-9.4/                  # WASD 键盘直控（Navside-9.1 轻量衍生）
│   ├── scripts/wasd_control.py   #   evdev 读键盘 → struct.pack("3f") → 8080
│   ├── config/nav.yaml
│   └── config/nav_deploy.yaml    #   Foxglove / API / cmd 目标
│
└── PhybotSoftware_c2 arm/        # 机器人侧 C++
    └── PhybotSoftware_c2/
        ├── RL_deploy_cpg/        #   CPG 策略部署（src/rl_deploy.cpp InitUDP 8080）
        ├── StateMachine/         #   tinyfsm 状态机
        ├── MotorList/            #   电机列表 / PD
        ├── MujocoInterface/      #   MuJoCo 接口
        ├── RobotStart/           #   realrobot / mujoco_sim 两个入口
        ├── ZeroState/ Mocap/ Joystick/ LowPassFilter/
        ├── autobuild.sh / autoclean.sh / install.sh
        └── build_realrobot_orin_* / build_*  # 各构建配置产物
```

## 默认模型与后端

**NavSide 当前 ONNX 资产（`Navside-9.1/asset/models/`）**

- 编码器（当前）：`vae_orbbec_mix_ep30_deploy.onnx`
- 编码器（备选）：`vae_orbbec_pure_ep30_deploy.onnx`、`vae_encoder.onnx`
- 策略（当前）：`nav_policy.onnx`
- 策略（备选）：`policy.onnx`、`1/policy.onnx`

**RobotSide 当前 CPG 资产**

- `PhybotSoftware_c2/RL_deploy_cpg/model/phybot_cpg_policy.onnx`

**接口契约（冻结事实，见 `Navside-9.1/docs/navside_baseline.md`）**

- 深度特征维：2560
- obs 维：2576
- 策略频率：5 Hz
- 动作输出：3 floats（vx, vy, wz）
- 限幅：vx ∈ [0, 0.9]，wz ∈ [-0.5, 0.5]，walk_threshold 0.3，goal 容差 0.20
- 失败安全：无效输入 / 到达目标 → 零指令

## 相机角色（双视觉）

| 相机 | 序列号（默认） | 用途 | 激光 |
|---|---|---|---|
| A | `CPC8763000J0` | cuVSLAM 被动双目惯性 SLAM，出位姿 | 关 |
| B | `CPC8763000MZ` | NavSide 深度（Y16 → 米） | 开 |

两台为对称、仅 z 轴差、SRU 固定值，A/B 可互换（互换即在 `run_dual_camera.sh` 交换 `CAM_A`/`CAM_B` 两行，或用 `CUVSLAM_CAMERA_SERIAL` / `NAVSIDE_CAMERA_SERIAL` 覆盖）。

## 启动方式

### 1. 双相机一键启动（推荐：SLAM + 深度 + SRU 全链路）

```bash
cd ~/桌面/双视觉版本/cuvslam_migrate-9.1
./run_dual_camera.sh                          # 默认 --mode localize，交互式选地图
./run_dual_camera.sh --mode localize --map 12 --no-viz   # 指定地图重定位
./run_dual_camera.sh --mode map --map orbbec_map         # 建图
```

脚本内部同时拉起：
- 相机 A：`run_orbbec.sh`（cuVSLAM，位姿 → UDP 8082）
- 相机 B：`Navside-9.1` 的 `run_nav.py --real`（深度 → VAE/SRU → 命令 → UDP 8080）

### 2. 单独跑相机 A（cuVSLAM 建图 / 重定位）

```bash
cd ~/桌面/双视觉版本/cuvslam_migrate-9.1
./run_orbbec.sh                     # 默认 --mode localize
./run_orbbec.sh --mode map --map 名称
```

> `run_orbbec.sh` 用隔离 `venv/` 跑，并设置 `LD_LIBRARY_PATH` 定位 `libOrbbecSDK.so.2`；无 X 转发时自动指到本机 `DISPLAY=:0`（GDM 会话）。

### 3. 单独跑 NavSide（相机 B 深度 + SRU，需先有相机 A 位姿）

```bash
cd ~/桌面/双视觉版本/Navside-9.1
.venv_navside/bin/python scripts/run_nav.py --real --config config/nav.yaml
```

切换深度后端只改 `config/nav.yaml` 一行：

```yaml
depth:
  backend: orbbec      # 原为 realsense / zed
```

### 4. WASD 键盘直控（Navside-9.4，绕过 SRU，用于底盘联调）

```bash
cd ~/桌面/双视觉版本/Navside-9.4
python3 scripts/wasd_control.py --list                 # 列出键盘设备
python3 scripts/wasd_control.py                        # 运行（自动发现键盘）
```

速度映射：W 前进 +0.5 / S 后退 −0.3 / A 左转 +0.4 / D 右转 −0.4 / Q·E 左右横移 ±0.1，松开即停车；起步带阶梯增长（`--ramp-step 0.02`）防抖。方向反了加 `--invert-x` / `--invert-yaw`。

> 读 `/dev/input/eventX` 需 `input` 组权限或 root，见 `Navside-9.4/README.md` 三种解法。

### 5. 机器人侧（PhybotSoftware_c2）

先编译，再运行对应入口：

```bash
cd ~/桌面/双视觉版本/PhybotSoftware_c2\ arm/PhybotSoftware_c2
./autobuild.sh          # 交互菜单选入口（mujoco_sim / realrobot）
./autoclean.sh          # 切换入口前先清一次
```

机器人侧 `RL_deploy_cpg` 的 `InitUDP()` 监听 UDP 8080，收到 `CommandPacket {vx, vy, omega_z}` 后更新期望速度。联调顺序：**先起机器人侧，再起 NavSide / wasd**。

## UDP 端口约定

| 端口 | 方向 | 协议 | 内容 |
|---|---|---|---|
| 8082 | cuVSLAM → NavSide | `<7d`（56B 小端） | `[px,py,pz, qw,qx,qy,qz]`，Z-up 世界系位姿 |
| 8080 | NavSide / wasd → 机器人 | `3f`（12B） | `CommandPacket {vx, vy, omega_z}` |
| 8081 | NavSide 本地监听 | — | sim 模式 `SimpleUdpRobotComm` 收 `NavStatePacketV2`（84B `<IHHId16f`） |

## timing / perf 开关

默认关闭，打开只用于观测，不改变控制逻辑：

- `NAVSIDE_TIMING=1`
- `ROBOTSIDE_TIMING=1`
- `SRU_TIMING=1`

## 成功标准与运行限制

**闭环关键日志 / 状态**

- 深度后端 `backend=orbbec`，编码器加载 `vae_orbbec_mix_ep30_deploy.onnx`
- `[RobotComm] UDP pose socket listening on 127.0.0.1:8082`（cuVSLAM 位姿已接入）
- 命令经 UDP 8080 发往机器人，机器人侧 `InitUDP()` 正常收 `CommandPacket`

**已知限制**

- NavSide viewer 必须在本地 Orin 图形会话运行，不能依赖 remote tty；`run_orbbec.sh` 已处理 `DISPLAY=:0`，但 `ssh -X` 场景行为不同。
- cuVSLAM 的 `pyorbbecsdk` 是 GitHub releases 自建 wheel，PyPI 上没有；`run_orbbec.sh` 用隔离 `venv/` 跑，避免污染 `Navside-9.1` 的 `.venv_navside`。
- 本方案是**双相机实机路线**：定位与深度分属两台 Orbbec 336L，两相机需各自稳定枚举（`enumerate_devices.py` 可查序列号），串口/udev 权限按 `orbbec_gemini336l_integration.md` 配置。
- `Navside-9.4` 为无深度、无地图、无 SRU 的轻量旁路，仅用于运动底盘联调，不是导航主路径。

## 总结

SRU 导航策略（IsaacLab 训练、ONNX 导出）在 Orin 上以**双相机**方式完成实机闭环：相机 A 用 cuVSLAM 被动双目惯性 SLAM 输出全局位姿，经 UDP 8082 取代原机器人位姿回传；相机 B 提供深度，经 VAE 编码 + SRU 策略输出 `vx/vy/wz`，再经 UDP 8080 驱动 Phybot RobotSide 的 CPG/RL 行走控制器。同时保留 `Navside-9.4` 的 WASD 直控旁路用于底盘快速联调，构成「感知 → 定位 → 推理 → 速度 → 执行 → 状态回馈」的完整链路。
