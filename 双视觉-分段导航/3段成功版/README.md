Navside-9.1   导航栈（调度器 + SRU）—— 3 段实机成功
cuvslam_migrate-9.1   相机 A 定位 + map2d
path-aware（x86仿真部署版）   x86 仿真 + PathAware 迁移


# 3段成功版 — 双视觉分段导航部署说明（2026-09-17）

本目录是分段导航「**方案一**」首次 **3 段实机跑通**的里程碑版本：**相机 A 的 cuVSLAM 被动双目惯性 SLAM（行进纯 VIO 出位姿 + 到点地图锚定）**、**相机 B 的 NavSide 深度感知 + SRU 导航推理（出速度）**、**任务导航调度器（切段、到达判定与重定位编排）**，另含一套 **x86 仿真部署的 PathAware（15 点路径）迁移**。三者构成「定位 → 推理 → 速度 → 到达停车 → localize 重定位 → 下一段」的完整分段闭环，实机 6 次测试 5 次成功。

## 当前交付范围

| 目录 | 角色 | 验证状态 |
|---|---|---|
| `cuvslam_migrate-9.1` | 相机 A：cuVSLAM SLAM/地图锚定 + map2d 工具链 | 真机已跑（纯 VIO 行进 + 锚定，位姿经 UDP 8082 发 NavSide） |
| `Navside-9.1` | 导航栈：任务调度器 + 深度 + SRU 推理 | 3 段实机成功（6 次测试 5 次通过，参考地图 8848） |
| `path-aware（x86仿真部署版）` | x86 仿真 + PathAware 迁移（15 点路径 + 新 SRU） | 仿真验证，待按说明移植到真机 `Navside-9.1/` |
| `Navside-9.1.zip` / `cuvslam_migrate-9.1.zip` | 上述两目录的打包副本 | 分发用 |
| `1.md` / `1~4.txt` | 6 次实机测试的日志与分析（`3.txt` 为失败崩溃样例） | 实测留档 |

> 与上级「双视觉直道版」的差异：本目录是**分段导航版**。同一条完整链路（cuVSLAM 位姿 → UDP 8082 → NavSide SRU → UDP 8080），但导航不再一次跑完全程，而是由调度器切成多段：每段行进（**纯 VIO 不建图**）→ 到点停车 → `localize` 地图锚定 → 下一段。这一改动的直接动因是全同步 SLAM 时后端内联工作（回环/PGO/关键帧优化）会饿死采集线程，导致行进位姿流卡顿 0.5s~125s，见「方案一核心设计」。

## 主流水线图

```
相机 A (Orbbec 336L #1, IR 双目 848×480@10, 激光关, CPC8763000MZ)
   │  行进：track() 纯 VIO 前端（slam_cfg=None）→ odom 增量
   │  到点：localize 地图锚定（专用同步 tracker，读磁盘参考图）
   │  pub_pose(10Hz) → OpenCV→Z-up 变换
   ▼  UDP 127.0.0.1:8082   struct "<7d" = [px,py,pz, qw,qx,qy,qz] (56B)
NavSide-9.1  RobotComm._pose_sock → pos_w
   │  文件通道 NAVSIDE_CMD_FILE / NAVSIDE_SCHED_FILE / SLAM_SCHED_FILE
   │
调度器 task_nav_scheduler.py —— 分段状态机（3 任务点，参考地图 8848）
   │  每段：下发 goal + S；到点：nav "A"（停推理）→ 0.5s → slam "pause"（关 VIO）
   │  → 提示 localize → 锚定偏差 ≤ 容差 → 下一段
   ▼
相机 B (Orbbec 336L #2, Y16 1280×720@15, 激光开, CPC8763000J0)
   │  OrbbecDepthPerception → float32 米
   ▼
VAE 编码器 (vae_orbbec_mix_ep30_deploy.onnx) → 深度特征 2560 维
   │  + 位姿/速度状态 → obs
   ▼
SRU 策略 (nav_policy.onnx, 5Hz) → vx, vy, wz (clamp)
   ▼  UDP 127.0.0.1:8080   struct.pack("3f", vx, vy, wz)
机器人侧 PhybotSoftware_c2 / RL_deploy_cpg
   │  InitUDP() 监听 8080 → CommandPacket {vx, vy, omega_z}
   ▼  CPG 策略 → 状态机 → MotorList → 电机
```

## 目录结构

```
3段成功版/
├── cuvslam_migrate-9.1/          # 相机 A：cuVSLAM 定位 + map2d
│   ├── run_orbbec.sh             #   cuVSLAM 启动（--mode localize|map，--map 名称）
│   ├── run_orbbec_tasknav.sh     #   任务导航版（--no-viz --ref-map <名> --scheduler）
│   ├── run_dual_camera.sh        #   双相机一键
│   ├── venv/                     #   隔离环境（pyorbbecsdk + cuvslam）
│   ├── orbbec/
│   │   ├── run_vio_tasknav.py    #   任务导航 VIO 主循环（方案一：纯 VIO + 锚定）
│   │   ├── camera_utils.py / enumerate_devices.py
│   │   ├── udp_pose_sender.py / foxglove_odom_server.py
│   │   ├── depth_shm.py / feature_map.py / map2d_picker.py
│   │   └── 8848/ …（参考地图目录，锚定只读，退出不写回）
│   └── read.md
│
├── Navside-9.1/                  # 导航栈：任务调度器 + SRU
│   ├── run_task_nav.sh           #   一键：调度器拉起 SLAM 与 NavSide
│   ├── scripts/
│   │   ├── task_nav_scheduler.py #   调度器主程序（分段状态机）
│   │   ├── run_nav.py            #   NavSide 入口（--real / --sim-control）
│   │   └── analyze_slam_log.py   #   SLAM 日志分析（位姿空洞 / 掉帧交叉验证）
│   ├── navside/
│   │   ├── runtime.py            #   顶层编排
│   │   ├── adapter.py            #   深度→特征→obs→ONNX→动作→clamp
│   │   ├── bridge.py             #   RobotComm：UDP 位姿(8082) + 命令(8080)
│   │   └── real.py / sim.py / state.py / mode.py / depth.py / timing.py
│   ├── perception/orbbec_depth_perception.py   # 深度后端
│   ├── config/
│   │   ├── task_nav.yaml         #   调度器配置（参考地图 8848、3 任务点、容差）
│   │   ├── nav.yaml              #   后端/模型/限幅/UDP 端口
│   │   └── nav_deploy.yaml
│   ├── asset/models/             #   VAE / policy ONNX
│   ├── .venv_navside/            #   正式运行环境
│   ├── 分段导航_架构文档.md       #   方案一架构与核心设计决策
│   └── cuvslam_上游问题总结.md
│
├── path-aware（x86仿真部署版）/   # x86 仿真 + PathAware 迁移
│   ├── NavSide/                  #   x86 仿真 NavSide（adapter 含路径编码）
│   │   ├── navside/path_planner.py / path_editor.py   # 15 点路径
│   │   ├── config/nav_molmospaces*.yaml               # MolmoSpaces / ProcTHOR 仿真
│   │   └── asset/models/         #   nav_policy / policy / policy_3 / vae_encoder
│   ├── 真机分段导航_PathAware迁移改动说明_GPT.md   # 迁移到真机的增量改动说明
│   └── NavSide.zip
│
├── Navside-9.1.zip               # 打包副本
├── cuvslam_migrate-9.1.zip
├── 1.md                          # 6 次实测分析（成功改动 + 潜在失败点）
└── 1.txt ~ 4.txt                 # 四遍测试日志（1/2/4 成功，3 失败崩溃）
```

## 默认模型与后端

**NavSide 真机 ONNX 资产（`Navside-9.1/asset/models/`）**

- 编码器（当前）：`vae_orbbec_mix_ep30_deploy.onnx`
- 编码器（备选）：`vae_orbbec_pure_ep30_deploy.onnx`、`vae_encoder.onnx`
- 策略（当前）：`nav_policy.onnx`
- 策略（备选）：`policy.onnx`

**PathAware（x86 仿真）资产（`path-aware（x86仿真部署版）/NavSide/asset/models/`）**

- `nav_policy.onnx`、`policy.onnx`、`policy_3.onnx`、`vae_encoder.onnx`；配合 `path_planner.py` / `path_editor.py` 做 15 点路径输入，观测维度由 2576 扩到 **2636**（2560 深度 + 16 状态 + 60 路径），迁移方式见 GPT 说明文档。

**接口契约（冻结事实，见 `Navside-9.1/config/task_nav.yaml`）**

- 参考地图：`8848`（22m 直道）；任务点：3 个 —— cuvslam 系 `(0,0,4.5)`、`(0,0,11.0)`、`(0,0,22.0)`
- 路径高度：`robot_height_z: 0.695`
- 策略频率：5 Hz；动作输出：3 floats（vx, vy, wz）
- 到达判定：VIO 位姿距任务点 ≤ `arrive_tolerance_m: 0.7` 且持续 `arrive_confirm_s: 1.0`
- 锚定校验：`anchor_verify_tolerance_m: 5.0`；单段超时 `segment_arrive_timeout_s: 600`（超时仅告警）
- 失败安全：无效输入 / 到达目标 → 零指令

## 相机角色（双视觉）

| 相机 | 序列号（默认） | 用途 | 激光 |
|---|---|---|---|
| A | `CPC8763000MZ` | cuVSLAM 被动双目惯性 SLAM（行进纯 VIO + 到点锚定） | 关 |
| B | `CPC8763000J0` | NavSide 深度（Y16 → 米） | 开 |

> 串号是权威标识（运行日志明确 `相机(SLAM)=CPC8763000MZ`、`相机(NavSide深度)=CPC8763000J0`）。各文档里 A/B 字母曾互换过，以串号为准；两台可互换，分别用 `CUVSLAM_CAMERA_SERIAL` / `NAVSIDE_CAMERA_SERIAL` 覆盖。查串号用 `cuvslam_migrate-9.1/orbbec/enumerate_devices.py`。

## 启动方式

### 1. 调度器一键启动（推荐：SLAM + SRU 全链路）

```bash
cd ~/navside_real/NavSide_log/Navside-9.1
./run_task_nav.sh
```

脚本内部自动拉起：`run_orbbec_tasknav.sh --no-viz --ref-map 8848 --scheduler`（cuVSLAM）+ `run_nav.py --real --config nav.yaml --show-depth`（NavSide，独立交互终端窗口）。参考地图与任务点数由 `config/task_nav.yaml` 决定，本版为 8848 / 3 点。

运行中终端指令（在调度器终端输入）：

| 指令 | 作用 |
|---|---|
| `localize` / `l` | 到点后开始下一段；中途停车/失败后继续当前未完成目标 |
| `pause` | 暂停当前段 |
| `status` / `s` | 打印当前状态 |
| `quit` / `q` | 优雅退出（NavSide 最后发零速） |

### 2. 独立双终端（手动，不带调度器）

```bash
# 终端 1 —— cuVSLAM（相机 A，激光关）
cd ~/cuvslam_migrate-9.1
CUVSLAM_CAMERA_SERIAL=CPC8763000MZ ./run_orbbec.sh --mode localize --map 8848

# 终端 2 —— NavSide（相机 B，激光开）
cd ~/navside_real/NavSide_log/Navside-9.1
NAVSIDE_CAMERA_SERIAL=CPC8763000J0 python3 scripts/run_nav.py --real --config config/nav.yaml --show-depth
```

### 3. 分析日志（定位卡顿 / 掉帧根因）

```bash
cd ~/navside_real/NavSide_log/Navside-9.1
python3 scripts/analyze_slam_log.py   # 位姿流空洞（>0.5s）+ IMU/相机掉帧交叉验证
```

### 4. PathAware x86 仿真（15 点路径验证，另见其目录）

PathAware 迁移说明见 [真机分段导航_PathAware迁移改动说明_GPT.md](path-aware（x86仿真部署版）/真机分段导航_PathAware迁移改动说明_GPT.md)，按说明把路径编码移植进真机 `Navside-9.1/`，不改 cuVSLAM / bridge / 机器人协议。

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

## 方案一核心设计（为什么 3 段能跑通）

见 `Navside-9.1/分段导航_架构文档.md`，关键四点：

1. **行进 = 纯 VIO 不建图**：`slam_cfg=None`，track 只跑 VIO 前端（特征帧间跟踪 + IMU 融合）；段内全局位姿 = 锚定位姿 ⊕ odom 帧间增量。根因是全同步 SLAM 的后端内联工作（回环/PGO/关键帧优化）饿死采集线程。
2. **锚定 = 专用同步 tracker + 预热**：根治绑定层 GIL 崩溃（sync 模式下回调在主线程持 GIL 同步执行）；相机线程预热取帧，主线程用同一帧同一时间戳执行 localize。
3. **定位搜索参数**：coarse 25m/1m/0.15rad → mid 4m/0.5m/0.06 → fine 1m/0.2m/0.03；猜测位姿 = pub_pose 的 XZ、Y=0、单位四元数（不注入累积姿态漂移）。
4. **到达与停机顺序**：先 nav `A`（停推理）→ 0.5s → 再 slam `pause`（关 VIO，冻结状态源）。

容错：SLAM 意外崩溃 → 10s 延迟自动重启（相机 USB 复位）→ 启动锚定续接当前任务点，上限 3 次。

## 成功标准与运行限制

**闭环关键日志 / 状态**

- `[TASK] 启动锚定成功，坐标系已对齐参考地图全局系`
- 每段 `开始第 N/3 段` → `第 N 段到达`（nav `A` → slam `pause`）→ 提示 `localize`
- `锚定位姿 (x=…, z=…) 距任务点 N 偏差 X.XXm`，偏差 ≤ 5.0m 即推进
- 3 点完成 → 两端优雅退出

**实机测试结论（6 次，5 成功 1 失败）**

- 成功样例 `1.txt` / `2.txt` / `4.txt`：全程 3 段走完，锚定偏差 0.06~2.04m。
- 失败样例 `3.txt`（崩溃）：① 启动即 `RuntimeError: 未找到序列号 CPC8763000MZ 的相机`（USB 需重插/重启）；② 行进中 SLAM 进程意外退出 2 次后 `localize FAILED (coarse): Can't localize in map using provided image and guess`。
- 分析思路见 `1.md`（成功改动点 + 潜在失败点排查，只给思路不改代码）。

**已知限制**

- 行进段存在周期性卡顿：`analyze_slam_log.py` 报「行进段异常卡顿 5 次（主线程被 SLAM 内联工作阻塞）」，伴随 `IMU drops` 与 `Camera stream message drop`（timestamp gap 超阈值）——方案一已把后端工作从行进路径移除，但仍偶发，后续版本继续优化。
- 相机枚举不稳定：SLAM 相机找不到是失败主因之一，需重插 USB 或重启。
- NavSide viewer 必须在本地 Orin 图形会话运行；`run_orbbec.sh` 已处理 `DISPLAY=:0`，`ssh -X` 场景行为不同。
- 文档中的 `/home/amov/...` 均为 Orin 部署机路径；`cuvslam_migrate-9.1/venv` 为符号链接，拷到新机器需重建或改链。

## 文档索引

| 文档 | 内容 |
|---|---|
| [Navside-9.1/分段导航_架构文档.md](Navside-9.1/分段导航_架构文档.md) | 方案一架构、状态机、四项核心设计决策（含实证） |
| [Navside-9.1/cuvslam_上游问题总结.md](Navside-9.1/cuvslam_上游问题总结.md) | cuVSLAM 上游问题记录 |
| [1.md](1.md) | 6 次实测分析：成功改动哪些方面 + 潜在失败原因（只给思路） |
| [1.txt](1.txt) / [2.txt](2.txt) / [3.txt](3.txt) / [4.txt](4.txt) | 四遍测试日志（1/2/4 成功，3 失败崩溃） |
| [path-aware（x86仿真部署版）/真机分段导航_PathAware迁移改动说明_GPT.md](path-aware（x86仿真部署版）/真机分段导航_PathAware迁移改动说明_GPT.md) | PathAware 15 点路径 + 新 SRU 迁移到真机的增量改动说明 |
| [cuvslam_migrate-9.1/read.md](cuvslam_migrate-9.1/read.md) | cuVSLAM 侧 VIO 脚本说明 |

## 更新记录

| 日期 | 变更 |
|---|---|
| 2026-09-17 | 「方案一」里程碑：行进纯 VIO 不建图 + 锚定专用同步 tracker，8848 地图 3 段实机跑通（6 测 5 成） |
| 2026-09-29 | 新增 `path-aware（x86仿真部署版）`：x86 仿真验证 15 点路径输入 + 新 SRU（2636 维观测），提供真机迁移说明 |

## 总结

分段导航「方案一」以 **3 段实机跑通**（6 测 5 成）验证了核心思路：行进阶段把 cuVSLAM 收敛为纯 VIO 前端，规避全同步 SLAM 后端内联工作导致的位姿流卡顿；到点后用专用同步 tracker 做地图锚定消除累计误差，再切下一段。相机 A 出全局位姿经 UDP 8082 取代原机器人位姿回传，相机 B 提供深度经 VAE + SRU 输出 `vx/vy/wz` 经 UDP 8080 驱动 Phybot CPG 行走。同目录的 `path-aware（x86仿真部署版）` 则把「单目标」扩展为「15 点路径 + 新 SRU」的下一阶段，供真机迁移，构成从「感知 → 定位 → 分段规划 → 推理 → 速度 → 执行」的完整分段链路。
