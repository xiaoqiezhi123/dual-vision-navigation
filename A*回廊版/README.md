Navside-9.1   导航栈（调度器 + PathAware SRU）
cuvslam_migrate-9.1   相机 A 定位 + A* 规划


# A* 回廊版 — 双视觉分段导航部署说明（9.1）

本目录只保留「A* 回廊版」分段导航主线交付的最小集合：**相机 A 的 cuVSLAM 被动双目惯性 SLAM（出位姿 + 地图锚定）**、**相机 B 的 NavSide 深度感知 + PathAware SRU 导航推理（出速度）**、**A\* 分段规划调度器（切段、到达判定与重定位编排）**，以及一套绕过 SRU 的 **WASD 键盘直控**旁路。四者构成「定位 → A* 规划 → 推理 → 速度 → 到达停车 → localize 重定位 → 下一段」的完整分段闭环。

## 当前交付范围

| 目录 | 角色 | 验证状态 |
|---|---|---|
| `cuvslam_migrate-9.1` | 相机 A：cuVSLAM SLAM/地图锚定 + map2d 工具链 + A* 路线 | 真机已跑（被动双目惯性 SLAM，位姿经 UDP 8082 发 NavSide） |
| `Navside-9.1` | 导航栈：调度器 + 深度 + PathAware SRU 推理 | 真机分段导航主路径（9.30 位姿积压/VIO 交接修复复测通过） |
| `Navside-9.1.zip` / `cuvslam_migrate-9.1.zip` | 上述两目录的打包副本 | 分发用 |

> 与上级「双视觉直道版」的差异：本目录是**回廊分段版**。同一条完整链路（cuVSLAM 位姿 → UDP 8082 → NavSide SRU → UDP 8080），但导航不再一次跑完全程，而是由调度器按 A* 路线切成多段：每段行进 → 到点停车（VIO 判定到达）→ `localize` 地图锚定 → 下一段。SRU 由普通 `nav_policy.onnx` 换成路径感知的 `nav_policy_pathaware.onnx`，并支持 A* 15 参考点（起点 + 13 中间点 + 终点）整段加载。

## 主流水线图

```
相机 A (Orbbec 336L #1, IR 双目 848×480@10, 激光关, CPC8763000MZ)
   │  track() → odom 增量；localize → 地图锚定全局位姿
   │  pub_pose(10Hz) → OpenCV→Z-up 变换
   ▼  UDP 127.0.0.1:8082   struct "<7d" = [px,py,pz, qw,qx,qy,qz] (56B)
NavSide-9.1  RobotComm._pose_sock → pos_w
   │  文件通道 NAVSIDE_CMD_FILE / NAVSIDE_SCHED_FILE / SLAM_SCHED_FILE
   │
调度器 task_nav_scheduler.py —— A* 分段规划 + 状态机编排
   │  每段：下发 goal → 行进；到点：nav "A"（停推理+清 LSTM+发零）
   │  → slam "pause"（关 VIO）→ 提示 localize → 锚定偏差 ≤ 容差 → 下一段
   ▼
相机 B (Orbbec 336L #2, Y16 1280×720@15, 激光开, CPC8763000J0)
   │  OrbbecDepthPerception → float32 米
   ▼
VAE 编码器 (vae_orbbec_mix_ep30_deploy.onnx) → 深度特征 2560 维
   │  + 路径/位姿状态 → obs
   ▼
PathAware SRU 策略 (nav_policy_pathaware.onnx, 5Hz) → vx, vy, wz (clamp)
   ▼  UDP 127.0.0.1:8080   struct.pack("3f", vx, vy, wz)
机器人侧 PhybotSoftware_c2 / RL_deploy_cpg
   │  InitUDP() 监听 8080 → CommandPacket {vx, vy, omega_z}
   ▼  CPG 策略 → 状态机 → MotorList → 电机
```

## 目录结构

```
A*回廊版/
├── cuvslam_migrate-9.1/          # 相机 A：cuVSLAM 定位 + map2d + A*
│   ├── run_orbbec.sh             #   cuVSLAM 启动（--mode localize|map，--map 名称）
│   ├── venv/                     #   隔离环境（符号链接 → /home/amov/cuvslam_migrate/venv）
│   ├── orbbec/
│   │   ├── run_vio_tasknav.py    #   任务导航 VIO 主循环（调度器 fork 管道子进程）
│   │   ├── run_vio.py / run_vio_localize_nolc / run_vio_mapnav.py
│   │   ├── camera_utils.py / enumerate_devices.py
│   │   ├── udp_pose_sender.py / foxglove_odom_server.py
│   │   └── visualizer.py / visualizer_mapping.py / feature_map.py
│   ├── maps2d/
│   │   └── viz_test_02/v6_reviewed/   # 默认任务地图（选点、A* 均基于它）
│   ├── run_map2d_*.sh            #   builder 建图 / annotator 标注 / picker 选点
│   │                             #   astar 规划 / rerun 可视化
│   ├── annotations.json          #   地图标注（选点窗口保存的任务点）
│   └── read.md
│
├── Navside-9.1/                  # 导航栈：调度器 + PathAware SRU
│   ├── run_task_nav.sh           #   一键：调度器拉起 SLAM 与 NavSide
│   ├── scripts/
│   │   ├── task_nav_scheduler.py #   调度器主程序
│   │   ├── run_nav.py            #   NavSide 入口（--real / --sim-control）
│   │   ├── pose_diagnostics.py   #   位姿链路诊断（prepare / watch / report）
│   │   ├── analyze_slam_log.py   #   SLAM 日志分析
│   │   └── wasd_control.py       #   WASD 键盘直控（实际在 ~/nav_arm_mujoco/）
│   ├── navside/
│   │   ├── runtime.py            #   顶层编排
│   │   ├── adapter.py            #   深度→特征→obs→ONNX→动作→clamp
│   │   ├── bridge.py             #   RobotComm：UDP 位姿(8082) + 命令(8080)
│   │   ├── segments.py / pose_health.py / pose_trace.py   # 分段与位姿健康
│   │   └── depth.py / real.py / state.py / mode.py / timing.py
│   ├── perception/orbbec_depth_perception.py   # 深度后端
│   ├── config/
│   │   ├── task_nav.yaml         #   调度器配置（SRU 开关、A*、容差）
│   │   ├── nav.yaml              #   后端/模型/限幅/UDP 端口
│   │   └── task_points_selected.json   # 已保存任务点
│   ├── asset/models/             #   VAE / policy ONNX
│   ├── .venv_navside/            #   正式运行环境
│   └── *.md                      #   全套文档（见「文档索引」）
│
├── Navside-9.1.zip               # 打包副本
└── cuvslam_migrate-9.1.zip
```

## 默认模型与后端

**NavSide 当前 ONNX 资产（`Navside-9.1/asset/models/`）**

- 编码器（当前）：`vae_orbbec_mix_ep30_deploy.onnx`
- 编码器（备选）：`vae_orbbec_pure_ep30_deploy.onnx`、`vae_encoder.onnx`
- 策略（当前）：`nav_policy_pathaware.onnx`（路径感知，A* 分段版）
- 策略（备选）：`nav_policy.onnx`（原单目标）

**接口契约（冻结事实，见 `Navside-9.1/config/task_nav.yaml`）**

- A* 参考点：15 点 = 起点 + 13 中间点 + 终点（含起点和终点）
- 路径高度：`robot_height_z: 0.695`
- 策略频率：5 Hz；动作输出：3 floats（vx, vy, wz）
- 到达判定：VIO 位姿距任务点 ≤ `arrive_tolerance_m: 0.7` 且持续 `arrive_confirm_s: 1.0`
- 锚定校验：`anchor_verify_tolerance_m: 5.0`（`force` 仅能跳过此项）
- A* 简化：中心偏好 `enabled: true`，`simplification_tolerance_m: 0.10`（简化折线最多偏离原 A* 路径的距离）
- 失败安全：短时位姿断流 → 零速等待；长断流/突变/非法数据 → 须 `localize`；无效输入 → 零指令

## 相机角色（双视觉）

| 相机 | 序列号（默认） | 用途 | 激光 |
|---|---|---|---|
| A | `CPC8763000MZ` | cuVSLAM 被动双目惯性 SLAM + 地图锚定 | 关 |
| B | `CPC8763000J0` | NavSide 深度（Y16 → 米） | 开 |

两台相机 cuVSLAM 独占其一、NavSide 独占另一台，分别用 `CUVSLAM_CAMERA_SERIAL` / `NAVSIDE_CAMERA_SERIAL` 环境变量指定串号，避免枚举错位。查串号用 `cuvslam_migrate-9.1/orbbec/enumerate_devices.py`。

## 启动方式

### 1. A* 调度器一键启动（推荐：SLAM + A* + SRU 全链路）

```bash
cd ~/navside_real/NavSide_log/Navside-9.1

# 完整导航：调度器自动拉起 cuVSLAM（重定位/锚定）与 NavSide（SRU）
./run_task_nav.sh --sru --task-points-file config/task_points_selected.json

# 先关 SRU 对照：只跑调度器 + SLAM + A*（仍启动 SLAM 相机）
./run_task_nav.sh --no-sru --task-points-file config/task_points_selected.json

# 只选任务点保存 / 只检查已保存任务点与地图（均不启动相机）
./run_task_nav.sh --pick-only
./run_task_nav.sh --check-only
```

调度器会自动启动 SLAM 并重定位，**不需要先单独跑 `run_orbbec.sh --mode localize`**。若先单独检查过定位，须退出该程序并等相机关闭后再启动调度器。

运行中终端指令（在调度器终端输入）：

| 指令 | 作用 |
|---|---|
| `localize` / `l` | 到点后开始下一段；中途停车/失败后继续当前未完成目标 |
| `pause` | 暂停当前段（SRU OFF 时须独立遥控停车） |
| `force` | 仅跳过成功锚定的偏差校验（不能跳过 A* 失败或缺失定位） |
| `status` / `s` | 打印当前状态 |
| `quit` / `q` | 优雅退出（NavSide 最后发零速） |

### 2. 独立双终端（手动，不带调度器）

**终端 1 —— cuVSLAM（相机 A，激光关）**

```bash
cd ~/cuvslam_migrate-9.1
CUVSLAM_CAMERA_SERIAL=CPC8763000MZ ./run_orbbec.sh --mode localize --map 12
```

**终端 2 —— NavSide（相机 B，激光开）**

```bash
cd ~/navside_real/NavSide_log/Navside-9.1
NAVSIDE_CAMERA_SERIAL=CPC8763000J0 python3 scripts/run_nav.py --real --config config/nav.yaml --show-depth
```

（`--show-depth` 是看深度图窗口用的，不需要可去掉。）

### 3. WASD 键盘直控（绕过 SRU，用于底盘联调）

```bash
sudo ~/nav_arm_mujoco/.venv_navside/bin/python scripts/wasd_control.py --
```

（`wasd_control.py` 在 Orin 的 `~/nav_arm_mujoco/` 仓库内，不在本目录。）

### 4. 位姿监听版（排查位姿问题用）

```bash
cd ~/navside_real/NavSide_log/Navside-9.1
# 准备并开启监听后启动调度器
python3 scripts/pose_diagnostics.py prepare
source logs/pose_diag/enable_latest.sh
./run_task_nav.sh --sru --task-points-file config/task_points_selected.json

# 另一终端实时查看
python3 scripts/pose_diagnostics.py watch --latest

# 结束后在调度器终端生成报告
python3 scripts/pose_diagnostics.py report --latest
```

### 5. 地图制作与选点（从零到导航的完整工作流）

```bash
# 1) 重定位（有现成地图时跳过建图，直接锚定）
cd ~/cuvslam_migrate-9.1
CUVSLAM_CAMERA_SERIAL=CPC8763000MZ ./run_orbbec.sh --mode localize

# 2) 地图标注（人工擦除/修正候选）
./run_map2d_annotator.sh \
  --source-json maps2d/viz_test_02/v6_reviewed/source_map.json \
  --annotations maps2d/viz_test_02/v6_reviewed/annotations.json \
  --hide-candidates --erase-radius-m 0.30 \
  --out maps2d/viz_test_02/annotations_next.json

# 3) A* 参考（生成 A* 路线与 15 参考点）
./run_map2d_astar.sh

# 4) 任务点选点（地图上按顺序选目标 1、2、3……，不预选起点）
cd ~/navside_real/NavSide_log/Navside-9.1
./run_task_nav.sh --pick-only

# 5) 启动 A* 版本调度器
./run_task_nav.sh --sru --task-points-file config/task_points_selected.json
```

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

- 调度器终端出现 `[TASK]` 行，SLAM 窗口 `anchor=busy/ok/fail` 正常流转
- `[RobotComm] UDP pose socket listening on 127.0.0.1:8082`（cuVSLAM 位姿已接入）
- 每段到达：先 nav `A`（停推理 + 清 LSTM + 发零）→ 再 slam `pause`（关 VIO）→ 提示 `localize`
- 全部任务点完成 → DONE → 两端优雅退出（NavSide 最后发零速）

**已知限制**

- NavSide viewer 必须在本地 Orin 图形会话运行；`run_orbbec.sh` 已处理 `DISPLAY=:0`，`ssh -X` 场景行为不同。
- `cuvslam_migrate-9.1/venv` 是指向 `/home/amov/cuvslam_migrate/venv` 的符号链接，拷到新机器需重建或改链；文档中的 `/home/amov/...` 均为 Orin 部署机路径。
- 任务点与地图绑定（地图指纹校验）：地图变了必须重新 `--pick-only`，不能绕过身份校验。
- 短时位姿断流：先零速等待，稳定恢复后继续原段；长断流/突变/非法数据及人工停车须 `localize`。
- 已知卡顿问题：8849 为卡顿版本 1 楼地图（伴随 IMU 采集断续），`test01`/`test02` 为流畅 1 楼两圈地图；排查方向见「改进方向」。

## 文档索引

**Navside-9.1**（按阅读顺序）：

| 文档 | 内容 |
|---|---|
| [分段A星接入_快速使用.md](Navside-9.1/分段A星接入_快速使用.md) | 主入口：启动、SRU 开关、地图选点（含 2026-10-08 A* 路线预览更新） |
| [分段导航_架构文档.md](Navside-9.1/分段导航_架构文档.md) | 三进程架构与数据流详解 |
| [现场测试与上下文交接_20260929.md](Navside-9.1/现场测试与上下文交接_20260929.md) | 已确认需求与恢复对话指令 |
| [今日产出_全部快速指令_20260929.md](Navside-9.1/今日产出_全部快速指令_20260929.md) | 地图制作到导航回退的常用命令汇总 |
| [明日实测步骤与排查_20260930.md](Navside-9.1/明日实测步骤与排查_20260930.md) | 现场测试步骤 |
| [两轮实测排查与启动顺序_20260930.md](Navside-9.1/两轮实测排查与启动顺序_20260930.md) | 相机找不到、任务点匹配失败、位姿跳变排查 |
| [位姿等待恢复_测试说明_20260930.md](Navside-9.1/位姿等待恢复_测试说明_20260930.md) | 短断流零速等待 / 长断流须 localize |
| [位姿积压修复与复测_20260930.md](Navside-9.1/位姿积压修复与复测_20260930.md) | 位姿积压问题修复与验证 |
| [VIO衔接与居中规划_修复复测_20260930.md](Navside-9.1/VIO衔接与居中规划_修复复测_20260930.md) | VIO/IMU 交接修复、中心偏好 A* |
| [位姿链路诊断_快速使用_20260930.md](Navside-9.1/位姿链路诊断_快速使用_20260930.md) | pose_diagnostics 工具 |
| [PathAware接入分析_20260929.md](Navside-9.1/PathAware接入分析_20260929.md) | PathAware SRU 接入分析 |
| [cuvslam_上游问题总结.md](Navside-9.1/cuvslam_上游问题总结.md) | cuVSLAM 上游问题记录 |

**cuvslam_migrate-9.1**：

| 文档 / 目录 | 说明 |
|---|---|
| [read.md](cuvslam_migrate-9.1/read.md) | 两个 VIO 脚本模式：关回环 `run_vio_localize_nolc` / 地图直发位姿 `run_vio_mapnav.py` |
| `maps2d/viz_test_02/v6_reviewed/` | 默认任务地图；`astar_routes/`、`astar_example_20260929/` 为 A* 路线产物 |

**地图数据备注**：`8848` 22m 直道 · `8849` 卡顿版本 1 楼 · `test01` 流畅 1 楼两圈 · `test02` 流畅 1 楼两圈 + 逆直道（当前默认）。

## 更新记录与改进方向

| 日期 | 变更 |
|---|---|
| 2026-09-17 | 「方案一」架构：行进纯 VIO 不建图 + 锚定专用同步 tracker |
| 2026-09-29 | 地图选目标、每次重定位后 A* 与 15 点 XYZ 打印、失败暂停、崩溃重启续跑；15 参考点接入 PathAware SRU |
| 2026-09-30 | 短时位姿断流零速等待；VIO/IMU 交接修复；调度器默认启用中心偏好 A*；位姿积压修复 |
| 2026-10-08 | 地图选点窗口加入相邻任务目标的 A* 路线与 15 点预览，已有目标自动载入规划 |

**改进方向（2026-09-30 提出，待办）**

1. 速度降低 2. SRU 频率降低 3. 换 VAE 4. A* 参考点中心化
5. cuVSLAM 的 `tracker()` 问题：卡顿伴随 IMU 采集断续 —— 真正切换 tracker → 修正暂停/预热握手 → 分离 IMU 时间戳与缓存 → 核对新旧位姿基线 → 静止及运动对照测试
6. 加上述保护措施的必要性
7. A* 目前保留转角，需确定 SRU 是否非全程等间距

## 总结

回廊场景下，把一条长路径交给 A* 切成多段、每段由 PathAware SRU 独立导航的分段方案，在 Orin 上以**双相机**方式完成实机闭环：相机 A 用 cuVSLAM 被动双目惯性 SLAM 输出全局位姿并经地图锚定消除累计误差，经 UDP 8082 取代原机器人位姿回传；调度器按 A* 15 参考点逐段下发目标，到点停车后 `localize` 重定位再续下一段；相机 B 提供深度，经 VAE 编码 + PathAware SRU 输出 `vx/vy/wz`，再经 UDP 8080 驱动 Phybot RobotSide 的 CPG/RL 行走控制器。同时保留 WASD 直控旁路用于底盘快速联调，构成「定位 → A* 规划 → 推理 → 速度 → 执行 → 到达 → 重定位」的完整分段链路。
