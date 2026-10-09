# Navside-9.4 — 键盘 WASD 直接控制机器人

本目录是 **Navside-9.1 的轻量衍生版本**，只做一件事：用键盘 WASD 直接产生速度命令，
通过 UDP 发送到机器人端，**绕过 SRU 模型推理**。原版本 `Navside-9.1` 完全未改动。

## 与 Navside-9.1 的区别

| | Navside-9.1（原版） | Navside-9.4（本目录） |
|---|---|---|
| 运动命令来源 | SRU 模型推理（按键 A/S/D/F/G 只选档：STANDBY/LOW/MED/EMERG） | 键盘 WASD 直接映射速度 |
| 深度相机 | 需要（Orbbec/RealSense） | 不需要 |
| 地图/重定位/任务加载 | 需要 | 不需要 |
| UDP 发送 | `RobotComm.send_command(vx,vy,wz)` | 同款 `struct.pack("3f", vx,vy,wz)` |

## 通信链路（与 9.1 完全一致）

```
键盘 WASD ──evdev(/dev/input/event5)──► wasd_control.py
                                            │ struct.pack("3f", vx, vy, wz)
                                            ▼ UDP 127.0.0.1:8080
                            机器人端 RL_deploy_cpg（PhybotSoftware_c2）
                              struct CommandPacket { float vx; float vy; float omega_z; }
```

机器人端 `RL_deploy_cpg/src/rl_deploy.cpp` 的 `InitUDP()` 监听 UDP 8080，`CommunicateWithPlanner()`
收到 `CommandPacket` 后更新 `js_vx_desire / js_vy_desire / js_OmegaZ_desire`（当 `control_mode != 0`）。

## 速度映射

| 按键 | 含义 | 速度 |
|---|---|---|
| W | 前进 | vx = +0.5 m/s |
| S | 后退 | vx = -0.3 m/s |
| A | 左转 | omega_z = +0.4 rad/s |
| D | 右转 | omega_z = -0.4 rad/s |
| Q | 左横移 | vy = +0.1 m/s |
| E | 右横移 | vy = -0.1 m/s |
| 松开全部 | 停车 | 发 0 速度 |
| Esc / Ctrl+C | 退出 | 退出前强制发 0 停车 |

方向反了？加 `--invert-x`（翻转前进后退）或 `--invert-yaw`（翻转左右转）。

**阶梯增长（起步防抖）**：速度不是一步到位，而是每周期向目标逐步逼近一个步长（默认
`--ramp-step 0.02`，50Hz 下即约 0.5 秒从 0 到满速）。起步/急停都平滑，避免冲击。步长越小越缓：
`--ramp-step 0.01`（约 1 秒到满速）、`--ramp-step 0.05`（约 0.2 秒到满速，更接近瞬发）。

## 用法

```bash
cd ~/navside_real/NavSide_log/Navside-9.4

# 列出键盘设备（检查权限/识别键盘）
python3 scripts/wasd_control.py --list

# 运行（自动发现键盘 + 读 config/nav_deploy.yaml）
python3 scripts/wasd_control.py

# 指定键盘 / 目标 / 半速 / 更缓的起步
python3 scripts/wasd_control.py --device /dev/input/event5 --host 127.0.0.1 --port 8080 --speed 0.5 --ramp-step 0.01
```

## 键盘读取权限（重要）

读 `/dev/input/eventX` 需要 `input` 组权限或 root。当前 `amov` 用户不在 `input` 组，
直接运行会报「无读取权限」。三种解法任选其一：

```bash
# 1) 临时（重启失效）
sudo chmod 666 /dev/input/event*

# 2) 持久（推荐，需重新登录生效）
sudo usermod -aG input $USER

# 3) 直接 root 运行
sudo python3 scripts/wasd_control.py
```

## 联调顺序

1. 先启动机器人端控制程序（`PhybotSoftware_c2` 的 realrobot 主程序，会 `InitUDP()` 监听 8080）。
2. 再运行 `wasd_control.py`。
3. 按键测试：W 前进 / S 后退 / A 左转 / D 右转，松开即停。
