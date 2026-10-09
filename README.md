# 双视觉导航：cuVSLAM + SRU + Phybot

本仓库整理双相机导航项目的五个模块：cuVSLAM 源码、Orbbec 定位部署、NavSide 导航推理、WASD 键盘控制，以及 Phybot 机器人控制端。相机 A 提供位姿，相机 B 提供深度，NavSide 将导航速度通过 UDP 发给机器人控制端。

这是现有工程的源码与资源快照，保留原目录名及层级。机器人端 `PhybotSoftware_c2 arm` 已包含源码和模型；构建产物及第三方二进制依赖需在部署环境中另行准备。

## 目录

```text
dual-vision-navigation/
├── README.md
├── UPLOAD.md                      # 首次上传 GitHub 的操作步骤
├── .gitignore
├── cuVSLAM-main1/
│   └── cuVSLAM-main/               # cuVSLAM 源码、构建配置、示例与上游许可
├── cuvslam_migrate-9.1/            # Orbbec 定位与双相机启动脚本
├── Navside-9.1/                    # 深度感知、VAE/SRU 推理及机器人资源
├── Navside-9.4/                    # WASD 键盘控制
├── PhybotSoftware_c2 arm/
│   └── PhybotSoftware_c2/          # CPG、状态机、电机控制及机器人资源
└── docs/
    ├── deployment-original.md     # 原根目录 README.md，完整保留供参考
    ├── robotside.md               # 机器人端上传范围与依赖说明
    └── lfs-placeholders.txt       # 原工程中尚未填充的上游 LFS 资源清单
```

| 模块 | 用途 | 主要入口 / 说明 |
| --- | --- | --- |
| cuVSLAM 源码 | CUDA 视觉里程计与建图库 | [上游 README](cuVSLAM-main1/cuVSLAM-main/README.md) |
| 定位部署 | Orbbec 双目惯性定位、位姿发送 | [run_vio.py](cuvslam_migrate-9.1/orbbec/run_vio.py)、[run_orbbec.sh](cuvslam_migrate-9.1/run_orbbec.sh) |
| NavSide 9.1 | 深度 → VAE/SRU → 速度命令 | [run_nav.py](Navside-9.1/scripts/run_nav.py)、[配置](Navside-9.1/config/nav.yaml)、[接口说明](Navside-9.1/docs/navside_baseline.md) |
| NavSide 9.4 | 键盘 → 速度命令 | [使用说明](Navside-9.4/README.md) |
| Phybot 机器人端 | UDP 速度 → CPG / 状态机 → 电机；另含仿真入口 | [整理与部署说明](docs/robotside.md) |

## 数据流

```text
相机 A → cuVSLAM → 位姿 UDP 8082 ─┐
                                 ├→ NavSide → 速度 UDP 8080 → Phybot 机器人控制端
相机 B → 深度 → VAE/SRU ──────────┘

键盘 → Navside-9.4 → 速度 UDP 8080 → Phybot 机器人控制端
```

## 本次保留与排除

保留源码、配置、文档、cuVSLAM 原始许可、ONNX 模型，以及仿真 XML/URDF/OBJ/STL/纹理资源。`Navside-9.1/asset` 中的历史模型也保留，便于追溯原配置。

排除虚拟环境、构建产物、安装 wheel、压缩包、运行日志、SLAM 地图数据库、Rerun 录制、TensorRT 缓存、临时备份，以及指向仓库外部的软链接。详见 [.gitignore](.gitignore)。这些文件仍保留在原工程目录。

机器人端沿用其原 `.gitignore` 对 `ThirdParty` 的排除规则，并排除预编译 `.so` / `.a` 库和 LCM 工具可执行文件。机器人模型、ONNX 策略和构建脚本保留，具体依赖恢复说明见 [robotside.md](docs/robotside.md)。

## 在新机器上使用前

1. 按实际硬件安装 CUDA、相机 SDK 和 Python 依赖，重建 `cuvslam_migrate-9.1/venv` 与 NavSide 环境。原来的环境路径只是指向 `/home/amov/...` 的软链接，不能随源码迁移。
2. `cuvslam_migrate-9.1/run_dual_camera.sh` 中的 `NAVSIDE_DIR` 仍指向旧部署位置。按当前仓库布局可将该赋值改为 `NAVSIDE_DIR="$HERE/../Navside-9.1"`；同时确保启动 NavSide 的 `python3` 使用正确环境。本次归档没有改动运行脚本。
3. 使用实际设备序列号配置相机。需要重定位时，单独恢复自己的地图；仓库不包含历史地图和 `*_last_pose.txt`。
4. 真机运动需先部署机器人控制端。本仓库已包含 `PhybotSoftware_c2 arm/PhybotSoftware_c2` 源码，需按目标平台准备第三方库及硬件 SDK，再编译；见 [机器人端说明](docs/robotside.md)。
5. cuVSLAM 原目录已有 **53 个 Git LFS 指针占位文件**，主要是文档图片、GIF 和少量测试图片。它们不是图片实体；对应路径、对象标识和期望大小列在 [清单](docs/lfs-placeholders.txt)。如需正常显示这些图片或运行相关测试，应从与当前源码版本匹配、已完整下载 LFS 对象的上游检出中补齐。仅在这个新仓库执行 `git lfs pull` 不能取回从未上传过的对象。

原工程没有提供这几个部署模块的完整依赖锁定清单，因此本次整理不代表已验证可在新机器上直接启动。完整历史部署说明见 [deployment-original.md](docs/deployment-original.md)，其中的绝对路径、地图名及运行环境按原部署环境理解。

## 上游来源

cuVSLAM 原说明指向 [NVIDIA cuVSLAM](https://github.com/nvidia-isaac/cuVSLAM)，原始许可保留在 [LICENSE](cuVSLAM-main1/cuVSLAM-main/LICENSE)。本整理未替换各模块已有的版权和许可声明。

首次上传步骤见 [UPLOAD.md](UPLOAD.md)。
