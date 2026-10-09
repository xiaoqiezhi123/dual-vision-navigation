# Phybot 机器人端

机器人端代码位于 `PhybotSoftware_c2 arm/PhybotSoftware_c2/`，保留原目录结构。进入目录时需要为带空格的路径加引号：

```bash
# 从仓库根目录执行
cd 'PhybotSoftware_c2 arm/PhybotSoftware_c2'
```

## 保留的内容

- `RL_deploy_cpg`：CPG 策略源码、配置和 ONNX 模型。
- `RobotStart`：真机与 MuJoCo 仿真入口。
- `StateMachine`、`MotorList`、`DataPackage`、`LowPassFilter`、`ZeroState`：控制与状态处理源码。
- `RobotModel`：机器人 XML/URDF、网格等资源。
- `RL_deploy_amp`、`RL_deploy_mimic`、`MujocoInterface`、`gazebo_sim`、`Joystick`、`Mocap`、`lcm_server`、`device`：原有源码、配置及未被排除的资源。
- `CMakeLists.txt`、`autobuild.sh`、`autoclean.sh`、`install.sh`、原 README 与 `.gitignore`。

复制过程中不修改控制逻辑，也不改写原构建脚本。

## 不放入普通 Git 的内容

- `build`、`build_*` 等构建目录。
- `ThirdParty`：机器人端原 `.gitignore` 已忽略此目录；本次保持该约定。其中包含体积较大的 TensorRT、CUDA、Pinocchio 库等依赖。
- 其他位置的预编译 `.so`、`.so.*`、`.a`、`.o`，以及 LCM `release/bin` 可执行文件。
- 压缩包、日志、缓存、编辑器状态和备份。

这些内容仍在原工程中。上传副本是源码与资源归档，不是完整的离线运行环境。

## 编译前恢复依赖

根 `CMakeLists.txt` 使用 CUDA，并引用 `ThirdParty` 中的头文件和库目录；真机分支还链接 `MotorDrive`、`yaml-cpp`、`nvinfer`、`nvonnxparser` 等库。由于库文件未上传，仅克隆仓库后直接执行构建脚本通常不足以完成编译。

在目标机器人或仿真机器上，需要安装或恢复与系统、CPU 架构和 CUDA 版本匹配的依赖，尤其是：

- CUDA / TensorRT。
- 原项目 `ThirdParty` 下对应的头文件与库，或经适配的系统安装位置。
- `MotorList/lib` 中的电机驱动库，以及所选运行模式需要的其他硬件 SDK 库。
- 仿真模式所需的 MuJoCo、图形库等依赖。

可从原部署环境恢复匹配的依赖目录；如果改用系统安装路径，需要同步调整 CMake 配置。原 `install.sh` 只包含少量系统包安装命令，并非完整依赖安装器。

依赖准备好后，再参考原部署说明选择构建入口。本次整理未运行编译、安装脚本或机器人程序，也未验证硬件运行。

## 与导航端的连接

主链路是 NavSide 通过 UDP 8080 发送 `vx, vy, omega_z`，由机器人端接收并执行；具体协议及原部署步骤见 [历史部署说明](deployment-original.md)。本次将机器人端加入同一仓库，没有改动通信参数。
