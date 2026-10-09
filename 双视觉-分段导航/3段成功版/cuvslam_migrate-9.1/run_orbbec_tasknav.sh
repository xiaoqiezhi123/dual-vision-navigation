#!/usr/bin/env bash
# 任务导航启动脚本 —— 用隔离 venv 的 Python 跑 run_vio_tasknav.py，不碰系统环境
# 用法：
#   ./run_orbbec_tasknav.sh                     # 默认参考地图 orbbec_map（或用环境变量 CUVSLAM_REF_MAP）
#   ./run_orbbec_tasknav.sh --ref-map 12        # 指定参考地图（任务点锚定目标，只读）
#   ./run_orbbec_tasknav.sh --ref-map 12 --no-viz
# 终端指令（机器人到达任务点静止后输入）：
#   localize / l   用参考地图锚定全局位姿（作为下一段子任务起点）
#   help / h       指令帮助
#   quit / q       优雅退出（不保存地图）
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE/orbbec"
# pyorbbecsdk 的 .so 带无效 RUNPATH，靠 LD_LIBRARY_PATH 定位 venv 里的 libOrbbecSDK.so.2
SP_DIR="$(echo "$HERE"/venv/lib/python*/site-packages)"
export LD_LIBRARY_PATH="$SP_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# Rerun Viewer 需要显示器。SSH 无 X 转发时 DISPLAY 为空，这里指到本机物理桌面 :0（GDM 会话）。
# 若用 `ssh -X` 连进来（DISPLAY 已有值，如 localhost:10.0），则保留，viewer 弹到本地笔记本。
if [ -z "${DISPLAY:-}" ]; then
  export DISPLAY=:0
  if [ -f "/run/user/$(id -u)/gdm/Xauthority" ]; then
    export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
  fi
fi
exec "$HERE/venv/bin/python" run_vio_tasknav.py "$@"
