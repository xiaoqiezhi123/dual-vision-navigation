#!/usr/bin/env bash
# cuVSLAM 启动脚本 —— 用隔离 venv 的 Python 跑，不碰系统环境
# 用法：
#   ./run_orbbec.sh                        # 默认 --mode localize（重定位）
#   ./run_orbbec.sh --mode map             # 建图
#   ./run_orbbec.sh --mode map --map 名称   # 建图并指定地图名
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
if [ $# -eq 0 ]; then
  exec "$HERE/venv/bin/python" run_vio.py --mode localize
else
  exec "$HERE/venv/bin/python" run_vio.py "$@"
fi
