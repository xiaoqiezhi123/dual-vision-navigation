#!/usr/bin/env bash
# 2D 地图选点工具启动脚本 —— 用隔离 venv 的 Python 跑 map2d_picker.py
# 用法：
#   ./run_map2d_picker.sh                     # 读 orbbec/vio_mapping_tracking.rrd
#   ./run_map2d_picker.sh --rrd 其它.rrd      # 指定 .rrd
#   ./run_map2d_picker.sh --y 0.0             # 选点高度固定
#   ./run_map2d_picker.sh --save 任务点.txt    # 结果追加写入文件
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE/orbbec"
# pyorbbecsdk 的 .so 带无效 RUNPATH，靠 LD_LIBRARY_PATH 定位 venv 里的 libOrbbecSDK.so.2
SP_DIR="$(echo "$HERE"/venv/lib/python*/site-packages)"
export LD_LIBRARY_PATH="$SP_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# 选点窗口需要显示器。SSH 无 X 转发时 DISPLAY 为空，这里指到本机物理桌面 :0（GDM 会话）。
if [ -z "${DISPLAY:-}" ]; then
  export DISPLAY=:0
  if [ -f "/run/user/$(id -u)/gdm/Xauthority" ]; then
    export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
  fi
fi
exec "$HERE/venv/bin/python" map2d_picker.py "$@"
