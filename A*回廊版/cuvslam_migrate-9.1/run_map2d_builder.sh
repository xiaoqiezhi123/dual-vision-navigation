#!/usr/bin/env bash
# 二维规划地图制作入口（V1）—— 不启动相机、不启动 VIO。
# 用法示例：
#   ./run_map2d_builder.sh \
#     --map-dir ./orbbec/8888 \
#     --extractor ../cuVSLAM-main1/cuVSLAM-main/build_jetson/bin/map_extractor \
#     --output ./maps2d/8888/v1 \
#     --resolution 0.05 \
#     --robot-radius-m 0.35 \
#     --safety-margin-m 0.10
# 机器人在场实测包络半径与安全余量后再填；未填地面高度时不自动识别障碍。
HERE="$(cd "$(dirname "$0")" && pwd)"
SP_DIR="$(echo "$HERE"/venv/lib/python*/site-packages)"
export LD_LIBRARY_PATH="$SP_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$HERE/venv/bin/python" "$HERE/orbbec/map2d_builder.py" "$@"
