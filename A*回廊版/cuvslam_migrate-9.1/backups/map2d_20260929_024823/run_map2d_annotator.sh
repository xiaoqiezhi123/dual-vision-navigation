#!/usr/bin/env bash
# 二维规划地图人工标注工具入口（需要显示器；不接相机）
# 用法示例：
#   ./run_map2d_annotator.sh \
#     --source-json ./maps2d/viz_test_02/v2/source_map.json \
#     --ground-y 0.0 --height-min 0.5 --height-max 5.0 \
#     --out ./annotations.json
HERE="$(cd "$(dirname "$0")" && pwd)"
exec "$HERE/venv/bin/python" "$HERE/orbbec/annotate_map2d.py" "$@"
