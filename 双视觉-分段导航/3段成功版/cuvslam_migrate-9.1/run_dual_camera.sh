#!/usr/bin/env bash
# 双相机一键启动：相机 A (cuVSLAM 被动 SLAM, 激光关) + 相机 B (NavSide 深度, 激光开)
#
# 用法（所有参数透传给 cuVSLAM 的 run_orbbec.sh）：
#   ./run_dual_camera.sh                                        # 默认 --mode localize（交互式选地图）
#   ./run_dual_camera.sh --mode localize --map 12 --no-viz      # 指定地图重定位，无 GUI
#   ./run_dual_camera.sh --mode map --map orbbec_map            # 建图
#
# 环境变量覆盖（可选）：
#   CUVSLAM_CAMERA_SERIAL=CPCxxxxxxxx  # 相机 A（SLAM，激光关）
#   NAVSIDE_CAMERA_SERIAL=CPCxxxxxxxx  # 相机 B（深度，激光开）
#   DUAL_SHOW_DEPTH=1                  # NavSide 加 --show-depth 打开深度图窗口
set -euo pipefail

# ===== 相机序列号（两台对称、仅 z 轴差、SRU 固定值 → A/B 可互换；要反过来就交换下面两行）=====
CAM_A="${CUVSLAM_CAMERA_SERIAL:-CPC8763000J0}"   # 相机 A：cuVSLAM SLAM，激光关
CAM_B="${NAVSIDE_CAMERA_SERIAL:-CPC8763000MZ}"   # 相机 B：NavSide 深度，激光开

HERE="$(cd "$(dirname "$0")" && pwd)"
NAVSIDE_DIR="$HOME/navside_real/NavSide_log/Navside-9.1"

echo "[dual] 相机 A (SLAM, 激光关) serial=$CAM_A"
echo "[dual] 相机 B (深度, 激光开) serial=$CAM_B"

# ---- 相机 A：cuVSLAM 被动双目惯性 SLAM，位姿经 UDP:8082 发给 NavSide ----
CUVSLAM_CAMERA_SERIAL="$CAM_A" "$HERE/run_orbbec.sh" "$@" &
SLAM_PID=$!

# ---- 相机 B：NavSide 原生深度(2.1.2, hole_filling_mode=2) + VAE/SRU ----
NAVSIDE_ARGS=(--real --config config/nav.yaml)
if [ "${DUAL_SHOW_DEPTH:-0}" = "1" ]; then
  NAVSIDE_ARGS+=(--show-depth)
fi
(
  cd "$NAVSIDE_DIR"
  NAVSIDE_CAMERA_SERIAL="$CAM_B" python3 scripts/run_vio.py "${NAVSIDE_ARGS[@]}"
) &
NAV_PID=$!

stop() {
  echo "[dual] 停止中... (SLAM=$SLAM_PID NavSide=$NAV_PID)"
  kill "$SLAM_PID" "$NAV_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap stop INT TERM

echo "[dual] 已启动。Ctrl+C 停止。"
wait
