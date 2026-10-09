"""枚举当前连接的 Orbbec 相机，打印每台序列号，供双相机方案填 SN。

双相机方案：
  - 相机 A（SLAM，激光关）：run_vio.py 的 CAMERA_A_SERIAL 或环境变量 CUVSLAM_CAMERA_SERIAL
  - 相机 B（深度，激光开）：NavSide 环境变量 NAVSIDE_CAMERA_SERIAL

用法（用任一 SDK 的 venv 跑，二选一）：
  # cuVSLAM venv (2.0.10)
  LD_LIBRARY_PATH=../cuvslam_migrate/venv/lib/python3.10/site-packages \
      ../cuvslam_migrate/venv/bin/python enumerate_devices.py
  # NavSide venv (2.1.2)
  /home/amov/nav_arm_mujoco/.venv_navside/bin/python enumerate_devices.py
"""

import pyorbbecsdk as ob


def main() -> None:
    ctx = ob.Context()
    devices = ctx.query_devices()
    n = devices.get_count()
    print(f"已连接 {n} 台 Orbbec 设备")
    if n == 0:
        print("没有检测到相机，请检查 USB 连接 / 权限后重试。")
        return
    for i in range(n):
        serial = devices.get_device_serial_number_by_index(i)
        name = "?"
        try:
            name = devices.get_device_by_index(i).get_device_info().get_name()
        except Exception:
            pass
        print(f"  [{i}] name={name}  serial={serial}")
    print()
    print("把 SLAM 相机(相机 A)的 serial 填入 run_vio.py 的 CAMERA_A_SERIAL，")
    print("或 export CUVSLAM_CAMERA_SERIAL=<SN>。")
    print("把深度相机(相机 B)的 serial 用环境变量 NAVSIDE_CAMERA_SERIAL=<SN> 传给 NavSide。")


if __name__ == "__main__":
    main()
