#!/usr/bin/env bash
set -euo pipefail

TTY_DEVICE="${TTY_DEVICE:-/dev/ttyimu}"
CUDA_TOOLKIT_ROOT_DIR="${CUDA_TOOLKIT_ROOT_DIR:-/usr/local/cuda}"

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_PATH1="$BASE_DIR/ThirdParty/urdfdom/lib"
LIB_PATH2="$BASE_DIR/ThirdParty/boost/lib"
export LD_LIBRARY_PATH="$LIB_PATH1:$LIB_PATH2:${LD_LIBRARY_PATH:-}"
echo "Using LD_LIBRARY_PATH=$LD_LIBRARY_PATH"

echo "==================PHYBOT SOFTWARE=================="
echo "1.realrobot_mini"
echo "2.webots_sim"
echo "3.mujoco_sim_mini"
echo "4.gazebo_sim (only for catkin build   !!!!)"

read -p "enter control system number:" control_system_enter
case "$control_system_enter" in
    1)
        control_system_enter=realrobot_mini
        ;;
    2)
        control_system_enter=webots_sim
        ;;
    3)
        control_system_enter=mujoco_sim_mini
        ;;
    4)
        control_system_enter=gazebo_sim
        ;;
    *)
        echo "!invalid enter!"
        exit 1
        ;;
esac

if [ "$control_system_enter" = "realrobot_mini" ]; then
    if [[ -e "$TTY_DEVICE" ]]; then
        sudo chmod 777 "$TTY_DEVICE"
    else
        echo "Warning: $TTY_DEVICE not found; realrobot runtime may need a serial device."
    fi
fi

root=$(pwd)
build_dir="$root/build"
mkdir -p "$build_dir"
echo "$build_dir"

if [ "$control_system_enter" = "mujoco_sim_mini" ]; then
    cmake -S "$root" -B "$build_dir" -DWHICH_ENV="$control_system_enter" -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR="$CUDA_TOOLKIT_ROOT_DIR"
    cmake --build "$build_dir" -- -j8
else
    pushd "$build_dir" >/dev/null
    cmake -DWHICH_ENV="${control_system_enter}" "$root" -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR="$CUDA_TOOLKIT_ROOT_DIR" .
    make -j8
    popd >/dev/null
fi
