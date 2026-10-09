#!/bin/bash
export LD_LIBRARY_PATH=./lcm/release/lib:$LD_LIBRARY_PATH
# 后台运行，输出日志到lcm.log
#nohup ./lcm_demo > lcm.log 2>&1 &
./lcm_std_cmake_demo
echo "程序启动"
