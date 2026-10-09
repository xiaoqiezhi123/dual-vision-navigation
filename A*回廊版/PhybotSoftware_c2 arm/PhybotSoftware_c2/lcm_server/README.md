sudo apt update
sudo apt install liblcm-dev
./lcm/release/bin/lcm-gen -x custom_msg.lcm
rm -rf CMakeCache.txt CMakeFiles/ cmake_install.cmake Makefile
cmake ./
make