cmake_minimum_required(VERSION 3.1.3)
set(CMAKE_CXX_STANDARD 14)
project(Mocap)
message(STATUS "Project source directory: ${PROJECT_SOURCE_DIR}/..")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(${PROJECT_SOURCE_DIR}/..)
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/yaml/include)
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty)
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/eigen3)
include_directories(${PROJECT_SOURCE_DIR}/../device/Imu_hipnuc/lib)
include_directories(${PROJECT_SOURCE_DIR}/../device/Imu_hipnuc/linux)
include_directories(${PROJECT_SOURCE_DIR}/../device/Imu_hipnuc/linux/fw_downloader)

link_directories(${PROJECT_SOURCE_DIR}/lib)
link_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/yaml/lib)

set(CMAKE_BUILD_RPATH "$ORIGIN/../lib")

add_executable(mocap_test ${PROJECT_SOURCE_DIR}/test/main.cpp
    ${PROJECT_SOURCE_DIR}/src/mocap_streamer.cpp)

target_link_libraries(mocap_test LuMoSDK pthread yaml-cpp)

