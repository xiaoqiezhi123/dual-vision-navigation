cmake_minimum_required(VERSION 3.5)

project(lcm_std_cmake_demo LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ========== 手动指定 LCM 路径（关键） ==========
# 替换为你的 LCM 安装路径（Debug/Release 二选一）
set(LCM_INSTALL_PATH ${CMAKE_SOURCE_DIR}/lcm/release)

# LCM 头文件路径
include_directories(${LCM_INSTALL_PATH}/include)
# LCM 库路径
link_directories(${LCM_INSTALL_PATH}/lib)


set(HEADERS
    custom_lcm/ControlCmd.hpp
    custom_lcm/SensorData.hpp
    src/lcmcommunicator.h
)

set(SOURCES
    src/lcmcommunicator.cpp
    # 如果有 main.cpp，也在这里加上
    # src/main.cpp
)

# 创建可执行文件，并关联所有源文件和头文件
add_executable(${PROJECT_NAME}
    ${HEADERS}
    ${SOURCES}
    main.cpp
)


# ========== 链接依赖库 ==========
# 链接 LCM 库
target_link_libraries(lcm_std_cmake_demo lcm)

# 链接 glib2（LCM 依赖）
target_link_libraries(lcm_std_cmake_demo glib-2.0)

# 链接线程库（跨平台兼容）
find_package(Threads REQUIRED)
target_link_libraries(lcm_std_cmake_demo Threads::Threads)
