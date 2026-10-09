cmake_minimum_required(VERSION 3.0)
add_definitions(-Wall -O2 -g)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)


get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

# 查找 yaml-cpp
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

#设定库的路径
find_library(MOTORDRIVE_LIB_PATH libMotorDrive.a ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

aux_source_directory(${PROJECT_SOURCE_DIR}/src dir_test)

# 添加可执行文件
add_executable(Test ${PROJECT_SOURCE_DIR}/Test/main.cpp  ${dir_test})


# 链接到可执行文件
include_directories(Test ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(Test ${MOTORDRIVE_LIB_PATH} ${YAML_CPP_LIBRARIES} pthread)

#将执行文件和动态库连接
target_link_libraries(Test)


