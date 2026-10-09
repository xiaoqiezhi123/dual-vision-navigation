cmake_minimum_required(VERSION 3.12)  # 更新最小 CMake 版本
# cmake_policy(SET CMP0148 NEW)

set(CMAKE_WARN_DEVELOPER OFF)
set(CMAKE_BUILD_TYPE "Release")


add_compile_options(-O3  -ffast-math )

# Set the C standard to C99
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
# add_compile_options(-std=c++17)

project(phybot_software)
message(STATUS "Project source directory: ${PROJECT_SOURCE_DIR}/..")
add_compile_options(-Wno-return-type)

find_package(CUDA REQUIRED)

# # catkin 包信息
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs
# )

# set(THREADS_PREFER_PTHREAD_FLAG ON)
# find_package(Threads REQUIRED)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# link_directories("/usr/local/lib")
# INCLUDE_DIRECTORIES("/usr/local/include")
# link_directories("/usr/lib")
# INCLUDE_DIRECTORIES("/usr/include")
# include_directories("/usr/include/GL")
# find_package(OpenGL REQUIRED)
# include_directories(${OPENGL_INCLUDE_DIR})

# find_package(yaml-cpp REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/MujocoInterface/include/simulate)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/eigen3)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/boost/include)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/spdlog/include)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(/usr/local/cuda/include)
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/TensorRT/include)

include_directories(${PROJECT_SOURCE_DIR}/Joystick/include)
include_directories(${PROJECT_SOURCE_DIR}/MotorList/include)
include_directories(${PROJECT_SOURCE_DIR}/MotorList)
link_directories(${PROJECT_SOURCE_DIR}/ThirdParty/boost/lib)
link_directories(${PROJECT_SOURCE_DIR}/ThirdParty/TensorRT/lib)
link_directories(/usr/local/cuda/lib64)

link_directories(${PROJECT_SOURCE_DIR}/MotorList/lib)

set(TENSORRT_CUDA_RUNTIME cudart)
set(TENSORRT_INFER_LIBRARY nvinfer)
set(TENSORRT_ONNX_PARSER_LIBRARY nvonnxparser)


aux_source_directory(${PROJECT_SOURCE_DIR}/DataPackage/src dir_datapackage)
# add_library(DataPackage SHARED ${dir_datapackage})

# target_link_libraries(DataPackage pinocchio yaml-cpp)

if(WHICH_ENV AND(WHICH_ENV STREQUAL "realrobot_mini"))
    add_compile_definitions(DISABLE_RL_MIMIC_FOR_REALROBOT)
    add_compile_definitions(DISABLE_MOCAP_FOR_REALROBOT)


    include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/yaml/include)
    include_directories(${PROJECT_SOURCE_DIR}/MujocoInterface/include)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib)
    include_directories(${PROJECT_SOURCE_DIR}/RL_deploy_cpg/include)


    include_directories(${PROJECT_SOURCE_DIR}/DataPackage/include)
    include_directories(${PROJECT_SOURCE_DIR}/StateMachine/include)
    include_directories(${PROJECT_SOURCE_DIR}/MotorList/include)
    include_directories(${PROJECT_SOURCE_DIR}/MotorList)
    include_directories(${PROJECT_SOURCE_DIR}/LowPassFilter/include)
    include_directories(${PROJECT_SOURCE_DIR}/ZeroState/include)

    aux_source_directory(${PROJECT_SOURCE_DIR}/Joystick/src dir_js)
    aux_source_directory(${PROJECT_SOURCE_DIR}/StateMachine/src dir_FSM)
    aux_source_directory(${PROJECT_SOURCE_DIR}/MotorList/src dir_ML)
    aux_source_directory(${PROJECT_SOURCE_DIR}/LowPassFilter/src dir_LPF)
    aux_source_directory(${PROJECT_SOURCE_DIR}/ZeroState/src dir_ZSM)
    aux_source_directory(${PROJECT_SOURCE_DIR}/RL_deploy_cpg/src dir_rl_cpg)

    list(REMOVE_ITEM dir_FSM
        ${PROJECT_SOURCE_DIR}/StateMachine/src/RL_mimic.cpp
    )


    link_directories(${PROJECT_SOURCE_DIR}/ThirdParty/yaml/lib)
    link_directories(${PROJECT_SOURCE_DIR}/MotorList/lib)

    set(IMU_SOURCES
    # test.cpp
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/HipnucReader.cpp
    # main.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/serial_port.c  # Change this to C++ source if serial_port.c actually contains C++ code
    # commands.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/log.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader/hex2bin.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader/kboot.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/example_data.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/hipnuc_dec.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/nmea_dec.c
    )

    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib)



    add_executable(main ${PROJECT_SOURCE_DIR}/RobotStart/realrobot/run_mini.cpp

                        ${IMU_SOURCES}
                        ${dir_js}
                        ${dir_FSM}
                        ${dir_datapackage}
                        ${dir_ML}
                        ${dir_LPF}
                        ${dir_ZSM}
                        ${dir_rl_cpg}
                        )
    target_link_libraries(main  PUBLIC 
                            pthread
                            yaml-cpp
                            MotorDrive
                            ${TENSORRT_CUDA_RUNTIME}
                            ${TENSORRT_INFER_LIBRARY}
                            ${TENSORRT_ONNX_PARSER_LIBRARY}
                            # HARDWARE_TOP
                            # Main_Control_Board_Device
                            )

    add_executable(set_zero ${PROJECT_SOURCE_DIR}/RobotStart/realrobot/set_zero_mini.cpp

                            # ${IMU_SOURCES}
                            ${dir_LPF}

                            ${dir_datapackage}
                            ${dir_ML}
                            ${dir_ZSM}
                            )
    target_link_libraries(set_zero  PUBLIC 
                                pthread
                                yaml-cpp
                                MotorDrive
                                # HARDWARE_TOP
                                # Main_Control_Board_Device
                                )
    
    add_executable(scan_motors ${PROJECT_SOURCE_DIR}/RobotStart/realrobot/scan_motors.cpp

                            ${dir_LPF}

                            ${dir_datapackage}
                            ${dir_ML}
                            ${dir_ZSM}
                            )
    target_link_libraries(scan_motors  PUBLIC
                                pthread
                                yaml-cpp
                                MotorDrive
                                )
    





elseif(WHICH_ENV AND(WHICH_ENV STREQUAL "mujoco_sim_mini"))
    include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/mujoco)
    include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/glfw)
    include_directories(${PROJECT_SOURCE_DIR}/MujocoInterface/include)
    include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/gl/include)


    # include_directories(${PROJECT_SOURCE_DIR}/DataLogger/include)
    include_directories(${PROJECT_SOURCE_DIR}/DataPackage/include)
    include_directories(${PROJECT_SOURCE_DIR}/StateMachine/include)
    include_directories(${PROJECT_SOURCE_DIR}/LowPassFilter/include)
    include_directories(${PROJECT_SOURCE_DIR}/ZeroState/include)
    include_directories(${PROJECT_SOURCE_DIR}/RL_deploy_amp/include)
    include_directories(${PROJECT_SOURCE_DIR}/RL_deploy_cpg/include)

    add_compile_definitions(DISABLE_RL_MIMIC_FOR_MUJOCO)


    aux_source_directory(${PROJECT_SOURCE_DIR}/MujocoInterface/src dir_mujoco)
    aux_source_directory(${PROJECT_SOURCE_DIR}/Joystick/src dir_js)
    aux_source_directory(${PROJECT_SOURCE_DIR}/StateMachine/src dir_FSM)
    aux_source_directory(${PROJECT_SOURCE_DIR}/LowPassFilter/src dir_LPF)
    aux_source_directory(${PROJECT_SOURCE_DIR}/ZeroState/src dir_ZSM)
    aux_source_directory(${PROJECT_SOURCE_DIR}/RL_deploy_cpg/src dir_rl_cpg)

    list(REMOVE_ITEM dir_FSM
        ${PROJECT_SOURCE_DIR}/StateMachine/src/RL_mimic.cpp
    )


    
    # message(STATUS "joystick: ${dir_js}")

    link_directories(${PROJECT_SOURCE_DIR}/ThirdParty/mujoco/lib)

    set(IMU_SOURCES
    # test.cpp
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/HipnucReader.cpp
    # main.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/serial_port.c  # Change this to C++ source if serial_port.c actually contains C++ code
    # commands.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/log.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader/hex2bin.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader/kboot.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/example_data.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/hipnuc_dec.c
    ${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib/nmea_dec.c
    )

    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/linux/fw_downloader)
    include_directories(${PROJECT_SOURCE_DIR}/device/Imu_hipnuc/lib)

    add_executable(main ${PROJECT_SOURCE_DIR}/RobotStart/mujoco_sim/main.cc
                        
                        ${dir_mujoco}
                        ${IMU_SOURCES}
                        ${dir_js}
                        ${dir_FSM}
                        ${dir_datapackage}
                        ${dir_LPF}
                        ${dir_ZSM}
                        ${dir_rl_cpg}
                        )
    target_link_libraries(main  PUBLIC 
                            ${TORCH_LIBRARIES}
                            mujoco
                            glfw 
                            # Threads::Threads
                            pthread

                            yaml-cpp
                            ${TENSORRT_CUDA_RUNTIME}
                            ${TENSORRT_INFER_LIBRARY}
                            ${TENSORRT_ONNX_PARSER_LIBRARY}
                            # tinysplinecxx 

                            # torch
                            )

                            # target_link_libraries(${PROJECT} "${TORCH_LIBRARIES}")
                            # set_property(TARGET ${PROJECT} PROPERTY CXX_STANDARD 17)
    
elseif(WHICH_ENV AND(WHICH_ENV STREQUAL "gazebo_sim"))
    set(CMAKE_PREFIX_PATH "/opt/ros/noetic" ${PROJECT_SOURCE_DIR}/ThirdParty/libtorch/share/cmake/Torch)
    find_package(Torch REQUIRED)
    include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/libtorch/include)

    ## 查找 ROS 依赖
    # set(phybot_msgs_DIR /home/kairan/Documents/phybot_ws/src/phybot_msgs)
    set(phybot_msgs_DIR "${PROJECT_SOURCE_DIR}/../../devel/share/phybot_msgs/cmake")
    add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/gazebo_sim/")

    find_package(catkin REQUIRED COMPONENTS
        phybot_msgs
        roscpp
        std_msgs
        tf
        geometry_msgs
        rospy
        gazebo_ros
        controller_manager
        joint_state_controller
        robot_state_publisher
        genmsg
        

    )
    
    catkin_package(
        CATKIN_DEPENDS roscpp std_msgs tf geometry_msgs
    )
    
    include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/gazebo_sim/include
        ${PROJECT_SOURCE_DIR}/gazebo_sim/library/loop
        ${PROJECT_SOURCE_DIR}/gazebo_sim/library/rl_sdk

    )

    include_directories(${PROJECT_SOURCE_DIR}/gazebo_sim/include)

    add_library(rl_sdk ${PROJECT_SOURCE_DIR}/gazebo_sim/library/rl_sdk/rl_sdk.cpp)
    target_link_libraries(rl_sdk "${TORCH_LIBRARIES}" yaml-cpp)
    set_property(TARGET rl_sdk PROPERTY CXX_STANDARD 17)


    add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/gazebo_sim/src/rl_sim.cpp
                        # ${PROJECT_SOURCE_DIR}/gazebo_sim/library/rl_sdk/rl_sdk.cpp
                        # ${dir_wbc}
                        # ${dir_est}
                        # ${dir_datapackage}
                        ) 
                           

    target_link_libraries(${PROJECT_NAME}
                                # pinocchio_default 
                                # pinocchio_parsers
                                yaml-cpp
                                torch
                                rl_sdk
                                # qpOASES 
                                # urdfdom_model
                                ${catkin_LIBRARIES})



endif()





# install(TARGETS DataPackage DESTINATION ${PROJECT_SOURCE_DIR}/lib)
