#ifndef SERVER_H_
#define SERVER_H_
#include "DataPackage/include/DataPackage.h"
#include "lcm_server/src/lcmcommunicator.h"
#include <iostream>
#include <chrono>
#include <csignal>
#include <random>
#include <thread>
#include <mutex>
#include <atomic>
#include "lcm_server/custom_lcm/system2Algorithm_packet.hpp"

// 定义帧类型常量
#define FRAME_TYPE_APP_STARTUP      0xD001  // 应用启动消息
#define FRAME_TYPE_APP_STARTUP_REP  0xE001  // 应用启动回报
#define FRAME_TYPE_ROBOT_CONTROL    0x0001  // 机器人控制指令
#define FRAME_TYPE_ROBOT_CONTROL_REP 0x0002 // 机器人控制回报

struct Base_DriveCmd {
    // 3. 模式（1字节）：
    //              0 ： 初始状态
    //              1 ： 零位
    //              2 ： 走路
    //              3 ： 挥手
    //              4 ： 出拳舞
    //              5 ： 出拳
    //              6 ： 踢腿
    //              7 ： 爬起
    //              8 ： 趴下
    //              9 ： 太极
    //              10： 功夫 
    int8_t model;    

    // 4. x速度（4字节）：（-0.9～0.9）
    float x_velocity; 

    // 5. y速度（4字节）：（-1.0～1.0）
    float y_velocity;  

    // 6. 角速度（4字节）：（-0.6～0.6）
    float angular_velocity;  
};

struct ApplicationStartupMessage
{
    //! \brief      当前机型支持的动作列表最大支持32个动作     bit占位
    //!             D0: 归零      1-支持该动作      0-不支持该动作
    //!             D1: 走路      1-支持该动作      0-不支持该动作
    //!             D2: 挥手      1-支持该动作      0-不支持该动作
    //!             D3: 出拳舞     1-支持该动作      0-不支持该动作
    //!             D4: 出拳      1-支持该动作      0-不支持该动作
    //!             D5: 踢腿      1-支持该动作      0-不支持该动作
    //!             D6: 爬起      1-支持该动作      0-不支持该动作
    //!             D7: 趴下      1-支持该动作      0-不支持该动作
    //!             D8: 太极      1-支持该动作      0-不支持该动作
    //!             D9: 功夫      1-支持该动作      0-不支持该动作
    //!             D10~D31 预留
    int32_t action_list ;

    //! \brief      线速度上限
    float vx_max ;

    //! \brief      线速度下限
    float vx_min;

    //! \brief      线速度上限
    float vy_max;

    //! \brief      线速度下限
    float vy_min ;

    //! \brief      角速度上限
    float omegaz_max ;

    //! \brief      角速度下限
    float omegaz_min ;
};
struct ApplicationStartupMessage_rep
{
    //! \brief      监听算法程序成功        0x01-成功，其他则失败
    int8_t listen_successful;
};

//! \brief      控制消息接收回报        周期(500Hz)
//!         算法->后端
struct Base_DriveCmd_rep
{
    //! \brief  当前执行动作
    int16_t action_cmd ;
    //! \brief  0x00-无效 0x01-执行成功 0x02-正在执行 0x03-执行失败
    int8_t action_Result;
};


class Server {
  public:
    Server();
    ~Server();
    void init();
    void SetDataToPackage(DataPackage &DataPackage);
    void run();
    void GetDataFromPackage(DataPackage &DataPackage);
    void Program_begin();
    
  private:
    void pack_ApplicationStartupMessage(const ApplicationStartupMessage &app_msg, 
                                       custom_lcm::system2Algorithm_packet &packet);
    void unpack_ApplicationStartupMessage_rep(const custom_lcm::system2Algorithm_packet &packet, 
                                             ApplicationStartupMessage_rep &app_rep_msg);
    void unpack_Base_DriveCmd(const custom_lcm::system2Algorithm_packet &packet, 
                             Base_DriveCmd &drive_cmd);
    void pack_Base_DriveCmd_rep(const Base_DriveCmd_rep &drive_rep_cmd, 
                               custom_lcm::system2Algorithm_packet &packet);


    std::atomic<bool> g_running{true};
    int8_t calculate_checksum(const custom_lcm::system2Algorithm_packet &packet);
    void get_cmd_vel(const std::string& channel, const unsigned char* data, long unsigned int size);
    void Pub_ApplicationStartupMessage();
    void cmd_vel_run();
    void Pub_Base_DriveCmd_rep();
    void Heartbeat();
    std::thread base_thread;
    std::thread heartbeat;
    std::mutex data_mutex;
    PHYBOT_TOOL::LcmCommunicator lcm;
    State NextState{State::ZERO};

    double js_vx_desire{0};
    double js_vy_desire{0};
    double js_OmegaZ_desire{0};

    double neck_yaw_xx{0};
    double neck_pitch_yy{0};
    int control_mode{0};
    int mimic_mode{1};
    int model{0};

    int32_t action_list{63};
    float vx_max{1.5};
    float vx_min{-1.4};
    float vy_max{0.4};
    float vy_min{-0.4};
    float omegaz_max{0.9};
    float omegaz_min{-0.9};
    bool ApplicationStartupMessage_rep_flag{false};

    /* 控制消息回报 */
    Base_DriveCmd_rep drive_rep_cmd;

};

#endif
