#include "lcm_server/include/server.h"
#include <iomanip> // 必须包含这个头文件以使用 std::hex


Server::Server() : lcm(7667, 0x01, "wlp0s20f3") {
    // std::string file_path = "../lcm_server/config/lcm_server.yaml";
    // YAML::Node config = YAML::LoadFile(file_path);
    // action_list = config["action_list"].as<int32_t>();
    // vx_max = config["vx_max"].as<float>();
    // vx_min = config["vx_min"].as<float>();
    // vy_max = config["vy_max"].as<float>();
    // vy_min = config["vy_min"].as<float>();
    // omegaz_max = config["omegaz_max"].as<float>();
    // omegaz_min = config["omegaz_min"].as<float>();
}


Server::~Server() {
    g_running = false;
    if (base_thread.joinable()) {
        base_thread.join(); // 等待线程退出
    }
    if (heartbeat.joinable()) {
        heartbeat.join(); // 等待线程退出
    }
}


// 计算system2Algorithm_packet的校验和
int8_t Server::calculate_checksum(const custom_lcm::system2Algorithm_packet &packet) {
    const uint8_t* packet_bytes = reinterpret_cast<const uint8_t*>(&packet);
    int32_t sum = 0;
    
    // 校验范围：下标2~63字节求和取模255
    for (int i = 2; i < 63; ++i) {
        sum += packet_bytes[i];
    }
    
    return static_cast<int8_t>(sum % 255);
}


// 打包ApplicationStartupMessage到system2Algorithm_packet
void Server::pack_ApplicationStartupMessage(const ApplicationStartupMessage &app_msg, 
                                           custom_lcm::system2Algorithm_packet &packet) {
    // 1. 设置固定头帧
    packet.header = 0xeb90;
    // 2. 设置帧类型
    packet.frame_type = FRAME_TYPE_APP_STARTUP;
    // 3. 初始化data数组为0
    memset(packet.data, 0, sizeof(packet.data));
    // 4. 打包数据到data数组
    size_t offset = 0;
    memcpy(packet.data + offset, &app_msg.action_list, sizeof(app_msg.action_list));
    offset += sizeof(app_msg.action_list);
    memcpy(packet.data + offset, &app_msg.vx_max, sizeof(app_msg.vx_max));
    offset += sizeof(app_msg.vx_max);
    memcpy(packet.data + offset, &app_msg.vx_min, sizeof(app_msg.vx_min));
    offset += sizeof(app_msg.vx_min);
    memcpy(packet.data + offset, &app_msg.vy_max, sizeof(app_msg.vy_max));
    offset += sizeof(app_msg.vy_max);
    memcpy(packet.data + offset, &app_msg.vy_min, sizeof(app_msg.vy_min));
    offset += sizeof(app_msg.vy_min);
    memcpy(packet.data + offset, &app_msg.omegaz_max, sizeof(app_msg.omegaz_max));
    offset += sizeof(app_msg.omegaz_max);
    memcpy(packet.data + offset, &app_msg.omegaz_min, sizeof(app_msg.omegaz_min));
    // 5. 计算并设置校验和
    packet.checksum = calculate_checksum(packet);
}


// 打包Base_DriveCmd_rep到system2Algorithm_packet
void Server::pack_Base_DriveCmd_rep(const Base_DriveCmd_rep &drive_rep_cmd, 
                                   custom_lcm::system2Algorithm_packet &packet) {
    packet.header = 0xeb90;
    packet.frame_type = FRAME_TYPE_ROBOT_CONTROL_REP;
    memset(packet.data, 0, sizeof(packet.data));
    
    size_t offset = 0;
    memcpy(packet.data + offset, &drive_rep_cmd.action_cmd, sizeof(drive_rep_cmd.action_cmd));
    offset += sizeof(drive_rep_cmd.action_cmd);
    memcpy(packet.data + offset, &drive_rep_cmd.action_Result, sizeof(drive_rep_cmd.action_Result));
    
    packet.checksum = calculate_checksum(packet);
}


void Server::unpack_Base_DriveCmd(const custom_lcm::system2Algorithm_packet &packet, 
                                 Base_DriveCmd &drive_cmd) {
    size_t offset = 0;
    memcpy(&drive_cmd.model, packet.data + offset, sizeof(drive_cmd.model));
    offset += sizeof(drive_cmd.model);
    memcpy(&drive_cmd.x_velocity, packet.data + offset, sizeof(drive_cmd.x_velocity));
    offset += sizeof(drive_cmd.x_velocity);
    memcpy(&drive_cmd.y_velocity, packet.data + offset, sizeof(drive_cmd.y_velocity));
    offset += sizeof(drive_cmd.y_velocity);
    memcpy(&drive_cmd.angular_velocity, packet.data + offset, sizeof(drive_cmd.angular_velocity));
}


void Server::unpack_ApplicationStartupMessage_rep(const custom_lcm::system2Algorithm_packet &packet, 
                                                 ApplicationStartupMessage_rep &app_rep_msg) {
    size_t offset = 0;
    memcpy(&app_rep_msg.listen_successful, packet.data + offset, sizeof(app_rep_msg.listen_successful));
}

void Server::init() {
    
    drive_rep_cmd.action_cmd = (int16_t)State::IDLE;
    drive_rep_cmd.action_Result = (int8_t)ControlMsg_Rep::No_Runing;


    // 设置适配后的回调函数
    lcm.set_global_callback([this](const std::string& channel, const unsigned char* data, long unsigned int size) {
        this->get_cmd_vel(channel, data, size);
    });

    lcm.subscribe("Robot_ControlData");
    lcm.subscribe("Application_Startup_Message_rep");
    base_thread = std::thread(&Server::cmd_vel_run, this);
}

void Server::cmd_vel_run(){
    std::cout << "[INFO] 已订阅主题，等待消息..." << std::endl;
    while (g_running) {
        if (model == 1) {
            NextState = State::ZERO;
        } 
        else if (model == 2) {
            NextState = State::RL_walk;
        } 
        else if (model == 3) { 
            NextState = State::RL_mimic;
            control_mode = 1;
        } 
        else if (model == 4) {
            NextState = State::RL_long_motion;
            control_mode = 0;
        } 
        else if (model == 5) {
            NextState = State::RL_forward_punch;
            mimic_mode = 0;
        } 
        else if (model == 6) {
            NextState = State::RL_forward_kick;
            mimic_mode = 1;
        } 
        else if (model == 7) {
            NextState = State::RL_climb_up;
        } 
        else if (model == 8) {
            NextState = State::RL_crane_down;
        }
        else if (model == 9) {
            NextState = State::RL_taichi;
        } 
        else if (model == 10) {
            NextState = State::RL_kongfu;
        }
        // std::cout<<int32_to_binary(action_list)<<std::endl;
       
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}


void Server::Program_begin(){
    heartbeat = std::thread(&Server::Heartbeat, this);
}


void Server::Heartbeat() {
    // 循环发送心跳包，直到程序退出
    while (g_running) {
        Pub_Base_DriveCmd_rep();
        if (!ApplicationStartupMessage_rep_flag) {
            Pub_Base_DriveCmd_rep();//Pub_ApplicationStartupMessage();
        } else {
            Pub_Base_DriveCmd_rep();
        }
        std::cout << "[INFO] 心跳包已发送！" << std::endl;
        // 正确的1秒休眠（每1秒发送一次心跳包）
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "[INFO] 心跳包线程已退出！" << std::endl;
}


void Server::SetDataToPackage(DataPackage &DataPackage){
    DataPackage.NextState = NextState;
    DataPackage.js_vx_desire = js_vx_desire;
    DataPackage.js_OmegaZ_desire = js_OmegaZ_desire;
    DataPackage.js_vy_desire = js_vy_desire;
    DataPackage.control_mode = control_mode;
    DataPackage.mimic_mode = mimic_mode;
    // DataPackage.neck_yaw_xx=neck_yaw_xx;
    // DataPackage.neck_pitch_yy=neck_pitch_yy;
}

/* 获取控制消息 */
void Server::GetDataFromPackage(DataPackage &DataPackage){
    
    drive_rep_cmd.action_cmd = DataPackage.action_cmd;
    drive_rep_cmd.action_Result = DataPackage.action_Result;

}


// 新版 LCM 回调函数（适配 GlobalLcmCallback 类型）
void Server::get_cmd_vel(const std::string& channel, const unsigned char* data, long unsigned int size) {

    custom_lcm::system2Algorithm_packet packet;
    if (!packet.decode(data, 0, size)) {
        std::cerr << "[ERROR] system2Algorithm_packet 消息解码失败！" << std::endl;
        return;
    }
    const uint16_t expected_header = 0xeb90;
    uint16_t received_header = static_cast<uint16_t>(packet.header);
    if (received_header != expected_header) {
        std::cerr << "[ERROR] 消息头帧错误！接收值=0x" << std::hex << static_cast<uint32_t>(received_header) 
                << "，期望值=0x" << static_cast<uint32_t>(expected_header) << std::dec << std::endl;
        return;
    }
    // 3. 验证校验和
    int8_t calculated_checksum = calculate_checksum(packet);
    if (calculated_checksum != packet.checksum) {
        std::cerr << "[ERROR] 校验和验证失败！" << std::endl;
        std::cerr << "  → 接收的校验和：0x" << std::hex << static_cast<int>(static_cast<uint8_t>(packet.checksum)) 
                << "，计算的校验和：0x" << static_cast<int>(static_cast<uint8_t>(calculated_checksum)) << std::dec << std::endl;
        return;
    }
      // 4. 根据通道和帧类型处理不同消息
    if (channel == "Robot_ControlData") {
        if (static_cast<uint16_t>(packet.frame_type) == FRAME_TYPE_ROBOT_CONTROL) {
            Base_DriveCmd drive_cmd;
            unpack_Base_DriveCmd(packet, drive_cmd);
            
            // 更新本地变量（加锁保护更佳）
            model = drive_cmd.model;
            js_vx_desire = drive_cmd.x_velocity;
            js_vy_desire = drive_cmd.y_velocity;
            js_OmegaZ_desire = drive_cmd.angular_velocity;
            js_vx_desire = std::clamp(js_vx_desire, (double)vx_min, (double)vx_max);
            js_vy_desire = std::clamp(js_vy_desire, (double)vy_min, (double)vy_max);
            js_OmegaZ_desire = std::clamp(js_OmegaZ_desire, (double)omegaz_min, (double)omegaz_max);

        } else {
            std::cerr << "[WARNING] 收到未知的帧类型：0x" << std::hex << packet.frame_type << std::dec << std::endl;
        }
    }
    else if (channel == "Application_Startup_Message_rep") {
        if (static_cast<uint16_t>(packet.frame_type) == FRAME_TYPE_APP_STARTUP_REP) {
            ApplicationStartupMessage_rep app_rep_msg;
            unpack_ApplicationStartupMessage_rep(packet, app_rep_msg);
            
            if(app_rep_msg.listen_successful == 0x01){
                std::cout << "[INFO] 监听算法程序成功！" << std::endl;
                ApplicationStartupMessage_rep_flag = true;
            }
        } else {
            std::cerr << "[WARNING] 收到未知的帧类型：0x" << std::hex << packet.frame_type << std::dec << std::endl;
        }
    }
}


void Server::Pub_ApplicationStartupMessage(){
    // 1. 构造应用启动消息
    ApplicationStartupMessage app_msg;
    app_msg.action_list = action_list;
    app_msg.vx_max = vx_max;
    app_msg.vx_min = vx_min;
    app_msg.vy_max = vy_max;
    app_msg.vy_min = vy_min;
    app_msg.omegaz_max = omegaz_max;
    app_msg.omegaz_min = omegaz_min;

    // 2. 打包到system2Algorithm_packet
    custom_lcm::system2Algorithm_packet packet;
    pack_ApplicationStartupMessage(app_msg, packet);

    // 3. 发布消息
    lcm.publish("Application_Startup_Message", packet);
}

/* 发送控制消息回报 */
void Server::Pub_Base_DriveCmd_rep(){

    // 打包到system2Algorithm_packet
    custom_lcm::system2Algorithm_packet packet;
    pack_Base_DriveCmd_rep(drive_rep_cmd, packet);

    // 发布消息
    lcm.publish("Robot_ControlData_rep", packet);
}

