#include "HipnucReader.h"
#include <cstdio>
#include <unistd.h> // for usleep
#include <iostream>
#include <thread>
#include <iostream>
#include <chrono>
#include <iomanip>  // for std::put_time
#include <ctime>    // for std::tm
#include "yaml-cpp/yaml.h"


#define CMD_REPLAY_TIMEOUT_MS (100)
#define DISPLAY_UPDATE_INTERVAL 0.05
#define MAX_ATTEMPTS 2

bool HipnucReader::start() {


    std::string file_path = "../device/config/device.yaml";
    YAML::Node config = YAML::LoadFile(file_path);
    std::string port_str = config["port_name"].as<std::string>();
    port_name = port_str.c_str();


    baud_rate = config["baud_rate"].as<int>();

    if ((fd_ = serial_port_open(port_name)) < 0 || serial_port_configure(fd_, baud_rate) < 0) {
        log_error("Failed to open or configure port %s with %d",port_name, baud_rate);
        return -1;
    }

    log_info("Reading from port %s at %d baud. Press CTRL+C to exit.", port_name, baud_rate);

    // Enable data output
    serial_send_then_recv_str(fd_, "AT+EOUT=1\r\n", "OK\r\n", recv_buf_, sizeof(recv_buf_), 200);



    clock_gettime(CLOCK_MONOTONIC, &last_time_);
    last_display_time_ = last_time_;

    std::thread main_loop_thread(&HipnucReader::mainLoop, this);
    main_loop_thread.detach();  // Detach the thread to run independently

    return true;
}

void HipnucReader::mainLoop() {
    while (true) {
        bool new_hipnuc_data = false;
        bool new_nmea_data = false;


        struct timespec start, end;
        double elapsed_ms;

        clock_gettime(CLOCK_MONOTONIC, &start);



        int len = serial_port_read(fd_, (char*)recv_buf_, sizeof(recv_buf_));

        clock_gettime(CLOCK_MONOTONIC, &end);   // 获取结束时间

        elapsed_ms = (end.tv_sec - start.tv_sec) * 1000.0
                    + (end.tv_nsec - start.tv_nsec) / 1e6;

        // printf("Elapsed time: %.3f ms\n", elapsed_ms);
        // printf("Elapsed time: %.3d ms\n", len);


        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (hipnuc_input(&hipnuc_raw_, recv_buf_[i]) > 0) {
                    new_hipnuc_data = true;
                    frame_count_++;
                }
                if (input_nmea(&nmea_raw_, recv_buf_[i]) > 0) {
                    new_nmea_data = true;
                    frame_count_++;
                }
            }
        }

        struct timespec current_time;
        clock_gettime(CLOCK_MONOTONIC, &current_time);

        elapsed_time_ = (current_time.tv_sec - last_time_.tv_sec) +
                        (current_time.tv_nsec - last_time_.tv_nsec) / 1e9;
        double display_elapsed_time = (current_time.tv_sec - last_display_time_.tv_sec) +
                                      (current_time.tv_nsec - last_display_time_.tv_nsec) / 1e9;

        if ((new_hipnuc_data || new_nmea_data) ) {
            // printf("\033[H\033[J");

            if (new_hipnuc_data) {
                hipnuc_dump_packet(&hipnuc_raw_, log_buf_, sizeof(log_buf_));

                acc(0) = hipnuc_raw_.hi91.acc[0] * 9.8;
                acc(1) = hipnuc_raw_.hi91.acc[1] * 9.8;
                acc(2) = hipnuc_raw_.hi91.acc[2] * 9.8;
                ang_vel(0) = hipnuc_raw_.hi91.gyr[0];
                ang_vel(1) = hipnuc_raw_.hi91.gyr[1];
                ang_vel(2) = hipnuc_raw_.hi91.gyr[2];
                rpy(0) = hipnuc_raw_.hi91.roll;

                rpy(1) = hipnuc_raw_.hi91.pitch;
                
                rpy(2) = hipnuc_raw_.hi91.yaw;

                quat(0) = hipnuc_raw_.hi91.quat[0];
                quat(1) = hipnuc_raw_.hi91.quat[1];
                quat(2) = hipnuc_raw_.hi91.quat[2];
                quat(3) = hipnuc_raw_.hi91.quat[3];


                rpyToQuaternion(hipnuc_raw_.hi91.roll, hipnuc_raw_.hi91.pitch, 0, quat_no_yaw(0), quat_no_yaw(1), quat_no_yaw(2), quat_no_yaw(3));

                rotateQuatAroundY180(quat_no_yaw(0), quat_no_yaw(1), quat_no_yaw(2), quat_no_yaw(3));
                // rotateZMinus90(quat_no_yaw(0), quat_no_yaw(1), quat_no_yaw(2), quat_no_yaw(3), quat_new(0), quat_new(1), quat_new(2), quat_new(3));
                quatToZyx(quat_no_yaw(0), quat_no_yaw(1), quat_no_yaw(2), quat_no_yaw(3), zyx(2), zyx(1), zyx(0));
                zyx(1) = -zyx(1);
                // zyx(0) = 

                // multiplyQuaternions(quat_no_yaw(0), quat_no_yaw(1), quat_no_yaw(2), quat_no_yaw(3), quat_new(0), quat_new(1), quat_new(2), quat_new(3));
                // removeYawFromQuaternion(quat(3),quat(0),quat(1),quat(2), quat_new(3),quat_new(0),quat_new(1),quat_new(2));

                acc_new(0) = - hipnuc_raw_.hi91.acc[1] * 9.8;
                acc_new(1) = - hipnuc_raw_.hi91.acc[0] * 9.8;
                acc_new(2) = - hipnuc_raw_.hi91.acc[2] * 9.8;

                ang_vel_new(0) = - hipnuc_raw_.hi91.gyr[1];
                ang_vel_new(1) = - hipnuc_raw_.hi91.gyr[0];
                ang_vel_new(2) = - hipnuc_raw_.hi91.gyr[2];


                // printf("%s\n", log_buf_);
            }

            if (new_nmea_data) {
                nmea_dec_dump_msg(&nmea_raw_, log_buf_, sizeof(log_buf_));
                printf("%s\n", log_buf_);
            }

            // printf("Frame Rate: %d fps\n", frame_rate_);

            last_display_time_ = current_time;
        }

        if (elapsed_time_ >= 1.0) {
            frame_rate_ = (int)(frame_count_ / elapsed_time_);
            frame_count_ = 0;
            last_time_ = current_time;
        }




        // 转换为系统时间（time_t 类型）
        // std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);


        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    }
}

int HipnucReader::safe_sleep(unsigned long usec) {
    struct timespec ts;
    ts.tv_sec = usec / 1000000;
    ts.tv_nsec = (usec % 1000000) * 1000;
    
    while (nanosleep(&ts, &ts) == -1) {
        if (errno != EINTR) {
            return -1;
        }
    }
    return 0;
}


// 将滚转角、俯仰角、偏航角转换为四元数
void HipnucReader::rpyToQuaternion(double roll, double pitch, double yaw, double& qw, double& qx, double& qy, double& qz) 
{
    // 将角度转换为弧度
    roll = roll * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    yaw = yaw * M_PI / 180.0;

    // 计算四元数
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}


void HipnucReader::quaternionToRPY(double qw, double qx, double qy, double qz, double& roll, double& pitch, double& yaw) {
    // 计算滚转角（Roll）
    roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));

    // 计算俯仰角（Pitch）
    pitch = asin(2.0 * (qw * qy - qz * qx));

    // 计算偏航角（Yaw）
    yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    // // 将弧度转换为度（可选）
    // roll = roll * 180.0 / M_PI;
    // pitch = pitch * 180.0 / M_PI;
    // yaw = yaw * 180.0 / M_PI;
}
void HipnucReader::quatToZyx(double qw, double qx, double qy, double qz, double& x, double& y, double& z)
{
    // 保证 sin(pitch) 的值不会超过 [-1, 1] 范围，避免 NaN
    double as = std::min(-2. * (qx * qz - qw * qy), 0.99999);

    // 计算欧拉角
    z = std::atan2(2 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
    y = std::asin(as);
    x = std::atan2(2 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz);
    
    // // 将弧度转换为度（可选）
    // z = z * 180.0 / M_PI;
    // y = y * 180.0 / M_PI;
    // x = x * 180.0 / M_PI;
}


void HipnucReader::rotateQuatAroundY180(double& qw, double& qx, double& qy, double& qz)
{
    // 定义绕y轴旋转180度的四元数
    double qw_rot = 0.0;
    double qx_rot = 0.0;
    double qy_rot = 1.0;
    double qz_rot = 0.0;

    // 保存原四元数
    double ow = qw, ox = qx, oy = qy, oz = qz;

    // 四元数乘法：new_q = original_q * rotation_q
    qw = ow * qw_rot - ox * qx_rot - oy * qy_rot - oz * qz_rot;
    qx = ow * qx_rot + ox * qw_rot + oy * qz_rot - oz * qy_rot;
    qy = ow * qy_rot - ox * qz_rot + oy * qw_rot + oz * qx_rot;
    qz = ow * qz_rot + ox * qy_rot - oy * qx_rot + oz * qw_rot;
}