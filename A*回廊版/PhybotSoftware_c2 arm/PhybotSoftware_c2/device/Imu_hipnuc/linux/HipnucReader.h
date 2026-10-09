// HipnucReader.h
#pragma once

#include <cstdint>
#include <ctime>
#include <cstring>
#include "serial_port.h"    // 假设你有串口操作的函数
#include "hipnuc_dec.h"
#include "nmea_dec.h"
#include "hex2bin.h"
#include "kboot.h"
#include "log.h"
#include <Eigen/Dense>
#include <thread>

class HipnucReader {
public:
    // HipnucReader();

    bool start();
    Eigen::Vector4d quat = Eigen::Vector4d::Zero();
    Eigen::Vector4d quat_no_yaw = Eigen::Vector4d::Zero();
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_new = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel_new = Eigen::Vector3d::Zero();


    Eigen::Vector3d zyx = Eigen::Vector3d::Zero();


    void rpyToQuaternion(double roll, double pitch, double yaw, double& qw, double& qx, double& qy, double& qz) ;
    void quaternionToRPY(double qw, double qx, double qy, double qz, double& roll, double& pitch, double& yaw);
    void quatToZyx(double qw, double qx, double qy, double qz, double& x, double& y, double& z);

    void rotateQuatAroundY180(double& qw, double& qx, double& qy, double& qz);
private:
    int fd_;
    char recv_buf_[100];
    char log_buf_[512];
    hipnuc_raw_t hipnuc_raw_ = {0};
    nmea_raw_t nmea_raw_= {0};
    struct timespec last_time_;
    struct timespec last_display_time_;
    long long frame_count_;
    int frame_rate_ = 0;
    double elapsed_time_;

    const char* port_name = "/dev/ttyUSB0";
    int baud_rate = 921600;
    int safe_sleep(unsigned long usec);

    double prev_x = 0.0; // 上一时刻的x（roll角），用于去跳变
    bool first_call = true; // 是否第一次调用
    
    // bool openSerialPort();
    // void closeSerialPort();
    void mainLoop();
};
