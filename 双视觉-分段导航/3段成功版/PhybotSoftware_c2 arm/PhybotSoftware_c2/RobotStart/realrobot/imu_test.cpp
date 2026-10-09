#include "HipnucReader.h"
#include "commands.h"
#include "global_options.h"
#include "log.h"
#include <cstdint>
#include <ctime>
#include <cstring>
#include "serial_port.h"    // 假设你有串口操作的函数
#include "hipnuc_dec.h"
#include "nmea_dec.h"
#include "hex2bin.h"
#include "kboot.h"
#include "log.h"
#include <iostream>
#include <thread>
#include <chrono>
#include"../MotorList/include/MotorList.hpp"
#include"../MotorList/include/realtime_controller.hpp"
// #include "DataPackage/include/DataPackage.h"

int main() {
    period_info P_info;

    periodic_task_init(&P_info, 0.02);

    // HipnucReader reader;
    // reader.start();

    // DataPackage package;
    // package.init();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    auto record_start_time = std::chrono::steady_clock::now();
    

    while(1)
    {
        // auto record_start_time = std::chrono::steady_clock::now();
        // std::cout<<"xx: "<<reader.ang_vel_new<<std::endl;
        // header.rpyToQuaternion()

        auto record_now_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_record = record_now_time - record_start_time;

        std::cout<<"elapsed_record: "<<elapsed_record.count()<<std::endl;
        wait_rest_of_period(&P_info);

    }


    // if (!reader.start()) {
    //     return -1;
    // }
    return 0;
}