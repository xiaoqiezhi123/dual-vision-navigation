#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <fstream>  // 追加: 用于写入文件

#include "HipnucReader.h"
// #include <mujoco/mujoco.h>
// #include "glfw_adapter.h"
// #include "simulate.h"
#include "array_safety.h"
#include "DataPackage/include/DataPackage.h"
// #include "mujoco_interface.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h> // 用于文件输出

#include "Joystick/include/joystick_int.h"
#include "StateMachine/include/statemachinemanager.h"
#include"../MotorList/include/realtime_controller.hpp"
#include"../MotorList/include/MotorList.hpp"
// #include "DataLogger/include/DataLogger.h"


// #define DATALOG


int main()
{

    DataPackage package;
    package.init();
  
    // StateEstimator state_estimator(package);
  
    Joystick joystick;
    joystick.init();

    HipnucReader reader;
    reader.start();
  
    // DataLogger Logger;
    // spdlog::info("DataLogger");
  
    StateMachineManager manager;
    // spdlog::info("StateMachineManager_RL");
    manager.init(package);
    // spdlog::info("manager.init");
    // 设置日志级别
    spdlog::set_level(spdlog::level::info); // 只记录INFO级别及以上的日志
    
  
    spdlog::info("model load success!");




    Eigen::VectorXd P_control_vector, D_control_vector, direction_vector;


    std::string config_yaml1 = "../MotorList/config/bigphybot1.yaml";
    std::string config_yaml2 = "../MotorList/config/bigphybot2.yaml";
    std::string config_yaml3 = "../MotorList/config/bigphybot3.yaml";
    MotorList motorlist;

    motorlist.Init(config_yaml1,config_yaml2, config_yaml3, package);

    int motorNum = motorlist.num_motors;
    direction_vector = motorlist.direction_vector;



#ifdef DATALOG
    std::ofstream foutData;
    foutData.open("datacollection.txt", std::ios::out);
    // Eigen::VectorXd dataL = Eigen::VectorXd::Zero(34 + 6 * motorNum);
    Eigen::VectorXd dataL = Eigen::VectorXd::Zero(400);
#endif

    std::cout << "Motorlist init complete! " << std::endl;

    Eigen::VectorXd pos_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd vel_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd tor_actual = Eigen::VectorXd::Zero(motorNum);

    Eigen::VectorXd pos_desire = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd vel_desire = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd tor_desire = Eigen::VectorXd::Zero(motorNum);

    // 保存数据的容器（使用 vector 存储每次循环的数据）
    std::vector<std::vector<double>> pos_actual_data;
    std::vector<std::vector<double>> vel_actual_data;
    std::vector<std::vector<double>> pos_desire_data;
    std::vector<std::vector<double>> vel_desire_data;

    std::vector<std::vector<double>> tor_actual_data;
    std::vector<std::vector<double>> tor_desire_data;
    

    auto record_start_time = std::chrono::steady_clock::now();
    auto log_start_time = std::chrono::steady_clock::now();



    // 多次获取当前电机位置
    for (int i = 0; i < 50; ++i) {
        motorlist.GetStatesToPackage(pos_actual, vel_actual, tor_actual, direction_vector, package, 0);
        int motor_idx = 0;
        std::cout << "**************@@@@@@@@@@@@@@@@@@@@@" << std::endl;
        for (const auto& motorPair : *motorlist.Motors_Map) {
            std::cout << motorPair.first << "\t\t" 
                    << pos_actual(motor_idx) << "\t\t"
                    << vel_actual(motor_idx) << "\t\t"
                    << tor_actual(motor_idx) << std::endl;
            motor_idx++;
        }

    }

    // 所有电机使能之后，睡2s.
    if (!motorlist.Enable()) {
        std::cerr << "Motor enable failed, exiting program..." << std::endl;
        return EXIT_FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(6000));

//-------------------------------------------------------------------------------------
    period_info P_info;
    periodic_task_init(&P_info, package.control_period);
    while(1){


        // std::cout << "motor_name_list: " << motor_name_list <<std::endl;
          // ************ test ****************

          auto run_start_time = std::chrono::steady_clock::now();

          joystick.GetDataFromPackage(package);
          joystick.run();
          joystick.SetDataToPackage(package);

          motorlist.GetStatesToPackage(pos_actual, vel_actual, tor_actual, direction_vector, package, 0);

          package.getIMUdata(reader);


          manager.GetDataFromPackage(package);
          manager.run(package);
          manager.SetDataToPackage(package); 

        //   motorlist.SetCommands(pos_desire, vel_desire, tor_desire, 2 * P_control_vector, 7 *D_control_vector, direction_vector, package);

          motorlist.SetCommandsFromPackage(pos_desire, vel_desire, tor_desire, direction_vector, package);
          // std::cout << "pos_desire: " << pos_desire << std::endl;
          // std::cout << "vel_desire: " << vel_desire << std::endl;
          // std::cout << "tor_desire: " << tor_desire << std::endl;
          // std::cout << "P_control_vector: " << P_control_vector << std::endl;
          // std::cout << "D_control_vector: " << D_control_vector << std::endl;
          // std::cout << "direction_vector: " << direction_vector << std::endl;
  
          wait_rest_of_period(&P_info);

          auto record_now_time = std::chrono::steady_clock::now();
         
          std::chrono::duration<double> elapsed_record = record_now_time - run_start_time;
          // std::cout<<"elapsed_record"<<elapsed_record.count()<<std::endl;

          auto log_now_time = std::chrono::steady_clock::now();
         
          std::chrono::duration<double> elapsed_log = log_now_time - log_start_time;


#ifdef DATALOG
          dataL[0] = elapsed_log.count();
        //   dataL.segment(1, 289) = package.input_nlp;
        //   dataL.segment(290, 19) = package.output_nlp;
        //   dataL.segment(309, 19) = package.motor_pos;
        //   dataL.segment(328, 19) = package.motor_vel;
        //   dataL.segment(347, 19) = package.motor_torque;

          dataL.block(1, 0, 289, 1) = package.input_nlp;
          dataL.block(290, 0, 19, 1) = package.output_nlp;
          dataL.block(309, 0, 19, 1) = package.motor_pos;
          dataL.block(328, 0, 19, 1) = package.motor_vel;
          dataL.block(347, 0, 19, 1) = package.motor_torque;
          dataL.block(366, 0, 19, 1) = package.motor_Pos_desire;


        //   dataL.block(25 , 0, motorNum, 1) = package.generalized_q_desired.tail(motorNum);
        //   dataL.block(25 + 1 * motorNum, 0, motorNum, 1) = package.generalized_q_dot_desired.tail(motorNum);
        //   dataL.block(25 + 2 * motorNum, 0, motorNum, 1) = package.motor_Torque_desire;  //only for real robot
        //   dataL.block(25 + 3 * motorNum, 0, motorNum, 1) = package.motor_pos;
        //   dataL.block(25 + 4 * motorNum, 0, motorNum, 1) = package.motor_vel;
        //   dataL.block(25 + 5 * motorNum, 0, motorNum, 1) = package.motor_torque;
        //   dataL.block(25 + 6 * motorNum, 0, 3, 1) = package.imu_lin_acc;
        //   dataL.block(25 + 6 * motorNum+3, 0, 3, 1) = package.vel_base_obser;
        //   dataL[31 + 6 * motorNum] = package.contact_force[2];
        //   dataL[32 + 6 * motorNum] = package.contact_force[8];
        //   dataL[33 + 6 * motorNum] = package.FootNum;
          
          // Write to file


        //   dataL[0] = elapsed_log.count();
        //   dataL.block(1, 0, 6, 1) = package.generalized_q_actual.head(6);
        //   dataL.block(7, 0, 6, 1) = package.generalized_q_dot_actual.head(6);
        //   dataL.block(13, 0, 6, 1) = package.generalized_q_desired.head(6);
        //   dataL.block(19, 0, 6, 1) = package.generalized_q_dot_desired.head(6);
        //   dataL.block(25 , 0, motorNum, 1) = package.generalized_q_desired.tail(motorNum);
        //   dataL.block(25 + 1 * motorNum, 0, motorNum, 1) = package.generalized_q_dot_desired.tail(motorNum);
        //   dataL.block(25 + 2 * motorNum, 0, motorNum, 1) = package.motor_Torque_desire;  //only for real robot
        //   dataL.block(25 + 3 * motorNum, 0, motorNum, 1) = package.motor_pos;
        //   dataL.block(25 + 4 * motorNum, 0, motorNum, 1) = package.motor_vel;
        //   dataL.block(25 + 5 * motorNum, 0, motorNum, 1) = package.motor_torque;
        //   dataL.block(25 + 6 * motorNum, 0, 3, 1) = package.imu_lin_acc;
        //   dataL.block(25 + 6 * motorNum+3, 0, 3, 1) = package.vel_base_obser;


          dataLog(dataL, foutData);
#endif


    }


#ifdef DATALOG
      foutData.close();
#endif

    return 0;
}


