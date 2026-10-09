#include"../MotorList/include/MotorList.hpp"
#include"../MotorList/include/realtime_controller.hpp"
#include "ZeroState/include/ZeroState.h"
#include <fstream>  // 追加: 用于写入文件
#include "DataPackage/include/DataPackage.h"
#include"../MotorList/include/MotorList.hpp"

// #include "HipnucReader.h"

#define DATALOG



bool dataLog(Eigen::VectorXd &v, std::ofstream &f) {
    for (int i = 0; i < v.size(); i++) {
        f << v[i] << " ";
    }
    f << std::endl;
    return true;
}





int main()
{

    double total_time = 6.0;
    int mode =1;
    double current_time;
    std::vector<std::string> motor_name_list;

    DataPackage package;
    package.init();
  

    // Eigen::VectorXd P_control_vector, D_control_vector, direction_vector;


    std::string config_yaml1 = "../MotorList/config/phybot_mini_1.yaml";
    std::string config_yaml2 = "../MotorList/config/phybot_mini_2.yaml";
    MotorList motorlist;

    motorlist.Init(config_yaml1,config_yaml2, package);

    // loadMotorConfig(config_yaml, motor_name_list, P_control_vector, D_control_vector, direction_vector);
    int motorNum = motorlist.num_motors;


    // HipnucReader reader;
    // reader.start();

    // std::cout << "Loaded motor count: " << motorNum << std::endl;
    // for (int i = 0; i < motorNum; ++i) {
    //     std::cout << motor_name_list[i] << " -> P_control: " << P_control_vector(i) 
    //     << " -> D_control: " << D_control_vector(i) 
    //     << " -> direction: " << dir_control_vector(i) 
    //     << std::endl;
    // }
    Eigen::VectorXd pos_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd vel_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd tor_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd init_pos_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd init_vel_actual = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd pos_desire = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd vel_desire = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd tor_desire = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd zero_vector = Eigen::VectorXd::Zero(motorNum);
    Eigen::VectorXd direction_vector = Eigen::VectorXd::Ones(motorNum) ;

    direction_vector = motorlist.direction_vector;

    ZeroState gotozero;



    std::cout << "Motorlist init complete! " << std::endl;

#ifdef DATALOG
    std::ofstream foutData;
    foutData.open("./datacollection.txt", std::ios::out);
    // Eigen::VectorXd dataL = Eigen::VectorXd::Zero(34 + 6 * motorNum);
    Eigen::VectorXd dataL = Eigen::VectorXd::Zero(400);
#endif

    period_info P_info;


        // 多次获取当前电机位置
    for (int i = 0; i < 50; ++i) {
        motorlist.GetStates(pos_actual, vel_actual, tor_actual, direction_vector);


        int motor_idx = 0;
        std::cout << "**************@@@@@@@@@@@@@@@@@@@@@" << std::endl;
        for (const auto& motorPair : *motorlist.Motors_Map) {
            std::cout << motorPair.first << "\t\t" 
                    << pos_actual(motor_idx) << "\t\t"
                    << vel_actual(motor_idx) << "\t\t"
                    << tor_actual(motor_idx) << std::endl;



            motor_idx++;

        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    }

    // // 所有电机使能之后，睡2s.
    if (!motorlist.Enable()) {
        std::cerr << "Motor enable failed, exiting program..." << std::endl;
        return EXIT_FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));



    // for (int i = 0; i < 500; ++i) {


    //     motorlist.GetStates(pos_actual, vel_actual, tor_actual, direction_vector, package, 0);


    //     int motor_idx = 0;
    //     std::cout << "**************@@@@@@@@@@@@@@@@@@@@@" << std::endl;
    //     for (const auto& motorPair : *motorlist.Motors_Map) {
    //         std::cout << motorPair.first << "\t\t" 
    //                 << pos_actual(motor_idx) << "\t\t"
    //                 << vel_actual(motor_idx) << "\t\t"
    //                 << tor_actual(motor_idx) << std::endl;



    //         motor_idx++;

    //     }

    //     motorlist.SetCommands(pos_actual, vel_desire, tor_desire, direction_vector, package);
    //     std::cout<<"pos_actual: "<< pos_actual(20)<<std::endl;

    //     std::this_thread::sleep_for(std::chrono::milliseconds(2));

    // }
    int i = 0;
    periodic_task_init(&P_info, 0.002);

    auto run_start_time = std::chrono::steady_clock::now();

    while(1){


        std::cout<<"I : "<<i<<std::endl;

        // std::cout<<"xx: "<<reader.ang_vel_new<<std::endl;make

        motorlist.GetStates(pos_actual, vel_actual, tor_actual, direction_vector);

        std::cout<<"pos_actual: "<< pos_actual(0)<<std::endl;

        if(mode == 1)
        {

            if(current_time< total_time)
            {
                ThirdpolyVector(pos_actual, zero_vector, gotozero.zero_pose, zero_vector,
                        total_time, current_time, pos_desire, vel_desire);
            }

            else {
                current_time = 0;
                mode = 2;

            }

        }

        else if(mode == 2)
        {

            if(current_time < total_time)
            {
                ThirdpolyVector(pos_actual, zero_vector, zero_vector, zero_vector,
                        total_time, current_time, pos_desire, vel_desire);
            }

            else {
                current_time = 0;
                mode = 1;

            }

        }

        // std::cout<<"pos_desire1: "<<pos_desire(0)<<std::endl;
        pos_desire(0) = sin(6.28/2 * i *0.02);

        // if(i>1000 && i<1500)
        // {
        //     pos_desire(0) = 0;
        // }
        motorlist.SetCommands(pos_desire , vel_desire, tor_desire, direction_vector);
        // motorlist.SetCommands(zero_vector, zero_vector, zero_vector, 1 * P_control_vector, 1 * D_control_vector, direction_vector);

        wait_rest_of_period(&P_info);

        // std::this_thread::sleep_for(std::chrono::milliseconds(2));

        current_time += 0.002;
        auto record_now_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_record = record_now_time - run_start_time;

        std::cout<<"elapsed_record: "<<elapsed_record.count()<<std::endl;
        i++;

#ifdef DATALOG

          dataL[0] = elapsed_record.count();
        //   dataL.segment(1, 289) = package.input_nlp;
        //   dataL.segment(290, 19) = package.output_nlp;
        //   dataL.segment(309, 19) = package.motor_pos;
        //   dataL.segment(328, 19) = package.motor_vel;
        //   dataL.segment(347, 19) = package.motor_torque;

          dataL.block(1, 0, 21, 1) = pos_desire;
          dataL.block(22, 0, 21, 1) = pos_actual;
          dataL.block(43, 0, 21, 1) = vel_actual;
          dataL.block(64, 0, 21, 1) = tor_actual;

          dataLog(dataL, foutData);
#endif
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(10000));





    return 0;
}


