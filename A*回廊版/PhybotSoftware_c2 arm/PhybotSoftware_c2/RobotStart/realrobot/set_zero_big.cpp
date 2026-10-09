#include"../MotorList/include/MotorList.hpp"
#include"../MotorList/include/realtime_controller.hpp"
#include "ZeroState/include/ZeroState.h"
#include <fstream>  // 追加: 用于写入文件
#include "DataPackage/include/DataPackage.h"
#include"../MotorList/include/MotorList.hpp"

// #include "HipnucReader.h"



void writeEigenVectorToYaml(const std::string& filename, 
                           const std::string& vectorName,
                           const Eigen::VectorXd& vec) {
    YAML::Node config;
    
    // 尝试读取已有的YAML文件
    try {
        config = YAML::LoadFile(filename);
    } catch (const YAML::BadFile& e) {
        // 如果文件不存在，创建一个新的配置
        std::cout << "File not found, creating new file: " << filename << std::endl;
    }
    
    // 直接创建序列节点并添加向量元素
    YAML::Node vecNode;
    for (int i = 0; i < vec.size(); ++i) {
        vecNode.push_back(vec[i]);
    }
    config[vectorName] = vecNode;
    
    // 写入文件
    std::ofstream fout(filename);
    fout << config;
    fout.close();
}



int main()
{

    double total_time = 15.0;
    int mode =1;
    double current_time;
    std::vector<std::string> motor_name_list;

    DataPackage package;
    package.init();
  

    // Eigen::VectorXd P_control_vector, D_control_vector, direction_vector;


    std::string config_yaml1 = "../MotorList/config/bigphybot1.yaml";
    std::string config_yaml2 = "../MotorList/config/bigphybot2.yaml";
    std::string config_yaml3 = "../MotorList/config/bigphybot3.yaml";
    std::string config_yaml_abs = "../MotorList/config/phybot_abszero.yaml";

    MotorList motorlist;

    motorlist.Init(config_yaml1,config_yaml2, config_yaml3, package);

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
    Eigen::VectorXd ones_vector = Eigen::VectorXd::Ones(motorNum) *0.1;
    Eigen::VectorXd direction_vector = Eigen::VectorXd::Ones(motorNum) ;

    direction_vector = motorlist.direction_vector;

    std::cout<<"direction_vector: "<<direction_vector<<std::endl;
    ZeroState gotozero;



    std::cout << "Motorlist init complete! " << std::endl;


    period_info P_info;
    periodic_task_init(&P_info, 0.002);


        // 多次获取当前电机位置
    for (int i = 0; i < 3; ++i) {
        motorlist.GetStates(pos_actual, vel_actual, tor_actual, direction_vector);


        int motor_idx = 0;
        std::cout << "**************@@@@@@@@@@@@@@@@@@@@@" << std::endl;
        for (const auto& motorPair : *motorlist.Motors_Map) {
            std::cout << motorPair.first << "\t\t" 
                    << pos_actual(motor_idx) << "\t\t"
                    << vel_actual(motor_idx) << "\t\t"
                    << tor_actual(motor_idx) << std::endl;


            init_pos_actual(motor_idx ) = pos_actual(motor_idx) * direction_vector(motor_idx);

            motor_idx++;

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    }
    std::cout<<"init_pos_actual: "<<init_pos_actual<<std::endl;


    writeEigenVectorToYaml(config_yaml_abs, "abs_zero", init_pos_actual);


    return 0;
}


