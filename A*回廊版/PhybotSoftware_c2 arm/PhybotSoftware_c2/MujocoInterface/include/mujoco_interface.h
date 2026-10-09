/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#pragma once

#include <mujoco/mujoco.h>
#include "DataPackage/include/DataPackage.h"
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"

class MujocoInterface {

public:

    MujocoInterface( DataPackage &data);




    std::vector<std::string> names;
    std::string baseName;
    std::string orientationSensorName;
    std::string velSensorName;
    std::string gyroSensorName;
    std::string accSensorName;

    double metorvel_noise_std;

    void GetDataFromSim(mjModel*mj_model, mjData* mj_data);
    void SetTorque(DataPackage &data, mjData* mj_data);
    void SetDataToPackage(DataPackage &data);
    Eigen::Matrix<double, 3, 1> quatToZyx(const Eigen::Quaternion<double>& q);
    // mjModel *mj_model;
    // mjData  *mj_data;
private:


    int jointNum = 0;
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_pos;
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_pos_old;
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_vel;
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_torque;
    Eigen::Matrix<double, 3,1> RPY; // roll,pitch and yaw of baselink
    Eigen::Quaternion<double> imu_quat; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    Eigen::Matrix<double, 4,1> baseQuat; // in quat, mujoco order is [w,x,y,z], here we rearrange to [x,y,z,w]
    Eigen::Matrix<double, 6,1> ContactForce; // 3D foot-end contact force, L for 1st col, R for 2nd col
    Eigen::Matrix<double, 3,1> base_pos; // position of baselink, in world frame
    Eigen::Matrix<double, 3,1> imu_lin_acc;  // acceleration of baselink, in body frame
    Eigen::Matrix<double, 3,1> imu_angular_vel; // angular velocity of baselink, in body frame
    Eigen::Matrix<double, 3,1> base_lin_vel; // linear velocity of baselink, in body frame

    Eigen::Matrix<double, 3,1> mocap_basePos;
    Eigen::Matrix<double, 4,1> mocap_baseQuat;
    Eigen::Matrix<double, 3,1> mocap_baseVel;

    std::vector<int> jntId_qpos, jntId_qvel, jntId_dctl;

    int orientataionSensorId;
    int velSensorId;
    int gyroSensorId;
    int accSensorId;
    int baseBodyId;

    Eigen::Matrix<double, 12 ,1> q_desire ;
    // double timeStep{0.001}; // second
    bool isIni{false};

    Eigen::Matrix<double, Eigen::Dynamic,1> motor_Pos_desire;    
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_Vel_desire;    
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_Torque_desire;  

    Eigen::Matrix<double, Eigen::Dynamic,1> sim_P, sim_D;
};



