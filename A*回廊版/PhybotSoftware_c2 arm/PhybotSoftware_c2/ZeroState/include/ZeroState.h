
#pragma once

#include <Eigen/Dense>
#include <utility>
#include <vector>
#include <string>
#include <iostream>


#include "DataPackage/include/DataPackage.h"



class ZeroState {
public:

    ZeroState();
    int actuatedDofNum;

    Eigen::VectorXd zero_pose;
    Eigen::VectorXd zero_p;
    Eigen::VectorXd zero_d;
    Eigen::VectorXd init_pose;
    bool If_first_Zero = true;
    double t_Zero, dt_Zero;
    double zero_totalTime;

    Eigen::Matrix<double, Eigen::Dynamic, 1> generalized_q_actual;
    Eigen::Matrix<double, Eigen::Dynamic, 1> generalized_q_dot_actual;

    Eigen::Matrix<double, Eigen::Dynamic,1> motor_pos;


    void GetDataFromPackage(DataPackage &data);
    void SetDataToPackage(DataPackage &data);
    void ThirdpolyVector(const Eigen::VectorXd& p0, const Eigen::VectorXd& p0_dot,
        const Eigen::VectorXd& p1, const Eigen::VectorXd& p1_dot,
        double totalTime, double currentTime,
        Eigen::VectorXd& pd, Eigen::VectorXd& pd_dot);


};



