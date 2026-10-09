#ifndef DATAPACKAGE_H
#define DATAPACKAGE_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include "yaml-cpp/yaml.h"



#include "StateMachine/include/fsmlist.h"

#include "device/Imu_hipnuc/linux/HipnucReader.h"
// #include "/home/phybot/Documents/PhybotSofware/StateMachine/include/fsmlist.h"

class DataPackage
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    DataPackage();
    // void UpdateAfterSetData();
    void init();
    void getIMUdata(HipnucReader reader);
    // data
    // dimension of the robot dim = motorNum + floatingNum


    int generalizedCoordinatesNum;
    int actuatedDofNum;
    // control cycle time
    double control_period;

    int IndexLegStart, IndexLegLength;
    int IndexArmStart, IndexArmLength;
    int IndexWaistStart, IndexWaistLength;
    /**
     * @brief q_a
     *     joint position sense
     * qa = [q_float, q_joint] for floating base robot and mobile robot
     * qa = [q_joint] for fixed robot
     */

    // only pinocchio use
    Eigen::Matrix<double, Eigen::Dynamic,1> generalized_q_actual;
    Eigen::Matrix<double, Eigen::Dynamic,1> generalized_q_desired;
 
    Eigen::Matrix<double, Eigen::Dynamic,1> generalized_q_dot_actual;
    Eigen::Matrix<double, Eigen::Dynamic,1> generalized_q_dot_desired;


    // from sensors 
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_torque;
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_vel;
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_pos;
    Eigen::Matrix<double, Eigen::Dynamic,1> sim_P;
    Eigen::Matrix<double, Eigen::Dynamic,1> sim_D;
    Eigen::Matrix<double, 3,1> imu_lin_acc;     
    Eigen::Quaternion<double> imu_quat;      
    Eigen::Matrix<double, 4,1>  baseQuat;      
    Eigen::Matrix<double, 3,1> imu_rpy;  
    Eigen::Matrix<double, 3,1> imu_zyx;     
    Eigen::Matrix<double, 3,1> imu_angular_vel;      

    //from state estimator
    Eigen::Matrix<double, Eigen::Dynamic,1> contact_force;      
    Eigen::Matrix<double, 3,1> base_lin_vel;     
    Eigen::Matrix<double, 3,1> base_pos;     
    std::vector<Eigen::VectorXd> contact_force_history;
    std::vector<double> speed_forward_history;
    double speed_forward_avg{0};

    // from mocap device
    Eigen::Matrix<double, 3,1> mocap_basePos;
    Eigen::Matrix<double, 4,1> mocap_baseQuat;
    Eigen::Matrix<double, 3,1> mocap_baseVel;

    Eigen::Vector3d mocap_hit_position;
    double mocap_hit_time;
    bool mocap_hit_flag = false;




    Eigen::Matrix<double, Eigen::Dynamic,1> q_desire;    // joint position command

    Eigen::Matrix<double, Eigen::Dynamic,1> q_dot_desire;    // joint velocity command

    Eigen::Matrix<double, Eigen::Dynamic,1> q_dot_dot_desire;      // joint acceleration command

    Eigen::Matrix<double, Eigen::Dynamic,1> torq_desire;    // joint torque command

    Eigen::Matrix<double, Eigen::Dynamic,1> motor_Pos_desire;    
    Eigen::Matrix<double, Eigen::Dynamic,1> motor_Vel_desire;    
   Eigen::Matrix<double, Eigen::Dynamic,1> motor_Torque_desire;  



    // head motor commands
    double neck_yaw_xx{0};
    double neck_pitch_yy{0};

    // fast motor feedback data (from GetPVCTFast)
    Eigen::Matrix<double, Eigen::Dynamic,1> FastStateMechine;
    Eigen::Matrix<double, Eigen::Dynamic,1> FastMosTemperature;
    Eigen::Matrix<double, Eigen::Dynamic,1> FastWindingTemperature;
    Eigen::Matrix<double, Eigen::Dynamic,1> FastBusVoltage;
    Eigen::Matrix<double, Eigen::Dynamic,1> FastErrorCode;

    // states and key variables for Joystick
    double js_vx_desire{0}, js_vy_desire{0};
    double js_OmegaZ_desire{0};
    // int control_mode{0} = 1;
    int control_mode = 1;
    double acc_max;
    


    State CurrentState{State::ZERO};
    State NextState{State::ZERO};

    


   bool If_first_Zero = true;

    bool zero_target_reached = false;

};

#endif // DATAPACKAGE_H
