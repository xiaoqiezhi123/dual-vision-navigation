#include "../include/DataPackage.h"



DataPackage::DataPackage()
{

}
void DataPackage::init()
{
    // 
    // std::cout << "DataPackage " << std::endl;
    std::string file_path = "../DataPackage/config/datapackage.yaml";
    YAML::Node config = YAML::LoadFile(file_path);
    control_period = config["control_period"].as<double>();
    actuatedDofNum = config["actuatedDofNum"].as<int>();




    generalizedCoordinatesNum =  actuatedDofNum + 6;
    // std::cout<<"actuatedDofNum: "<<robot_model<<std::endl;

    //
    // dt = ControlPeriod;
    //




    generalized_q_actual.setZero(generalizedCoordinatesNum,1);
    generalized_q_desired.setZero(generalizedCoordinatesNum,1);
    generalized_q_dot_actual.setZero(generalizedCoordinatesNum,1);
    generalized_q_dot_desired.setZero(generalizedCoordinatesNum,1);


    motor_torque.setZero(actuatedDofNum,1);
    motor_vel.setZero(actuatedDofNum,1);
    motor_pos.setZero(actuatedDofNum,1);
    sim_P.setZero(actuatedDofNum,1);
    sim_D.setZero(actuatedDofNum,1);
    
    q_desire.setZero(actuatedDofNum,1);
    q_dot_desire.setZero(actuatedDofNum,1);
    q_dot_dot_desire.setZero(actuatedDofNum,1);
    torq_desire.setZero(actuatedDofNum,1);



    motor_Pos_desire.setZero(actuatedDofNum,1);
    motor_Vel_desire.setZero(actuatedDofNum,1);
    motor_Torque_desire.setZero(actuatedDofNum,1);


}
void DataPackage::getIMUdata(HipnucReader reader){
    imu_zyx = reader.zyx;

    // std::cout<<"imu_zyx: "<<imu_zyx<<std::endl;
    imu_angular_vel = reader.ang_vel_new*M_PI/180;
    imu_lin_acc = reader.acc_new;
}

// void DataPackage::UpdateAfterSetData()
// {
//         base_omega_W << baseAngVel[0],baseAngVel[1],baseAngVel[2];
//         auto Rcur= eul2Rot(rpy[0], rpy[1], rpy[2]);
//         base_omega_W=Rcur*base_omega_W;

//         //  q = [global_base_position, global_base_quaternion, joint_positions]
//         //  dq = [global_base_velocity_linear, global_base_velocity_angular, joint_velocities]

//         auto quatNow=eul2quat(rpy[0], rpy[1], rpy[2]);
//         q(0)=basePos[0];
//         q(1)=basePos[1];
//         q(2)=basePos[2];
//         q(3)=quatNow.x();
//         q(4)=quatNow.y();
//         q(5)=quatNow.z();
//         q(6)=quatNow.w();
//         for (int i=0;i<model_nv-6;i++)
//             q(i+7)=motors_pos_cur[i];

//         Eigen::Vector3d vCoM_W;
//         vCoM_W << baseLinVel[0],baseLinVel[1],baseLinVel[2];
//         dq.block<3,1>(0,0)= vCoM_W;
//         dq.block<3,1>(3,0) << base_omega_W[0],base_omega_W[1],base_omega_W[2];
// //        dq.block<3,1>(3,0) << baseAngVel[0],baseAngVel[1],baseAngVel[2];
//         for (int i=0;i<model_nv-6;i++)
//         {
//             dq(i+6)=motors_vel_cur[i];
//         }

//         base_pos<<q(0),q(1),q(2);
//         base_rpy << rpy[0], rpy[1], rpy[2];
//         base_rot=Rcur;
//         qOld=q;
// }