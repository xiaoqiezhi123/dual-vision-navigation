
#pragma once

#include <iostream>
#include <string> 
#include <thread>
#include <time.h>
#include <mutex>
#include <queue>
#include <map>
#include "Mocap/include/LuMoSDKBase.hpp"
#include "DataPackage/include/DataPackage.h"


class MocapStreamer {
  public:
    MocapStreamer();
    ~MocapStreamer();
    void init();
    void Step();
    void GetDataFromPackage(DataPackage &DataPackage);
    void SetDataToPackage(DataPackage &DataPackage);

    // properties
    std::string broadcaster_ip;
    std::string base_link_name;
    std::string shuttlecock_name;
    std::vector<double> base_offsets;
    double waist_yaw_angle;

    // data retrivers
    Eigen::Matrix<double, 3, 1> mocap_baseOrigin;
    Eigen::Matrix<double, 3, 1> mocap_basePos;
     Eigen::Matrix<double, 3, 1> mocap_basePos_Prev;
    Eigen::Matrix<double, 4, 1> mocap_baseQuat;
    Eigen::Matrix<double, 3, 1> mocap_baseVel;
    Eigen::Matrix<double, 3, 1> mocap_baseOmega;
    Eigen::Matrix<double, 3, 1> mocap_shuttlecock_pos;

    int count = 0;
    int count1 = 0;
    int count2 = 0;
    int count3 = 0;
    int pre_num = 0;
    int pre_num1 = 0;
    int pre_num2 = 0;
    bool hitflag = false;
    bool hittrueflag = false;
    bool hittrueflag1 = false;
    double truex1;
    double truey1;
    double truez1;
    double truet1;
    double truex;
    double truey;
    double truez;
    double truet;


    std::vector<lusternet::LST_RIGID_DATA> FrameRigidBody;
    std::vector<lusternet::LST_BODY_DATA> FrameSkeleton;
    std::vector<lusternet::LST_MarkerINFO> Frame3DMarker;

    struct MarkerPos{
      std::string name;
      Eigen::Matrix<double, 3, 1> pos;
    };
    MarkerPos get_mocap_marker_pos(std::string marker_name);

    struct RigidBodyPose {
        std::string name;
        Eigen::Matrix<double, 3, 1> pos;
        Eigen::Matrix<double, 4, 1> quat;
    };
    RigidBodyPose get_mocap_rigid_body_pose(std::string rigid_body_name);

    struct RigidBodyTwist {
        std::string name;
        Eigen::Matrix<double, 3, 1> linear;
        Eigen::Matrix<double, 3, 1> angular;
    };
    RigidBodyTwist get_mocap_rigid_body_twist(std::string rigid_body_name);

    RigidBodyPose last_base_link_pose;
    RigidBodyTwist last_base_link_twist;
    MarkerPos last_shuttlecock_pos;

  private:
    int mocap_init();
    void mocap_run(int start);

    uint32_t FrameID;
    double TimeStamp;

    std::shared_ptr<lusternet::CReceiveBase> LusterServer;
    lusternet::LusterMocapData MocapData;
    std::thread mocap_thread;
    bool bExit = false;
};

// 轨迹数据结构
struct TrajectoryData {
    std::vector<double> times;
    std::vector<double> x_traj;
    std::vector<double> y_traj;
    std::vector<double> z_traj;
    double dt;
};
// 预测结果结构
struct PredictionResult {
    Eigen::Vector3d hit_position;
    double hit_time;
    Eigen::Vector3d hit_velocity;
    Eigen::Vector4d hit_quaternion;
    bool in_hit_zone;
    double prediction_start_time;
    double total_trajectory_time;
    bool valid;
    
    // 误差信息
    bool has_error_analysis;
    Eigen::Vector3d actual_position_at_hit_time;
    Eigen::Vector3d position_error;
    double position_error_3d;
};
Eigen::Vector4d computeQuaternion(const Eigen::Vector3d& vel);
PredictionResult predictHitPoint(TrajectoryData data,
                      double target_z = 1.54,
                      const std::vector<double>& hit_zone = {-10.8, 10.8, -10.0, 10.2},
                      double use_fraction = 1);