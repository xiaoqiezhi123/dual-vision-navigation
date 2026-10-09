#ifndef RL_SIM_HPP
#define RL_SIM_HPP

#include "rl_sdk.hpp"
#include "loop.hpp"
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>
#include "std_srvs/Empty.h"
#include <geometry_msgs/Twist.h>
// #include "phybot_msgs/MotorCommand.h"
#include <csignal>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <std_msgs/Float64.h>


class RL_Sim : public RL
{
public:
    RL_Sim();
    ~RL_Sim();
private:
    // rl functions
    torch::Tensor Forward() override;
    torch::Tensor ComputeObservation() override;
    void GetState(RobotState<double> *state) override;
    void SetCommand(const RobotCommand<double> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;

    // ros interface
    std::string ros_namespace;
    geometry_msgs::Twist vel;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber model_state_subscriber;
    ros::Subscriber joint_state_subscriber;
    ros::Subscriber cmd_vel_subscriber;
    ros::Subscriber height_map_subscriber;
    ros::ServiceClient gazebo_set_model_state_client;
    ros::ServiceClient gazebo_set_joint_state_client;
    ros::ServiceClient gazebo_pause_physics_client;
    ros::ServiceClient gazebo_unpause_physics_client;
    std::map<std::string, ros::Publisher> joint_publishers;
    // std::vector<phybot_msgs::MotorCommand> joint_publishers_commands;
    void ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void JointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);


    // others
    std::map<std::string, size_t> sorted_to_original_index;
    std::vector<double> mapped_joint_positions;
    std::vector<double> mapped_joint_velocities;
    std::vector<double> mapped_joint_efforts;
    // std::vector<std::vector<float>> height_map(15, std::vector<float>(15));
    std::vector<std::vector<float>> previous_matrix;
    void MapData(const std::vector<double>& source_data, std::vector<double>& target_data);
};

#endif