#include "../include/rl_sim.hpp"
#include "qpOASES.hpp"

RL_Sim::RL_Sim()
{
    ros::NodeHandle nh;

    // read params from yaml
    nh.param<std::string>("robot_name", this->robot_name, "");
    this->ReadYaml(this->robot_name);

    // Due to the fact that the robot_state_publisher sorts the joint names alphabetically,
    // the mapping table is established according to the order defined in the YAML file
    std::vector<std::string> sorted_joint_controller_names = this->params.joint_controller_names;
    std::sort(sorted_joint_controller_names.begin(), sorted_joint_controller_names.end());
    for(size_t i = 0; i < this->params.joint_controller_names.size(); ++i)
    {
        this->sorted_to_original_index[sorted_joint_controller_names[i]] = i;
    }
    this->mapped_joint_positions = std::vector<double>(this->params.num_of_dofs, 0.0);
    this->mapped_joint_velocities = std::vector<double>(this->params.num_of_dofs, 0.0);
    this->mapped_joint_efforts = std::vector<double>(this->params.num_of_dofs, 0.0);

    // init
    torch::autograd::GradMode::set_enabled(false);
    // this->joint_publishers_commands.resize(this->params.num_of_dofs);
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();

    // model
    std::string stand_model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + this->robot_name + "/" + this->params.stand_model_name;
    this->stand_model = torch::jit::load(stand_model_path);
    std::string walk_model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + this->robot_name + "/" + this->params.walk_model_name;
    this->walk_model = torch::jit::load(walk_model_path);
    this->model = this->walk_model; // default to walk model

    // publisher
    nh.param<std::string>("ros_namespace", this->ros_namespace, "");
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->joint_publishers[this->params.joint_controller_names[i]] = nh.advertise<std_msgs::Float64>(
            this->ros_namespace + this->params.joint_controller_names[i] + "/command", 2);
    }

    // subscriber
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 2, &RL_Sim::CmdvelCallback, this);
    this->model_state_subscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 2, &RL_Sim::ModelStatesCallback, this);
    this->joint_state_subscriber = nh.subscribe<sensor_msgs::JointState>(this->ros_namespace + "joint_states", 2, &RL_Sim::JointStatesCallback, this);
    this->height_map_subscriber = nh.subscribe<grid_map_msgs::GridMap>("/elevation_mapping/elevation_map_filter",1, &RL_Sim::gridMapCallback, this);

    // service
    this->gazebo_set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    this-> gazebo_set_joint_state_client = nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

    this->gazebo_pause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    this->gazebo_unpause_physics_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    // loops
    this->loop_control  = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;
}

RL_Sim::~RL_Sim()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::GetState(RobotState<double> *state)
{
    state->imu.quaternion[3] = this->pose.orientation.w;
    state->imu.quaternion[0] = this->pose.orientation.x;
    state->imu.quaternion[1] = this->pose.orientation.y;
    state->imu.quaternion[2] = this->pose.orientation.z;

    state->imu.gyroscope[0] = this->vel.angular.x;
    state->imu.gyroscope[1] = this->vel.angular.y;
    state->imu.gyroscope[2] = this->vel.angular.z;

    // state->imu.accelerometer

    for(int i = 0; i < this->params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = this->mapped_joint_positions[i];
        state->motor_state.dq[i] = this->mapped_joint_velocities[i];
        state->motor_state.tauEst[i] = this->mapped_joint_efforts[i];
    }
}

void RL_Sim::SetCommand(const RobotCommand<double> *command)
{
    // for(int i = 0; i < this->params.num_of_dofs; ++i)
    // {
    //     this->joint_publishers_commands[i].q = command->motor_command.q[i];
    //     this->joint_publishers_commands[i].dq = command->motor_command.dq[i];
    //     this->joint_publishers_commands[i].kp = command->motor_command.kp[i];
    //     this->joint_publishers_commands[i].kd = command->motor_command.kd[i];
    //     this->joint_publishers_commands[i].tau = command->motor_command.tau[i];
    // }
    
    // for(int i = 0; i < this->params.num_of_dofs; ++i)
    // {
    //     this->joint_publishers[this->params.joint_controller_names[i]].publish(this->joint_publishers_commands[i]);
    // }


    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
      std_msgs::Float64 msg;
      msg.data = command->motor_command.q[i]; // 示例：每个关节的目标位置不同
      this->joint_publishers[this->params.joint_controller_names[i]].publish(msg);
    }
}

void RL_Sim::RobotControl()
{
    if(this->control.control_state == STATE_RESET_SIMULATION)
    {
        gazebo_msgs::SetModelState set_model_state;
        std::string gazebo_model_name = this->robot_name + "_gazebo";
        set_model_state.request.model_state.model_name = gazebo_model_name;
        set_model_state.request.model_state.pose.position.z = 1.0;
        set_model_state.request.model_state.pose.position.x = -10.0;
        
        set_model_state.request.model_state.reference_frame = "world";

 

        this->gazebo_set_model_state_client.call(set_model_state);

        this->control.control_state = STATE_WAITING;
    }
    if(this->control.control_state == STATE_TOGGLE_SIMULATION)
    {
        std_srvs::Empty empty;
        if(simulation_running)
        {
            this->gazebo_pause_physics_client.call(empty);
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            this->gazebo_unpause_physics_client.call(empty);
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
        this->control.control_state = STATE_WAITING;
    }

    if(simulation_running)
    {
        this->GetState(&this->robot_state);
        this->StateController(&this->robot_state, &this->robot_command);
        this->SetCommand(&this->robot_command);
    }
}

void RL_Sim::ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    this->vel = msg->twist[2];
    this->pose = msg->pose[2];
    // std::cout<<"this->pose: "<<this->pose.position.z<<std::endl;
}

void RL_Sim::CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    this->cmd_vel = *msg;
}



void RL_Sim::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
    if (msg->data.empty()) {
        ROS_WARN("GridMap message has no data.");
        return;
    }

  // 找 elevation 层索引
  int elevation_index = -1;
  for (size_t i = 0; i < msg->layers.size(); ++i) {
    if (msg->layers[i] == "min_filter") {
      elevation_index = static_cast<int>(i);
      break;
    }
  }

  if (elevation_index == -1) {
    ROS_WARN("No elevation layer found!");
    return;
  }

  // 获取 elevation 层数据
  const std_msgs::Float32MultiArray& elevation_layer = msg->data[elevation_index];

//   ROS_INFO_STREAM("Elevation layer data size: " << elevation_layer.data.size());

//   // 打印所有数据
//   for (size_t i = 0; i < elevation_layer.data.size(); ++i) {
//     float v = elevation_layer.data[i];
//     if (std::isnan(v)) {
//       ROS_INFO_STREAM("Index " << i << ": nan");
//     } else {
//       ROS_INFO_STREAM("Index " << i << ": " << v);
//     }
//   }



    std::vector<std::vector<float>> matrix(25, std::vector<float>(25));

    for (size_t col = 0; col < 25; ++col) {
        for (size_t row = 0; row < 25; ++row) {
            size_t index = col * 25 + row;  // 列优先读取
            float value = elevation_layer.data[index] +0.0;
            matrix[row][col] = std::isnan(value) ? 0.0f : value;  // 直接放入对应位置（从左上角开始）
        }
    }

    std::vector<float> flattened_matrix;

    for (int row = 24; row >= 0; --row) {
        for (int col = 24; col >= 0; --col) {
            flattened_matrix.push_back(matrix[row][col]);
        }
    }


    this->obs.heights = torch::from_blob(
        flattened_matrix.data(), 
        {1, this->params.num_of_heights}, 
        torch::TensorOptions().dtype(torch::kFloat32)).clone();
}

void RL_Sim::MapData(const std::vector<double>& source_data, std::vector<double>& target_data)
{
    for(size_t i = 0; i < source_data.size(); ++i)
    {
        target_data[i] = source_data[this->sorted_to_original_index[this->params.joint_controller_names[i]]];

    }
}

void RL_Sim::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    this->MapData(msg->position, this->mapped_joint_positions);
    this->MapData(msg->velocity, this->mapped_joint_velocities);
    this->MapData(msg->effort, this->mapped_joint_efforts);
}

void RL_Sim::RunModel()
{
    if(running_state == STATE_RL_RUNNING && simulation_running)
    {
        this->obs.lin_vel = torch::tensor({{this->vel.linear.x, this->vel.linear.y, this->vel.linear.z}});
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        // this->obs.commands = torch::tensor({{this->cmd_vel.linear.x, this->cmd_vel.linear.y, this->cmd_vel.angular.z}});
        this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw, 1.0}});
        this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);


        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);

        torch::Tensor clamped_actions = this->Forward();

        this->obs.actions = clamped_actions;

        torch::Tensor origin_output_torques = this->ComputeTorques(this->obs.actions);

        // this->TorqueProtect(origin_output_torques);

        this->output_torques = torch::clamp(origin_output_torques, -(this->params.torque_limits), this->params.torque_limits);
        this->output_dof_pos = this->ComputePosition(this->obs.actions);
        // this->obs.history_pos_buf = torch.cat((this->obs.history_pos_buf[1, self.cfg.env.num_dof: ],(self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos),dim = -1)
        torch::Tensor history_vel_trimmed = this->obs.history_vel_buf.index({torch::indexing::Slice(), torch::indexing::Slice(this->params.num_of_dofs, torch::indexing::None)});

        // 2. 当前帧做缩放
        torch::Tensor current_vel_scaled = this->obs.dof_vel * this->params.dof_vel_scale;

        // 3. 拼接更新
        this->obs.history_vel_buf = torch::cat({history_vel_trimmed, current_vel_scaled}, 1);


        torch::Tensor history_pos_trimmed = this->obs.history_pos_buf.index({torch::indexing::Slice(), torch::indexing::Slice(this->params.num_of_dofs, torch::indexing::None)});

        // 2. 当前帧做缩放
        torch::Tensor current_pos_scaled =(this->obs.dof_pos  - this->params.default_dof_pos)   * this->params.dof_pos_scale;

        // 3. 拼接更新
        this->obs.history_pos_buf = torch::cat({history_pos_trimmed, current_pos_scaled}, 1);


        torch::Tensor history_actions_trimmed = this->obs.history_actions_buf.index({torch::indexing::Slice(), torch::indexing::Slice(this->params.num_of_dofs, torch::indexing::None)});



        // 3. 拼接更新
        this->obs.history_actions_buf = torch::cat({history_actions_trimmed, this->obs.actions}, 1);

    }
}

torch::Tensor RL_Sim::ComputeObservation()
{

    int64_t end_dim = this->obs.history_pos_buf.size(1);  // 假设我们在 dim=1 上切
    int64_t start_dim = end_dim - this->params.num_of_dofs* 10;


    // this->obs.heights = torch::clamp(this->obs.heights, /*min=*/-1.0, /*max=*/1.0);
    torch::Tensor pos_history_buf_short = this->obs.history_pos_buf.index({torch::indexing::Slice(), torch::indexing::Slice(start_dim, torch::indexing::None)});
    torch::Tensor vel_history_buf_short = this->obs.history_vel_buf.index({torch::indexing::Slice(), torch::indexing::Slice(start_dim, torch::indexing::None)});
    torch::Tensor actions_history_buf_short = this->obs.history_actions_buf.index({torch::indexing::Slice(), torch::indexing::Slice(start_dim, torch::indexing::None)});
    torch::Tensor obs = torch::cat({
        // this->QuatRotateInverse(this->obs.base_quat, this->obs.lin_vel)  * this->params.lin_vel_scale,
        // this->obs.ang_vel * this->params.ang_vel_scale, // TODO is QuatRotateInverse necessery?
        this->QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.ang_vel_scale,
        this->QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec),
        this->obs.commands * this->params.commands_scale,
        (this->obs.dof_pos - this->params.default_dof_pos) * this->params.dof_pos_scale,
        this->obs.dof_vel * this->params.dof_vel_scale,
        this->obs.actions,
        pos_history_buf_short,
        vel_history_buf_short,
        // actions_history_buf_short,
        this->obs.history_pos_buf,
        this->obs.history_vel_buf,
        this->obs.history_actions_buf,
        this->pose.position.z -0.9 -this->obs.heights ,
        },1);
    // auto flat_tensor = this->obs.heights.view(-1);  // 展平为1维

    // std::cout << "Heights (last 15 values): "<<this->obs.heights.index({0, torch::indexing::Slice(-25, torch::indexing::None)});

        // torch::PrintOptions options;
        // options.precision(6);         // 小数精度
        // options.threshold(1000);      // 超过这个值就省略打印
        // options.edgeitems(25);        // 打印边缘项个数
        // options.sci_mode(false);    
        // std::cout << this->obs.heights << std::endl;

        // std::cout<<"vel: "<<this->QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec)<<std::endl;
    torch::Tensor clamped_obs = torch::clamp(obs, -this->params.clip_obs, this->params.clip_obs);
    return clamped_obs;
}

torch::Tensor RL_Sim::Forward()
{
    torch::autograd::GradMode::set_enabled(false);
    ros::Time t1 = ros::Time::now();

    torch::Tensor clamped_obs = this->ComputeObservation();
    // clamped_obs.fill_(1);
    torch::Tensor actions = this->model.forward({clamped_obs}).toTensor();

    ros::Time t2 = ros::Time::now();

    ros::Duration dt = t2 - t1;
    double milliseconds = dt.toSec() * 1000.0;
    // ROS_INFO_STREAM("Duration: " << milliseconds << " ms");
    torch::Tensor clamped_actions = torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);


    return clamped_actions;
}

void signalHandler(int signum)
{
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "rl_sim");
    RL_Sim rl_sim;
    ros::spin();
    return 0;
}
