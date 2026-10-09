#include "RL_deploy_amp/include/rl_deploy.h"
#include <cmath>  // 包含 sin、cos 等数学函数


rl_deploy_amp::rl_deploy_amp()
{
    torch::autograd::GradMode::set_enabled(false);
    std::string file_path = "../RL_deploy_amp/config/rl_params.yaml";
    YAML::Node config = YAML::LoadFile(file_path);
    // load the policy
    policy_path = config["policy_path"].as<std::string>();
    policy = torch::jit::load(policy_path, torch::kCPU);
    std::cout << "valinia policy init complete" << std::endl;


    // read rl params from yaml
    decimation = config["decimation"].as<int>();
    num_proprio = config["num_proprio"].as<int>();

    clip_obs = config["clip_obs"].as<double>();
    clip_actions_lower = config["clip_actions_lower"].as<double>();
    clip_actions_upper = config["clip_actions_upper"].as<double>();
    // action_scale = config["action_scale"].as<double>();
    num_of_dofs = config["num_of_dofs"].as<int>();
    history_len = config["history_len"].as<int>();
    lin_vel_scale = config["lin_vel_scale"].as<double>();
    ang_vel_scale = config["ang_vel_scale"].as<double>();
    dof_pos_scale = config["dof_pos_scale"].as<double>();
    dof_vel_scale = config["dof_vel_scale"].as<double>();

    last_actions_eigen = Eigen::VectorXd::Zero(num_of_dofs);
    outputdata_eigen = Eigen::VectorXd::Zero(num_of_dofs);
    pos_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);
    vel_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);
    action_hist_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);
    rot_hist_buf_eigen = Eigen::VectorXd::Zero(15);
    ang_vel_hist_buf_eigen = Eigen::VectorXd::Zero(15);


    // Convert std::vector<double> to Eigen::VectorXd
    std::vector<double> default_dof_pos_vec = ReadVectorFromYaml<double>(config["default_dof_pos"]);
    default_dof_pos_eigen = Eigen::Map<Eigen::VectorXd>(default_dof_pos_vec.data(), default_dof_pos_vec.size());
    
    std::vector<double> rl_p_vec = ReadVectorFromYaml<double>(config["rl_p"]);
    rl_p = Eigen::Map<Eigen::VectorXd>(rl_p_vec.data(), rl_p_vec.size());
    std::vector<double> rl_d_vec = ReadVectorFromYaml<double>(config["rl_d"]);
    rl_d = Eigen::Map<Eigen::VectorXd>(rl_d_vec.data(), rl_d_vec.size());


    std::vector<double> action_scale_vec = ReadVectorFromYaml<double>(config["action_scale"]);
    action_scale = Eigen::Map<Eigen::VectorXd>(action_scale_vec.data(), action_scale_vec.size());


    indices_to_remove = ReadVectorFromYaml<int>(config["useless_joint_id"]);



    lin_vel = Eigen::VectorXd::Zero(3);
    imu_angular_vel = Eigen::VectorXd::Zero(3);
    commands = Eigen::VectorXd::Zero(3);
    // task_commands = Eigen::VectorXd::Zero(num_task_commands);
    base_euler = Eigen::VectorXd::Zero(3);
    dof_pos = Eigen::VectorXd::Zero(num_of_dofs);
    dof_vel = Eigen::VectorXd::Zero(num_of_dofs);
    last_actions = Eigen::VectorXd::Zero(num_of_dofs);

    torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"])).view({1, -1});
    default_dof_pos = torch::tensor(ReadVectorFromYaml<double>(config["default_dof_pos"])).view({1, -1});
    commands_scale = torch::tensor({lin_vel_scale, lin_vel_scale, ang_vel_scale});
    output_dof_pos = torch::zeros({1, num_of_dofs});
    inputdata_eigen = Eigen::VectorXd::Zero(num_observations);

    num_observations = num_proprio + num_of_dofs * 5 *2  ;
    inputdata_eigen = Eigen::VectorXd::Zero(num_observations);



    // std::cout<<"ccccccccccccccc"<<std::endl;
}



void rl_deploy_amp::Exit()
{
    // clear the hist buf and the step counter
    pos_hist_buf_eigen.setZero();
    vel_hist_buf_eigen.setZero();

}



rl_deploy_amp::~rl_deploy_amp() {
}


void rl_deploy_amp::GetDataFromPackage(DataPackage &DataPackage){

    // js_vx_desire = DataPackage.js_vx_desire;

    imu_angular_vel = DataPackage.imu_angular_vel;
    gravity_vec_eigen = Euler_ZYXToGravityVec(DataPackage.imu_zyx);

    q_origin = DataPackage.motor_pos;




    dot_q_origin = DataPackage.motor_vel;

    
    // dot_q = DataPackage.motor_vel.data();

    js_vx_desire = DataPackage.js_vx_desire;
    js_vy_desire = DataPackage.js_vy_desire;
    js_OmegaZ_desire = DataPackage.js_OmegaZ_desire;
    std::cout<<"js_OmegaZ_desire: "<<js_OmegaZ_desire<<std::endl;

    // vx_Final = DataPackage.vx_Final;
}


void rl_deploy_amp::SetDataToPackage(DataPackage &data)
{

    data.torq_desire.setZero() ;
    // //******************************for real robot**************************************** */
    // std::cout<<action_scale<<std::endl;
// Eigen::VectorXd dof_pos_desire;
    for(int i =0; i<num_of_dofs; i++)
    {
        data.motor_Pos_desire(i) = outputdata_eigen[i] * action_scale[i] + default_dof_pos_eigen[i];
    }
    // Eigen::VectorXd dof_pos_desire = outputdata_eigen * action_scale + default_dof_pos_eigen;
    // assign_with_skipped_zero(data.motor_Pos_desire, dof_pos_desire, indices_to_remove);
    // data.motor_Pos_desire = dof_pos_desire;
    data.motor_Vel_desire.setZero();
    data.motor_Torque_desire.setZero();

    data.sim_P = rl_p*1.0;
    data.sim_D = rl_d*1.0;
}




void rl_deploy_amp::Step()
{
    if (step_num == 0){
        step_num = 1;
    }
    else{
        step_num += 1;
    }
    // refresh imu reading
    // set the input data
    if(step_num % decimation == 0)
    {   

        // commands = Eigen::VectorXd::Zero(num_commands);
        commands(0) = js_vx_desire * lin_vel_scale;
        commands(1) = js_vy_desire * lin_vel_scale;
        commands(2) = js_OmegaZ_desire * ang_vel_scale;



        int num_rows = commands.rows();
        
        // 计算目标行前3列的L2范数（作为单独变量）
        double norm3 = commands.norm();
        
        // 根据norm3计算周期（单一double变量）
        double period = (norm3 > 1.0f) ? 0.9 : 0.9;
        
        const double offset = 0.5;  // 偏移量（double类型）
        


        // 计算所有行的相位
        

        if (norm3 > 0.2) {
            // 计算 (episode_length_buf * dt) % period / period
            double val = episode_length * 0.02;
            double remainder = std::fmod(val, period);
            phase = remainder / period;
        } else {
            phase = 0.0;
        }
        

        // std::cout<<"1: "<<phase<<std::endl;
        // std::cout<<"2: "<<gravity_vec_eigen<<std::endl;


        inputdata_eigen.segment(0, 3) =  imu_angular_vel * ang_vel_scale ;     
        inputdata_eigen.segment(3, 3) =  gravity_vec_eigen ;
        inputdata_eigen.segment(6, 3) = commands;
        inputdata_eigen.segment(9, num_of_dofs) = (q_origin - default_dof_pos_eigen) * dof_pos_scale;
        inputdata_eigen.segment(9 + num_of_dofs, num_of_dofs) = dot_q_origin * dof_vel_scale ;
        inputdata_eigen.segment(9 + 2*num_of_dofs, num_of_dofs) = last_actions_eigen;
        inputdata_eigen(72) =  std::sin(2 * 3.1415 * phase);
        inputdata_eigen(73) =   std::cos(2 * 3.1415 * phase);

        // short history obs inputdata_nlp
        inputdata_eigen.segment(11 + 3 * num_of_dofs, 5 * num_of_dofs) = pos_hist_buf_eigen.tail(5 * num_of_dofs);
        inputdata_eigen.segment(11 + 3 * num_of_dofs +  5 * num_of_dofs, 5 * num_of_dofs) = vel_hist_buf_eigen.tail(5 * num_of_dofs);


        // inputdata_eigen.segment(11 + 3 * num_of_dofs +  5 * num_of_dofs*2, 15) = rot_hist_buf_eigen *0;
        // inputdata_eigen.segment(11 + 3 * num_of_dofs +  5 * num_of_dofs*2 + 15, 15) = ang_vel_hist_buf_eigen * 0;


        
        // // long history obs inputdata_nlp
        // inputdata_eigen.segment(11 + 3 * num_of_dofs +  5 * num_of_dofs * 2, history_len * num_of_dofs) = pos_hist_buf_eigen;
        // inputdata_eigen.segment(11 + 3 * num_of_dofs +  5 * num_of_dofs * 2 + history_len * num_of_dofs , history_len * num_of_dofs) = vel_hist_buf_eigen;
        // inputdata_eigen.segment(11 + 3 * num_of_dofs +  5 * num_of_dofs * 2 + history_len * num_of_dofs * 2, history_len * num_of_dofs) = action_hist_buf_eigen;





        // torch::Device inference_device(torch::kCPU);
        std::vector<torch::jit::IValue> inputs;
        torch::Tensor inputdata_tensor = torch::zeros({num_observations}).toType(torch::kFloat);
        // Copy data safely without using copytoinputs
        auto input_accessor = inputdata_tensor.accessor<float, 1>();
        for (int i = 0; i < num_observations; i++) {
            input_accessor[i] = static_cast<float>(inputdata_eigen(i));
        }
        inputdata_tensor = inputdata_tensor.to(torch::kCPU);
        inputdata_tensor = inputdata_tensor.unsqueeze(0);
        inputs.push_back(inputdata_tensor);

        
        torch::Tensor output_tensor;

        output_tensor = policy.forward(inputs).toTensor();

        // std::vector<double> temp_vec(action_scale.data(), action_scale.data() + action_scale.size());
    
        // // 2. 从std::vector创建libtorch Tensor
        // torch::Tensor action_scale_tensor = torch::tensor(temp_vec, torch::kDouble);

        // std::cout<<action_scale_tensor<<std::endl;
        
        // output_dof_pos = output_tensor * action_scale_tensor + default_dof_pos;
        



        // Remove batch dimension if present: (1, action_num) -> (action_num)
        if (output_tensor.dim() > 1 && output_tensor.size(0) == 1){
            output_tensor = output_tensor.squeeze(0);
        }

        // Safe direct access without creating vector
        auto accessor = output_tensor.accessor<float, 1>();
        for (int i = 0; i < num_of_dofs; i++) {
            outputdata_eigen(i) = static_cast<double>(accessor[i]);
        }
        


        last_actions_eigen = outputdata_eigen;


        // write the history obs inputdata_nlp
        // write the history obs inputdata_nlp
        pos_hist_buf_eigen.head(pos_hist_buf_eigen.size() - num_of_dofs) = pos_hist_buf_eigen.tail(pos_hist_buf_eigen.size() - num_of_dofs);
        pos_hist_buf_eigen.tail(num_of_dofs) = (q_origin - default_dof_pos_eigen) * dof_pos_scale;
        
        vel_hist_buf_eigen.head(vel_hist_buf_eigen.size() - num_of_dofs) = vel_hist_buf_eigen.tail(vel_hist_buf_eigen.size() - num_of_dofs);
        vel_hist_buf_eigen.tail(num_of_dofs) = dot_q_origin * dof_vel_scale;

        action_hist_buf_eigen.head(action_hist_buf_eigen.size() - num_of_dofs) = action_hist_buf_eigen.tail(action_hist_buf_eigen.size() - num_of_dofs);
        action_hist_buf_eigen.tail(num_of_dofs) = last_actions_eigen;



        rot_hist_buf_eigen.head(12) = rot_hist_buf_eigen.tail(12);
        rot_hist_buf_eigen.tail(3) = gravity_vec_eigen;

        ang_vel_hist_buf_eigen.head(12) = ang_vel_hist_buf_eigen.tail(12);
        ang_vel_hist_buf_eigen.tail(3) = imu_angular_vel * ang_vel_scale;

        episode_length ++;

    }
}


Eigen::VectorXd rl_deploy_amp::remove_indices(const Eigen::VectorXd& input, const std::vector<int>& indices_to_remove) 
{
    int original_size = input.size();

    // 构建保留的索引列表
    std::vector<int> keep_indices;
    for (int i = 0; i < original_size; ++i) {
        if (std::find(indices_to_remove.begin(), indices_to_remove.end(), i) == indices_to_remove.end()) {
            keep_indices.push_back(i);
        }
    }
    // 构造新向量
    Eigen::VectorXd output(keep_indices.size());
    for (size_t i = 0; i < keep_indices.size(); ++i) {
        output[i] = input[keep_indices[i]];
    }

    return output;
}


Eigen::Vector3d rl_deploy_amp::Euler_ZYXToGravityVec(Eigen::Vector3d euler_a) {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Vector3d grav_vec = -R.transpose().col(2);
    return grav_vec;
}


void rl_deploy_amp::assign_with_skipped_zero(
    Eigen::VectorXd& target,
    const Eigen::VectorXd& source,
    const std::vector<int>& indices_to_skip)
{
    int target_size = target.size();
    int source_size = source.size();

    std::unordered_set<int> skip_set(indices_to_skip.begin(), indices_to_skip.end());

    int source_idx = 0;

    for (int i = 0; i < target_size; ++i) {
        if (skip_set.count(i)) {
            target[i] = 0.0;  // 跳过位置赋0
        } else {
            if (source_idx < source_size) {
                target[i] = source[source_idx++];
            } else {
                target[i] = 0.0;  // 如果 source 不够长，剩余位置补0（可选）
            }
        }
    }
    // 可选：保证 source 完全用完
    assert(source_idx == source_size && "Source vector is longer than available non-skipped slots");
}


// Task observation logging functions
void rl_deploy_amp::InitTaskObsLogging(const std::string& log_file_path) {
    try {
        // Create logs directory if it doesn't exist
        std::string dir_path = "../RL_deploy/logs/";
        system(("mkdir -p " + dir_path).c_str());

        task_obs_log_file.open(log_file_path, std::ios::out | std::ios::trunc);
        if (!task_obs_log_file.is_open()) {
            std::cerr << "Failed to open task_obs log file: " << log_file_path << std::endl;
            enable_task_obs_logging = false;
            return;
        }

        // Write CSV header
        task_obs_log_file << "step_num,frame_count,control_mode";
        for (int i = 0; i < num_observations; ++i) {
            task_obs_log_file << ",proprio_obs_" << i;
        }
        task_obs_log_file << std::endl;

        std::cout << "Task observation logging initialized: " << log_file_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing task_obs logging: " << e.what() << std::endl;
        enable_task_obs_logging = false;
    }
}


void rl_deploy_amp::LogTaskObs(const torch::Tensor& task_obs_tensor) {
    if (!enable_task_obs_logging || !task_obs_log_file.is_open()) {
        return;
    }

    try {
        // Convert tensor to CPU and double precision for logging
        torch::Tensor task_obs_cpu = task_obs_tensor.to(torch::kCPU).to(torch::kDouble);

        // Write step info
        // task_obs_log_file << step_num << "," << frame_count << "," << control_mode;

        // Write task_obs values
        auto accessor = task_obs_cpu.accessor<double, 2>();
        for (int i = 0; i < num_observations; ++i) {
            task_obs_log_file << " " << std::fixed << std::setprecision(6) << accessor[0][i];
        }
        task_obs_log_file << std::endl;

        // Flush every 100 steps to ensure data is written
        if (step_num % 100 == 0) {
            task_obs_log_file.flush();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error logging task_obs: " << e.what() << std::endl;
    }
}


void rl_deploy_amp::CloseTaskObsLogging() {
    if (task_obs_log_file.is_open()) {
        task_obs_log_file.close();
        std::cout << "Task observation logging closed." << std::endl;
    }
}


Eigen::MatrixXd rl_deploy_amp::LoadMotionLib(const std::string& filename, bool skipHeader) {
    std::ifstream file(filename);
    std::string line;
    std::vector<std::vector<double>> data;

    // read w/o process
    if (skipHeader) {
        std::getline(file, line);
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        while (std::getline(ss, value, ',')) {
            try {
                row.push_back(std::stod(value));
            } catch (...) {
                row.push_back(0.0);  // 可以自定义处理方式
            }
        }

        if (!row.empty())
            data.push_back(row);
    }

    if (data.empty()) {
        throw std::runtime_error("No data found in file: " + filename);
    }

    int rows = data.size();
    int cols = data[0].size();
    num_motion_frames = rows;
    Eigen::MatrixXd mat(rows, cols);

    for (int i = 0; i < rows; ++i)
        mat.row(i) = Eigen::VectorXd::Map(&data[i][0], cols);
    std::cout << "motion lib loaded !!!" << std::endl;

    return mat;
}
