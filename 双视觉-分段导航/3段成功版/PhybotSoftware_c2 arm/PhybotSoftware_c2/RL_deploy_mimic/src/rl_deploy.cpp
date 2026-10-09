#include "RL_deploy_mimic/include/rl_deploy.h"



rl_deploy_mimic::rl_deploy_mimic()
{
    torch::autograd::GradMode::set_enabled(false);
    std::string file_path = "../RL_deploy_mimic/config/rl_params.yaml";
    YAML::Node config = YAML::LoadFile(file_path);
    // load the policy
    mimic_policy_path = config["mimic_policy_path"].as<std::string>();
    mimic_policy = torch::jit::load(mimic_policy_path, torch::kCPU);
    std::cout << "mimic policy init complete" << std::endl;
    
    // read offline motion library
    motion_lib_path = config["motion_lib_path"].as<std::string>();
    motion_frames = LoadMotionLib(motion_lib_path, false);

    // read rl params from yaml
    decimation = config["decimation"].as<int>();
    num_proprio_mimic = config["num_proprio_mimic"].as<int>();
    num_mimic_task_obs = config["num_task_obs"].as<int>();
    num_mimic_commands = config["num_mimic_commands"].as<int>();
    clip_obs = config["clip_obs"].as<double>();
    clip_actions_lower = config["clip_actions_lower"].as<double>();
    clip_actions_upper = config["clip_actions_upper"].as<double>();
    action_scale = config["action_scale"].as<double>();
    num_of_dofs = config["num_of_dofs"].as<int>();
    history_len = config["history_len"].as<int>();
    lin_vel_scale = config["lin_vel_scale"].as<double>();
    ang_vel_scale = config["ang_vel_scale"].as<double>();
    dof_pos_scale = config["dof_pos_scale"].as<double>();
    dof_vel_scale = config["dof_vel_scale"].as<double>();

    last_actions_eigen = Eigen::VectorXd::Zero(num_of_dofs);
    outputdata_eigen = Eigen::VectorXd::Zero(num_of_dofs);
    hist_mimic_obs_buf_eigen = Eigen::VectorXd::Zero(history_len * num_proprio_mimic);
    // history_pos_buf_eigen = Eigen::VectorXd::Zero(history_len * num_of_dofs);
    // history_vel_buf = Eigen::VectorXd::Zero(history_len * num_of_dofs);
    // history_actions_buf = Eigen::VectorXd::Zero(history_len * num_of_dofs);

    // Convert std::vector<double> to Eigen::VectorXd
    std::vector<double> default_dof_pos_vec = ReadVectorFromYaml<double>(config["default_dof_pos"]);
    default_dof_pos_eigen = Eigen::Map<Eigen::VectorXd>(default_dof_pos_vec.data(), default_dof_pos_vec.size());
    
    std::vector<double> rl_p_vec = ReadVectorFromYaml<double>(config["rl_p"]);
    rl_p = Eigen::Map<Eigen::VectorXd>(rl_p_vec.data(), rl_p_vec.size());
    std::vector<double> rl_d_vec = ReadVectorFromYaml<double>(config["rl_d"]);
    rl_d = Eigen::Map<Eigen::VectorXd>(rl_d_vec.data(), rl_d_vec.size());

    indices_to_remove = ReadVectorFromYaml<int>(config["useless_joint_id"]);

    lin_vel = Eigen::VectorXd::Zero(3);
    last_lin_vel = Eigen::VectorXd::Zero(3);
    imu_ang_vel = Eigen::VectorXd::Zero(3);
    // commands = Eigen::VectorXd::Zero(3);
    // task_commands = Eigen::VectorXd::Zero(num_task_commands);
    base_euler = Eigen::VectorXd::Zero(3);
    base_quat = Eigen::VectorXd::Zero(4);



    q_origin = Eigen::VectorXd::Zero(num_of_dofs);
    dot_q_origin = Eigen::VectorXd::Zero(num_of_dofs);
    last_actions = Eigen::VectorXd::Zero(num_of_dofs);
    task_obs = Eigen::VectorXd::Zero(num_mimic_task_obs);
    
    torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"])).view({1, -1});
    default_dof_pos = torch::tensor(ReadVectorFromYaml<double>(config["default_dof_pos"])).view({1, -1});
    commands_scale = torch::tensor({lin_vel_scale, lin_vel_scale, ang_vel_scale});
    output_dof_pos = torch::zeros({1, num_of_dofs});

}


rl_deploy_mimic::~rl_deploy_mimic() {}


void rl_deploy_mimic::GetDataFromPackage(DataPackage &DataPackage){
    gravity_vec_eigen = Euler_ZYXToGravityVec(DataPackage.imu_zyx);
    imu_ang_vel = DataPackage.imu_angular_vel;

    imu_euler_zyx.assign(DataPackage.imu_zyx.data(), DataPackage.imu_zyx.data() + DataPackage.imu_zyx.size());   
    Eigen::Vector4d vec;
    vec << DataPackage.imu_quat.x(), DataPackage.imu_quat.y(), DataPackage.imu_quat.z(), DataPackage.imu_quat.w();  // 注意 Eigen 内部存储顺序是 x y z w
    imu_quat.assign(vec.data(), vec.data() + vec.size());   

    // std::cout<<"imu_euler_zyx: "<<imu_euler_zyx<<std::endl;
    q_origin = remove_indices(DataPackage.motor_pos, indices_to_remove);
    // q.assign(q_origin.data(), q_origin.data() + q_origin.size());  
    dot_q_origin = remove_indices(DataPackage.motor_pos, indices_to_remove);
    // dot_q.assign(dot_q_origin.data(), dot_q_origin.data() + dot_q_origin.size());    
    // dot_q = DataPackage.motor_vel.data();

    js_vx_desire = DataPackage.js_vx_desire;
    js_vy_desire = DataPackage.js_vy_desire;
    js_OmegaZ_control = DataPackage.js_OmegaZ_desire;
    control_mode = DataPackage.control_mode;
    // vx_Final = DataPackage.vx_Final;
}


void rl_deploy_mimic::SetDataToPackage(DataPackage &data)
{

    data.torq_desire.setZero() ;
    // //******************************for real robot**************************************** */
    
    Eigen::VectorXd dof_pos_desire = outputdata_eigen * action_scale + default_dof_pos_eigen;
    assign_with_skipped_zero(data.motor_Pos_desire, dof_pos_desire, indices_to_remove);
    // data.motor_Pos_desire = dof_pos_desire;
    data.motor_Vel_desire.setZero();
    data.motor_Torque_desire.setZero();

    data.sim_P = rl_p*1.0;
    data.sim_D = rl_d*1.0;
}


Eigen::MatrixXd rl_deploy_mimic::LoadMotionLib(const std::string& filename, bool skipHeader) {
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


void rl_deploy_mimic::Step()
{
    if (step_num == 0){
        step_num = 1;
    }
    else{
        step_num += 1;
    }
    // set the input_data/obs
    if(step_num % decimation == 0)
    {   
        if (frame_count < num_motion_frames){
            frame_count++;
        }
        int frame_id = (frame_count - 1) % num_motion_frames;  // Start from row 0
        num_commands = num_mimic_commands;
        num_proprio = num_proprio_mimic;
        commands = Eigen::VectorXd::Zero(num_commands);
        commands.segment(0, num_commands) = motion_frames.row(frame_id).segment(0, num_commands);

        num_task_obs = num_mimic_task_obs;
        num_observations = num_proprio + num_task_obs + history_len * num_proprio;
        inputdata_eigen = Eigen::VectorXd::Zero(num_observations);
        inputdata_eigen.segment(num_proprio, num_task_obs) = motion_frames.row(frame_id).segment(num_commands, num_task_obs);
        
        // write the history obs inputdata_nlp
        inputdata_eigen.segment(num_commands+6+3*num_of_dofs+num_task_obs, history_len * num_proprio) = hist_mimic_obs_buf_eigen;
        // write commands and proprio obs
        inputdata_eigen.segment(0, num_commands) = commands.segment(0, num_commands);
        inputdata_eigen.segment(num_commands, 3) = imu_ang_vel * ang_vel_scale;
        inputdata_eigen.segment(num_commands+3, 3) = gravity_vec_eigen;
        inputdata_eigen.segment(num_commands+6, num_of_dofs) = (q_origin - default_dof_pos_eigen) * dof_pos_scale;
        inputdata_eigen.segment(num_commands+6+num_of_dofs, num_of_dofs) = dot_q_origin * dof_vel_scale;
        inputdata_eigen.segment(num_commands+6+2*num_of_dofs, num_of_dofs) = last_actions_eigen;

        // refresh the history buf
        hist_mimic_obs_buf_eigen.head(hist_mimic_obs_buf_eigen.size() - num_proprio) = hist_mimic_obs_buf_eigen.tail(hist_mimic_obs_buf_eigen.size() - num_proprio);
        hist_mimic_obs_buf_eigen.tail(num_proprio) = inputdata_eigen.head(num_proprio);
    
    }
    // policy inference
    if (step_num % decimation == 0)
    {
  

        torch::Device inference_device(torch::kCPU);
        std::vector<torch::jit::IValue> inputs;
        torch::Tensor inputdata_tensor = torch::zeros({num_observations}).toType(torch::kFloat);
        // Copy data safely without using copytoinputs
        auto input_accessor = inputdata_tensor.accessor<float, 1>();
        for (int i = 0; i < num_observations; i++) {
            input_accessor[i] = static_cast<float>(inputdata_eigen(i));
        }
        inputdata_tensor = inputdata_tensor.to(inference_device);
        inputdata_tensor = inputdata_tensor.unsqueeze(0);
        inputs.push_back(inputdata_tensor);


        
        torch::Tensor output_tensor;
        output_tensor = mimic_policy.forward(inputs).toTensor();
        output_dof_pos = output_tensor * action_scale + default_dof_pos;
        
  

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
    }
}


void rl_deploy_mimic::Exit()
{
    // clear the hist buf and the step counter
    hist_mimic_obs_buf_eigen.setZero();
    step_num = 0;
    frame_count =0;
}


Eigen::VectorXd rl_deploy_mimic::remove_indices(const Eigen::VectorXd& input, const std::vector<int>& indices_to_remove) 
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


Eigen::Vector3d rl_deploy_mimic::Euler_ZYXToGravityVec(Eigen::Vector3d euler_a) {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R = Eigen::AngleAxisd(euler_a[0], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(euler_a[1], Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(euler_a[2], Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Vector3d grav_vec = -R.transpose().col(2);
    return grav_vec;
}


void rl_deploy_mimic::assign_with_skipped_zero(
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
