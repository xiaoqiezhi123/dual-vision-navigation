

#include "DataPackage/include/DataPackage.h"
#include <torch/script.h>
#include <fstream>
#include <iomanip>

class rl_deploy_amp{

public:

    rl_deploy_amp();
    ~rl_deploy_amp();

    // void Init();

    void GetDataFromPackage(DataPackage &data);
    void SetDataToPackage(DataPackage &data);
    void Step();
    void Exit();
    Eigen::MatrixXd LoadMotionLib(const std::string& filename, bool skipHeader);
    Eigen::Vector3d Euler_ZYXToGravityVec(Eigen::Vector3d euler_zyx);



private:
    torch::jit::script::Module policy;
    Eigen::Vector3d gravity_vec_eigen;
    Eigen::VectorXd default_dof_pos_eigen;
    Eigen::VectorXd last_actions_eigen;
    Eigen::VectorXd inputdata_eigen;
    Eigen::VectorXd outputdata_eigen;

    Eigen::VectorXd pos_hist_buf_eigen;
    Eigen::VectorXd vel_hist_buf_eigen;
    Eigen::VectorXd action_hist_buf_eigen;
    Eigen::VectorXd rot_hist_buf_eigen;
    Eigen::VectorXd ang_vel_hist_buf_eigen;

    Eigen::VectorXd q_origin;
    Eigen::VectorXd dot_q_origin;

    Eigen::VectorXd lin_vel;           
    Eigen::VectorXd imu_angular_vel;      
    Eigen::VectorXd commands;
    // Eigen::VectorXd task_commands;    
    Eigen::VectorXd base_euler;   
    Eigen::VectorXd base_quat;   
    Eigen::VectorXd dof_pos;           
    Eigen::VectorXd dof_vel;           
    Eigen::VectorXd last_actions;
    Eigen::VectorXd task_obs;

    torch::Tensor torque_limits;
    torch::Tensor commands_scale;
    torch::Tensor default_dof_pos;
    torch::Tensor output_dof_pos;

    int decimation;
    int num_proprio;
    int num_observations;

    int num_js_commands;
    int num_mimic_commands;
    int num_commands;
    double clip_obs;
    double clip_actions_lower;
    double clip_actions_upper;
    // double action_scale;
    int num_of_dofs;
    int history_len;
    double lin_vel_scale;
    double ang_vel_scale;
    double dof_pos_scale;
    double dof_vel_scale;
    double js_vx_desire;
    double js_vy_desire;
    double js_OmegaZ_desire;
    int step_num = 0;
    int episode_length = 0;
    int frame_count = 0;
    int num_motion_frames;



    double phase;
    std::string policy_path;
    std::string mimic_policy_path;
    std::string motion_lib_path;
    std::vector<int> indices_to_remove;

    Eigen::VectorXd rl_p;
    Eigen::VectorXd rl_d;
    Eigen::VectorXd action_scale;

    template<typename T>
    std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
    {
        std::vector<T> values;
        for(const auto& val : node)
        {
            values.push_back(val.as<T>());
        }
        return values;
    }

    Eigen::MatrixXd motion_frames;
    Eigen::VectorXd remove_indices(const Eigen::VectorXd& input, const std::vector<int>& indices_to_remove);
    void assign_with_skipped_zero(Eigen::VectorXd& target,const Eigen::VectorXd& source,const std::vector<int>& indices_to_skip);

    // torch::Tensor EulerZYXRotateInverse(torch::Tensor euler, torch::Tensor v);
    // torch::Tensor QuatRotateInverse(torch::Tensor q, torch::Tensor v);

    // Logging members
    std::ofstream task_obs_log_file;
    bool enable_task_obs_logging;
    // Logging functions
    void InitTaskObsLogging(const std::string& log_file_path = "../RL_deploy/logs/task_obs_log.csv");
    void LogTaskObs(const torch::Tensor& task_obs_tensor);
    void CloseTaskObsLogging();
};


