#include <iostream>
#include "Mocap/include/mocap_streamer.h"


MocapStreamer::MocapStreamer() {
    // std::string file_path = "../config/fzmotion.yaml";
    std::string file_path = "../Mocap/config/fzmotion.yaml";
    YAML::Node config = YAML::LoadFile(file_path);
    broadcaster_ip = config["broadcaster_ip"].as<std::string>();
    base_link_name = config["base_link_name"].as<std::string>();
    shuttlecock_name = config["shuttlecock_name"].as<std::string>();
}


MocapStreamer::~MocapStreamer() {
    bExit = true;
    if (mocap_thread.joinable()) {
        mocap_thread.join();
    }
    if (LusterServer->IsConnected()) {
        LusterServer->Disconnect(broadcaster_ip);
    }
    LusterServer->Close();
    printf("Mocap Close.\n");
}


// public funcs
void MocapStreamer::init(){
    int start = MocapStreamer::mocap_init();
}


void MocapStreamer::GetDataFromPackage(DataPackage &datapackage) {
    // get the waist yaw joint dof
    waist_yaw_angle = datapackage.motor_pos(12); // 12 is the waist yaw joint id
}


TrajectoryData true_data;
TrajectoryData data;
PredictionResult result;


void MocapStreamer::Step() {
    // refresh the Mocap data
    mocap_run(0);
    // get base quat
    RigidBodyPose base_link_pose = get_mocap_rigid_body_pose(base_link_name);
    // get some other terms
    RigidBodyTwist base_link_twist = get_mocap_rigid_body_twist(base_link_name);
    // derive the desired state variables and set to data package
    
    //get shuttlecock pos
    MarkerPos shuttlecock_pos = get_mocap_marker_pos(shuttlecock_name);
    mocap_shuttlecock_pos = shuttlecock_pos.pos / 1000;
    // pos[1]<<" "<<mocap_shuttlecock_pos[2]<<std::endl;

    std::vector<double> hit_zone = {-10.8, 10.8, -10.0, 10.2};
    // std::cout<<data.x_traj.size()<<std::endl;
    // std::cout<<mocap_shuttlecock_pos[0]<<mocap_shuttlecock_pos[1]<<mocap_shuttlecock_pos[2]<<std::endl;
    if (shuttlecock_pos.pos[0] != last_shuttlecock_pos.pos[0] & shuttlecock_pos.pos[2] >= 0.00001 & last_shuttlecock_pos.pos[2] >= 0.00001){
        // std::cout<<(mocap_shuttlecock_pos[0]-last_shuttlecock_pos.pos[0]/1000)/0.0047618<<std::endl;
        if ((mocap_shuttlecock_pos[0]-last_shuttlecock_pos.pos[0]/1000)/0.0047618 < -6 & hitflag == false){ 
            hitflag = true;
            count1 = 0;
            data.times.clear();
            data.x_traj.clear();
            data.y_traj.clear();
            data.z_traj.clear();
            // std::cout<<"////////////////"<<std::endl;
        }
        if (hitflag){
            data.dt = 0.0047618; 
            data.times.push_back(count1*data.dt);
            data.x_traj.push_back(mocap_shuttlecock_pos[0]);
            data.y_traj.push_back(mocap_shuttlecock_pos[1]);
            data.z_traj.push_back(mocap_shuttlecock_pos[2]); 
            count1 += 1;
            count2 += 1;
            count3 += 1;
            // if (count1 > 80 & pre_num == 0){
            //     result = predictHitPoint(data,1.54,hit_zone,1);
            //     count1 = 0;
            //     pre_num = 1;
            // }
            if (count1 > 80){
                
                result = predictHitPoint(data,1.54,hit_zone,1);
            }
            if (mocap_shuttlecock_pos[2] < last_shuttlecock_pos.pos[2]/1000 & mocap_shuttlecock_pos[2] < 1.6 ){ 
                // pre_num += 1;
                hitflag = false;
                count1 = 0;
                data.times.clear();
                data.x_traj.clear();
                data.y_traj.clear();
                data.z_traj.clear();}
            // if (hittrueflag & pre_num2 == 0){
            //     truex1 = mocap_shuttlecock_pos[0];
            //     truey1 = mocap_shuttlecock_pos[1];
            //     truez1 = mocap_shuttlecock_pos[2];
            //     truet1 = count3*0.0047618;
            //     pre_num2 += 1;
            //     hittrueflag = false;
            //     count3 =0;
            // }
            // if (mocap_shuttlecock_pos[2] < last_shuttlecock_pos.pos[2]/1000 & mocap_shuttlecock_pos[2] < 1.5 & hittrueflag1 == false){ 
            //     hittrueflag1 = true;
            //     }
            // if (hittrueflag1 & pre_num1 == 0){
            //     truex = mocap_shuttlecock_pos[0];
            //     truey = mocap_shuttlecock_pos[1];
            //     truez = mocap_shuttlecock_pos[2];
            //     truet = count2*0.0047618;
            //     pre_num1 += 1;
            //     hittrueflag1 = false;
            //     double a1 = (1.54 - truez)/(truez1 - truez);
            //     double x1 = (truex1 - truex)*a1 + truex;
            //     double y1 = (truey1 - truey)*a1 + truey;
            //     double t1 = (truet-truet1)*a1+truet1;
            //     count2 =0;
            // }
            
            // if (mocap_shuttlecock_pos[2] < last_shuttlecock_pos.pos[2]/1000 & mocap_shuttlecock_pos[2] < 1.55 & mocap_shuttlecock_pos[0] < 4.04){count1 = 0;pre_num += 1;}
            
            
        }
        
    }
    count +=1;
    

    // rotate the rigid body quat to robot's base heading
    Eigen::Quaterniond q_mocap(base_link_pose.quat(3), base_link_pose.quat(0), base_link_pose.quat(1), base_link_pose.quat(2));

    mocap_basePos = base_link_pose.pos / 1000;
    // Eigen::Vector3d local_offset(base_offsets[0], base_offsets[1], base_offsets[2]);
    // mocap_basePos = mocap_basePos + q_mocap * local_offset;

    // Difference to acquire velocity
    // std::cout<<mocap_basePos[0]<<" "<<mocap_basePos[1]<<" "<<mocap_basePos[2]<<std::endl;
    // std::cout<<mocap_basePos_Prev[0]<<" "<<mocap_basePos_Prev[1]<<" "<<mocap_basePos_Prev[2]<<std::endl;

    mocap_baseVel = base_link_twist.linear;

    // debug out
    // std::cout<<q_mocap.x()<<" "<<q_mocap.y()<<" "<<q_mocap.z()<<" "<<q_mocap.w()<<std::endl;
    // std::cout<< mocap_basePos(0)<<" "<<mocap_basePos(1)<<" "<<mocap_basePos(2)<<" "<<mocap_baseVel(0)<<" "<<mocap_baseVel(1)<<" "<<mocap_baseVel(2)<<std::endl;
    Eigen::Vector3d ypr = q_mocap.normalized().toRotationMatrix().eulerAngles(0,1,2);
    double base_roll = ypr[0];
    double base_pitch = ypr[1];
    double base_yaw = ypr[2];
    // std::cout<<base_roll<<" "<<base_pitch<<" "<<base_yaw<<std::endl;

    mocap_baseQuat(0) = q_mocap.x();
    mocap_baseQuat(1) = q_mocap.y();
    mocap_baseQuat(2) = q_mocap.z();
    mocap_baseQuat(3) = q_mocap.w();
    // std::cout<<mocap_baseQuat(0)<<" "<<mocap_baseQuat(1)<<" "<<mocap_baseQuat(2)<<" "<<mocap_baseQuat(3)<<std::endl;

    // store the last value for handling the refresh latancy
    last_base_link_pose = base_link_pose;
    last_base_link_twist = base_link_twist;
    if (shuttlecock_pos.pos[2] >= 0.000001)
    {
        last_shuttlecock_pos = shuttlecock_pos;
    }
    // last_shuttlecock_pos = shuttlecock_pos;
    // if (shuttlecock_pos.pos[2] <= 0.000001)
    // {
    //     last_shuttlecock_pos.pos[0] = 4851;
    //     last_shuttlecock_pos.pos[1] = -300;
    //     last_shuttlecock_pos.pos[2] = 850;
    // }
}


void MocapStreamer::SetDataToPackage(DataPackage &datapackage) {
    datapackage.mocap_basePos = mocap_basePos;
    datapackage.mocap_baseQuat = mocap_baseQuat;
    datapackage.mocap_baseVel = mocap_baseVel;

    datapackage.mocap_hit_position = result.hit_position;
    datapackage.mocap_hit_time = result.hit_time;
    datapackage.mocap_hit_flag = hitflag;
    // std::cout<<"mmocap: "<<datapackage.mocap_basePos[0]<<","<<datapackage.mocap_basePos[1]<<","<<datapackage.mocap_baseQuat[0]<<","<<datapackage.mocap_baseQuat[1]<<","<<datapackage.mocap_baseQuat[2]<<","<<datapackage.mocap_baseQuat[3]<<std::endl;
}


// private funcs
int MocapStreamer::mocap_init() {
    std::cout << "connecting to broadcaster_ip: " << broadcaster_ip << std::endl;
    // establish mocap server
    LusterServer = lusternet::getFZReceive();
    LusterServer->Init();
    // connect server
    LusterServer->Connect(broadcaster_ip);
    
    if (LusterServer->IsConnected()) {
        std::cout << "Mocap connected !!!" << std::endl;
        // read the first frame as origins
        LusterServer->ReceiveData(MocapData, 0); // blocking mode
        FrameRigidBody = MocapData.FrameRigidBody;
        RigidBodyPose base_link_pose = get_mocap_rigid_body_pose(base_link_name);
        std::cout << "the base_link init pos is: " << base_link_pose.pos.segment(0, 3) / 1000 << std::endl;
        Frame3DMarker = MocapData.Frame3DMarker;
        MarkerPos shuttlecock_pos = get_mocap_marker_pos(shuttlecock_name);
        last_shuttlecock_pos = shuttlecock_pos;
        std::cout << "the shuttlecock init pos is: \n" << shuttlecock_pos.pos.segment(0, 3) / 1000 << std::endl;
        return 0;
    }
    else {
        std::cout << "Mocap not connected !!!" << std::endl;
        return -1;
    }
}


void MocapStreamer::mocap_run(int start) {
    if (LusterServer->IsConnected()) {
        // Non-blocking mode
        LusterServer->ReceiveData(MocapData, 1);
        // Frame ID
        FrameID = MocapData.FrameID;
        // Timestamp
        TimeStamp = MocapData.TimeStamp;
        // RigidBody data
        FrameRigidBody = MocapData.FrameRigidBody;
        // Skeleton data
        FrameSkeleton = MocapData.FrameBodysPose;
        // Marker Data
        Frame3DMarker = MocapData.Frame3DMarker;
    } else {
        std::cout << "Mocap disconnected !!!" << std::endl;
    }
}


MocapStreamer::RigidBodyPose MocapStreamer::get_mocap_rigid_body_pose(std::string rigid_body_name)
{   
    for (int i =0; i < FrameRigidBody.size(); i++)
    {
        if (FrameRigidBody[i].RigidName.c_str() == rigid_body_name){
            if (FrameRigidBody[i].IsTrack){
                Eigen::Matrix<double, 3, 1> pos;
                pos(0) = FrameRigidBody[i].X;
                pos(1) = FrameRigidBody[i].Y;
                pos(2) = FrameRigidBody[i].Z;
                Eigen::Matrix<double, 4, 1> quat;
                quat(0) = FrameRigidBody[i].qx;
                quat(1) = FrameRigidBody[i].qy;
                quat(2) = FrameRigidBody[i].qz;
                quat(3) = FrameRigidBody[i].qw;
                return {rigid_body_name, pos, quat};
            }
            else{
                // std::cout << "Rigid body " << rigid_body_name << " track failed !!!" << std::endl;
                // TODO: add the failed handling
                return {rigid_body_name, Eigen::Matrix<double, 3, 1>::Zero(), Eigen::Matrix<double, 4, 1>::Zero()};
            }
        }
    }
    // handle the refresh latancy
    // std::cout << "Rigid body " << rigid_body_name << " pose not found !!!" << std::endl;
    return last_base_link_pose;
}


MocapStreamer::MarkerPos MocapStreamer::get_mocap_marker_pos(std::string marker_name)
{   
    for (int i = 0; i < Frame3DMarker.size(); ++i)
    {   
        // printf("MarkerID = %d.\n", Frame3DMarker[i].MarkerID);
        // printf("MarkerName = %s.\n", Frame3DMarker[i].MarkerName.c_str());
        // printf("Pose: [X] = %f, [Y] = %f, [Z] = %f\n", Frame3DMarker[i].X, Frame3DMarker[i].Y, Frame3DMarker[i].Z);
        // if (Frame3DMarker[i].MarkerName.c_str() == marker_name){
        //     Eigen::Matrix<double, 3, 1> pos;
        //     pos(0) = Frame3DMarker[i].X;
        //     pos(1) = Frame3DMarker[i].Y;
        //     pos(2) = Frame3DMarker[i].Z;
        //     return {marker_name, pos};
        // }
        if (Frame3DMarker[i].Z > 910 & Frame3DMarker[i].X > -1000 & Frame3DMarker[i].X < 5100)
        {
            Eigen::Matrix<double, 3, 1> pos;
            pos(0) = Frame3DMarker[i].X;
            pos(1) = Frame3DMarker[i].Y;
            pos(2) = Frame3DMarker[i].Z;
            return {marker_name, pos};
        }
    }
     return last_shuttlecock_pos;
}


MocapStreamer::RigidBodyTwist MocapStreamer::get_mocap_rigid_body_twist(std::string rigid_body_name)
{
    for (int i =0; i < FrameRigidBody.size(); i++){
        if (FrameRigidBody[i].RigidName.c_str() == rigid_body_name){
            if (FrameRigidBody[i].IsTrack){
                Eigen::Matrix<double, 3, 1> linear;
                linear(0) = FrameRigidBody[i].fXSpeed;
                linear(1) = FrameRigidBody[i].fYSpeed;
                linear(2) = FrameRigidBody[i].fZSpeed;
                Eigen::Matrix<double, 3, 1> angular;
                angular(0) = FrameRigidBody[i].fXPalstance;
                angular(1) = FrameRigidBody[i].fYPalstance;
                angular(2) = FrameRigidBody[i].fZPalstance;
                return {rigid_body_name, linear, angular};
            }
            else{
                // std::cout << "Rigid body " << rigid_body_name << " track failed !!!" << std::endl;
                return {rigid_body_name, Eigen::Matrix<double, 3, 1>::Zero(), Eigen::Matrix<double, 3, 1>::Zero()};
            }
        }
    }
    // handle the refresh latancy
    // std::cout << "Rigid body " << rigid_body_name << " not found !!!" << std::endl;
    return last_base_link_twist;
}

constexpr double m = 0.006;
constexpr double rho = 1.225;
constexpr double S = 0.003;
constexpr double C_D = 0.75; 
constexpr double g = 9.85; 


class ExtendedKalmanFilter {
private:
    Eigen::Matrix<double, 6, 1> state;  // [x, y, z, vx, vy, vz]
    Eigen::Matrix<double, 6, 6> P;      // 状态协方差矩阵
    Eigen::Matrix<double, 6, 6> Q;      // 过程噪声协方差
    Eigen::Matrix<double, 3, 3> R;      // 测量噪声协方差
    
    double m_, rho_, S_, C_D_, g_, L_;
    double dt_;
    
public:
    ExtendedKalmanFilter(const Eigen::Matrix<double, 6, 1>& initial_state,
                        double m, double rho, double S, double C_D, double g,
                        double dt, const Eigen::Matrix<double, 6, 6>& initial_covariance)
        : state(initial_state), m_(m), rho_(rho), S_(S), C_D_(C_D), g_(g), dt_(dt) {
        
        L_ = (2 * m_) / (rho_ * S_ * C_D_);

        Q.setZero();
        Q.diagonal() << 0.5, 0.1, 0.1, 30, 10, 5;

        R.setZero();
        R.diagonal() << 0.0001, 0.0001, 0.0001;
 
        P = initial_covariance;
    }
    
    Eigen::Matrix<double, 6, 1> predict() {
        double x = state[0], y = state[1], z = state[2];
        double vx = state[3], vy = state[4], vz = state[5];
        
        double speed = std::sqrt(vx*vx + vy*vy + vz*vz);
        
        double drag_coeff = speed / L_;
        Eigen::Vector3d drag_acc = drag_coeff * Eigen::Vector3d(vx, vy, vz);
        Eigen::Vector3d gravity(0, 0, -g_);
        Eigen::Vector3d a = gravity - drag_acc;
  
        if(speed>1e-3){
            state[0] = x + vx * dt_;
            state[1] = y + vy * dt_;
            state[2] = z + vz * dt_;
            state[3] = vx + a[0] * dt_;
            state[4] = vy + a[1] * dt_;
            state[5] = vz + a[2] * dt_;
        } else{
            state[0] = x + vx * dt_;
            state[1] = y + vy * dt_;
            state[2] = z + vz * dt_;
            state[3] = vx;
            state[4] = vy;
            state[5] = vz - g_ * dt_;
        }

        for (int i = 0; i < 6; i++) {
            if (!std::isfinite(state[i])) {
                return state;
            }
        }

        Eigen::Matrix<double, 6, 6> F;
        F.setZero();
        F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt_;
        
        if (speed > 1e-3) {
            double speed_sq = speed * speed;
            
            double dvx_dvx = 1 - dt_ * drag_coeff * (1 + (vx*vx)/speed_sq);
            double dvx_dvy = -dt_ * drag_coeff * (vx * vy)/speed_sq;
            double dvx_dvz = -dt_ * drag_coeff * (vx * vz)/speed_sq;
            
            double dvy_dvx = -dt_ * drag_coeff * (vy * vx)/speed_sq;
            double dvy_dvy = 1 - dt_ * drag_coeff * (1 + (vy*vy)/speed_sq);
            double dvy_dvz = -dt_ * drag_coeff * (vy * vz)/speed_sq;
            
            double dvz_dvx = -dt_ * drag_coeff * (vz * vx)/speed_sq;
            double dvz_dvy = -dt_ * drag_coeff * (vz * vy)/speed_sq;
            double dvz_dvz = 1 - dt_ * drag_coeff * (1 + (vz*vz)/speed_sq);
            
            F.block<3, 3>(3, 3) << dvx_dvx, dvx_dvy, dvx_dvz,
                                   dvy_dvx, dvy_dvy, dvy_dvz,
                                   dvz_dvx, dvz_dvy, dvz_dvz;
        } else {
            F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
        }

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                if (!std::isfinite(F(i, j))) {
                    F = Eigen::Matrix<double, 6, 6>::Identity();
                    break;
                }
            }
        }
        
 
        try {
            P = F * P * F.transpose() + Q;
        
            P = (P + P.transpose()) / 2;
            P += Eigen::Matrix<double, 6, 6>::Identity() * 1e-10;
        } catch (...) {
 
            P.setZero();
            P.diagonal() << 0.1, 0.1, 0.1, 1.0, 1.0, 1.0;
        }
        
        return state;
    }
    
    Eigen::Matrix<double, 6, 1> update(const Eigen::Vector3d& measurement) {
 
        for (int i = 0; i < 3; i++) {
            if (!std::isfinite(measurement[i])) {

                return state;
            }
        }
        Eigen::Matrix<double, 3, 6> H;
        H.setZero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        
        try {
            Eigen::Matrix3d S = H * P * H.transpose() + R;
            S = (S + S.transpose()) / 2;
            S += Eigen::Matrix3d::Identity() * 1e-10;
            
            Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();
            Eigen::Vector3d y = measurement - H * state;
            state = state + K * y;
            Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
            P = (I - K * H) * P;
            P = (P + P.transpose()) / 2;
            P += Eigen::Matrix<double, 6, 6>::Identity() * 1e-10;
            
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
        } catch (...) {
            std::cout  << std::endl;
        }
        
        return state;
    }
    
    Eigen::Matrix<double, 6, 1> getState() const {
        return state;
    }
};
Eigen::Vector4d computeQuaternion(const Eigen::Vector3d& vel) {
    try {

        for (int i = 0; i < 3; i++) {
            if (!std::isfinite(vel[i])) {
                return Eigen::Vector4d(0, 0, 0, 1.0);
            }
        }
        
        double speed = vel.norm();
        if (speed < 1e-6) {
            return Eigen::Vector4d(0, 0, 0, 1.0);
        }
        
        Eigen::Vector3d z_axis = vel / speed;
        Eigen::Vector3d forward(-1.0, 0.0, 0.0);
        Eigen::Vector3d x_axis = forward - forward.dot(z_axis) * z_axis;
        double x_axis_norm = x_axis.norm();
        
        if (x_axis_norm < 1e-6) {
            Eigen::Vector3d alternate(0.0, 1.0, 0.0);
            x_axis = alternate - alternate.dot(z_axis) * z_axis;
            x_axis_norm = x_axis.norm();
        }
        
        if (x_axis_norm < 1e-6) {
            Eigen::Vector3d alternate(0.0, 0.0, 1.0);
            x_axis = alternate - alternate.dot(z_axis) * z_axis;
            x_axis_norm = x_axis.norm();
        }
        
        if (x_axis_norm > 1e-6) {
            x_axis /= x_axis_norm;
        } else {
            return Eigen::Vector4d(0, 0, 0, 1.0);
        }
        Eigen::Vector3d y_axis = z_axis.cross(x_axis);
        double y_axis_norm = y_axis.norm();
        if (y_axis_norm > 1e-6) {
            y_axis /= y_axis_norm;
        } else {
            return Eigen::Vector4d(0, 0, 0, 1.0);
        }
        Eigen::Matrix3d rot_mat;
        rot_mat.col(0) = x_axis;
        rot_mat.col(1) = y_axis;
        rot_mat.col(2) = z_axis;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (!std::isfinite(rot_mat(i, j))) {
                    return Eigen::Vector4d(0, 0, 0, 1.0);
                }
            }
        }
        double trace = rot_mat.trace();
        Eigen::Vector4d quat;
        
        if (trace > 0) {
            double s = std::sqrt(trace + 1.0) * 2;
            quat[3] = 0.25 * s;
            quat[0] = (rot_mat(2, 1) - rot_mat(1, 2)) / s;
            quat[1] = (rot_mat(0, 2) - rot_mat(2, 0)) / s;
            quat[2] = (rot_mat(1, 0) - rot_mat(0, 1)) / s;
        } else if (rot_mat(0, 0) > rot_mat(1, 1) && rot_mat(0, 0) > rot_mat(2, 2)) {
            double s = std::sqrt(1.0 + rot_mat(0, 0) - rot_mat(1, 1) - rot_mat(2, 2)) * 2;
            quat[3] = (rot_mat(2, 1) - rot_mat(1, 2)) / s;
            quat[0] = 0.25 * s;
            quat[1] = (rot_mat(0, 1) + rot_mat(1, 0)) / s;
            quat[2] = (rot_mat(0, 2) + rot_mat(2, 0)) / s;
        } else if (rot_mat(1, 1) > rot_mat(2, 2)) {
            double s = std::sqrt(1.0 + rot_mat(1, 1) - rot_mat(0, 0) - rot_mat(2, 2)) * 2;
            quat[3] = (rot_mat(0, 2) - rot_mat(2, 0)) / s;
            quat[0] = (rot_mat(0, 1) + rot_mat(1, 0)) / s;
            quat[1] = 0.25 * s;
            quat[2] = (rot_mat(1, 2) + rot_mat(2, 1)) / s;
        } else {
            double s = std::sqrt(1.0 + rot_mat(2, 2) - rot_mat(0, 0) - rot_mat(1, 1)) * 2;
            quat[3] = (rot_mat(1, 0) - rot_mat(0, 1)) / s;
            quat[0] = (rot_mat(0, 2) + rot_mat(2, 0)) / s;
            quat[1] = (rot_mat(1, 2) + rot_mat(2, 1)) / s;
            quat[2] = 0.25 * s;
        }
        
        return quat;
        
    } catch (const std::exception& e) {
        std::cout << e.what()  << std::endl;
        return Eigen::Vector4d(0, 0, 0, 1.0);
    }
}
PredictionResult predictHitPoint(TrajectoryData data,double target_z,
                                const std::vector<double>& hit_zone,
                                double use_fraction) {
    
    PredictionResult result;
    result.valid = false;
    result.has_error_analysis = false;
    if (data.times.empty()) {
        return result;
    }
    if (data.x_traj.size() < 3) {
        return result;
    }
    size_t cutoff_idx = std::max(3, static_cast<int>(data.x_traj.size() * use_fraction));
    double cutoff_time = data.times[cutoff_idx-1];
    
    // std::cout << "\n使用前 " << cutoff_idx << " 个点 (t=" << cutoff_time << "s) 进行滤波和预测" << std::endl;
    
    // 计算初始速度
    double vx0 = (data.x_traj[1] - data.x_traj[0]) / data.dt;
    double vy0 = (data.y_traj[1] - data.y_traj[0]) / data.dt;
    double vz0 = (data.z_traj[1] - data.z_traj[0]) / data.dt;
    
    // 初始状态
    Eigen::Matrix<double, 6, 1> initial_state;
    initial_state << data.x_traj[0], data.y_traj[0], data.z_traj[0], vx0, vy0, vz0;
    
    // std::cout << "初始状态:" << std::endl;
    // std::cout << "  位置: (" << initial_state[0] << ", " << initial_state[1] << ", " << initial_state[2] << ")" << std::endl;
    // std::cout << "  速度: (" << initial_state[3] << ", " << initial_state[4] << ", " << initial_state[5] << ")" << std::endl;
    
    // 初始化EKF
    Eigen::Matrix<double, 6, 6> initial_covariance;
    initial_covariance.setZero();
    initial_covariance.diagonal() << 0.1, 0.1, 0.1, 1.0, 1.0, 1.0;
    
    ExtendedKalmanFilter ekf(initial_state, m, rho, S, C_D, g, data.dt, initial_covariance);
    
    // 滤波阶段
    std::vector<Eigen::Matrix<double, 6, 1>> ekf_states;
    ekf_states.push_back(initial_state);
    
    for (size_t k = 1; k < cutoff_idx; k++) {
        ekf.predict();
        Eigen::Vector3d measurement(data.x_traj[k], data.y_traj[k], data.z_traj[k]);
        ekf.update(measurement);
        ekf_states.push_back(ekf.getState());
    }
    
    // std::cout << "开始预测击球点..." << std::endl;
    
    // 预测阶段
    std::vector<Eigen::Matrix<double, 6, 1>> pred_states;
    std::vector<double> pred_times;
    std::vector<Eigen::Vector3d> predicted_trajectory;

    Eigen::Matrix<double,6 ,1>filtered_state = ekf_states.back();
    pred_states.push_back(filtered_state);
    pred_times.push_back(cutoff_time);
    predicted_trajectory.push_back(filtered_state.head<3>());
    
    bool hit_found = false;
    double x_pred, y_pred, time_pred;
    Eigen::Vector3d velocity;
    
    for (int k = 0; k < 2000; k++) {
        // 预测下一状态
        Eigen::Matrix<double, 6, 1> state = ekf.predict();
        
        // 检查状态有效性
        bool state_valid = true;
        for (int i = 0; i < 6; i++) {
            if (!std::isfinite(state[i])) {
                state_valid = false;
                break;
            }
        }
        
        if (!state_valid) {
            // std::cout << "警告：预测状态包含无效值，停止预测" << std::endl;
            break;
        }
        
        // 记录预测状态
        double current_time = pred_times.back() + data.dt;
        pred_times.push_back(current_time);
        pred_states.push_back(state);
        predicted_trajectory.push_back(state.head<3>());
        
        // 检查是否穿越目标高度
        if (pred_states.size() >= 2) {
            double prev_z = pred_states[pred_states.size()-2][2];
            double current_z = state[2];
            
            if (prev_z > target_z && current_z <= target_z) {
                // 线性插值计算精确穿越点
                double t_frac = (target_z - prev_z) / (current_z - prev_z);
                
                // 插值位置
                Eigen::Vector3d prev_pos = pred_states[pred_states.size()-2].head<3>();
                Eigen::Vector3d curr_pos = pred_states[pred_states.size()-1].head<3>();
                Eigen::Vector3d hit_pos = prev_pos + t_frac * (curr_pos - prev_pos);
                
                x_pred = hit_pos[0];
                y_pred = hit_pos[1];
                
                // 插值速度
                Eigen::Vector3d v_prev = pred_states[pred_states.size()-2].tail<3>();
                Eigen::Vector3d v_curr = pred_states[pred_states.size()-1].tail<3>();
                velocity = v_prev + t_frac * (v_curr - v_prev);
                
                // 预测的穿越时间
                time_pred = pred_times[pred_times.size()-2] + t_frac * data.dt;  
                
                hit_found = true;
                break;
            }
        }
        
        // 检查是否落地
        if (state[2] < 0) {
            // std::cout << "警告：预测中球落地而未穿越目标高度" << std::endl;
            x_pred = state[0];
            y_pred = state[1];
            velocity = state.tail<3>();
            time_pred = current_time;
            break;
        }
    }
    
    if (!hit_found && pred_states.empty()) {
        // std::cout << "错误：无法进行预测" << std::endl;
        return result;
    }
    
    if (!hit_found) {
        auto last_state = pred_states.back();
        x_pred = last_state[0];
        y_pred = last_state[1];
        velocity = last_state.tail<3>();
        time_pred = pred_times.back();
    }
    
    // 计算四元数
    Eigen::Vector4d quat_pred = computeQuaternion(velocity);
    
    // 检查是否在击球区域内
    bool in_hit_zone = (x_pred >= hit_zone[0] && x_pred <= hit_zone[1] &&
                       y_pred >= hit_zone[2] && y_pred <= hit_zone[3]);
    
    // 计算误差分析
    Eigen::Vector3d actual_pos_at_hit_time;
    bool has_error_analysis = false;
    
    if (time_pred <= data.times.back()) {
        // 通过插值找到预测击球时刻的实际位置
        for (size_t i = 0; i < data.times.size() - 1; i++) {
            if (data.times[i] <= time_pred && time_pred <= data.times[i + 1]) {
                double t_frac = (time_pred - data.times[i]) / (data.times[i + 1] - data.times[i]);
                actual_pos_at_hit_time[0] = data.x_traj[i] + t_frac * (data.x_traj[i + 1] - data.x_traj[i]);
                actual_pos_at_hit_time[1] = data.y_traj[i] + t_frac * (data.y_traj[i + 1] - data.y_traj[i]);
                actual_pos_at_hit_time[2] = data.z_traj[i] + t_frac * (data.z_traj[i + 1] - data.z_traj[i]);
                has_error_analysis = true;
                break;
            }
        }
    } else {
        // 如果预测时间超出了实际轨迹时间，使用最后一个点
        actual_pos_at_hit_time[0] = data.x_traj.back();
        actual_pos_at_hit_time[1] = data.y_traj.back();
        actual_pos_at_hit_time[2] = data.z_traj.back();
        has_error_analysis = true;
    }
    
    // 输出结果
    // std::cout << "\n===== 预测结果 =====" << std::endl;
    // std::cout << "预测击球点 (z=" << std::fixed << std::setprecision(4) << target_z << "):" << std::endl;
    // std::cout << "  (" << x_pred<< ", " << y_pred  << ", " << target_z << ")" << std::endl;//0.23625 0.07984
    // std::cout << "  时间: " << time_pred<< " s" << std::endl;
    // std::cout << "  速度: " << std::setprecision(2) << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << " m/s" << std::endl;
    // std::cout << "  速度大小: " << velocity.norm() << " m/s" << std::endl;
    // std::cout << "  在击球区域内: " << (in_hit_zone ? "是" : "否") << std::endl;
    
    if (has_error_analysis) {
        // std::cout << "\n===== 误差分析 =====" << std::endl;
        // std::cout << "预测击球时刻 t=" << std::setprecision(4) << time_pred << "s 的实际位置:" << std::endl;
        // std::cout << "  实际位置: (" << actual_pos_at_hit_time[0] << ", " << actual_pos_at_hit_time[1] << ", " << actual_pos_at_hit_time[2] << ") m" << std::endl;
        
        Eigen::Vector3d position_error;
        position_error[0] = x_pred - actual_pos_at_hit_time[0];
        position_error[1] = y_pred - actual_pos_at_hit_time[1];
        position_error[2] = target_z - actual_pos_at_hit_time[2];
        double position_error_3d = position_error.norm();
        
        // std::cout << "位置误差:" << std::endl;
        // std::cout << "  X方向: " << position_error[0] << " m, Y方向: " << position_error[1] << " m, Z方向: " << position_error[2] << " m" << std::endl;
        // std::cout << "  3D位置误差: " << position_error_3d << " m (" << position_error_3d * 1000 << " mm)" << std::endl;
        
        // 存储误差信息到结果中
        result.has_error_analysis = true;
        result.actual_position_at_hit_time = actual_pos_at_hit_time;
        result.position_error = position_error;
        result.position_error_3d = position_error_3d;
    }
    
    if (!in_hit_zone) {
        // std::cout << "  击球区域: X[" << hit_zone[0] << ", " << hit_zone[1] << "], Y[" << hit_zone[2] << ", " << hit_zone[3] << "]" << std::endl;
    }
    
    // 填充结果结构
    result.hit_position = Eigen::Vector3d(x_pred, y_pred, target_z);
    result.hit_time = time_pred;
    result.hit_velocity = velocity;
    result.hit_quaternion = quat_pred;
    result.in_hit_zone = in_hit_zone;
    result.prediction_start_time = cutoff_time;
    result.total_trajectory_time = data.times.back();
    result.valid = true;
    
    return result;}