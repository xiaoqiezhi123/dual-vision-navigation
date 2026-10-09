
#include "mujoco_interface.h"

MujocoInterface::MujocoInterface(DataPackage &data) {

    std::string file_path = "../MujocoInterface/config/mujoco_sim.yaml";
    YAML::Node config = YAML::LoadFile(file_path);
    char error[1000] = "Could not load binary model";
    std::string env_path = config["env_path"].as<std::string>();
    mjModel*mj_model = mj_loadXML(env_path.c_str(), 0, error, 10000);

    // // mj_data = mj_makeData(mj_model);

    jointNum = data.actuatedDofNum;
    motor_pos.setZero(jointNum);
    motor_pos_old.setZero(jointNum);
    motor_vel.setZero(jointNum);
    motor_torque.setZero(jointNum);



    std::vector<std::string> JointName = config["joint_names"].as<std::vector<std::string>>();

    baseName= config["baseName"].as<std::string>();
    orientationSensorName=config["orientationSensorName"].as<std::string>();
    velSensorName=config["velSensorName"].as<std::string>();
    gyroSensorName=config["gyroSensorName"].as<std::string>();
    accSensorName=config["accSensorName"].as<std::string>();

    metorvel_noise_std=config["metorvel_noise_std"].as<double>();



    // timeStep=mj_model->opt.timestep;
    // jointNum=JointName.size();
    jntId_qpos.assign(jointNum,0); 
    jntId_qvel.assign(jointNum,0);
    jntId_dctl.assign(jointNum,0);
    // motor_pos.assign(jointNum,0);
    // motor_vel.assign(jointNum,0);
    // motor_pos_Old.assign(jointNum,0);


    sim_P.resize(jointNum);
    sim_D.resize(jointNum);

    for (int i=0;i<jointNum;i++)
    {
        int tmpId= mj_name2id(mj_model,mjOBJ_JOINT,JointName[i].c_str());
        if (tmpId==-1)
        {
            std::cerr <<JointName[i]<< " not found in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_qpos[i]=mj_model->jnt_qposadr[tmpId];
        jntId_qvel[i]=mj_model->jnt_dofadr[tmpId];

        // motorName="M"+motorName.substr(1);
        std::cout<<JointName[i]<<std::endl;
        tmpId= mj_name2id(mj_model,mjOBJ_ACTUATOR,JointName[i].c_str());
        if (tmpId==-1)
        {
            std::cerr <<JointName[i]<< " not found in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_dctl[i]=tmpId;


    }


    baseBodyId= mj_name2id(mj_model,mjOBJ_BODY, baseName.c_str());
    orientataionSensorId= mj_name2id(mj_model, mjOBJ_SENSOR, orientationSensorName.c_str());
    velSensorId= mj_name2id(mj_model,mjOBJ_SENSOR,velSensorName.c_str());
    gyroSensorId= mj_name2id(mj_model,mjOBJ_SENSOR,gyroSensorName.c_str());
    accSensorId= mj_name2id(mj_model,mjOBJ_SENSOR,accSensorName.c_str());



    // q_desire<<-0.2,0,0,0.4,-0.2,0,-0.2,0,0,0.4,-0.2,0;

    // mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco


    // std::string file_path2 = "../WholeBodyControl/config/wholebodycontrol.yaml";
    // YAML::Node config2 = YAML::LoadFile(file_path2);

    // torq_P = config2["torq_P"].as<double>();
    // torq_D = config2["torq_D"].as<double>(); 

}

void MujocoInterface::GetDataFromSim(mjModel*mj_model, mjData* mj_data) {

    // std::default_random_engine generator;

    for (int i=0;i<jointNum;i++)
    {
        motor_pos[i]=mj_data->qpos[jntId_qpos[i]];
        motor_vel[i]=mj_data->qvel[jntId_qvel[i]] ;
        motor_torque[i] = mj_data->qfrc_actuator[jntId_qvel[i]];
    }

    // std::cout<<"xanchor: "<< mj_data->xanchor[2] <<std::endl;
    // std::cout<<"njnt: "<< mj_model-> njnt <<std::endl;
    // for (int i = 0; i < mj_model->nq; ++i) {  // mj_model->njnt 是关节的数量
    //     std::cout << "Joint " << i << " position: " << mj_data->qpos[i] << " ";
    // }
    // std::cout << std::endl;



    imu_quat.x() = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+1];
    imu_quat.y() = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+2];
    imu_quat.z() = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+3];
    imu_quat.w() = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+0];

    mocap_baseQuat[0] = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+1];
    mocap_baseQuat[1] = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+2];
    mocap_baseQuat[2] = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+3];
    mocap_baseQuat[3] = mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+0];

    // imu_quat.x() =  mj_data->qpos[4];
    // imu_quat.y() =  mj_data->qpos[5];
    // imu_quat.z() = mj_data->qpos[6];
    // imu_quat.w() = mj_data->qpos[3];

    // std::cout<<"mj_data->qpos[3]: "<<mj_data->qpos[3]<<std::endl;
    // std::cout<<"mj_data->qpos[4]: "<<mj_data->qpos[4]<<std::endl;
    // std::cout<<"mj_data->qpos[5]: "<<mj_data->qpos[5]<<std::endl;
    // std::cout<<"mj_data->qpos[6]: "<<mj_data->qpos[6]<<std::endl;


    // std::cout<<"imu_quat.x(): "<<imu_quat.x()<<std::endl;
    // std::cout<<"imu_quat.y(): "<<imu_quat.y()<<std::endl;
    // std::cout<<"imu_quat.z(): "<<imu_quat.z()<<std::endl;
    // std::cout<<"imu_quat.w(): "<<imu_quat.w()<<std::endl;
    // double total_mass = 0.0;
    // for (int i = 1; i < mj_model->nbody; ++i) {  // 从 1 开始，跳过 world body
    //     total_mass += mj_model->body_mass[i];
    // }
    // std::cout << "Total robot mass: " << total_mass << " kg" << std::endl;

    // for (int i=0;i<4;i++)
    //     baseQuat[i]=mj_data->sensordata[mj_model->sensor_adr[orientataionSensorId]+i];
    // double tmp=baseQuat[0];
    // baseQuat[0]=baseQuat[1];
    // baseQuat[1]=baseQuat[2];
    // baseQuat[2]=baseQuat[3];
    // baseQuat[3]=tmp;

    


    for (int i=0;i<3;i++)
    {
        double last_pos = base_pos[i];
        base_pos[i]=mj_data->xpos[3*baseBodyId+i];
        imu_lin_acc[i]=mj_data->sensordata[mj_model->sensor_adr[accSensorId]+i];
        imu_angular_vel[i]=mj_data->sensordata[mj_model->sensor_adr[gyroSensorId]+i];
        base_lin_vel[i]=(base_pos[i]-last_pos)/(mj_model->opt.timestep);
        

    }

    for (int i=0;i<3;i++)
    {
        double last_pos = mocap_basePos[i];
        mocap_basePos[i]=mj_data->xpos[3*baseBodyId+i];
        mocap_baseVel[i]=(mocap_basePos[i]-last_pos)/(mj_model->opt.timestep);
        

    }


    // 计算整个模型的质心（bodyid = -1表示所有身体）


    // imu_angular_vel[0] = mj_data->qvel[3];
    // imu_angular_vel[1] = mj_data->qvel[4];
    // imu_angular_vel[2] = mj_data->qvel[5];

    // std::cout<<"mj_data->qvel[3]: "<<mj_data->qvel[3]<<std::endl;
    // std::cout<<"mj_data->qvel[4]: "<<mj_data->qvel[4]<<std::endl;
    // std::cout<<"mj_data->qvel[5]: "<<mj_data->qvel[5]<<std::endl;



    // std::cout<<"imu_angular_vel1: "<<imu_angular_vel[0]<<std::endl;
    // std::cout<<"imu_angular_vel2: "<<imu_angular_vel[1]<<std::endl;
    // std::cout<<"imu_angular_vel3: "<<imu_angular_vel[2]<<std::endl;

}

void MujocoInterface::SetTorque(DataPackage &data, mjData* mj_data) 
{

    sim_P = data.sim_P;
    sim_D = data.sim_D;

        for (int i=0;i<jointNum;i++)
        {
            mj_data->ctrl[i] = data.motor_Torque_desire[i] + sim_P[i] * (data.motor_Pos_desire[i] - motor_pos[i]) + sim_D[i] * (data.motor_Vel_desire[i] - motor_vel[i]);

            // std::cout<<"data.torq_desire: "<<i<<" : "<<sim_D[i] <<  <<std::endl;
        }
            // mj_data->ctrl[i]=50*(data.generalized_q_desired[6+i] - motor_pos[i]) - 10 * motor_vel[i] + data.torq_desire[i];
}   

void MujocoInterface::SetDataToPackage(DataPackage &data) 
{


    data.motor_pos = motor_pos;
    data.motor_vel = motor_vel;
    data.motor_torque = motor_torque;
    data.base_pos = base_pos;
    // std::cout<<"base_pos_mea: "<<base_pos<<std::endl;
    // std::cout<<"base_lin_vel_mea: "<<base_lin_vel<<std::endl;
    data.imu_angular_vel = imu_angular_vel;
    data.imu_lin_acc = imu_lin_acc;
    data.base_lin_vel = base_lin_vel;
    data.imu_quat = imu_quat;
    data.imu_zyx = quatToZyx(imu_quat);
    // data.baseQuat = baseQuat;

    data.mocap_basePos = mocap_basePos;
    data.mocap_baseQuat = mocap_baseQuat;


}

Eigen::Matrix<double, 3, 1> MujocoInterface::quatToZyx(const Eigen::Quaternion<double>& q)
{
    Eigen::Matrix<double, 3, 1> zyx;

    double as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), 0.99999);

    as = std::max(std::min(as, 1.0), -0.99999);  // 【双向夹紧在[-1, 1]】

    zyx(0) =
        std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    zyx(1) = std::asin(as);
    zyx(2) =
        std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
    return zyx;
}









