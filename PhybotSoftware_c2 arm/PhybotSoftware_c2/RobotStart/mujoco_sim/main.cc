// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <cmath>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"
#include"DataPackage/include/DataPackage.h"
// #include "GLFW_callbacks.h"
#include "mujoco_interface.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h> // 用于文件输出

#include "StateMachine/include/statemachinemanager.h"
// #include "DataLogger/include/DataLogger.h"
#include "Joystick/include/joystick_int.h"

//  ************************* lcm ****************************

// #include "lcm_interface/LcmInterface.h"

// *****************************************************

// #define DATALOG
#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

bool g_start_rl_walk = false;

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

YAML::Node LoadSpawnConfig(std::string& source_path) {
  const std::vector<std::string> navside_paths = {
    "../NavSide/config/nav_molmospaces_procthor.yaml",
    "../../NavSide/config/nav_molmospaces_procthor.yaml",
    "/home/ubuntu/sru_mujoco_sim/NavSide/config/nav_molmospaces_procthor.yaml",
  };

  for (const std::string& navside_path : navside_paths) {
    try {
      YAML::Node navside_config = YAML::LoadFile(navside_path);
      if (navside_config["sim"] && navside_config["sim"]["spawn_w"]) {
        source_path = navside_path;
        return navside_config["sim"];
      }
    } catch (const YAML::Exception& e) {
      std::cout << "[RobotSide] NavSide spawn config unavailable at "
                << navside_path << ": " << e.what() << std::endl;
    }
  }

  const std::string robotside_path = "../MujocoInterface/config/mujoco_sim.yaml";
  source_path = robotside_path;
  return YAML::LoadFile(robotside_path);
}

void ApplyConfiguredSpawn(mjModel* model, mjData* data) {
  if (!model || !data) {
    return;
  }

  std::string file_path;
  YAML::Node config = LoadSpawnConfig(file_path);
  if (!config["spawn_w"]) {
    return;
  }

  const std::vector<double> spawn = config["spawn_w"].as<std::vector<double>>();
  if (spawn.size() != 3 || model->nq < 7) {
    std::cerr << "[RobotSide] invalid spawn_w in " << file_path << std::endl;
    return;
  }

  const double yaw = config["spawn_yaw"] ? config["spawn_yaw"].as<double>() : 0.0;
  const double half_yaw = 0.5 * yaw;
  data->qpos[0] = spawn[0];
  data->qpos[1] = spawn[1];
  data->qpos[2] = spawn[2];
  data->qpos[3] = std::cos(half_yaw);
  data->qpos[4] = 0.0;
  data->qpos[5] = 0.0;
  data->qpos[6] = std::sin(half_yaw);
  mj_forward(model, data);

  std::cout << "[RobotSide] spawn source=" << file_path << std::endl;
  std::cout << "[RobotSide] applied spawn_w=[" << spawn[0] << ", " << spawn[1]
            << ", " << spawn[2] << "] spawn_yaw=" << yaw << std::endl;
}

// ******
// low_cmd_t recvCmd;
// ******

// control noise variables
// mjtNum* ctrlnoise = nullptr;

using Seconds = std::chrono::duration<double>;


bool dataLog(Eigen::VectorXd &v, std::ofstream &f) {
    for (int i = 0; i < v.size(); i++) {
        f << v[i] << " ";
    }
    f << std::endl;
    return true;
}

//------------------------------------------- simulation -------------------------------------------


mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}



void update_camera(mjvCamera* cam, mjData* data) {
    // 设置相机类型为自由相机
    cam->type = mjCAMERA_FREE;

    // 设置相机的目标点为基座的位置
    cam->lookat[0] = data->xpos[3]; // X 坐标
    cam->lookat[1] = data->xpos[4]; // Y 坐标
    cam->lookat[2] = data->xpos[5]; // Z 坐标
}

// 在仿真循环中调用此函数
void print_mouse_force(const mjModel* model, const mjData* data) {
  mjtNum total_force[3] = {0, 0, 0};
  for (int body_id = 0; body_id < model->nbody; body_id++) {
      int offset = body_id * 6;
      total_force[0] += data->xfrc_applied[offset];
      total_force[1] += data->xfrc_applied[offset + 1];
      total_force[2] += data->xfrc_applied[offset + 2];
  }
  mjtNum magnitude = mju_norm3(total_force);
  // 打印三维力和标量
  // printf("Total mouse force vector: [Fx=%.2f N, Fy=%.2f N, Fz=%.2f N]\n", 
  //   total_force[0], total_force[1], total_force[2]);
  // printf("Total mouse force magnitude: %.2f N\n", magnitude);
  
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim, mjvCamera* cam) {

//  cpu_set_t cpuset;
//  CPU_ZERO(&cpuset);
//  CPU_SET(0,&cpuset);
//  pthread_t current_thread = pthread_self();
//  int ret = pthread_setaffinity_np(current_thread,sizeof(cpu_set_t), &cpuset);
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 1;

  DataPackage package;
  package.init();


  Joystick joystick;
  joystick.init();
  if (g_start_rl_walk) {
    joystick.SetNextState(State::RL_walk);
  }
  
  // DataLogger Logger;
  // spdlog::info("DataLogger");

  StateMachineManager manager;
  // spdlog::info("StateMachineManager");
  manager.init(package);

  // std::cout<<"ssssssssssss"<<std::endl;

  // spdlog::info("manager.init");
  // 设置日志级别
  spdlog::set_level(spdlog::level::info); // 只记录INFO级别及以上的日志


  MujocoInterface MujocoInterface(package); // data interface for Mujoco
  spdlog::info("interface load success!");


  spdlog::info("model load success!");


#ifdef DATALOG
  int motorNum = package.actuatedDofNum;
  std::ofstream foutData;
  foutData.open("../logdata/datacollection.txt", std::ios::out);
  Eigen::VectorXd dataL = Eigen::VectorXd::Zero(74 + 6 * motorNum);
#endif

  cam->distance = 3;
  cam->elevation = 0;
  cam->azimuth = 180;
  
  // run until asked to exit
  while (!sim.exitrequest.load()) {
    // if (sim.droploadrequest.load()) {

    

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        ApplyConfiguredSpawn(mnew, dnew);
        sim.Load(mnew, dnew, sim.filename);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        // ********************************
        // init_cmd(d);
        // ********************************

        mj_forward(m, d);

        // allocate ctrlnoise
        // free(ctrlnoise);
        // ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
        // mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    { // todo 控制变量的生命周期
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // ************ test ****************



          MujocoInterface.GetDataFromSim(m,d);
          MujocoInterface.SetDataToPackage(package);



          joystick.GetDataFromPackage(package);
          joystick.run();
          joystick.SetDataToPackage(package);


          manager.GetDataFromPackage(package);


          manager.run(package);



          manager.SetDataToPackage(package); 
          

          MujocoInterface.SetTorque(package,d);

          mj_step(m, d);
          // mj_step1(m, d);

          double sim_time = d->time;
          

          double x1= d->xanchor[36];
          double y1= d->xanchor[37];
          double z1= d->xanchor[38];



          print_mouse_force(m, d);

          // record_start_time = std::chrono::steady_clock::now();


          // record_now_time = std::chrono::steady_clock::now();
          // elapsed_record = record_now_time - record_start_time;
          // std::cout<< "calculate time:"<<elapsed_record.count()<<std::endl;

          double x2= d->xanchor[36];
          double y2= d->xanchor[37];
          double z2= d->xanchor[38];

          double vx=(x2-x1)/package.control_period;
          double vy=(y2-y1)/package.control_period;
          double vz=(z2-z1)/package.control_period;

          Eigen::Vector3d velocity(vx, vy, vz);

          std::this_thread::sleep_for(std::chrono::microseconds(800));

          
          // Logger.SaveDataToFile(package, sim_time, d, velocity, state_estimator);
#ifdef DATALOG
          dataL[0] = 1;
          dataL.block(1, 0, 6, 1) = package.generalized_q_actual.head(6);
          dataL.block(7, 0, 6, 1) = package.generalized_q_dot_actual.head(6);
          dataL.block(13, 0, 6, 1) = package.generalized_q_desired.head(6);
          dataL.block(19, 0, 6, 1) = package.generalized_q_dot_desired.head(6);
          dataL.block(25 , 0, motorNum, 1) = package.generalized_q_desired.tail(motorNum);
          dataL.block(25 + 1 * motorNum, 0, motorNum, 1) = package.motor_Pos_desire;
          dataL.block(25 + 2 * motorNum, 0, motorNum, 1) = package.motor_Torque_desire;  //only for mujoco
          dataL.block(25 + 3 * motorNum, 0, motorNum, 1) = package.motor_pos;
          dataL.block(25 + 4 * motorNum, 0, motorNum, 1) = package.motor_vel;
          dataL.block(25 + 5 * motorNum, 0, motorNum, 1) = package.motor_torque;
          dataL.block(25 + 6 * motorNum, 0, 3, 1) = package.imu_lin_acc;
          dataL.block(25 + 6 * motorNum+3, 0, 3, 1) = package.vel_base_obser_left;
          dataL[31 + 6 * motorNum] = package.contact_force[2];
          dataL[32 + 6 * motorNum] = package.contact_force[8];
          dataL[33 + 6 * motorNum] = package.FootNum;       
          dataL[34 + 6 * motorNum] = package.contact_flag[0];
          dataL[35 + 6 * motorNum] = package.contact_flag[1]; 
          //180
          dataL.block(36 + 6 * motorNum, 0, 3, 1) = package.feet_l_Pos_W_actual;
          dataL.block(39 + 6 * motorNum, 0, 3, 1) = package.feet_r_Pos_W_actual;
          //186
          dataL.block(42 + 6 * motorNum, 0, 3, 1) = package.feet_l_Pos_W_desire;
          dataL.block(45 + 6 * motorNum, 0, 3, 1) = package.feet_r_Pos_W_desire;
          
          dataL.block(48 + 6 * motorNum, 0, 3, 1) = package.feet_l_LinVel_W_actual;
          dataL.block(51 + 6 * motorNum, 0, 3, 1) = package.feet_r_LinVel_W_actual;
          dataL.block(54 + 6 * motorNum, 0, 3, 1) = package.feet_l_LinVel_W_desire;
          dataL.block(57 + 6 * motorNum, 0, 3, 1) = package.feet_r_LinVel_W_desire;
          dataL.block(60 + 6 * motorNum, 0, 3, 1) = package.feet_l_EulerZYX_W_actual;
          dataL.block(63 + 6 * motorNum, 0, 3, 1) = package.feet_r_EulerZYX_W_actual;
          dataL[66 + 6 * motorNum] = package.feet_l_EulerZ_W_desire;
          dataL[67 + 6 * motorNum] = package.feet_r_EulerZ_W_desire;
          dataL[68 + 6 * motorNum] = package.feet_l_OmegaZ_W_desire;
          dataL[69 + 6 * motorNum] = package.feet_r_OmegaZ_W_desire;
          dataL.block(70 + 6 * motorNum, 0, 2, 1) = package.u_B;
          dataL.block(70 + 6 * motorNum+2, 0, 3, 1) = package.vel_base_obser_right;
          // std::cout<<"package.u_B: "<<package.u_B<<std::endl;
          // Write to file
          dataLog(dataL, foutData);
#endif

          // std::cout<< "log time:"<<elapsed_record.count()<<std::endl;          

          update_camera(cam, d);
          // ****************************

          // // record cpu time at start of iteration
          // const auto startCPU = mj::Simulate::Clock::now();

          // // elapsed CPU and simulation time since last sync
          // const auto elapsedCPU = startCPU - syncCPU;
          // double elapsedSim = d->time - syncSim;


          // double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // // misalignment condition: distance from target sim time is bigger than syncmisalign
          // bool misaligned =
          //     mju_abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;
      
          // // out-of-sync (for any reason): reset sync times, step
          // if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
          //     misaligned || sim.speed_changed) {
          //   // re-sync
          //   syncCPU = startCPU;
          //   syncSim = d->time;
          //   sim.speed_changed = false;

          //   // run single step, let next iteration deal with timing
          //   mj_step(m, d);
          //   stepped = true;
          // }

          // // in-sync: step until ahead of cpu
          // else {
          //   bool measured = false;
          //   mjtNum prevSim = d->time;

          //   double refreshTime = simRefreshFraction/sim.refresh_rate;

          //   // step while sim lags behind cpu and within refreshTime
          //   while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
          //           mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
          //     // measure slowdown before first step
          //     if (!measured && elapsedSim) {
          //       sim.measured_slowdown =
          //           std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
          //       measured = true;
          //     }

          //     // call mj_step
          //     mj_step(m, d);
          //     stepped = true;

          //     // break if reset
          //     if (d->time < prevSim) {
          //       break;
          //     }
          //   }
          // }

          // // save current state to history buffer
          // if (stepped) {
          //   sim.AddToHistory();
          // }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
  }
  // ****************************
  // mujocolcm.joinLCMThread();
  // ****************************
#ifdef DATALOG
      foutData.close();
#endif
}
}  // namespace



//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename, mjvCamera* cam) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) d = mj_makeData(m);
    if (d) {
      // ********************************

      mju_copy(d->qpos, m->key_qpos, m->nq*1); // set ini pos in Mujoco
      ApplyConfiguredSpawn(m, d);
      //   // ********************************

      mj_forward(m, d);
      // ********************************
      sim->Load(m, d, filename);
      mj_forward(m, d);

      // allocate ctrlnoise
      // free(ctrlnoise);
      // ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
      // mju_zero(ctrlnoise, m->nu);
    } else {
      sim->LoadMessageClear();
    }
  }
  

  PhysicsLoop(*sim, cam);

  // delete everything we allocated

  // free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------


//**************************
// run event loop
int main(int argc, char** argv) {


 

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  // scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);






  std::cout << "Start in RL_walk state? [y/N]: ";
  std::string start_choice;
  std::getline(std::cin, start_choice);
  if (!start_choice.empty()) {
    char c = start_choice[0];
    if (c == 'y' || c == 'Y') {
      g_start_rl_walk = true;
    }
  }

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );


  std::string file_path = "../MujocoInterface/config/mujoco_sim.yaml";
  YAML::Node config = YAML::LoadFile(file_path);
  std::string env_path = config["env_path"].as<std::string>();
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), env_path.c_str(), &cam);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}
