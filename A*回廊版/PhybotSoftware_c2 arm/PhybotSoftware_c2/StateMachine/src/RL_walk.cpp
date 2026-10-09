#include "RL_walk.h"
#include <iostream>

RL_walk::RL_walk() : rl_walk(nullptr) {
    setCurrentState(State::RL_walk);
    printCurrentState();
}

void RL_walk::start(Event event, DataPackage& data) {
    if (rl_walk == nullptr) {

        rl_walk = std::make_unique<rl_deploy_cpg>();
    }
}

void RL_walk::run(Event event, DataPackage& data) {


    rl_walk->GetDataFromPackage(data);
    rl_walk->Step(data);
    rl_walk->SetDataToPackage(data);
}

void RL_walk::exit(Event event, DataPackage& data) {
    // 可添加清理操作
}
bool RL_walk::CanExit(State& current, State& next, DataPackage& data)
 {
    return true;
}
