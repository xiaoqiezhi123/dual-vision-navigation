#include "RL_mimic.h"
#include <iostream>

RL_mimic::RL_mimic() : rl_mimic(nullptr) {
    setCurrentState(State::RL_mimic);
    printCurrentState();
}

void RL_mimic::start(Event event, DataPackage& data) {
    if (rl_mimic == nullptr) {

        rl_mimic = std::make_unique<rl_deploy_mimic>();
    }
}

void RL_mimic::run(Event event, DataPackage& data) {


    rl_mimic->GetDataFromPackage(data);
    rl_mimic->Step();
    rl_mimic->SetDataToPackage(data);
}

void RL_mimic::exit(Event event, DataPackage& data) {
    // 可添加清理操作
}
bool RL_mimic::CanExit(State& current, State& next, DataPackage& data)
 {
    rl_mimic->Exit();
    return true;
}
