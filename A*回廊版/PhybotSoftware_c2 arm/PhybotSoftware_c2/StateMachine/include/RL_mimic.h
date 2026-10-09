#pragma once

#include "StateMachine/include/statemacine.h"

#include "RL_deploy_mimic/include/rl_deploy.h"

class RL_mimic : public StateMachine {
public:
    RL_mimic();

    void start(Event event, DataPackage& data) override;
    void run(Event event, DataPackage& data) override;
    void exit(Event event, DataPackage& data) override;
    bool CanExit(State& current, State& next, DataPackage& data) ;

private:
    // std::unique_ptr<WBC_Solver::WeightedWBCSolver> wbc;
    // std::unique_ptr<GaitPlanner> gaitplanner;
    // std::unique_ptr<Pino_Kinematics> pino;
    std::unique_ptr<rl_deploy_mimic> rl_mimic;
};
