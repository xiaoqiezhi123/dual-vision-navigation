#pragma once

#include "StateMachine/include/statemacine.h"
#include "ZeroState/include/ZeroState.h"

class Zero : public StateMachine {
public:
    Zero();

    void start(Event event, DataPackage& data) override;
    void run(Event event, DataPackage& data) override;
    void exit(Event event, DataPackage& data) override;
    bool CanExit(State& current, State& next, DataPackage& data) ;

private:
    std::unique_ptr<ZeroState> zerostate;
};
