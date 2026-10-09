#include "Zero.h"
#include <iostream>

Zero::Zero() : zerostate(nullptr) {
    setCurrentState(State::ZERO);
    printCurrentState();
}

void Zero::start(Event event, DataPackage& data) {
    if (zerostate == nullptr) {
        zerostate = std::make_unique<ZeroState>();
    }
}

void Zero::run(Event event, DataPackage& data) {
    zerostate->GetDataFromPackage(data);
    zerostate->SetDataToPackage(data);

}

void Zero::exit(Event event, DataPackage& data) {
   data.If_first_Zero = true;
    data.zero_target_reached = false;
   // 可选的退出逻辑
}
bool Zero::CanExit(State& current, State& next, DataPackage& data)
 {
    return data.zero_target_reached;
}
