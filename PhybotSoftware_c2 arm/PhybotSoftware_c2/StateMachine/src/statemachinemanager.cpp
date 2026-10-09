#include "StateMachine/include/statemachinemanager.h"

void StateMachineManager::GetDataFromPackage(DataPackage &DataPackage)
{
    NextState = DataPackage.NextState;

    // IfWalkStart = DataPackage.IfWalkStart;
    // IfGaitEnd = DataPackage.IfGaitEnd;
    // IfStand2Walk = DataPackage.IfStand2Walk;
    // IfCanStop = DataPackage.IfCanStop;
    // std::cout<< "GetDataFromPackage IfWalkStart:"<<IfWalkStart<<std::endl;

}
void StateMachineManager::SetDataToPackage(DataPackage &DataPackage)
{
    DataPackage.CurrentState = CurrentState;

    // DataPackage.IfWalkStart = IfWalkStart;
    // DataPackage.IfStand2Walk = IfStand2Walk;
    // std::cout<< "SetDataToPackage IfWalkStart:"<<IfWalkStart<<std::endl;

}


void StateMachineManager::run(DataPackage& data) {
    auto currentHandler = handlers[CurrentState];
    if (CurrentState != NextState) {
        // 传入 current state 给 handler

        if (currentHandler && currentHandler->CanExit(CurrentState, NextState, data)) {

            handleEvent(CurrentState, Event::EXIT, data);

            std::cout << "Success transition: " << static_cast<int>(CurrentState)
            << " → " << static_cast<int>(NextState) << std::endl;


            CurrentState = NextState;

        } else {
            std::cout << "Blocked transition: " << static_cast<int>(CurrentState)
                      << " → " << static_cast<int>(NextState) << std::endl;
        }
    }

    handleEvent(CurrentState, Event::RUN, data);

}


