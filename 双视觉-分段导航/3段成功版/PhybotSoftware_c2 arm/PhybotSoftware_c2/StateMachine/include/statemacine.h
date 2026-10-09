
#include <iostream>
#include <unordered_map>
#include <memory>
#include <functional>
#include <thread>
#include <chrono>
#include "StateMachine/include/fsmlist.h"
#include"DataPackage/include/DataPackage.h"


#pragma once 

class StateMachine {
public:
    virtual ~StateMachine() = default;

    // 设置当前状态
    void setCurrentState(State state) {
        currentState = state;
    }

    // 处理事件，虚拟函数，子类需要覆盖
    virtual void run(Event event, DataPackage& package) = 0;

    virtual void start (Event event, DataPackage& data) = 0;
    virtual void exit (Event event, DataPackage& data) = 0;
    virtual bool CanExit(State &current, State & next,  DataPackage& data) { return true; }



    // 打印当前状态
    void printCurrentState() const {
        std::cout << "Current State: " << static_cast<int>(currentState) << std::endl;
    }

    State currentState = State::IDLE; // 默认初始状态是StateA

    std::string toString(State state) {
        switch (state) {
            case State::IDLE:
                return "IDLE";
            case State::ZERO:
                return "ZERO";
            case State::RL_walk:
                return "RL";
            case State::RL_mimic:
                return "RL";
            default:
                return "UNKNOWN";
        }
    }


};


// // 定义事件处理器类，支持动态添加事件处理函数
// class EventHandler : public StateMachine {
// public:
//     // 注册事件处理函数
//     void registerEventHandler(Event event, std::function<void()> handler) {
//         eventHandlers[event] = handler;
//     }

//     // 处理事件
//     void handleEvent(Event event) {
//         if (eventHandlers.find(event) != eventHandlers.end()) {
//             std::cout << "Handling event " << static_cast<int>(event) << " in state " << static_cast<int>(currentState) << std::endl;
//             eventHandlers[event]();
//         } else {
//             std::cout << "No handler registered for event " << static_cast<int>(event) << " in state " << static_cast<int>(currentState) << std::endl;
//         }
//     }

// private:
//     // 存储事件到函数的映射
//     std::unordered_map<Event, std::function<void()>> eventHandlers;
// };