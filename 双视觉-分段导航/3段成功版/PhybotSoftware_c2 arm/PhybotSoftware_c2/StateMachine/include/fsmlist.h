#pragma once 

#include <iostream>

// 定义状态枚举
enum class State {
IDLE,

ZERO,
RL_walk,
RL_mimic,

};

// 定义事件枚举
enum class Event {
START,
RUN,
EXIT
};

inline std::ostream& operator<<(std::ostream& os, const State& state) {
    switch (state) {
        case State::IDLE:     os << "IDLE"; break;
        case State::RL_walk:   os << "RL_walk"; break;
        case State::RL_mimic:   os << "RL_mimic"; break;
        case State::ZERO:   os << "ZERO"; break;
        default:              os << "Unknown State"; break;
    }
    return os;
}