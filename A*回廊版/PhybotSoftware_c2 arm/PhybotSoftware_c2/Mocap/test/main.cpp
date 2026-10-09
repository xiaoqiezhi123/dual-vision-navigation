#include <iostream>
#include "Mocap/include/mocap_streamer.h"

int main() {
    MocapStreamer mocap_streamer;
    mocap_streamer.init();
    while (1) {
        mocap_streamer.Step();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return 0;
}