#pragma once
#include <array>
#include <cstddef>

struct TuningParameters {
    float kp;
    float kd;
    int speed;
    int trim;
    // L+, L-, R+, R-. Each value is a measured motor-duty floor.
    std::array<int, 4> start;
    std::array<int, 4> sustain;
    unsigned long limitMs;
    unsigned long pulseMs;
};

void tuningInit();
const TuningParameters& tuningParameters();
// Handles GET, SAVE and KP/KD/SPD assignments. Call from the control task.
bool tuningCommand(const char* command, bool running, char* reply, std::size_t size);
