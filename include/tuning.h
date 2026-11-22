#pragma once
#include <cstddef>

struct TuningParameters {
    float kp;
    float kd;
    int speed;
};

void tuningInit();
const TuningParameters& tuningParameters();
// Handles GET, SAVE and KP/KD/SPD assignments. Call from the control task.
bool tuningCommand(const char* command, bool running, char* reply, std::size_t size);
