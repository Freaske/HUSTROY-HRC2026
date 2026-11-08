#pragma once
#include "sensors.h"

enum class RobotMode { MOTOR_TEST, SENSOR_TEST, LINE_FOLLOW };
struct RobotStatus {
    RobotMode mode = RobotMode::MOTOR_TEST;
    bool armed = false; // Operator authorization, separate from hardware readiness.
    bool running = false;
    int leftCommand = 0;
    int rightCommand = 0;
    float error = 0;
    float correction = 0;
    SensorFrame sensors;
    const char* stopReason = "STARTUP";
};

void modesInit(unsigned long now);
void modesUpdate(unsigned long now);
void modesSelect(RobotMode mode, unsigned long now); // Always stops; never arms.
bool modesRun(unsigned long now); // LINE_FOLLOW start request; requires prior ARM.
bool modesArm(); // Stops first; no motion. Only MOTOR_TEST / LINE_FOLLOW.
void modesDisarm(const char* reason); // Stops and revokes operator authorization.
void modesStop(const char* reason);
const RobotStatus& modesStatus();
const char* modeName(RobotMode mode);
