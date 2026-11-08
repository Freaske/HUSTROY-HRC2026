#pragma once

struct PidSettings {
    float kp;
    float ki;
    float kd;
    float integralLimit;
    float outputLimit;
};
struct PidResult {
    bool valid = false;
    float output = 0;
    float p = 0;
    float i = 0;
    float d = 0;
};
class PidController {
public:
    explicit PidController(PidSettings settings) : settings_(settings) {}
    void reset();
    PidResult update(float error, float dtSeconds);
    float integral() const { return integral_; }
private:
    PidSettings settings_;
    float integral_ = 0;
    float previousError_ = 0;
    bool hasPrevious_ = false;
};

struct WheelSpeeds { int left = 0; int right = 0; bool valid = false; };
float forwardCorrectionLimit(int base, int maximum, float requested);
WheelSpeeds mixForwardSpeeds(int base, float correction, int maximum);
