#include "pid.h"
#include <algorithm>
#include <cmath>

void PidController::reset() {
    integral_ = 0;
    previousError_ = 0;
    hasPrevious_ = false;
}

PidResult PidController::update(float error, float dtSeconds) {
    PidResult result;
    if (!std::isfinite(error) || !std::isfinite(dtSeconds) || dtSeconds < 0.001f || dtSeconds > 1.0f ||
        !std::isfinite(settings_.kp) || !std::isfinite(settings_.ki) || !std::isfinite(settings_.kd) ||
        !std::isfinite(settings_.integralLimit) || !std::isfinite(settings_.outputLimit) ||
        settings_.kp < 0 || settings_.ki < 0 || settings_.kd < 0 ||
        settings_.integralLimit <= 0 || settings_.outputLimit <= 0) {
        reset();
        return result;
    }
    // Teammate's P + I + D, corrected to seconds and with first-step D = 0.
    const float candidate = std::clamp(integral_ + error * dtSeconds,
                                       -settings_.integralLimit, settings_.integralLimit);
    result.p = settings_.kp * error;
    result.d = hasPrevious_ ? settings_.kd * (error - previousError_) / dtSeconds : 0.0f;
    const float candidateOutput = result.p + settings_.ki * candidate + result.d;
    if (!std::isfinite(candidateOutput)) {
        reset();
        return PidResult{};
    }
    // Conditional integration: never accumulate further into saturation.
    if (!((candidateOutput > settings_.outputLimit && error > 0) ||
          (candidateOutput < -settings_.outputLimit && error < 0))) integral_ = candidate;
    // Use the candidate term for this command, but only store it when allowed.
    // This also handles an I-only controller whose first increment saturates.
    result.i = settings_.ki * candidate;
    result.output = std::clamp(candidateOutput,
                              -settings_.outputLimit, settings_.outputLimit);
    result.valid = true;
    previousError_ = error;
    hasPrevious_ = true;
    return result;
}

float forwardCorrectionLimit(int base, int maximum, float requested) {
    if (base <= 0 || base >= maximum || !std::isfinite(requested) || requested <= 0) return 0;
    return std::min(requested, static_cast<float>(std::min(base, maximum - base)));
}

WheelSpeeds mixForwardSpeeds(int base, float correction, int maximum) {
    WheelSpeeds speeds;
    if (maximum <= 0 || base < 0 || base > maximum || !std::isfinite(correction)) return speeds;
    // LINE_FOLLOW is forward-only. Manual MOTOR_TEST still permits reverse.
    const float left = std::clamp(static_cast<float>(base) + correction, 0.0f, static_cast<float>(maximum));
    const float right = std::clamp(static_cast<float>(base) - correction, 0.0f, static_cast<float>(maximum));
    speeds.left = static_cast<int>(std::lround(left));
    speeds.right = static_cast<int>(std::lround(right));
    speeds.valid = true;
    return speeds;
}
