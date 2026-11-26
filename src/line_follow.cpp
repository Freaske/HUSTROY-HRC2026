#include "line_follow.h"
#include "config.h"
#include <algorithm>
#include <cmath>

namespace {
float unit(float value) { return std::clamp(value, 0.0f, 1.0f); }
int sign(float value) { return (value > 0) - (value < 0); }
}

bool lineCalibrationValid(const LineCalibration& cal, bool inverted) {
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        const int white = cal.white[i], black = cal.black[i], floor = cal.floor[i];
        if (white < 0 || white > SENSOR_ADC_MAX || black < 0 || black > SENSOR_ADC_MAX ||
            floor < 0 || floor > SENSOR_ADC_MAX || !std::isfinite(cal.coordinates[i]) ||
            std::abs(cal.coordinates[i]) > 4.0f ||
            (i > 0 && cal.coordinates[i] <= cal.coordinates[i - 1])) return false;
        const int polarity = inverted ? -1 : 1;
        const int span = polarity * (black - white);
        const int background = polarity * (floor - white);
        if (span < 128 || background < 0 || background >= span / 2) return false;
    }
    return cal.coordinates.front() < 0 && cal.coordinates.back() > 0;
}

LineObservation observeLine(const SensorFrame& frame, SensorInputMode mode,
                            const LineCalibration& cal) {
    LineObservation line;
    if (!frame.rawAvailable) return line;
    std::array<float, SENSOR_COUNT> weights{};
    int strong = 0;
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        if (mode == SensorInputMode::DIGITAL_DO) {
            weights[i] = line.strength[i] = frame.active[i] ? 1.0f : 0.0f;
        } else {
            const float span = static_cast<float>(cal.black[i] - cal.white[i]);
            const float floor = (cal.floor[i] - cal.white[i]) / span;
            line.strength[i] = unit((frame.raw[i] - cal.white[i]) / span);
            weights[i] = unit((line.strength[i] - floor) / (1.0f - floor));
        }
        if (weights[i] >= 0.55f) ++strong;
    }

    unsigned clusters = 0;
    for (std::size_t first = 0; first < SENSOR_COUNT;) {
        if (weights[first] <= 0.025f) { ++first; continue; }
        float total = 0, moment = 0, peak = 0;
        std::size_t end = first;
        while (end < SENSOR_COUNT && weights[end] > 0.025f) {
            total += weights[end];
            moment += weights[end] * cal.coordinates[end];
            peak = std::max(peak, weights[end]);
            ++end;
        }
        if (peak >= LINE_MIN_PEAK && total >= LINE_MIN_MASS) {
            ++clusters;
            line.position = moment / total;
            line.confidence = std::max(line.confidence, unit(0.75f * peak + 0.25f * total));
            line.outer = line.outer || (first == 0 && weights.front() >= LINE_MIN_MASS) ||
                         (end == SENSOR_COUNT && weights.back() >= LINE_MIN_MASS);
        }
        first = end;
    }
    line.signal = clusters != 0;
    if (clusters > 1 || strong >= 4 ||
        (weights.front() >= LINE_MIN_MASS && weights.back() >= LINE_MIN_MASS)) {
        line.reading = LineReading::Ambiguous;
        line.position = 0;
    } else {
        line.reading = clusters == 1 ? LineReading::Position : LineReading::Missing;
    }
    return line;
}

const char* followStateName(FollowState state) {
    switch (state) {
        case FollowState::Search: return "SEARCH";
        case FollowState::Recover: return "RECOVER";
        case FollowState::Ambiguous: return "AMBIGUOUS";
        default: return "TRACKING";
    }
}

int LineFollower::wheelDuty(float request, unsigned wheel, unsigned nowUs,
                            const TuningParameters& settings) {
    // Hysteresis avoids cycling through zero or reversing on every small correction.
    const int direction = sign(request);
    const float threshold = direction == motorSigns_[wheel] ? 2.0f : 5.0f;
    if (std::abs(request) < threshold) {
        motorSigns_[wheel] = 0;
        return 0;
    }
    if (direction != motorSigns_[wheel]) {
        // A commanded sign reversal gets one neutral control tick.
        const bool reversing = motorSigns_[wheel] != 0;
        motorSigns_[wheel] = reversing ? 0 : direction;
        motorStartedAt_[wheel] = nowUs;
        if (reversing) return 0;
    }
    const unsigned index = 2 * wheel + (direction < 0 ? 1 : 0);
    const int floor = nowUs - motorStartedAt_[wheel] < 60000 ? settings.start[index]
                                                           : settings.sustain[index];
    return direction * std::max(static_cast<int>(std::lround(std::abs(request))), floor);
}

FollowOutput LineFollower::drive(FollowOutput output, float steering, int reverseLimit,
                                 unsigned nowUs, const TuningParameters& settings) {
    // Lower common forward demand first as steering uses the available positive headroom.
    const float ceiling = std::min(MAX_SPEED, settings.speed + 18);
    const float bound = std::min(ceiling, (ceiling + reverseLimit) * 0.5f);
    output.correction = std::clamp(steering, -bound, bound);
    const float turn = std::abs(output.correction);
    output.base = std::clamp(output.base, std::max(0.0f, turn - reverseLimit), ceiling - turn);
    output.left = std::clamp(wheelDuty(output.base + output.correction, 0, nowUs, settings),
                             -reverseLimit, MAX_SPEED);
    output.right = std::clamp(wheelDuty(output.base - output.correction, 1, nowUs, settings),
                              -reverseLimit, MAX_SPEED);
    return output;
}

FollowOutput LineFollower::update(const LineObservation& line, const TuningParameters& settings,
                                  unsigned nowUs, unsigned dtUs) {
    FollowOutput output;
    if (line.reading == LineReading::Invalid) { output.fault = "SENSOR_FAULT"; return output; }
    if (dtUs == 0 || dtUs >= LINE_STALE_US) { output.fault = "STALE_CONTROL"; return output; }
    const float dt = dtUs * 0.000001f;
    if (line.reading != LineReading::Position) {
        if (!interrupted_) {
            interrupted_ = true;
            interruptedAt_ = nowUs;
        }
        derivativeReady_ = false;
        settling_ = false;
        reacquireSamples_ = 0;
        directionCandidate_ = 0;
        reverseAtEdge_ = false;
        const unsigned unavailableUs = nowUs - interruptedAt_;
        if (line.reading == LineReading::Ambiguous) {
            if (!ambiguity_) ambiguityAt_ = nowUs;
            ambiguity_ = true;
            wasCentered_ = false;
            base_ = 0;
            motorSigns_ = {};
            output.state = FollowState::Ambiguous;
            if (nowUs - ambiguityAt_ >= LINE_AMBIGUITY_LIMIT_US || unavailableUs >= LINE_LOSS_LIMIT_US)
                output.fault = "LINE_AMBIGUOUS";
            return output;
        }
        ambiguity_ = false;
        output.state = FollowState::Search;
        if (unavailableUs >= LINE_LOSS_LIMIT_US) { output.fault = "LINE_LOST_TIMEOUT"; return output; }
        if (wasCentered_ && unavailableUs < LINE_BLIND_BRIDGE_US) {
            base_ = std::min(base_, settings.speed * 0.55f);
            output.base = base_;
            const float trim = std::clamp(static_cast<float>(settings.trim), -base_, base_);
            return drive(output, trim, 0, nowUs, settings);
        }
        wasCentered_ = false;
        base_ = 0;
        const int primary = direction_ != 0 ? direction_ : -1;
        const int direction = unavailableUs < LINE_PRIMARY_SEARCH_US ? primary : -primary;
        const int effort = std::min(settings.speed, LINE_SEARCH_PWM_LIMIT);
        output.correction = static_cast<float>(direction * effort);
        // A pivot is commanded with zero forward demand, irrespective of target speed and trim.
        // Use the same absolute limit for both wheels before individual motor compensation.
        output.left = std::clamp(wheelDuty(output.correction, 0, nowUs, settings),
                                 -LINE_SEARCH_PWM_LIMIT, LINE_SEARCH_PWM_LIMIT);
        output.right = std::clamp(wheelDuty(-output.correction, 1, nowUs, settings),
                                  -LINE_SEARCH_PWM_LIMIT, LINE_SEARCH_PWM_LIMIT);
        if (output.left == 0 || output.right == 0) output.left = output.right = 0;
        return output;
    }

    ambiguity_ = false;
    output.error = INVERT_PID_ERROR ? -line.position : line.position;
    const float magnitude = std::abs(output.error);
    if (interrupted_) {
        if (++reacquireSamples_ >= LINE_REACQUIRE_SAMPLES) {
            interrupted_ = false;
            settling_ = true;
            settledAt_ = nowUs;
        } else if (nowUs - interruptedAt_ >= LINE_LOSS_LIMIT_US) {
            output.fault = "LINE_LOST_TIMEOUT";
            return output;
        }
    }
    if (settling_ && nowUs - settledAt_ >= LINE_SETTLE_US) settling_ = false;
    output.state = interrupted_ || settling_ ? FollowState::Recover : FollowState::Tracking;
    if (!derivativeReady_) derivative_ = 0;
    else {
        const float instantaneous = std::clamp((output.error - lastError_) / dt, -65.0f, 65.0f);
        derivative_ += dt / (LINE_DERIVATIVE_TAU_S + dt) * (instantaneous - derivative_);
    }
    lastError_ = output.error;
    derivativeReady_ = true;
    const int side = magnitude >= 0.30f && line.confidence >= 0.35f ? sign(output.error) : 0;
    if (!interrupted_ && side != 0 && side == directionCandidate_) direction_ = side;
    directionCandidate_ = side;
    wasCentered_ = output.state == FollowState::Tracking && magnitude < 0.32f &&
                   std::abs(derivative_) < 3.0f && line.confidence >= 0.55f;

    const float risk = std::max({unit(magnitude / 1.9f), unit(std::abs(derivative_) / 22.0f) * 0.75f,
                                 unit((0.68f - line.confidence) / 0.50f), line.outer ? 0.65f : 0.0f});
    float target = settings.speed * (1.0f - 0.62f * risk);
    if (output.state == FollowState::Recover) target = std::min(target, settings.speed * 0.48f);
    if (interrupted_) target = 0; // Proportional steering on the first candidate, forward only after confirmation.
    base_ = std::clamp(target, std::max(0.0f, base_ - 1800.0f * dt), base_ + 340.0f * dt);
    output.base = base_;
    output.p = settings.kp * output.error;
    output.d = settings.kd * derivative_;
    const float steering = output.p + output.d;
    const float trim = std::clamp(static_cast<float>(settings.trim), -base_, base_);
    reverseAtEdge_ = line.outer && sign(steering) == sign(output.error) &&
                     magnitude >= (reverseAtEdge_ ? 1.25f : 1.55f);
    const int reverse = reverseAtEdge_ || interrupted_ ? LINE_REVERSE_PWM_LIMIT : 0;
    // A recovered centered sample must never generate motion solely due to static trim.
    const float request = interrupted_ ? std::clamp(steering, -float(reverse), float(reverse)) : steering + trim;
    return drive(output, request, reverse, nowUs, settings);
}
