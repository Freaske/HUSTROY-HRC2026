#include <Arduino.h>
#include <algorithm>
#include "sensors.h"
#include "hardware_config.h"

namespace {
constexpr int ANALOG_AMBIGUITY_MARGIN = 8; // ADC counts above or below the calibrated threshold.
constexpr int ANALOG_DOMINANT_PEAK_MIN_MARGIN = 24;
bool ready = false;
}

SensorFrame analyzeSensorPattern(const std::array<bool, SENSOR_COUNT>& active) {
    // Attribution: Line_Follower.zip, src/main.cpp and sensor_logic_backup.txt.
    SensorFrame frame;
    frame.active = active;
    frame.valid = true;
    int sum = 0;
    int first = -1;
    int last = -1;
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        if (!active[i]) continue;
        sum += static_cast<int>(i) - 2;
        ++frame.activeCount;
        if (first < 0) first = static_cast<int>(i);
        last = static_cast<int>(i);
    }
    frame.allActive = frame.activeCount == SENSOR_COUNT;
    if (frame.activeCount == 0) {
        frame.state = LineState::LINE_LOST;
        frame.direction = LineDirection::LOST;
    } else {
        frame.position = static_cast<float>(sum) / frame.activeCount;
        if (frame.allActive || (active.front() && active.back())) {
            // Preserve backup's conservative outer-both intersection rule.
            frame.state = LineState::INTERSECTION;
            frame.direction = LineDirection::INTERSECTION;
        } else if (last - first + 1 != frame.activeCount) {
            // Multiple separated patches cannot be treated as one centered line.
            frame.valid = false;
        } else {
            frame.state = LineState::LINE_DETECTED;
            frame.direction = frame.position < 0 ? LineDirection::LEFT :
                              frame.position > 0 ? LineDirection::RIGHT : LineDirection::FORWARD;
        }
    }
    return frame;
}

SensorFrame interpretSensorRaw(const std::array<int, SENSOR_COUNT>& raw,
                              SensorInputMode mode,
                              const std::array<int, SENSOR_COUNT>& thresholds,
                              bool invert, bool calibrationConfirmed) {
    SensorFrame frame;
    frame.raw = raw;
    if (mode != SensorInputMode::DIGITAL_DO && mode != SensorInputMode::ANALOG_AO) return frame;
    std::array<bool, SENSOR_COUNT> active{};
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        if (raw[i] < 0 || raw[i] > (mode == SensorInputMode::DIGITAL_DO ? 1 : SENSOR_ADC_MAX)) return frame;
        if (mode == SensorInputMode::ANALOG_AO && (thresholds[i] <= 0 || thresholds[i] >= SENSOR_ADC_MAX)) return frame;
        const bool high = mode == SensorInputMode::DIGITAL_DO ? raw[i] == 1 : raw[i] >= thresholds[i];
        active[i] = invert ? !high : high;
    }
    frame = analyzeSensorPattern(active);
    if (mode == SensorInputMode::ANALOG_AO && !frame.valid) {
        // A weak, separate patch can flicker across its threshold. Recover only
        // if a single contiguous line remains clearly above the noise margin.
        std::array<bool, SENSOR_COUNT> confident{};
        std::array<int, SENSOR_COUNT> signedDistance{};
        int peakMargin = raw[0] - thresholds[0];
        if (invert) peakMargin = -peakMargin;
        for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
            const int distance = raw[i] - thresholds[i];
            signedDistance[i] = invert ? -distance : distance;
            peakMargin = std::max(peakMargin, signedDistance[i]);
            confident[i] = signedDistance[i] >= ANALOG_AMBIGUITY_MARGIN;
        }
        const auto candidate = analyzeSensorPattern(confident);
        if (candidate.valid && candidate.state == LineState::LINE_DETECTED) {
            frame = candidate;
        } else if (peakMargin >= ANALOG_DOMINANT_PEAK_MIN_MARGIN) {
            // Reject motor-load baseline shift when one contiguous peak dominates.
            const int cutoff = std::max(ANALOG_AMBIGUITY_MARGIN, peakMargin / 2);
            std::array<bool, SENSOR_COUNT> dominant{};
            for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
                dominant[i] = signedDistance[i] >= cutoff;
            }
            const auto dominantCandidate = analyzeSensorPattern(dominant);
            if (dominantCandidate.valid && dominantCandidate.state == LineState::LINE_DETECTED) {
                frame = dominantCandidate;
            }
        }
    }
    frame.raw = raw;
    frame.rawAvailable = true;
    if (!calibrationConfirmed) {
        // Keep raw and provisional bits/position for calibration, never use to drive.
        frame.valid = false;
        frame.state = LineState::INVALID;
        frame.direction = LineDirection::INVALID;
    }
    return frame;
}

void sensorsInit() {
    if (!SENSOR_HARDWARE_ENABLED || !hardwareSensorsValid()) {
        Serial.println("SENSORS DISABLED: confirm AO/DO, GPIOs and safe signal voltages in config.h.");
        return;
    }
    for (int pin : SENSOR_PINS) pinMode(pin, INPUT); // No phantom pullups on GPIO34..39.
    if (SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO) {
        analogReadResolution(SENSOR_ADC_BITS);
        for (int pin : SENSOR_PINS) analogSetPinAttenuation(pin, ADC_11db);
    }
    ready = true;
    Serial.println("SENSORS: raw reads ready; LINE_FOLLOW additionally requires confirmed calibration/bias.");
}

bool sensorsIsReady() { return ready; }

SensorFrame sensorsRead(unsigned long now) {
    SensorFrame frame;
    if (ready) {
        std::array<int, SENSOR_COUNT> raw{};
        for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
            raw[i] = SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO ?
                     analogRead(SENSOR_PINS[i]) : digitalRead(SENSOR_PINS[i]);
        }
        frame = interpretSensorRaw(raw, SENSOR_INPUT_MODE, SENSOR_ANALOG_THRESHOLDS,
                                   INVERT_LINE_SENSOR, SENSOR_CALIBRATION_CONFIRMED);
    }
    frame.sampledAt = now;
    return frame;
}

const char* lineStateName(LineState state) {
    switch (state) {
        case LineState::LINE_DETECTED: return "LINE_DETECTED";
        case LineState::LINE_LOST: return "LINE_LOST";
        case LineState::INTERSECTION: return "INTERSECTION";
        default: return "INVALID";
    }
}
const char* lineDirectionName(LineDirection direction) {
    switch (direction) {
        case LineDirection::LEFT: return "LEFT";
        case LineDirection::RIGHT: return "RIGHT";
        case LineDirection::FORWARD: return "FORWARD";
        case LineDirection::INTERSECTION: return "INTERSECTION";
        case LineDirection::LOST: return "LOST";
        default: return "INVALID";
    }
}
