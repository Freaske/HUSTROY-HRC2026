#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

constexpr std::size_t SENSOR_COUNT = 5;
enum class SensorInputMode { UNCONFIGURED, DIGITAL_DO, ANALOG_AO };
enum class LineState { LINE_DETECTED, LINE_LOST, INTERSECTION, INVALID };
enum class LineDirection { LEFT, RIGHT, FORWARD, INTERSECTION, LOST, INVALID };

struct SensorFrame {
    std::array<int, SENSOR_COUNT> raw{{-1, -1, -1, -1, -1}};
    std::array<bool, SENSOR_COUNT> active{}; // 1 always means ON THE LINE.
    bool rawAvailable = false;
    bool valid = false; // Requires confirmed polarity/threshold calibration.
    bool allActive = false;
    uint8_t activeCount = 0;
    float position = 0.0f; // Only usable when state == LINE_DETECTED.
    LineState state = LineState::INVALID;
    LineDirection direction = LineDirection::INVALID;
    unsigned long sampledAt = 0;
};

// Pure interpretation; original -2..+2 mean supplied by the teammate.
SensorFrame analyzeSensorPattern(const std::array<bool, SENSOR_COUNT>& active);
SensorFrame interpretSensorRaw(const std::array<int, SENSOR_COUNT>& raw,
                              SensorInputMode mode,
                              const std::array<int, SENSOR_COUNT>& thresholds,
                              bool invert, bool calibrationConfirmed);
void sensorsInit();
bool sensorsIsReady(); // Configured for raw reads; does NOT imply calibration.
SensorFrame sensorsRead(unsigned long now);
const char* lineStateName(LineState state);
const char* lineDirectionName(LineDirection direction);
