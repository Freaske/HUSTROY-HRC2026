#pragma once

#include "sensors.h"
#include "tuning.h"

struct LineCalibration {
    std::array<int, SENSOR_COUNT> white;
    std::array<int, SENSOR_COUNT> black;
    std::array<int, SENSOR_COUNT> floor; // Background bound in the direction of black.
    std::array<float, SENSOR_COUNT> coordinates;
};

// V5 USB-only white and lateral tape captures. See docs/line-follow-v5.vi.md.
// A live or saved CAL command is required before analog LINE_FOLLOW can move.
constexpr LineCalibration LINE_BENCH_CALIBRATION{
    {{240, 227, 206, 197, 1186}}, {{3851, 4095, 3671, 3579, 4095}},
    {{360, 310, 320, 315, 2190}}, {{-2, -1, 0, 1, 2}}};

constexpr unsigned LINE_PERIOD_US = 10000;
constexpr unsigned LINE_STALE_US = 50000;
constexpr unsigned LINE_BLIND_BRIDGE_US = 20000;
constexpr unsigned LINE_LOSS_LIMIT_US = 300000;
constexpr unsigned LINE_AMBIGUITY_LIMIT_US = 80000;
constexpr unsigned LINE_SETTLE_US = 140000;
constexpr unsigned LINE_PRIMARY_SEARCH_US = 110000;
constexpr unsigned LINE_REACQUIRE_SAMPLES = 3;
constexpr int LINE_SEARCH_PWM_LIMIT = 55;
constexpr int LINE_REVERSE_PWM_LIMIT = 45;
constexpr float LINE_DERIVATIVE_TAU_S = 0.045f;
constexpr float LINE_MIN_PEAK = 0.14f;
constexpr float LINE_MIN_MASS = 0.21f;

const LineCalibration& lineCalibration();
bool lineCalibrationApplied();
unsigned lineCalibrationRevision();
bool lineCalibrationValid(const LineCalibration& calibration, bool inverted);

enum class LineReading { Invalid, Missing, Position, Ambiguous };
struct LineObservation {
    std::array<float, SENSOR_COUNT> strength{};
    LineReading reading = LineReading::Invalid;
    float position = 0;
    float confidence = 0;
    bool signal = false; // Qualified reflectance, including ambiguous clusters.
    bool outer = false;
};

LineObservation observeLine(const SensorFrame& frame, SensorInputMode mode,
                            const LineCalibration& calibration);

enum class FollowState { Tracking, Search, Recover, Ambiguous };
const char* followStateName(FollowState state);

struct FollowOutput {
    FollowState state = FollowState::Tracking;
    const char* fault = nullptr;
    float error = 0;
    float p = 0;
    float d = 0;
    float base = 0;
    float correction = 0;
    int left = 0;
    int right = 0;
};

class LineFollower {
public:
    FollowOutput update(const LineObservation& line, const TuningParameters& settings,
                        unsigned nowUs, unsigned dtUs);

private:
    int wheelDuty(float request, unsigned wheel, unsigned nowUs, const TuningParameters& settings);
    FollowOutput drive(FollowOutput output, float steering, int reverseLimit,
                      unsigned nowUs, const TuningParameters& settings);

    float lastError_ = 0;
    float derivative_ = 0;
    float base_ = 0;
    bool derivativeReady_ = false;
    bool interrupted_ = false;
    bool settling_ = false;
    bool wasCentered_ = false;
    bool ambiguity_ = false;
    bool reverseAtEdge_ = false;
    unsigned interruptedAt_ = 0;
    unsigned ambiguityAt_ = 0;
    unsigned settledAt_ = 0;
    unsigned reacquireSamples_ = 0;
    int direction_ = 0;
    int directionCandidate_ = 0;
    std::array<int, 2> motorSigns_{};
    std::array<unsigned, 2> motorStartedAt_{};
};
