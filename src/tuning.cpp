#include "tuning.h"
#include "config.h"
#include "line_follow.h"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#ifdef ARDUINO_ARCH_ESP32
#include <Preferences.h>
#endif

namespace {
constexpr float MAX_GAIN = 1000.0f;
constexpr int MAX_TRIM = 30;
constexpr TuningParameters defaults{PID_KP, PID_KD, BASE_SPEED, LINE_FOLLOW_STEER_TRIM,
                                    LINE_FF_START, LINE_FF_SUSTAIN, LINE_FOLLOW_MAX_RUN_MS, MOTOR_TEST_TIMEOUT_MS};
constexpr bool defaultFloorsValid() {
    for (unsigned i = 0; i < 4; ++i) {
        const int limit = i % 2 == 0 ? MAX_SPEED : LINE_REVERSE_PWM_LIMIT;
        if (LINE_FF_SUSTAIN[i] < 0 || LINE_FF_START[i] < LINE_FF_SUSTAIN[i] || LINE_FF_START[i] > limit) return false;
    }
    return true;
}
static_assert(defaultFloorsValid(), "LINE_FF_START/SUSTAIN: need 0 <= sustain <= start <= limit (reverse <= LINE_REVERSE_PWM_LIMIT)");
TuningParameters parameters = defaults;
LineCalibration calibration = LINE_BENCH_CALIBRATION;
bool calibrated = false;
unsigned calibrationRevision = 0;
static_assert(PID_KP <= MAX_GAIN && PID_KD <= MAX_GAIN, "Default gains exceed tuning range");

bool validGain(float value) {
    return std::isfinite(value) && value >= 0 && value <= MAX_GAIN;
}

bool parseInt(const char*& input, long low, long high, int& value) {
    char* end = nullptr;
    errno = 0;
    const long result = std::strtol(input, &end, 10);
    if (end == input || errno == ERANGE || result < low || result > high) return false;
    value = static_cast<int>(result);
    input = end;
    return true;
}

template <std::size_t N>
bool readList(const char*& input, std::array<int, N>& values, int max) {
    for (std::size_t i = 0; i < N; ++i) {
        if (i != 0 && *input++ != ',') return false;
        if (!parseInt(input, 0, max, values[i])) return false;
    }
    return true;
}

bool validFloors(const std::array<int, 4>& start, const std::array<int, 4>& sustain) {
    for (unsigned i = 0; i < start.size(); ++i) {
        const int limit = i % 2 == 0 ? MAX_SPEED : LINE_REVERSE_PWM_LIMIT;
        if (sustain[i] < 0 || start[i] < sustain[i] || start[i] > limit) return false;
    }
    return true;
}

#ifdef ARDUINO_ARCH_ESP32
constexpr char NVS_NAMESPACE[] = "hustroy-test";
// A saved profile also records the config.h defaults it was saved with. When a default
// is changed in code (e.g. BASE_SPEED), the code value wins; the sensor calibration is kept.
constexpr char NVS_KEY[] = "line-v5-r4";
// Older layouts without the defaults record: only their sensor calibration is reused.
constexpr const char* LEGACY_NVS_KEYS[] = {"line-v5-r3", "line-v5-r1"};
struct SavedProfile { TuningParameters tuning; TuningParameters defaultsAtSave; LineCalibration cal; uint32_t hasCal; };
struct LegacyProfile { TuningParameters tuning; LineCalibration cal; uint32_t hasCal; };

bool sameTuning(const TuningParameters& a, const TuningParameters& b) {
    return a.kp == b.kp && a.kd == b.kd && a.speed == b.speed && a.trim == b.trim && a.start == b.start &&
           a.sustain == b.sustain && a.limitMs == b.limitMs && a.pulseMs == b.pulseMs;
}

bool validTuning(const TuningParameters& t) {
    return validGain(t.kp) && validGain(t.kd) && t.speed > 0 && t.speed <= MAX_SPEED &&
           t.trim >= -MAX_TRIM && t.trim <= MAX_TRIM && t.limitMs >= 250 && t.limitMs <= LINE_FOLLOW_MAX_RUN_MS &&
           t.pulseMs >= 100 && t.pulseMs <= MOTOR_TEST_TIMEOUT_MS && validFloors(t.start, t.sustain);
}

template <typename Profile>
bool loadProfile(Preferences& flash, const char* key, Profile& out) {
    return flash.getBytesLength(key) == sizeof(out) && flash.getBytes(key, &out, sizeof(out)) == sizeof(out);
}

bool saveParameters() {
    Preferences flash;
    if (!flash.begin(NVS_NAMESPACE, false)) return false;
    const SavedProfile saved{parameters, defaults, calibration, calibrated ? 1u : 0u};
    return flash.putBytes(NVS_KEY, &saved, sizeof(saved)) == sizeof(saved);
}
#endif
}

void tuningInit() {
    parameters = defaults;
    calibration = LINE_BENCH_CALIBRATION;
    if (INVERT_LINE_SENSOR) { // The measured profile uses high ADC counts for black.
        for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
            calibration.white[i] = SENSOR_ADC_MAX - calibration.white[i];
            calibration.black[i] = SENSOR_ADC_MAX - calibration.black[i];
            calibration.floor[i] = SENSOR_ADC_MAX - calibration.floor[i];
        }
    }
    calibrated = LINE_USE_BENCH_CALIBRATION && lineCalibrationValid(calibration, INVERT_LINE_SENSOR);
    ++calibrationRevision;
#ifdef ARDUINO_ARCH_ESP32
    Preferences flash;
    if (!flash.begin(NVS_NAMESPACE, true)) return;
    const auto useCalibration = [](const LineCalibration& cal, uint32_t hasCal) {
        if (hasCal == 1 && lineCalibrationValid(cal, INVERT_LINE_SENSOR)) {
            calibration = cal;
            calibrated = true;
        }
    };
    SavedProfile saved{};
    if (loadProfile(flash, NVS_KEY, saved)) {
        useCalibration(saved.cal, saved.hasCal);
        // Saved KP/KD/SPD... apply only while config.h defaults are unchanged since SAVE.
        if (sameTuning(saved.defaultsAtSave, defaults) && validTuning(saved.tuning)) parameters = saved.tuning;
        return;
    }
    for (const char* key : LEGACY_NVS_KEYS) {
        LegacyProfile legacy{};
        if (loadProfile(flash, key, legacy)) {
            useCalibration(legacy.cal, legacy.hasCal);
            return;
        }
    }
#endif
}

const TuningParameters& tuningParameters() { return parameters; }
const LineCalibration& lineCalibration() { return calibration; }
bool lineCalibrationApplied() { return calibrated; }
unsigned lineCalibrationRevision() { return calibrationRevision; }

bool tuningCommand(const char* command, bool running, char* reply, std::size_t size) {
    const auto respond = [reply, size](const char* message) { std::snprintf(reply, size, "%s", message); };
    if (std::strcmp(command, "GET") == 0) {
        const auto& t = parameters;
        std::snprintf(reply, size, "KP=%.9g KI=0 KD=%.9g SPD=%d MAX=%d TRIM=%d LIMIT=%lu PULSE=%lu "
                      "FF=%d,%d,%d,%d/%d,%d,%d,%d",
                      t.kp, t.kd, t.speed, MAX_SPEED, t.trim, t.limitMs, t.pulseMs,
                      t.start[0], t.start[1], t.start[2], t.start[3],
                      t.sustain[0], t.sustain[1], t.sustain[2], t.sustain[3]);
        return true;
    }
    if (std::strcmp(command, "CAL?") == 0) {
        const auto& c = calibration;
        std::snprintf(reply, size, "CAL=%d WHITE=%d,%d,%d,%d,%d BLACK=%d,%d,%d,%d,%d FLOOR=%d,%d,%d,%d,%d "
                      "X=%.5g,%.5g,%.5g,%.5g,%.5g",
                      calibrated, c.white[0], c.white[1], c.white[2], c.white[3], c.white[4],
                      c.black[0], c.black[1], c.black[2], c.black[3], c.black[4],
                      c.floor[0], c.floor[1], c.floor[2], c.floor[3], c.floor[4],
                      c.coordinates[0], c.coordinates[1], c.coordinates[2], c.coordinates[3], c.coordinates[4]);
        return true;
    }
    if (std::strcmp(command, "SAVE") == 0) {
        if (running) respond("ERR STOP before SAVE");
#ifdef ARDUINO_ARCH_ESP32
        else respond(saveParameters() ? "OK SAVED" : "ERR NVS write failed");
#else
        else respond("ERR NVS unavailable");
#endif
        return true;
    }
    const char* input = std::strchr(command, '=');
    if (!input) return false;
    const auto is = [command, input](const char* name) {
        const auto length = static_cast<std::size_t>(input - command);
        return length == std::strlen(name) && std::strncmp(command, name, length) == 0;
    };
    const bool kp = is("KP"), kd = is("KD"), speed = is("SPD"), trim = is("TRIM"),
               limit = is("LIMIT"), pulse = is("PULSE"), cal = is("CAL"), coords = is("X"), ff = is("FF");
    if (!(kp || kd || speed || trim || limit || pulse || cal || coords || ff)) return false;
    if (running) { respond("ERR STOP before changing settings"); return true; }
    ++input;
    if (kp || kd) {
        char* end = nullptr;
        errno = 0;
        const float gain = std::strtof(input, &end);
        if (end == input || *end || errno == ERANGE || !validGain(gain)) respond("ERR invalid gain");
        else { (kp ? parameters.kp : parameters.kd) = gain; respond("OK gain"); }
    } else if (cal) {
        auto c = calibration;
        if (readList(input, c.white, SENSOR_ADC_MAX) && *input++ == '/' &&
            readList(input, c.black, SENSOR_ADC_MAX) && *input++ == '/' &&
            readList(input, c.floor, SENSOR_ADC_MAX) && !*input && lineCalibrationValid(c, INVERT_LINE_SENSOR)) {
            calibration = c;
            calibrated = true;
            ++calibrationRevision;
            respond("OK CAL applied");
        } else respond("ERR CAL needs valid white/black/floor ADC tuples");
    } else if (coords) {
        auto c = calibration;
        bool valid = true;
        for (std::size_t i = 0; i < SENSOR_COUNT && valid; ++i) {
            char* end = nullptr;
            errno = 0;
            c.coordinates[i] = std::strtof(input, &end);
            valid = end != input && errno != ERANGE && (i + 1 == SENSOR_COUNT ? !*end : *end == ',');
            input = valid && *end ? end + 1 : end;
        }
        if (valid && lineCalibrationValid(c, INVERT_LINE_SENSOR)) { calibration = c; ++calibrationRevision; respond("OK X applied"); }
        else respond("ERR X needs 5 increasing coordinates with both sides of zero");
    } else if (ff) {
        auto t = parameters;
        if (readList(input, t.start, MAX_SPEED) && *input++ == '/' && readList(input, t.sustain, MAX_SPEED) &&
            !*input && validFloors(t.start, t.sustain)) {
            parameters = t;
            respond("OK FF applied");
        } else respond("ERR FF needs start/sustain duty tuples ordered L+,L-,R+,R-");
    } else {
        int value = 0;
        const long low = trim ? -MAX_TRIM : (limit ? 250 : (pulse ? 100 : 1));
        const long high = trim ? MAX_TRIM : (limit ? LINE_FOLLOW_MAX_RUN_MS : (pulse ? MOTOR_TEST_TIMEOUT_MS : MAX_SPEED));
        if (!parseInt(input, low, high, value) || *input) respond("ERR setting out of range");
        else {
            if (trim) parameters.trim = value;
            else if (speed) parameters.speed = value;
            else if (limit) parameters.limitMs = value;
            else parameters.pulseMs = value;
            respond("OK setting");
        }
    }
    return true;
}
