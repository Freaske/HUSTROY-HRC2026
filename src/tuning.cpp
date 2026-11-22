#include "tuning.h"
#include "config.h"
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
constexpr TuningParameters defaults{PID_KP, PID_KD, BASE_SPEED};
TuningParameters parameters = defaults;
static_assert(PID_KP <= MAX_GAIN && PID_KD <= MAX_GAIN, "Default gains exceed tuning range");

bool validGain(float value) {
    return std::isfinite(value) && value >= 0 && value <= MAX_GAIN;
}

#ifdef ARDUINO_ARCH_ESP32
constexpr char NVS_NAMESPACE[] = "hustroy-test";
constexpr char NVS_KEY[] = "tuning-v1";
// Store all values in one versioned NVS entry so SAVE commits them together.
struct SavedTuning { float kp; float kd; int32_t speed; };

bool saveParameters() {
    Preferences flash;
    if (!flash.begin(NVS_NAMESPACE, false)) return false;
    const SavedTuning saved{parameters.kp, parameters.kd, parameters.speed};
    return flash.putBytes(NVS_KEY, &saved, sizeof(saved)) == sizeof(saved);
}
#endif
}

void tuningInit() {
    parameters = defaults;
#ifdef ARDUINO_ARCH_ESP32
    Preferences flash;
    SavedTuning saved{};
    if (flash.begin(NVS_NAMESPACE, true) && flash.getBytesLength(NVS_KEY) == sizeof(saved) &&
        flash.getBytes(NVS_KEY, &saved, sizeof(saved)) == sizeof(saved) &&
        validGain(saved.kp) && validGain(saved.kd) && saved.speed > 0 && saved.speed <= MAX_SPEED) {
        parameters = {saved.kp, saved.kd, saved.speed};
    }
#endif
}

const TuningParameters& tuningParameters() { return parameters; }

bool tuningCommand(const char* command, bool running, char* reply, std::size_t size) {
    if (std::strcmp(command, "GET") == 0) {
        std::snprintf(reply, size, "KP=%.9g KI=%.9g KD=%.9g SPD=%d MAX=%d",
                      parameters.kp, PID_KI, parameters.kd, parameters.speed, MAX_SPEED);
        return true;
    }
    if (std::strcmp(command, "SAVE") == 0) {
        const char* message = "ERR NVS unavailable";
        if (running) message = "ERR STOP before SAVE";
#ifdef ARDUINO_ARCH_ESP32
        else message = saveParameters() ? "OK SAVED" : "ERR NVS write failed";
#endif
        std::snprintf(reply, size, "%s", message);
        return true;
    }
    char* end = nullptr;
    errno = 0;
    if (std::strncmp(command, "SPD=", 4) == 0) {
        const char* value = command + 4;
        const long speed = std::strtol(value, &end, 10);
        if (end == value || *end || errno == ERANGE || speed < 1 || speed > MAX_SPEED) {
            std::snprintf(reply, size, "ERR SPD must be an integer in 1..%d", MAX_SPEED);
        } else {
            parameters.speed = static_cast<int>(speed);
            std::snprintf(reply, size, "OK SPD=%d", parameters.speed);
        }
        return true;
    }
    const bool proportional = std::strncmp(command, "KP=", 3) == 0;
    if (!proportional && std::strncmp(command, "KD=", 3) != 0) return false;
    const char* value = command + 3;
    const float gain = std::strtof(value, &end);
    if (end == value || *end || errno == ERANGE || !validGain(gain)) {
        std::snprintf(reply, size, "ERR %.2s must be finite in 0..%.0f", command, MAX_GAIN);
    } else {
        (proportional ? parameters.kp : parameters.kd) = gain;
        std::snprintf(reply, size, "OK %.2s=%.9g", command, gain);
    }
    return true;
}
