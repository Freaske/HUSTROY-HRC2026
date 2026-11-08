#include <Arduino.h>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include "hardware_config.h"
#include "modes.h"
#include "motor.h"
#include "pid.h"

namespace {
RobotStatus status;
PidController controller({PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT,
                         forwardCorrectionLimit(BASE_SPEED, MAX_SPEED, PID_CORRECTION_LIMIT)});
unsigned long lastControlAt = 0;
unsigned long lastLogAt = 0;
unsigned long motionStartedAt = 0;
unsigned int validSamples = 0;
char commandBuffer[64];
std::size_t commandLength = 0;
bool discardLine = false;
bool buttonCandidate = true;
bool buttonStable = true;
bool buttonReleased = false;
unsigned long buttonChangedAt = 0;

void printHelp() {
    Serial.println("HELP | STATUS | ARM | STOP | DISARM | ! (disarm immediately, then Enter)");
    Serial.println("MODE MOTOR_TEST | MODE SENSOR_TEST | MODE LINE_FOLLOW");
    Serial.println("MOTOR_TEST: ARM, then LEFT n | RIGHT n | BOTH n | SET left right");
    Serial.println("Signed duty: positive=forward, negative=reverse, zero=coast. LEFT/RIGHT stop the other wheel.");
    Serial.println("LINE_FOLLOW: ARM, wait for stable line, then RUN. Mode changes always DISARM.");
    Serial.printf("Duty range -%d..%d/255, manual timeout=%lu ms, RUN limit=%lu ms.\n",
                  MAX_SPEED, MAX_SPEED, MOTOR_TEST_TIMEOUT_MS, LINE_FOLLOW_MAX_RUN_MS);
}

void printStatus() {
    Serial.printf("MODE=%s ARMED=%d RUN=%d MOTOR=%s SENSORS=%s CAL=%d BIAS=%d STOP=%s\n",
                  modeName(status.mode), status.armed, status.running, motorIsReady() ? "READY" : "DISABLED/FAULT",
                  sensorsIsReady() ? "RAW_READY" : "DISABLED", SENSOR_CALIBRATION_CONFIRMED,
                  SENSOR_DISCONNECT_BIAS_CONFIRMED, status.stopReason);
}

void printSensors() {
    const auto& frame = status.sensors;
    Serial.printf("RAW=[%d,%d,%d,%d,%d] BITS=%d%d%d%d%d RAW_OK=%d VALID=%d ALL=%d ",
                  frame.raw[0], frame.raw[1], frame.raw[2], frame.raw[3], frame.raw[4],
                  frame.active[0], frame.active[1], frame.active[2], frame.active[3], frame.active[4],
                  frame.rawAvailable, frame.valid, frame.allActive);
    Serial.printf("STATE=%s DIR=%s POS=%s%.2f ERR=%.2f CORR=%.2f L=%d R=%d RUN=%d\n",
                  lineStateName(frame.state), lineDirectionName(frame.direction),
                  frame.valid && frame.state == LineState::LINE_DETECTED ? "" : "UNUSABLE:",
                  frame.position, status.error, status.correction,
                  status.leftCommand, status.rightCommand, status.running);
}

void rejectCommand() {
    modesDisarm("BAD_COMMAND");
    Serial.println("ERROR: command/mode/range invalid; stopped and DISARMED. Type HELP.");
}

void checkMotionDeadline(unsigned long now) {
    if (!status.running) return;
    if (!motorIsReady()) {
        modesDisarm("MOTOR_FAULT");
    } else if (status.mode == RobotMode::MOTOR_TEST &&
               now - motionStartedAt >= MOTOR_TEST_TIMEOUT_MS) {
        modesDisarm("TIMEOUT");
    } else if (status.mode == RobotMode::LINE_FOLLOW &&
               now - motionStartedAt >= LINE_FOLLOW_MAX_RUN_MS) {
        modesDisarm("RUN_TIMEOUT");
    } else if (status.mode == RobotMode::LINE_FOLLOW &&
               now - lastControlAt > CONTROL_MAX_GAP_MS) {
        modesDisarm("STALE_CONTROL");
    }
}

bool parseTestSpeed(const char* token, int& speed) {
    if (!token) return false;
    errno = 0;
    char* end = nullptr;
    const long value = strtol(token, &end, 10);
    if (errno == ERANGE || end == token || *end || value < -MAX_SPEED || value > MAX_SPEED) return false;
    speed = static_cast<int>(value);
    return true;
}

void processCommand(char* line, unsigned long now) {
    char* context = nullptr;
    char* command = strtok_r(line, " \t", &context);
    if (!command) return;
    char* argument = strtok_r(nullptr, " \t", &context);
    char* extra = strtok_r(nullptr, " \t", &context);
    char* tail = strtok_r(nullptr, " \t", &context);
    if (strcmp(command, "STOP") == 0) {
        if (argument) modesDisarm("BAD_STOP");
        else modesStop("STOP"); // Normal STOP retains ARM; it never restarts motion.
        Serial.println(argument ? "ERROR: STOP takes no arguments; motors stopped." : "STOP: motors stopped (coast).");
        return;
    }
    if (strcmp(command, "DISARM") == 0) {
        modesDisarm("DISARM"); // Even malformed DISARM must stop and revoke authorization.
        Serial.println(argument ? "ERROR: DISARM takes no arguments; stopped and DISARMED." : "DISARMED: both motors stopped.");
        return;
    }
    if (strcmp(command, "HELP") == 0 && !argument) { printHelp(); return; }
    if (strcmp(command, "STATUS") == 0 && !argument) { printStatus(); return; }
    if (strcmp(command, "ARM") == 0 && !argument) { modesArm(); return; }
    if (strcmp(command, "RUN") == 0 && !argument) { modesRun(now); return; }
    if (strcmp(command, "MODE") == 0 && argument && !extra) {
        if (strcmp(argument, "MOTOR_TEST") == 0) modesSelect(RobotMode::MOTOR_TEST, now);
        else if (strcmp(argument, "SENSOR_TEST") == 0) modesSelect(RobotMode::SENSOR_TEST, now);
        else if (strcmp(argument, "LINE_FOLLOW") == 0) modesSelect(RobotMode::LINE_FOLLOW, now);
        else rejectCommand();
        return;
    }
    const bool left = strcmp(command, "LEFT") == 0;
    const bool right = strcmp(command, "RIGHT") == 0;
    const bool both = strcmp(command, "BOTH") == 0;
    const bool separate = strcmp(command, "SET") == 0;
    if ((!left && !right && !both && !separate) || !argument ||
        (separate ? (!extra || tail) : extra != nullptr) || status.mode != RobotMode::MOTOR_TEST) {
        rejectCommand();
        return;
    }
    int first = 0;
    int second = 0;
    if (!parseTestSpeed(argument, first) || (separate && !parseTestSpeed(extra, second))) {
        rejectCommand();
        return;
    }
    const int leftSpeed = right ? 0 : first;
    const int rightSpeed = separate ? second : (left ? 0 : first);
    if (leftSpeed == 0 && rightSpeed == 0) {
        modesStop("ZERO");
        Serial.println("STOP: zero command (coast).");
        return;
    }
    if (!motorIsReady() || !status.armed) {
        modesDisarm("MOTOR_NOT_ARMED");
        Serial.println("BLOCKED: motor hardware disabled/fault or DISARMED; ARM is required.");
        return;
    }
    status.leftCommand = leftSpeed;
    status.rightCommand = rightSpeed;
    setMotorSpeeds(status.leftCommand, status.rightCommand);
    status.running = motorIsReady();
    motionStartedAt = now;
    if (!status.running) modesDisarm("MOTOR_FAULT");
    else Serial.printf("TEST: left=%d right=%d; TIMEOUT in %lu ms.\n",
                       status.leftCommand, status.rightCommand, MOTOR_TEST_TIMEOUT_MS);
}

void pollSerial() {
    for (unsigned int count = 0; count < 64 && Serial.available() > 0; ++count) {
        checkMotionDeadline(millis());
        const char c = static_cast<char>(Serial.read());
        if (c == '!') {
            modesDisarm("EMERGENCY_COMMAND");
            commandLength = 0;
            discardLine = true;
            Serial.println("STOP + DISARM: ! received; press Enter before the next command.");
        } else if (c == '\r' || c == '\n') {
            if (!discardLine) {
                commandBuffer[commandLength] = '\0';
                processCommand(commandBuffer, millis());
            }
            commandLength = 0;
            discardLine = false;
        } else if (!discardLine) {
            if ((c != '\t' && (c < 32 || c > 126)) || commandLength >= sizeof(commandBuffer) - 1) {
                rejectCommand();
                commandLength = 0;
                discardLine = true;
            } else commandBuffer[commandLength++] = c;
        }
    }
}

void pollButton(unsigned long now) {
    if (!MODE_BUTTON_ENABLED) return;
    const bool high = digitalRead(MODE_BUTTON_PIN) == HIGH;
    if (high != buttonCandidate) { buttonCandidate = high; buttonChangedAt = now; }
    if (now - buttonChangedAt < MODE_BUTTON_DEBOUNCE_MS || high == buttonStable) return;
    buttonStable = high;
    if (high) buttonReleased = true;
    else if (buttonReleased) {
        buttonReleased = false;
        const RobotMode next = status.mode == RobotMode::MOTOR_TEST ? RobotMode::SENSOR_TEST :
                               status.mode == RobotMode::SENSOR_TEST ? RobotMode::LINE_FOLLOW : RobotMode::MOTOR_TEST;
        modesSelect(next, now); // Button only selects, NEVER arms LINE_FOLLOW.
    }
}
} // namespace

const char* modeName(RobotMode mode) {
    switch (mode) {
        case RobotMode::MOTOR_TEST: return "MOTOR_TEST";
        case RobotMode::SENSOR_TEST: return "SENSOR_TEST";
        case RobotMode::LINE_FOLLOW: return "LINE_FOLLOW";
        default: return "INVALID";
    }
}
const RobotStatus& modesStatus() { return status; }

void modesStop(const char* reason) {
    const bool wasRunning = status.running;
    motorStop();
    status.running = false;
    status.leftCommand = status.rightCommand = 0;
    status.error = status.correction = 0;
    status.stopReason = reason;
    validSamples = 0;
    controller.reset();
    if (wasRunning) Serial.printf("STOP: %s (coast). Explicit command required to restart.\n", reason);
}

void modesDisarm(const char* reason) {
    status.armed = false;
    modesStop(reason);
}

bool modesArm() {
    modesDisarm("ARM_REQUEST"); // ARM itself never moves the wheels, even if repeated.
    if (!motorIsReady() || status.mode == RobotMode::SENSOR_TEST) {
        Serial.println("BLOCKED: ARM needs motor hardware READY and MOTOR_TEST or LINE_FOLLOW.");
        return false;
    }
    status.armed = true;
    Serial.println("ARMED: motors remain stopped. Send a test command, or wait for a stable line then RUN.");
    return true;
}

void modesSelect(RobotMode mode, unsigned long now) {
    modesDisarm("MODE_CHANGE");
    if (mode != RobotMode::MOTOR_TEST && mode != RobotMode::SENSOR_TEST && mode != RobotMode::LINE_FOLLOW) return;
    status.mode = mode;
    validSamples = 0;
    lastControlAt = lastLogAt = now;
    Serial.printf("MODE=%s STOPPED and DISARMED. ARM required before motion.\n", modeName(mode));
}

bool modesRun(unsigned long now) {
    if (status.running) {
        // Repeated RUN cannot keep extending a finite trial.
        Serial.println("BLOCKED: already running; STOP before a new RUN.");
        return false;
    }
    if (!status.armed || status.mode != RobotMode::LINE_FOLLOW || !motorIsReady() || !sensorsIsReady() ||
        !SENSOR_CALIBRATION_CONFIRMED || !SENSOR_DISCONNECT_BIAS_CONFIRMED ||
        !status.sensors.valid || status.sensors.state != LineState::LINE_DETECTED ||
        now - status.sensors.sampledAt > CONTROL_MAX_GAP_MS || validSamples < LINE_START_VALID_SAMPLES ||
        forwardCorrectionLimit(BASE_SPEED, MAX_SPEED, PID_CORRECTION_LIMIT) <= 0) {
        modesDisarm("RUN_BLOCKED");
        Serial.println("BLOCKED: RUN needs ARM + LINE_FOLLOW, motor/sensors ready, calibration/bias and fresh stable line.");
        return false;
    }
    controller.reset();
    status.running = true;
    motionStartedAt = lastControlAt = now;
    Serial.println("RUN armed; first wheel command occurs at the next control tick.");
    return true;
}

void modesInit(unsigned long now) {
    status = RobotStatus{};
    modesSelect(RobotMode::MOTOR_TEST, now);
    if (MODE_BUTTON_ENABLED) {
        pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
        buttonCandidate = buttonStable = digitalRead(MODE_BUTTON_PIN) == HIGH;
        buttonReleased = buttonStable; // A held button at boot must be released first.
        buttonChangedAt = now;
    }
    Serial.println("HUSTROY integrated firmware: startup STOPPED and DISARMED. Type HELP.");
}

void modesUpdate(unsigned long now) {
    checkMotionDeadline(now);
    pollButton(now);
    const unsigned long elapsed = now - lastControlAt;
    if (elapsed >= CONTROL_PERIOD_MS) {
        lastControlAt = now; // One update, never a burst of catch-up commands.
        status.sensors = sensorsRead(now);
        if (status.sensors.valid && status.sensors.state == LineState::LINE_DETECTED) {
            if (validSamples < LINE_START_VALID_SAMPLES) ++validSamples;
        } else validSamples = 0;
        if (status.mode == RobotMode::LINE_FOLLOW && status.running) {
            if (!status.sensors.valid || status.sensors.state != LineState::LINE_DETECTED) {
                modesDisarm(lineStateName(status.sensors.state)); // Latched stop; no blind recovery.
            } else {
                status.error = INVERT_PID_ERROR ? -status.sensors.position : status.sensors.position;
                const PidResult pid = controller.update(status.error, static_cast<float>(elapsed) / 1000.0f);
                const WheelSpeeds speeds = mixForwardSpeeds(BASE_SPEED, pid.output, MAX_SPEED);
                if (!pid.valid || !speeds.valid) modesDisarm("PID_INVALID");
                else {
                    status.correction = pid.output;
                    status.leftCommand = speeds.left;
                    status.rightCommand = speeds.right;
                    setMotorSpeeds(speeds.left, speeds.right);
                    if (!motorIsReady()) modesDisarm("MOTOR_FAULT");
                }
            }
        }
    }
    pollSerial();
    checkMotionDeadline(millis());
    if (status.mode != RobotMode::MOTOR_TEST && now - lastLogAt >= DEBUG_LOG_INTERVAL_MS) {
        lastLogAt = now;
        printSensors();
    }
}
