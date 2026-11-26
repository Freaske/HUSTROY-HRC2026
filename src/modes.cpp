// =============================================================================
// modes.cpp — "Bộ não" của robot: quản lý chế độ, an toàn và vòng điều khiển.
// - Nhận lệnh qua Serial (HELP, STATUS, ARM, RUN, STOP, DISARM, !, MODE ...,
//   LEFT/RIGHT/BOTH/SET n) và nút chọn chế độ (nếu bật).
// - Mỗi CONTROL_PERIOD_MS: đọc cảm biến; ở LINE_FOLLOW thì tính PID và đặt
//   tốc độ motor. Mất line -> dừng (hoặc tạm dừng chờ line nếu LINE_AUTO_RESUME).
// - Các khóa an toàn: phải ARM mới chạy, timeout thử motor, chặn khi vòng điều
//   khiển bị trễ, lỗi cảm biến/motor -> dừng và DISARM.
// - Tự khởi động dò line sau khi bật nguồn nếu AUTO_START_ON_BOOT = true.
// =============================================================================
#include <Arduino.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#ifdef ARDUINO_ARCH_ESP32
#include <Preferences.h>
#endif
#include "hardware_config.h"
#include "modes.h"
#include "motor.h"
#include "line_follow.h"
#include "tuning.h"

namespace {
constexpr std::size_t CENTER_SENSOR = SENSOR_COUNT / 2;
constexpr int RIGHT_LINE_CENTER_CONTRAST = 12; // Minimum AO contrast against each neighbor.
constexpr unsigned long SENSOR_TEST_LOG_INTERVAL_MS = 180;
RobotStatus status;
LineFollower follower;
LineObservation observation;
FollowOutput followOutput;
unsigned long lastControlAt = 0;
unsigned long lastLogAt = 0;
unsigned long motionStartedAt = 0;
unsigned long motorTestDuration = 0;
bool bootBenchRunning = false;
unsigned validSamples = 0;
unsigned lastLineTick = 0;
unsigned nextLineTick = 0;
unsigned lastSampleUs = 0;
unsigned adcUs = 0;
unsigned sampleRevision = 0;
bool traceEnabled = false;
char lastSummary[768]{};
char commandBuffer[128];
std::size_t commandLength = 0;
bool discardLine = false;

struct RunStats {
    unsigned start = 0;
    unsigned observed = 0;
    unsigned detected = 0;
    unsigned lost = 0;
    unsigned zeroEpisodes = 0;
    unsigned binaryZeroUs = 0;
    unsigned binaryZeroEpisodes = 0;
    unsigned zeroDuration = 0;
    unsigned maxZero = 0;
    unsigned usable = 0;
    unsigned edge = 0;
    unsigned tracking = 0;
    unsigned search = 0;
    unsigned recover = 0;
    unsigned ambiguous = 0;
    unsigned recoveries = 0;
    unsigned blindForward = 0;
    unsigned reverse = 0;
    unsigned samples = 0;
    unsigned missed = 0;
    unsigned late = 0;
    unsigned maxPeriod = 0;
    unsigned minPeriod = LINE_STALE_US;
    unsigned maxWork = 0;
    unsigned maxAdc = 0;
    unsigned txDrops = 0;
    bool zero = false;
    bool binaryZero = false;
    bool sampled = false;
    bool recovering = false;
    double errorArea = 0;
    std::array<int, 2> wheelSigns{};
    LineObservation lastLine;
    FollowOutput lastOutput;

    void advance(unsigned interval) {
        if (!sampled) return;
        // Credit only one scheduled period to a sample. Report remaining wall time as unknown.
        const unsigned dt = std::min(interval, LINE_PERIOD_US);
        observed += dt;
        if (lastLine.signal) detected += dt;
        if (binaryZero) binaryZeroUs += dt;
        if (zero) { lost += dt; maxZero = std::max(maxZero, zeroDuration + dt); zeroDuration += dt; }
        if (lastLine.reading == LineReading::Position) {
            usable += dt;
            errorArea += std::abs(lastOutput.error) * dt;
            if (lastLine.outer) edge += dt;
        }
        switch (lastOutput.state) {
            case FollowState::Tracking: tracking += dt; break;
            case FollowState::Search: search += dt; break;
            case FollowState::Recover: recover += dt; break;
            case FollowState::Ambiguous: ambiguous += dt; break;
        }
        if (zero && lastOutput.left + lastOutput.right > 0 &&
            lastOutput.left >= 0 && lastOutput.right >= 0) blindForward += dt;
    }

    void record(const LineObservation& line, const FollowOutput& output, const SensorFrame& raw) {
        const bool bitsZero = raw.rawAvailable && raw.activeCount == 0;
        if (bitsZero && !binaryZero) ++binaryZeroEpisodes;
        binaryZero = bitsZero;
        const bool noSignal = line.reading == LineReading::Missing;
        if (noSignal && !zero) { ++zeroEpisodes; zeroDuration = 0; }
        zero = noSignal;
        if (output.state == FollowState::Search || output.state == FollowState::Ambiguous) recovering = true;
        if (recovering && output.state == FollowState::Tracking && !output.fault) {
            ++recoveries;
            recovering = false;
        }
        const int wheels[] = {output.left, output.right};
        for (unsigned i = 0; i < 2; ++i) {
            const int sign = (wheels[i] > 0) - (wheels[i] < 0);
            if (sign != 0) {
                if (wheelSigns[i] != 0 && wheelSigns[i] != sign) ++reverse;
                wheelSigns[i] = sign;
            }
        }
        lastLine = line;
        lastOutput = output;
        sampled = true;
    }
} stats;

bool writeSerial(const char* text, std::size_t length) {
    // Leave room for the ESP32 UART TX ring item header/alignment.
    if (Serial.availableForWrite() < static_cast<int>(length + 20)) return false;
    return Serial.write(reinterpret_cast<const uint8_t*>(text), length) == length;
}

template <typename... Args>
void logf(const char* format, Args... args) {
    char text[512];
    const int size = std::snprintf(text, sizeof(text), format, args...);
    if (size <= 0 || static_cast<std::size_t>(size) >= sizeof(text)) return;
    if (status.running) {
        if (!writeSerial(text, size) && status.mode == RobotMode::LINE_FOLLOW) ++stats.txDrops;
    } else Serial.write(reinterpret_cast<const uint8_t*>(text), size);
}

void logLine(const char* text) { logf("%s\n", text); }

bool lineStartReady() {
    return observation.reading == LineReading::Position && std::abs(observation.position) <= 1.1f &&
           observation.confidence >= 0.42f;
}

bool buttonCandidate = true;
bool buttonStable = true;
bool buttonReleased = false;
unsigned long buttonChangedAt = 0;
bool autoStartPending = false;
unsigned long bootAt = 0;
bool bootCandidate = true;
bool bootButtonInitialized = false;
bool bootStable = true;
bool bootArmedForStart = false;
bool bootInhibitUntilRelease = false;
unsigned long bootChangedAt = 0;
bool bootCountdownActive = false;
unsigned long bootCountdownStartedAt = 0;

bool rightLineIsCentered(const SensorFrame& frame) {
    if (!frame.rawAvailable || !frame.active[CENTER_SENSOR] || frame.activeCount > 3) return false;
    if (SENSOR_INPUT_MODE != SensorInputMode::ANALOG_AO) return frame.activeCount == 1;

    // Compare neighboring ADC readings so a shift shared by all eyes does not
    // look like a loss of the centered line when the right motor starts.
    const int direction = INVERT_LINE_SENSOR ? -1 : 1;
    const int center = frame.raw[CENTER_SENSOR];
    if (direction * (center - frame.raw[CENTER_SENSOR - 1]) >= RIGHT_LINE_CENTER_CONTRAST &&
        direction * (center - frame.raw[CENTER_SENSOR + 1]) >= RIGHT_LINE_CENTER_CONTRAST) return true;

    // A wider stripe can cover the center and a neighbor. In that case use
    // the eyes outside the single stripe as the background reference.
    const auto pattern = analyzeSensorPattern(frame.active);
    if (!pattern.valid || pattern.state != LineState::LINE_DETECTED) return false;
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        if (!frame.active[i] && direction * (center - frame.raw[i]) < RIGHT_LINE_CENTER_CONTRAST) return false;
    }
    return true;
}

// In danh sách lệnh Serial (lệnh HELP).
void printHelp() {
    logLine("HELP | STATUS | ARM | STOP | DISARM | ! (disarm immediately, then Enter)");
    logLine("MODE MOTOR_TEST | MODE SENSOR_TEST | MODE LINE_FOLLOW | MODE RIGHT_LINE");
    logLine("MOTOR_TEST: ARM, then LEFT n | RIGHT n | BOTH n | SET left right");
    if (BOOT_BENCH_PROFILE) logf("BOOT: press and release for BOTH %d, %lu ms. Press again to stop.\n",
                                BOOT_BENCH_PWM, BOOT_BENCH_RUN_MS);
    logLine("Signed duty: positive=forward, negative=reverse, zero=coast. LEFT/RIGHT stop the other wheel.");
    logLine("LINE_FOLLOW: CAL=white/black/floor, center line, ARM, then GO. Mode changes DISARM.");
    logf("RIGHT_LINE: ARM, center the line, then RUN. Right PWM %d, up to %lu ms.\n",
                  RIGHT_LINE_TEST_PWM, RIGHT_LINE_TEST_MAX_RUN_MS);
    logLine("TUNING while stopped: KP= | KD= | SPD= | TRIM= | LIMIT= | PULSE= | FF= | X= | CAL? | GET | SAVE");
    logLine("TRACE=1/0 samples at 50 Hz in sensor and bench modes. LAST shows latest run.");
    logf("Duty range -%d..%d/255, manual timeout=%lu ms, RUN limit=%lu ms.\n",
                  MAX_SPEED, MAX_SPEED, MOTOR_TEST_TIMEOUT_MS, LINE_FOLLOW_MAX_RUN_MS);
}

// In trạng thái tổng quát: chế độ, ARM, RUN, motor, cảm biến, lý do dừng (lệnh STATUS).
void printStatus() {
    logf("MODE=%s ARMED=%d RUN=%d MOTOR=%s SENSORS=%s CAL=%d BIAS=%d STOP=%s\n",
                  modeName(status.mode), status.armed, status.running, motorIsReady() ? "READY" : "DISABLED/FAULT",
                  sensorsIsReady() ? "RAW_READY" : "DISABLED", SENSOR_CALIBRATION_CONFIRMED,
                  SENSOR_DISCONNECT_BIAS_CONFIRMED, status.stopReason);
    // Configuration/request diagnostics, NOT measured voltage or wheel motion.
    logf("FW=%s LED=GPIO%u DOUBLE_BLINK PWM=%u-bit/%luHz "
                  "A(PWM/IN1/IN2)=%d/%d/%d B=%d/%d/%d STBY=%d L_REQ=%d R_REQ=%d TRIM=%d\n",
                  FIRMWARE_ID, static_cast<unsigned int>(STATUS_LED_PIN),
                  static_cast<unsigned int>(MOTOR_PWM_BITS), static_cast<unsigned long>(MOTOR_PWM_FREQUENCY_HZ),
                  MOTOR_PWMA_PIN, MOTOR_AIN1_PIN, MOTOR_AIN2_PIN,
                  MOTOR_PWMB_PIN, MOTOR_BIN1_PIN, MOTOR_BIN2_PIN, MOTOR_STBY_PIN,
                  status.leftCommand, status.rightCommand, tuningParameters().trim);
    if (status.mode == RobotMode::RIGHT_LINE) {
        logf("RIGHT_LINE_CENTERED=%d PWM=%d LIMIT_MS=%lu\n",
                      rightLineIsCentered(status.sensors), RIGHT_LINE_TEST_PWM, RIGHT_LINE_TEST_MAX_RUN_MS);
    }
}

void printSensorSample(const SensorFrame& frame) {
    logf("RAW=[%d,%d,%d,%d,%d] BITS=%d%d%d%d%d RAW_OK=%d VALID=%d ALL=%d",
                  frame.raw[0], frame.raw[1], frame.raw[2], frame.raw[3], frame.raw[4],
                  frame.active[0], frame.active[1], frame.active[2], frame.active[3], frame.active[4],
                  frame.rawAvailable, frame.valid, frame.allActive);
}

void printSensorDiagnostic(const SensorFrame& frame) {
    const bool analog = SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO;
    const char* input = SENSOR_INPUT_MODE == SensorInputMode::DIGITAL_DO ? "DIG" : "RAW";
    if (sensorsIsReady()) {
        logf("%s:[%d,%d,%d,%d,%d]", input,
                      frame.raw[0], frame.raw[1], frame.raw[2], frame.raw[3], frame.raw[4]);
    } else {
        logf("%s:N/A", input);
    }
    if (analog) {
        const auto& thresholds = SENSOR_ANALOG_THRESHOLDS;
        logf(" | THR:[%d,%d,%d,%d,%d]",
                      thresholds[0], thresholds[1], thresholds[2], thresholds[3], thresholds[4]);
    }
    // Comparing counts to configuration cannot verify the physical black/white mapping.
    if (!frame.rawAvailable) {
        logLine(" | BIN:N/A | ACT:N/A | CLASS:UNVERIFIED");
        return;
    }
    logf(" | BIN:%d%d%d%d%d | ACT:%u/5 | CLASS:UNVERIFIED\n",
                  frame.active[0], frame.active[1], frame.active[2], frame.active[3], frame.active[4],
                  static_cast<unsigned int>(frame.activeCount));
}

void printSensors() {
    const auto& frame = status.sensors;
    printSensorSample(frame);
    logf(" STATE=%s DIR=%s POS=%s%.2f ERR=%.2f CORR=%.2f L=%d R=%d RUN=%d\n",
                  lineStateName(frame.state), lineDirectionName(frame.direction),
                  frame.valid && frame.state == LineState::LINE_DETECTED ? "" : "UNUSABLE:",
                  frame.position, status.error, status.correction,
                  status.leftCommand, status.rightCommand, status.running);
}

void printTrace() {
    const auto& r = status.sensors.raw;
    const auto& b = status.sensors.active;
    const auto& n = observation.strength;
    const bool following = status.running && status.mode == RobotMode::LINE_FOLLOW;
    logf("LINE T_US=%u RAW=%d,%d,%d,%d,%d NORM=%.3f,%.3f,%.3f,%.3f,%.3f BIN=%d%d%d%d%d "
         "POS=%s%.3f CONF=%.3f SIGNAL=%d EDGE=%d P=%.2f D=%.2f BASE=%.2f L=%d R=%d STATE=%s ADC_US=%u\n",
         lastSampleUs, r[0], r[1], r[2], r[3], r[4], n[0], n[1], n[2], n[3], n[4],
         b[0], b[1], b[2], b[3], b[4], observation.reading == LineReading::Position ? "" : "NA:",
         observation.position, observation.confidence, observation.signal, observation.outer,
         followOutput.p, followOutput.d, followOutput.base, status.leftCommand, status.rightCommand,
         following ? followStateName(followOutput.state) : modeName(status.mode), adcUs);
}

void printProfile() {
    char settings[192], cal[192];
    tuningCommand("GET", false, settings, sizeof(settings));
    tuningCommand("CAL?", false, cal, sizeof(cal));
    logf("LINE_CFG FW=%s PERIOD_US=%u LOSS_US=%u BRIDGE_US=%u AMBIG_US=%u SETTLE_US=%u "
         "SEARCH_SWITCH_US=%u SEARCH_PWM=%d REV_PWM=%d REACQ=%u D_TAU=%.3f PEAK=%.2f MASS=%.2f %s\n",
         FIRMWARE_ID, LINE_PERIOD_US, LINE_LOSS_LIMIT_US, LINE_BLIND_BRIDGE_US, LINE_AMBIGUITY_LIMIT_US,
         LINE_SETTLE_US, LINE_PRIMARY_SEARCH_US, LINE_SEARCH_PWM_LIMIT, LINE_REVERSE_PWM_LIMIT,
         LINE_REACQUIRE_SAMPLES, LINE_DERIVATIVE_TAU_S, LINE_MIN_PEAK, LINE_MIN_MASS, settings);
    auto& t = SENSOR_ANALOG_THRESHOLDS;
    logf("LINE_CAL INVERT=%d STEER_INVERT=%d MOTOR=%d,%d,%d THR=%d,%d,%d,%d,%d %s\n", INVERT_LINE_SENSOR,
         INVERT_PID_ERROR, LEFT_MOTOR_IS_CHANNEL_A, INVERT_LEFT_MOTOR, INVERT_RIGHT_MOTOR,
         t[0], t[1], t[2], t[3], t[4], cal);
}

void stopForLine(const char* reason, unsigned long now) {
    const int previousLeft = status.leftCommand;
    const int previousRight = status.rightCommand;
    modesDisarm(reason);
    logf("LINE_STOP REASON=%s T_MS=%lu PREV_L=%d PREV_R=%d ",
                  reason, now - motionStartedAt, previousLeft, previousRight);
    printSensorSample(status.sensors);
    logLine("");
}

// Từ chối lệnh sai: dừng motor, DISARM và báo lỗi.
void rejectCommand() {
    modesDisarm("BAD_COMMAND");
    logLine("ERROR: command/mode/range invalid; stopped and DISARMED. Type HELP.");
}

// Kiểm tra các điều kiện dừng khẩn khi motor đang chạy: lỗi motor, hết giờ
// thử motor, hết giờ RUN (khi tắt auto-resume), vòng điều khiển bị trễ.
void checkMotionDeadline(unsigned long now) {
    if (!status.running) return;
    if (!motorIsReady()) {
        modesDisarm("MOTOR_FAULT");
    } else if (status.mode == RobotMode::MOTOR_TEST &&
               now - motionStartedAt >= motorTestDuration) {
        modesDisarm("TIMEOUT");
    } else if (status.mode == RobotMode::LINE_FOLLOW &&
               now - motionStartedAt >= tuningParameters().limitMs) {
        modesDisarm("RUN_TIMEOUT");
    } else if (status.mode == RobotMode::RIGHT_LINE &&
               now - motionStartedAt >= RIGHT_LINE_TEST_MAX_RUN_MS) {
        modesDisarm("RUN_TIMEOUT");
    } else if ((status.mode == RobotMode::LINE_FOLLOW &&
                static_cast<unsigned>(micros()) - lastLineTick >= LINE_STALE_US) ||
               (status.mode == RobotMode::RIGHT_LINE && now - lastControlAt > CONTROL_MAX_GAP_MS)) {
        modesDisarm("STALE_CONTROL");
    }
}

// Đọc số tốc độ từ chuỗi lệnh; chỉ chấp nhận số nguyên trong [-MAX_SPEED, MAX_SPEED].
bool parseTestSpeed(const char* token, int& speed) {
    if (!token) return false;
    errno = 0;
    char* end = nullptr;
    const long value = strtol(token, &end, 10);
    if (errno == ERANGE || end == token || *end || value < -MAX_SPEED || value > MAX_SPEED) return false;
    speed = static_cast<int>(value);
    return true;
}

void startMotorTest(int left, int right, unsigned long duration, bool fromBoot) {
    if (!motorIsReady() || !status.armed) {
        modesDisarm("MOTOR_NOT_ARMED");
        logLine("BLOCKED: motor hardware disabled/fault or DISARMED; ARM is required.");
        return;
    }
    status.leftCommand = left;
    status.rightCommand = right;
    motorTestDuration = duration;
    bootBenchRunning = fromBoot;
    motionStartedAt = millis();
    setMotorSpeeds(left, right);
    status.running = motorIsReady();
    if (!status.running) modesDisarm("MOTOR_FAULT");
    else logf("TEST: left=%d right=%d; TIMEOUT in %lu ms.\n", left, right, duration);
}

// Tách và thực hiện một dòng lệnh Serial (STOP, DISARM, HELP, STATUS, ARM,
// RUN, MODE ..., LEFT/RIGHT/BOTH n, SET l r). Lệnh sai -> rejectCommand().
void processCommand(char* line, unsigned long now) {
    char reply[192];
    if (strcmp(line, "TRACE=1") == 0 || strcmp(line, "TRACE=0") == 0) {
        if (status.running) logLine("ERR STOP before TRACE");
        else { traceEnabled = line[6] == '1'; logLine(traceEnabled ? "OK TRACE=1" : "OK TRACE=0"); }
        return;
    }
    if (tuningCommand(line, status.running, reply, sizeof(reply))) {
        if (std::strncmp(reply, "OK", 2) == 0) validSamples = 0;
        logLine(reply);
        return;
    }
    char* context = nullptr;
    char* command = strtok_r(line, " \t", &context);
    if (!command) return;
    char* argument = strtok_r(nullptr, " \t", &context);
    char* extra = strtok_r(nullptr, " \t", &context);
    char* tail = strtok_r(nullptr, " \t", &context);
    if (strcmp(command, "STOP") == 0) {
        if (argument) modesDisarm("BAD_STOP");
        else modesStop("STOP"); // Normal STOP retains ARM; it never restarts motion.
        logLine(argument ? "ERROR: STOP takes no arguments; motors stopped." : "STOP: motors stopped (coast).");
        return;
    }
    if (strcmp(command, "DISARM") == 0) {
        modesDisarm("DISARM"); // Even malformed DISARM must stop and revoke authorization.
        logLine(argument ? "ERROR: DISARM takes no arguments; stopped and DISARMED." : "DISARMED: both motors stopped.");
        return;
    }
    if (strcmp(command, "HELP") == 0 && !argument) { printHelp(); return; }
    if (strcmp(command, "STATUS") == 0 && !argument) { printStatus(); return; }
    if (strcmp(command, "LAST") == 0 && !argument) {
        if (status.running) logLine("ERR STOP before LAST");
        else if (lastSummary[0]) Serial.write(reinterpret_cast<const uint8_t*>(lastSummary), std::strlen(lastSummary));
        else {
#ifdef ARDUINO_ARCH_ESP32
            Preferences flash;
            if (flash.begin("hustroy-test", true)) {
                const String saved = flash.getString("last-v5", "");
                if (saved.length()) Serial.write(reinterpret_cast<const uint8_t*>(saved.c_str()), saved.length());
                else logLine("LAST: NONE for V5");
            } else logLine("LAST: NONE for V5");
#else
            logLine("LAST: NONE for V5");
#endif
        }
        return;
    }
    if (strcmp(command, "ARM") == 0 && !argument) { modesArm(); return; }
    if ((strcmp(command, "RUN") == 0 || strcmp(command, "GO") == 0) && !argument) { modesRun(now); return; }
    if (strcmp(command, "MODE") == 0 && argument && !extra) {
        if (strcmp(argument, "MOTOR_TEST") == 0) modesSelect(RobotMode::MOTOR_TEST, now);
        else if (strcmp(argument, "SENSOR_TEST") == 0) modesSelect(RobotMode::SENSOR_TEST, now);
        else if (strcmp(argument, "LINE_FOLLOW") == 0) modesSelect(RobotMode::LINE_FOLLOW, now);
        else if (strcmp(argument, "RIGHT_LINE") == 0) modesSelect(RobotMode::RIGHT_LINE, now);
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
        logLine("STOP: zero command (coast).");
        return;
    }
    if (bootBenchRunning) {
        logLine("BLOCKED: STOP before changing a BOOT bench pulse.");
        return;
    }
    startMotorTest(leftSpeed, rightSpeed, tuningParameters().pulseMs, false);
}

// Đọc từng ký tự Serial (tối đa 64 ký tự mỗi lần): '!' dừng khẩn ngay,
// Enter thì thực hiện dòng lệnh. Có ký tự bất kỳ thì hủy tự khởi động.
void pollSerial() {
    for (unsigned int count = 0; count < 64 && Serial.available() > 0; ++count) {
        checkMotionDeadline(millis());
        const char c = static_cast<char>(Serial.read());
        if (autoStartPending) {
            autoStartPending = false; // Operator is at the keyboard: never auto-start.
            logLine("AUTO START cancelled by Serial input.");
        }
        if (c == '!') {
            modesDisarm("EMERGENCY_COMMAND");
            commandLength = 0;
            discardLine = true;
            logLine("STOP + DISARM: ! received; press Enter before the next command.");
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

// Đọc nút chọn chế độ (nếu bật), có chống dội. Mỗi lần nhấn chuyển
// MOTOR_TEST -> SENSOR_TEST -> LINE_FOLLOW -> ... (không bao giờ tự ARM).
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
void pollBootButton(unsigned long now) {
    if (!BOOT_START_BUTTON_ENABLED) return;
    const bool high = digitalRead(BOOT_START_BUTTON_PIN) == HIGH;
    if (!high && (status.running || bootCountdownActive)) {
        modesDisarm("BOOT_STOP");
        bootCandidate = bootStable = false;
        bootChangedAt = now;
        return;
    }
    if (high != bootCandidate) {
        bootCandidate = high;
        bootChangedAt = now;
    }
    if (now - bootChangedAt >= MODE_BUTTON_DEBOUNCE_MS && high != bootStable) {
        bootStable = high;
        if (!high) {
            bootArmedForStart = !bootInhibitUntilRelease;
        } else if (bootInhibitUntilRelease) {
            bootInhibitUntilRelease = false;
        } else if (bootArmedForStart) {
            // A full button press and release authorizes one countdown.
            bootArmedForStart = false;
            modesSelect(BOOT_BENCH_PROFILE ? RobotMode::MOTOR_TEST : RobotMode::LINE_FOLLOW, now);
            if (modesArm()) {
                bootCountdownActive = true;
                bootCountdownStartedAt = now;
                logf("BOOT_START: countdown %lu ms; %s.\n", BOOT_START_COUNTDOWN_MS,
                     BOOT_BENCH_PROFILE ? "bench pulse" : "keep line centered");
            }
        }
    }
    if (bootCountdownActive && now - bootCountdownStartedAt >= BOOT_START_COUNTDOWN_MS) {
        bootCountdownActive = false;
        if (BOOT_BENCH_PROFILE) startMotorTest(BOOT_BENCH_PWM, BOOT_BENCH_PWM, BOOT_BENCH_RUN_MS, true);
        else modesRun(now);
    }
}
} // namespace

// Đổi chế độ thành chuỗi để in Serial.
const char* modeName(RobotMode mode) {
    switch (mode) {
        case RobotMode::MOTOR_TEST: return "MOTOR_TEST";
        case RobotMode::SENSOR_TEST: return "SENSOR_TEST";
        case RobotMode::LINE_FOLLOW: return "LINE_FOLLOW";
        case RobotMode::RIGHT_LINE: return "RIGHT_LINE";
        default: return "INVALID";
    }
}
// Trả về trạng thái hiện tại của robot (chỉ đọc).
const RobotStatus& modesStatus() { return status; }
bool modesCountdownActive() { return bootCountdownActive; }

// Dừng motor, xóa lệnh/PID, ghi lý do dừng. Vẫn giữ ARM.
void modesStop(const char* reason) {
    const bool wasRunning = status.running;
    const unsigned stopped = micros();
    bootCountdownActive = false;
    bootArmedForStart = false;
    bootBenchRunning = false;
    autoStartPending = false;
    motorStop();
    if (BOOT_START_BUTTON_ENABLED && bootButtonInitialized && digitalRead(BOOT_START_BUTTON_PIN) == LOW)
        bootInhibitUntilRelease = true;
    status.running = false;
    status.paused = false;
    status.leftCommand = status.rightCommand = 0;
    status.error = status.correction = 0;
    status.stopReason = reason;
    validSamples = 0;
    if (wasRunning && status.mode == RobotMode::LINE_FOLLOW) {
        stats.advance(stopped - lastLineTick);
        const unsigned wall = stopped - stats.start;
        const unsigned unknown = wall - std::min(wall, stats.observed);
        std::snprintf(lastSummary, sizeof(lastSummary),
                      "LINE_STATS FW=%s RUN_US=%u KNOWN_US=%u UNKNOWN_US=%u DETECTED_US=%u USABLE_US=%u "
                      "TRACK_US=%u SEARCH_US=%u RECOVER_US=%u AMBIG_US=%u ZERO_US=%u ZERO_EP=%u MAX_ZERO_US=%u "
                      "BIN_ZERO_US=%u BIN_ZERO_EP=%u RECOVERIES=%u MAE=%.4f EDGE_US=%u BLIND_FWD_US=%u REV=%u REASON=%s\n"
                      "LINE_TIMING N=%u MISSED=%u LATE=%u PERIOD_MIN_US=%u PERIOD_MAX_US=%u WORK_MAX_US=%u ADC_MAX_US=%u TX_DROP=%u\n",
                      FIRMWARE_ID, wall, stats.observed, unknown, stats.detected, stats.usable, stats.tracking,
                      stats.search, stats.recover, stats.ambiguous, stats.lost, stats.zeroEpisodes, stats.maxZero,
                      stats.binaryZeroUs, stats.binaryZeroEpisodes, stats.recoveries, stats.usable ? stats.errorArea / stats.usable : 0.0,
                      stats.edge, stats.blindForward, stats.reverse, reason, stats.samples, stats.missed, stats.late,
                      stats.samples ? stats.minPeriod : 0, stats.maxPeriod, stats.maxWork, stats.maxAdc, stats.txDrops);
        Serial.write(reinterpret_cast<const uint8_t*>(lastSummary), std::strlen(lastSummary));
#ifdef ARDUINO_ARCH_ESP32
        Preferences flash;
        if (flash.begin("hustroy-test", false)) flash.putString("last-v5", lastSummary);
#endif
    }
    if (wasRunning) logf("STOP: %s (coast). Explicit command required to restart.\n", reason);
}

// Thu hồi ARM rồi dừng motor; muốn chạy lại phải ARM lại.
void modesDisarm(const char* reason) {
    status.armed = false;
    modesStop(reason);
}

// Cho phép motor chạy (lệnh ARM). Bản thân ARM không làm bánh quay.
// Bị chặn nếu motor chưa sẵn sàng hoặc đang ở SENSOR_TEST.
bool modesArm() {
    modesDisarm("ARM_REQUEST"); // ARM itself never moves the wheels, even if repeated.
    if (!motorIsReady() || status.mode == RobotMode::SENSOR_TEST) {
        logLine("BLOCKED: ARM needs motor hardware READY and a motor-capable mode.");
        return false;
    }
    status.armed = true;
    logLine("ARMED: motors remain stopped. Send a test command, or wait for a stable line then RUN.");
    return true;
}

// Chuyển sang chế độ mới; luôn dừng motor và DISARM trước.
void modesSelect(RobotMode mode, unsigned long now) {
    modesDisarm("MODE_CHANGE");
    if (mode != RobotMode::MOTOR_TEST && mode != RobotMode::SENSOR_TEST &&
        mode != RobotMode::LINE_FOLLOW && mode != RobotMode::RIGHT_LINE) return;
    status.mode = mode;
    validSamples = 0;
    lastControlAt = lastLogAt = now;
    nextLineTick = static_cast<unsigned>(micros()) + LINE_PERIOD_US;
    logf("MODE=%s STOPPED and DISARMED. ARM required before motion.\n", modeName(mode));
}

// Bắt đầu dò line (lệnh RUN). Cần: LINE_FOLLOW + ARM, motor/cảm biến sẵn
// sàng, đã xác nhận hiệu chuẩn. Tắt auto-resume thì còn cần đang thấy line
// ổn định; bật auto-resume thì bắt đầu ở trạng thái tạm dừng chờ line.
bool modesRun(unsigned long now) {
    if (status.running) {
        // Repeated RUN cannot keep extending a finite trial.
        logLine("BLOCKED: already running; STOP before a new RUN.");
        return false;
    }
    const bool rightLineMode = status.mode == RobotMode::RIGHT_LINE;
    const bool lineModeReady = status.mode == RobotMode::LINE_FOLLOW && SENSOR_CALIBRATION_CONFIRMED &&
        SENSOR_DISCONNECT_BIAS_CONFIRMED &&
        (SENSOR_INPUT_MODE != SensorInputMode::ANALOG_AO || lineCalibrationApplied());
    const bool waitingAllowed = !rightLineMode && LINE_AUTO_RESUME;
    const bool readyToDrive = now - status.sensors.sampledAt <= CONTROL_MAX_GAP_MS &&
        validSamples >= LINE_START_VALID_SAMPLES &&
        (rightLineMode ? rightLineIsCentered(status.sensors) :
         sampleRevision == lineCalibrationRevision() && lineStartReady());
    if (!status.armed || !motorIsReady() || !sensorsIsReady() || (!rightLineMode && !lineModeReady) ||
        (!waitingAllowed && !readyToDrive)) {
        const unsigned heldSamples = validSamples;
        modesDisarm("RUN_BLOCKED");
        logf("BLOCKED: %s needs ARM and stable line (LINE_CAL=%d CONF=%.2f SAMPLES=%u/%u).\n",
             modeName(status.mode), lineCalibrationApplied(), observation.confidence, heldSamples, LINE_START_VALID_SAMPLES);
        return false;
    }
    if (!rightLineMode) {
        printProfile(); // Print run metadata while wheel outputs are still disabled.
        stats = RunStats{};
        follower = LineFollower{};
        followOutput = FollowOutput{};
        lastLineTick = stats.start = micros();
        nextLineTick = lastLineTick + LINE_PERIOD_US;
    }
    status.running = true;
    status.paused = waitingAllowed;
    motionStartedAt = lastControlAt = millis();
    logLine("RUN armed; first wheel command occurs at the next control tick.");
    return true;
}

// Khởi tạo lúc bật nguồn: về MOTOR_TEST, STOPPED + DISARMED, cấu hình nút
// (nếu bật) và hẹn giờ tự khởi động dò line (nếu AUTO_START_ON_BOOT).
void modesInit(unsigned long now) {
    tuningInit();
    status = RobotStatus{};
    modesSelect(RobotMode::MOTOR_TEST, now);
    if (MODE_BUTTON_ENABLED) {
        pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
        buttonCandidate = buttonStable = digitalRead(MODE_BUTTON_PIN) == HIGH;
        buttonReleased = buttonStable; // A held button at boot must be released first.
        buttonChangedAt = now;
    }
    if (BOOT_START_BUTTON_ENABLED) {
        pinMode(BOOT_START_BUTTON_PIN, INPUT_PULLUP);
        bootButtonInitialized = true;
        bootCandidate = bootStable = digitalRead(BOOT_START_BUTTON_PIN) == HIGH;
        bootArmedForStart = false; // A held BOOT button at startup must be released first.
        bootChangedAt = now;
        bootCountdownActive = false;
    }
    bootAt = now;
    autoStartPending = AUTO_START_ON_BOOT && !BOOT_BENCH_PROFILE;
    if (autoStartPending) logf("AUTO START in %lu ms (LINE_FOLLOW). Send any key to cancel.\n", AUTO_START_DELAY_MS);
    logLine("HUSTROY integrated firmware: startup STOPPED and DISARMED. Type HELP.");
    if (BOOT_BENCH_PROFILE) logf("BOOT BENCH: BOTH %d for %lu ms after a press and release.\n",
                                BOOT_BENCH_PWM, BOOT_BENCH_RUN_MS);
}

// Một vòng cập nhật (gọi trong loop()): tự khởi động nếu đến giờ, kiểm tra
// timeout, đọc nút; mỗi CONTROL_PERIOD_MS đọc cảm biến và (nếu đang dò line)
// tính PID -> đặt tốc độ motor hoặc tạm dừng/dừng khi mất line; xử lý Serial
// và in log cảm biến định kỳ.
void modesUpdate(unsigned long now) {
    checkMotionDeadline(now);
    pollSerial(); // Process STOP before issuing the next motor command.
    now = millis();
    if (autoStartPending && now - bootAt >= AUTO_START_DELAY_MS) {
        autoStartPending = false; // One attempt only; after STOP/! it never restarts itself.
        modesSelect(RobotMode::LINE_FOLLOW, now);
        if (modesArm()) modesRun(now);
    }
    pollButton(now);
    pollBootButton(now);
    const unsigned sampled = micros();
    const bool following = status.mode == RobotMode::LINE_FOLLOW;
    const bool rightLine = status.mode == RobotMode::RIGHT_LINE;
    const bool scheduled = static_cast<int32_t>(sampled - nextLineTick) >= 0;
    if (((following || traceEnabled) && scheduled) || (!following && now - lastControlAt >= CONTROL_PERIOD_MS)) {
        unsigned missed = 0;
        if (scheduled) {
            missed = (sampled - nextLineTick) / LINE_PERIOD_US;
            nextLineTick += (missed + 1) * LINE_PERIOD_US;
        }
        lastControlAt = now;
        lastSampleUs = sampled;
        status.sensors = sensorsRead(now);
        adcUs = static_cast<unsigned>(micros()) - sampled;
        if (following || traceEnabled) {
            if (sampleRevision != lineCalibrationRevision()) validSamples = 0;
            sampleRevision = lineCalibrationRevision();
            observation = observeLine(status.sensors, SENSOR_INPUT_MODE, lineCalibration());
        }
        const bool ready = rightLine ? rightLineIsCentered(status.sensors) : following && lineStartReady();
        validSamples = ready ? std::min(validSamples + 1, LINE_START_VALID_SAMPLES) : 0;
        if (rightLine && status.running) {
            if (!ready) stopForLine("RIGHT_LINE_LOST", now);
            else if (status.rightCommand == 0) {
                status.rightCommand = RIGHT_LINE_TEST_PWM;
                motorRunRightLineTest();
                if (!motorIsReady()) modesDisarm("MOTOR_FAULT");
            }
        } else if (following && status.running) {
            const unsigned period = sampled - lastLineTick;
            stats.advance(period);
            lastLineTick = sampled;
            ++stats.samples;
            stats.missed += missed;
            stats.minPeriod = std::min(stats.minPeriod, period);
            stats.maxPeriod = std::max(stats.maxPeriod, period);
            stats.maxAdc = std::max(stats.maxAdc, adcUs);
            if (LINE_AUTO_RESUME && observation.reading != LineReading::Invalid &&
                (observation.reading != LineReading::Position ||
                 (status.paused && validSamples < LINE_START_VALID_SAMPLES))) {
                if (!status.paused) follower = LineFollower{};
                status.paused = true;
                followOutput = FollowOutput{};
                followOutput.state = FollowState::Recover;
            } else {
                status.paused = false;
                followOutput = follower.update(observation, tuningParameters(), sampled, period);
            }
            status.error = followOutput.error;
            status.correction = followOutput.correction;
            status.leftCommand = followOutput.left;
            status.rightCommand = followOutput.right;
            stats.record(observation, followOutput, status.sensors);
            if (static_cast<unsigned>(micros()) - sampled >= LINE_STALE_US) followOutput.fault = "STALE_CONTROL";
            if (!followOutput.fault) setMotorSpeeds(status.leftCommand, status.rightCommand);
            const unsigned work = static_cast<unsigned>(micros()) - sampled;
            stats.maxWork = std::max(stats.maxWork, work);
            if (work >= nextLineTick - sampled) ++stats.late;
            if (followOutput.fault) stopForLine(followOutput.fault, now);
            else if (!motorIsReady()) modesDisarm("MOTOR_FAULT");
        }
    }
    const unsigned long logNow = millis();
    checkMotionDeadline(logNow);
    const bool sensorTest = status.mode == RobotMode::SENSOR_TEST;
    const unsigned long interval = traceEnabled || (following && status.running) ? 20 :
                                   sensorTest ? SENSOR_TEST_LOG_INTERVAL_MS : DEBUG_LOG_INTERVAL_MS;
    if (logNow - lastLogAt >= interval) {
        lastLogAt = logNow;
        if (traceEnabled || following) printTrace();
        else if (sensorTest) printSensorDiagnostic(status.sensors);
        else if (rightLine) printSensors();
    }
}
