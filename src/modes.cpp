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
#include <cstdlib>
#include <cstring>
#include "hardware_config.h"
#include "modes.h"
#include "motor.h"
#include "pid.h"
#include "tuning.h"

namespace {
constexpr std::size_t CENTER_SENSOR = SENSOR_COUNT / 2;
constexpr int RIGHT_LINE_CENTER_CONTRAST = 12; // Minimum AO contrast against each neighbor.
RobotStatus status;
PidController controller({PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT,
                         forwardCorrectionLimit(BASE_SPEED, MAX_SPEED, PID_CORRECTION_LIMIT)});
unsigned long lastControlAt = 0;
unsigned long lastLogAt = 0;
unsigned long motionStartedAt = 0;
unsigned int validSamples = 0;
float lastValidError = 0.0f;
bool lineRecovering = false;
unsigned long lineRecoveryStartedAt = 0;
char commandBuffer[64];
std::size_t commandLength = 0;
bool discardLine = false;
bool buttonCandidate = true;
bool buttonStable = true;
bool buttonReleased = false;
unsigned long buttonChangedAt = 0;
bool autoStartPending = false;
unsigned long bootAt = 0;

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

// Tạm dừng dò line (chỉ dùng khi LINE_AUTO_RESUME): tắt motor nhưng vẫn giữ
// RUNNING/ARMED để tự chạy lại khi thấy line ổn định. In lý do một lần.
void pauseLineFollow(const char* reason) {
    if (status.paused) return;
    motorStop();
    status.paused = true;
    status.leftCommand = status.rightCommand = 0;
    status.error = status.correction = 0;
    controller.reset();
    Serial.printf("PAUSE: %s; motors stopped, waiting for stable line.\n", reason);
}

// In danh sách lệnh Serial (lệnh HELP).
void printHelp() {
    Serial.println("HELP | STATUS | ARM | STOP | DISARM | ! (disarm immediately, then Enter)");
    Serial.println("MODE MOTOR_TEST | MODE SENSOR_TEST | MODE LINE_FOLLOW | MODE RIGHT_LINE");
    Serial.println("MOTOR_TEST: ARM, then LEFT n | RIGHT n | BOTH n | SET left right");
    Serial.println("Signed duty: positive=forward, negative=reverse, zero=coast. LEFT/RIGHT stop the other wheel.");
    Serial.println("LINE_FOLLOW: ARM, wait for stable line, then RUN. Mode changes always DISARM.");
    Serial.printf("RIGHT_LINE: ARM, center the line, then RUN. Right PWM %d, up to %lu ms.\n",
                  RIGHT_LINE_TEST_PWM, RIGHT_LINE_TEST_MAX_RUN_MS);
    Serial.println("TUNING: KP=<gain> | KD=<gain> | SPD=<duty> | GET | SAVE (stopped) | GO (RUN)");
    Serial.println(LINE_AUTO_RESUME ?
                   "LINE_FOLLOW: ARM, then RUN. Line lost = pause, line found = resume. Mode changes always DISARM." :
                   "LINE_FOLLOW: ARM, wait for stable line, then RUN. Mode changes always DISARM.");
    Serial.printf("Duty range -%d..%d/255, manual timeout=%lu ms, RUN limit=%lu ms.\n",
                  MAX_SPEED, MAX_SPEED, MOTOR_TEST_TIMEOUT_MS, LINE_FOLLOW_MAX_RUN_MS);
}

// In trạng thái tổng quát: chế độ, ARM, RUN, motor, cảm biến, lý do dừng (lệnh STATUS).
void printStatus() {
    Serial.printf("MODE=%s ARMED=%d RUN=%d MOTOR=%s SENSORS=%s CAL=%d BIAS=%d STOP=%s\n",
                  modeName(status.mode), status.armed, status.running, motorIsReady() ? "READY" : "DISABLED/FAULT",
                  sensorsIsReady() ? "RAW_READY" : "DISABLED", SENSOR_CALIBRATION_CONFIRMED,
                  SENSOR_DISCONNECT_BIAS_CONFIRMED, status.stopReason);
    // Configuration/request diagnostics, NOT measured voltage or wheel motion.
    Serial.printf("FW=%s LED=GPIO%u DOUBLE_BLINK PWM=%u-bit/%luHz "
                  "A(PWM/IN1/IN2)=%d/%d/%d B=%d/%d/%d STBY=%d L_REQ=%d R_REQ=%d\n",
                  FIRMWARE_ID, static_cast<unsigned int>(STATUS_LED_PIN),
                  static_cast<unsigned int>(MOTOR_PWM_BITS), static_cast<unsigned long>(MOTOR_PWM_FREQUENCY_HZ),
                  MOTOR_PWMA_PIN, MOTOR_AIN1_PIN, MOTOR_AIN2_PIN,
                  MOTOR_PWMB_PIN, MOTOR_BIN1_PIN, MOTOR_BIN2_PIN, MOTOR_STBY_PIN,
                  status.leftCommand, status.rightCommand);
    if (status.mode == RobotMode::RIGHT_LINE) {
        Serial.printf("RIGHT_LINE_CENTERED=%d PWM=%d LIMIT_MS=%lu\n",
                      rightLineIsCentered(status.sensors), RIGHT_LINE_TEST_PWM, RIGHT_LINE_TEST_MAX_RUN_MS);
    }
}

void printSensorSample(const SensorFrame& frame) {
    Serial.printf("RAW=[%d,%d,%d,%d,%d] BITS=%d%d%d%d%d RAW_OK=%d VALID=%d ALL=%d",
                  frame.raw[0], frame.raw[1], frame.raw[2], frame.raw[3], frame.raw[4],
                  frame.active[0], frame.active[1], frame.active[2], frame.active[3], frame.active[4],
                  frame.rawAvailable, frame.valid, frame.allActive);
}

void printSensors() {
    const auto& frame = status.sensors;
    printSensorSample(frame);
    Serial.printf(" STATE=%s DIR=%s POS=%s%.2f ERR=%.2f CORR=%.2f L=%d R=%d RUN=%d\n",
// In một dòng log cảm biến + PID: giá trị thô, bit, trạng thái, vị trí,
// sai số, hiệu chỉnh, lệnh motor trái/phải. Gọi định kỳ ngoài MOTOR_TEST.
void printSensors() {
    const auto& frame = status.sensors;
    Serial.printf("RAW=[%d,%d,%d,%d,%d] BITS=%d%d%d%d%d RAW_OK=%d VALID=%d ALL=%d ",
                  frame.raw[0], frame.raw[1], frame.raw[2], frame.raw[3], frame.raw[4],
                  frame.active[0], frame.active[1], frame.active[2], frame.active[3], frame.active[4],
                  frame.rawAvailable, frame.valid, frame.allActive);
    Serial.printf("STATE=%s DIR=%s POS=%s%.2f ERR=%.2f CORR=%.2f L=%d R=%d RUN=%d PAUSE=%d\n",
                  lineStateName(frame.state), lineDirectionName(frame.direction),
                  frame.valid && frame.state == LineState::LINE_DETECTED ? "" : "UNUSABLE:",
                  frame.position, status.error, status.correction,
                  status.leftCommand, status.rightCommand, status.running, status.paused);
}

void stopForLine(const char* reason, unsigned long now) {
    const int previousLeft = status.leftCommand;
    const int previousRight = status.rightCommand;
    modesDisarm(reason);
    Serial.printf("LINE_STOP REASON=%s T_MS=%lu PREV_L=%d PREV_R=%d ",
                  reason, now - motionStartedAt, previousLeft, previousRight);
    printSensorSample(status.sensors);
    Serial.println("");
}

// Từ chối lệnh sai: dừng motor, DISARM và báo lỗi.
void rejectCommand() {
    modesDisarm("BAD_COMMAND");
    Serial.println("ERROR: command/mode/range invalid; stopped and DISARMED. Type HELP.");
}

// Kiểm tra các điều kiện dừng khẩn khi motor đang chạy: lỗi motor, hết giờ
// thử motor, hết giờ RUN (khi tắt auto-resume), vòng điều khiển bị trễ.
void checkMotionDeadline(unsigned long now) {
    if (!status.running) return;
    if (!motorIsReady()) {
        modesDisarm("MOTOR_FAULT");
    } else if (status.mode == RobotMode::MOTOR_TEST &&
               now - motionStartedAt >= MOTOR_TEST_TIMEOUT_MS) {
        modesDisarm("TIMEOUT");
    } else if (!LINE_AUTO_RESUME && status.mode == RobotMode::LINE_FOLLOW &&
               now - motionStartedAt >= LINE_FOLLOW_MAX_RUN_MS) {
        modesDisarm("RUN_TIMEOUT");
    } else if (status.mode == RobotMode::RIGHT_LINE &&
               now - motionStartedAt >= RIGHT_LINE_TEST_MAX_RUN_MS) {
        modesDisarm("RUN_TIMEOUT");
    } else if ((status.mode == RobotMode::LINE_FOLLOW || status.mode == RobotMode::RIGHT_LINE) &&
               now - lastControlAt > CONTROL_MAX_GAP_MS) {
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

// Tách và thực hiện một dòng lệnh Serial (STOP, DISARM, HELP, STATUS, ARM,
// RUN, MODE ..., LEFT/RIGHT/BOTH n, SET l r). Lệnh sai -> rejectCommand().
void processCommand(char* line, unsigned long now) {
    char reply[96];
    if (tuningCommand(line, status.running, reply, sizeof(reply))) {
        Serial.println(reply);
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

// Đọc từng ký tự Serial (tối đa 64 ký tự mỗi lần): '!' dừng khẩn ngay,
// Enter thì thực hiện dòng lệnh. Có ký tự bất kỳ thì hủy tự khởi động.
void pollSerial() {
    for (unsigned int count = 0; count < 64 && Serial.available() > 0; ++count) {
        checkMotionDeadline(millis());
        const char c = static_cast<char>(Serial.read());
        if (autoStartPending) {
            autoStartPending = false; // Operator is at the keyboard: never auto-start.
            Serial.println("AUTO START cancelled by Serial input.");
        }
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

// Dừng motor, xóa lệnh/PID, ghi lý do dừng. Vẫn giữ ARM.
void modesStop(const char* reason) {
    const bool wasRunning = status.running;
    motorStop();
    status.running = false;
    status.paused = false;
    status.leftCommand = status.rightCommand = 0;
    status.error = status.correction = 0;
    status.stopReason = reason;
    validSamples = 0;
    lastValidError = 0.0f;
    lineRecovering = false;
    lineRecoveryStartedAt = 0;
    controller.reset();
    if (wasRunning) Serial.printf("STOP: %s (coast). Explicit command required to restart.\n", reason);
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
        Serial.println("BLOCKED: ARM needs motor hardware READY and a motor-capable mode.");
        return false;
    }
    status.armed = true;
    Serial.println("ARMED: motors remain stopped. Send a test command, or wait for a stable line then RUN.");
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
    Serial.printf("MODE=%s STOPPED and DISARMED. ARM required before motion.\n", modeName(mode));
}

// Bắt đầu dò line (lệnh RUN). Cần: LINE_FOLLOW + ARM, motor/cảm biến sẵn
// sàng, đã xác nhận hiệu chuẩn. Tắt auto-resume thì còn cần đang thấy line
// ổn định; bật auto-resume thì bắt đầu ở trạng thái tạm dừng chờ line.
bool modesRun(unsigned long now) {
    if (status.running) {
        // Repeated RUN cannot keep extending a finite trial.
        Serial.println("BLOCKED: already running; STOP before a new RUN.");
        return false;
    }
    const bool rightLineMode = status.mode == RobotMode::RIGHT_LINE;
    const bool lineReady = rightLineMode ? rightLineIsCentered(status.sensors) :
        status.mode == RobotMode::LINE_FOLLOW && SENSOR_CALIBRATION_CONFIRMED &&
        SENSOR_DISCONNECT_BIAS_CONFIRMED && status.sensors.valid &&
        status.sensors.state == LineState::LINE_DETECTED &&
        forwardCorrectionLimit(tuningParameters().speed, MAX_SPEED, PID_CORRECTION_LIMIT) > 0;
    if (!status.armed || !motorIsReady() || !sensorsIsReady() || !lineReady ||
        now - status.sensors.sampledAt > CONTROL_MAX_GAP_MS || validSamples < LINE_START_VALID_SAMPLES) {
        const unsigned int heldSamples = validSamples;
    // Auto-resume starts PAUSED, so it need not be on the line yet: it waits for one.
    const bool lineReady = LINE_AUTO_RESUME ||
        (status.sensors.valid && status.sensors.state == LineState::LINE_DETECTED &&
         now - status.sensors.sampledAt <= CONTROL_MAX_GAP_MS && validSamples >= LINE_START_VALID_SAMPLES);
    if (!status.armed || status.mode != RobotMode::LINE_FOLLOW || !motorIsReady() || !sensorsIsReady() ||
        !SENSOR_CALIBRATION_CONFIRMED || !SENSOR_DISCONNECT_BIAS_CONFIRMED || !lineReady ||
        forwardCorrectionLimit(BASE_SPEED, MAX_SPEED, PID_CORRECTION_LIMIT) <= 0) {
        modesDisarm("RUN_BLOCKED");
        if (rightLineMode) {
            Serial.printf("BLOCKED: RIGHT_LINE needs ARM, motor/sensors ready and a stable centered line "
                          "(ARM=%d MOTOR=%d SENS=%d CAL=%d BIAS=%d STATE=%s VALID=%d SAMPLES=%u/%u).\n",
                          status.armed, motorIsReady(), sensorsIsReady(),
                          SENSOR_CALIBRATION_CONFIRMED, SENSOR_DISCONNECT_BIAS_CONFIRMED,
                          lineStateName(status.sensors.state), status.sensors.valid,
                          heldSamples, LINE_START_VALID_SAMPLES);
        } else {
            Serial.printf("BLOCKED: RUN needs ARM + LINE_FOLLOW, motor/sensors ready, calibration/bias and fresh stable line "
                          "(ARM=%d MOTOR=%d SENS=%d CAL=%d BIAS=%d STATE=%s VALID=%d SAMPLES=%u/%u).\n",
                          status.armed, motorIsReady(), sensorsIsReady(),
                          SENSOR_CALIBRATION_CONFIRMED, SENSOR_DISCONNECT_BIAS_CONFIRMED,
                          lineStateName(status.sensors.state), status.sensors.valid,
                          heldSamples, LINE_START_VALID_SAMPLES);
        }
        return false;
    }
    controller.reset();
    lastValidError = INVERT_PID_ERROR ? -status.sensors.position : status.sensors.position;
    lineRecovering = false;
    lineRecoveryStartedAt = now;
    status.running = true;
    status.paused = LINE_AUTO_RESUME;
    motionStartedAt = lastControlAt = now;
    Serial.println(LINE_AUTO_RESUME ?
                   "RUN (auto-resume): motors run while a stable line is seen. STOP or ! to end." :
                   "RUN armed; first wheel command occurs at the next control tick.");
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
    Serial.println("HUSTROY integrated firmware: startup STOPPED and DISARMED. Type HELP.");
    bootAt = now;
    autoStartPending = AUTO_START_ON_BOOT;
    if (autoStartPending) {
        Serial.printf("AUTO START in %lu ms (LINE_FOLLOW). Send any key to cancel.\n", AUTO_START_DELAY_MS);
    }
}

// Một vòng cập nhật (gọi trong loop()): tự khởi động nếu đến giờ, kiểm tra
// timeout, đọc nút; mỗi CONTROL_PERIOD_MS đọc cảm biến và (nếu đang dò line)
// tính PID -> đặt tốc độ motor hoặc tạm dừng/dừng khi mất line; xử lý Serial
// và in log cảm biến định kỳ.
void modesUpdate(unsigned long now) {
    if (autoStartPending && now - bootAt >= AUTO_START_DELAY_MS) {
        autoStartPending = false; // One attempt only; after STOP/! it never restarts itself.
        modesSelect(RobotMode::LINE_FOLLOW, now);
        if (modesArm()) modesRun(now);
    }
    checkMotionDeadline(now);
    pollButton(now);
    const unsigned long elapsed = now - lastControlAt;
    if (elapsed >= CONTROL_PERIOD_MS) {
        lastControlAt = now; // One update, never a burst of catch-up commands.
        status.sensors = sensorsRead(now);
        const bool rightLineMode = status.mode == RobotMode::RIGHT_LINE;
        const bool lineDetected = rightLineMode ? rightLineIsCentered(status.sensors) :
            status.sensors.valid && status.sensors.state == LineState::LINE_DETECTED;
        if (lineDetected) {
            if (validSamples < LINE_START_VALID_SAMPLES) ++validSamples;
        } else validSamples = 0;
        if (rightLineMode && status.running) {
            if (!lineDetected) stopForLine("RIGHT_LINE_LOST", now);
            else if (status.rightCommand == 0) {
                status.rightCommand = RIGHT_LINE_TEST_PWM;
                motorRunRightLineTest();
                if (!motorIsReady()) modesDisarm("MOTOR_FAULT");
            }
        } else if (status.mode == RobotMode::LINE_FOLLOW && status.running) {
            if (!lineDetected) {
                if (!status.sensors.rawAvailable) {
                    stopForLine(lineStateName(status.sensors.state), now);
                } else {
                    if (!lineRecovering) {
                        lineRecovering = true;
                        lineRecoveryStartedAt = now;
                        controller.reset();
                    }
                    const unsigned long limitMs =
                        status.sensors.state == LineState::LINE_LOST ? LINE_LOST_GRACE_MS :
                        status.sensors.state == LineState::INTERSECTION ? LINE_INTERSECTION_STOP_MS :
                        LINE_INVALID_GRACE_MS;
                    if (now - lineRecoveryStartedAt >= limitMs) {
                        stopForLine(lineStateName(status.sensors.state), now);
                    } else {
                        const auto& tuning = tuningParameters();
                        if (status.sensors.state == LineState::LINE_LOST && std::abs(lastValidError) >= 0.25f) {
                            const int pivotSpeed = std::clamp(tuning.speed, 1, MAX_SPEED);
                            const bool turnRight = lastValidError > 0.0f;
                            const int left = turnRight ? pivotSpeed : -pivotSpeed;
                            const int right = turnRight ? -pivotSpeed : pivotSpeed;
                            status.error = lastValidError;
                            status.correction = turnRight ? static_cast<float>(pivotSpeed) : -static_cast<float>(pivotSpeed);
                            status.leftCommand = left;
                            status.rightCommand = right;
                            setMotorSpeeds(left, right);
                            if (!motorIsReady()) modesDisarm("MOTOR_FAULT");
                        } else {
                            const float corrLimit = forwardCorrectionLimit(tuning.speed, MAX_SPEED, PID_CORRECTION_LIMIT);
                            float recoveryCorrection = 0.0f;
                            if (status.sensors.state == LineState::INTERSECTION) {
                                recoveryCorrection = 0.0f;
                            } else if (status.sensors.state != LineState::LINE_LOST && std::abs(lastValidError) >= 0.25f) {
                                recoveryCorrection = std::clamp(tuning.kp * lastValidError, -corrLimit, corrLimit);
                            }
                            const WheelSpeeds speeds = mixForwardSpeeds(tuning.speed, recoveryCorrection, MAX_SPEED);
                            if (!speeds.valid) {
                                modesDisarm("PID_INVALID");
                            } else {
                                status.error = status.sensors.state == LineState::INTERSECTION ? 0.0f : lastValidError;
                                status.correction = recoveryCorrection;
                                status.leftCommand = speeds.left;
                                status.rightCommand = speeds.right;
                                setMotorSpeeds(speeds.left, speeds.right);
                                if (!motorIsReady()) modesDisarm("MOTOR_FAULT");
                            }
                        }
                    }
                }
            } else {
                lineRecovering = false;
        if (status.mode == RobotMode::LINE_FOLLOW && status.running) {
            const bool onLine = status.sensors.valid && status.sensors.state == LineState::LINE_DETECTED;
            if (!onLine && (!LINE_AUTO_RESUME || !status.sensors.rawAvailable)) {
                // Latched stop; raw/ADC faults always latch even with auto-resume.
                modesDisarm(status.sensors.rawAvailable ? lineStateName(status.sensors.state) : "SENSOR_FAULT");
            } else if (!onLine) {
                pauseLineFollow(lineStateName(status.sensors.state));
            } else if (!status.paused || validSamples >= LINE_START_VALID_SAMPLES) {
                if (status.paused) {
                    status.paused = false;
                    controller.reset();
                    Serial.println("RESUME: stable line found, motors running.");
                }
                status.error = INVERT_PID_ERROR ? -status.sensors.position : status.sensors.position;
                lastValidError = status.error;
                const auto& tuning = tuningParameters();
                controller.configure({tuning.kp, PID_KI, tuning.kd, PID_INTEGRAL_LIMIT,
                                      forwardCorrectionLimit(tuning.speed, MAX_SPEED, PID_CORRECTION_LIMIT)});
                const PidResult pid = controller.update(status.error, static_cast<float>(elapsed) / 1000.0f);
                const WheelSpeeds speeds = mixForwardSpeeds(tuning.speed, pid.output, MAX_SPEED);
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
