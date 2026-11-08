#include <Arduino.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "motor.h"

namespace {
char commandBuffer[48];
size_t commandLength = 0;
bool discardLine = false;
bool testRunning = false;
unsigned long testStartedAt = 0;
unsigned long lastLedAt = 0;
bool ledOn = false;

void stopTest() {
    motorStop();
    testRunning = false;
}

void checkTestTimeout() {
    // Unsigned subtraction also works across millis() rollover.
    if (testRunning && millis() - testStartedAt >= MOTOR_TEST_TIMEOUT_MS) {
        stopTest();
        Serial.println("TIMEOUT: motors stopped (coast).");
    }
}

void printHelp() {
    Serial.println("Commands (UPPERCASE, Enter/LF or CRLF):");
    Serial.println("  HELP | STATUS | STOP");
    Serial.println("  LEFT n  : left only; right stopped");
    Serial.println("  RIGHT n : right only; left stopped");
    Serial.println("  BOTH n  : both at the same signed duty");
    Serial.printf("n is an integer from -%d to %d; negative = reverse, 0 = stop.\n",
                  MAX_SPEED, MAX_SPEED);
    Serial.printf("Each motion command expires after %lu ms. Start with LEFT 30.\n",
                  MOTOR_TEST_TIMEOUT_MS);
    Serial.println("Send ! to stop immediately without Enter; then press Enter to clear line.");
}

void rejectCommand() {
    stopTest();
    Serial.println("ERROR: invalid command/range; motors stopped. Type HELP.");
}

void processCommand(char* line) {
    char* context = nullptr;
    char* command = strtok_r(line, " \t", &context);
    if (command == nullptr) return;
    char* argument = strtok_r(nullptr, " \t", &context);
    char* extra = strtok_r(nullptr, " \t", &context);
    if (strcmp(command, "STOP") == 0) {
        stopTest();  // Stop even if someone accidentally adds arguments.
        if (argument) Serial.println("ERROR: STOP takes no arguments; motors stopped.");
        else Serial.println("STOP: motors stopped (coast).");
        return;
    }
    if (strcmp(command, "HELP") == 0 && argument == nullptr) {
        printHelp();
        return;
    }
    if (strcmp(command, "STATUS") == 0 && argument == nullptr) {
        Serial.printf("Driver: %s | Test: %s | Limit: %d/255 | Timeout: %lu ms\n",
                      motorIsReady() ? "READY" : "DISABLED/FAULT",
                      testRunning ? "RUNNING" : "STOPPED", MAX_SPEED, MOTOR_TEST_TIMEOUT_MS);
        return;
    }
    const bool left = strcmp(command, "LEFT") == 0;
    const bool right = strcmp(command, "RIGHT") == 0;
    const bool both = strcmp(command, "BOTH") == 0;
    if ((!left && !right && !both) || argument == nullptr || extra != nullptr) {
        rejectCommand();
        return;
    }

    errno = 0;
    char* end = nullptr;
    const long speed = strtol(argument, &end, 10);
    if (errno == ERANGE || end == argument || *end != '\0' ||
        speed < -MAX_SPEED || speed > MAX_SPEED) {
        rejectCommand();
        return;
    }
    const int leftSpeed = right ? 0 : static_cast<int>(speed);
    const int rightSpeed = left ? 0 : static_cast<int>(speed);
    if (speed == 0) {
        stopTest();
        Serial.println("STOP: zero command; both motors stopped (coast).");
        return;
    }
    if (!motorIsReady()) {
        stopTest();
        Serial.println("BLOCKED: motor hardware disabled/fault; no motion. Check config.h.");
        return;
    }
    setMotorSpeeds(leftSpeed, rightSpeed);
    testRunning = motorIsReady();
    testStartedAt = millis();
    if (testRunning) {
        Serial.printf("TEST: left=%d right=%d duty/255; auto-stop in %lu ms.\n",
                      leftSpeed, rightSpeed, MOTOR_TEST_TIMEOUT_MS);
    }
}

void pollSerial() {
    // Bounded work: a continuous stream cannot starve the timeout/other modules.
    for (unsigned int count = 0; count < 64 && Serial.available() > 0; ++count) {
        checkTestTimeout();
        const char c = static_cast<char>(Serial.read());
        if (c == '!') {
            stopTest();
            commandLength = 0;
            discardLine = true;
            Serial.println("STOP: ! received; press Enter before the next command.");
        } else if (c == '\r' || c == '\n') {
            if (!discardLine) {
                commandBuffer[commandLength] = '\0';
                processCommand(commandBuffer);
            }
            commandLength = 0;
            discardLine = false;
        } else if (!discardLine) {
            // Reject control bytes (including NUL) and overlong lines safely.
            if ((c != '\t' && (c < 32 || c > 126)) ||
                commandLength >= sizeof(commandBuffer) - 1) {
                rejectCommand();
                commandLength = 0;
                discardLine = true;
            } else {
                commandBuffer[commandLength++] = c;
            }
        }
    }
}
}  // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    motorInit();
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    Serial.println("HUSTROY TB6612 bench test. Startup: STOPPED. Type HELP.");
}

void loop() {
    checkTestTimeout();
    pollSerial();
    checkTestTimeout();
    const unsigned long now = millis();
    if (now - lastLedAt >= STATUS_LED_INTERVAL_MS) {
        lastLedAt = now;
        ledOn = !ledOn;
        digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
    }
}
