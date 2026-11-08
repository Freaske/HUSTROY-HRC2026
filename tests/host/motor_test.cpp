#include <Arduino.h>
#include <cassert>
#include <climits>
#include <iostream>
#include <map>
#include "config.h"
#include "motor.h"
#include "modes.h"
#include "pid.h"
#include <cmath>
#include <limits>

void setup();
void loop();

FakeSerial Serial;
unsigned long nowMs = 0;
std::map<int, int> levels;
std::map<int, int> modes;
std::map<int, uint32_t> duties;
int attachCalls = 0;
int writeCalls = 0;
int failAttachAt = -1;
int failWriteAt = -1;
std::map<int, int> rawInputs;

unsigned long millis() { return nowMs; }
int gpio_set_level(int pin, unsigned int value) {
    assert(pin >= 0 && pin <= 33);
    levels[pin] = static_cast<int>(value);
    return 0;
}
void pinMode(int pin, int mode) {
    assert(pin >= 0 && pin <= 39);
    if (mode == OUTPUT && pin != STATUS_LED_PIN) assert(levels.at(pin) == LOW);
    modes[pin] = mode;
}
int digitalRead(int pin) {
    assert(modes.at(pin) == INPUT || modes.at(pin) == INPUT_PULLUP);
    return rawInputs.count(pin) ? rawInputs.at(pin) : (modes.at(pin) == INPUT_PULLUP ? HIGH : LOW);
}
int analogRead(int pin) {
    assert(modes.at(pin) == INPUT);
    return rawInputs.count(pin) ? rawInputs.at(pin) : 0;
}
void analogReadResolution(uint8_t bits) { assert(bits == 12); }
void analogSetPinAttenuation(int pin, int attenuation) {
    assert(pin == 32 || pin == 33 || pin == 34 || pin == 35 || pin == 36 || pin == 39);
    assert(attenuation == ADC_11db);
}
void digitalWrite(int pin, int value) {
    assert(modes.at(pin) == OUTPUT);  // Core 3.x requires pinMode first.
    if (pin != STATUS_LED_PIN && pin != MOTOR_STBY_PIN) {
        assert(levels.at(MOTOR_STBY_PIN) == LOW);
    }
    levels[pin] = value;
}
bool ledcAttach(uint8_t pin, uint32_t frequency, uint8_t bits) {
    assert(levels.at(MOTOR_STBY_PIN) == LOW);
    assert(pin == static_cast<uint8_t>(MOTOR_PWMA_PIN) ||
           pin == static_cast<uint8_t>(MOTOR_PWMB_PIN));
    assert(frequency == 20000 && bits == 8);
    return ++attachCalls != failAttachAt;
}
bool ledcWrite(uint8_t pin, uint32_t duty) {
    assert(levels.at(MOTOR_STBY_PIN) == LOW);  // No live direction/duty updates.
    assert(pin == static_cast<uint8_t>(MOTOR_PWMA_PIN) ||
           pin == static_cast<uint8_t>(MOTOR_PWMB_PIN));
    assert(duty <= 255);
    if (++writeCalls == failWriteAt) return false;
    duties[pin] = duty;
    return true;
}

void send(const std::string& command) {
    for (unsigned char c : command) Serial.input.push_back(c);
    // One loop is deliberately bounded; drive the remaining UART input too.
    do { loop(); } while (!Serial.input.empty());
}
void expectStopped() {
    assert(levels.at(MOTOR_STBY_PIN) == LOW);
    if (motorIsReady()) {
        assert(duties.at(MOTOR_PWMA_PIN) == 0);
        assert(duties.at(MOTOR_PWMB_PIN) == 0);
    }
}
void expectChannel(int pwm, int in1, int in2, int speed) {
    assert(levels.at(in1) == (speed > 0 ? HIGH : LOW));
    assert(levels.at(in2) == (speed < 0 ? HIGH : LOW));
    const auto duty = static_cast<uint32_t>(speed == 0 ? 255 : std::abs(speed));
    assert(duties.at(pwm) == duty);
}
void expectSpeeds(int left, int right) {
    assert(levels.at(MOTOR_STBY_PIN) == HIGH);
    if (INVERT_LEFT_MOTOR) left = -left;
    if (INVERT_RIGHT_MOTOR) right = -right;
    expectChannel(MOTOR_PWMA_PIN, MOTOR_AIN1_PIN, MOTOR_AIN2_PIN,
                  LEFT_MOTOR_IS_CHANNEL_A ? left : right);
    expectChannel(MOTOR_PWMB_PIN, MOTOR_BIN1_PIN, MOTOR_BIN2_PIN,
                  LEFT_MOTOR_IS_CHANNEL_A ? right : left);
}

void sendArmed(const std::string& command) {
    send("ARM\n");
    assert(modesStatus().armed);
    expectStopped(); // ARM never drives either channel.
    send(command);
}

#include "integration_checks.h"

int main(int argc, char** argv) {
    assert(argc == 2);
    const std::string scenario = argv[1];
    // All APIs must do nothing before initialization, even extreme inputs.
    setMotorSpeeds(INT_MIN, INT_MAX);
    motorForward(INT_MIN);
    motorStop();
    assert(levels.empty() && attachCalls == 0 && writeCalls == 0);

    if (scenario == "logic") {
        runPureLogicChecks();
        std::cout << "PASS: logic (all 32 patterns, normalization, PID and mixing)\n";
        return 0;
    }
    if (scenario.rfind("system-", 0) == 0) {
        if (SENSOR_HARDWARE_ENABLED) setPattern("00100");
        if (MODE_BUTTON_ENABLED) rawInputs[MODE_BUTTON_PIN] = scenario == "system-held-button" ? LOW : HIGH;
        setup();
        runSystemChecks(scenario);
        std::cout << "PASS: " << scenario << '\n';
        return 0;
    }

    if (scenario == "attach-a") failAttachAt = 1;
    if (scenario == "attach-b") failAttachAt = 2;
    if (scenario == "init-write") failWriteAt = 1;
    if (scenario == "init-stop") failWriteAt = 3;
    setup();

    if (scenario == "arming") {
        runArmingChecks();
        std::cout << "PASS: arming, DISARM, SET and safe mode transitions\n";
        return 0;
    }

    if (!MOTOR_HARDWARE_ENABLED) {
        assert(!motorIsReady() && attachCalls == 0 && writeCalls == 0);
        send("ARM\nLEFT 30\nRIGHT -30\nBOTH 50\nSET 30 -30\nDISARM\nSTOP\nHELP\nSTATUS\n");
        assert(!modesStatus().armed);
        assert(!modesStatus().running);
        assert(Serial.output.find("startup STOPPED and DISARMED") != std::string::npos);
        assert(Serial.output.find("HELP | STATUS | ARM") != std::string::npos);
        assert(Serial.output.find("MODE=MOTOR_TEST ARMED=0 RUN=0 MOTOR=DISABLED/FAULT") != std::string::npos);
        assert(attachCalls == 0 && writeCalls == 0); // Diagnostics never allocate motor PWM.
        send("MODE SENSOR_TEST\nMODE LINE_FOLLOW\nRUN\nSTOP\nMODE MOTOR_TEST\n");
        assert(levels.size() == 1 && levels.count(STATUS_LED_PIN));
        assert(Serial.output.find("BLOCKED") != std::string::npos);
    } else if (scenario == "attach-a" || scenario == "attach-b" ||
               scenario == "init-write" || scenario == "init-stop") {
        assert(!motorIsReady());
        expectStopped();
        send("BOTH 30\n");
        expectStopped();
        assert(Serial.output.find("MOTOR READY") == std::string::npos);
    } else {
        assert(motorIsReady());
        expectStopped();
        if (scenario == "driver") {
            setMotorSpeeds(INT_MAX, INT_MIN);
            expectSpeeds(MAX_SPEED, -MAX_SPEED);
            setMotorSpeeds(0, 40);
            expectSpeeds(0, 40);
            setMotorSpeeds(-40, 0);
            expectSpeeds(-40, 0);
            motorForward(-30);
            expectSpeeds(30, 30);
            motorForward(INT_MIN);
            expectSpeeds(MAX_SPEED, MAX_SPEED);
            setMotorSpeeds(0, 0);
            expectStopped();
            setMotorSpeeds(30, 30);
            motorInit();  // Re-init stops and does not allocate more channels.
            expectStopped();
            assert(attachCalls == 2);
        } else if (scenario == "runtime-fault" || scenario == "stop-fault") {
            setMotorSpeeds(30, 30);
            failWriteAt = writeCalls + 1;
            if (scenario == "stop-fault") motorStop();
            else setMotorSpeeds(-30, 30);
            assert(!motorIsReady());
            expectStopped();
            failWriteAt = -1;
            send("BOTH 30\n");
            motorInit();
            assert(!motorIsReady());
            expectStopped();
        } else {
            sendArmed("LEFT 30\r\n");
            expectSpeeds(30, 0);
            nowMs += MOTOR_TEST_TIMEOUT_MS - 1;
            loop();
            expectSpeeds(30, 0);
            ++nowMs;
            loop();
            expectStopped();
            sendArmed("RIGHT -40\n");
            expectSpeeds(0, -40);
            sendArmed("BOTH 50\n");
            expectSpeeds(50, 50);
            send("STOP\n");
            expectStopped();
            sendArmed("BOTH -50\n");
            expectSpeeds(-50, -50);
            send("!");  // No newline needed; remainder of line is discarded.
            expectStopped();
            send("BOTH 100\n");
            expectStopped();
            send("BOTH 0\n");
            expectStopped();
            const std::string invalid[] = {
                "LEFT\n", "BOTH 151\n", "BOTH -151\n", "RIGHT 3.5\n",
                "BOTH 999999999999999999999999999999\n", "BOTH 30 junk\n",
                "left 30\n", "REVERSE 50\n", "BOTH 0x20\n", "BOTH +\n",
                "STOP extra\n", std::string(90, 'A') + "\n",
                std::string("BOTH 30\0junk\n", 13)};
            for (const auto& bad : invalid) {
                sendArmed("BOTH 30\n");
                expectSpeeds(30, 30);
                send(bad);
                expectStopped();
            }
            nowMs = ULONG_MAX - 100;
            sendArmed("BOTH 30\n");
            nowMs += MOTOR_TEST_TIMEOUT_MS;
            loop();
            expectStopped();  // Unsigned millis rollover.
            sendArmed("BOTH 30\n");
            nowMs += 400;
            send("STATUS\nHELP\n");
            nowMs += 100;
            loop();
            expectStopped();  // HELP/STATUS must not extend the deadline.
            sendArmed("BOTH 30\n");
            nowMs += 400;
            sendArmed("RIGHT 40\n");
            nowMs += 100;
            loop();
            expectSpeeds(0, 40);  // A new valid motion starts a new pulse.
            nowMs += 400;
            loop();
            expectStopped();
            assert(Serial.output.find("TIMEOUT") != std::string::npos);
        }
    }
    std::cout << "PASS: " << scenario << '\n';
}
