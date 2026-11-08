#include <Arduino.h>
#include <esp_arduino_version.h>
#include <driver/gpio.h>
#include "motor.h"
#include "config.h"

#if ESP_ARDUINO_VERSION_MAJOR != 3
#error "This driver uses Arduino-ESP32 3.x pin-based LEDC; install the documented platform."
#endif

namespace {
constexpr int motorPins[] = {MOTOR_PWMA_PIN, MOTOR_AIN1_PIN, MOTOR_AIN2_PIN,
                             MOTOR_PWMB_PIN, MOTOR_BIN1_PIN, MOTOR_BIN2_PIN,
                             MOTOR_STBY_PIN};

// Conservative classic ESP32 policy: reject flash/PSRAM, UART0, boot straps,
// input-only/nonexistent pins and the existing status LED. This is a validity
// check, NOT a proposed wiring map. Re-audit for any different ESP32 board.
constexpr bool isSafeMotorPin(int pin) {
    return (pin == 4 || pin == 13 || pin == 14 || (pin >= 18 && pin <= 19) ||
            (pin >= 21 && pin <= 23) || (pin >= 25 && pin <= 27) ||
            pin == 32 || pin == 33) && pin != STATUS_LED_PIN;
}

constexpr bool motorPinsValid() {
    for (unsigned int i = 0; i < sizeof(motorPins) / sizeof(motorPins[0]); ++i) {
        if (!isSafeMotorPin(motorPins[i])) return false;
        for (unsigned int j = 0; j < i; ++j) {
            if (motorPins[i] == motorPins[j]) return false;
        }
    }
    return true;
}

static_assert(!MOTOR_HARDWARE_ENABLED || motorPinsValid(),
              "Motor GPIOs unassigned, duplicated, reserved or unsafe; ask Toan.");
#if !defined(CONFIG_IDF_TARGET_ESP32)
static_assert(!MOTOR_HARDWARE_ENABLED, "Re-audit GPIO policy for this ESP32 variant.");
#endif

bool initAttempted = false;
bool ready = false;

int clampSpeed(int speed) {
    if (speed > MAX_SPEED) return MAX_SPEED;
    if (speed < -MAX_SPEED) return -MAX_SPEED;
    return speed;
}

// Called with STBY LOW. PWM HIGH + IN1/IN2 LOW selects high-impedance
// coast when the other wheel is moving. PWM LOW alone would select braking.
bool writeChannel(int pwm, int in1, int in2, int speed) {
    digitalWrite(in1, speed > 0 ? HIGH : LOW);
    digitalWrite(in2, speed < 0 ? HIGH : LOW);
    const int duty = speed == 0 ? MOTOR_PWM_MAX : (speed < 0 ? -speed : speed);
    return ledcWrite(pwm, duty);
}

void latchPwmFault() {
    digitalWrite(MOTOR_STBY_PIN, LOW);
    ready = false;  // No automatic restart after a failed PWM operation.
    Serial.println("ERROR: PWM failure; STBY LOW. Fix cause and reset ESP32.");
}
}  // namespace

void motorInit() {
    if (initAttempted) {
        motorStop();
        return;
    }
    initAttempted = true;
    if (!MOTOR_HARDWARE_ENABLED || !motorPinsValid()) {
        Serial.println("MOTOR DISABLED: confirm 7 GPIOs and wiring in config.h with Toan.");
        return;  // Do not touch ANY motor GPIO/LEDC in unconfigured mode.
    }

    // Hold hardware disabled before configuring any other control signals.
    // Arduino core 3.x digitalWrite cannot preload a pin before pinMode.
    // The native GPIO API preloads LOW before enabling the output driver.
    gpio_set_level(static_cast<gpio_num_t>(MOTOR_STBY_PIN), 0);
    pinMode(MOTOR_STBY_PIN, OUTPUT);
    digitalWrite(MOTOR_STBY_PIN, LOW);
    for (int pin : motorPins) {
        if (pin == MOTOR_STBY_PIN) continue;
        gpio_set_level(static_cast<gpio_num_t>(pin), 0);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    if (!ledcAttach(MOTOR_PWMA_PIN, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_BITS) ||
        !ledcAttach(MOTOR_PWMB_PIN, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_BITS) ||
        !ledcWrite(MOTOR_PWMA_PIN, 0) || !ledcWrite(MOTOR_PWMB_PIN, 0)) {
        latchPwmFault();
        return;
    }
    ready = true;
    motorStop();
    if (ready) Serial.println("MOTOR READY: stopped, STBY LOW. Awaiting an explicit command.");
}

bool motorIsReady() {
    return ready;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if (!ready) return;
    leftSpeed = clampSpeed(leftSpeed);
    rightSpeed = clampSpeed(rightSpeed);
    if (leftSpeed == 0 && rightSpeed == 0) {
        motorStop();
        return;
    }

    if (INVERT_LEFT_MOTOR) leftSpeed = -leftSpeed;
    if (INVERT_RIGHT_MOTOR) rightSpeed = -rightSpeed;
    const int speedA = LEFT_MOTOR_IS_CHANNEL_A ? leftSpeed : rightSpeed;
    const int speedB = LEFT_MOTOR_IS_CHANNEL_A ? rightSpeed : leftSpeed;

    // Disable both bridges during direction/duty updates (including reversal).
    digitalWrite(MOTOR_STBY_PIN, LOW);
    if (!writeChannel(MOTOR_PWMA_PIN, MOTOR_AIN1_PIN, MOTOR_AIN2_PIN, speedA) ||
        !writeChannel(MOTOR_PWMB_PIN, MOTOR_BIN1_PIN, MOTOR_BIN2_PIN, speedB)) {
        latchPwmFault();
        return;
    }
    digitalWrite(MOTOR_STBY_PIN, HIGH);
}

void motorForward(int speed) {
    // Clamp before negating so even INT_MIN is safe.
    speed = clampSpeed(speed);
    if (speed < 0) speed = -speed;
    setMotorSpeeds(speed, speed);
}

void motorStop() {
    if (!ready) return;
    digitalWrite(MOTOR_STBY_PIN, LOW);  // Immediate electrical coast, not brake.
    const bool stoppedA = ledcWrite(MOTOR_PWMA_PIN, 0);
    const bool stoppedB = ledcWrite(MOTOR_PWMB_PIN, 0);
    digitalWrite(MOTOR_AIN1_PIN, LOW);
    digitalWrite(MOTOR_AIN2_PIN, LOW);
    digitalWrite(MOTOR_BIN1_PIN, LOW);
    digitalWrite(MOTOR_BIN2_PIN, LOW);
    if (!stoppedA || !stoppedB) latchPwmFault();
}
