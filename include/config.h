#pragma once
#include <Arduino.h>


constexpr uint8_t STATUS_LED_PIN = 2;
constexpr unsigned long STATUS_LED_INTERVAL_MS = 500;

// PWM 8 bit: 0..255. These team starting values are NOT calibrated RPM.
constexpr int MAX_SPEED = 150;
constexpr int BASE_SPEED = 50;

// HARDWARE LOCK: only enable after Toan confirms wiring, board and supplies.
// -1 means UNASSIGNED, never an ESP32 GPIO. No final mapping is assumed.
constexpr bool MOTOR_HARDWARE_ENABLED = false;
constexpr int MOTOR_PWMA_PIN = -1;
constexpr int MOTOR_AIN1_PIN = -1;
constexpr int MOTOR_AIN2_PIN = -1;
constexpr int MOTOR_PWMB_PIN = -1;
constexpr int MOTOR_BIN1_PIN = -1;
constexpr int MOTOR_BIN2_PIN = -1;
constexpr int MOTOR_STBY_PIN = -1;

// Logical left/right can be swapped without changing the driver's A/B wiring.
// This provisional convention must be confirmed by Toan before enabling.
constexpr bool LEFT_MOTOR_IS_CHANNEL_A = true;
constexpr bool INVERT_LEFT_MOTOR = false;
constexpr bool INVERT_RIGHT_MOTOR = false;

constexpr uint8_t MOTOR_PWM_BITS = 8;
constexpr int MOTOR_PWM_MAX = (1 << MOTOR_PWM_BITS) - 1;
constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ = 20000;
constexpr unsigned long MOTOR_TEST_TIMEOUT_MS = 500;
constexpr uint32_t SERIAL_BAUD = 115200;

static_assert(MOTOR_PWM_BITS == 8, "Motor interface requires 8-bit PWM");
static_assert(MAX_SPEED > 0 && MAX_SPEED <= MOTOR_PWM_MAX, "MAX_SPEED must be 1..255");
static_assert(BASE_SPEED >= 0 && BASE_SPEED <= MAX_SPEED, "BASE_SPEED exceeds limit");
static_assert(MOTOR_TEST_TIMEOUT_MS > 0 && MOTOR_TEST_TIMEOUT_MS <= 2000,
              "Bench test timeout must be 1..2000 ms");
static_assert(MOTOR_PWM_FREQUENCY_HZ > 0 && MOTOR_PWM_FREQUENCY_HZ <= 100000,
              "TB6612 PWM must be at most 100 kHz");

// Future module settings, preserved for later phases.
constexpr bool INVERT_LINE_SENSOR = false;
constexpr bool INVERT_PID_ERROR = false;
