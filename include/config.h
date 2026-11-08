#pragma once
#include <Arduino.h>
#include "sensors.h"


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

// Sensors: no wiring or AO/DO choice has been confirmed. No GPIO is touched
// until enabled with a valid mapping. Order is LEFT -> RIGHT facing forward.
constexpr bool SENSOR_HARDWARE_ENABLED = false;
constexpr SensorInputMode SENSOR_INPUT_MODE = SensorInputMode::UNCONFIGURED;
constexpr std::array<int, SENSOR_COUNT> SENSOR_PINS = {{-1, -1, -1, -1, -1}};
constexpr std::array<int, SENSOR_COUNT> SENSOR_ANALOG_THRESHOLDS = {{2048, 2048, 2048, 2048, 2048}};
constexpr uint8_t SENSOR_ADC_BITS = 12;
constexpr int SENSOR_ADC_MAX = (1 << SENSOR_ADC_BITS) - 1;
// False: raw HIGH / above-threshold is provisionally active. True reverses it.
// Raw readings are available for calibration while this confirmation is false.
constexpr bool INVERT_LINE_SENSOR = false;
constexpr bool SENSOR_CALIBRATION_CONFIRMED = false;
// Confirm input bias/continuity and unplug tests before authorizing RUN.
// GPIO34..39 have no internal pull resistors; floating inputs are unsafe.
constexpr bool SENSOR_DISCONNECT_BIAS_CONFIRMED = false;
constexpr bool INVERT_PID_ERROR = false;

// Teammate's starting gains, NOT tuned. Ki/Kd now use seconds.
constexpr float PID_KP = 20.0f;
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 0.0f;
constexpr float PID_INTEGRAL_LIMIT = 2.0f; // error * seconds
constexpr float PID_CORRECTION_LIMIT = 50.0f; // duty units, further limited by wheel headroom
constexpr unsigned long CONTROL_PERIOD_MS = 20;
constexpr unsigned long CONTROL_MAX_GAP_MS = 100;
constexpr unsigned long DEBUG_LOG_INTERVAL_MS = 250;
constexpr unsigned int LINE_START_VALID_SAMPLES = 3;
constexpr unsigned long LINE_FOLLOW_MAX_RUN_MS = 5000; // finite bench trial per RUN

constexpr bool MODE_BUTTON_ENABLED = false;
constexpr int MODE_BUTTON_PIN = -1; // proposal: GPIO27, normally-open switch to GND
constexpr unsigned long MODE_BUTTON_DEBOUNCE_MS = 30;
// Reservations only, never initialized here. HC-SR04P/Bluetooth not implemented.
constexpr int RESERVED_ULTRASONIC_TRIG_PIN = -1; // proposal: 25
constexpr int RESERVED_ULTRASONIC_ECHO_PIN = -1; // proposal: 26; verify ECHO voltage

static_assert(SENSOR_ADC_BITS == 12, "Threshold interface uses 12-bit ADC counts");
static_assert(CONTROL_PERIOD_MS >= 5 && CONTROL_PERIOD_MS <= 100,
              "Control period must be 5..100 ms");
static_assert(CONTROL_MAX_GAP_MS > CONTROL_PERIOD_MS && CONTROL_MAX_GAP_MS <= 1000,
              "Control stale limit must exceed period and be at most 1000 ms");
static_assert(DEBUG_LOG_INTERVAL_MS >= 100, "Do not flood the serial control loop");
static_assert(LINE_START_VALID_SAMPLES > 0, "Require valid frames before RUN");
static_assert(LINE_FOLLOW_MAX_RUN_MS > 0 && LINE_FOLLOW_MAX_RUN_MS <= 30000,
              "Initial line trials must be finite: 1..30000 ms per RUN");
static_assert(PID_KP >= 0 && PID_KI >= 0 && PID_KD >= 0 &&
              PID_INTEGRAL_LIMIT > 0 && PID_CORRECTION_LIMIT > 0,
              "PID defaults must be finite positive limits and nonnegative gains");
