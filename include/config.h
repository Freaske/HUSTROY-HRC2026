#pragma once
#include <Arduino.h>

// Week 0 smoke test: GPIO 2 drives a simulated external LED.
// Confirm the actual board's LED pin before testing on hardware.
constexpr uint8_t STATUS_LED_PIN = 2;
constexpr unsigned long STATUS_LED_INTERVAL_MS = 500;

// Proposed initial PWM values from team task assignment;
// NOT validated on actual motors, motor driver, or PWM resolution.
constexpr int MAX_SPEED = 150;
constexpr int BASE_SPEED = 100;

// Future wiring & behavior switches (no hardware is driven by these yet).
constexpr bool INVERT_LEFT_MOTOR = false;
constexpr bool INVERT_RIGHT_MOTOR = false;
constexpr bool INVERT_LINE_SENSOR = false;
constexpr bool INVERT_PID_ERROR = false;

// TODO (Toan): Confirm each ESP32 GPIO for:
//   TCRT5000 1..5 sensor outputs (five separate inputs),
//   TB6612 AIN1/AIN2/BIN1/BIN2/PWMA/PWMB/STBY,
//   ultrasonic TRIG/ECHO, push button (if used).
// Do not guess pin mapping. Record actual wiring in docs/pin_mapping.md.
