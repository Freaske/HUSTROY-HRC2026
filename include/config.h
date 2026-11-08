#pragma once
#include <Arduino.h>


constexpr uint8_t STATUS_LED_PIN = 2;
constexpr unsigned long STATUS_LED_INTERVAL_MS = 500;

// Proposed initial PWM values from team task assignment;
// NOT validated on actual motors, motor driver, or PWM resolution.
constexpr int MAX_SPEED = 150;
constexpr int BASE_SPEED = 50;

// Future wiring & behavior switches (no hardware is driven by these yet).
constexpr bool INVERT_LEFT_MOTOR = false;
constexpr bool INVERT_RIGHT_MOTOR = false;
constexpr bool INVERT_LINE_SENSOR = false;
constexpr bool INVERT_PID_ERROR = false;
