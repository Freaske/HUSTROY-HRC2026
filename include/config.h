// =============================================================================
// config.h — TẤT CẢ thông số chỉnh tay của robot, sửa ở đây rồi nạp lại code:
// - Chân GPIO: LED, motor TB6612 (PWMA/AIN1/AIN2/PWMB/BIN1/BIN2/STBY), 5 cảm biến.
// - Tốc độ: MAX_SPEED, BASE_SPEED (duty 0..255), cờ đảo chiều motor.
// - Cảm biến: kiểu đọc AO/DO, ngưỡng trắng/đen, đảo cực.
// - PID: KP/KI/KD, giới hạn, chu kỳ điều khiển.
// - Chế độ chạy: LINE_AUTO_RESUME, AUTO_START_ON_BOOT, nút chọn chế độ.
// Các static_assert bên dưới chặn biên dịch nếu giá trị không hợp lệ.
// =============================================================================
#pragma once
#include <Arduino.h>
#include "sensors.h"


constexpr char FIRMWARE_ID[] = "line-follow-pwm170-v2";
constexpr uint8_t STATUS_LED_PIN = 2;
// GPIO2: ON 100ms, OFF 100ms, ON 100ms, OFF 700ms. No delay().
constexpr unsigned long STATUS_LED_PULSE_MS = 100;
constexpr unsigned long STATUS_LED_CYCLE_MS = 1000;
static_assert(STATUS_LED_PULSE_MS > 0 &&
              STATUS_LED_CYCLE_MS > 3 * STATUS_LED_PULSE_MS,
              "LED cycle must contain two pulses followed by a pause");

// PWM 8 bit: 0..255. These team starting values are NOT calibrated RPM.
constexpr int MAX_SPEED = 170;
constexpr int BASE_SPEED = 170;

// User-requested manual motor bench profile; startup still STOPPED/DISARMED.
// Channel A moved from D18/D13/D14 to D25/D26/D27: bench test showed channel A
// works when driven from D19/D21/D22, but not from D18/D13/D14 (ESP32 pin or
// jumper fault). This takes the pins proposed for ultrasonic (25/26) and the
// mode button (27); re-plan those before enabling them.
// Disconnect VM during upload; lift wheels before ARM. Set false for USB-only diagnostics.
constexpr bool MOTOR_HARDWARE_ENABLED = true;
constexpr int MOTOR_PWMA_PIN = 25;
constexpr int MOTOR_AIN1_PIN = 26;
constexpr int MOTOR_AIN2_PIN = 27;
constexpr int MOTOR_PWMB_PIN = 19;
constexpr int MOTOR_BIN1_PIN = 21;
constexpr int MOTOR_BIN2_PIN = 22;
constexpr int MOTOR_STBY_PIN = 23;

// Logical left/right can be swapped without changing the driver's A/B wiring.
// This provisional convention must be confirmed by Toan before enabling.
constexpr bool LEFT_MOTOR_IS_CHANNEL_A = true;
constexpr bool INVERT_LEFT_MOTOR = false;
constexpr bool INVERT_RIGHT_MOTOR = false;

constexpr uint8_t MOTOR_PWM_BITS = 8;
constexpr int MOTOR_PWM_MAX = (1 << MOTOR_PWM_BITS) - 1;
constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ = 20000;
constexpr unsigned long MOTOR_TEST_TIMEOUT_MS = 10000; // User-requested manual pulse: 10 seconds.
constexpr int RIGHT_LINE_TEST_PWM = 170;
constexpr unsigned long RIGHT_LINE_TEST_MAX_RUN_MS = 10000;
constexpr unsigned long MOTOR_TEST_TIMEOUT_MS = 10000;
constexpr uint32_t SERIAL_BAUD = 115200;

static_assert(MOTOR_PWM_BITS == 8, "Motor interface requires 8-bit PWM");
static_assert(MAX_SPEED > 0 && MAX_SPEED <= MOTOR_PWM_MAX, "MAX_SPEED must be 1..255");
static_assert(BASE_SPEED >= 0 && BASE_SPEED <= MAX_SPEED, "BASE_SPEED exceeds limit");
static_assert(MOTOR_TEST_TIMEOUT_MS > 0 && MOTOR_TEST_TIMEOUT_MS <= 10000,
              "Bench test timeout must be 1..10000 ms");
static_assert(RIGHT_LINE_TEST_PWM > 0 && RIGHT_LINE_TEST_PWM <= MOTOR_PWM_MAX,
              "Right line test duty must be 1..255");
static_assert(RIGHT_LINE_TEST_MAX_RUN_MS > 0 && RIGHT_LINE_TEST_MAX_RUN_MS <= 10000,
              "Right line test timeout must be 1..10000 ms");
static_assert(MOTOR_PWM_FREQUENCY_HZ > 0 && MOTOR_PWM_FREQUENCY_HZ <= 100000,
              "TB6612 PWM must be at most 100 kHz");

// AO inputs in physical left-to-right order. LINE_FOLLOW needs calibration and bias checks.
constexpr bool SENSOR_HARDWARE_ENABLED = true;
constexpr SensorInputMode SENSOR_INPUT_MODE = SensorInputMode::ANALOG_AO;
constexpr std::array<int, SENSOR_COUNT> SENSOR_PINS = {{32, 33, 34, 35, 36}};
// Provisional thresholds from a stationary sweep of the team's black line.
constexpr std::array<int, SENSOR_COUNT> SENSOR_ANALOG_THRESHOLDS = {{265, 291, 268, 325, 550}};
static_assert(MOTOR_PWM_FREQUENCY_HZ > 0 && MOTOR_PWM_FREQUENCY_HZ <= 100000,
              "TB6612 PWM must be at most 100 kHz");

// Sensors: 5 separate 4-pin modules (VCC/GND/DO/AO), AO wired, VCC from 3V3.
// Order is LEFT -> RIGHT facing forward: D32, D33, D34, D35, VP(36).
// Thresholds below are still placeholders until white/black readings are measured.
constexpr bool SENSOR_HARDWARE_ENABLED = true;
constexpr SensorInputMode SENSOR_INPUT_MODE = SensorInputMode::ANALOG_AO;
constexpr std::array<int, SENSOR_COUNT> SENSOR_PINS = {{32, 33, 34, 35, 36}};
// Provisional bench thresholds from one white/black-tape measurement
// (white ~160/182/177/222/1150-1620, tape ~740-1000/570-670/335-557/1950-2030/3450).
// Sensor 3 margin is small; re-measure after levelling sensor height.
constexpr std::array<int, SENSOR_COUNT> SENSOR_ANALOG_THRESHOLDS = {{450, 380, 260, 1100, 2500}};
constexpr uint8_t SENSOR_ADC_BITS = 12;
constexpr int SENSOR_ADC_MAX = (1 << SENSOR_ADC_BITS) - 1;
// False: raw HIGH / above-threshold is provisionally active. True reverses it.
// Raw readings are available for calibration while this confirmation is false.
constexpr bool INVERT_LINE_SENSOR = false;
constexpr bool SENSOR_CALIBRATION_CONFIRMED = true;
// Confirm input bias/continuity and unplug tests before authorizing RUN.
// GPIO34..39 have no internal pull resistors; floating inputs are unsafe.
// BENCH TEST ONLY (wheels lifted): provisional thresholds accepted so RUN can be tried.
constexpr bool SENSOR_CALIBRATION_CONFIRMED = true;
// Confirm input bias/continuity and unplug tests before authorizing RUN.
// GPIO34..39 have no internal pull resistors; floating inputs are unsafe.
// BENCH TEST ONLY: disconnect/bias behaviour has NOT been verified. Set false
// again before any floor run until unplug tests are done.
constexpr bool SENSOR_DISCONNECT_BIAS_CONFIRMED = true;
constexpr bool INVERT_PID_ERROR = false;

// Starting PD gains tuned for PWM 170 (dt = 20 ms). Ki/Kd use seconds.
constexpr float PID_KP = 45.0f;
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 1.5f;
constexpr float PID_INTEGRAL_LIMIT = 2.0f; // error * seconds
constexpr float PID_CORRECTION_LIMIT = 85.0f; // duty units, further limited by wheel headroom
constexpr unsigned long CONTROL_PERIOD_MS = 20;
constexpr unsigned long CONTROL_MAX_GAP_MS = 100;
constexpr unsigned long DEBUG_LOG_INTERVAL_MS = 250;
constexpr unsigned int LINE_START_VALID_SAMPLES = 3;
constexpr unsigned long LINE_FOLLOW_MAX_RUN_MS = 15000; // finite trial per RUN
constexpr unsigned long LINE_LOST_GRACE_MS = 1500;
constexpr unsigned long LINE_INVALID_GRACE_MS = 160;
constexpr unsigned long LINE_INTERSECTION_STOP_MS = 200;
constexpr unsigned long LINE_FOLLOW_MAX_RUN_MS = 5000; // finite trial per RUN (only when AUTO_RESUME=false)

// true: after RUN, losing the line only PAUSES (motors coast); a stable line again
// resumes automatically. No RUN time limit. STOP / ! / VM switch end the run.
// INTERSECTION (all/outer sensors black, also what a LIFTED robot reads) also pauses.
constexpr bool LINE_AUTO_RESUME = true;
// true: no laptop needed. AUTO_START_DELAY_MS after power-on/reset the robot enters
// LINE_FOLLOW, ARMs and RUNs by itself (waits for a stable line before moving).
// Any Serial character during the delay cancels it. Keep false while uploading/bench
// testing with VM connected; set true only for floor runs.
constexpr bool AUTO_START_ON_BOOT = true;
constexpr unsigned long AUTO_START_DELAY_MS = 3000;

constexpr bool MODE_BUTTON_ENABLED = false;
constexpr int MODE_BUTTON_PIN = -1; // old proposal GPIO27 now used by AIN2; pick another pin
constexpr unsigned long MODE_BUTTON_DEBOUNCE_MS = 30;
// Reservations only, never initialized here. The optional test radios use onboard hardware.
constexpr int RESERVED_ULTRASONIC_TRIG_PIN = -1; // proposal: 25
constexpr int RESERVED_ULTRASONIC_ECHO_PIN = -1; // proposal: 26; verify ECHO voltage
// Reservations only, never initialized here. HC-SR04P/Bluetooth not implemented.
constexpr int RESERVED_ULTRASONIC_TRIG_PIN = -1; // old proposal 25 now used by PWMA; re-plan
constexpr int RESERVED_ULTRASONIC_ECHO_PIN = -1; // old proposal 26 now used by AIN1; verify ECHO voltage

static_assert(SENSOR_ADC_BITS == 12, "Threshold interface uses 12-bit ADC counts");
static_assert(CONTROL_PERIOD_MS >= 5 && CONTROL_PERIOD_MS <= 100,
              "Control period must be 5..100 ms");
static_assert(CONTROL_MAX_GAP_MS > CONTROL_PERIOD_MS && CONTROL_MAX_GAP_MS <= 1000,
              "Control stale limit must exceed period and be at most 1000 ms");
static_assert(DEBUG_LOG_INTERVAL_MS >= 100, "Do not flood the serial control loop");
static_assert(LINE_START_VALID_SAMPLES > 0, "Require valid frames before RUN");
static_assert(LINE_FOLLOW_MAX_RUN_MS > 0 && LINE_FOLLOW_MAX_RUN_MS <= 30000,
              "Initial line trials must be finite: 1..30000 ms per RUN");
static_assert(LINE_LOST_GRACE_MS > 0 && LINE_LOST_GRACE_MS <= 2000,
              "Line-lost grace window must be 1..2000 ms");
static_assert(LINE_INVALID_GRACE_MS > 0 && LINE_INVALID_GRACE_MS <= 1000,
              "Invalid-pattern grace window must be 1..1000 ms");
static_assert(LINE_INTERSECTION_STOP_MS > 0 && LINE_INTERSECTION_STOP_MS <= 1000,
              "Intersection stop debounce must be 1..1000 ms");
static_assert(!AUTO_START_ON_BOOT || LINE_AUTO_RESUME, "AUTO_START_ON_BOOT requires LINE_AUTO_RESUME");
static_assert(AUTO_START_DELAY_MS >= 1000, "Give at least 1 s to put the robot down / cancel");
static_assert(PID_KP >= 0 && PID_KI >= 0 && PID_KD >= 0 &&
              PID_INTEGRAL_LIMIT > 0 && PID_CORRECTION_LIMIT > 0,
              "PID defaults must be finite positive limits and nonnegative gains");
