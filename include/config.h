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


#ifdef HUSTROY_BOOT_BENCH
constexpr char FIRMWARE_ID[] = "line-follow-v5-boot-bench";
constexpr bool BOOT_BENCH_PROFILE = true;
#else
constexpr char FIRMWARE_ID[] = "line-follow-v5-r2";
constexpr bool BOOT_BENCH_PROFILE = false;
#endif
constexpr uint8_t STATUS_LED_PIN = 2;
// GPIO2: ON 100ms, OFF 100ms, ON 100ms, OFF 700ms. No delay().
constexpr unsigned long STATUS_LED_PULSE_MS = 100;
constexpr unsigned long STATUS_LED_CYCLE_MS = 1000;
static_assert(STATUS_LED_PULSE_MS > 0 &&
              STATUS_LED_CYCLE_MS > 3 * STATUS_LED_PULSE_MS,
              "LED cycle must contain two pulses followed by a pause");

// PWM 8 bit: 0..255. These team starting values are NOT calibrated RPM.
constexpr int MAX_SPEED = 200;
constexpr int BASE_SPEED = 60;

// User-requested motor profile; startup still STOPPED/DISARMED.
// Pins match the current wiring table (PWMA=D25, AIN1=D26, AIN2=D27).
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
constexpr bool INVERT_LEFT_MOTOR = true;
constexpr bool INVERT_RIGHT_MOTOR = false;

constexpr uint8_t MOTOR_PWM_BITS = 8;
constexpr int MOTOR_PWM_MAX = (1 << MOTOR_PWM_BITS) - 1;
constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ = 20000;
constexpr unsigned long MOTOR_TEST_TIMEOUT_MS = 10000; // User-requested manual pulse: 10 seconds.
constexpr int RIGHT_LINE_TEST_PWM = 170;
constexpr unsigned long RIGHT_LINE_TEST_MAX_RUN_MS = 10000;
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
constexpr std::array<int, SENSOR_COUNT> SENSOR_PINS = {{33, 32, 34, 35, 36}};
// Binary diagnostics use the thresholds captured from the pre-V5 ESP32.
constexpr std::array<int, SENSOR_COUNT> SENSOR_ANALOG_THRESHOLDS = {{2326, 2322, 572, 2031, 2343}};
constexpr uint8_t SENSOR_ADC_BITS = 12;
constexpr int SENSOR_ADC_MAX = (1 << SENSOR_ADC_BITS) - 1;
// False: raw HIGH / above-threshold is provisionally active. True reverses it.
// Raw readings are available for calibration while this confirmation is false.
constexpr bool INVERT_LINE_SENSOR = false;
constexpr bool SENSOR_CALIBRATION_CONFIRMED = true;
// Confirm input bias/continuity and unplug tests before authorizing RUN.
// GPIO34..39 have no internal pull resistors; floating inputs are unsafe.
constexpr bool SENSOR_DISCONNECT_BIAS_CONFIRMED = true;
constexpr bool INVERT_PID_ERROR = false;

// Initial V5 bench gains in duty per sensor pitch and duty per sensor pitch/s.
constexpr float PID_KP = 34.0f;
constexpr float PID_KD = 0.45f;
constexpr int LINE_FOLLOW_STEER_TRIM = 4; // + boosts left / reduces right to counter leftward drift
constexpr unsigned long CONTROL_PERIOD_MS = 20;
constexpr unsigned long CONTROL_MAX_GAP_MS = 100;
constexpr unsigned long DEBUG_LOG_INTERVAL_MS = 250;
constexpr unsigned int LINE_START_VALID_SAMPLES = 3;
constexpr unsigned long LINE_FOLLOW_MAX_RUN_MS = 120000; // 120 s lap budget per RUN

// Optional pause/resume and line auto-start. All runs retain the configured deadline.
constexpr bool LINE_AUTO_RESUME = false;
constexpr bool AUTO_START_ON_BOOT = false;
constexpr unsigned long AUTO_START_DELAY_MS = 3000;
static_assert(!AUTO_START_ON_BOOT || LINE_AUTO_RESUME, "Auto-start needs LINE_AUTO_RESUME");

constexpr bool MODE_BUTTON_ENABLED = false;
constexpr int MODE_BUTTON_PIN = -1; // unassigned (GPIO27 is used by MOTOR_AIN2_PIN)
constexpr unsigned long MODE_BUTTON_DEBOUNCE_MS = 30;
constexpr bool BOOT_START_BUTTON_ENABLED = true;
constexpr int BOOT_START_BUTTON_PIN = 0; // Onboard ESP32 BOOT button (GPIO0, active LOW)
constexpr unsigned long BOOT_START_COUNTDOWN_MS = 1500;
constexpr int BOOT_BENCH_PWM = 100;
constexpr unsigned long BOOT_BENCH_RUN_MS = 5000;
static_assert(!BOOT_BENCH_PROFILE || (BOOT_START_BUTTON_ENABLED && BOOT_BENCH_PWM <= MAX_SPEED &&
              BOOT_BENCH_RUN_MS <= MOTOR_TEST_TIMEOUT_MS), "Invalid BOOT bench profile");
// Reservations only, never initialized here. The optional test radios use onboard hardware.
constexpr int RESERVED_ULTRASONIC_TRIG_PIN = -1; // unassigned (GPIO25 is used by MOTOR_PWMA_PIN)
constexpr int RESERVED_ULTRASONIC_ECHO_PIN = -1; // unassigned (GPIO26 is used by MOTOR_AIN1_PIN)

static_assert(SENSOR_ADC_BITS == 12, "Threshold interface uses 12-bit ADC counts");
static_assert(CONTROL_PERIOD_MS >= 5 && CONTROL_PERIOD_MS <= 100,
              "Control period must be 5..100 ms");
static_assert(CONTROL_MAX_GAP_MS > CONTROL_PERIOD_MS && CONTROL_MAX_GAP_MS <= 1000,
              "Control stale limit must exceed period and be at most 1000 ms");
static_assert(DEBUG_LOG_INTERVAL_MS >= 100, "Do not flood the serial control loop");
static_assert(LINE_START_VALID_SAMPLES > 0, "Require valid frames before RUN");
static_assert(LINE_FOLLOW_MAX_RUN_MS > 0 && LINE_FOLLOW_MAX_RUN_MS <= 120000,
              "Initial line trials must be finite: 1..120000 ms per RUN");
static_assert(PID_KP >= 0 && PID_KD >= 0, "PD defaults must be nonnegative");
static_assert(LINE_FOLLOW_STEER_TRIM >= -30 && LINE_FOLLOW_STEER_TRIM <= 30,
              "Steer trim must be -30..30");
static_assert(!BOOT_START_BUTTON_ENABLED ||
              (BOOT_START_BUTTON_PIN == 0 && BOOT_START_COUNTDOWN_MS >= 500 && BOOT_START_COUNTDOWN_MS <= 5000),
              "Onboard BOOT start button must use GPIO0 with 500..5000 ms countdown");
