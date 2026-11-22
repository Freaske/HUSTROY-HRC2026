// =============================================================================
// main.cpp — Điểm vào của firmware (Arduino setup/loop).
// - setup(): mở Serial, khởi tạo motor (luôn ở trạng thái DỪNG), cảm biến và
//   bộ quản lý chế độ (MOTOR_TEST / SENSOR_TEST / LINE_FOLLOW).
// - loop(): gọi modesUpdate() liên tục để đọc cảm biến, chạy PID, xử lý lệnh
//   Serial; đồng thời nháy LED trạng thái để biết ESP32 đang chạy.
// Mọi logic điều khiển nằm trong modes.cpp; file này chỉ nối các module lại.
// =============================================================================
#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "sensors.h"
#include "modes.h"
#include "test_link.h"

namespace {
unsigned long ledStartedAt = 0;
bool ledOn = false;
}

// Chạy một lần khi ESP32 có điện/reset: khởi tạo Serial, motor, cảm biến,
// chế độ và LED. Sau hàm này robot luôn ở trạng thái STOPPED + DISARMED.
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.printf("FW=%s | LED GPIO%u: double blink\n", FIRMWARE_ID,
                  static_cast<unsigned int>(STATUS_LED_PIN));
    motorInit();
    sensorsInit();
    modesInit(millis());
    testLinkInit();
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    ledStartedAt = millis();
}

// Lặp vô hạn: cập nhật bộ điều khiển chế độ và nháy LED mỗi
// STATUS_LED_INTERVAL_MS (không dùng delay để vòng lặp luôn nhanh).
void loop() {
    const unsigned long now = millis();
    modesUpdate(now);
    testLinkUpdate(millis());
    const unsigned long phase = (now - ledStartedAt) % STATUS_LED_CYCLE_MS;
    const bool nextLedOn = phase < STATUS_LED_PULSE_MS ||
                          (phase >= 2 * STATUS_LED_PULSE_MS && phase < 3 * STATUS_LED_PULSE_MS);
    if (nextLedOn != ledOn) {
        ledOn = nextLedOn;
        digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
    }
}
