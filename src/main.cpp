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

namespace {
unsigned long lastLedAt = 0;
bool ledOn = false;
}

// Chạy một lần khi ESP32 có điện/reset: khởi tạo Serial, motor, cảm biến,
// chế độ và LED. Sau hàm này robot luôn ở trạng thái STOPPED + DISARMED.
void setup() {
    Serial.begin(SERIAL_BAUD);
    motorInit();
    sensorsInit();
    modesInit(millis());
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
}

// Lặp vô hạn: cập nhật bộ điều khiển chế độ và nháy LED mỗi
// STATUS_LED_INTERVAL_MS (không dùng delay để vòng lặp luôn nhanh).
void loop() {
    const unsigned long now = millis();
    modesUpdate(now);
    if (now - lastLedAt >= STATUS_LED_INTERVAL_MS) {
        lastLedAt = now;
        ledOn = !ledOn;
        digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
    }
}
