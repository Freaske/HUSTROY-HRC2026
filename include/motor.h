// =============================================================================
// motor.h — Giao diện điều khiển 2 motor qua TB6612FNG (cài đặt ở motor.cpp).
// Các hàm đều an toàn khi gọi trước init hoặc khi phần cứng motor bị tắt.
// =============================================================================
#pragma once

// Khởi tạo chân và PWM cho driver; motor luôn ở trạng thái dừng sau khi gọi.
void motorInit();

// Đặt tốc độ trái/phải. Signed 8-bit duty command, clamped to +/-MAX_SPEED.
// NOT an RPM target. A zero-speed wheel coasts.
void setMotorSpeeds(int leftSpeed, int rightSpeed);
// Cả hai bánh chạy tiến với độ lớn |speed|.
void motorForward(int speed);
// Dừng ngay cả hai motor (thả trôi, không phanh).
void motorStop();

// True only after validated pins and successful PWM initialization.
bool motorIsReady();
