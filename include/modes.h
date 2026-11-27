// =============================================================================
// modes.h — Bộ quản lý chế độ của robot (cài đặt ở modes.cpp).
// 3 chế độ: MOTOR_TEST (thử motor bằng lệnh), SENSOR_TEST (chỉ in cảm biến),
// LINE_FOLLOW (dò line bằng PID). Motor chỉ chạy khi đã ARM.
// =============================================================================
#pragma once
#include "sensors.h"

enum class RobotMode { MOTOR_TEST, SENSOR_TEST, LINE_FOLLOW, RIGHT_LINE };
// Toàn bộ trạng thái hiện tại của robot (chế độ, ARM, lệnh motor, cảm biến...).
struct RobotStatus {
    RobotMode mode = RobotMode::MOTOR_TEST;
    bool armed = false; // Operator authorization, separate from hardware readiness.
    bool running = false;
    int leftCommand = 0;
    int rightCommand = 0;
    float error = 0;
    float correction = 0;
    SensorFrame sensors;
    const char* stopReason = "STARTUP";
};

// Khởi tạo trạng thái (STOPPED + DISARMED) và hẹn giờ tự khởi động nếu bật.
void modesInit(unsigned long now);
// Gọi liên tục trong loop(): đọc cảm biến, chạy PID, xử lý Serial/nút, timeout.
void modesUpdate(unsigned long now);
// Chuyển chế độ. Always stops; never arms.
void modesSelect(RobotMode mode, unsigned long now);
// Bắt đầu dò line. LINE_FOLLOW start request; requires prior ARM.
bool modesRun(unsigned long now);
// Cho phép motor chạy. Stops first; no motion. Only MOTOR_TEST / LINE_FOLLOW.
bool modesArm();
// Dừng motor và thu hồi quyền chạy (phải ARM lại).
void modesDisarm(const char* reason);
// Dừng motor nhưng vẫn giữ ARM.
void modesStop(const char* reason);
// Đọc trạng thái hiện tại (chỉ đọc).
const RobotStatus& modesStatus();
// True trong lúc đếm ngược sau khi nhấn-thả nút BOOT / chờ tự bám line (LED nháy nhanh).
bool modesCountdownActive();
// True khi chế độ tự bám line đã ARM và đang chờ line hoặc đang chạy.
bool modesAutoLineActive();
// Bật (đếm ngược rồi chờ line) hoặc tắt chế độ tự bám line. Bật bị bỏ qua khi đang chạy.
void modesAutoLine(bool enable, unsigned long now);
// Tên chuỗi của chế độ (để in Serial).
const char* modeName(RobotMode mode);
