// =============================================================================
// pid.h — Khai báo bộ điều khiển PID và kiểu dữ liệu tốc độ 2 bánh
// (cài đặt ở pid.cpp). Dùng trong chế độ LINE_FOLLOW.
// =============================================================================
#pragma once

// Hệ số và giới hạn của PID.
struct PidSettings {
    float kp;
    float ki;
    float kd;
    float integralLimit;
    float outputLimit;
};
// Kết quả một bước PID: output tổng và từng thành phần P/I/D.
struct PidResult {
    bool valid = false;
    float output = 0;
    float p = 0;
    float i = 0;
    float d = 0;
};
class PidController {
public:
    // Tạo bộ PID với hệ số/giới hạn cho trước.
    explicit PidController(PidSettings settings) : settings_(settings) {}
    void configure(PidSettings settings);
    // Xóa tích phân và sai số trước.
    void reset();
    // Tính một bước PID từ sai số và dt (giây).
    PidResult update(float error, float dtSeconds);
    // Đọc giá trị tích phân hiện tại (dùng cho test).
    float integral() const { return integral_; }
private:
    PidSettings settings_;
    float integral_ = 0;
    float previousError_ = 0;
    bool hasPrevious_ = false;
};

// Tốc độ bánh trái/phải sau khi trộn hiệu chỉnh PID.
struct WheelSpeeds { int left = 0; int right = 0; bool valid = false; };
// Hiệu chỉnh tối đa để base ± hiệu chỉnh không vượt 0..maximum.
float forwardCorrectionLimit(int base, int maximum, float requested);
// Tính tốc độ trái/phải (chỉ tiến) từ tốc độ nền và hiệu chỉnh PID.
WheelSpeeds mixForwardSpeeds(int base, float correction, int maximum);
