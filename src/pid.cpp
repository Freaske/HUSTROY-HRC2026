// =============================================================================
// pid.cpp — Bộ điều khiển PID và hàm trộn tốc độ 2 bánh cho chế độ dò line.
// - PidController: tính hiệu chỉnh từ sai số vị trí line (-2..+2), có giới hạn
//   tích phân (chống bão hòa) và giới hạn đầu ra; thời gian tính bằng giây.
// - forwardCorrectionLimit / mixForwardSpeeds: biến hiệu chỉnh PID thành tốc
//   độ trái/phải, chỉ cho chạy tiến (không bao giờ ra số âm).
// Không gọi phần cứng nên kiểm thử được trên máy tính.
// =============================================================================
#include "pid.h"
#include <algorithm>
#include <cmath>

void PidController::configure(PidSettings settings) {
    if (settings.kp != settings_.kp || settings.ki != settings_.ki || settings.kd != settings_.kd) reset();
    settings_ = settings;
}

// Xóa trạng thái PID (tích phân, sai số trước) — gọi khi bắt đầu/tiếp tục chạy.
void PidController::reset() {
    integral_ = 0;
    previousError_ = 0;
    hasPrevious_ = false;
}

// Tính một bước PID từ sai số hiện tại và khoảng thời gian dtSeconds.
// Trả về output = P + I + D đã giới hạn; valid = false (và reset) nếu đầu vào
// hoặc hệ số không hợp lệ (NaN, dt ngoài 1 ms..1 s, hệ số âm...).
PidResult PidController::update(float error, float dtSeconds) {
    PidResult result;
    if (!std::isfinite(error) || !std::isfinite(dtSeconds) || dtSeconds < 0.001f || dtSeconds > 1.0f ||
        !std::isfinite(settings_.kp) || !std::isfinite(settings_.ki) || !std::isfinite(settings_.kd) ||
        !std::isfinite(settings_.integralLimit) || !std::isfinite(settings_.outputLimit) ||
        settings_.kp < 0 || settings_.ki < 0 || settings_.kd < 0 ||
        settings_.integralLimit <= 0 || settings_.outputLimit <= 0) {
        reset();
        return result;
    }
    // Teammate's P + I + D, corrected to seconds and with first-step D = 0.
    const float candidate = std::clamp(integral_ + error * dtSeconds,
                                       -settings_.integralLimit, settings_.integralLimit);
    result.p = settings_.kp * error;
    result.d = hasPrevious_ ? settings_.kd * (error - previousError_) / dtSeconds : 0.0f;
    const float candidateOutput = result.p + settings_.ki * candidate + result.d;
    if (!std::isfinite(candidateOutput)) {
        reset();
        return PidResult{};
    }
    // Conditional integration: never accumulate further into saturation.
    if (!((candidateOutput > settings_.outputLimit && error > 0) ||
          (candidateOutput < -settings_.outputLimit && error < 0))) integral_ = candidate;
    // Use the candidate term for this command, but only store it when allowed.
    // This also handles an I-only controller whose first increment saturates.
    result.i = settings_.ki * candidate;
    result.output = std::clamp(candidateOutput,
                              -settings_.outputLimit, settings_.outputLimit);
    result.valid = true;
    previousError_ = error;
    hasPrevious_ = true;
    return result;
}

// Giới hạn hiệu chỉnh PID tối đa sao cho base ± hiệu chỉnh vẫn nằm trong
// 0..maximum. Trả về 0 nếu tham số không hợp lệ (khi đó RUN bị chặn).
float forwardCorrectionLimit(int base, int maximum, float requested) {
    if (base <= 0 || base > maximum || !std::isfinite(requested) || requested <= 0) return 0;
    return std::min({requested, static_cast<float>(base), maximum / 2.0f});
}

// Trộn tốc độ: trái = base + correction, phải = base - correction, kẹp trong
// 0..maximum (chỉ chạy tiến). valid = false nếu tham số không hợp lệ.
WheelSpeeds mixForwardSpeeds(int base, float correction, int maximum) {
    WheelSpeeds speeds;
    if (maximum <= 0 || base < 0 || base > maximum || !std::isfinite(correction)) return speeds;
    const float ceiling = static_cast<float>(maximum);
    const float offset = std::clamp(correction, -ceiling, ceiling);
    // Reduce the common PWM at the ceiling to preserve the steering difference.
    const float center = std::min(static_cast<float>(base), ceiling - std::abs(offset));
    const float left = std::max(0.0f, center + offset);
    const float right = std::max(0.0f, center - offset);
    speeds.left = static_cast<int>(std::lround(left));
    speeds.right = static_cast<int>(std::lround(right));
    speeds.valid = true;
    return speeds;
}
