// =============================================================================
// sensors.h — Kiểu dữ liệu và hàm của 5 cảm biến dò line (cài đặt ở sensors.cpp).
// Thứ tự mắt: chỉ số 0 = ngoài cùng trái ... 4 = ngoài cùng phải.
// =============================================================================
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

constexpr std::size_t SENSOR_COUNT = 5;
// Cách đọc cảm biến: chưa cấu hình / chân DO (digital) / chân AO (analog).
enum class SensorInputMode { UNCONFIGURED, DIGITAL_DO, ANALOG_AO };
// Trạng thái line: thấy line / mất line / ngã tư / không hợp lệ.
enum class LineState { LINE_DETECTED, LINE_LOST, INTERSECTION, INVALID };
// Hướng line so với tâm xe.
enum class LineDirection { LEFT, RIGHT, FORWARD, INTERSECTION, LOST, INVALID };

// Một lần đọc đầy đủ của 5 cảm biến cùng kết quả diễn giải.
struct SensorFrame {
    std::array<int, SENSOR_COUNT> raw{{-1, -1, -1, -1, -1}};
    std::array<bool, SENSOR_COUNT> active{}; // 1 always means ON THE LINE.
    bool rawAvailable = false;
    bool valid = false; // Requires confirmed polarity/threshold calibration.
    bool allActive = false;
    uint8_t activeCount = 0;
    float position = 0.0f; // Only usable when state == LINE_DETECTED.
    LineState state = LineState::INVALID;
    LineDirection direction = LineDirection::INVALID;
    unsigned long sampledAt = 0;
};

// Tính vị trí/trạng thái line từ 5 bit (trung bình -2..+2 của đồng đội).
SensorFrame analyzeSensorPattern(const std::array<bool, SENSOR_COUNT>& active);
// Chuyển giá trị thô thành bit theo ngưỡng rồi phân tích.
SensorFrame interpretSensorRaw(const std::array<int, SENSOR_COUNT>& raw,
                              SensorInputMode mode,
                              const std::array<int, SENSOR_COUNT>& thresholds,
                              bool invert, bool calibrationConfirmed);
// Cấu hình chân/ADC cho 5 cảm biến.
void sensorsInit();
// Configured for raw reads; does NOT imply calibration.
bool sensorsIsReady();
// Đọc và diễn giải một khung cảm biến tại thời điểm now (ms).
SensorFrame sensorsRead(unsigned long now);
// Tên chuỗi của trạng thái line (để in Serial).
const char* lineStateName(LineState state);
// Tên chuỗi của hướng line (để in Serial).
const char* lineDirectionName(LineDirection direction);
