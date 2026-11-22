// =============================================================================
// motor.cpp — Driver điều khiển 2 motor qua mạch cầu H TB6612FNG.
// - Kênh A: PWMA/AIN1/AIN2, kênh B: PWMB/BIN1/BIN2, chung chân STBY.
// - PWM dùng LEDC của Arduino-ESP32 3.x (8 bit, 20 kHz).
// - Kiểm tra GPIO an toàn ngay lúc biên dịch (static_assert).
// - Mọi lỗi PWM đều "khóa" driver (STBY LOW) cho đến khi reset ESP32.
// - Tốc độ là duty có dấu (-MAX_SPEED..MAX_SPEED), không phải vòng/phút.
// =============================================================================
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <driver/gpio.h>
#include "motor.h"
#include "config.h"

#if ESP_ARDUINO_VERSION_MAJOR != 3
#error "This driver uses Arduino-ESP32 3.x pin-based LEDC; install the documented platform."
#endif

namespace {
constexpr int motorPins[] = {MOTOR_PWMA_PIN, MOTOR_AIN1_PIN, MOTOR_AIN2_PIN,
                             MOTOR_PWMB_PIN, MOTOR_BIN1_PIN, MOTOR_BIN2_PIN,
                             MOTOR_STBY_PIN};

// Kiểm tra một GPIO có được phép dùng làm chân điều khiển motor không
// (loại chân flash, UART0, chân strapping, chân chỉ-input và LED trạng thái).
// Conservative classic ESP32 policy: reject flash/PSRAM, UART0, boot straps,
// input-only/nonexistent pins and the existing status LED. This is a validity
// check, NOT a proposed wiring map. Re-audit for any different ESP32 board.
constexpr bool isSafeMotorPin(int pin) {
    return (pin == 4 || pin == 13 || pin == 14 || (pin >= 18 && pin <= 19) ||
            (pin >= 21 && pin <= 23) || (pin >= 25 && pin <= 27) ||
            pin == 32 || pin == 33) && pin != STATUS_LED_PIN;
}

// Kiểm tra cả 7 chân motor: chân nào cũng an toàn và không có chân nào trùng nhau.
constexpr bool motorPinsValid() {
    for (unsigned int i = 0; i < sizeof(motorPins) / sizeof(motorPins[0]); ++i) {
        if (!isSafeMotorPin(motorPins[i])) return false;
        for (unsigned int j = 0; j < i; ++j) {
            if (motorPins[i] == motorPins[j]) return false;
        }
    }
    return true;
}

static_assert(!MOTOR_HARDWARE_ENABLED || motorPinsValid(),
              "Motor GPIOs unassigned, duplicated, reserved or unsafe; ask Toan.");
#if !defined(CONFIG_IDF_TARGET_ESP32)
static_assert(!MOTOR_HARDWARE_ENABLED, "Re-audit GPIO policy for this ESP32 variant.");
#endif

bool initAttempted = false;
bool ready = false;

// Giới hạn tốc độ trong khoảng [-MAX_SPEED, MAX_SPEED].
int clampSpeed(int speed) {
    if (speed > MAX_SPEED) return MAX_SPEED;
    if (speed < -MAX_SPEED) return -MAX_SPEED;
    return speed;
}

// Ghi chiều (IN1/IN2) và duty (PWM) cho MỘT kênh motor. speed > 0: tiến,
// < 0: lùi, = 0: thả trôi (coast). Trả về false nếu ghi PWM thất bại.
// Called with STBY LOW. PWM HIGH + IN1/IN2 LOW selects high-impedance
// coast when the other wheel is moving. PWM LOW alone would select braking.
bool writeChannel(int pwm, int in1, int in2, int speed) {
    digitalWrite(in1, speed > 0 ? HIGH : LOW);
    digitalWrite(in2, speed < 0 ? HIGH : LOW);
    const int duty = speed == 0 ? MOTOR_PWM_MAX : (speed < 0 ? -speed : speed);
    return ledcWrite(pwm, duty);
}

// Xử lý lỗi PWM: hạ STBY (tắt cả 2 cầu H) và đánh dấu driver không sẵn sàng.
// Driver chỉ hoạt động lại sau khi reset ESP32.
void latchPwmFault() {
    digitalWrite(MOTOR_STBY_PIN, LOW);
    ready = false;  // No automatic restart after a failed PWM operation.
    Serial.println("ERROR: PWM failure; STBY LOW. Fix cause and reset ESP32.");
}
}  // namespace

// Khởi tạo driver một lần: đặt mọi chân motor về LOW (STBY trước tiên),
// gắn 2 kênh PWM. Thành công thì ready = true và motor ở trạng thái dừng.
// Gọi lại lần nữa chỉ dừng motor, không khởi tạo lại.
void motorInit() {
    if (initAttempted) {
        motorStop();
        return;
    }
    initAttempted = true;
    if (!MOTOR_HARDWARE_ENABLED || !motorPinsValid()) {
        Serial.println("MOTOR DISABLED: confirm 7 GPIOs and wiring in config.h with Toan.");
        return;  // Do not touch ANY motor GPIO/LEDC in unconfigured mode.
    }

    // Hold hardware disabled before configuring any other control signals.
    // Arduino core 3.x digitalWrite cannot preload a pin before pinMode.
    // The native GPIO API preloads LOW before enabling the output driver.
    gpio_set_level(static_cast<gpio_num_t>(MOTOR_STBY_PIN), 0);
    pinMode(MOTOR_STBY_PIN, OUTPUT);
    digitalWrite(MOTOR_STBY_PIN, LOW);
    for (int pin : motorPins) {
        if (pin == MOTOR_STBY_PIN) continue;
        gpio_set_level(static_cast<gpio_num_t>(pin), 0);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    if (!ledcAttach(MOTOR_PWMA_PIN, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_BITS) ||
        !ledcAttach(MOTOR_PWMB_PIN, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_BITS) ||
        !ledcWrite(MOTOR_PWMA_PIN, 0) || !ledcWrite(MOTOR_PWMB_PIN, 0)) {
        latchPwmFault();
        return;
    }
    ready = true;
    motorStop();
    if (ready) Serial.println("MOTOR READY: stopped, STBY LOW. Awaiting an explicit command.");
}

// Trả về true nếu driver đã khởi tạo thành công và chưa gặp lỗi PWM.
bool motorIsReady() {
    return ready;
}

// Đặt tốc độ bánh trái/phải (duty có dấu). Tự áp dụng giới hạn MAX_SPEED,
// cờ đảo chiều và quy ước trái/phải = kênh A/B. Cả hai bằng 0 thì dừng hẳn.
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if (!ready) return;
    leftSpeed = clampSpeed(leftSpeed);
    rightSpeed = clampSpeed(rightSpeed);
    if (leftSpeed == 0 && rightSpeed == 0) {
        motorStop();
        return;
    }

    if (INVERT_LEFT_MOTOR) leftSpeed = -leftSpeed;
    if (INVERT_RIGHT_MOTOR) rightSpeed = -rightSpeed;
    const int speedA = LEFT_MOTOR_IS_CHANNEL_A ? leftSpeed : rightSpeed;
    const int speedB = LEFT_MOTOR_IS_CHANNEL_A ? rightSpeed : leftSpeed;

    // Disable both bridges during direction/duty updates (including reversal).
    digitalWrite(MOTOR_STBY_PIN, LOW);
    if (!writeChannel(MOTOR_PWMA_PIN, MOTOR_AIN1_PIN, MOTOR_AIN2_PIN, speedA) ||
        !writeChannel(MOTOR_PWMB_PIN, MOTOR_BIN1_PIN, MOTOR_BIN2_PIN, speedB)) {
        latchPwmFault();
        return;
    }
    digitalWrite(MOTOR_STBY_PIN, HIGH);
}

// Cho cả hai bánh chạy tiến cùng một tốc độ (lấy giá trị tuyệt đối của speed).
void motorForward(int speed) {
    // Clamp before negating so even INT_MIN is safe.
    speed = clampSpeed(speed);
    if (speed < 0) speed = -speed;
    setMotorSpeeds(speed, speed);
}

// Dừng cả hai motor ngay: STBY LOW, PWM = 0, mọi chân IN = LOW (thả trôi,
// không phanh). Nếu ghi PWM lỗi thì khóa driver.
void motorStop() {
    if (!ready) return;
    digitalWrite(MOTOR_STBY_PIN, LOW);  // Immediate electrical coast, not brake.
    const bool stoppedA = ledcWrite(MOTOR_PWMA_PIN, 0);
    const bool stoppedB = ledcWrite(MOTOR_PWMB_PIN, 0);
    digitalWrite(MOTOR_AIN1_PIN, LOW);
    digitalWrite(MOTOR_AIN2_PIN, LOW);
    digitalWrite(MOTOR_BIN1_PIN, LOW);
    digitalWrite(MOTOR_BIN2_PIN, LOW);
    if (!stoppedA || !stoppedB) latchPwmFault();
}
