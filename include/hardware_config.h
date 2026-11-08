#pragma once
#include "config.h"

// Conservative classic ESP32 policy; does not claim a board/wiring is confirmed.
constexpr bool hardwareOutputPinAllowed(int pin) {
    return pin == 4 || pin == 13 || pin == 14 || pin == 18 || pin == 19 ||
           pin == 21 || pin == 22 || pin == 23 || pin == 25 || pin == 26 ||
           pin == 27 || pin == 32 || pin == 33;
}
constexpr bool hardwareAdc1Pin(int pin) {
    return pin == 32 || pin == 33 || pin == 34 || pin == 35 || pin == 36 || pin == 39;
}
constexpr bool hardwareMotorUses(int pin) {
    return MOTOR_HARDWARE_ENABLED && (pin == MOTOR_PWMA_PIN || pin == MOTOR_AIN1_PIN ||
           pin == MOTOR_AIN2_PIN || pin == MOTOR_PWMB_PIN || pin == MOTOR_BIN1_PIN ||
           pin == MOTOR_BIN2_PIN || pin == MOTOR_STBY_PIN);
}
constexpr bool hardwareFutureUses(int pin) {
    return pin >= 0 && (pin == RESERVED_ULTRASONIC_TRIG_PIN ||
                        pin == RESERVED_ULTRASONIC_ECHO_PIN);
}
constexpr bool hardwareSensorsValid() {
    if (SENSOR_INPUT_MODE != SensorInputMode::DIGITAL_DO &&
        SENSOR_INPUT_MODE != SensorInputMode::ANALOG_AO) return false;
    for (std::size_t i = 0; i < SENSOR_COUNT; ++i) {
        const int pin = SENSOR_PINS[i];
        if (pin < 0 || pin == STATUS_LED_PIN || hardwareMotorUses(pin) ||
            hardwareFutureUses(pin) || (MODE_BUTTON_ENABLED && pin == MODE_BUTTON_PIN)) return false;
        if (SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO ? !hardwareAdc1Pin(pin) :
            !(hardwareOutputPinAllowed(pin) || hardwareAdc1Pin(pin))) return false;
        if (SENSOR_INPUT_MODE == SensorInputMode::ANALOG_AO &&
            (SENSOR_ANALOG_THRESHOLDS[i] <= 0 || SENSOR_ANALOG_THRESHOLDS[i] >= SENSOR_ADC_MAX)) return false;
        for (std::size_t j = 0; j < i; ++j) if (pin == SENSOR_PINS[j]) return false;
    }
    return true;
}
constexpr bool hardwareButtonValid() {
    if (!hardwareOutputPinAllowed(MODE_BUTTON_PIN) || MODE_BUTTON_PIN == STATUS_LED_PIN ||
        hardwareMotorUses(MODE_BUTTON_PIN) || hardwareFutureUses(MODE_BUTTON_PIN)) return false;
    if (SENSOR_HARDWARE_ENABLED) {
        for (int pin : SENSOR_PINS) if (pin == MODE_BUTTON_PIN) return false;
    }
    return true;
}
constexpr bool hardwareReservationsValid() {
    const int pins[] = {RESERVED_ULTRASONIC_TRIG_PIN, RESERVED_ULTRASONIC_ECHO_PIN};
    if (pins[0] >= 0 && pins[0] == pins[1]) return false;
    for (int pin : pins) {
        if (pin < 0) continue;
        if ((!hardwareOutputPinAllowed(pin) && !hardwareAdc1Pin(pin)) ||
            pin == STATUS_LED_PIN || hardwareMotorUses(pin)) return false;
    }
    return RESERVED_ULTRASONIC_TRIG_PIN < 0 || hardwareOutputPinAllowed(RESERVED_ULTRASONIC_TRIG_PIN);
}
static_assert(!SENSOR_HARDWARE_ENABLED || hardwareSensorsValid(),
              "Sensor mode/pins/thresholds invalid or conflicting");
static_assert(!MODE_BUTTON_ENABLED || hardwareButtonValid(), "Mode button GPIO invalid/conflicting");
static_assert(hardwareReservationsValid(), "Future GPIO reservations invalid/conflicting");
#if !defined(CONFIG_IDF_TARGET_ESP32)
static_assert(!SENSOR_HARDWARE_ENABLED && !MODE_BUTTON_ENABLED,
              "Re-audit GPIO/ADC policy for this ESP32 variant");
#endif
