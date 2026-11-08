#!/usr/bin/env python3
"""Compile REAL firmware sources against host doubles. No hardware is accessed.

GPIO fixtures below exist ONLY in temporary host-test config copies. They are
NOT a physical/simulation wiring recommendation. Tests never access hardware.
Run: python3 tests/run_host_tests.py (requires a C++ compiler, e.g. macOS clang++).
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CONFIG = (ROOT / "include/config.h").read_text()
FIXTURE = dict(zip(
    ["MOTOR_PWMA_PIN", "MOTOR_AIN1_PIN", "MOTOR_AIN2_PIN", "MOTOR_PWMB_PIN",
     "MOTOR_BIN1_PIN", "MOTOR_BIN2_PIN", "MOTOR_STBY_PIN"],
    [18, 19, 21, 22, 23, 25, 26],
))


def configure(changes):
    text = CONFIG
    for key, value in changes.items():
        text, count = re.subn(rf"({key} = )[^;]+;", rf"\g<1>{value};", text)
        assert count == 1, key
    return text


def build(folder, changes, expect_failure=False,
          error_text="Motor GPIOs unassigned, duplicated, reserved or unsafe"):
    # Mirror headers so quoted sibling config.h includes use this fixture too.
    # Otherwise hardware_config.h would silently read the locked production config.
    for header in (ROOT / "include").glob("*.h"):
        if header.name != "config.h":
            (folder / header.name).write_bytes(header.read_bytes())
    (folder / "config.h").write_text(configure(changes))
    command = ["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
               "-DCONFIG_IDF_TARGET_ESP32", "-I", str(folder),
               "-I", str(ROOT / "tests/host"), "-I", str(ROOT / "include"),
               str(ROOT / "src/motor.cpp"), str(ROOT / "src/main.cpp"),
               str(ROOT / "src/sensors.cpp"), str(ROOT / "src/pid.cpp"), str(ROOT / "src/modes.cpp"),
               str(ROOT / "tests/host/motor_test.cpp"), "-o", str(folder / "test")]
    result = subprocess.run(command, text=True, capture_output=True)
    if expect_failure:
        assert result.returncode != 0, "Unsafe configuration unexpectedly compiled"
        assert error_text in result.stderr, result.stderr
    else:
        assert result.returncode == 0, result.stderr


def main():
    with tempfile.TemporaryDirectory(prefix="hustroy-motor-tests-") as temporary:
        folder = Path(temporary)
        # Verify the actual enabled bench mapping too; no physical GPIO access.
        if "MOTOR_HARDWARE_ENABLED = true;" in CONFIG:
            build(folder, {})
            for scenario in ["driver", "arming"]:
                subprocess.run([str(folder / "test"), scenario], check=True)
            print("PASS: current bench config driver and arming", flush=True)
        # Exercise USB-only diagnostics even when the production bench config is enabled.
        build(folder, {"MOTOR_HARDWARE_ENABLED": "false"})
        subprocess.run([str(folder / "test"), "disabled"], check=True)
        subprocess.run([str(folder / "test"), "logic"], check=True)
        enabled = {**FIXTURE, "MOTOR_HARDWARE_ENABLED": "true"}
        build(folder, enabled)
        for scenario in ["driver", "serial", "arming", "attach-a", "attach-b", "init-write",
                         "init-stop", "runtime-fault", "stop-fault"]:
            subprocess.run([str(folder / "test"), scenario], check=True)
        for changes in [{"INVERT_LEFT_MOTOR": "true"}, {"INVERT_RIGHT_MOTOR": "true"},
                        {"LEFT_MOTOR_IS_CHANNEL_A": "false"},
                        {"LEFT_MOTOR_IS_CHANNEL_A": "false", "INVERT_LEFT_MOTOR": "true",
                         "INVERT_RIGHT_MOTOR": "true"}]:
            build(folder, {**enabled, **changes})
            subprocess.run([str(folder / "test"), "driver"], check=True)
        for name, changes in [
            ("unassigned", {**enabled, "MOTOR_AIN1_PIN": -1}),
            ("duplicate", {**enabled, "MOTOR_BIN2_PIN": 18}),
            ("input-only", {**enabled, "MOTOR_BIN2_PIN": 34}),
            ("flash", {**enabled, "MOTOR_BIN2_PIN": 6}),
            ("boot-strap", {**enabled, "MOTOR_BIN2_PIN": 12}),
            ("status-led", {**enabled, "MOTOR_BIN2_PIN": 2}),
            ("uart", {**enabled, "MOTOR_BIN2_PIN": 1}),
        ]:
            build(folder, changes, expect_failure=True)
            print(f"PASS: compile rejects {name}", flush=True)

        sensor_config = {**enabled, "SENSOR_HARDWARE_ENABLED": "true",
                         "SENSOR_INPUT_MODE": "SensorInputMode::ANALOG_AO",
                         "SENSOR_PINS": "{{32,33,34,35,36}}",
                         "SENSOR_CALIBRATION_CONFIRMED": "true",
                         "SENSOR_DISCONNECT_BIAS_CONFIRMED": "true"}
        for name, changes in [
            ("analog", {}),
            ("digital", {"SENSOR_INPUT_MODE": "SensorInputMode::DIGITAL_DO",
                         "SENSOR_ANALOG_THRESHOLDS": "{{0,0,0,0,0}}"}),
            ("polarity", {"INVERT_LINE_SENSOR": "true"}),
            ("pid-sign", {"INVERT_PID_ERROR": "true"}),
        ]:
            build(folder, {**sensor_config, **changes})
            subprocess.run([str(folder / "test"), "system-follow"], check=True)
            print(f"PASS: integrated {name}", flush=True)
        for name, changes in [
            ("calibration", {"SENSOR_CALIBRATION_CONFIRMED": "false"}),
            ("disconnect-bias", {"SENSOR_DISCONNECT_BIAS_CONFIRMED": "false"}),
            ("disabled-sensors", {"SENSOR_HARDWARE_ENABLED": "false"}),
        ]:
            build(folder, {**sensor_config, **changes})
            subprocess.run([str(folder / "test"), "system-gates"], check=True)
            print(f"PASS: RUN blocked by {name}", flush=True)
        build(folder, {**sensor_config, "MODE_BUTTON_ENABLED": "true", "MODE_BUTTON_PIN": 27})
        for scenario in ["system-button", "system-held-button"]:
            subprocess.run([str(folder / "test"), scenario], check=True)
        sensor_error = "Sensor mode/pins/thresholds invalid or conflicting"
        for name, changes, message in [
            ("sensor-mode", {"SENSOR_INPUT_MODE": "SensorInputMode::UNCONFIGURED"}, sensor_error),
            ("sensor-duplicate", {"SENSOR_PINS": "{{32,32,34,35,36}}"}, sensor_error),
            ("sensor-non-adc1", {"SENSOR_PINS": "{{13,33,34,35,36}}"}, sensor_error),
            ("sensor-motor-conflict", {"SENSOR_INPUT_MODE": "SensorInputMode::DIGITAL_DO",
                                       "SENSOR_PINS": "{{18,33,34,35,36}}"}, sensor_error),
            ("sensor-threshold", {"SENSOR_ANALOG_THRESHOLDS": "{{0,2048,2048,2048,2048}}"}, sensor_error),
            ("button-input-only", {"MODE_BUTTON_ENABLED": "true", "MODE_BUTTON_PIN": 34}, "Mode button GPIO invalid/conflicting"),
            ("reserved-motor-conflict", {"RESERVED_ULTRASONIC_TRIG_PIN": 18}, "Future GPIO reservations invalid/conflicting"),
        ]:
            build(folder, {**sensor_config, **changes}, expect_failure=True, error_text=message)
            print(f"PASS: compile rejects {name}", flush=True)
    print("All host checks passed. Not an ESP32, Wokwi or physical motor test.")


if __name__ == "__main__":
    main()
