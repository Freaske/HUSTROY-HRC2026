#!/usr/bin/env python3
"""Compile REAL firmware sources against host doubles. No hardware is accessed.

GPIO fixtures below exist ONLY in temporary host-test config copies. They are
NOT a physical/simulation wiring recommendation. Production config stays locked.
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


def build(folder, changes, expect_failure=False):
    (folder / "config.h").write_text(configure(changes))
    command = ["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
               "-DCONFIG_IDF_TARGET_ESP32", "-I", str(folder),
               "-I", str(ROOT / "tests/host"), "-I", str(ROOT / "include"),
               str(ROOT / "src/motor.cpp"), str(ROOT / "src/main.cpp"),
               str(ROOT / "tests/host/motor_test.cpp"), "-o", str(folder / "test")]
    result = subprocess.run(command, text=True, capture_output=True)
    if expect_failure:
        assert result.returncode != 0, "Unsafe configuration unexpectedly compiled"
        assert "Motor GPIOs unassigned, duplicated, reserved or unsafe" in result.stderr
    else:
        assert result.returncode == 0, result.stderr


with tempfile.TemporaryDirectory(prefix="hustroy-motor-tests-") as temporary:
    folder = Path(temporary)
    build(folder, {})
    subprocess.run([str(folder / "test"), "disabled"], check=True)
    enabled = {**FIXTURE, "MOTOR_HARDWARE_ENABLED": "true"}
    build(folder, enabled)
    for scenario in ["driver", "serial", "attach-a", "attach-b", "init-write",
                     "init-stop", "runtime-fault", "stop-fault"]:
        subprocess.run([str(folder / "test"), scenario], check=True)
    for changes in [{"INVERT_LEFT_MOTOR": "true"}, {"INVERT_RIGHT_MOTOR": "true"},
                    {"LEFT_MOTOR_IS_CHANNEL_A": "false"},
                    {"LEFT_MOTOR_IS_CHANNEL_A": "false", "INVERT_LEFT_MOTOR": "true",
                     "INVERT_RIGHT_MOTOR": "true"}]:
        build(folder, {**enabled, **changes})
        subprocess.run([str(folder / "test"), "driver"], check=True)
    for name, changes in [
        ("unassigned", {"MOTOR_HARDWARE_ENABLED": "true"}),
        ("duplicate", {**enabled, "MOTOR_BIN2_PIN": 18}),
        ("input-only", {**enabled, "MOTOR_BIN2_PIN": 34}),
        ("flash", {**enabled, "MOTOR_BIN2_PIN": 6}),
        ("boot-strap", {**enabled, "MOTOR_BIN2_PIN": 12}),
        ("status-led", {**enabled, "MOTOR_BIN2_PIN": 2}),
        ("uart", {**enabled, "MOTOR_BIN2_PIN": 1}),
    ]:
        build(folder, changes, expect_failure=True)
        print(f"PASS: compile rejects {name}", flush=True)
print("All host checks passed. Not an ESP32, Wokwi or physical motor test.")
