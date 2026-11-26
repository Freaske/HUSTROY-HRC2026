#!/usr/bin/env python3
"""Build the host simulator and serve its dashboard on localhost."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile
from urllib.parse import parse_qs, urlsplit


DIRECTORY = Path(__file__).resolve().parent
ROOT = DIRECTORY.parent
TRACKS = ("straight", "curve", "corner", "cross", "gap", "finish")
SIM_CONFIG = {
    "MOTOR_HARDWARE_ENABLED": "true",
    "MOTOR_PWMA_PIN": "18",
    "MOTOR_AIN1_PIN": "19",
    "MOTOR_AIN2_PIN": "21",
    "MOTOR_PWMB_PIN": "22",
    "MOTOR_BIN1_PIN": "23",
    "MOTOR_BIN2_PIN": "25",
    "MOTOR_STBY_PIN": "26",
    "SENSOR_HARDWARE_ENABLED": "true",
    "SENSOR_INPUT_MODE": "SensorInputMode::DIGITAL_DO",
    "SENSOR_PINS": "{{32,33,34,35,36}}",
    "SENSOR_CALIBRATION_CONFIRMED": "true",
    "SENSOR_DISCONNECT_BIAS_CONFIRMED": "true",
    "LINE_FOLLOW_STEER_TRIM": "0",
    "MODE_BUTTON_ENABLED": "false",
    "RESERVED_ULTRASONIC_TRIG_PIN": "-1",
    "RESERVED_ULTRASONIC_ECHO_PIN": "-1",
}
ASSETS = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
}


def build(folder):
    headers = ROOT / "include"
    for header in headers.glob("*.h"):
        (folder / header.name).write_bytes(header.read_bytes())
    config = (headers / "config.h").read_text()
    for name, value in SIM_CONFIG.items():
        config, count = re.subn(rf"(\b{name}\s*=\s*)[^;]+;", rf"\g<1>{value};", config)
        if count != 1:
            raise ValueError(f"Không tìm thấy đúng một khai báo {name} trong config.h")
    (folder / "config.h").write_text(config)
    executable = folder / "robot"
    sources = ("main", "modes", "motor", "sensors", "pid", "line_follow", "tuning", "test_link")
    subprocess.run([
        "c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-DCONFIG_IDF_TARGET_ESP32",
        "-I", str(folder), "-I", str(ROOT / "tests/host"), str(DIRECTORY / "robot.cpp"),
        *(str(ROOT / "src" / f"{source}.cpp") for source in sources), "-o", str(executable),
    ], check=True)
    return executable


def invoke(executable, *arguments):
    return subprocess.check_output([str(executable), *(str(value) for value in arguments)], timeout=5)


def run_arguments(query, config):
    values = parse_qs(query, max_num_fields=4)
    if values.keys() != {"track", "kp", "kd", "speed"} or any(len(v) != 1 for v in values.values()):
        raise ValueError("Cần chọn bài chạy và nhập KP, KD, PWM nền.")
    track, kp, kd, speed = (values[key][0] for key in ("track", "kp", "kd", "speed"))
    if track not in TRACKS:
        raise ValueError("Bài chạy không hợp lệ.")
    try:
        kp, kd, speed = float(kp), float(kd), int(speed)
    except ValueError:
        raise ValueError("KP, KD phải là số và PWM nền phải là số nguyên.") from None
    if not all(math.isfinite(gain) and 0 <= gain <= 1000 for gain in (kp, kd)):
        raise ValueError("KP và KD phải nằm trong khoảng 0–1000.")
    if any(0 < gain < float.fromhex("0x1p-126") for gain in (kp, kd)):
        raise ValueError("KP hoặc KD quá nhỏ cho firmware. Nhập 0 hoặc tăng giá trị.")
    if not 1 <= speed <= config["maxSpeed"]:
        raise ValueError(f"PWM nền phải nằm trong khoảng 1–{config['maxSpeed']}.")
    return track, kp, kd, speed


def serve(executable, config, port):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, status, content_type, content):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

        def respond_json(self, status, data):
            self.respond(status, "application/json; charset=utf-8", data)

        def do_GET(self):
            request = urlsplit(self.path)
            if request.path == "/api/config":
                self.respond_json(200, json.dumps(config).encode())
            elif request.path == "/api/run":
                try:
                    arguments = run_arguments(request.query, config)
                except ValueError as error:
                    self.respond_json(400, json.dumps({"error": str(error)}).encode())
                    return
                try:
                    result = invoke(executable, *arguments)
                except (subprocess.SubprocessError, OSError) as error:
                    self.log_error("%s", error)
                    self.respond_json(500, json.dumps({"error": "Không chạy được firmware mô phỏng."}).encode())
                    return
                self.respond_json(200, result)
            elif request.path in ASSETS:
                filename, content_type = ASSETS[request.path]
                self.respond(200, content_type, (DIRECTORY / filename).read_bytes())
            else:
                self.send_error(404)

    with ThreadingHTTPServer(("127.0.0.1", port), Handler) as server:
        print(f"Mở sa bàn tại http://127.0.0.1:{server.server_port}", flush=True)
        server.serve_forever()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765, help="Cổng localhost (mặc định 8765)")
    parser.add_argument("--demo", action="store_true", help="Chạy các bài mẫu và in kết quả trong terminal")
    options = parser.parse_args()
    try:
        with tempfile.TemporaryDirectory(prefix="hustroy-sim-") as directory:
            executable = build(Path(directory))
            config = json.loads(invoke(executable))
            if options.demo:
                for track in TRACKS:
                    run = json.loads(invoke(executable, track, config["kp"], config["kd"], config["speed"]))
                    last = run["frames"][-1]
                    result = "Qua mốc" if run["checkpointReached"] else last["stop"]
                    print(f"{track:8} {last['t'] / 1000:5.2f} s  {last['distance']:7.1f} mm  {result}")
            else:
                serve(executable, config, options.port)
    except KeyboardInterrupt:
        pass
    except FileNotFoundError as error:
        if error.filename == "c++":
            parser.exit(1, "Cần trình biên dịch C++17 (c++). Trên macOS có thể cài với xcode-select --install.\n")
        raise


if __name__ == "__main__":
    main()
