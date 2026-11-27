#!/usr/bin/env python3
"""Bàn thử xe HUSTROY: trang web trên laptop điều khiển và theo dõi xe qua USB Serial.

Chạy:  python dashboard/server.py        rồi mở http://127.0.0.1:8770 (tự mở trình duyệt)
Firmware không cần sửa: trang chỉ gửi các lệnh Serial có sẵn (MODE, ARM, SET, BOTH, STOP, !,
TRACE=1, PULSE=, FF=, CAL=, SAVE, AUTO) và đọc các dòng LINE / LINE_STATS mà firmware in ra.
"""
from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import queue
import re
import sys
import threading
import time
import webbrowser
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "scripts"))
import line_trial  # noqa: E402  (calibration maths shared with the CLI script)

STATIC = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
}
RUNS = HERE / "runs"
RUNS_CSV = RUNS / "runs.csv"
FIELDS = re.compile(r"([A-Z][A-Z_0-9]*)=(\S+)")
RUN_COLUMNS = ["time", "file", "fw", "spd", "kp", "kd", "trim", "ff", "reason", "run_s", "detected_pct",
               "search_pct", "edge_pct", "line_losses", "max_loss_ms", "mae", "reversals", "late", "missed"]


def numbers(text, kind=int):
    try:
        return [kind(x) for x in text.split(",")]
    except (AttributeError, ValueError):
        return []


def parse_frame(line):
    """One 50 Hz trace line: LINE T_US=.. RAW=a,b,c,d,e NORM=.. POS=[NA:]x CONF=.. L=.. R=.. STATE=.."""
    d = dict(FIELDS.findall(line))
    raw = numbers(d.get("RAW"))
    if len(raw) != 5:
        return None
    position = d.get("POS", "NA:0")
    try:
        return {
            "type": "frame", "raw": raw, "norm": numbers(d.get("NORM"), float), "bin": d.get("BIN", ""),
            "pos": float(position.split(":")[-1]), "posValid": not position.startswith("NA:"),
            "conf": float(d.get("CONF", 0)), "signal": d.get("SIGNAL") == "1", "edge": d.get("EDGE") == "1",
            "p": float(d.get("P", 0)), "d": float(d.get("D", 0)), "base": float(d.get("BASE", 0)),
            "l": int(d.get("L", 0)), "r": int(d.get("R", 0)), "state": d.get("STATE", ""),
        }
    except ValueError:
        return None


class Robot:
    """Owns the serial port; fans every parsed line out to the browser event streams."""

    def __init__(self):
        self.port = None
        self.write_lock = threading.Lock()
        self.listeners: set[queue.Queue] = set()
        self.listeners_lock = threading.Lock()
        self.frames: deque = deque(maxlen=600)  # (monotonic time, raw tuple), ~12 s at 50 Hz
        self.state = {"connected": False, "port": None, "mode": None, "armed": None, "auto": None,
                      "fw": None, "lineCal": None, "cal": None, "params": None}
        self.run_lines = None
        self.run_started = None
        self.calibration = {"white": None, "eyes": [None] * 5}

    # ---- event fan-out -------------------------------------------------------------------
    def subscribe(self):
        q = queue.Queue(maxsize=600)
        with self.listeners_lock:
            self.listeners.add(q)
        q.put({"type": "status", "state": dict(self.state)})
        return q

    def unsubscribe(self, q):
        with self.listeners_lock:
            self.listeners.discard(q)

    def publish(self, event):
        with self.listeners_lock:
            listeners = list(self.listeners)
        for q in listeners:
            try:
                q.put_nowait(event)
            except queue.Full:  # A slow tab drops its oldest events, never blocks the reader.
                with contextlib.suppress(queue.Empty):
                    q.get_nowait()
                with contextlib.suppress(queue.Full):
                    q.put_nowait(event)

    def set_state(self, **changes):
        changed = {k: v for k, v in changes.items() if self.state.get(k) != v}
        if changed:
            self.state.update(changed)
            self.publish({"type": "status", "state": dict(self.state)})

    # ---- serial port ---------------------------------------------------------------------
    def connect(self, device):
        import serial  # pyserial: available in the PlatformIO Python and in the team's Python.

        self.disconnect()
        port = serial.Serial(baudrate=115200, timeout=0.2, write_timeout=0.5)
        port.port, port.dtr, port.rts = device, False, False  # Opening must not reset the ESP32.
        port.open()
        self.port = port
        threading.Thread(target=self.read_loop, args=(port,), daemon=True).start()
        self.set_state(connected=True, port=device, mode=None, armed=None, auto=None)

    def disconnect(self):
        port, self.port = self.port, None
        if port:
            with contextlib.suppress(Exception):
                port.close()
        self.set_state(connected=False)

    def send(self, lines, gap=0.08):
        port = self.port
        if not port:
            raise ValueError("Chưa kết nối xe: chọn cổng COM rồi bấm Kết nối.")
        for line in lines:
            if len(line) > 100 or any(not 32 <= ord(c) <= 126 for c in line):
                raise ValueError(f"Lệnh không hợp lệ: {line!r}")
        with self.write_lock:
            for index, line in enumerate(lines):
                port.write(line.encode("ascii") + b"\n")
                if gap and index + 1 < len(lines):
                    time.sleep(gap)
        for line in lines:
            self.publish({"type": "sent", "text": line})

    def emergency_stop(self):
        # '!' stops and DISARMs as soon as the byte arrives; '\n' clears the discarded line.
        self.send(["!"], gap=0)

    def read_loop(self, port):
        buffer = b""
        while self.port is port:
            try:
                chunk = port.read(port.in_waiting or 1)
            except Exception as error:  # Cable pulled, port closed by another program...
                if self.port is port:
                    self.publish({"type": "line", "text": f"Mất kết nối Serial: {error}"})
                    self.disconnect()
                return
            if not chunk:
                continue
            buffer += chunk
            *lines, buffer = buffer.split(b"\n")
            for raw in lines:
                self.handle(raw.decode("utf-8", errors="replace").rstrip("\r"))

    # ---- parsing -------------------------------------------------------------------------
    def handle(self, line):
        if not line:
            return
        if self.run_lines is not None:
            self.run_lines.append(line)
        if line.startswith("LINE ") and "RAW=" in line:
            frame = parse_frame(line)
            if frame:
                self.frames.append((time.monotonic(), tuple(frame["raw"])))
                self.publish(frame)
            return
        if line.startswith("RAW:["):  # SENSOR_TEST without TRACE: 5 raw values only.
            raw = numbers(line[5:line.find("]")])
            if len(raw) == 5:
                self.frames.append((time.monotonic(), tuple(raw)))
                self.publish({"type": "frame", "raw": raw, "norm": [], "posValid": False, "signal": False,
                              "pos": 0, "conf": 0, "l": 0, "r": 0, "state": "SENSOR_TEST"})
            return
        if line.startswith("LINE_CFG"):
            self.run_lines, self.run_started = [line], datetime.now()
        elif line.startswith("LINE_TIMING") and self.run_lines and any(
                l.startswith("LINE_STATS") for l in self.run_lines):
            self.finish_run()
        self.track_state(line)
        self.publish({"type": "line", "text": line})

    def track_state(self, line):
        fields = dict(FIELDS.findall(line))
        changes = {}
        mode = re.search(r"\bMODE=([A-Z_]+)", line)
        if mode:
            changes["mode"] = mode.group(1)
        if "ARMED=" in line:
            changes["armed"] = fields.get("ARMED") == "1"
        elif line.startswith("ARMED:"):
            changes["armed"] = True
        elif "DISARM" in line or line.startswith("BLOCKED"):
            changes["armed"] = False
        if "AUTO=" in line:
            changes["auto"] = "on" if fields.get("AUTO") == "1" else "off"
        if line.startswith("AUTO_LINE: on"):
            changes["auto"] = "countdown"
        elif line.startswith(("AUTO_LINE: armed", "AUTO_LINE: waiting")):
            changes["auto"] = "on"
        elif line.startswith("AUTO_LINE: off"):
            changes["auto"] = "off"
        if line.startswith("FW=") or line.startswith("LINE_STATS"):
            changes["fw"] = fields.get("FW")
        if "LINE_CAL=" in line:
            changes["lineCal"] = fields.get("LINE_CAL") == "1"
        if line.startswith("CAL=") and "WHITE=" in line:
            changes["cal"] = {k.lower(): numbers(fields.get(k)) for k in ("WHITE", "BLACK", "FLOOR")}
            changes["lineCal"] = fields.get("CAL") == "1"
        if line.startswith("KP=") and "SPD=" in line:
            changes["params"] = {k: fields[k] for k in ("KP", "KD", "SPD", "MAX", "TRIM", "LIMIT", "PULSE", "FF")
                                 if k in fields}
        if changes:
            self.set_state(**changes)

    def finish_run(self):
        lines, started = self.run_lines, self.run_started
        self.run_lines = None
        first = lambda prefix: dict(FIELDS.findall(next((l for l in lines if l.startswith(prefix)), "")))
        cfg, stats, timing = first("LINE_CFG"), first("LINE_STATS"), first("LINE_TIMING")
        RUNS.mkdir(exist_ok=True)
        name = f"run-{started:%Y%m%d-%H%M%S}.txt"
        (RUNS / name).write_text("\n".join(lines) + "\n", encoding="utf-8")
        known = max(1, int(stats.get("KNOWN_US", 0) or 0))
        pct = lambda key: round(100 * int(stats.get(key, 0) or 0) / known, 1)
        gain = lambda key: f"{float(cfg[key]):g}" if key in cfg else ""
        row = {
            "time": f"{started:%Y-%m-%d %H:%M:%S}", "file": name, "fw": stats.get("FW", cfg.get("FW", "")),
            "spd": cfg.get("SPD", ""), "kp": gain("KP"), "kd": gain("KD"), "trim": cfg.get("TRIM", ""),
            "ff": cfg.get("FF", ""), "reason": stats.get("REASON", ""),
            "run_s": round(int(stats.get("RUN_US", 0) or 0) / 1e6, 1),
            "detected_pct": pct("DETECTED_US"), "search_pct": pct("SEARCH_US"), "edge_pct": pct("EDGE_US"),
            "line_losses": stats.get("ZERO_EP", ""), "max_loss_ms": round(int(stats.get("MAX_ZERO_US", 0) or 0) / 1000),
            "mae": stats.get("MAE", ""), "reversals": stats.get("REV", ""),
            "late": timing.get("LATE", ""), "missed": timing.get("MISSED", ""),
        }
        new_file = not RUNS_CSV.exists()
        with RUNS_CSV.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=RUN_COLUMNS)
            if new_file:
                writer.writeheader()
            writer.writerow(row)
        self.publish({"type": "run", "run": row})

    # ---- calibration (same maths and checks as scripts/line_trial.py calibrate) ----------
    def record_calibration(self, step, seconds=2.0):
        if self.state.get("mode") != "SENSOR_TEST":
            raise ValueError("Bấm \"Xem cảm biến\" trước: xe phải ở chế độ đọc cảm biến.")
        time.sleep(0.3)  # Let the robot settle after the operator lets go.
        start = time.monotonic()
        time.sleep(seconds)
        frames = [raw for stamp, raw in list(self.frames) if stamp >= start]
        if len(frames) < 10:
            raise ValueError(f"Chỉ nhận được {len(frames)} mẫu: kiểm tra TRACE=1 và kết nối.")
        if step == "white":
            self.calibration["white"] = frames
        else:
            self.calibration["eyes"][int(step)] = frames
        medians = [sorted(c)[len(c) // 2] for c in zip(*frames)]
        return {"step": step, "count": len(frames), "median": medians}

    def compute_calibration(self):
        white, eyes = self.calibration["white"], self.calibration["eyes"]
        if not white or not all(eyes):
            raise ValueError("Ghi đủ 6 bước (giấy trắng và S1…S5) trước khi tính.")
        cal = line_trial.compute_step_calibration(white, eyes)
        report = io.StringIO()
        with contextlib.redirect_stdout(report):
            problems = line_trial.print_step_calibration(cal, white, eyes)
        white_row, black, floor = cal
        return {"cal": {"white": white_row, "black": black, "floor": floor},
                "command": line_trial.cal_command(cal), "problems": problems, "report": report.getvalue()}


def list_ports():
    from serial.tools import list_ports as tools

    ports = []
    for info in tools.comports():
        text = f"{info.description or ''} {info.hwid or ''}"
        ports.append({"device": info.device, "description": info.description or info.device,
                      "likely": any(k in text for k in ("CH340", "CP210", "USB-SERIAL", "VID:PID=1A86", "VID:PID=10C4"))})
    return sorted(ports, key=lambda p: (not p["likely"], p["device"]))


def read_runs():
    if not RUNS_CSV.exists():
        return []
    with RUNS_CSV.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def make_handler(robot: Robot):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):  # Keep the terminal readable.
            pass

        def respond(self, status, content_type, body: bytes, extra=None):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for key, value in (extra or {}).items():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def json(self, data, status=200):
            self.respond(status, "application/json; charset=utf-8", json.dumps(data, ensure_ascii=False).encode())

        def body(self):
            length = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(length) or b"{}") if length else {}

        def do_GET(self):
            path = urlsplit(self.path).path
            try:
                if path in STATIC:
                    name, kind = STATIC[path]
                    self.respond(200, kind, (HERE / "static" / name).read_bytes())
                elif path == "/api/ports":
                    self.json(list_ports())
                elif path == "/api/state":
                    self.json(robot.state)
                elif path == "/api/runs":
                    self.json(read_runs())
                elif path == "/api/runs.csv":
                    data = RUNS_CSV.read_bytes() if RUNS_CSV.exists() else b""
                    self.respond(200, "text/csv; charset=utf-8", data,
                                 {"Content-Disposition": "attachment; filename=hustroy-runs.csv"})
                elif path.startswith("/runs/"):
                    name = Path(unquote(path[len("/runs/"):])).name  # No directory traversal.
                    file = RUNS / name
                    if not name.startswith("run-") or not file.is_file():
                        self.send_error(404)
                        return
                    self.respond(200, "text/plain; charset=utf-8", file.read_bytes(),
                                 {"Content-Disposition": f"attachment; filename={name}"})
                elif path == "/api/events":
                    self.stream()
                else:
                    self.send_error(404)
            except (ConnectionError, BrokenPipeError):
                pass

        def stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            q = robot.subscribe()
            try:
                while True:
                    try:
                        event = q.get(timeout=10)
                        self.wfile.write(f"data: {json.dumps(event, ensure_ascii=False)}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
            except (ConnectionError, BrokenPipeError, OSError):
                pass
            finally:
                robot.unsubscribe(q)

        def do_POST(self):
            path = urlsplit(self.path).path
            try:
                data = self.body()
                if path == "/api/connect":
                    robot.connect(str(data.get("port", "")))
                    self.json(robot.state)
                elif path == "/api/disconnect":
                    robot.disconnect()
                    self.json(robot.state)
                elif path == "/api/send":
                    lines = data.get("lines")
                    if not isinstance(lines, list) or not lines:
                        raise ValueError("Thiếu danh sách lệnh.")
                    robot.send([str(line) for line in lines])
                    self.json({"ok": True})
                elif path == "/api/stop":
                    robot.emergency_stop()
                    self.json({"ok": True})
                elif path == "/api/calibration/record":
                    step = data.get("step")
                    if step != "white" and step not in (0, 1, 2, 3, 4):
                        raise ValueError("Bước hiệu chuẩn không hợp lệ.")
                    self.json(robot.record_calibration(step))
                elif path == "/api/calibration/compute":
                    self.json(robot.compute_calibration())
                elif path == "/api/calibration/reset":
                    robot.calibration = {"white": None, "eyes": [None] * 5}
                    self.json({"ok": True})
                else:
                    self.send_error(404)
            except (ValueError, KeyError, json.JSONDecodeError) as error:
                self.json({"error": str(error)}, 400)
            except Exception as error:  # Serial errors (port busy, cable pulled) reach the page as text.
                self.json({"error": f"{type(error).__name__}: {error}"}, 500)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--http-port", type=int, default=8770, help="Cổng web trên laptop (mặc định 8770)")
    parser.add_argument("--serial", help="Tự kết nối cổng này khi khởi động, ví dụ COM11")
    parser.add_argument("--no-browser", action="store_true", help="Không tự mở trình duyệt")
    options = parser.parse_args()
    robot = Robot()
    if options.serial:
        robot.connect(options.serial)
    server = ThreadingHTTPServer(("127.0.0.1", options.http_port), make_handler(robot))
    server.daemon_threads = True
    url = f"http://127.0.0.1:{server.server_port}"
    print(f"Bàn thử xe HUSTROY: {url}  (Ctrl+C để tắt)", flush=True)
    if not options.no_browser:
        threading.Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if robot.port:
            with contextlib.suppress(Exception):
                robot.emergency_stop()  # Leaving the bench never leaves the wheels driven.
        robot.disconnect()


if __name__ == "__main__":
    main()
