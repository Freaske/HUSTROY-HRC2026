#!/usr/bin/env python3
"""Capture USB telemetry or compare evidence from HUSTROY LINE_FOLLOW V5."""

import argparse
from collections import Counter
import csv
import gzip
import json
from pathlib import Path
import re
import statistics
import time

FIELDS = re.compile(r"([A-Z][A-Z_0-9]*)=(\S+)")


def read_records(path, prefix):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", errors="replace") as stream:
        return [dict(FIELDS.findall(line)) for line in stream if line.startswith(prefix + " ")]


def capture(args):
    import serial  # Available in the PlatformIO Python environment.

    port = serial.Serial(baudrate=115200, timeout=0.1, write_timeout=0.3, exclusive=True)
    port.port, port.dtr, port.rts = args.port, False, False
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as stream, port:
        def record_for(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if data := port.read(port.in_waiting or 1):
                    stream.write(data)

        def send(command):
            port.write((command + "\n").encode("ascii"))

        # Each capture starts disarmed. Timed motor trials require an explicit command.
        send("!")
        record_for(0.45)
        for command in ("STATUS", "GET", "CAL?", "TRACE=1", *args.setup):
            send(command)
            record_for(0.05)
        try:
            if args.motion:
                record_for(0.25)
                send("ARM")
                record_for(0.15)
                send(args.motion)
            print(f"Capture started ({args.seconds:g} s)", flush=True)
            record_for(args.seconds)
        finally:
            send("!" if args.motion else "DISARM")
            record_for(0.35)
            send("TRACE=0")
            record_for(0.1)
    print(args.output)


def trace(args):
    keys = {"T_US", "RAW", "STATE", "BIN", "SIGNAL", "ADC_US", "L", "R"}
    rows = [row for row in read_records(args.log, "LINE") if keys <= row.keys() and
            len(row["RAW"].split(",")) == 5 and all(x >= 0 for x in values(row, "RAW"))]
    if args.duty is not None:
        rows = [row for row in rows if int(row["L"]) == int(row["R"]) == args.duty]
    if len(rows) < 2:
        raise ValueError(f"Insufficient valid raw samples in {args.log}")
    channels = list(zip(*(values(row, "RAW") for row in rows)))
    times = [int(row["T_US"]) for row in rows]
    periods = [(end - start) % (1 << 32) for start, end in zip(times, times[1:])]
    states = Counter(row["STATE"] for row in rows)
    return {
        "log": str(args.log), "samples": len(rows),
        "median_sample_period_us": statistics.median(periods), "max_sample_period_us": max(periods),
        "max_adc_us": max(int(row["ADC_US"]) for row in rows),
        "requested_equal_duty": args.duty,
        "raw": [dict(min=min(c), median=statistics.median(c), max=max(c),
                     stdev=round(statistics.pstdev(c), 2), zero=c.count(0), rail=c.count(4095)) for c in channels],
        "states": states, "binary": Counter(row["BIN"] for row in rows),
        "missing_samples": sum(row["SIGNAL"] == "0" for row in rows),
    }


def values(row, label):
    return tuple(map(int, row[label].split(",")))


# ---------------------------------------------------------------------------
# calibrate: build a CAL=white/black/floor profile from a white hold + a sweep.
# Mirrors lineCalibrationValid() and observeLine() in src/line_follow.cpp.
ADC_MAX = 4095
COORDINATES = (-2, -1, 0, 1, 2)
MIN_PEAK, MIN_MASS = 0.14, 0.21  # LINE_MIN_PEAK / LINE_MIN_MASS in include/line_follow.h


def raw_frames(lines):
    frames = []
    for line in lines:
        if not line.startswith("LINE "):
            continue
        row = dict(FIELDS.findall(line))
        try:
            raw = values(row, "RAW")
        except (KeyError, ValueError):
            continue
        if len(raw) == 5 and all(0 <= x <= ADC_MAX for x in raw):
            frames.append(raw)
    return frames


def log_frames(path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", errors="replace") as stream:
        return raw_frames(stream)


def percentile(data, q):
    ordered = sorted(data)
    return ordered[min(len(ordered) - 1, max(0, round(q * (len(ordered) - 1))))]


def observe(raw, cal, with_position=False):
    """Python copy of observeLine() for ANALOG_AO: returns Position/Ambiguous/Missing
    (and the line position in sensor pitches when with_position is set)."""
    state, position = observe_line(raw, cal)
    return (state, position) if with_position else state


def observe_line(raw, cal):
    white, black, floor = cal
    weights, strong, position = [], 0, 0.0
    for i in range(5):
        span = max(1, black[i] - white[i])
        background = (floor[i] - white[i]) / span
        strength = min(1.0, max(0.0, (raw[i] - white[i]) / span))
        weight = 0.0 if background >= 1 else min(1.0, max(0.0, (strength - background) / (1 - background)))
        weights.append(weight)
        strong += weight >= 0.55
    clusters, first = 0, 0
    while first < 5:
        if weights[first] <= 0.025:
            first += 1
            continue
        end = first
        while end < 5 and weights[end] > 0.025:
            end += 1
        cluster = weights[first:end]
        if max(cluster) >= MIN_PEAK and sum(cluster) >= MIN_MASS:
            clusters += 1
            position = sum(w * COORDINATES[first + k] for k, w in enumerate(cluster)) / sum(cluster)
        first = end
    if clusters > 1 or strong >= 4 or (weights[0] >= MIN_MASS and weights[4] >= MIN_MASS):
        return "Ambiguous", 0.0
    return ("Position", position) if clusters == 1 else ("Missing", 0.0)


def calibration_problems(cal):
    """Same rules as lineCalibrationValid(cal, inverted=false)."""
    problems = []
    for i, (w, b, f) in enumerate(zip(*cal)):
        span, background = b - w, f - w
        if not all(0 <= x <= ADC_MAX for x in (w, b, f)):
            problems.append(f"S{i + 1}: giá trị ngoài 0..4095")
        elif span < 128:
            problems.append(f"S{i + 1}: đen-trắng chỉ {span} (<128) — vạch chưa đi qua mắt này, "
                            "mắt quá cao hoặc cảm biến hỏng")
        elif not 0 <= background < span // 2:
            problems.append(f"S{i + 1}: nền {background} >= nửa biên độ {span // 2} — nền sáng/nhiễu quá lớn")
    return problems


def compute_calibration(white_frames, sweep_frames):
    if len(white_frames) < 20 or len(sweep_frames) < 50:
        raise ValueError(f"Thiếu mẫu: trắng {len(white_frames)} (cần >=20), quét {len(sweep_frames)} (cần >=50)")
    white = [round(statistics.median(c)) for c in zip(*white_frames)]
    white_max = [max(c) for c in zip(*white_frames)]
    black = [min(ADC_MAX, percentile(c, 0.97)) for c in zip(*sweep_frames)]
    # Background of eye i = its reading while the line is clearly under an eye two or more pitches away.
    # Frames with 3+ dark eyes (robot lifted/tilted, eyes off the paper) or two separate dark
    # spots say nothing about the background and are skipped.
    far = [[] for _ in range(5)]
    for raw in sweep_frames:
        strength = [(raw[i] - white[i]) / max(1, black[i] - white[i]) for i in range(5)]
        strong = [i for i in range(5) if strength[i] >= 0.6]
        if not strong or len(strong) >= 3 or strong[-1] - strong[0] > 1:
            continue
        peak = max(strong, key=strength.__getitem__)
        for i in range(5):
            if abs(i - peak) >= 2:
                far[i].append(raw[i])
    floor = []
    for i in range(5):
        observed = max(white_max[i], percentile(far[i], 0.95) if far[i] else white_max[i])
        margin = max(20, 0.03 * (black[i] - white[i]))
        floor.append(min(ADC_MAX, -(-int(observed + margin) // 10) * 10))  # Round up to 10 counts.
    return white, black, floor


def calibration_report(cal, white_frames, sweep_frames):
    white_states = Counter(observe(raw, cal) for raw in white_frames)
    sweep_states = Counter(observe(raw, cal) for raw in sweep_frames)
    # Time order in which each eye first saw the line strongly: checks left->right wiring order.
    first_seen = {}
    for index, raw in enumerate(sweep_frames):
        for i in range(5):
            if i not in first_seen and raw[i] - cal[0][i] >= 0.8 * (cal[1][i] - cal[0][i]):
                first_seen[i] = index
    order = [f"S{i + 1}" for i in sorted(first_seen, key=first_seen.get)]
    return white_states, sweep_states, order


def dark_frame_share(cal, frames):
    """Share of frames where 3+ eyes read clearly dark: lifted/tilted robot or eyes off the paper."""
    white, black, _ = cal
    dark = sum(sum(raw[i] - white[i] >= 0.6 * max(1, black[i] - white[i]) for i in range(5)) >= 3
               for raw in frames)
    return dark / max(1, len(frames))


def print_calibration(cal, white_frames, sweep_frames):
    white, black, floor = cal
    print("      " + "".join(f"{f'S{i + 1}':>7}" for i in range(5)))
    for name, row in (("WHITE", white), ("BLACK", black), ("FLOOR", floor)):
        print(f"{name:6}" + "".join(f"{x:>7}" for x in row))
    dark = dark_frame_share(cal, sweep_frames)
    print(f"Mẫu quét có >=3 mắt cùng đen: {100 * dark:.0f}%"
          + ("  — xe bị nhấc/nghiêng hoặc mắt ra ngoài giấy; giữ xe áp sát giấy khi quét" if dark > 0.1 else ""))
    problems = calibration_problems(cal)
    if problems:
        return problems  # The observe() report is meaningless for an invalid profile.
    white_states, sweep_states, order = calibration_report(cal, white_frames, sweep_frames)
    total = len(sweep_frames)
    print(f"Nền trắng ({len(white_frames)} mẫu): {dict(white_states)}  — phải toàn 'Missing'")
    print(f"Lượt quét ({total} mẫu): " + ", ".join(
        f"{state} {100 * count / total:.0f}%" for state, count in sweep_states.most_common()))
    print("Thứ tự mắt thấy vạch lần đầu: " + (" → ".join(order) or "không mắt nào"))
    if white_states.get("Missing", 0) != len(white_frames):
        problems.append("Nền trắng bị đọc thành có vạch: nâng FLOOR hoặc kiểm tra độ cao/ánh sáng")
    if len(order) < 5:
        problems.append("Vạch chưa đi qua đủ 5 mắt trong lượt quét: quét lại rộng hơn")
    elif order not in (["S1", "S2", "S3", "S4", "S5"], ["S5", "S4", "S3", "S2", "S1"]):
        problems.append("Thứ tự mắt không phải S1→S5 hay S5→S1: kiểm tra SENSOR_PINS/đi dây "
                        "(hoặc lượt quét bị giật qua lại)")
    return problems


def compute_step_calibration(white_frames, eye_frames):
    """eye_frames[i]: frames recorded while the line sat under eye i only."""
    if len(white_frames) < 20 or any(len(f) < 10 for f in eye_frames):
        raise ValueError("Thiếu mẫu ở một bước: kiểm tra cổng Serial và firmware V5")
    white = [round(statistics.median(c)) for c in zip(*white_frames)]
    white_max = [max(c) for c in zip(*white_frames)]
    black = [round(statistics.median(raw[i] for raw in eye_frames[i])) for i in range(5)]
    floor = []
    for j in range(5):
        # Background of eye j: white paper, plus its reading while the line is two+ eyes away.
        far = [raw[j] for i in range(5) if abs(i - j) >= 2 for raw in eye_frames[i]]
        observed = max(white_max[j], percentile(far, 0.95) if far else white_max[j])
        margin = max(30, 0.22 * (black[j] - white[j]))  # Room for paper/height variation.
        floor.append(min(ADC_MAX, -(-int(observed + margin) // 10) * 10))
    return white, black, floor


def print_step_calibration(cal, white_frames, eye_frames):
    white, black, floor = cal
    print("      " + "".join(f"{f'S{i + 1}':>7}" for i in range(5)))
    for name, row in (("WHITE", white), ("BLACK", black), ("FLOOR", floor)):
        print(f"{name:6}" + "".join(f"{x:>7}" for x in row))
    # At step i the operator put the physical eye i on the line: firmware input i must be the darkest.
    problems = []
    for i, frames in enumerate(eye_frames):
        rise = [statistics.median(raw[j] for raw in frames) - white[j] for j in range(5)]
        darkest = max(range(5), key=rise.__getitem__)
        if darkest != i:
            problems.append(f"Bước đặt S{i + 1} lên vạch: firmware lại thấy S{darkest + 1} tối nhất — "
                            f"dây S{i + 1}/S{darkest + 1} cắm đảo so với SENSOR_PINS, hoặc đặt nhầm mắt")
    problems += calibration_problems(cal)
    if problems:
        return problems
    white_states = Counter(observe(raw, cal) for raw in white_frames)
    print(f"Nền trắng: {dict(white_states)}  — phải toàn 'Missing'")
    if white_states.get("Missing", 0) != len(white_frames):
        problems.append("Nền trắng bị đọc thành có vạch")
    positions = []
    for i, frames in enumerate(eye_frames):
        seen = [observe(raw, cal, with_position=True) for raw in frames]
        states = Counter(state for state, _ in seen)
        found = [p for state, p in seen if state == "Position"]
        position = statistics.median(found) if found else None
        positions.append(position)
        print(f"Vạch dưới S{i + 1}: {dict(states)}  vị trí "
              + (f"{position:+.2f} (mong đợi {COORDINATES[i]:+d})" if found else "không thấy"))
        if len(found) < 0.8 * len(frames):
            problems.append(f"S{i + 1}: firmware không thấy rõ vạch khi vạch nằm dưới mắt này")
    if None not in positions and any(b <= a for a, b in zip(positions, positions[1:])):
        problems.append("Vị trí không tăng dần S1→S5: thứ tự dây cảm biến (SENSOR_PINS) sai, "
                        "hoặc đặt nhầm mắt ở một bước")
    return problems


def cal_command(cal):
    return "CAL=" + "/".join(",".join(map(str, row)) for row in cal)


def calibrate(args):
    if args.port:
        white_frames, eye_frames, port = capture_calibration(args)
        cal = compute_step_calibration(white_frames, eye_frames)
        problems = print_step_calibration(cal, white_frames, eye_frames)
    else:
        if not (args.white and args.sweep):
            raise ValueError("Cần PORT, hoặc --white LOG và --sweep LOG để tính từ log có sẵn")
        white_frames, sweep_frames, port = log_frames(args.white), log_frames(args.sweep), None
        cal = compute_calibration(white_frames, sweep_frames)
        problems = print_calibration(cal, white_frames, sweep_frames)
    command = cal_command(cal)
    print(command)
    if problems:
        print("KHÔNG ĐẠT:\n  - " + "\n  - ".join(problems))
        if port:
            port.close()
        raise SystemExit(1)
    if port:
        with port:
            for line in [command, "CAL?"] + (["SAVE"] if args.save else []):
                print(f"> {line}\n" + "\n".join(serial_exchange(port, line)))
        if not args.save:
            print("Profile đang trong RAM. Chạy lại với --save để giữ sau khi reset.")


def serial_exchange(port, command, seconds=0.4):
    port.reset_input_buffer()
    port.write((command + "\n").encode("ascii"))
    deadline, text = time.monotonic() + seconds, b""
    while time.monotonic() < deadline:
        text += port.read(port.in_waiting or 1)
    return [l for l in text.decode(errors="replace").splitlines() if l and not l.startswith("LINE ")]


def capture_calibration(args):
    import serial  # Available in the PlatformIO Python environment.

    port = serial.Serial(baudrate=115200, timeout=0.1, write_timeout=0.3)
    port.port, port.dtr, port.rts = args.port, False, False  # Opening must not reset the ESP32.
    port.open()
    log = args.log.open("w", encoding="utf-8") if args.log else None

    def record(seconds):
        buffer, deadline = b"", time.monotonic() + seconds
        while time.monotonic() < deadline:
            buffer += port.read(port.in_waiting or 1)
        text = buffer.decode(errors="replace")
        if log:
            log.write(text)
        return raw_frames(text.splitlines())

    try:
        for command in ("!", "", "MODE SENSOR_TEST", "TRACE=1"):  # Stopped + DISARMED before sampling.
            port.write((command + "\n").encode("ascii"))
            record(0.2)
        def step(prompt, seconds):
            input(prompt + " Giữ yên rồi nhấn Enter...")
            port.reset_input_buffer()  # Drop trace lines queued while waiting for Enter.
            record(0.3)
            frames = record(seconds)
            print(f"   đã ghi {len(frames)} mẫu")
            return frames

        print("Đặt xe nằm trên giấy như khi chạy (không nhấc). S1 = mắt ngoài cùng BÊN TRÁI "
              "khi nhìn theo hướng xe chạy.")
        white_frames = step("1) Cả 5 mắt trên GIẤY TRẮNG, không mắt nào chạm vạch.", args.seconds + 1)
        eye_frames = []
        for i in range(5):
            side = {0: " (ngoài cùng trái)", 2: " (giữa)", 4: " (ngoài cùng phải)"}.get(i, "")
            eye_frames.append(step(f"{i + 2}) Đẩy xe ngang để MẮT S{i + 1}{side} đè đúng lên vạch đen, "
                                   "các mắt khác trên giấy trắng.", args.seconds))
        port.write(b"TRACE=0\n")
        record(0.2)
    except BaseException:
        port.close()
        raise
    finally:
        if log:
            log.close()
    if not white_frames or not all(eye_frames):
        port.close()
        raise ValueError("Không nhận được dòng 'LINE RAW=' nào: kiểm tra cổng, baud 115200 và firmware V5")
    return white_frames, eye_frames, port


def compare(args):
    # Optional independent lap observations. Columns: log, laps, lap_ms, interventions, evidence.
    observations = {}
    if args.observations:
        with args.observations.open(newline="") as stream:
            observations = {row["log"]: row for row in csv.DictReader(stream)}
    print("log\tSPD\tstop\tdetected%\tsearch%\tmax_zero_ms\tdeadline%\tunknown_ms\tphysical_laps\tlap_ms\tinterventions\tevidence")
    for path in args.logs:
        configs = read_records(path, "LINE_CFG")
        runs = read_records(path, "LINE_STATS")
        timing = read_records(path, "LINE_TIMING")
        if not (len(configs) == len(runs) == len(timing) == 1):
            raise ValueError(f"Expected one complete firmware trial in {path}")
        config, run, tick = configs[0], runs[0], timing[0]
        known = int(run["KNOWN_US"])
        total = int(tick["N"]) + int(tick["MISSED"])
        ratio = lambda num, den: f"{100 * int(num) / den:.2f}" if den else "?"
        observed = observations.get(path.name, {})
        print("\t".join(map(str, [path.name, config["SPD"], run["REASON"],
              ratio(run["DETECTED_US"], known), ratio(run["SEARCH_US"], known),
              int(run["MAX_ZERO_US"]) / 1000, ratio(max(0, int(tick["N"]) - int(tick.get("LATE", 0))), total),
              int(run["UNKNOWN_US"]) / 1000, *[observed.get(k, "?") for k in
              ("laps", "lap_ms", "interventions", "evidence")]])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    record = commands.add_parser("capture")
    record.add_argument("port")
    record.add_argument("output", type=Path)
    record.add_argument("--seconds", type=float, default=3)
    record.add_argument("--setup", action="append", default=[], help="Stopped configuration or mode selection")
    record.add_argument("--motion", help="One explicitly confirmed trial command, e.g. 'LEFT 50' or GO")
    inspect = commands.add_parser("trace")
    inspect.add_argument("log", type=Path)
    inspect.add_argument("--duty", type=int, help="Restrict to frames with both requested duties equal to this value")
    comparison = commands.add_parser("compare")
    comparison.add_argument("logs", type=Path, nargs="+")
    comparison.add_argument("--observations", type=Path)
    cal = commands.add_parser("calibrate", help="Đo nền trắng + từng mắt trên vạch, tính và gửi CAL=")
    cal.add_argument("port", nargs="?", help="Cổng Serial của ESP32, ví dụ COM7. Bỏ trống để dùng --white/--sweep")
    cal.add_argument("--seconds", type=float, default=2, help="Thời gian ghi mỗi bước (mặc định 2 s)")
    cal.add_argument("--save", action="store_true", help="Gửi SAVE để giữ profile sau khi reset")
    cal.add_argument("--log", type=Path, help="Lưu log thô của lượt đo")
    cal.add_argument("--white", type=Path, help="Log có sẵn của lượt giữ nền trắng (TRACE=1)")
    cal.add_argument("--sweep", type=Path, help="Log có sẵn của lượt quét vạch (TRACE=1)")
    args = parser.parse_args()
    if args.action == "calibrate":
        if not 1 <= args.seconds <= 60:
            parser.error("--seconds must be in [1, 60]")
        calibrate(args)
    elif args.action == "capture":
        if not 0 < args.seconds <= 120:
            parser.error("--seconds must be in (0, 120]")
        capture(args)
    elif args.action == "trace":
        print(json.dumps(trace(args), indent=2))
    else:
        compare(args)


if __name__ == "__main__":
    main()
