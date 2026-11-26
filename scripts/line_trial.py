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
    args = parser.parse_args()
    if args.action == "capture":
        if not 0 < args.seconds <= 120:
            parser.error("--seconds must be in (0, 120]")
        capture(args)
    elif args.action == "trace":
        print(json.dumps(trace(args), indent=2))
    else:
        compare(args)


if __name__ == "__main__":
    main()
