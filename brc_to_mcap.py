#!/usr/bin/env python3
"""
brc_to_mcap.py  --  Convert Car Scanner ``.brc`` recordings to ROS 2 MCAP files.

Car Scanner (the OBD-II app) exports each recorded trip as a proprietary binary
``.brc`` file.  This tool re-encodes it as a standard ROS 2 ``.mcap`` bag that can
be opened directly in Foxglove Studio, ``ros2 bag`` / ``rosbag2``, Lichtblick,
PlotJuggler, etc.

The ``.brc`` decoding lives in the dependency-free :mod:`brc_parser` module; this
file only turns a parsed :class:`~brc_parser.BrcRecording` into MCAP and provides
the command-line interface.

What ends up in the bag
-----------------------
* ``/pid/<name>``   -- one ``std_msgs/msg/Float64`` topic per telemetry channel
                       (Engine RPM, coolant temp, boost, fuel trims, ...).
* ``/gps/fix``      -- ``sensor_msgs/msg/NavSatFix`` (latitude / longitude).
* ``/dtc/events``   -- ``std_msgs/msg/String`` carrying the raw diagnostic
                       trouble-code JSON, when the recording contains any.
* ``/session/pid_catalog`` -- a one-shot ``std_msgs/msg/String`` (JSON) that maps
                       each sanitized topic name back to the original PID label.
* MCAP metadata record ``car_scanner`` with vehicle / adapter / session info.

Message ``log_time`` is the recording's wall-clock time.  The absolute time
stored by Car Scanner is a naive local timestamp; it is written to the bag as-is
(interpreted as UTC) so the clock digits match the app, and relative timing
between samples is exact.

Usage
-----
    python brc_to_mcap.py INPUT [-o OUTPUT] [--compression zstd|lz4|none]

INPUT may be a single ``.brc`` file or a directory (all ``*.brc`` inside are
converted).  With a single file, ``-o`` names the output file; otherwise outputs
are written next to their inputs (or into ``-o`` if it is a directory).
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
import sys
from typing import Dict, List, Optional

from brc_parser import BrcError, BrcRecording, parse_brc, UNIX_EPOCH

# --------------------------------------------------------------------------- #
# ROS 2 schema definitions
# --------------------------------------------------------------------------- #

FLOAT64_DEF = "float64 data"

STRING_DEF = "string data"

NAVSATFIX_DEF = """std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: sensor_msgs/NavSatStatus
int8 status
uint16 service
"""


# --------------------------------------------------------------------------- #
# Topic-name sanitising
# --------------------------------------------------------------------------- #

_slug_re = re.compile(r"[^a-z0-9]+")


def slugify(name: str) -> str:
    """Turn a PID label into a valid ROS topic token."""
    s = name.lower()
    s = s.replace("%", "percent").replace("°", "deg").replace("℃", "degc")
    s = _slug_re.sub("_", s).strip("_")
    if not s:
        s = "pid"
    if s[0].isdigit():
        s = "_" + s
    return s


# --------------------------------------------------------------------------- #
# Time helpers
# --------------------------------------------------------------------------- #

def _unix_nanos(session_dt: _dt.datetime, t_sec: float) -> int:
    """Absolute UNIX-epoch nanoseconds for a sample at t_sec into the session.

    The session timestamp is a naive local wall-clock; it is treated as UTC so
    the clock digits shown in viewers match the Car Scanner app.
    """
    total_seconds = (session_dt - UNIX_EPOCH).total_seconds() + t_sec
    return int(round(total_seconds * 1e9))


def _abs_nanos(dt: _dt.datetime) -> int:
    return int(round((dt - UNIX_EPOCH).total_seconds() * 1e9))


def _time_msg(nanos: int) -> Dict[str, int]:
    return {"sec": nanos // 1_000_000_000, "nanosec": nanos % 1_000_000_000}


# --------------------------------------------------------------------------- #
# Conversion
# --------------------------------------------------------------------------- #

def convert(rec: BrcRecording, out_path: str, compression: str = "zstd") -> Dict[str, int]:
    """Write a parsed recording to an MCAP file. Returns per-topic message counts."""
    from mcap.writer import CompressionType
    from mcap_ros2.writer import Writer

    comp = {
        "zstd": CompressionType.ZSTD,
        "lz4": CompressionType.LZ4,
        "none": CompressionType.NONE,
    }[compression]

    # Resolve unique topic slugs up front (guard against collisions).
    used: Dict[str, bytes] = {}
    pid_topic: Dict[bytes, str] = {}
    catalog: Dict[str, str] = {}
    for pid_id, name in rec.pids.items():
        base = slugify(name)
        slug = base
        i = 2
        while slug in used and used[slug] != pid_id:
            slug = f"{base}_{i}"
            i += 1
        used[slug] = pid_id
        topic = f"/pid/{slug}"
        pid_topic[pid_id] = topic
        catalog[topic] = name

    counts: Dict[str, int] = {}

    with open(out_path, "wb") as fh:
        writer = Writer(fh, compression=comp)

        f64_schema = writer.register_msgdef("std_msgs/msg/Float64", FLOAT64_DEF)
        str_schema = writer.register_msgdef("std_msgs/msg/String", STRING_DEF)
        nav_schema = writer.register_msgdef("sensor_msgs/msg/NavSatFix", NAVSATFIX_DEF)

        session_start_ns = _unix_nanos(rec.session_dt, 0.0)

        # Vehicle / session metadata (queryable via `mcap info` / readers).
        writer._writer.add_metadata(
            "car_scanner",
            {
                "brc_version": str(rec.version),
                "vehicle_short": rec.short_name,
                "vehicle_full": rec.full_name,
                "adapter": rec.adapter,
                "session_start": rec.session_dt.isoformat(),
                "pid_count": str(len(rec.pids)),
                "sample_count": str(len(rec.samples)),
                "gps_count": str(len(rec.gps)),
                "dtc_count": str(len(rec.dtcs)),
            },
        )

        # One-shot catalog message so topic slugs can be decoded downstream.
        writer.write_message(
            "/session/pid_catalog",
            str_schema,
            {"data": json.dumps({
                "vehicle": rec.full_name,
                "adapter": rec.adapter,
                "session_start": rec.session_dt.isoformat(),
                "pids": catalog,
            }, ensure_ascii=False)},
            log_time=session_start_ns,
            publish_time=session_start_ns,
        )
        counts["/session/pid_catalog"] = 1

        # Numeric PID samples.
        for t, pid_id, value in rec.samples:
            topic = pid_topic.get(pid_id)
            if topic is None:  # value before its definition -- name it by id
                topic = f"/pid/unknown_{pid_id.hex()}"
                pid_topic[pid_id] = topic
                catalog[topic] = f"unknown ({pid_id.hex()})"
            ns = _unix_nanos(rec.session_dt, t)
            writer.write_message(topic, f64_schema, {"data": value},
                                 log_time=ns, publish_time=ns)
            counts[topic] = counts.get(topic, 0) + 1

        # GPS fixes.
        for t, lat, lon in rec.gps:
            ns = _unix_nanos(rec.session_dt, t)
            msg = {
                "header": {"stamp": _time_msg(ns), "frame_id": "gps"},
                "status": {"status": 0, "service": 1},
                "latitude": lat,
                "longitude": lon,
                "altitude": 0.0,
                "position_covariance": [0.0] * 9,
                "position_covariance_type": 0,
            }
            writer.write_message("/gps/fix", nav_schema, msg,
                                 log_time=ns, publish_time=ns)
            counts["/gps/fix"] = counts.get("/gps/fix", 0) + 1

        # Diagnostic trouble-code events (absolute .NET ticks -> UNIX ns).
        for dt, text in rec.dtcs:
            ns = _abs_nanos(dt)
            writer.write_message("/dtc/events", str_schema, {"data": text},
                                 log_time=ns, publish_time=ns)
            counts["/dtc/events"] = counts.get("/dtc/events", 0) + 1

        writer.finish()

    return counts


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def _iter_inputs(input_path: str) -> List[str]:
    if os.path.isdir(input_path):
        return [
            os.path.join(input_path, f)
            for f in sorted(os.listdir(input_path))
            if f.lower().endswith(".brc")
        ]
    return [input_path]


def _output_for(inp: str, out: Optional[str], multi: bool) -> str:
    if out is None:
        return os.path.splitext(inp)[0] + ".mcap"
    if multi or os.path.isdir(out):
        os.makedirs(out, exist_ok=True)
        base = os.path.splitext(os.path.basename(inp))[0] + ".mcap"
        return os.path.join(out, base)
    return out


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Convert Car Scanner .brc recordings to ROS 2 MCAP bags.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("input", help=".brc file or a directory containing .brc files")
    ap.add_argument("-o", "--output",
                    help="output .mcap file (single input) or output directory")
    ap.add_argument("--compression", choices=["zstd", "lz4", "none"], default="zstd",
                    help="MCAP chunk compression (default: zstd)")
    ap.add_argument("-q", "--quiet", action="store_true", help="suppress per-file output")
    args = ap.parse_args(argv)

    try:
        import mcap  # noqa: F401
        import mcap_ros2  # noqa: F401
    except ImportError:
        print("error: missing dependencies. Install with:\n"
              "    pip install mcap mcap-ros2-support", file=sys.stderr)
        return 2

    inputs = _iter_inputs(args.input)
    if not inputs:
        print(f"error: no .brc files found at {args.input}", file=sys.stderr)
        return 1
    multi = len(inputs) > 1

    rc = 0
    for inp in inputs:
        out = _output_for(inp, args.output, multi)
        try:
            rec = parse_brc(inp)
        except (BrcError, OSError) as e:
            print(f"[SKIP] {inp}: {e}", file=sys.stderr)
            rc = 1
            continue

        counts = convert(rec, out, compression=args.compression)

        if not args.quiet:
            total = sum(counts.values())
            print(f"[OK] {os.path.basename(inp)} -> {out}")
            print(f"     vehicle : {rec.full_name}")
            print(f"     adapter : {rec.adapter}")
            print(f"     start   : {rec.session_dt}")
            print(f"     topics  : {len(rec.pids)} PIDs + GPS/DTC | "
                  f"{total} messages | {len(rec.gps)} GPS | {len(rec.dtcs)} DTC")
            if not rec.fully_parsed:
                print(f"     WARNING : parsed only {rec.coverage * 100:.2f}% of file "
                      f"({rec.parsed_bytes}/{rec.total_bytes} bytes) -- "
                      f"unknown record tag encountered", file=sys.stderr)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
