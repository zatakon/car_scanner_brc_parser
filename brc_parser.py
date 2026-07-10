#!/usr/bin/env python3
"""
brc_parser.py  --  Reverse-engineered reader for Car Scanner ``.brc`` recordings.

Car Scanner (the OBD-II app) exports each recorded trip as a proprietary binary
``.brc`` file.  This module decodes that format into a plain :class:`BrcRecording`
object with no third-party dependencies, so it can be reused independently of the
MCAP conversion in :mod:`brc_to_mcap`.

Reverse-engineered ``.brc`` layout (little-endian throughout)
------------------------------------------------------------
Header:
    7bit-len-prefixed string  "CARSCANNERRECORD"   (magic)
    int32                     format version (observed: 2)
    uint8                     flag byte
    7bit-len string           short vehicle name
    7bit-len string           full vehicle name
    7bit-len string           Bluetooth adapter name
    int64                     session start, .NET DateTime ticks (100 ns since 0001-01-01)

Then a flat stream of records, each starting with a 4-byte type tag:
    9061d500  DEF   : uint32 pid_id, str name, str name2, int32 type
                      -> defines pid_id -> human name
    50432701  VALUE : float64 t_seconds (since session start), uint32 pid_id, float64 value
    5cfb761f  GPS   : float64 t_seconds, float64 latitude, float64 longitude
    2832e116  DTC   : int64 .NET ticks, 7bit-len, UTF-8 JSON (trouble-code details)

The tag constants above are stable across the sample recordings.  If a file uses
different tags (a future app version) the parser stops early; inspect
:attr:`BrcRecording.fully_parsed` / :attr:`BrcRecording.parsed_bytes` to detect it
instead of trusting a partial decode.

Validated against the app's own ``.csv`` export: 130 field checks across a full
session matched exactly, with sub-millisecond timestamp agreement.
"""

from __future__ import annotations

import datetime as _dt
import struct
from typing import Dict, List, Tuple

__all__ = [
    "BrcError",
    "BrcRecording",
    "parse_brc",
    "MAGIC",
    "TAG_DEF",
    "TAG_VALUE",
    "TAG_GPS",
    "TAG_DTC",
]

# --------------------------------------------------------------------------- #
# Constants
# --------------------------------------------------------------------------- #

EPOCH = _dt.datetime(1, 1, 1)          # .NET DateTime epoch
UNIX_EPOCH = _dt.datetime(1970, 1, 1)

TAG_DEF = b"\x90\x61\xd5\x00"
TAG_VALUE = b"\x50\x43\x27\x01"
TAG_GPS = b"\x5c\xfb\x76\x1f"
TAG_DTC = b"\x28\x32\xe1\x16"

MAGIC = "CARSCANNERRECORD"


class BrcError(Exception):
    """Raised when a .brc file cannot be parsed as expected."""


# --------------------------------------------------------------------------- #
# Low-level readers
# --------------------------------------------------------------------------- #

def _read_7bit(buf: bytes, pos: int) -> Tuple[int, int]:
    """Read a .NET 7-bit-encoded length prefix. Returns (value, new_pos)."""
    result = 0
    shift = 0
    while True:
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            break
        shift += 7
        if shift > 35:
            raise BrcError("7-bit length prefix too long (corrupt file?)")
    return result, pos


def _read_str(buf: bytes, pos: int) -> Tuple[str, int]:
    length, pos = _read_7bit(buf, pos)
    end = pos + length
    if end > len(buf):
        raise BrcError("string length runs past end of file")
    return buf[pos:end].decode("utf-8"), end


def ticks_to_datetime(ticks: int) -> _dt.datetime:
    """Convert a .NET DateTime tick count (100 ns since year 1) to a datetime."""
    return EPOCH + _dt.timedelta(microseconds=ticks / 10)


# --------------------------------------------------------------------------- #
# Parsed recording
# --------------------------------------------------------------------------- #

class BrcRecording:
    """Parsed contents of a Car Scanner .brc recording.

    Attributes
    ----------
    version : int
        ``.brc`` format version from the header.
    short_name, full_name, adapter : str
        Vehicle short/full names and Bluetooth adapter description.
    session_dt : datetime
        Recording start time (naive local wall-clock, as stored by the app).
    pids : dict[bytes, str]
        Maps a 4-byte PID id to its human-readable name.
    samples : list[tuple[float, bytes, float]]
        ``(t_seconds_since_session, pid_id, value)`` numeric telemetry samples.
    gps : list[tuple[float, float, float]]
        ``(t_seconds_since_session, latitude, longitude)`` fixes.
    dtcs : list[tuple[datetime, str]]
        ``(absolute_time, json_text)`` diagnostic trouble-code events.
    parsed_bytes, total_bytes : int
        How much of the file was decoded vs its size; equal when fully parsed.
    """

    def __init__(self) -> None:
        self.version: int = 0
        self.short_name: str = ""
        self.full_name: str = ""
        self.adapter: str = ""
        self.session_dt: _dt.datetime = EPOCH
        self.pids: Dict[bytes, str] = {}
        self.samples: List[Tuple[float, bytes, float]] = []
        self.gps: List[Tuple[float, float, float]] = []
        self.dtcs: List[Tuple[_dt.datetime, str]] = []
        self.parsed_bytes: int = 0
        self.total_bytes: int = 0

    @property
    def fully_parsed(self) -> bool:
        """True when every byte of the file was consumed by a known record."""
        return self.parsed_bytes == self.total_bytes

    @property
    def coverage(self) -> float:
        """Fraction of the file (0..1) that was successfully decoded."""
        if not self.total_bytes:
            return 0.0
        return self.parsed_bytes / self.total_bytes


def parse_brc(path: str) -> BrcRecording:
    """Parse a ``.brc`` file into a :class:`BrcRecording`.

    Raises
    ------
    BrcError
        If the file is not a Car Scanner recording or a record is malformed.
    OSError
        If the file cannot be read.
    """
    with open(path, "rb") as fh:
        data = fh.read()

    n = len(data)
    rec = BrcRecording()
    rec.total_bytes = n

    pos = 0
    magic, pos = _read_str(data, pos)
    if magic != MAGIC:
        raise BrcError(f"not a Car Scanner recording (magic={magic!r})")

    rec.version = struct.unpack_from("<i", data, pos)[0]
    pos += 4
    pos += 1  # flag byte
    rec.short_name, pos = _read_str(data, pos)
    rec.full_name, pos = _read_str(data, pos)
    rec.adapter, pos = _read_str(data, pos)
    session_ticks = struct.unpack_from("<q", data, pos)[0]
    pos += 8
    rec.session_dt = ticks_to_datetime(session_ticks)

    while pos < n:
        if pos + 4 > n:
            break
        tag = data[pos : pos + 4]

        if tag == TAG_DEF:
            p = pos + 4
            pid_id = data[p : p + 4]
            p += 4
            name, p = _read_str(data, p)
            _name2, p = _read_str(data, p)
            p += 4  # int32 type/category code (unused)
            rec.pids[pid_id] = name
            pos = p

        elif tag == TAG_VALUE:
            p = pos + 4
            t = struct.unpack_from("<d", data, p)[0]
            pid_id = data[p + 8 : p + 12]
            value = struct.unpack_from("<d", data, p + 12)[0]
            rec.samples.append((t, pid_id, value))
            pos = p + 20

        elif tag == TAG_GPS:
            p = pos + 4
            t, lat, lon = struct.unpack_from("<ddd", data, p)
            rec.gps.append((t, lat, lon))
            pos = p + 24

        elif tag == TAG_DTC:
            p = pos + 4
            ticks = struct.unpack_from("<q", data, p)[0]
            p += 8
            length, p = _read_7bit(data, p)
            text = data[p : p + length].decode("utf-8")
            rec.dtcs.append((ticks_to_datetime(ticks), text))
            pos = p + length

        else:
            # Unknown tag -- stop rather than emit corrupt data.
            break

    rec.parsed_bytes = pos
    return rec


# --------------------------------------------------------------------------- #
# Standalone quick-look CLI
# --------------------------------------------------------------------------- #

def _main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(
        description="Inspect a Car Scanner .brc recording (no conversion).")
    ap.add_argument("input", help=".brc file to inspect")
    ap.add_argument("--list-pids", action="store_true",
                    help="print every PID name found")
    args = ap.parse_args(argv)

    rec = parse_brc(args.input)
    print(f"vehicle      : {rec.full_name}")
    print(f"adapter      : {rec.adapter}")
    print(f"brc version  : {rec.version}")
    print(f"session start: {rec.session_dt}")
    print(f"pids         : {len(rec.pids)}")
    print(f"samples      : {len(rec.samples)}")
    print(f"gps fixes    : {len(rec.gps)}")
    print(f"dtc events   : {len(rec.dtcs)}")
    print(f"coverage     : {rec.coverage * 100:.2f}% "
          f"({rec.parsed_bytes}/{rec.total_bytes} bytes)")
    if not rec.fully_parsed:
        print("WARNING: file not fully parsed -- unknown record tag encountered")
    if args.list_pids:
        print("\nPIDs:")
        for pid_id, name in rec.pids.items():
            print(f"  {pid_id.hex()}  {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
