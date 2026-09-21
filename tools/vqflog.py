"""Reads and writes the board's UDP recordings.

The wire format is defined by src/record_format.h; RECORD_DTYPE below is its
mirror, and tools/tests/test_vqflog.py checks the two against a packet built by
the firmware's own encoder. Change one and that test fails, rather than the log
parsing into plausible numbers in the wrong columns.

A recording on disk (.vqflog) is a small header followed by the datagrams
exactly as they arrived, each stamped with the receiver's clock:

    b"VQFLOG01"  u32 meta_len  meta (JSON)
    repeated:    f64 host_unix_time  u16 length  <datagram>

Keeping the datagrams verbatim means the parser can be improved later without
having lost anything, and the host stamp is what anchors the board's own
free-running microsecond clock to a wall-clock date.
"""

from __future__ import annotations

import json
import struct
import time
from collections.abc import Iterator
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import BinaryIO

import numpy as np
from loguru import logger

# ── Wire format (mirrors src/record_format.h) ────────────────────────────────
MAGIC = b"VQFL"
VERSION = 1
HEADER = struct.Struct("<4sBBHII")  # magic, version, count, record_size, seq, dropped

RECORD_DTYPE = np.dtype(
    [
        ("t_us", "<u4"),
        ("accel", "<f4", (3,)),
        ("gyro", "<f4", (3,)),
        ("mag", "<f4", (3,)),
        ("quat", "<f4", (4,)),
        ("pos", "<f4", (3,)),
        ("vel", "<f4", (3,)),
        ("baro_pa", "<f4"),
        ("baro_height_m", "<f4"),
        ("gps_lat", "<f8"),
        ("gps_lon", "<f8"),
        ("gps_alt_m", "<f4"),
        ("gps_hdop", "<f4"),
        ("gps_update_count", "<u4"),
        ("gps_satellites", "u1"),
        ("flags", "u1"),
        ("reserved", "<u2"),
    ]
)
assert RECORD_DTYPE.itemsize == 120 and HEADER.size == 16

FLAG_IMU_OK = 1 << 0
FLAG_MAG_OK = 1 << 1
FLAG_GPS_FIX = 1 << 2
FLAG_BARO_OK = 1 << 3
FLAG_REST = 1 << 4
FLAG_MAG_DISTURBED = 1 << 5

# ── File format ──────────────────────────────────────────────────────────────
FILE_MAGIC = b"VQFLOG01"
_ENTRY = struct.Struct("<dH")
SUFFIX = ".vqflog"


class LogFormatError(ValueError):
    """A datagram or file that is not something this firmware version writes."""


@dataclass(frozen=True)
class Packet:
    seq: int
    dropped: int  # records the board lost before sending, cumulative
    records: np.ndarray


def default_path(directory: Path, when: datetime | None = None) -> Path:
    """recordings/vqf_20260921_143012.vqflog -- local time, sortable, no colons."""
    when = when or datetime.now().astimezone()
    return directory / f"vqf_{when:%Y%m%d_%H%M%S}{SUFFIX}"


def decode_packet(data: bytes) -> Packet:
    """Validates one datagram. Raises LogFormatError for anything foreign."""
    if len(data) < HEADER.size:
        raise LogFormatError(f"{len(data)} bytes is shorter than the {HEADER.size}-byte header")
    magic, version, count, record_size, seq, dropped = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise LogFormatError(f"bad magic {magic!r}")
    if version != VERSION:
        raise LogFormatError(f"format version {version}; this tool reads {VERSION}")
    if record_size != RECORD_DTYPE.itemsize:
        raise LogFormatError(f"record size {record_size}, expected {RECORD_DTYPE.itemsize}")
    if len(data) != HEADER.size + count * record_size:
        raise LogFormatError(f"{len(data)} bytes does not hold {count} records")
    records = np.frombuffer(data, dtype=RECORD_DTYPE, count=count, offset=HEADER.size)
    return Packet(seq=seq, dropped=dropped, records=records)


# ── Writing ──────────────────────────────────────────────────────────────────
class LogWriter:
    """Appends datagrams to a .vqflog. Use as a context manager."""

    def __init__(self, path: Path, **meta: object) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._file: BinaryIO = open(path, "wb")
        now = datetime.now().astimezone()
        info = {"format": 1, "started": now.isoformat(timespec="milliseconds"),
                "started_unix": now.timestamp(), **meta}
        blob = json.dumps(info).encode()
        self._file.write(FILE_MAGIC + struct.pack("<I", len(blob)) + blob)
        self._last_flush = time.monotonic()

    def write(self, datagram: bytes, host_time: float | None = None) -> None:
        stamp = time.time() if host_time is None else host_time
        self._file.write(_ENTRY.pack(stamp, len(datagram)) + datagram)
        # A crash or a pulled plug should cost a second of data, not the file.
        if time.monotonic() - self._last_flush > 1.0:
            self._file.flush()
            self._last_flush = time.monotonic()

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> LogWriter:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


# ── Reading ──────────────────────────────────────────────────────────────────
def read_meta(handle: BinaryIO) -> dict:
    if handle.read(len(FILE_MAGIC)) != FILE_MAGIC:
        raise LogFormatError("not a .vqflog file")
    (length,) = struct.unpack("<I", handle.read(4))
    return json.loads(handle.read(length))


def iter_entries(handle: BinaryIO) -> Iterator[tuple[float, bytes]]:
    """Yields (host_time, datagram). A file cut off mid-entry ends cleanly: the
    usual cause is a recording still being written, or a crash."""
    while True:
        head = handle.read(_ENTRY.size)
        if len(head) < _ENTRY.size:
            return
        host_time, length = _ENTRY.unpack(head)
        datagram = handle.read(length)
        if len(datagram) < length:
            logger.warning("file ends inside a datagram; ignoring the partial entry")
            return
        yield host_time, datagram


@dataclass
class Recording:
    meta: dict
    samples: np.ndarray  # RECORD_DTYPE, in arrival order
    t_s: np.ndarray  # seconds since the first sample, from the board's clock
    unix_time: np.ndarray  # wall-clock estimate per sample
    packets: int = 0
    lost_packets: int = 0  # sequence-number gaps: the network's losses
    board_dropped: int = 0  # records the board itself dropped
    skipped: int = 0  # datagrams that did not parse
    notes: list[str] = field(default_factory=list)

    @property
    def duration_s(self) -> float:
        return float(self.t_s[-1]) if len(self.t_s) else 0.0

    @property
    def rate_hz(self) -> float:
        """The rate the samples really arrived at -- not the nominal 200."""
        if len(self.t_s) < 2:
            return 0.0
        return float(1.0 / np.median(np.diff(self.t_s)))

    def euler_deg(self) -> np.ndarray:
        """(N, 3) roll, pitch, yaw in degrees. The same formulas as
        attitude::QuatToEulerDegrees in the firmware, so this agrees with the
        dashboard's readouts."""
        w, x, y, z = self.samples["quat"].astype(np.float64).T
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return np.degrees(np.stack([roll, pitch, yaw], axis=1))

    def flag(self, bit: int) -> np.ndarray:
        return (self.samples["flags"] & bit) != 0

    def gps_fixes(self) -> np.ndarray:
        """One row per fix the filter accepted, dropping the repeats a record
        carries between updates. Indices into `samples`."""
        count = self.samples["gps_update_count"]
        first = np.concatenate([[True], np.diff(count.astype(np.int64)) != 0])
        return np.flatnonzero(first & self.flag(FLAG_GPS_FIX))


def _unwrap_time(t_us: np.ndarray) -> np.ndarray:
    """Seconds from a 32-bit microsecond counter that wraps every 71 minutes.
    Differences are taken in uint32, where a wrap subtracts correctly."""
    steps = np.diff(t_us.astype(np.uint32))
    return np.concatenate([[0.0], np.cumsum(steps, dtype=np.float64)]) * 1e-6


def _wall_clock(t_s: np.ndarray, host_time: np.ndarray) -> np.ndarray:
    """Places the board's clock on the wall clock.

    Every sample in a packet arrives at the packet's timestamp, so each one's
    arrival is its packet's stamp minus how much earlier the board took it. The
    board's clock drifts nowhere near as fast as the network's latency varies,
    so one offset serves the whole file -- and since latency only ever adds,
    the low end of the offsets is the least-delayed, hence the truest, one.
    """
    offset = np.percentile(host_time - t_s, 2.0)
    return t_s + offset


def load(path: Path, progress: bool = False) -> Recording:
    """Parses a .vqflog into a Recording."""
    from tqdm import tqdm

    chunks: list[np.ndarray] = []
    arrival: list[np.ndarray] = []
    last_seq: int | None = None
    lost = skipped = packets = board_dropped = 0

    size = path.stat().st_size
    with open(path, "rb") as handle, tqdm(
        total=size, unit="B", unit_scale=True, desc=path.name, disable=not progress, leave=False
    ) as bar:
        meta = read_meta(handle)
        bar.update(handle.tell())
        for host_time, datagram in iter_entries(handle):
            bar.update(_ENTRY.size + len(datagram))
            try:
                packet = decode_packet(datagram)
            except LogFormatError as error:
                skipped += 1
                logger.warning("skipping datagram: {}", error)
                continue
            if last_seq is not None and packet.seq != last_seq + 1:
                # The board restarted (sequence back to 0) or the network lost
                # some. A restart is not a loss, and the gap in the sample
                # clock is visible in the data either way.
                if packet.seq > last_seq:
                    lost += packet.seq - last_seq - 1
            last_seq = packet.seq
            board_dropped = max(board_dropped, packet.dropped)
            packets += 1
            chunks.append(packet.records)
            arrival.append(np.full(len(packet.records), host_time))

    if not chunks:
        raise LogFormatError(f"{path} holds no records")
    samples = np.concatenate(chunks)
    t_s = _unwrap_time(samples["t_us"])
    # Time of the last sample in each packet, to measure how early the others were.
    lengths = [len(c) for c in chunks]
    ends = np.cumsum(lengths) - 1
    packet_end = np.repeat(t_s[ends], lengths)
    unix = _wall_clock(t_s, np.concatenate(arrival) - (packet_end - t_s))

    return Recording(meta=meta, samples=samples, t_s=t_s, unix_time=unix, packets=packets,
                     lost_packets=lost, board_dropped=board_dropped, skipped=skipped)


# ── Export ───────────────────────────────────────────────────────────────────
CSV_COLUMNS = (
    ["unix_time", "t_s"]
    + [f"accel_{a}" for a in "xyz"] + [f"gyro_{a}" for a in "xyz"]
    + [f"mag_{a}" for a in "xyz"] + ["qw", "qx", "qy", "qz", "roll_deg", "pitch_deg", "yaw_deg"]
    + [f"pos_{a}" for a in "enu"] + [f"vel_{a}" for a in "enu"]
    + ["baro_pa", "baro_height_m", "gps_lat", "gps_lon", "gps_alt_m", "gps_hdop",
       "gps_update_count", "gps_satellites", "imu_ok", "mag_ok", "gps_fix", "baro_ok", "rest",
       "mag_disturbed"]
)


def to_csv(rec: Recording, path: Path) -> None:
    """Every sample as one row. Angles are in degrees and the rest in the
    firmware's units (m/s^2, rad/s, uT, m, m/s, Pa)."""
    s = rec.samples
    columns = [rec.unix_time, rec.t_s, *s["accel"].T, *s["gyro"].T, *s["mag"].T, *s["quat"].T,
               *rec.euler_deg().T, *s["pos"].T, *s["vel"].T, s["baro_pa"], s["baro_height_m"],
               s["gps_lat"], s["gps_lon"], s["gps_alt_m"], s["gps_hdop"],
               s["gps_update_count"], s["gps_satellites"]]
    columns += [rec.flag(bit).astype(int) for bit in (
        FLAG_IMU_OK, FLAG_MAG_OK, FLAG_GPS_FIX, FLAG_BARO_OK, FLAG_REST, FLAG_MAG_DISTURBED)]
    assert len(columns) == len(CSV_COLUMNS)
    # Full precision where it matters: a unix time and a latitude both need
    # digits a float32 default would round away.
    fmt = {"unix_time": "%.6f", "t_s": "%.6f", "gps_lat": "%.9f", "gps_lon": "%.9f"}
    formats = [fmt.get(name, "%.7g") for name in CSV_COLUMNS]
    for name in ("gps_update_count", "gps_satellites", "imu_ok", "mag_ok", "gps_fix", "baro_ok",
                 "rest", "mag_disturbed"):
        formats[CSV_COLUMNS.index(name)] = "%d"
    np.savetxt(path, np.column_stack(columns), delimiter=",", header=",".join(CSV_COLUMNS),
               comments="", fmt=formats)
