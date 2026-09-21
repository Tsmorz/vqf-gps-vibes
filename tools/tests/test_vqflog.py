"""Tests for the recording parser and receiver -- no board needed.

The one test that matters most is `test_parses_the_firmwares_own_packet`: the
wire format is written twice (src/record_format.h and vqflog.py), and this feeds
the Python side bytes the C++ side produced. `task test-py` generates them.
"""

from __future__ import annotations

import argparse
import os
import socket
import threading
import time
from pathlib import Path

import numpy as np
import pytest

import vqf_record
import vqfplot
import vqflog

REFERENCE = Path(os.environ.get("VQF_REFERENCE_PACKET", ".pio/recordcheck/packet.hex"))


def make_packets(n_packets: int, start_seq: int = 0, t0_us: int = 0, skip: tuple = ()) -> list:
    """Synthetic 200 Hz packets: a 25 Hz tone on accel z, a slow yaw ramp."""
    packets = []
    for p in range(n_packets):
        seq = start_seq + p
        if seq in skip:
            continue
        records = np.zeros(10, dtype=vqflog.RECORD_DTYPE)
        index = seq * 10 + np.arange(10)
        t = index / 200.0
        records["t_us"] = (t0_us + (index * 5000)) & 0xFFFFFFFF
        records["accel"][:, 2] = 9.8 + 0.5 * np.sin(2 * np.pi * 25 * t)
        half = np.radians(t * 10) / 2  # yaw ramps at 10 deg/s
        records["quat"][:, 0] = np.cos(half)
        records["quat"][:, 3] = np.sin(half)
        records["flags"] = vqflog.FLAG_IMU_OK
        data = vqflog.HEADER.pack(vqflog.MAGIC, vqflog.VERSION, 10, 120, seq, 0) + records.tobytes()
        packets.append(data)
    return packets


def write_log(path: Path, packets: list, latency: float = 0.02) -> None:
    base = 1_800_000_000.0
    with vqflog.LogWriter(path) as log:
        for i, data in enumerate(packets):
            log.write(data, base + i * 0.05 + latency)


@pytest.mark.skipif(not REFERENCE.exists(), reason="run `task test-py` to build the reference packet")
def test_parses_the_firmwares_own_packet():
    packet = vqflog.decode_packet(bytes.fromhex(REFERENCE.read_text().strip()))
    assert (packet.seq, packet.dropped, len(packet.records)) == (12, 5, 2)
    first, second = packet.records
    assert (first["t_us"], second["t_us"]) == (1_000_000, 1_005_000)
    assert first["accel"].tolist() == [0.25, -0.5, 9.75]
    assert first["gyro"].tolist() == [0.125, -0.0625, 0.03125]
    assert first["mag"].tolist() == [12.5, -30.25, 41.75]
    assert first["quat"].tolist() == [0.5, 0.5, 0.5, 0.5]
    assert first["pos"].tolist() == [1.5, -2.5, 3.5]
    assert first["vel"].tolist() == [0.75, -0.25, 0.125]
    assert first["baro_pa"] == 101325.0 and first["baro_height_m"] == 12.5
    assert first["gps_lat"] == 51.5007292 and first["gps_lon"] == -0.1246254
    assert first["gps_update_count"] == 77 and first["gps_satellites"] == 9
    expected = (vqflog.FLAG_IMU_OK | vqflog.FLAG_MAG_OK | vqflog.FLAG_GPS_FIX | vqflog.FLAG_REST)
    assert first["flags"] == expected  # baro unhealthy, mag not disturbed in the reference


@pytest.mark.parametrize("mutate,message", [
    (lambda d: d[:10], "shorter"),
    (lambda d: b"XXXX" + d[4:], "magic"),
    (lambda d: d[:4] + bytes([9]) + d[5:], "version"),
    (lambda d: d[:6] + bytes([100, 0]) + d[8:], "record size"),
    (lambda d: d[:-1], "does not hold"),
])
def test_foreign_datagrams_are_rejected(mutate, message):
    with pytest.raises(vqflog.LogFormatError, match=message):
        vqflog.decode_packet(mutate(make_packets(1)[0]))


def test_a_log_round_trips_and_is_timed_from_the_boards_clock(tmp_path):
    path = tmp_path / "a.vqflog"
    write_log(path, make_packets(40))
    rec = vqflog.load(path)
    assert len(rec.samples) == 400 and rec.packets == 40
    assert rec.rate_hz == pytest.approx(200.0)
    assert rec.duration_s == pytest.approx(399 / 200.0)
    assert (rec.lost_packets, rec.board_dropped, rec.skipped) == (0, 0, 0)
    # Wall time advances with the board's clock, not with when packets landed.
    # (a float64 unix time resolves ~0.2 us, hence the tolerance)
    assert np.diff(rec.unix_time) == pytest.approx(0.005, abs=1e-6)


def test_wall_clock_is_anchored_to_the_least_delayed_packets(tmp_path):
    path = tmp_path / "a.vqflog"
    base = 1_800_000_000.0
    packets = make_packets(200)
    rng = np.random.default_rng(1)
    with vqflog.LogWriter(path) as log:
        for i, data in enumerate(packets):
            # The last sample of packet i was taken at 0.05*i + 0.045; latency is 10-60 ms.
            log.write(data, base + 0.05 * i + 0.045 + 0.010 + rng.uniform(0, 0.05))
    rec = vqflog.load(path)
    # The first sample was taken at base + 0.0, and the quietest packets had 10 ms latency.
    assert rec.unix_time[0] == pytest.approx(base + 0.010, abs=0.004)


def test_the_32_bit_clock_wrapping_is_invisible(tmp_path):
    path = tmp_path / "a.vqflog"
    write_log(path, make_packets(20, t0_us=2**32 - 250_000))  # wraps half way through the run
    rec = vqflog.load(path)
    assert rec.samples["t_us"].min() < rec.samples["t_us"][0]  # it really did wrap
    assert np.all(np.diff(rec.t_s) == pytest.approx(0.005))


def test_network_losses_are_counted_from_the_sequence(tmp_path):
    path = tmp_path / "a.vqflog"
    write_log(path, make_packets(30, skip=(5, 6, 20)))
    rec = vqflog.load(path)
    assert rec.lost_packets == 3 and rec.packets == 27


def test_a_board_restart_is_not_counted_as_loss(tmp_path):
    path = tmp_path / "a.vqflog"
    write_log(path, make_packets(10) + make_packets(10, t0_us=99_000_000))
    assert vqflog.load(path).lost_packets == 0


def test_a_truncated_file_still_loads(tmp_path):
    path = tmp_path / "a.vqflog"
    write_log(path, make_packets(10))
    path.write_bytes(path.read_bytes()[:-50])
    assert vqflog.load(path).packets == 9


def test_junk_between_packets_is_skipped_not_fatal(tmp_path):
    path = tmp_path / "a.vqflog"
    packets = make_packets(6)
    write_log(path, packets[:3] + [b"hello"] + packets[3:])
    rec = vqflog.load(path)
    assert rec.skipped == 1 and rec.packets == 6


def test_a_file_that_is_not_a_log_is_refused(tmp_path):
    path = tmp_path / "a.vqflog"
    path.write_bytes(b"not a log at all")
    with pytest.raises(vqflog.LogFormatError):
        vqflog.load(path)


def test_an_empty_recording_is_refused(tmp_path):
    path = tmp_path / "a.vqflog"
    write_log(path, [])
    with pytest.raises(vqflog.LogFormatError, match="no records"):
        vqflog.load(path)


def test_file_names_carry_the_date_and_time(tmp_path):
    from datetime import datetime
    name = vqflog.default_path(tmp_path, datetime(2026, 9, 21, 14, 30, 12)).name
    assert name == "vqf_20260921_143012.vqflog"


def test_euler_angles_agree_with_the_firmware_convention(tmp_path):
    path = tmp_path / "a.vqflog"
    write_log(path, make_packets(20))
    rec = vqflog.load(path)
    # make_packets ramps yaw at 10 deg/s about z.
    assert rec.euler_deg()[:, 2] == pytest.approx(10 * rec.t_s, abs=1e-3)
    assert rec.euler_deg()[:, :2] == pytest.approx(0.0, abs=1e-3)


def test_csv_has_a_column_for_every_name_and_keeps_full_precision(tmp_path):
    path = tmp_path / "a.vqflog"
    write_log(path, make_packets(5))
    rec = vqflog.load(path)
    rec.samples["gps_lat"][0] = 51.500729212
    out = tmp_path / "a.csv"
    vqflog.to_csv(rec, out)
    lines = out.read_text().splitlines()
    assert lines[0].split(",") == vqflog.CSV_COLUMNS
    assert len(lines) == 51
    row = dict(zip(vqflog.CSV_COLUMNS, lines[1].split(",")))
    assert float(row["gps_lat"]) == pytest.approx(51.500729212, abs=1e-9)
    assert float(row["unix_time"]) > 1.7e9


def test_the_spectrum_puts_a_tone_at_its_frequency_and_amplitude(tmp_path):
    path = tmp_path / "a.vqflog"
    write_log(path, make_packets(100))
    rec = vqflog.load(path)
    dt = np.diff(rec.t_s)
    freq, amp, used = vqfplot.amplitude_spectrum(rec.samples["accel"].astype(float), 200.0, dt)
    peak = np.argmax(amp[1:, 2]) + 1
    assert freq[peak] == pytest.approx(25.0, abs=0.8)
    assert amp[peak, 2] == pytest.approx(0.5, rel=0.1)  # the tone's amplitude, DC removed
    assert used > 1


def test_segments_across_a_gap_are_left_out_of_the_spectrum(tmp_path):
    path = tmp_path / "a.vqflog"
    write_log(path, make_packets(60, skip=(30, 31, 32, 33, 34)))
    rec = vqflog.load(path)
    dt = np.diff(rec.t_s)
    with_gap = vqfplot.amplitude_spectrum(rec.samples["accel"].astype(float), 200.0, dt)[2]
    without = vqfplot.amplitude_spectrum(rec.samples["accel"].astype(float), 200.0,
                                         np.full_like(dt, 0.005))[2]
    assert with_gap < without


def test_the_report_builds_and_names_every_section(tmp_path):
    path = tmp_path / "vqf_20260921_143012.vqflog"
    write_log(path, make_packets(100))
    html = vqfplot.build_report(vqflog.load(path), path)
    for text in ("Accelerometer", "Track", "Amplitude spectrum", "vqf_20260921_143012.vqflog",
                 "no GPS fix in this recording"):
        assert text in html
    assert "cdn.plot.ly" in html


def test_listen_records_a_stream_into_a_new_file_and_ignores_strangers(tmp_path):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    args = argparse.Namespace(port=port, bind="127.0.0.1", out_dir=tmp_path, seconds=2.0)
    vqf_record.configure_logging(False)
    result = []
    thread = threading.Thread(target=lambda: result.append(vqf_record.listen(args)))
    thread.start()
    time.sleep(0.4)

    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for data in make_packets(8, skip=(3,)):
        sender.sendto(data, ("127.0.0.1", port))
        time.sleep(0.01)
    sender.sendto(b"garbage", ("127.0.0.1", port))
    thread.join(timeout=10)

    assert result == [0]
    (log,) = tmp_path.glob("vqf_*.vqflog")
    rec = vqflog.load(log)
    assert rec.packets == 7 and rec.lost_packets == 1


def test_listen_with_nothing_arriving_leaves_no_file(tmp_path):
    args = argparse.Namespace(port=0, bind="127.0.0.1", out_dir=tmp_path, seconds=0.6)
    vqf_record.configure_logging(False)
    assert vqf_record.listen(args) == 1
    assert list(tmp_path.glob("*.vqflog")) == []
