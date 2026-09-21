#!/usr/bin/env python3
"""Records the board's UDP stream to a date-stamped file, and reads it back.

    tools/vqf_record.py listen [--port 5005] [--out-dir recordings] [--seconds N]
    tools/vqf_record.py parse  recordings/vqf_20260921_143012.vqflog [--csv out.csv]
    tools/vqf_record.py plot   recordings/vqf_20260921_143012.vqflog [--open]

Run `listen`, then press "Start recording" on the dashboard (http://192.168.4.1/
on the board's own network). The board streams every 200 Hz estimator tick to
the machine whose browser pressed the button. Each `listen` writes a new file
named for the moment it started, so nothing is ever overwritten.
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
import webbrowser
from pathlib import Path

from loguru import logger
from tqdm import tqdm

import vqfplot
import vqflog


def configure_logging(verbose: bool) -> None:
    logger.remove()
    # tqdm owns the terminal line while a bar is up; routing log records
    # through it keeps them from being drawn over the bar.
    logger.add(lambda message: tqdm.write(message, end="", file=sys.stderr), level="DEBUG" if verbose else "INFO",
               colorize=True, format="<green>{time:HH:mm:ss}</green> <level>{level: <7}</level> "
                                     "{message}")


def listen(args: argparse.Namespace) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 200 Hz of 1.2 kB datagrams is small, but the default buffer is smaller
    # than a scheduling hiccup on a busy laptop.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    try:
        sock.bind((args.bind, args.port))
    except OSError as error:
        logger.error("cannot listen on {}:{}: {}", args.bind, args.port, error)
        return 1
    sock.settimeout(0.5)

    path = vqflog.default_path(args.out_dir)
    logger.info("listening on UDP {}:{} -> {}", args.bind, args.port, path)
    logger.info("press 'Start recording' on the dashboard (port {})", args.port)

    records = packets = bad = 0
    source: tuple[str, int] | None = None
    started = time.monotonic()
    deadline = started + args.seconds if args.seconds else None
    last_seq: int | None = None
    lost = 0

    bar = tqdm(unit=" rec", desc="waiting for the board", dynamic_ncols=True)
    try:
        with vqflog.LogWriter(path, port=args.port) as log:
            while deadline is None or time.monotonic() < deadline:
                try:
                    data, address = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                host_time = time.time()
                try:
                    packet = vqflog.decode_packet(data)
                except vqflog.LogFormatError as error:
                    bad += 1
                    logger.warning("ignoring datagram from {}: {}", address[0], error)
                    continue
                if source is None:
                    source = address
                    logger.info("receiving from {}", address[0])
                    bar.set_description("recording")
                    started = time.monotonic()
                elif address != source:
                    continue  # a stray sender must not corrupt the file
                if last_seq is not None and packet.seq > last_seq + 1:
                    lost += packet.seq - last_seq - 1
                last_seq = packet.seq
                log.write(data, host_time)
                packets += 1
                records += len(packet.records)
                bar.update(len(packet.records))
                bar.set_postfix(lost=lost, board_dropped=packet.dropped, refresh=False)
    except KeyboardInterrupt:
        pass
    finally:
        bar.close()
        sock.close()

    if packets == 0:
        logger.warning("nothing arrived; removing the empty {}", path.name)
        path.unlink(missing_ok=True)
        logger.info("check the board is on this network, and that UDP {} is not blocked by a "
                    "firewall", args.port)
        return 1
    elapsed = time.monotonic() - started
    logger.success("{:,} records in {:.1f} s ({:.1f} Hz), {} packets lost, {} bad -> {}", records,
                   elapsed, records / max(elapsed, 1e-9), lost, bad, path)
    return 0


def parse(args: argparse.Namespace) -> int:
    rec = vqflog.load(args.file, progress=True)
    logger.info("{:,} samples, {:.1f} s at {:.1f} Hz, from {}", len(rec.samples), rec.duration_s,
                rec.rate_hz, rec.meta.get("started", "?"))
    logger.info("{} packets, {} lost in the network, {} dropped on the board, {} unreadable",
                rec.packets, rec.lost_packets, rec.board_dropped, rec.skipped)
    fixes = rec.gps_fixes()
    logger.info("GPS: {} distinct fixes; barometer healthy for {:.0f}% of the run", len(fixes),
                100.0 * rec.flag(vqflog.FLAG_BARO_OK).mean())
    if args.csv is not None:
        out = args.csv if args.csv != Path("-") else args.file.with_suffix(".csv")
        vqflog.to_csv(rec, out)
        logger.success("wrote {}", out)
    return 0


def plot(args: argparse.Namespace) -> int:
    rec = vqflog.load(args.file, progress=True)
    out = args.out or args.file.with_suffix(".html")
    out.write_text(vqfplot.build_report(rec, args.file, offline=args.offline), encoding="utf-8")
    logger.success("wrote {}", out)
    if args.open:
        webbrowser.open(out.resolve().as_uri())
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("-v", "--verbose", action="store_true")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("listen", help="receive the UDP stream into a new date-stamped file")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--bind", default="0.0.0.0")
    p.add_argument("--out-dir", type=Path, default=Path("recordings"))
    p.add_argument("--seconds", type=float, default=0, help="stop after this long (default: Ctrl+C)")
    p.set_defaults(func=listen)

    p = sub.add_parser("parse", help="summarise a recording, optionally exporting CSV")
    p.add_argument("file", type=Path)
    p.add_argument("--csv", type=Path, nargs="?", const=Path("-"),
                   help="write every sample as CSV (default name: next to the recording)")
    p.set_defaults(func=parse)

    p = sub.add_parser("plot", help="write an interactive Plotly report")
    p.add_argument("file", type=Path)
    p.add_argument("--out", type=Path)
    p.add_argument("--offline", action="store_true", help="embed plotly.js (4.5 MB) instead of the CDN")
    p.add_argument("--open", action="store_true", help="open the report in a browser")
    p.set_defaults(func=plot)

    args = parser.parse_args()
    configure_logging(args.verbose)
    try:
        return args.func(args)
    except (vqflog.LogFormatError, FileNotFoundError) as error:
        logger.error("{}", error)
        return 1


if __name__ == "__main__":
    sys.exit(main())
