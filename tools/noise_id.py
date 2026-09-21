#!/usr/bin/env python3
"""Records a stationary run from the board and derives the noise parameters.

    tools/noise_id.py record --seconds 600 --out noise.jsonl [--host vqf-gps.local]
    tools/noise_id.py analyze noise.jsonl

The board must sit perfectly still for the whole recording. `record` streams the
dashboard's telemetry frames over the WebSocket (stdlib only, no dependencies);
`analyze` needs numpy.

The frames arrive at TELEMETRY_INTERVAL_MS (20 Hz), a decimation of the 200 Hz
estimator. Decimating does not change a white-noise density or a random walk,
so the Allan analysis is unaffected; it only limits the shortest tau to 0.05 s.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import socket
import struct
import sys
import time
from collections.abc import Iterator
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # numpy is imported lazily, so `record` needs only the stdlib
    import numpy as np


# ── Minimal WebSocket client ─────────────────────────────────────────────────
def ws_connect(host: str, port: int = 81) -> tuple[socket.socket, bytes]:
    sock = socket.create_connection((host, port), timeout=10)
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall((f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
                  f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
                  f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
    reply = b""
    while b"\r\n\r\n" not in reply:
        chunk = sock.recv(1024)
        if not chunk:
            raise ConnectionError("closed during handshake")
        reply += chunk
    if b" 101 " not in reply.split(b"\r\n")[0]:
        raise ConnectionError(reply.split(b"\r\n")[0].decode())
    # Anything after the header already belongs to the frame stream.
    return sock, reply.split(b"\r\n\r\n", 1)[1]


def ws_messages(sock: socket.socket, buffered: bytes) -> Iterator[str]:
    """Yields text messages. Server frames are unmasked; handles fragments."""
    buf, message = buffered, b""

    def need(n: int) -> None:
        nonlocal buf
        while len(buf) < n:
            chunk = sock.recv(65536)
            if not chunk:
                raise ConnectionError("socket closed")
            buf += chunk

    while True:
        need(2)
        fin, opcode, length = buf[0] & 0x80, buf[0] & 0x0F, buf[1] & 0x7F
        offset = 2
        if length == 126:
            need(4)
            length, offset = struct.unpack(">H", buf[2:4])[0], 4
        elif length == 127:
            need(10)
            length, offset = struct.unpack(">Q", buf[2:10])[0], 10
        need(offset + length)
        payload, buf = buf[offset:offset + length], buf[offset + length:]
        if opcode == 0x8:
            raise ConnectionError("server closed the socket")
        if opcode == 0x9:  # ping -> pong (masked, as clients must)
            mask = os.urandom(4)
            sock.sendall(bytes([0x8A, 0x80 | len(payload)]) + mask +
                         bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))
            continue
        if opcode in (0x1, 0x0):
            message += payload
            if fin:
                yield message.decode()
                message = b""


# ── Recording ────────────────────────────────────────────────────────────────
def record(args: argparse.Namespace) -> None:
    sock, rest = ws_connect(args.host)
    start = time.time()
    count = 0
    with open(args.out, "w") as out:
        for text in ws_messages(sock, rest):
            frame = json.loads(text)
            out.write(json.dumps({
                "t": frame["t"], "a": frame["imu"]["a"], "g": frame["imu"]["g"],
                "rest": frame["att"]["rest"], "pa": frame["baro"]["pa"],
                "alt": frame["baro"]["alt"], "tc": frame["baro"]["tc"],
                "bok": frame["baro"]["ok"], "iok": frame["health"]["imu"],
                "fix": frame["gps"]["fix"], "n": frame["gps"]["n"],
                "enu": frame["gps"]["enu"], "enuok": frame["gps"]["enuok"],
                "sat": frame["gps"]["sat"],
            }) + "\n")
            count += 1
            elapsed = time.time() - start
            if count % 200 == 0:
                print(f"\r{elapsed:6.0f}/{args.seconds} s  {count} frames", end="", flush=True)
            if elapsed >= args.seconds:
                break
    print(f"\nwrote {count} frames to {args.out}")


# ── Analysis ─────────────────────────────────────────────────────────────────
def allan(x: np.ndarray, tau0: float, ms: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Overlapping Allan deviation of rate data x at cluster sizes ms."""
    import numpy as np
    theta = np.concatenate([[0.0], np.cumsum(x)]) * tau0  # integrated signal
    taus, devs = [], []
    for m in ms:
        if 2 * m + 1 > len(theta):
            break
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        taus.append(m * tau0)
        devs.append(np.sqrt(0.5 * np.mean(d * d)) / (m * tau0))
    return np.array(taus), np.array(devs)


def noise_terms(x: np.ndarray, tau0: float) -> tuple[float, float]:
    """(N, K): white-noise density from sigma*sqrt(tau) at short tau, random
    walk from sigma*sqrt(3/tau) at the longest taus. K is an upper bound: any
    flicker floor in the record reads as random walk."""
    import numpy as np
    top = len(x) // 2
    ms = np.unique(np.logspace(0, np.log10(max(top, 2)), 60).astype(int))
    taus, devs = allan(x, tau0, ms)
    short = (taus >= 2 * tau0) & (taus <= 1.0)
    long_ = taus >= taus.max() / 3
    n = np.median(devs[short] * np.sqrt(taus[short]))
    k = np.median(devs[long_] * np.sqrt(3.0 / taus[long_]))
    return n, k


def analyze(args: argparse.Namespace) -> None:
    import numpy as np
    rows: list[dict] = [json.loads(line) for line in open(args.file)]
    t = np.array([r["t"] for r in rows], dtype=float) / 1000.0
    keep = np.concatenate([[True], np.diff(t) > 0])  # drop duplicate timestamps
    rows = [r for r, k in zip(rows, keep) if k]
    t = t[keep]
    tau0 = float(np.median(np.diff(t)))
    print(f"{len(rows)} frames, {t[-1] - t[0]:.0f} s, tau0 = {tau0:.4f} s "
          f"({1 / tau0:.1f} Hz), gaps > 0.5 s: {int(np.sum(np.diff(t) > 0.5))}")

    a = np.array([r["a"] for r in rows])
    g = np.array([r["g"] for r in rows])
    rest = np.mean([r["rest"] for r in rows])
    print(f"rest detector true for {100 * rest:.0f}% of the run "
          f"(anything much below ~100% means the board moved)")
    print(f"|a| mean {np.linalg.norm(a, axis=1).mean():.4f} m/s^2, "
          f"gyro mean {np.array2string(g.mean(axis=0), precision=5)} rad/s")

    print("\nAllan-derived terms (worst axis is what we report):")
    result: dict[str, tuple[float, ...]] = {}
    for name, data, unit in (("accel", a, "m/s^2"), ("gyro", g, "rad/s")):
        terms = [noise_terms(data[:, i], tau0) for i in range(3)]
        for axis, (n, k) in zip("xyz", terms):
            print(f"  {name} {axis}: N = {n:.5f} {unit}/sqrt(Hz)   K = {k:.6f} {unit}/sqrt(s)")
        result[name] = (float(np.mean([n for n, _ in terms])),
                        float(np.mean([k for _, k in terms])))
        print(f"  {name} mean: N = {result[name][0]:.5f}   K = {result[name][1]:.6f}")

    # Barometer. Only samples where the reading changed are independent: the
    # sensor is polled slower than the telemetry, so frames repeat values.
    alt = np.array([r["alt"] for r in rows])
    pa = np.array([r["pa"] for r in rows])
    tc = np.array([r["tc"] for r in rows])
    ok = np.array([r["bok"] for r in rows], dtype=bool)
    if ok.mean() > 0.5:
        # High-pass by first difference: immune to the slow drift.
        sample_sigma = np.std(np.diff(alt[ok])) / np.sqrt(2)
        pa_sigma = np.std(np.diff(pa[ok])) / np.sqrt(2)
        print(f"\nbarometer: per-sample sigma {sample_sigma:.4f} m (pressure {pa_sigma:.2f} Pa, "
              f"1 Pa quantisation); altitude range {np.ptp(alt[ok]):.2f} m, "
              f"temperature {tc[ok].min():.1f}..{tc[ok].max():.1f} C")
        # Drift as a random walk: variance of altitude change grows as K^2 * tau.
        lags = np.unique(np.logspace(np.log10(5), np.log10(len(alt) // 4), 25).astype(int))
        k_est = [np.std(alt[ok][l:] - alt[ok][:-l]) / np.sqrt(l * tau0) for l in lags]
        long_k = float(np.median(k_est[len(k_est) // 2:]))
        print(f"  altitude random walk K = {long_k:.4f} m/sqrt(s) "
              f"(from increments at tau {lags[len(lags) // 2] * tau0:.0f}..{lags[-1] * tau0:.0f} s)")
        result["baro"] = (float(sample_sigma), long_k)
    else:
        print("\nbarometer: not healthy for most of the run, skipped")

    # GPS: one sample per fix (n increments), only when the fix was valid.
    gps = [(r["n"], r["enu"]) for r in rows if r["fix"] and r["enuok"]]
    fixes: dict[int, list[float]] = {}
    for n, enu in gps:
        fixes[n] = enu
    if len(fixes) >= 60:
        e = np.array(list(fixes.values()))
        s = e.std(axis=0, ddof=1)
        print(f"\nGPS: {len(e)} distinct fixes, scatter east {s[0]:.2f} m, north {s[1]:.2f} m, "
              f"up {s[2]:.2f} m (RMS horizontal {np.hypot(s[0], s[1]):.2f} m)")
        result["gps"] = tuple(float(v) for v in s)
    else:
        print(f"\nGPS: only {len(fixes)} fixes with a valid position; not enough to "
              f"estimate scatter (need 60+). Leave the GPS values alone.")

    print("\n" + json.dumps(result))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    rec = sub.add_parser("record")
    rec.add_argument("--seconds", type=int, default=600)
    rec.add_argument("--out", default="noise.jsonl")
    rec.add_argument("--host", default="vqf-gps.local")
    rec.set_defaults(func=record)
    ana = sub.add_parser("analyze")
    ana.add_argument("file")
    ana.set_defaults(func=analyze)
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    sys.exit(main())
