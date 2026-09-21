"""Builds a self-contained Plotly report from a Recording.

One HTML page: the time series stacked on a shared, zoomable time axis, the
horizontal track, and an amplitude spectrum of the raw accelerometer and
gyroscope. The dashboard's spectrum panel shows 1.28 s of the last moment; this
shows the whole run.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from vqflog import FLAG_BARO_OK, FLAG_GPS_FIX, Recording

# The dashboard's axis colours, so a trace means the same thing in both places.
AXIS_COLORS = {"x": "#ef6b73", "y": "#4fd49c", "z": "#5aa9ff"}
DARK = {"template": "plotly_dark", "paper_bgcolor": "#0f141b", "plot_bgcolor": "#0f141b"}


def _lines(fig: go.Figure, row: int, x: np.ndarray, data: np.ndarray, names: str, unit: str) -> None:
    for column, axis in enumerate(names):
        fig.add_trace(
            go.Scattergl(x=x, y=data[:, column], name=axis, mode="lines", legendgroup=str(row),
                         line={"width": 1, "color": AXIS_COLORS.get(axis)},
                         hovertemplate=f"%{{y:.4g}} {unit}"),
            row=row, col=1)
    fig.update_yaxes(title_text=unit, row=row, col=1)


def time_series(rec: Recording) -> go.Figure:
    s, t = rec.samples, rec.t_s
    rows = ["accel", "gyro", "mag", "attitude", "velocity", "height"]
    fig = make_subplots(rows=len(rows), cols=1, shared_xaxes=True, vertical_spacing=0.03,
                        subplot_titles=[
                            "Accelerometer (raw)", "Gyroscope (raw)", "Magnetometer (calibrated)",
                            "Orientation (roll, pitch, yaw)", "Velocity (ENU)",
                            "Height: filter vs barometer"])
    _lines(fig, 1, t, s["accel"], "xyz", "m/s²")
    _lines(fig, 2, t, s["gyro"], "xyz", "rad/s")
    _lines(fig, 3, t, s["mag"], "xyz", "µT")
    for column, name in enumerate(("roll", "pitch", "yaw")):
        fig.add_trace(go.Scattergl(x=t, y=rec.euler_deg()[:, column], name=name, mode="lines",
                                   line={"width": 1}), row=4, col=1)
    fig.update_yaxes(title_text="deg", row=4, col=1)
    _lines(fig, 5, t, s["vel"], "xyz", "m/s")
    fig.add_trace(go.Scattergl(x=t, y=s["pos"][:, 2], name="filter up", mode="lines",
                               line={"width": 1.4}), row=6, col=1)
    baro = rec.flag(FLAG_BARO_OK)
    if baro.any():
        fig.add_trace(go.Scattergl(x=t[baro], y=s["baro_height_m"][baro], name="baro height",
                                   mode="lines", line={"width": 1, "dash": "dot"}), row=6, col=1)
    fig.update_yaxes(title_text="m", row=6, col=1)
    fig.update_xaxes(title_text="seconds since the first sample", row=len(rows), col=1)
    fig.update_layout(height=190 * len(rows) + 120, hovermode="x unified", showlegend=False,
                      margin={"t": 50, "b": 40}, **DARK)
    return fig


def track(rec: Recording) -> go.Figure | None:
    """The filter's horizontal path against the raw GPS fixes. Only the filter's
    is drawn without a fix: with none there is nothing to compare it to."""
    s = rec.samples
    fig = go.Figure()
    fig.add_trace(go.Scattergl(x=s["pos"][:, 0], y=s["pos"][:, 1], mode="lines", name="filter",
                               line={"width": 1.5}))
    fixes = rec.gps_fixes()
    if len(fixes):
        lat, lon = s["gps_lat"][fixes], s["gps_lon"][fixes]
        # Local tangent plane about the first fix: good to centimetres over the
        # few hundred metres this is meant for, and needs no map tiles.
        east = (lon - lon[0]) * 111_320.0 * np.cos(np.radians(lat[0]))
        north = (lat - lat[0]) * 110_540.0
        fig.add_trace(go.Scattergl(x=east, y=north, mode="markers", name="GPS fixes",
                                   marker={"size": 4}))
    elif not rec.flag(FLAG_GPS_FIX).any():
        fig.add_annotation(text="no GPS fix in this recording", showarrow=False,
                           xref="paper", yref="paper", x=0.5, y=0.5)
    fig.update_xaxes(title_text="east (m)")
    fig.update_yaxes(title_text="north (m)", scaleanchor="x", scaleratio=1)
    fig.update_layout(title="Track (local ENU; the filter's frame is anchored at the first fix)",
                      height=520, **DARK)
    return fig


def amplitude_spectrum(x: np.ndarray, fs: float, dt: np.ndarray, size: int = 256) -> tuple:
    """Welch-averaged amplitude spectrum of each column of `x`.

    Hann-windowed segments with 50 % overlap, mean removed, and scaled so a
    sine of amplitude A peaks at A -- the same convention as the dashboard's
    panel, so a resonance reads the same in both. Segments spanning a gap in
    the samples are discarded: the step at the seam reads as broadband energy.
    """
    window = np.hanning(size)
    gain = window.sum() / 2.0
    bad = np.concatenate([[False], dt > 2.5 / fs])
    total = np.zeros((size // 2 + 1, x.shape[1]))
    used = 0
    for start in range(0, len(x) - size + 1, size // 2):
        if bad[start + 1:start + size].any():
            continue
        segment = x[start:start + size]
        segment = (segment - segment.mean(axis=0)) * window[:, None]
        total += np.abs(np.fft.rfft(segment, axis=0)) / gain
        used += 1
    if used == 0:
        return None, None, 0
    return np.fft.rfftfreq(size, 1.0 / fs), total / used, used


def spectra(rec: Recording) -> go.Figure | None:
    if len(rec.t_s) < 512:
        return None
    dt = np.diff(rec.t_s)
    fs = 1.0 / float(np.median(dt))
    fig = make_subplots(rows=1, cols=2, subplot_titles=("Accelerometer", "Gyroscope"),
                        shared_yaxes=False)
    for col, (name, unit) in enumerate((("accel", "m/s²"), ("gyro", "rad/s")), start=1):
        freq, amp, used = amplitude_spectrum(rec.samples[name].astype(np.float64), fs, dt)
        if freq is None:
            return None
        for i, axis in enumerate("xyz"):
            fig.add_trace(go.Scatter(x=freq[1:], y=amp[1:, i], name=f"{name} {axis}", mode="lines",
                                     line={"width": 1, "color": AXIS_COLORS[axis]},
                                     legendgroup=axis, showlegend=col == 1),
                          row=1, col=col)
        fig.update_yaxes(type="log", title_text=unit, row=1, col=col)
        fig.update_xaxes(title_text=f"Hz  ({used} averages, {freq[1]:.2f} Hz bins)", row=1, col=col)
    fig.update_layout(title=f"Amplitude spectrum, sampled at {fs:.1f} Hz", height=420, **DARK)
    return fig


def summary_html(rec: Recording, source: Path) -> str:
    start = datetime.fromtimestamp(float(rec.unix_time[0])).astimezone()
    loss = rec.lost_packets * 10 + rec.board_dropped
    return (
        f"<h2>{source.name}</h2><p>Started {start:%Y-%m-%d %H:%M:%S %Z}. "
        f"{len(rec.samples):,} samples over {rec.duration_s:.1f} s, arriving at "
        f"{rec.rate_hz:.1f} Hz. {rec.packets:,} packets; "
        f"{rec.lost_packets} lost in the network and {rec.board_dropped} records dropped on the "
        f"board (about {loss} samples in all).</p>"
    )


def build_report(rec: Recording, source: Path, offline: bool = False) -> str:
    """The whole page as a string. Plotly.js comes from the CDN unless `offline`
    embeds it (about 4.5 MB), which is what to use with no internet."""
    figures = [f for f in (time_series(rec), track(rec), spectra(rec)) if f is not None]
    parts = []
    for index, fig in enumerate(figures):
        include = ("cdn" if not offline else True) if index == 0 else False
        parts.append(fig.to_html(full_html=False, include_plotlyjs=include))
    return (
        "<!doctype html><html><head><meta charset='utf-8'>"
        f"<title>{source.stem}</title>"
        "<style>body{background:#0f141b;color:#d6dde6;font:14px system-ui,sans-serif;"
        "margin:0 auto;max-width:1400px;padding:16px}</style></head><body>"
        + summary_html(rec, source) + "".join(parts) + "</body></html>"
    )
