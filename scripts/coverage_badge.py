#!/usr/bin/env python3
"""Renders .github/badges/coverage.svg from the Cobertura report gcovr writes.

    python3 scripts/coverage_badge.py [coverage.xml] [out.svg]

Reads `line-rate` off the report's root element rather than taking a second
output format from gcovr, so `task coverage` needs no extra flag. Stdlib only:
this runs in CI before anything but gcovr is installed.

The SVG is written only when its contents change, so the CI job that commits it
does nothing on a run where coverage held steady.
"""

from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ElementTree

# Shields' own flat palette, so the badge sits next to the workflow badge
# without looking like it came from somewhere else.
THRESHOLDS: list[tuple[float, str]] = [
    (95.0, "#4c1"),      # brightgreen
    (90.0, "#97ca00"),   # green
    (80.0, "#a4a61d"),   # yellowgreen
    (70.0, "#dfb317"),   # yellow
    (60.0, "#fe7d37"),   # orange
    (0.0, "#e05d44"),    # red
]

# Verdana at 11px, averaged. Only needs to be close enough that the text sits
# inside its half of the badge.
CHAR_WIDTH = 6.6
PADDING = 10.0


def colour_for(percent: float) -> str:
    for floor, colour in THRESHOLDS:
        if percent >= floor:
            return colour
    return THRESHOLDS[-1][1]


def line_coverage(report_path: str) -> float:
    root = ElementTree.parse(report_path).getroot()
    rate = root.get("line-rate")
    if rate is None:
        raise ValueError(f"{report_path} has no line-rate attribute")
    return float(rate) * 100.0


def render(label: str, value: str, colour: str) -> str:
    label_width = round(len(label) * CHAR_WIDTH + PADDING * 2)
    value_width = round(len(value) * CHAR_WIDTH + PADDING * 2)
    total = label_width + value_width
    # Text is positioned in tenths of a pixel and scaled down, which is how
    # shields keeps glyphs crisp at this size.
    label_mid = label_width * 5
    value_mid = (label_width + value_width / 2) * 10
    return f"""<svg xmlns="http://www.w3.org/2000/svg" \
xmlns:xlink="http://www.w3.org/1999/xlink" width="{total}" height="20" \
role="img" aria-label="{label}: {value}">
  <title>{label}: {value}</title>
  <linearGradient id="s" x2="0" y2="100%">
    <stop offset="0" stop-color="#bbb" stop-opacity=".1"/>
    <stop offset="1" stop-opacity=".1"/>
  </linearGradient>
  <clipPath id="r"><rect width="{total}" height="20" rx="3" fill="#fff"/></clipPath>
  <g clip-path="url(#r)">
    <rect width="{label_width}" height="20" fill="#555"/>
    <rect x="{label_width}" width="{value_width}" height="20" fill="{colour}"/>
    <rect width="{total}" height="20" fill="url(#s)"/>
  </g>
  <g fill="#fff" text-anchor="middle" font-family="Verdana,Geneva,DejaVu Sans,sans-serif" \
text-rendering="geometricPrecision" font-size="110">
    <text x="{label_mid}" y="150" fill="#010101" fill-opacity=".3" \
transform="scale(.1)" textLength="{(label_width - PADDING * 2) * 10:.0f}">{label}</text>
    <text x="{label_mid}" y="140" transform="scale(.1)" \
textLength="{(label_width - PADDING * 2) * 10:.0f}">{label}</text>
    <text x="{value_mid:.0f}" y="150" fill="#010101" fill-opacity=".3" \
transform="scale(.1)" textLength="{(value_width - PADDING * 2) * 10:.0f}">{value}</text>
    <text x="{value_mid:.0f}" y="140" transform="scale(.1)" \
textLength="{(value_width - PADDING * 2) * 10:.0f}">{value}</text>
  </g>
</svg>
"""


def main(argv: list[str]) -> int:
    report = argv[1] if len(argv) > 1 else ".pio/coverage/coverage.xml"
    out = argv[2] if len(argv) > 2 else ".github/badges/coverage.svg"
    if not os.path.exists(report):
        print(f"coverage_badge: {report} not found -- run `task coverage` first")
        return 1

    percent = line_coverage(report)
    svg = render("coverage", f"{percent:.1f}%", colour_for(percent))

    os.makedirs(os.path.dirname(out), exist_ok=True)
    if os.path.exists(out) and open(out, encoding="utf-8").read() == svg:
        print(f"coverage_badge: {percent:.1f}% -- unchanged")
        return 0
    with open(out, "w", encoding="utf-8") as f:
        f.write(svg)
    print(f"coverage_badge: {percent:.1f}% -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
