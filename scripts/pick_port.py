# PlatformIO pre-upload hook: choose the board's native-USB serial port.
#
# Auto-detection picks the first port it finds, which on macOS is usually a
# Bluetooth device -- the upload then fails with "No serial data received".
# The FeatherS3 enumerates as an ESP32-S3 USB JTAG/serial unit (VID 0x303A),
# so match on that and fall back to any usbmodem device.
from __future__ import annotations

import glob

Import("env")  # noqa: F821 -- provided by PlatformIO/SCons

ESP32_VID = "303A"


def pick_port() -> str | None:
    try:
        from serial.tools import list_ports
        for port in list_ports.comports():
            if port.hwid and ESP32_VID in port.hwid.upper():
                return port.device
    except ImportError:
        pass
    candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
    return candidates[0] if candidates else None


port: str | None = pick_port()
if port:
    env.Replace(UPLOAD_PORT=port, MONITOR_PORT=port)  # noqa: F821
    print("pick_port: using " + port)
else:
    print("pick_port: no ESP32 USB port found -- is the board plugged in?")
