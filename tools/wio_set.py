#!/usr/bin/env python3
"""Write one board setting over USB, the same way the app writes it over BLE.

    pixi run wio-set ble-off 30      # BLE modem down 30 s between windows
    pixi run wio-set ble-off 0       # never take it down (the default)
    pixi run wio-set adv-window 10
    pixi run wio-set sleep 60
    pixi run wio-set gps-sleep 1

The board clamps every value and the ack carries what it actually stored,
so what this prints is the setting the board is running - not the one it
was asked for.

`ble-off` is the power one. BLE measures 71 mA of this board's 126 and
cannot be reduced while the controller exists, so the firmware drops the
whole stack between advertising windows instead. During the off period the
board keeps beaconing over LoRa, keeps tracking and keeps logging; what it
cannot do is be connected to. That is the same trade deep sleep makes, and
the ceiling is set the same way for the same reason.
"""

import argparse
import struct
import sys

import wio_link as link

# name -> (config id, value width in bytes)
SETTINGS = {
    "rail": (0x10, 1),
    "radio-standby": (0x11, 1),
    "gps-sleep": (0x12, 1),
    "sleep": (0x13, 4),
    "adv-window": (0x14, 4),
    "ble-off": (0x16, 4),
}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("setting", choices=sorted(SETTINGS), help="which setting to write")
    ap.add_argument("value", type=int, help="the value; the board clamps it")
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    args = ap.parse_args()

    if args.value < 0:
        sys.exit("value cannot be negative")

    cfg_id, width = SETTINGS[args.setting]
    if width == 1:
        if args.value > 1:
            sys.exit(f"{args.setting} takes 0 or 1")
        payload = bytes([args.value])
    else:
        payload = struct.pack("<I", args.value)

    ser = link.open_port(args.port)
    if not link.ping(ser):
        sys.exit("no PING reply - is the board running wio-s3-gps firmware?")

    reply = link.set_config(ser, cfg_id, payload)
    if reply is None:
        sys.exit("the board did not ack the write")
    status, value = reply
    if status != 0:
        sys.exit(f"the board rejected the write (status {status})")

    if len(value) >= 4:
        applied = struct.unpack("<I", value[:4])[0]
    elif value:
        applied = value[0]
    else:
        applied = args.value
    if applied != args.value:
        print(f"{args.setting} = {applied} (asked {args.value}, clamped)")
    else:
        print(f"{args.setting} = {applied}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
