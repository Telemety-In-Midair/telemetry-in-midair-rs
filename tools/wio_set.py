#!/usr/bin/env python3
"""Write one board setting over USB, the same way the app writes it over BLE.

    pixi run wio-set mode tracking   # gps up, beacons out, card logging
    pixi run wio-set mode idle       # reachable, gps in backup, radio down
    pixi run wio-set mode stored     # ack, then deep sleep on the cadence
    pixi run wio-set ble-off 30      # BLE modem down 30 s between windows
    pixi run wio-set ble-off 0       # never take it down (the default)
    pixi run wio-set adv-window 10
    pixi run wio-set sleep 60
    pixi run wio-set idle-timeout 600
    pixi run wio-set gps-sleep 1

`mode` is the one that means something on its own; the rest are knobs it
scopes. Tracking is the only mode that survives a power cycle, so a board
put down in it comes back tracking - which is the point, since a brownout on
the object is exactly when it must. Everything else comes back reachable for
one idle timeout and then stores itself.

`mode stored` drops the USB port, exactly as `wio-sleep` does. That is the
command working.

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
    "mode": (0x17, 1),
    "idle-timeout": (0x18, 4),
}

# The wire values of `midair_proto::ble::Mode`. Named rather than numbered
# on the command line because "2" is not a thing anyone should have to
# remember about their tracker.
MODES = {"stored": 0, "idle": 1, "tracking": 2}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("setting", choices=sorted(SETTINGS), help="which setting to write")
    ap.add_argument(
        "value",
        help="the value; the board clamps it. For `mode`: "
        + "/".join(sorted(MODES)),
    )
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    args = ap.parse_args()

    cfg_id, width = SETTINGS[args.setting]
    if args.setting == "mode":
        if args.value not in MODES:
            sys.exit(f"mode takes one of {', '.join(sorted(MODES))}")
        value = MODES[args.value]
    else:
        try:
            value = int(args.value)
        except ValueError:
            sys.exit(f"{args.setting} takes a number")
        if value < 0:
            sys.exit("value cannot be negative")

    if width == 1:
        if args.setting != "mode" and value > 1:
            sys.exit(f"{args.setting} takes 0 or 1")
        payload = bytes([value])
    else:
        payload = struct.pack("<I", value)

    ser = link.open_port(args.port)
    if not link.ping(ser):
        sys.exit("no PING reply - is the board running wio-s3-gps firmware?")

    reply = link.set_config(ser, cfg_id, payload)
    if reply is None:
        sys.exit("the board did not ack the write")
    status, echoed = reply
    if status != 0:
        sys.exit(f"the board rejected the write (status {status})")

    if len(echoed) >= 4:
        applied = struct.unpack("<I", echoed[:4])[0]
    elif echoed:
        applied = echoed[0]
    else:
        applied = value

    if args.setting == "mode":
        names = {v: k for k, v in MODES.items()}
        print(f"mode = {names.get(applied, applied)}")
        if applied == MODES["stored"]:
            print("the port will disappear as the board sleeps")
        return 0
    if applied != value:
        print(f"{args.setting} = {applied} (asked {value}, clamped)")
    else:
        print(f"{args.setting} = {applied}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
