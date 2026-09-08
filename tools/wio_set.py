#!/usr/bin/env python3
"""Write one board setting over USB, the same way the app writes it over BLE.

    pixi run wio-set mode tracking   # gps up, beacons out, card logging
    pixi run wio-set mode idle       # reachable, gps in backup, radio down
    pixi run wio-set mode stored     # ack, then deep sleep on the cadence
    pixi run wio-set mode listening  # gps and receiver up, nothing sent, ble up
    pixi run wio-set ble-off 30      # BLE modem down 30 s between windows
    pixi run wio-set ble-off 0       # never take it down (the default)
    pixi run wio-set ble-on 20       # BLE up 20 s between those, while tracking
    pixi run wio-set adv-window 10   # each wake check advertises 10 s
    pixi run wio-set sleep 60
    pixi run wio-set idle-timeout 600
    pixi run wio-set idle-timeout 0  # idle never stores itself (the default)
    pixi run wio-set gps-sleep 1
    pixi run wio-set name sky-1      # advertises as ws3gps-sky-1
    pixi run wio-set name ""         # back to the address-derived name

`mode` is the one that means something on its own; the rest are knobs it
scopes. Tracking and listening survive a power cycle, so a board put down in
either comes back in it - which is the point, since a brownout on the object
is exactly when it must. Everything else comes back reachable, and stays so
unless an idle timeout has been set.

`mode stored` drops the USB port, exactly as `wio-sleep` does. That is the
command working.

The board clamps every value and the ack carries what it actually stored,
so what this prints is the setting the board is running - not the one it
was asked for.

`name` is what the board calls itself in a scan list: it advertises as
`<prefix>-<label>`, and an unnamed board falls back to the tail of its BLE
address so two boards out of the same box are still told apart. The label
survives a deep sleep and a flat cell, because it is stored with the
settings that decide reachability rather than on the card.

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

# name -> (config id, value width in bytes; 0 means an ASCII label). The
# ids come from the protocol crate through wire_consts.json: the five
# durations under their config-file names with the `_s` dropped and `_`
# for `-`, the rest by their own names.
SETTINGS = {
    "radio-standby": (link.CFG_IDS["radio_standby"], 1),
    "gps-sleep": (link.CFG_IDS["gps_sleep"], 1),
    "mode": (link.CFG_IDS["mode"], 1),
    "name": (link.CFG_IDS["name"], 0),
}
SETTINGS.update({
    name.removesuffix("_s").replace("_", "-"): (knob["id"], 4)
    for name, knob in link.KNOBS.items()
})

# `midair_proto::ble::NAME_LABEL_MAX` and the charset `valid_label` takes.
# Checked here as well as on the board so a typo comes back as a message
# rather than as a rejected write with a status number.
NAME_MAX = 15
NAME_CHARS = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_"
# `midair_proto::ble::NAME_PREFIX`: the firmware puts it in front of every
# label, so what is printed below is what a scan list will show.
NAME_PREFIX = "ws3gps"

# The wire values of `midair_proto::ble::Mode`. Named rather than numbered
# on the command line because "2" is not a thing anyone should have to
# remember about their tracker.
MODES = {"stored": 0, "idle": 1, "tracking": 2, "listening": 3}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("setting", choices=sorted(SETTINGS), help="which setting to write")
    ap.add_argument(
        "value",
        help="the value; the board clamps it. For `mode`: "
        + "/".join(sorted(MODES))
        + '. For `name`: a label, or "" to clear it',
    )
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    args = ap.parse_args()

    cfg_id, width = SETTINGS[args.setting]
    if args.setting == "name":
        label = args.value
        if len(label.encode()) > NAME_MAX:
            sys.exit(f"a name is at most {NAME_MAX} characters")
        bad = sorted(set(label) - set(NAME_CHARS))
        if bad:
            sys.exit(
                "a name takes letters, digits, - and _ only; "
                f"remove {' '.join(repr(c) for c in bad)}"
            )
        value = label
    elif args.setting == "mode":
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

    if width == 0:
        payload = value.encode()
    elif width == 1:
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

    if args.setting == "name":
        # The ack carries the stored length, not the label: an ack has four
        # value bytes and no name fits in one.
        if value:
            print(f"name = {value} (advertises as {NAME_PREFIX}-{value})")
        else:
            print("name cleared; the board advertises under its address")
        print("the new name goes out with the next advertising window")
        return 0

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
