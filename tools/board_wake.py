#!/usr/bin/env python3
"""Call a stored board over LoRa, through a board that is awake.

    pixi run board-wake --target 3             # wake node 3 into idle
    pixi run board-wake --target 3 --tracking  # and have it come up tracking
    pixi run board-wake --target 0             # every sleeping node in earshot

The board on this USB port does the calling: it sends a burst of wake
frames behind a preamble sized to the target's sentry cycle, listens for
the target's answer between them, and gives up after three. The target -
stored, in deep sleep, its radio listening on its own duty cycle - wakes,
boots, answers with a ping, and can then be connected to over BLE.

What this prints is only that the board accepted the request. How the
burst went is on the calling board's console: `wake: node 3 answered` or
`wake: no answer from node 3`. Both boards must share the [wake] section
of their radio config, since the preamble one sends is computed from it,
and the calling board has to be in a mode that keeps its radio up
(tracking or listening) with a role that transmits.
"""

import argparse
import sys

import board_link as link

# `midair_proto::lora::WAKE_FLAG_TRACKING`.
FLAG_TRACKING = 1 << 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    ap.add_argument(
        "--target",
        type=int,
        required=True,
        help="the address to wake, 1-255, or 0 for every sleeping node in earshot",
    )
    ap.add_argument(
        "--tracking",
        action="store_true",
        help="ask the woken board to come up tracking rather than idle",
    )
    args = ap.parse_args()
    if not 0 <= args.target <= 255:
        sys.exit("--target takes 0-255")

    ser = link.open_port(args.port)
    if not link.ping(ser):
        sys.exit("no PING reply - is the board running wio-s3-gps firmware?")

    flags = FLAG_TRACKING if args.tracking else 0
    reply = link.set_config(ser, link.CFG_IDS["wake"], bytes([args.target, flags]))
    if reply is None:
        sys.exit("the board did not ack the request")
    status, _ = reply
    if status != 0:
        sys.exit(f"the board refused the request (status {status})")

    who = "every sleeping node in earshot" if args.target == 0 else f"node {args.target}"
    print(f"calling {who}{' into tracking' if args.tracking else ''}")
    print("watch this board's console for the answer: wake: node N answered")
    return 0


if __name__ == "__main__":
    sys.exit(main())
