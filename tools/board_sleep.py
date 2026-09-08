#!/usr/bin/env python3
"""Put the board into deep sleep now, over its USB port.

    pixi run board-sleep                 # for the board's configured cadence
    pixi run board-sleep --seconds 30

Every other route into deep sleep is indirect: the board sleeps when a
wake-check advertising window expires with nobody connected, so proving on
a bench that sleep works at all means configuring a cadence, disconnecting,
and waiting the window out. This is the direct one.

With no --seconds the board sleeps for its configured wake-check interval,
or a default when sleep mode is off. Either way the board resolves it and
the ack says what it settled on, so what this prints is what the board will
actually do rather than what it was asked for.

Nothing is stored. The board comes back to whatever it was configured for,
including advertising continuously.
"""

import argparse
import sys

import board_link as link


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    ap.add_argument(
        "--seconds",
        type=int,
        default=0,
        help="how long to sleep; 0 (default) uses the board's wake-check interval",
    )
    args = ap.parse_args()

    if args.seconds < 0:
        sys.exit("--seconds cannot be negative")

    ser = link.open_port(args.port)
    if not link.ping(ser):
        sys.exit("no PING reply - is the board running wio-s3-gps firmware?")

    secs = link.sleep_now(ser, args.seconds)
    if secs is None:
        sys.exit("the board did not ack the sleep request")

    print(f"board sleeping for {secs} s")
    # Said plainly because the alternative reading - that this tool knocked
    # the board over - is the obvious one, and it is wrong.
    print("the serial port will disappear now and come back when it wakes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
