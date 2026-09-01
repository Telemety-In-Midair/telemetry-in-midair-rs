#!/usr/bin/env python3
"""Ask the board what it is, over its USB port.

    pixi run wio-info

Prints the firmware's protocol version, the board's BLE address and the
name it advertises under. The address is the one thing about a board that is
otherwise only visible in a single line at boot, and a board that has been
running for a week has scrolled that away - so this asks for it on demand
instead.

Nothing here is a scan: both are properties of the board, and reading them
from the board is what makes it possible to tell two of them apart without
connecting to either. `wio-set name` is what changes the name.
"""

import argparse
import sys

import wio_link as link


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    args = ap.parse_args()

    ser = link.open_port(args.port)
    if not link.ping(ser):
        sys.exit("no PING reply - is the board running wio-s3-gps firmware?")
    print("firmware responding")

    info = link.query_info(ser)
    if info is None:
        sys.exit("the board did not answer the info query")
    address, name = info
    print(f"BLE address {address}")
    if name:
        print(f"advertises as {name}")
    else:
        # Firmware from before names existed answers with the address alone.
        print("no name reported (firmware predates board names)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
