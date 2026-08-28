#!/usr/bin/env python3
"""Generate a random BLE static-random address.

Prints an address in the display format the firmware uses (most-significant
octet first), ready for the BLE_ADDRESS build env var that firmware/build.rs
validates:

  pixi run gen-ble-address
  cd ../firmware && BLE_ADDRESS=FF:C6:A1:53:50:47 cargo run --release

Only needed to pin a board to a fixed address. Left unset, each board
derives its own from its eFuse MAC, so two boards are already distinct out
of the box; `pixi run wio-info` reads back whichever it ended up with.

Static-random (Bluetooth Core Spec): the two most-significant bits of the
address are 1, and the remaining 46 bits are neither all-zero nor all-one.

By default the address is also marked locally administered and unicast in
the IEEE-802 sense (the 0x02 and 0x01 bits of the most-significant octet).
That is not required for a BLE random address - those bits are a MAC concept
- but it is harmless and keeps the address out of real vendor OUI space and
from looking multicast if a host ever surfaces it as a MAC. --no-local skips
it.
"""

import argparse
import secrets

# 46-bit random part all-zero / all-one, which a static-random address must
# avoid. The first byte is masked to the 6 bits below the two fixed MSBs.
_ALL_ZERO = b"\x00\x00\x00\x00\x00\x00"
_ALL_ONE = b"\x3f\xff\xff\xff\xff\xff"


def gen(local: bool = True) -> str:
    while True:
        b = bytearray(secrets.token_bytes(6))
        # b[0] is the most-significant (display-first) octet.
        b[0] |= 0xC0  # static random: top two bits set
        if local:
            b[0] |= 0x02  # locally administered
            b[0] &= 0xFE  # unicast (clear the group bit)
        lower = bytes([b[0] & 0x3F]) + bytes(b[1:])
        if lower not in (_ALL_ZERO, _ALL_ONE):
            return ":".join(f"{x:02X}" for x in b)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("-n", type=int, default=1, metavar="COUNT",
                    help="how many addresses to print (default 1)")
    ap.add_argument("--no-local", action="store_true",
                    help="do not set the locally-administered / unicast bits")
    args = ap.parse_args()

    for _ in range(max(1, args.n)):
        print(gen(local=not args.no_local))


if __name__ == "__main__":
    main()
