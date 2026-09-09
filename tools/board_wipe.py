#!/usr/bin/env python3
"""Reset a board to what it was out of the box.

    pixi run board-wipe            # settings, name and config backup, then a reboot
    pixi run board-wipe --flash    # every byte of flash, then a rebuild and a reflash

Without `--flash` this is one command over USB. The board erases its
settings record, its name and its radio config backup from flash, drops the
copy of the settings it keeps in RTC RAM, acks, and restarts. The firmware,
the OTA slots and the card are untouched, so the board comes back running
the same image on its defaults.

With `--flash` the whole part is erased with `espflash erase-flash` - the
partition table, both app slots, `otadata`, `nvs`, `phy_init` and whatever
an earlier firmware left anywhere else - and the firmware is rebuilt and
flashed again. That is the reset for a board that has been through several
firmwares, and the only one that removes what this firmware does not know
about.

Why a command at all: `espflash erase-flash` on its own did not clear the
name. The settings live in RTC RAM as well as flash, and RTC RAM survives
every reset short of a power cycle - including the reset the flashing tool
issues - so the erased board came up on its RTC copy and wrote it straight
back into the flash that had just been cleared. The firmware now drops that
copy on any boot that is not a deep-sleep wake, which makes an erase stick;
the USB command is the same result without a reflash.

Neither form touches the SD card. A `RADIO.CFG` there is read at the next
boot and wins over the erased backup, `[power]` section included. Pull the
card, or delete the file, to reset that too.
"""

import argparse
import shutil
import subprocess
import sys
import time

import board_link as link

FIRMWARE = link.ROOT / "firmware"
ELF = FIRMWARE / "target" / "xtensa-esp32s3-none-elf" / "release" / "wio-s3-gps"


def wipe_over_usb(port: str | None) -> int:
    ser = link.open_port(port)
    if not link.ping(ser):
        sys.exit("no PING reply - is the board running wio-s3-gps firmware?")
    info = link.query_info(ser)
    if info is not None:
        address, name = info
        print(f"board {address}" + (f", advertising as {name}" if name else ""))
    print("wiping settings, name and config backup...")
    ok = link.wipe(ser)
    if ok is None:
        sys.exit("the board did not ack the wipe")
    if not ok:
        print("the board could not erase its flash records; the RTC copy is "
              "dropped, so the next boot is on whatever flash still holds")
    print("the board is restarting; the port drops and comes back on defaults")
    return 0 if ok else 1


def erase_and_reflash(port: str | None) -> int:
    for tool in ("cargo", "espflash"):
        if shutil.which(tool) is None:
            sys.exit(f"{tool} not found on PATH")
    port_args = ["--port", port] if port else []
    print("erasing the whole flash...")
    subprocess.run(["espflash", "erase-flash", *port_args], check=True)
    print("building the firmware...")
    subprocess.run(["cargo", "build", "--release"], cwd=FIRMWARE, check=True)
    if not ELF.exists():
        sys.exit(f"build produced no ELF at {ELF}")
    # The same arguments the cargo runner flashes with, minus the monitor.
    # `otadata` was erased with the rest; naming it keeps the two in step
    # should the erase ever be narrowed.
    print("flashing...")
    subprocess.run([
        "espflash", "flash", *port_args,
        "--chip", "esp32s3", "--flash-size", "16mb",
        "--partition-table", str(FIRMWARE / "partitions.csv"),
        "--target-app-partition", "ota_0", "--erase-parts", "otadata",
        str(ELF),
    ], check=True)
    # The board resets into the new image with its RTC RAM intact, and
    # drops the copy there because flash holds nothing: that line on the
    # console is the erase having stuck.
    print("done; the board boots on its defaults (watch for "
          "'nvs: nothing stored' on the console)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    ap.add_argument(
        "--flash",
        action="store_true",
        help="erase every byte of flash, then rebuild and reflash the firmware",
    )
    args = ap.parse_args()
    if args.flash:
        return erase_and_reflash(args.port)
    return wipe_over_usb(args.port)


if __name__ == "__main__":
    sys.exit(main())
