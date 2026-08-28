#!/usr/bin/env python3
"""Push a firmware image to the board over its USB port, without a reflash.

    pixi run wio-ota

With no arguments this builds the firmware in ../firmware, converts it to an
ESP-IDF application image, and streams it into whichever OTA slot the board
is not running from. The board verifies the CRC, points the bootloader at
the new slot and reboots into it.

To send an image you already have:

    pixi run wio-ota --image firmware.bin

That file must be an *application image*, not the ELF cargo produces -
`espflash save-image --chip esp32s3 --flash-size 16mb <elf> <bin>` is the
conversion, and is what --image-less runs for you.

What makes this safe to do to a board you cannot reach: the image goes into
the slot that is not executing, nothing is pointed at it until the whole
transfer has been received and its CRC checked, and the firmware marks
itself confirmed only once it has booted far enough to run. A bootloader
built with rollback enabled reverts to the previous slot if that never
happens, so an image that cannot start costs a reboot rather than a board.

The transfer shares its protocol and its one-at-a-time guarantee with the
config push, so this cannot run while `wio-config` is mid-transfer, over
USB or over BLE.
"""

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

import wio_link as link

FIRMWARE = link.ROOT / "firmware"
ELF = FIRMWARE / "target" / "xtensa-esp32s3-none-elf" / "release" / "wio-s3-gps"

# An OTA slot on this board's partition table (see firmware/partitions.csv). The
# firmware refuses an image larger than the slot at OP_BEGIN; checking here
# too means a mistake costs a message rather than a whole upload.
SLOT_MAX = 0x3F0000


def build_image(out: Path) -> Path:
    """Build the firmware and convert it to an application image."""
    for tool in ("cargo", "espflash"):
        if shutil.which(tool) is None:
            sys.exit(f"{tool} not found on PATH; build the image yourself and pass --image")
    print(f"building {FIRMWARE}")
    subprocess.run(["cargo", "build", "--release"], cwd=FIRMWARE, check=True)
    if not ELF.is_file():
        sys.exit(f"expected {ELF} after the build, but it is not there")
    print("converting to an application image")
    subprocess.run(
        [
            "espflash", "save-image",
            "--chip", "esp32s3",
            "--flash-size", "16mb",
            str(ELF), str(out),
        ],
        check=True,
    )
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--image",
        type=Path,
        help="application image to send (default: build one from ../firmware)",
    )
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    ap.add_argument(
        "--keep",
        action="store_true",
        help="keep the built image next to the ELF instead of a temporary path",
    )
    args = ap.parse_args()

    if args.image is not None:
        image = args.image
        if not image.is_file():
            sys.exit(f"image not found: {image}")
    else:
        out = ELF.with_suffix(".ota.bin") if args.keep else Path("/tmp/wio-s3-gps.ota.bin")
        image = build_image(out)

    data = image.read_bytes()
    if not data:
        sys.exit(f"{image} is empty")
    if len(data) > SLOT_MAX:
        sys.exit(f"{image} is {len(data)} bytes, larger than the {SLOT_MAX}-byte OTA slot")
    # An ESP-IDF application image starts with the 0xE9 magic. An ELF
    # ("\x7fELF") is the mistake this is here to name, because the board
    # would take it, write it, and then fail to boot it.
    if data[0] != 0xE9:
        sys.exit(
            f"{image} does not look like an ESP-IDF application image "
            f"(first byte {data[0]:#04x}, expected 0xe9).\n"
            "If this is the ELF, convert it: espflash save-image --chip esp32s3 "
            "--flash-size 16mb <elf> <bin>"
        )

    print(f"sending {len(data)} bytes from {image}")
    ser = link.open_port(args.port)
    if not link.ping(ser):
        sys.exit("no PING reply - is the board running wio-s3-gps firmware?")
    print("board responding")

    started = time.monotonic()
    try:
        link.send_bulk(
            ser, link.KIND_OTA, data, version=0,
            hint="\nA board flashed with a single-app partition table has no "
                 "second slot to write into, and says so at begin. Reflash it "
                 "over USB once (cargo run --release in firmware/) to move it onto "
                 "the two-slot table.",
        )
    except (TimeoutError, RuntimeError) as e:
        sys.exit(f"\nfirmware push failed: {e}")
    print(f"transfer complete in {time.monotonic() - started:.1f}s")

    # The board reboots about half a second after acking the final step.
    # Its USB Serial/JTAG port is part of the chip, not a separate bridge,
    # so the reset takes the port down with it and the OS hands back a new
    # one - the handle open here goes stale rather than going quiet.
    print("rebooting into the new image")
    ser.close()
    booted = wait_for_boot(args.port)
    if booted is None:
        print("image installed, but the board did not come back with a boot "
              "line within 20 s.\nIf it does not, a bootloader built with "
              "rollback reverts to the previous slot on its own.")
        return 2
    for line in booted:
        print(line)
    return 0


def wait_for_boot(port: str | None, timeout: float = 20.0) -> list[str] | None:
    """Re-open the port after the reboot and read the board's boot lines.

    Returns the version line and the OTA slot line, or None if the board did
    not announce itself in time.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.5)
        try:
            ser = link.open_port(port)
        except (OSError, SystemExit):
            continue
        version = link.read_console(ser, "wio-s3-gps v", timeout=max(1.0, deadline - time.monotonic()))
        if version is None:
            ser.close()
            continue
        lines = [version]
        slot = link.read_console(ser, "ota: booted", timeout=3.0)
        if slot:
            lines.append(slot)
        ser.close()
        return lines
    return None


if __name__ == "__main__":
    sys.exit(main())
