#!/usr/bin/env python3
"""Push a radio config to the board over its USB port.

Sets this node's address without editing a file by hand:

    pixi run wio-config --address 3

Other keys go through --set, repeatably:

    pixi run wio-config --address 3 --set role=tx_only --set interval_s=30

The board applies the config immediately and stores it in two places - the
SD card's RADIO.CFG and a backup in internal flash - so the change survives
a power cycle with or without a card. Nothing is reflashed.
The board reports which stores it reached, and this exits non-zero if it
reached neither.

IMPORTANT - this sends a whole file, not a patch. The firmware parses a
config starting from its own defaults, so any key absent from what is sent
reverts to its default rather than keeping the board's current value. There
is no way to read a config back off the board, so the file this tool starts
from is the whole truth about the resulting settings.

That file is RADIO.example.toml by default, which a test in the proto crate
pins to the firmware defaults - so with no --set the board ends up on stock
settings plus the address given. If the board is running tuned radio
settings, pass the file holding them with --file and edit that instead:

    pixi run wio-config --file mynet.toml --address 3

--file also takes the SD card's own RADIO.CFG, which is the same format.

Comments and the `_description`/`_type` documentation keys are stripped
before sending. They are inert to the firmware, and the reference file is
several times the 1024-byte ceiling the firmware enforces, so this is
what makes a push fit at all. To produce a card-ready RADIO.CFG without
sending anything:

    pixi run wio-config --address 3 --dry-run --save ../RADIO.CFG
"""

import argparse
import sys
from pathlib import Path

import wio_link as link

DEFAULT_CONFIG = link.ROOT / "RADIO.example.toml"

# The firmware's parser accepts 1-255; 0 is reserved to mark a packet as not
# one of ours (see proto/src/lora.rs).
ADDRESS_MIN, ADDRESS_MAX = 1, 255

# Largest config the firmware will take, enforced in three places that must
# agree: the firmware's OP_BEGIN check and the buffer it reads RADIO.CFG
# into at boot.
CONFIG_MAX = 1024


def set_key(text: str, key: str, value: str) -> tuple[str, bool]:
    """Return `text` with `key` set to `value`, and whether it was already there.

    Matching is on the whole key before the '=', so `address` does not also
    hit the `address_description` line that documents it. The quoting style
    of the value being replaced is kept, which keeps a string value quoted
    for editors that read the file as real TOML (the firmware's own parser
    accepts it either way).
    """
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.split("=", 1)[0].strip() != key:
            continue
        old = line.split("=", 1)[1].strip()
        quoted = len(old) >= 2 and old[0] == old[-1] and old[0] in "\"'"
        if quoted and not (value[:1] in "\"'"):
            value = f'"{value}"'
        lines[i] = f"{key} = {value}"
        return "\n".join(lines) + "\n", True
    # Not in the file: append it. A key is valid under any section header,
    # so the end of the file is as good a place as any.
    return text.rstrip("\n") + f"\n\n{key} = {value}\n", False


def strip_docs(text: str) -> str:
    """Drop everything the firmware does not read: comments, blank lines and
    the `<key>_description` / `<key>_type` strings.

    The reference file spends most of its bytes documenting itself for an
    editor, and the firmware ignores every one of those keys. That matters
    because CONFIG_MAX is 1024 bytes on both the begin check and the transfer
    buffer, and the documented file is several times that -
    so this is what makes a push possible at all, not a size optimization.
    """
    out = []
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        key = line.split("=", 1)[0].strip()
        if key.endswith("_description") or key.endswith("_type"):
            continue
        out.append(stripped)
    return "\n".join(out) + "\n"


def parse_set(arg: str) -> tuple[str, str]:
    key, sep, value = arg.partition("=")
    if not sep or not key.strip() or not value.strip():
        raise argparse.ArgumentTypeError(f"--set wants key=value, got {arg!r}")
    return key.strip(), value.strip()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--address", type=int, help=f"node address, {ADDRESS_MIN}-{ADDRESS_MAX}")
    ap.add_argument(
        "--set",
        type=parse_set,
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="set any other config key (repeatable), e.g. --set role=rx_only",
    )
    ap.add_argument(
        "--file",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"config to start from and edit (default: {DEFAULT_CONFIG})",
    )
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    ap.add_argument(
        "--save",
        type=Path,
        help="also write the edited config here, to keep a record of what was sent",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="print the config that would be sent and exit without touching the board",
    )
    args = ap.parse_args()

    if args.address is None and not args.set:
        ap.error("nothing to change: pass --address and/or --set KEY=VALUE")
    if args.address is not None and not ADDRESS_MIN <= args.address <= ADDRESS_MAX:
        ap.error(f"--address {args.address} out of range {ADDRESS_MIN}-{ADDRESS_MAX}")

    if not args.file.is_file():
        sys.exit(f"config file not found: {args.file}")
    text = args.file.read_text()

    edits = ([("address", str(args.address))] if args.address is not None else []) + args.set
    unknown = []
    for key, value in edits:
        text, existed = set_key(text, key, value)
        print(f"  {key} = {value}" + ("" if existed else "   <- not in the file, appended"))
        if not existed:
            unknown.append(key)
    # The firmware ignores keys it does not recognize, so a misspelled one is
    # accepted and then does nothing. Every real key appears in the reference
    # file, so "not in the file" is the only warning available for a typo.
    if unknown and args.file == DEFAULT_CONFIG:
        print(f"warning: {', '.join(unknown)} not in the reference config. "
              "The firmware ignores keys it does not know, so a misspelling "
              "here is accepted and has no effect.")

    # What actually goes on the wire, and what a card copy should hold.
    text = strip_docs(text)
    data = text.encode()
    if len(data) > CONFIG_MAX:
        sys.exit(
            f"config is {len(data)} bytes after stripping comments and "
            f"descriptions, over the firmware's {CONFIG_MAX}-byte limit.\n"
            "Remove keys that are at their default - an absent key keeps it."
        )

    if args.save:
        args.save.write_text(text)
        print(f"saved the config as sent to {args.save}")

    if args.dry_run:
        print(f"--- would send {len(data)}/{CONFIG_MAX} bytes (from {args.file}) ---")
        print(text, end="")
        return 0

    print(f"sending {len(data)}/{CONFIG_MAX} bytes based on {args.file}")
    ser = link.open_port(args.port)
    if not link.ping(ser):
        sys.exit("no PING reply - is the board running wio-s3-gps firmware?")
    print("board responding")

    try:
        link.send_bulk(ser, link.KIND_TOML, data, version=0)
    except (TimeoutError, RuntimeError) as e:
        # The board parses the file at the end of the transfer, so a value it
        # rejects surfaces here rather than at begin - and looks identical to
        # a link fault unless it is spelled out.
        sys.exit(f"\nconfig push failed: {e}\n"
                 "A NAK at this stage usually means the file itself was "
                 "rejected: a value out of range or a string that is not one "
                 "of the choices. Check it with --dry-run.")

    # The board logs "config applied, node N, <where it was saved>" once it has
    # parsed and adopted the file, which is the only read-back there is: an
    # ack proves the bytes arrived, this proves which address is now live and
    # whether it will still be there after a power cycle.
    applied = link.read_console(ser, "config applied", timeout=3.0)
    if not applied:
        print("config accepted (no 'config applied' line seen; it may have "
              "scrolled past - the transfer itself was acked)")
        return 0

    print(applied)
    if "NOT SAVED" in applied:
        print("\nWARNING: the config is live but reached neither the SD card "
              "nor the flash backup.\nIt will be lost on the next power "
              "cycle, reverting to firmware defaults.\nCheck that the card is "
              "seated and readable, or that flash is not worn out.")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
