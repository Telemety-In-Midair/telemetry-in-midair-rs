#!/usr/bin/env python3
"""Read the board's event log over USB: what it wrote down about itself.

    pixi run board-log             # every record, oldest first
    pixi run board-log --last 10   # the ten newest
    pixi run board-log --clear     # erase the log

The board keeps a ring of records in its own flash (proto/src/evlog.rs):
one per boot with the reset reason, one per panic with the message and the
location, one per stall with the task and the phase it stopped in, and one
for each fault worth a line - a radio that restarted underneath the
firmware, a BLE controller that would not come up, a park that did not
finish before a sleep. A board found dark in a field tells its story here,
and the story survives a reflash.

Each line is `#seq boot +uptime kind: text`: the record's sequence number
across every boot, the boot it was written in, the seconds since that boot,
and the text. A panic's text ends with the program counters of its
backtrace, which `xtensa-esp32s3-elf-addr2line -e <the elf>` turns into
lines.
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
    ap.add_argument("--last", type=int, metavar="N", help="only the N newest records")
    ap.add_argument("--clear", action="store_true", help="erase the log instead of reading it")
    args = ap.parse_args()

    ser = link.open_port(args.port)
    if not link.ping(ser):
        sys.exit("no PING reply - is the board running wio-s3-gps firmware?")

    if args.clear:
        reply = link.read_evlog(ser, link.EVLOG_ERASE_INDEX)
        if reply is None:
            sys.exit("the board did not answer the erase")
        print("event log erased")
        return 0

    first = link.read_evlog(ser, 0)
    if first is None:
        sys.exit("the board did not answer (firmware predates the event log?)")
    count, _, record = first
    if count == 0 or record is None:
        print("the event log is empty")
        return 0

    wanted = count if args.last is None else min(count, max(args.last, 0))
    records = [record]
    for index in range(1, wanted):
        reply = link.read_evlog(ser, index)
        if reply is None:
            print(f"  (no reply for record {index}, stopping)", file=sys.stderr)
            break
        _, _, record = reply
        if record is None:
            break
        records.append(record)

    print(f"{count} record(s) on the board, showing {len(records)}, oldest first")
    for r in reversed(records):
        print(format_record(r))
    return 0


def format_record(r: dict) -> str:
    return f"#{r['seq']:<6} b{r['boot']:<4} +{r['uptime_s']:>6}s  {r['kind']:<8} {r['text']}"


if __name__ == "__main__":
    sys.exit(main())
