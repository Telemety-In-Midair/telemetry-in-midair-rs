#!/usr/bin/env python3
"""Shared host-side transport for talking to the board over USB.

The firmware exposes the same bulk-transfer protocol it serves over BLE on
its USB Serial/JTAG console, framed with the midair-proto link framing (see
proto/src/link.rs, module `usb`). Framing, port detection and retry logic
live here so the tools on top stay short.

This outlived the two-MCU board it was written for. The framed protocol was
the ESP32-C6's UART link to the WIO-E5 as well as its USB console; a single
Wio-S3 has nothing to link to, but the host still speaks the console half of
it, so the framing survives with only the transport gone.

The console shares this port, so firmware text is interleaved with the reply
frames; the frame parser resyncs past it by sync byte and CRC.
"""

import struct
import time
from pathlib import Path

import serial
from serial.tools import list_ports

# -- Wire protocol constants ------------------------------------------------
#
# Read from wire_consts.json beside this file, which the protocol crate
# prints (cargo run --example wire_consts --features std in proto/) and a
# test there holds current. Nothing here restates a byte the crate owns.

import json

_WIRE = json.loads((Path(__file__).resolve().parent / "wire_consts.json").read_text())

SYNC = _WIRE["link"]["sync"]
MAX_PAYLOAD = _WIRE["link"]["max_payload"]
RESP_ACK = _WIRE["link"]["ack"]

USB_PING = _WIRE["usb"]["ping"]
USB_BULK = _WIRE["usb"]["bulk"]
USB_BULK_ACK = _WIRE["usb"]["bulk_ack"]
USB_INFO = _WIRE["usb"]["info"]
USB_SLEEP = _WIRE["usb"]["sleep"]
USB_CFG = _WIRE["usb"]["cfg"]
USB_WIPE = _WIRE["usb"]["wipe"]
USB_EVLOG = _WIRE["usb"]["evlog"]

# The event log's record, as the board stores it and sends it back.
EVLOG_RECORD_LEN = _WIRE["evlog"]["record_len"]
EVLOG_HEADER_LEN = _WIRE["evlog"]["header_len"]
EVLOG_TEXT_MAX = _WIRE["evlog"]["text_max"]
EVLOG_MAGIC = _WIRE["evlog"]["magic"]
EVLOG_ERASE_INDEX = _WIRE["evlog"]["erase_index"]
EVLOG_KINDS = {v: k for k, v in _WIRE["evlog"]["kinds"].items()}

OP_BEGIN = _WIRE["bulk"]["begin"]
OP_DATA = _WIRE["bulk"]["data"]
OP_END = _WIRE["bulk"]["end"]
OP_ABORT = _WIRE["bulk"]["abort"]
KIND_TOML = _WIRE["bulk"]["kind_toml"]
KIND_OTA = _WIRE["bulk"]["kind_ota"]
ACK_ID_BULK = _WIRE["bulk"]["ack_id"]
# Data bytes per OP_DATA.
DATA_CHUNK = _WIRE["bulk"]["data_max"]
# Largest config the board reads or takes in a transfer.
CONFIG_MAX = _WIRE["bulk"]["config_max"]

ACK_OK = _WIRE["ack"]["ok"]
STATUS_NAMES = {
    _WIRE["ack"]["ok"]: "OK",
    _WIRE["ack"]["unknown_id"]: "unknown id",
    _WIRE["ack"]["bad_value"]: "bad value",
    _WIRE["ack"]["board_error"]: "the board could not carry it out (flash?)",
    _WIRE["ack"]["bad_state"]: "bad state (a transfer is already active, or on the other transport?)",
}

# Config characteristic ids: the fixed ones, plus the five durations by
# their config-file names, each with its bounds and what a zero means.
CFG_IDS = _WIRE["cfg"]
KNOBS = {k["name"]: k for k in _WIRE["knobs"]}

# Espressif USB vendor id, used to auto-detect the port.
ESPRESSIF_VID = 0x303A

# How many times to retry a bulk op before giving up (transport hiccups,
# and the apply that OP_END triggers taking longer than one timeout).
ATTEMPTS = 10

# Repo root, resolved from this file so a task works from any CWD.
ROOT = Path(__file__).resolve().parent.parent


def status_str(status: int) -> str:
    return f"{status} ({STATUS_NAMES.get(status, 'unknown')})"


def crc8(data: bytes) -> int:
    """CRC-8/ITU (poly 0x07, init 0) over the given bytes."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, payload: bytes) -> bytes:
    plen = len(payload)
    if plen > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = bytes([cmd]) + payload
    return bytes([SYNC, plen & 0xFF, (plen >> 8) & 0xFF]) + body + bytes([crc8(body)])


class FrameParser:
    """Byte-at-a-time parser matching proto::link::FrameParser."""

    def __init__(self):
        self.state = "sync"
        self.buf = bytearray()
        self.expected = 0

    def feed(self, byte: int):
        """Return (cmd, payload) on a complete, CRC-valid frame, else None."""
        if self.state == "sync":
            if byte == SYNC:
                self.state = "lenlo"
        elif self.state == "lenlo":
            self.expected = byte
            self.state = "lenhi"
        elif self.state == "lenhi":
            self.expected |= byte << 8
            if self.expected > MAX_PAYLOAD:
                self.state = "sync"
            else:
                self.expected += 1  # + cmd byte
                self.buf = bytearray()
                self.state = "data"
        elif self.state == "data":
            self.buf.append(byte)
            if len(self.buf) >= self.expected:
                self.state = "crc"
        elif self.state == "crc":
            self.state = "sync"
            if crc8(bytes(self.buf)) == byte:
                return self.buf[0], bytes(self.buf[1:])
        return None


def read_frame(ser: serial.Serial, wanted: set, timeout: float):
    """Read until a frame whose cmd is in `wanted` arrives, or timeout.

    Returns (frame_or_None, info). `info` describes what was seen while
    waiting - bytes read, any other frames, and a sample of console text -
    so a caller can explain *why* it is retrying instead of just "no ack".
    """
    parser = FrameParser()
    deadline = time.monotonic() + timeout
    nbytes = 0
    others: list[int] = []
    text = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        nbytes += len(chunk)
        for byte in chunk:
            got = parser.feed(byte)
            if got is not None:
                if got[0] in wanted:
                    return got, ""
                others.append(got[0])
            elif 32 <= byte < 127 and len(text) < 96:
                text.append(byte)
    info = f"{nbytes} B in {timeout:.0f}s"
    if others:
        info += " other frames " + ",".join(f"0x{c:02x}" for c in others)
    if text:
        info += f" console {bytes(text).decode('ascii', 'replace')!r}"
    return None, info


def read_console(ser: serial.Serial, match: str, timeout: float) -> str | None:
    """Watch the console for a line containing `match`, ignoring frames.

    The firmware's own status lines reach this port, so a tool can confirm
    what the board did rather than only that the transfer was acked.
    """
    deadline = time.monotonic() + timeout
    line = bytearray()
    while time.monotonic() < deadline:
        for byte in ser.read(64):
            if byte in (0x0A, 0x0D):
                text = bytes(line).decode("ascii", "replace")
                line.clear()
                if match in text:
                    return text.strip()
            elif 32 <= byte < 127:
                if len(line) < 200:
                    line.append(byte)
            else:
                # A frame byte mid-line: the line is not console text.
                line.clear()
    return None


def open_port(port: str | None) -> serial.Serial:
    if port is None:
        for p in list_ports.comports():
            if p.vid == ESPRESSIF_VID:
                port = p.device
                print(f"auto-detected board port {port} ({p.description})")
                break
    if port is None:
        raise SystemExit("no --port given and no Espressif USB serial port found")
    # USB CDC-ACM ignores the baud rate; the value is a placeholder.
    return serial.Serial(port, 115200, timeout=0.05)


def bulk_op(ser: serial.Serial, op_payload: bytes, timeout: float = 3.0):
    """Send one bulk op and return (status, next_seq) from the board's ack."""
    ser.reset_input_buffer()
    ser.write(build_frame(USB_BULK, op_payload))
    ser.flush()
    frame, info = read_frame(ser, {USB_BULK_ACK}, timeout)
    if frame is None:
        raise TimeoutError(f"no ack from the board ({info})")
    _, payload = frame
    if len(payload) < 2 or payload[0] != ACK_ID_BULK:
        raise ValueError(f"unexpected ack payload {payload.hex()}")
    status = payload[1]
    next_seq = int.from_bytes(payload[2:6], "little") if len(payload) >= 6 else 0
    return status, next_seq


def bulk_op_retry(ser: serial.Serial, op_payload: bytes, timeout: float, label: str,
                  attempts: int = ATTEMPTS) -> tuple[int, int]:
    """`bulk_op` with retries on transport hiccups (a lost/garbled ack).

    Retrying is safe: the firmware de-duplicates by sequence number,
    so re-sending the same frame either re-acks (already applied) or applies
    it now. A returned protocol status (incl. a NAK) is passed straight back
    to the caller; only transport failures (timeout / bad ack frame) retry.
    """
    last: Exception = TimeoutError(f"{label}: no attempts made")
    for attempt in range(1, attempts + 1):
        try:
            return bulk_op(ser, op_payload, timeout)
        except (TimeoutError, ValueError) as e:
            last = e
            if attempt >= attempts:
                break
            print(f"\n  {label}: {e}; retry {attempt}/{attempts - 1}", flush=True)
            ser.reset_input_buffer()
            time.sleep(0.1 * attempt)
    raise TimeoutError(f"{label}: gave up after {attempts} tries ({last})") from last


def send_end(ser: serial.Serial, attempts: int = ATTEMPTS) -> None:
    """Finalize the transfer (OP_END). Returns on success; raises on a
    definitive failure.

    The end step erases and CRC-checks flash, so its ack can be lost more
    easily than a data ack - hence the retries on top of the transport ones.
    The firmware keeps the transfer open when it does not confirm, so
    re-sending OP_END is safe.
    """
    for attempt in range(1, attempts + 1):
        last = attempt >= attempts
        try:
            status, _ = bulk_op(ser, bytes([OP_END]), timeout=8.0)
        except (TimeoutError, ValueError) as e:
            if last:
                raise TimeoutError(f"end: no reply after {attempts} tries ({e})") from e
            print(f"\n  end: {e}; retry {attempt}/{attempts - 1}", flush=True)
            ser.reset_input_buffer()
            time.sleep(0.2 * attempt)
            continue
        if status == ACK_OK:
            return
        # 0x11: the round-trip did not complete and the transfer stayed
        # open, so retrying is safe.
        if status == 0x11 and not last:
            print(f"\n  end: {status_str(status)}; retry {attempt}/{attempts - 1}", flush=True)
            time.sleep(0.3)
            continue
        # 0x12 = no active transfer. On a retry this means a previous
        # OP_END already finalized it (its ack was lost) - the work is
        # committed, so treat as success. On the first try it is a real error
        # (state vanished without finishing).
        if status == 0x12 and attempt > 1:
            print("\n  end: board reports the transfer already finalized; "
                  "treating as success")
            return
        raise RuntimeError(f"end/verify failed: status {status_str(status)}")
    raise TimeoutError(f"end: never confirmed after {attempts} tries")


def ping(ser: serial.Serial) -> bool:
    ser.reset_input_buffer()
    ser.write(build_frame(USB_PING, b""))
    ser.flush()
    frame, _ = read_frame(ser, {RESP_ACK}, 2.0)
    return frame is not None and frame[1][:1] == bytes([USB_PING])


def query_ble_address(ser: serial.Serial, timeout: float = 2.0) -> str | None:
    """Ask the board for its BLE address on demand; return "FF:.." or None.

    The reply is [USB_INFO, addr[0]..addr[5]] with the address most-
    significant octet first, so it prints directly.
    """
    info = query_info(ser, timeout)
    return None if info is None else info[0]


def query_info(
    ser: serial.Serial, timeout: float = 2.0
) -> tuple[str, str] | None:
    """Ask the board for its BLE address and its name in one round trip.

    The reply is [USB_INFO, addr[0]..addr[5], name...], the address most-
    significant octet first and the name in the form it advertises under.
    Firmware that predates the name sends the address alone, which reads
    back as an empty name rather than as a failure.
    """
    ser.reset_input_buffer()
    ser.write(build_frame(USB_INFO, b""))
    ser.flush()
    frame, _ = read_frame(ser, {RESP_ACK}, timeout)
    if frame is None:
        return None
    payload = frame[1]
    if len(payload) < 7 or payload[0] != USB_INFO:
        return None
    address = ":".join(f"{b:02X}" for b in payload[1:7])
    name = payload[7:].decode("ascii", "replace")
    return address, name


def sleep_now(ser: serial.Serial, secs: int, timeout: float = 2.0) -> int | None:
    """Tell the board to deep sleep for `secs`; return the seconds it agreed to.

    0 means "for the configured wake-check interval", which the board
    resolves (and falls back to a default for when sleep mode is off) - so
    the ack is the answer, not an echo of the request. `None` means the
    board never acked.

    The port disappears immediately afterwards. That is the command working:
    deep sleep takes the USB device down with the rest of the chip, and it
    comes back as a fresh device when the board wakes.
    """
    ser.reset_input_buffer()
    ser.write(build_frame(USB_SLEEP, struct.pack("<I", secs)))
    ser.flush()
    frame, _ = read_frame(ser, {RESP_ACK}, timeout)
    if frame is None:
        return None
    payload = frame[1]
    if len(payload) < 3 or payload[0] != USB_SLEEP:
        return None
    return int(struct.unpack("<H", payload[1:3])[0])


def wipe(ser: serial.Serial, timeout: float = 3.0) -> bool | None:
    """Tell the board to forget everything it stores about itself.

    The settings record, the name and the radio config backup in flash, and
    the RTC RAM copy of the settings that a reset alone never clears. The
    board acks with whether the flash records were erased and then
    restarts, which drops the USB device; the port coming back is the board
    on its defaults. `None` means the board never acked.

    The card is not touched. A `RADIO.CFG` on it is read again at the boot
    that follows, so a board that keeps its radio config there keeps it.
    """
    ser.reset_input_buffer()
    ser.write(build_frame(USB_WIPE, b""))
    ser.flush()
    frame, _ = read_frame(ser, {RESP_ACK}, timeout)
    if frame is None:
        return None
    payload = frame[1]
    if len(payload) < 2 or payload[0] != USB_WIPE:
        return None
    return payload[1] == 1


def decode_evlog_record(raw: bytes) -> dict | None:
    """One event log record as a dict, or None for bytes that are not one.

    Layout (proto/src/evlog.rs): magic u16, kind u8, text length u8, seq
    u32, uptime seconds u32, boot u16, reserved u16, then the text, then a
    crc32 the board already checked before sending.
    """
    if len(raw) < EVLOG_RECORD_LEN:
        return None
    magic, kind, tlen, seq, uptime, boot = struct.unpack_from("<HBBIIH", raw, 0)
    if magic != EVLOG_MAGIC or tlen > EVLOG_TEXT_MAX:
        return None
    text = raw[EVLOG_HEADER_LEN:EVLOG_HEADER_LEN + tlen].decode("utf-8", "replace")
    return {
        "seq": seq,
        "boot": boot,
        "uptime_s": uptime,
        "kind": EVLOG_KINDS.get(kind, f"kind{kind}"),
        "text": text,
    }


def read_evlog(ser: serial.Serial, index: int, timeout: float = 2.0):
    """Read the `index`-th newest event log record, 0 being the newest.

    Returns `(count, index, record)` where `record` is None past the end of
    the log, or None when the board never acked. `EVLOG_ERASE_INDEX` erases
    the log instead and comes back with a count of 0.
    """
    ser.reset_input_buffer()
    ser.write(build_frame(USB_EVLOG, struct.pack("<H", index)))
    ser.flush()
    frame, _ = read_frame(ser, {RESP_ACK}, timeout)
    if frame is None:
        return None
    payload = frame[1]
    if len(payload) < 5 or payload[0] != USB_EVLOG:
        return None
    count, echoed = struct.unpack_from("<HH", payload, 1)
    record = decode_evlog_record(payload[5:]) if len(payload) >= 5 + EVLOG_RECORD_LEN else None
    return count, echoed, record


def set_config(ser: serial.Serial, cfg_id: int, value: bytes,
               timeout: float = 2.0):
    """Write one settings id and return `(status, value_bytes)` from the ack.

    The board clamps, so the returned value is what it stored rather than
    what was asked for. Status 0 is OK; anything else is a rejection and the
    setting did not change. `None` means the board never acked.

    Same wire format and the same handler as a BLE config write, so this is
    not a bench-only side door - it is the same operation the app performs.
    """
    ser.reset_input_buffer()
    ser.write(build_frame(USB_CFG, bytes([cfg_id, len(value)]) + value))
    ser.flush()
    frame, _ = read_frame(ser, {RESP_ACK}, timeout)
    if frame is None:
        return None
    payload = frame[1]
    # [USB_CFG, ack id, ack status, ack value...]
    if len(payload) < 4 or payload[0] != USB_CFG:
        return None
    return payload[2], bytes(payload[3:])


def send_bulk(ser: serial.Serial, kind: int, data: bytes, version: int = 0,
              progress: bool = True, hint: str = "") -> None:
    """Run a whole begin/data/end bulk transfer, aborting on failure.

    Raises SystemExit with a diagnosis on a protocol rejection, or lets
    TimeoutError/RuntimeError out after a best-effort abort. `hint` is
    appended to the "board is not answering" diagnosis, for advice that only
    makes sense for one kind of transfer.
    """
    import zlib

    total = len(data)
    crc = zlib.crc32(data) & 0xFFFFFFFF
    begin = bytes([OP_BEGIN, kind]) + total.to_bytes(4, "little") \
        + crc.to_bytes(4, "little") + version.to_bytes(2, "little")
    try:
        status, _ = bulk_op_retry(ser, begin, timeout=3.0, label="begin")
        if status != ACK_OK:
            msg = f"begin rejected: status {status_str(status)}"
            if status in (0x10, 0x11):
                msg += (
                    "\nthe firmware did not accept the transfer. Is the board running "
                    "working firmware and not held in reset?"
                ) + hint
            raise SystemExit(msg)
        seq = 0
        sent = 0
        for off in range(0, total, DATA_CHUNK):
            chunk = data[off:off + DATA_CHUNK]
            op = bytes([OP_DATA]) + seq.to_bytes(2, "little") + chunk
            status, next_seq = bulk_op_retry(ser, op, timeout=3.0, label=f"chunk seq {seq}")
            if status != ACK_OK:
                raise SystemExit(f"\nchunk seq {seq} rejected: status {status_str(status)}")
            seq = next_seq & 0xFFFF
            sent += len(chunk)
            if progress:
                print(f"\r  {sent}/{total} bytes ({100 * sent // total}%)", end="", flush=True)
        if progress:
            print()
        send_end(ser)
    except (TimeoutError, RuntimeError, SystemExit):
        # Best-effort abort so the board does not sit waiting for the rest.
        try:
            bulk_op(ser, bytes([OP_ABORT]), timeout=1.0)
        except (TimeoutError, ValueError):
            pass
        raise
