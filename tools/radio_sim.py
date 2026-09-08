#!/usr/bin/env python3
"""Simulate the telemetry-in-midair radio network: hop clock, slot turns,
the SX1262 receive path, the GPS UART, the SD card stalls and the BLE
notifier, for a handful of boards over a few minutes.

The point is overlap. Two boards that beacon every second on a shared
hop channel either take turns or collide, and the firmware's loop has a
few places where one subsystem's blocking holds another's timing. This
models each of them from the firmware's own numbers (the hop clock is a
line-for-line port of proto/src/hop.rs, checked against vectors that
crate prints) and reports what a receiver and a phone would see.

    python3 radio_sim.py --selftest          # port matches the Rust crate
    python3 radio_sim.py                     # every scenario, summary table
    python3 radio_sim.py --json out.json     # ...and the numbers as JSON
    python3 radio_sim.py --gantt B --variant legacy   # a mermaid gantt

Only the standard library is used, so it runs wherever python3 does.
"""

from __future__ import annotations

import argparse
import bisect
import heapq
import json
import math
import os
import random
import sys
from dataclasses import dataclass, field

U32 = 0xFFFFFFFF

# ---------------------------------------------------------------------------
# Port of proto/src/hop.rs
# ---------------------------------------------------------------------------

STRATUM_GPS = 0
STRATUM_MAX = 15
AGE_STEP_MS = 10 * 60 * 1000
SLOT_BITS = 20
SLOT_MASK = (1 << SLOT_BITS) - 1
GUARD_MAX_MS = 100


def mix(x: int) -> int:
    h = (x ^ 0x5BD1E995) & U32
    h ^= h >> 16
    h = (h * 0x85EBCA6B) & U32
    h ^= h >> 13
    h = (h * 0xC2B2AE35) & U32
    h ^= h >> 16
    return h | 1


def xorshift(s: int) -> int:
    s ^= (s << 13) & U32
    s ^= s >> 17
    s ^= (s << 5) & U32
    return s & U32


def permutation(cycle: int, n: int) -> list[int]:
    n = max(n, 1)
    out = list(range(n))
    s = mix(cycle & U32)
    for i in range(n - 1, 0, -1):
        s = xorshift(s)
        j = s % (i + 1)
        out[i], out[j] = out[j], out[i]
    return out


def div_euclid(a: int, b: int) -> int:
    return a // b  # python floors, which is euclid for b > 0


def rem_euclid(a: int, b: int) -> int:
    return a % b


@dataclass
class Plan:
    channels: int
    step_khz: int
    center_hz: int
    dwell_ms: int
    sub_slots: int = 1

    @staticmethod
    def new(channels, step_khz, center_hz, dwell_ms, unit_ms) -> "Plan":
        p = Plan(channels, step_khz, center_hz, dwell_ms, 1)
        p.sub_slots = min(max(p.window_ms() // max(unit_ms, 1), 1), 255)
        return p

    def channel_hz(self, index: int) -> int:
        n = max(self.channels, 1)
        i = min(index, n - 1)
        half_step = self.step_khz * 500
        off = (2 * i - (n - 1)) * half_step
        return min(max(self.center_hz + off, 0), U32)

    def cycle(self) -> int:
        return max(self.channels, 1)

    def index_for_slot(self, slot: int) -> int:
        n = self.cycle()
        slot &= SLOT_MASK
        return permutation(slot // n, max(self.channels, 1))[slot % n]

    def guard_ms(self) -> int:
        return min(self.dwell_ms // 5, GUARD_MAX_MS)

    def window_ms(self) -> int:
        return max(self.dwell_ms - 2 * self.guard_ms(), 0)

    def fits(self, airtime_ms: int) -> bool:
        return airtime_ms <= self.window_ms()

    def start_range_ms(self, airtime_ms: int) -> tuple[int, int]:
        guard = self.guard_ms()
        latest = max(max(self.dwell_ms - (guard + airtime_ms), 0), guard)
        return guard, latest

    def sub_slot_ms(self) -> int:
        return self.window_ms() // max(self.sub_slots, 1)

    def turn_slot(self, address: int, interval_slots: int) -> int:
        return max(address - 1, 0) % max(interval_slots, 1)

    def sub_slot_of(self, address: int, interval_slots: int) -> int:
        n = max(interval_slots, 1)
        return (max(address - 1, 0) // n) % max(self.sub_slots, 1)

    def turns(self, interval_slots: int) -> int:
        return max(self.sub_slots, 1) * max(interval_slots, 1)

    def start_range_for(self, address: int, interval_slots: int, airtime_ms: int) -> tuple[int, int]:
        lo, hi = self.start_range_ms(airtime_ms)
        if self.sub_slots <= 1:
            return lo, hi
        width = self.sub_slot_ms()
        turn_lo = lo + self.sub_slot_of(address, interval_slots) * width
        slack = max(width - airtime_ms, 0)
        turn_hi = turn_lo + slack // 2
        a = min(turn_lo, hi)
        return a, max(min(turn_hi, hi), a)


@dataclass
class SyncWord:
    slot: int = 0
    stratum: int = 0
    phase: int = 0

    def to_u32(self) -> int:
        return (((self.slot & SLOT_MASK) << 12) | ((self.stratum & 0xF) << 8) | (self.phase & 0xFF)) & U32

    @staticmethod
    def from_u32(v: int) -> "SyncWord":
        return SyncWord(v >> 12, (v >> 8) & 0xF, v & 0xFF)


class Clock:
    def __init__(self, dwell_ms: int, now_ms: int, seed: int):
        s = mix(seed & U32)
        self.dwell_ms = max(dwell_ms, 1)
        self.origin_ms = int(now_ms)
        self.origin_slot = s & SLOT_MASK
        self.base_stratum = STRATUM_MAX
        self.synced_ms = None
        self.rng = xorshift(s)

    def slot(self, now_ms: int) -> int:
        elapsed = div_euclid(int(now_ms) - self.origin_ms, self.dwell_ms)
        return (self.origin_slot + elapsed) & SLOT_MASK

    def phase_ms(self, now_ms: int) -> int:
        return rem_euclid(int(now_ms) - self.origin_ms, self.dwell_ms)

    def stratum(self, now_ms: int) -> int:
        if self.synced_ms is None:
            return STRATUM_MAX
        aged = self.base_stratum + max(int(now_ms) - self.synced_ms, 0) // AGE_STEP_MS
        return min(aged, STRATUM_MAX)

    def synced(self) -> bool:
        return self.synced_ms is not None

    def from_gps(self, now_ms: int) -> bool:
        return self.stratum(now_ms) == STRATUM_GPS

    def discipline_gps(self, tod_ms: int, at_ms: int):
        slot = tod_ms // self.dwell_ms
        phase = tod_ms % self.dwell_ms
        self.set_origin(at_ms, phase, slot)
        self.base_stratum = STRATUM_GPS
        self.synced_ms = int(at_ms)

    def offer(self, word: SyncWord, src: int, my_addr: int, tx_start_ms: int, now_ms: int):
        mine = self.stratum(now_ms)
        if mine == STRATUM_GPS:
            return None
        theirs = min(word.stratum, STRATUM_MAX)
        if not (theirs < mine or (theirs == mine and src < my_addr)):
            return None
        phase = word.phase * self.dwell_ms // 256
        self.set_origin(tx_start_ms, phase, word.slot)
        self.base_stratum = min(theirs + 1, STRATUM_MAX)
        self.synced_ms = int(now_ms)
        return self.base_stratum

    def set_origin(self, at_ms: int, phase_ms: int, slot: int):
        self.origin_ms = int(at_ms) - phase_ms
        self.origin_slot = slot & SLOT_MASK

    def slots_for(self, interval_ms: int) -> int:
        return max(-(-interval_ms // self.dwell_ms), 1)

    def interval_elapsed(self, last_ms: int, interval_ms: int, now_ms: int) -> bool:
        due = (self.slot(last_ms) + self.slots_for(interval_ms)) & SLOT_MASK
        ahead = (self.slot(now_ms) - due) & SLOT_MASK
        return ahead < (1 << (SLOT_BITS - 1))

    def next_slot_start_ms(self, now_ms: int) -> int:
        return int(now_ms) + (self.dwell_ms - self.phase_ms(now_ms))

    def word_at(self, tx_start_ms: int) -> SyncWord:
        return SyncWord(
            self.slot(tx_start_ms),
            self.stratum(tx_start_ms),
            min(self.phase_ms(tx_start_ms) * 256 // self.dwell_ms, 255),
        )

    def turn_due(self, plan: Plan, address: int, last_ms, interval_ms: int, now_ms: int) -> bool:
        n = self.slots_for(interval_ms)
        slot = self.slot(now_ms)
        if slot % n != plan.turn_slot(address, n):
            return False
        return last_ms is None or self.slot(last_ms) != slot

    def tx_start(self, plan: Plan, address: int, interval_ms: int, now_ms: int, airtime_ms: int, turns: bool = True) -> int:
        n = self.slots_for(interval_ms)
        lo, hi = plan.start_range_for(address, n, airtime_ms) if turns else plan.start_range_ms(airtime_ms)
        self.rng = xorshift(self.rng)
        target = lo + self.rng % (hi - lo + 1)
        phase = self.phase_ms(now_ms)
        slot = self.slot(now_ms)
        # The node's own slot of the interval, not merely the next one:
        # with turns off (the legacy schedule) every slot is the node's.
        mine = plan.turn_slot(address, n) if turns else slot % n
        if slot % n == mine and phase <= target:
            return int(now_ms) + (target - phase)
        ahead = 1
        while ((slot + ahead) & SLOT_MASK) % n != mine:
            ahead += 1
        return int(now_ms) + (self.dwell_ms - phase) + (ahead - 1) * self.dwell_ms + target

    def wait_for_window_ms(self, plan: Plan, address: int, interval_ms: int, now_ms: int, airtime_ms: int, turns: bool = True) -> int:
        lo, hi = plan.start_range_for(address, self.slots_for(interval_ms), airtime_ms) if turns else plan.start_range_ms(airtime_ms)
        phase = self.phase_ms(now_ms)
        if phase < lo:
            return lo - phase
        if phase > hi:
            return self.dwell_ms - phase + lo
        return 0


def ldro(sf: int, bw_khz: int) -> bool:
    bw = 62 if bw_khz == 62 else bw_khz
    return (1 << sf) * 100 // bw >= 1638


def time_on_air_us(sf: int, bw_khz: int, cr: int, payload_len: int) -> int:
    sf = min(max(sf, 5), 12)
    bw_hz = 62_500 if bw_khz == 62 else bw_khz * 1000
    t_sym = (1 << sf) * (1_000_000 // bw_hz)
    t_pre = (4 * 8 + 17) * t_sym // 4
    crv = min(max(cr, 5), 8) - 4
    de = 1 if ldro(sf, bw_khz) else 0
    num = 8 * payload_len - 4 * sf + 28 + 16
    den = 4 * (sf - 2 * de)
    syms = 8 if num <= 0 else 8 + ((num + den - 1) // den) * (crv + 4)
    return t_pre + syms * t_sym


def selftest(path: str) -> int:
    with open(path, encoding="utf-8") as f:
        v = json.load(f)
    fails = 0

    def check(name, got, want):
        nonlocal fails
        if got != want:
            fails += 1
            print(f"MISMATCH {name}: got {got!r} want {want!r}")

    p = v["plan"]
    plan = Plan.new(p["channels"], p["step_khz"], p["center_hz"], p["dwell_ms"], v["unit_ms"])
    check("sub_slots", plan.sub_slots, p["sub_slots"])
    check("guard", plan.guard_ms(), p["guard_ms"])
    check("window", plan.window_ms(), p["window_ms"])
    check("sub_slot_ms", plan.sub_slot_ms(), p["sub_slot_ms"])
    check("channel_hz", [plan.channel_hz(i) for i in range(plan.channels)], v["channel_hz"])
    check("index_for_slot", [plan.index_for_slot(s) for s in range(250)], v["index_for_slot"])
    check("index_for_slot_high", [plan.index_for_slot(0xFFF00 + s) for s in range(60)], v["index_for_slot_high"])
    got = []
    for a in range(1, 7):
        for n in (1, 5):
            for t in (289, 240, 370, 450, 900):
                lo, hi = plan.start_range_for(a, n, t)
                got += [a, n, t, lo, hi]
    check("start_range_for", got, v["start_range_for"])
    got = []
    for a in range(1, 13):
        for n in (1, 5, 20):
            got += [plan.turn_slot(a, n), plan.sub_slot_of(a, n), plan.turns(n)]
    check("turn_slot", got, v["turn_slot"])
    toa = v["toa"]
    for i in range(0, len(toa), 5):
        sf, bw, cr, ln, want = toa[i : i + 5]
        check(f"toa sf{sf} bw{bw} cr{cr} len{ln}", time_on_air_us(sf, bw, cr, ln), want)
    got = []
    for seed in (1, 2, 3, 7, 200):
        c = Clock(1000, 0, seed)
        got += [seed, c.slot(0), c.phase_ms(0), c.stratum(0)]
        for i in range(12):
            now = 10_000 + i * 37
            got.append(c.tx_start(plan, seed % 3 + 1, 1000 * (1 + seed % 2), now, 289) & U32)
        got.append(c.word_at(12_345).to_u32())
    check("clocks", got, v["clocks"])
    a = Clock(1000, 0, 1)
    a.discipline_gps(1_000_000, 10_000)
    word = a.word_at(20_300)
    b = Clock(1000, 3, 2)
    adopted = b.offer(word, 1, 2, 77_300, 77_589)
    c = Clock(1000, 5_000, 7)
    c.discipline_gps(45_296_250, 5_000)
    got = [
        word.to_u32(),
        int(word.slot == a.slot(20_300)),
        99 if adopted is None else adopted,
        b.slot(77_300),
        b.phase_ms(77_300),
        b.stratum(77_589),
        b.word_at(77_800).to_u32(),
        c.slot(5_000),
        c.phase_ms(5_000),
        c.slot(5_750),
        int(c.turn_due(plan, 1, 10_100, 1000, 10_300)),
        int(c.turn_due(plan, 1, 10_100, 1000, 11_000)),
        int(c.turn_due(plan, 3, None, 5000, 10_500)),
        int(c.turn_due(plan, 3, None, 5000, 11_500)),
        c.wait_for_window_ms(plan, 1, 1000, 10_300, 289),
        c.wait_for_window_ms(plan, 2, 1000, 10_300, 289),
        c.wait_for_window_ms(plan, 2, 5000, 10_300, 289),
        c.wait_for_window_ms(plan, 7, 5000, 10_300, 289),
        c.next_slot_start_ms(10_300) & U32,
        SyncWord(0xABCDE, 9, 200).to_u32(),
    ]
    check("sync", got, v["sync"])
    check("header_ms", -(-time_on_air_us(12, 500, 5, 0) // 1000), v["header_ms"])
    print("selftest:", "FAILED" if fails else "ok", f"({fails} mismatches)")
    return 1 if fails else 0


# ---------------------------------------------------------------------------
# The network model
# ---------------------------------------------------------------------------

# Firmware and radio constants the model leans on. Each is either read from
# the firmware or an estimate marked as such in docs/RADIO-AUDIT.md.
SF, BW_KHZ, CR = 12, 500, 5
T_SYM_MS = (1 << SF) / BW_KHZ                 # 8.192 ms
HEADER_SYNC_LEN = 7                           # frame header + sync word
BEACON_LEN = HEADER_SYNC_LEN + 10             # lat/lon position
PING_LEN = HEADER_SYNC_LEN + 4
FRAME_MAX = HEADER_SYNC_LEN + 32
TOA_BEACON = time_on_air_us(SF, BW_KHZ, CR, BEACON_LEN) / 1000
TOA_PING = time_on_air_us(SF, BW_KHZ, CR, PING_LEN) / 1000
TOA_MAX = time_on_air_us(SF, BW_KHZ, CR, FRAME_MAX) / 1000
HEADER_MS = time_on_air_us(SF, BW_KHZ, CR, 0) / 1000
TICK_MS = 10.0                                # hardware loop period
TX_POLL_MS = 1.0                              # TxDone poll granularity
TCXO_MS = 10.3                                # STDBY_RC -> RF: TCXO start + PLL + ramp
XOSC_MS = 0.4                                 # STDBY_XOSC -> RF
SPI_SETUP_MS = 0.8                            # the commands around a mode change
PD_ON_BY_SYMS = 2.0                           # receiver must be on channel this far into the preamble (estimate)
PD_AT_SYMS = 6.0                              # PreambleDetected fires this far in (estimate)
CAPTURE_DB = 6.0
SD_FLUSH_MS = 40.0                            # 1 KB at 400 kHz plus card overhead (estimate)
SD_FLUSH_SLOW_MS = 250.0                      # a card on wear levelling (estimate)
SD_FLUSH_SLOW_P = 0.1
SD_PERIOD_MS = 5000.0
OLED_MS = 11.0                                # async I2C frame, awaited
OLED_PERIOD_MS = 500.0
NMEA_BYTES = 150                              # RMC + GGA at 9600 baud
UART_BPS = 960.0
UART_FIFO = 128
UART_PIPE = 512
GPS_LATENCY_MS = 180.0                        # epoch to first NMEA byte (estimate)
GPS_LATENCY_NODE_JITTER = 25.0
GPS_LATENCY_TICK_JITTER = 15.0
LATE_TICK_MS = 30.0                           # a tick this late is not a clock reference
FIRST_BEACON_STAGGER_MS = 2000.0
NOTIFY_MS = 1000.0


@dataclass
class Fixes:
    turns: bool = False        # address-based turns inside a slot and across an interval
    notify_wait: bool = False  # BLE notifier waits out a transmit instead of skipping the tick
    stamp_lead: bool = False   # sync word stamped for the RF start, not the command
    xosc: bool = False         # listening node keeps the TCXO running between modes
    staged_hold: bool = False  # preamble hold bounded by the header time until a header lands
    replan: bool = False       # a beacon whose window passed is re-planned, not waited for
    late_guard: bool = False   # a late tick's timestamps do not discipline the clock
    uart_pipe: bool = False    # GPS bytes pumped by an async task into a pipe
    dual_core: bool = False    # hardware loop on its own core

    @staticmethod
    def legacy() -> "Fixes":
        return Fixes()

    @staticmethod
    def fixed() -> "Fixes":
        return Fixes(True, True, True, True, True, True, True, True, False)

    @staticmethod
    def dual() -> "Fixes":
        return Fixes(True, True, True, True, True, True, True, True, True)


@dataclass
class NodeSpec:
    address: int
    fix: bool = True           # has a GPS fix throughout
    transmits: bool = True     # tracking (True) or listening (False)
    phone: bool = False        # a central is connected and subscribed
    rssi_dbm: float = -80.0    # as heard by everyone else
    beacon_s: int = 1
    ping_s: int = 5


@dataclass
class Frame:
    src: int
    channel: int
    rf_start: float
    rf_end: float
    word: SyncWord
    length: int
    toa: float
    rssi: float
    kind: str
    seen: dict = field(default_factory=dict)   # receiver -> stage processed


class Node:
    def __init__(self, spec: NodeSpec, plan: Plan, fixes: Fixes, rng: random.Random, sim: "Sim"):
        self.spec = spec
        self.plan = plan
        self.fx = fixes
        self.rng = rng
        self.sim = sim
        self.addr = spec.address
        # Local clock: Instant::now() since boot, with a per-board crystal.
        self.boot_offset = rng.uniform(0, 1_000_000)
        self.ppm = rng.uniform(-20, 20)
        # Boards boot at unrelated instants, so a free-running clock's slot
        # boundary sits anywhere in the slot.
        self.clock = Clock(plan.dwell_ms, int(self.local(0) - rng.uniform(0, plan.dwell_ms)), self.addr)
        self.gps_bias = rng.uniform(-GPS_LATENCY_NODE_JITTER, GPS_LATENCY_NODE_JITTER)
        self.listen = True
        self.first_beacon_at = self.local(0) + (self.addr % 8) * 1000 + FIRST_BEACON_STAGGER_MS
        self.last_tx = None          # (start_local, end_local)
        self.beacon_at = None
        self.rx_busy_since = None    # local ms
        self.rx_stage = None         # "preamble" | "header"
        self.rx_slot = None
        self.rx_armed = False
        self.segments: list[tuple[float, int, bool]] = [(0.0, -1, False)]
        self.prev_tick_end = 0.0
        self.next_sd = SD_PERIOD_MS
        self.next_oled = 0.0
        self.pending_packet = None   # (frame, survived)
        self.polls: list[float] = [0.0]           # true times gps.poll ran
        self.blocked: list[tuple[float, float]] = []   # blocking stalls (true time)
        self.position_dirty = False
        self.notify_next = 500.0 + rng.uniform(0, 1000)
        self.busy_until = 0.0        # true time the current transmit ends (radio_busy)
        self.busy_from = 0.0
        self.gps_seconds_done = set()
        self.last_word_tx = None
        # Stats
        self.sent = 0
        self.heard: dict[int, int] = {}
        self.lost: dict[str, int] = {}
        self.notified: list[float] = []
        self.notify_skips = 0
        self.notify_delay_max = 0.0
        self.rmc_lost = 0
        self.rmc_ok = 0
        self.joined_at = None
        self.sync_errors: list[float] = []
        self.deferrals = 0
        self.replans = 0
        self.overrun_waits = 0.0
        self.remote_gaps: dict[int, list[float]] = {}
        self.remote_last: dict[int, float] = {}

    # -- clocks --------------------------------------------------------------
    def local(self, t: float) -> float:
        return t * (1 + self.ppm * 1e-6) + self.boot_offset

    def lms(self, t: float) -> int:
        return int(self.local(t))

    # -- receiver timeline -----------------------------------------------------
    def set_radio(self, t: float, channel: int, armed: bool):
        self.segments.append((t, channel, armed))
        self.rx_armed = armed

    def on_throughout(self, a: float, b: float, channel: int) -> bool:
        if not self.segments:
            return False
        starts = [s[0] for s in self.segments]
        i = bisect.bisect_right(starts, a) - 1
        if i < 0:
            return False
        while i < len(self.segments) and self.segments[i][0] <= b:
            _, ch, armed = self.segments[i]
            if ch != channel or not armed:
                return False
            i += 1
        return True

    def current_channel(self) -> int:
        return self.segments[-1][1]

    # -- radio helpers -----------------------------------------------------------
    def mode_lead(self) -> float:
        return XOSC_MS if (self.fx.xosc and self.listen) else TCXO_MS

    def hold_bound(self) -> float:
        if self.fx.staged_hold and self.rx_stage == "preamble":
            return HEADER_MS + TICK_MS
        return min(TOA_MAX, self.plan.dwell_ms)

    def rx_in_progress(self, now_l: float) -> bool:
        return self.rx_busy_since is not None and now_l - self.rx_busy_since < self.hold_bound()

    def hop_tick(self, t: float):
        now_l = self.lms(t)
        slot = self.clock.slot(now_l)
        if self.rx_slot == slot:
            return t
        if self.rx_busy_since is not None:
            if now_l - self.rx_busy_since < self.hold_bound():
                return t
            self.rx_busy_since = None
            self.rx_stage = None
        self.rx_slot = slot
        ch = self.plan.index_for_slot(slot)
        # Standby, retune, re-arm: deaf for the SPI and the oscillator.
        self.set_radio(t, -1, False)
        t2 = t + SPI_SETUP_MS + self.mode_lead()
        self.set_radio(t2, ch, True)
        self.rx_armed = True
        return t

    def frame_visible(self, f: Frame) -> bool:
        return self.on_throughout(f.rf_start + PD_ON_BY_SYMS * T_SYM_MS, f.rf_start + PD_AT_SYMS * T_SYM_MS, f.channel)

    def frame_received(self, f: Frame) -> tuple[bool, str]:
        if not self.on_throughout(f.rf_start + PD_ON_BY_SYMS * T_SYM_MS, f.rf_end, f.channel):
            return False, "off-channel"
        for g in self.sim.frames:
            if g is f or g.src == f.src or g.channel != f.channel:
                continue
            if g.rf_start >= f.rf_end or g.rf_end <= f.rf_start:
                continue
            # Only an interferer this receiver can hear at all interferes.
            if not self.on_throughout(max(g.rf_start, f.rf_start), min(g.rf_end, f.rf_end), f.channel):
                continue
            if g.rf_start > f.rf_start and f.rssi - g.rssi >= CAPTURE_DB:
                continue
            return False, "overlap"
        return True, ""

    # -- the hardware loop ------------------------------------------------------
    def tick(self, t: float) -> float:
        """One pass of hardware_task at true time t. Returns the true time
        the next pass is due."""
        # A pass is late when the previous poll was long ago: whatever the
        # radio or the UART delivered since then has an unknown arrival
        # time inside that gap.
        late = (t - self.polls[-1]) > TICK_MS + LATE_TICK_MS
        now_l = self.lms(t)

        # ---- GPS -----------------------------------------------------------
        self.polls.append(t)
        mark = self.gps_poll(t)
        if mark is not None and self.spec.fix:
            tod_ms, at_l = mark
            if not (self.fx.late_guard and late):
                self.clock.discipline_gps(tod_ms, at_l)

        # ---- beacon ------------------------------------------------------------
        interval_ms = (self.spec.beacon_s if self.spec.fix else self.spec.ping_s) * 1000
        length = BEACON_LEN if self.spec.fix else PING_LEN
        toa = TOA_BEACON if self.spec.fix else TOA_PING
        last_start = self.last_tx[0] if self.last_tx else None
        if self.fx.turns:
            due = self.clock.turn_due(self.plan, self.addr, last_start, interval_ms, now_l)
        else:
            due = last_start is None or self.clock.interval_elapsed(last_start, interval_ms, now_l)
        owed = self.spec.transmits and now_l >= self.first_beacon_at and due
        if owed and self.beacon_at is None:
            self.beacon_at = self.clock.tx_start(self.plan, self.addr, interval_ms, now_l, int(math.ceil(toa)), self.fx.turns)
        if owed and self.beacon_at is not None and now_l >= self.beacon_at:
            if self.rx_in_progress(now_l):
                self.deferrals += 1
            else:
                wait = self.clock.wait_for_window_ms(self.plan, self.addr, interval_ms, now_l, int(math.ceil(toa)), self.fx.turns)
                if wait > 0 and self.fx.replan:
                    self.replans += 1
                    self.beacon_at = self.clock.tx_start(self.plan, self.addr, interval_ms, now_l, int(math.ceil(toa)), self.fx.turns)
                else:
                    self.beacon_at = None
                    t = self.send(t, wait, length, toa)
                    now_l = self.lms(t)
        else:
            # `beacon_at` is only ever consumed on a transmit, as in the firmware.
            pass

        # ---- receive ------------------------------------------------------------
        # A transmit on this pass makes the receive poll late as well.
        late = late or (t - self.polls[-1]) > LATE_TICK_MS
        t = self.poll_recv(t, late)

        # ---- card, panel --------------------------------------------------------
        if self.local(t) - self.boot_offset >= self.next_sd:
            self.next_sd += SD_PERIOD_MS
            stall = SD_FLUSH_SLOW_MS if self.rng.random() < SD_FLUSH_SLOW_P else SD_FLUSH_MS
            self.blocked.append((t, t + stall))
            t += stall
        if self.local(t) - self.boot_offset >= self.next_oled:
            self.next_oled += OLED_PERIOD_MS
            t += OLED_MS   # awaited: other tasks run, this loop does not
        # Executor contention from the BLE host on a shared core.
        if not self.fx.dual_core and self.sim.any_phone:
            d = self.rng.expovariate(1 / 0.5)
            if self.rng.random() < 0.02:
                d += 15.0
            self.blocked.append((t, t + d))
            t += d
        self.prev_tick_end = t
        return t + TICK_MS

    def send(self, t: float, wait: int, length: int, toa: float) -> float:
        if wait > 0:
            self.overrun_waits += wait
            t += wait                      # Timer::after inside send(): the loop is held
        self.set_radio(t, -1, False)       # set_standby: the receiver is off
        t_cmd = t + SPI_SETUP_MS
        tx_start_l = self.lms(t_cmd)
        lead = self.mode_lead()
        stamp_at = tx_start_l + (int(round(lead)) if self.fx.stamp_lead else 0)
        word = self.clock.word_at(stamp_at)
        slot = self.clock.slot(tx_start_l)
        channel = self.plan.index_for_slot(slot)
        self.rx_slot = slot
        rf_start = t_cmd + lead
        rf_end = rf_start + toa
        kind = "beacon" if self.spec.fix else "ping"
        self.sim.frames.append(Frame(self.addr, channel, rf_start, rf_end, word, length, toa, self.spec.rssi_dbm, kind))
        self.sent += 1
        self.busy_from, self.busy_until = t, rf_end + TX_POLL_MS
        t_done = rf_end + self.rng.uniform(0, TX_POLL_MS)
        self.last_tx = ((tx_start_l + lead) if self.fx.stamp_lead else tx_start_l, self.lms(t_done))
        # Back into RX on the same channel after the oscillator settles.
        self.set_radio(t_done, -1, False)
        self.set_radio(t_done + SPI_SETUP_MS + self.mode_lead(), channel, True)
        self.rx_armed = True
        return t_done

    def poll_recv(self, t: float, late: bool) -> float:
        t = self.hop_tick(t)
        if not self.rx_armed:
            ch = self.plan.index_for_slot(self.clock.slot(self.lms(t)))
            self.set_radio(t, -1, False)
            self.set_radio(t + SPI_SETUP_MS + self.mode_lead(), ch, True)
            self.rx_slot = self.clock.slot(self.lms(t))
        now_l = self.lms(t)
        # What the radio latched since the last poll.
        done = None
        for f in self.sim.frames:
            if f.src == self.addr:
                continue
            stage = f.seen.get(self.addr)
            if stage == "done":
                continue
            pd_at = f.rf_start + PD_AT_SYMS * T_SYM_MS
            if pd_at > t:
                continue
            if stage is None:
                if self.frame_visible(f):
                    self.rx_busy_since = now_l
                    self.rx_stage = "preamble"
                    f.seen[self.addr] = "preamble"
                else:
                    f.seen[self.addr] = "invisible"
                    continue
                stage = f.seen[self.addr]
            if stage == "invisible":
                if f.rf_end <= t:
                    f.seen[self.addr] = "done"
                    self.lost["off-channel"] = self.lost.get("off-channel", 0) + 1
                continue
            if stage == "preamble" and f.rf_start + HEADER_MS <= t:
                if self.on_throughout(f.rf_start + PD_ON_BY_SYMS * T_SYM_MS, f.rf_start + HEADER_MS, f.channel):
                    self.rx_stage = "header"
                    f.seen[self.addr] = "header"
                    stage = "header"
            if f.rf_end <= t:
                f.seen[self.addr] = "done"
                ok, why = self.frame_received(f)
                if ok:
                    done = f  # the last one to land is what the buffer holds
                else:
                    self.lost[why] = self.lost.get(why, 0) + 1
                self.rx_busy_since = None
                self.rx_stage = None
        if done is not None:
            f = done
            self.heard[f.src] = self.heard.get(f.src, 0) + 1
            if self.joined_at is None and not self.spec.fix:
                self.joined_at = t
            toa = int(math.ceil(f.toa))
            if not (self.fx.late_guard and late and self.clock.synced()):
                self.clock.offer(f.word, f.src, self.addr, now_l - toa, now_l)
            last = self.remote_last.get(f.src)
            if last is not None:
                self.remote_gaps.setdefault(f.src, []).append(t - last)
            self.remote_last[f.src] = t
            self.sim.deliveries.append((t, self.addr, f.src))
        return t

    # -- GPS UART ---------------------------------------------------------------
    def gps_poll(self, t: float):
        """Model the NMEA burst of each second and whether its bytes survived
        the FIFO until this poll. Returns a time mark when the RMC of a
        second was parsed on this pass."""
        if not self.spec.fix:
            return None
        sec = int(t // 1000)
        mark = None
        for s in (sec - 1, sec):
            if s < 0 or s in self.gps_seconds_done:
                continue
            start = s * 1000 + GPS_LATENCY_MS + self.gps_bias + self.rng.uniform(-GPS_LATENCY_TICK_JITTER, GPS_LATENCY_TICK_JITTER)
            end = start + NMEA_BYTES / UART_BPS * 1000
            if end > t:
                continue
            self.gps_seconds_done.add(s)
            lost = self.uart_lost(start, end)
            if lost:
                self.rmc_lost += 1
            else:
                self.rmc_ok += 1
                self.position_dirty = True
                mark = (((self.sim.tod0 + s) * 1000) % 86_400_000, self.lms(t))
        return mark

    def uart_lost(self, start: float, end: float) -> bool:
        def bytes_in(a: float, b: float) -> float:
            lo, hi = max(a, start), min(b, end)
            return max(hi - lo, 0) * UART_BPS / 1000

        if not self.fx.uart_pipe:
            # The 128-byte FIFO is drained only by gps.poll, once per pass.
            polls = self.polls
            i = bisect.bisect_left(polls, start) - 1
            i = max(i, 0)
            while i + 1 < len(polls) and polls[i] < end:
                if bytes_in(polls[i], polls[i + 1]) > UART_FIFO:
                    return True
                i += 1
            return False
        # The pump drains the FIFO whenever its executor runs. It stops
        # only while that core is blocked, which on one core is every
        # blocking stall of the hardware loop and on two is never.
        if not self.fx.dual_core:
            for a, b in self.blocked:
                if b < start or a > end:
                    continue
                if bytes_in(a, b) > UART_FIFO:
                    return True
        # The pipe itself holds 512 bytes between passes of gps.poll.
        polls = self.polls
        i = max(bisect.bisect_left(polls, start) - 1, 0)
        while i + 1 < len(polls) and polls[i] < end:
            if bytes_in(polls[i], polls[i + 1]) > UART_FIFO + UART_PIPE:
                return True
            i += 1
        return False

    # -- BLE notifier ------------------------------------------------------------
    def notifier_tick(self, t: float) -> float:
        """The position notifier's tick at true time t. Returns the next tick."""
        due_next = t + NOTIFY_MS
        # On one core the tick fires when the executor is free.
        fire = t
        if not self.fx.dual_core:
            for a, b in self.blocked:
                if a <= fire < b:
                    fire = b
        busy = self.busy_from <= fire < self.busy_until
        if busy:
            if self.fx.notify_wait:
                fire = self.busy_until
            else:
                self.notify_skips += 1
                return due_next
        if self.position_dirty:
            self.position_dirty = False
            self.notified.append(fire)
            self.notify_delay_max = max(self.notify_delay_max, fire - t)
        return due_next


class Sim:
    def __init__(self, specs: list[NodeSpec], fixes: Fixes, seconds: float, seed: int, plan: Plan | None = None):
        self.rng = random.Random(seed)
        self.plan = plan or Plan.new(50, 500, 915_000_000, 1000, int(math.ceil(TOA_BEACON)))
        self.fixes = fixes
        # The time of day the run starts at, in whole seconds: the GPS
        # nodes' slot numbers, and so their channel sequence, follow it.
        self.tod0 = self.rng.randrange(0, 86_400)
        self.frames: list[Frame] = []
        self.deliveries: list[tuple[float, int, int]] = []
        self.any_phone = any(s.phone for s in specs)
        self.nodes = [Node(s, self.plan, fixes, random.Random(seed * 1000 + s.address), self) for s in specs]
        self.seconds = seconds
        self.overlaps = 0
        self.overlap_pairs: list[tuple[float, int, int]] = []

    def run(self):
        heap: list[tuple[float, int, int, int]] = []
        for i, n in enumerate(self.nodes):
            heapq.heappush(heap, (n.rng.uniform(0, TICK_MS), 0, i, 0))
            if n.spec.phone:
                heapq.heappush(heap, (n.notify_next, 1, i, 1))
        end = self.seconds * 1000
        checked = 0
        last_prune = 0.0
        while heap:
            t, kind, i, _ = heapq.heappop(heap)
            if t > end:
                break
            n = self.nodes[i]
            if kind == 0:
                nxt = n.tick(t)
                heapq.heappush(heap, (nxt, 0, i, 0))
                # Sync error against the first node with a fix, sampled at ticks.
                ref = self.nodes[0]
                if n is not ref and n.clock.synced() and ref.clock.synced():
                    e = (n.clock.phase_ms(n.lms(t)) - ref.clock.phase_ms(ref.lms(t))) % self.plan.dwell_ms
                    if e > self.plan.dwell_ms / 2:
                        e -= self.plan.dwell_ms
                    n.sync_errors.append(e)
            else:
                nxt = n.notifier_tick(t)
                heapq.heappush(heap, (nxt, 1, i, 1))
            # Overlap census over frames that have ended, then prune.
            if t - last_prune > 5000:
                last_prune = t
                self.frames.sort(key=lambda f: f.rf_start)
                keep = []
                for f in self.frames:
                    if f.rf_end < t - 3000 and all(v == "done" for v in f.seen.values()) and len(f.seen) >= len(self.nodes) - 1:
                        continue
                    keep.append(f)
                self.count_overlaps(checked)
                checked = len(self.overlap_pairs)
                self.frames = keep

    def count_overlaps(self, _):
        fs = sorted(self.frames, key=lambda f: f.rf_start)
        for i, f in enumerate(fs):
            if getattr(f, "counted", False):
                continue
            f.counted = True
            for g in fs[i + 1 :]:
                if g.rf_start >= f.rf_end:
                    break
                if g.channel == f.channel and g.src != f.src:
                    self.overlaps += 1
                    self.overlap_pairs.append((f.rf_start, f.src, g.src))

    def report(self) -> dict:
        self.count_overlaps(0)
        sent = sum(n.sent for n in self.nodes)
        out = {"sent": sent, "overlaps": self.overlaps, "nodes": {}}
        for n in self.nodes:
            heard_total = sum(n.heard.values())
            senders = [m for m in self.nodes if m is not n and m.spec.transmits]
            sendable = sum(m.sent for m in senders)
            gaps = [g for gs in n.remote_gaps.values() for g in gs]
            notify_gaps = [b - a for a, b in zip(n.notified, n.notified[1:])]
            errs = n.sync_errors
            out["nodes"][n.addr] = {
                "sent": n.sent,
                "heard": heard_total,
                "hearable": sendable,
                "delivery_pct": (100.0 * heard_total / sendable) if sendable else None,
                "lost": dict(n.lost),
                "deferrals": n.deferrals,
                "replans": n.replans,
                "overrun_wait_ms": n.overrun_waits,
                "rmc_ok": n.rmc_ok,
                "rmc_lost": n.rmc_lost,
                "joined_s": None if n.joined_at is None else n.joined_at / 1000,
                "stratum": n.clock.stratum(n.lms(self.seconds * 1000)),
                "sync_err_max_ms": max((abs(e) for e in errs), default=None),
                "sync_err_p99_ms": (sorted(abs(e) for e in errs)[int(0.99 * (len(errs) - 1))]) if errs else None,
                "sync_err_mean_ms": (sum(errs) / len(errs)) if errs else None,
                "remote_gap_max_s": (max(gaps) / 1000) if gaps else None,
                "remote_gap_over_2s": sum(1 for g in gaps if g > 2000),
                "notified": len(n.notified),
                "notify_skips": n.notify_skips,
                "notify_gap_max_s": (max(notify_gaps) / 1000) if notify_gaps else None,
                "notify_delay_max_ms": n.notify_delay_max,
            }
        return out


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

def scenarios() -> dict[str, list[NodeSpec]]:
    return {
        "A": [NodeSpec(1), NodeSpec(2)],
        "B": [NodeSpec(1), NodeSpec(2), NodeSpec(3, fix=False, transmits=False, phone=True)],
        "C": [NodeSpec(1), NodeSpec(2), NodeSpec(3), NodeSpec(4), NodeSpec(5, fix=False, transmits=False, phone=True)],
        "C2": [NodeSpec(1, beacon_s=2), NodeSpec(2, beacon_s=2), NodeSpec(3, beacon_s=2), NodeSpec(4, beacon_s=2), NodeSpec(5, fix=False, transmits=False, phone=True)],
        "G": [NodeSpec(1), NodeSpec(2), NodeSpec(3), NodeSpec(4, fix=False, transmits=False, phone=True)],
        "D": [NodeSpec(1, phone=True), NodeSpec(2)],
        "E": [NodeSpec(1, fix=False), NodeSpec(2, fix=False), NodeSpec(3, fix=False, transmits=False, phone=True)],
        "F": [NodeSpec(1, beacon_s=5), NodeSpec(2, beacon_s=5), NodeSpec(3, fix=False, transmits=False, phone=True)],
    }


SCENARIO_TEXT = {
    "A": "two trackers with a fix, beacon every second",
    "B": "two trackers and a listening node with no fix and a phone",
    "C": "four trackers every second (over the air's capacity) and a listener",
    "C2": "four trackers every 2 s and a listening node with a phone",
    "G": "three trackers every second (one turn short) and a listener",
    "D": "a tracker with the phone on it, and one other tracker",
    "E": "two trackers with no fix (pings every 5 s) and a listener",
    "F": "two trackers beaconing every 5 s and a listener",
}

VARIANTS = {"legacy": Fixes.legacy, "fixed": Fixes.fixed, "dual": Fixes.dual}


def merge(reports: list[dict]) -> dict:
    """Average the numeric fields of several seeds' reports; sums stay sums
    per run, so a mean of counts is a mean over runs."""
    def avg(vals):
        vals = [v for v in vals if v is not None]
        return (sum(vals) / len(vals)) if vals else None

    out = {"sent": avg(r["sent"] for r in reports), "overlaps": avg(r["overlaps"] for r in reports), "runs": len(reports), "nodes": {}}
    for addr in reports[0]["nodes"]:
        rows = [r["nodes"][addr] for r in reports]
        merged = {}
        for key in rows[0]:
            if key == "lost":
                keys = {k for row in rows for k in row["lost"]}
                merged["lost"] = {k: avg(row["lost"].get(k, 0) for row in rows) for k in keys}
            elif key == "stratum":
                merged[key] = max(row[key] for row in rows)
            elif key == "joined_s":
                merged[key] = avg(row[key] for row in rows)
                merged["unjoined"] = sum(1 for row in rows if row[key] is None)
            elif key.endswith("_max_ms") or key.endswith("_max_s"):
                merged[key] = max((row[key] for row in rows if row[key] is not None), default=None)
            else:
                merged[key] = avg(row[key] for row in rows)
        out["nodes"][addr] = merged
    return out


def make_plan(channels: int) -> Plan:
    """The plan the runs are on. One channel is the firmware default: the
    slot clock and the turns with nowhere to hop to."""
    return Plan.new(channels, 500, 915_000_000, 1000, int(math.ceil(TOA_BEACON)))


def run_all(seconds: float, seed: int, which: list[str], variants: list[str], seeds: int = 1,
            channels: int = 50) -> dict:
    results = {}
    plan = make_plan(channels)
    for name in which:
        specs = scenarios()[name]
        results[name] = {}
        for var in variants:
            reports = []
            for k in range(seeds):
                sim = Sim(specs, VARIANTS[var](), seconds, seed + k, plan)
                sim.run()
                reports.append(sim.report())
            results[name][var] = merge(reports) if seeds > 1 else reports[0]
    return results


def fmt(v, digits=1):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def print_summary(results: dict, seconds: float):
    for name, vars_ in results.items():
        runs = next(iter(vars_.values())).get("runs", 1)
        print(f"\n== Scenario {name}: {SCENARIO_TEXT[name]} ({seconds:.0f} s, {runs} seed{'s' if runs > 1 else ''})")
        print(f"{'variant':8} {'sent':>5} {'overlap':>7} | {'node':>4} {'heard%':>6} {'off-ch':>6} {'ovl':>5} {'defer':>5} {'wait':>5} {'rmc-':>5} {'join':>5} {'strat':>5} {'sync':>5} {'p99':>4} {'gap>2s':>6} {'ntf':>5} {'skip':>5} {'ntfgap':>6}")
        for var, r in vars_.items():
            first = True
            for addr, n in r["nodes"].items():
                head = f"{var:8} {fmt(r['sent'], 0):>5} {fmt(r['overlaps'], 0):>7}" if first else " " * 22
                first = False
                print(
                    f"{head} | {addr:>4} {fmt(n['delivery_pct']):>6} {fmt(n['lost'].get('off-channel', 0), 0):>6} {fmt(n['lost'].get('overlap', 0), 0):>5} "
                    f"{fmt(n['deferrals'], 0):>5} {fmt(n['overrun_wait_ms'] / 1000, 1):>5} {fmt(n['rmc_lost'], 1):>5} {fmt(n['joined_s'], 0):>5} {n['stratum']:>5} {fmt(n['sync_err_max_ms'], 0):>5} {fmt(n['sync_err_p99_ms'], 0):>4} "
                    f"{fmt(n['remote_gap_over_2s'], 0):>6} {fmt(n['notified'], 0):>5} {fmt(n['notify_skips'], 0):>5} {fmt(n['notify_gap_max_s']):>6}"
                )


def gantt(name: str, var: str, seconds: float, seed: int, from_s: float, span_s: float,
          channels: int = 50) -> str:
    sim = Sim(scenarios()[name], VARIANTS[var](), seconds, seed, make_plan(channels))
    sim.run()
    a, b = from_s * 1000, (from_s + span_s) * 1000
    frames = [f for f in sim.frames if f.rf_end > a and f.rf_start < b]
    overlapped = set()
    for f in frames:
        for g in frames:
            if g is not f and g.channel == f.channel and g.src != f.src and g.rf_start < f.rf_end and f.rf_start < g.rf_end:
                overlapped.add(id(f))
    lines = [
        "```mermaid",
        "gantt",
        f"    title Scenario {name}, {var}: {span_s:.0f} s of air, ms from {from_s:.0f} s",
        "    dateFormat x",
        "    axisFormat %L",
    ]
    for n in sim.nodes:
        if not n.spec.transmits:
            continue
        lines.append(f"    section node {n.addr}")
        for f in sorted(frames, key=lambda f: f.rf_start):
            if f.src != n.addr:
                continue
            tag = "crit" if id(f) in overlapped else "active"
            lines.append(f"    ch {f.channel} {f.kind}{' OVERLAP' if id(f) in overlapped else ''} :{tag}, {int(f.rf_start - a)}, {int(f.toa)}ms")
    listener = next((n for n in sim.nodes if not n.spec.transmits), None)
    if listener is not None:
        lines.append(f"    section node {listener.addr} rx")
        segs = [s for s in listener.segments if s[0] < b]
        for i, (t0, ch, armed) in enumerate(segs):
            t1 = segs[i + 1][0] if i + 1 < len(segs) else b
            if t1 <= a or not armed:
                continue
            lines.append(f"    on ch {ch} :done, {int(max(t0, a) - a)}, {int(min(t1, b) - max(t0, a))}ms")
    lines.append("```")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true", help="check the hop port against hop_vectors.json")
    ap.add_argument("--vectors", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "hop_vectors.json"))
    ap.add_argument("--seconds", type=float, default=600.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--seeds", type=int, default=1, help="run this many seeds from --seed and average")
    ap.add_argument("--scenario", action="append", help="A-F, repeatable; default all")
    ap.add_argument("--variant", action="append", help="legacy, fixed, dual; default all")
    ap.add_argument("--json", help="write the results here")
    ap.add_argument("--gantt", metavar="SCENARIO", help="print a mermaid gantt of a few seconds of one scenario")
    ap.add_argument("--from", dest="from_s", type=float, default=120.0)
    ap.add_argument("--span", type=float, default=4.0)
    ap.add_argument("--channels", type=int, default=50,
                    help="channels in the plan; 1 is the firmware default - the slot "
                         "clock and the turns on a single carrier (default 50)")
    args = ap.parse_args()

    if args.selftest:
        return selftest(args.vectors)
    variants = args.variant or list(VARIANTS)
    if args.gantt:
        print(gantt(args.gantt, variants[0], args.seconds, args.seed, args.from_s, args.span,
                    args.channels))
        return 0
    which = args.scenario or list(scenarios())
    results = run_all(args.seconds, args.seed, which, variants, args.seeds, args.channels)
    print_summary(results, args.seconds)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=1)
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
