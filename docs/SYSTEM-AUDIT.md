# System audit: redundancies and the systems that are not what they should be

*2026-09-08. Firmware at `5580dfb`, app at `137bc4e`. Both repos read
end to end: every source file, the tools, the docs. Nothing here is a
change; it is the list to work from, ranked at the end.*

The system has grown by accretion since the two-MCU board: a port that
kept the old vocabulary, three modes added over a pair of sleep flags,
hopping added over a fixed carrier and then defaulted back to one channel,
the state space work that pulled policy into `proto/`. Each step was
reasonable and each left something behind. What follows is what is now
duplicated, what is dead, and where the shape of the code works against
the next change rather than for it.

Sizes, for scale:

| Piece | Lines | Tests |
|---|---|---|
| `firmware/src/bin/main.rs` | 2,561 | 0 (no_std) |
| `firmware/src/*.rs` (13 modules) | 5,600 | 0 |
| `proto/src/*.rs` (12 modules) | 10,700 (about half tests) | 233 + 4 models |
| `explore/` | 758 | 8 |
| `tools/*.py` | 2,312 | 1 self-test |
| `gps-gui-rs/src/app.rs` | 4,688 | 52 |
| `gps-gui-rs/src/ble/*.rs` | 3,832 | 27 |
| Docs (`README`, `ARCHITECTURE`, `docs/`) | 4,900 | - |

## 1. Redundancies

### 1.1 The power settings exist in five shapes

Eight numbers - mode, sleep interval, advertising window, BLE off, BLE
on, idle timeout, two override flags, plus the name - are kept and
described in five places that have to agree by hand:

```mermaid
flowchart LR
    TOML["RADIO.CFG [power]<br/>radiocfg::PowerConfig<br/>five Option fields"]
    REC["nvs settings record<br/>Stored::encode_record<br/>version 7, 56 bytes"]
    RTC["RTC RAM<br/>settings.rs<br/>ten atomics + a byte array"]
    ST["session::Stored<br/>eight fields<br/>the policy's type"]
    WIRE["BLE settings blob<br/>ble::Settings<br/>version 6, 28 bytes"]
    APP["gps-gui-rs<br/>seven text fields on MyApp<br/>plus board_settings"]

    TOML -->|adopt_power| ST
    ST <-->|encode_record / decode_record| REC
    ST <-->|get / set| RTC
    ST -->|settings()| WIRE
    WIRE -->|Settings::decode| APP
```

Adding `ble_on_s` touched all five plus the app's page and its text
field; the record and the blob each carry a version ladder for it. The
two version numbers (6 and 7) count the same additions and are off by one
from each other for no reason a reader can recover.

**Recommendation.** One field table in `session.rs` - a macro listing
(name, type, default, record offset, wire offset) - from which `Stored`,
`Settings::encode/decode`, `encode_record/decode_record` and the RTC
statics are generated. `PowerConfig` then becomes "which of these keys the
file mentioned", derived from the same list. One version number.

### 1.2 The two-MCU board's vocabulary is still the API

The port kept names and ids for hardware this board does not have:

| Vocabulary | Refs | What it is now |
|---|---|---|
| `Action::Rail`, `PFLAG_PWR_OFF`, `CFG_PWR_EN`, `SFLAG_PWR_EN`, `Stored::pwr_en`, `rail_at_boot` | firmware logs "no hardware here"; five tests | The GPS/LoRa rail switch the old board had. Accepted, stored, persisted, reported to the app, and never acted on. `rail_at_boot` has no caller outside its tests. |
| `Action::WioSleep`, `CFG_WIO_SLEEP`, `PFLAG_WIO_SLEEP`, `Stored::wio_sleep`, `Request::RadioStandby` | everywhere | One thing - radio standby - under two names, with the translation `WioSleep -> RadioStandby` done in `dispatch`. |
| `link::resp::NAK`, `link::err::*` (eight codes), `link::DATA_CHUNK` | 0, 0, 1 | The old UART link's failure vocabulary. Nothing emits a NAK; the one use of `DATA_CHUNK` is to define `ble::BULK_DATA_MAX`. |
| `ble::ACK_WIO_ERROR` | bulk sink refusals | "The WIO refused", on a board with no WIO. |
| `tools/wio_*.py`, `wio-config`, `wio-set` | the tool names | Named for the chip they no longer talk to. |

Three ack vocabularies coexist: gps-proto's `packet::ACK_*`, this crate's
`ble::ACK_BAD_STATE` / `ACK_WIO_ERROR`, and `link::err::*`. The tools
carry a fourth in `STATUS_NAMES`.

**Recommendation.** Keep the wire ids reserved (an old app must not be
misread) and delete the policy behind the rail: `Action::Rail`,
`rail_at_boot`, `PFLAG_PWR_OFF`, `Settings::pwr_en`. Rename the WIO
family to radio standby in code, ids unchanged. Delete `link::resp::NAK`
and `link::err`; move `DATA_CHUNK` to `ble::BULK_DATA_MAX`. Rename the
tools to `board-*`. This is the cheapest item on the list and it removes
the most reading.

### 1.3 Two ways to say "do not transmit", and a 4 x 4 matrix nobody wrote down

`Role` (in the radio config, travels with the fleet) has `TxOnly` and
`RxOnly`; `Mode` (in the settings, per board) has `Listening`. A node
that must not transmit can be either `role = "rx_only"` or `mode =
listening`, and the firmware ANDs them (`Posture::may_transmit`,
`Node::broadcast` on `Role::transmits`). The sixteen combinations are
each handled somewhere, and the equivalences (`Listening` with any
transmitting role behaves as `RxOnly`) are nowhere stated.

**Recommendation.** Keep both - the role is the network's word and the
mode is the device's - but write the matrix into `posture.rs` as one
function (`fn on_air(mode, role) -> OnAir { transmits, receives, repeats
}`) and drive both the beacon gate and `Node` from it. Then decide whether
`rx_only` earns its keep: a listening node is what it was for.

### 1.4 `hop_channels` carries two decisions, and 0 keeps a second scheduler alive

`0` is no schedule (random jitter on the interval, no sync word), `1` is
the schedule on one carrier, `N` is the schedule with hopping. Every
consumer tests it differently: `Plan::from_config` (`> 0`),
`tx_worst_case_ms` (`> 0`), the app's `airtime` (`> 1`),
`Sx1262Driver::scheduled()`. The `0` path is a whole second scheduler:
`beacon_due` has two arms, so do `tx_window_start`, `tx_wait_ms`,
`shares_turn`, `hop_status`, `frame_overhead`, `sync_placeholder`, and
the hardware loop's jitter branch (`node.random(interval / 2)`). The
radio audit showed the schedule is the part that is always worth having;
the audit's own numbers say the jitter path delivers 76% where the
schedule delivers 99.6%.

**Recommendation.** Retire `hop_channels = 0`: the plan always exists,
`hop_channels` is 1 or more, and the sync word is unconditional. That
deletes a branch from seven places and the jitter path from the loop.
Wire-compatible: a frame's `FLAG_SYNC` stays set, and a config saying `0`
can parse as `1` with a console line.

### 1.5 Three enums for "what the serve loop does next"

`session::Next` (`Stored::at_expiry`), `session::Pass` (`Serve::pass`,
which is `Next` plus a `bounded` flag), and `session::Then` (after an
accept or a session). `Pass::BleDown` and `Then::Return` mean the same
thing. Introduced by the state space work, so this one is mine.

**Recommendation.** Fold `Pass` into `Next` (`Next::Advertise { bounded
}`) and have `Then::Return` carry the off period. Two enums, not three.

### 1.6 The app's two transports are one session written twice

`desktop.rs` (608 lines) and `android.rs` (1,016) each contain the same
session: scan, connect, discover, subscribe, read three characteristics,
then a pump that drains writes, paces a push ack by ack, dispatches
notifications and checks the inbox. The push block alone is sixty lines
in each, differing only in the write call (`peripheral.write(c, ..)`
against `bridge.write_characteristic(..)`); the ack dispatch, the
`OP_ABORT` on timeout and the `ConfigPushed` reporting are copied. The
worker model in `ble/statespace.rs` had to *model* these phases because
they are not shared code, which is the weakest part of that model.

**Recommendation.** A `Session` phase machine in `ble/mod.rs` - `enum
Phase`, `fn on(&mut self, LinkEvent) -> Vec<LinkOp>` - and a `Link` trait
(`write_config`, `write_bulk`, `read`, `subscribe`, `disconnect`) that
each transport implements in a hundred lines. The model then drives the
real phases, the Android setup-loss bug becomes impossible to reintroduce
in one transport only, and ISSUES.md's "BLE needs to be much more
resilient" has one place to be made so.

### 1.7 Each config key is described three times

`RADIO.example.toml` carries a comment and a `<key>_description` per
key; `radiocfg::parse` carries the range check and the type; the app's
editor derives an input from the value's TOML type or a `<key>_type`
hint. The app embeds the firmware repo's example by relative path
(`include_str!("../../telemetry-in-midair-rs/RADIO.example.toml")`), and
the firmware reads at most 1,024 bytes of a file that is 17 KB, so the
tool strips the descriptions before sending - the documented file can
never be the card file.

**Recommendation.** A key table in `radiocfg.rs` (name, type, range,
default, one-line doc) that drives the parser, is printed as the example
TOML by an example binary (like `hop_vectors`), and is what the app's
editor reads through the proto crate rather than through a path.

### 1.8 The host tools mirror the protocol by hand

`wio_link.py` restates `SYNC`, the USB ids, the bulk ops, the kinds, the
ack ids and a status name table from `link.rs` and `ble.rs`;
`radio_sim.py` ports the hop clock and the time-on-air formula. Only the
last two are checked (`--selftest` against `hop_vectors.json`); the
constants are checked by nothing, and the review of this session found
the vectors themselves stale.

**Recommendation.** One example binary in `proto/` prints every wire
constant and the vectors as JSON; the tools load it; the self-test covers
the constants. Regenerating it is one line in the tool's docstring.

### 1.9 The documents overlap

Three power documents - `POWER.md` (the settings), `POWER-S3.md` (924
lines: seven numbered levers, some landed), `POWER-AUDIT.md` (an audit of
the second) - plus `STATES-PLAN.md` (a plan marked implemented) and
`PLAN.md`. `README.md` is 977 lines against a stated preference for a
concise one: its BLE section alone is 350 lines of characteristic
reference, and it carries module, connector, charging and power notes
that are hardware reference. `ARCHITECTURE.md` is 1,103 lines.

**Recommendation.** `README.md` to about 150 lines: what, build, flash,
configure, where the rest is. `docs/BLE.md` takes the characteristic
reference, `docs/HARDWARE.md` the module and connector notes. One
`docs/POWER.md` with a measured table and an open-levers list, the two
audits folded in as history. `STATES-PLAN.md` into `TODO_complete.md`.

## 2. Systems that are not what they should be

### 2.1 `main.rs` is the firmware

2,561 lines and 87 console prints: the BLE duty cycle, the GATT server
and its five-arm session, the serve loop, deep sleep, the J5 probe, a
700-line hardware loop (GPS supervision, beacon planning, receive,
repeat, telemetry, panel, status line), config adoption and apply. The
`effect!` macro the state space work added is the tell: the hardware
loop's locals are a struct that was never declared, so effects that need
them had to expand in place.

**Recommendation.** Four modules. `ble.rs`: the server, `serve`,
`gatt_session`. `hardware.rs`: a `Hardware` struct owning the node, the
GPS, the card, the panel and the config, with `fn effect(&mut self,
Effect)` replacing the macro and `fn pass(&mut self)` as the loop body.
`beacon.rs`: the planner (`beacon_owed`, `beacon_at`, `last_beacon`, the
late-pass and re-plan rules) as a pure struct - it belongs in `proto/`
beside `hop`, where the model can walk it; the review found a bug in
exactly this logic that the model could not see because the planner is
inline. `gpsctl.rs`: the settings retry, the self-wake detector, the
grace-period diagnosis. `main.rs` keeps init and spawn.

### 2.2 The signals in `state.rs`

The serve loop is steered by two single-slot `Signal`s (`SLEEP_NOW_SIGNAL`,
`MODE_SIGNAL`), a cell beside one of them (`sleep_now_s`), four helpers to
keep the pair coherent (`request_sleep_now`, `take_sleep_now`,
`clear_sleep_now`, `sleep_now_pending`), and a convention that the loop
resets `MODE_SIGNAL` before it reads the settings. Two places wait on each
signal and the design note says they are never concurrent. The board
model reproduces this wake-whichever-waits glue by hand in `Board::write`,
which is the part of that model most likely to drift from the firmware.

**Recommendation.** One channel to the serve loop, `enum ServeCommand {
SleepNow(u32), ModeChanged }`, one consumer. `Serve::on_accept` takes
`Accepted::Command(ServeCommand)`; the cell, the reset convention and
three of the helpers go. The model's glue becomes a push onto a queue.

### 2.3 Two clocks

The hardware loop carries `now: u32` (wrapping, with a `due()` helper)
and `now_ms: u64` side by side. `node.rs`, `sdlog.rs` and `Blinker` use
the u32; `radio.rs`, `hop.rs`, `roster.rs` and `state.rs` the u64. The
repeat queue bridges them with `now.wrapping_add((start - now_ms.min(start))
as u32)`.

**Recommendation.** `u64` everywhere - embassy's `Instant` already is -
and delete `due()` and the wrapping arithmetic.

### 2.4 The dedup table and the repeat queue are untested

`node.rs` is pure logic on fixed arrays: the `(src, id)` table with its
TTL and oldest-eviction, the four-slot repeat queue. It is firmware-only,
so it has no tests, and it has a known edge the parser only comments on:
the 8-bit `id` wraps every 256 frames (4.3 min at 1 Hz) while
`dedup_ttl_s` may be set to an hour, at which point a node suppresses its
own later frames.

**Recommendation.** Move `Seen` and `Repeat` to `proto/src/lora.rs` as a
`Dedup` type, walk it with the explorer (every record/expire/wrap order),
and clamp `dedup_ttl_s` against `beacon_interval_s x 200` in the parser
with a line in the example file.

### 2.5 The park before a sleep is a request with a timeout

`enter_deep_sleep` on core 0 asks the hardware task on core 1 to park and
waits `tx_worst_case_ms + 1500`; on expiry it sleeps over whatever was
not parked. The board model excludes that path deliberately, because it
is a real-time property; what it costs when it happens is a whole sleep
interval at the receiver's 10 mA plus the radio's 5.7, and the only
evidence is one console line nobody is reading.

**Recommendation.** Count it: a `parks_missed` word in RTC RAM beside
`WAKE_COUNT`, reported on the boot line and in telemetry. And on expiry,
retry the park once before sleeping - the longest thing in the way is a
transmit, which is bounded.

### 2.6 The vendored `esp-radio`

844 KB vendored with three local changes (a `TxPower` re-export, the
modem sleep port from ESP-IDF, the teardown fix). Two migration guides in
the vendored tree say how fast upstream moves; every bump is a manual
re-port of a change that is not visible as a diff.

**Recommendation.** Keep `firmware/vendor/esp-radio.patch` against the
upstream tag in the repo, regenerated by a one-line script, so the delta
is reviewable and re-appliable. Upstreaming the modem sleep port is the
real answer.

### 2.7 The app's `MyApp`

4,688 lines with 22 BLE-related fields and a 300-line event drain inside
`drain_sources`. Every page reads and writes those fields directly. The
worker model's UI half (`press`, `drain`, the fence) is a model of this
code rather than the code, for the same reason as the transports.

**Recommendation.** A `BoardLink` struct in `src/board.rs` owning the 22
fields with `fn on_event(&mut self, BleEvent)` and `fn press(&mut self,
Intent)`, testable without egui. The model then drives it, and
`app.rs` shrinks by a page.

### 2.8 What has no tests

Firmware has none, by construction; the pattern is extraction to
`proto/`. Still untested after this session: `node.rs` (2.4), the beacon
planner (2.1), `sdlog.rs`'s buffering and flush policy, `flash.rs`'s OTA
slot arithmetic (testable against an in-memory `Storage`), the OLED
framebuffer and font, the compass's hard-iron correction.

### 2.9 Small things noticed, not fixed

- `Roster::expire` forgets at `>= TTL_MS`; `newest_position` at
  `> TTL_MS`. One millisecond apart.
- Slot numbers wrap at 2^20 and `slot % n` does not survive the wrap for
  `n` that does not divide 2^20. Twelve days free-running; never on GPS
  time. Worth a comment in `hop.rs`.
- `settings::save()` runs on every accepted write with `save = true`,
  inside a BLE session; the content compare makes an unchanged value
  free, a changed one is 40 ms with interrupts off and the other core
  parked.
- `sdlog` retries a missing card every 60 s, so a card inserted after
  boot takes up to a minute to appear.
- `Plan::retunes` is used only by its own test now that the driver
  compares carriers; fine as a pinned property, but say so or drop it.
- The Android `wait_cb` change has been type-checked only by eye; no NDK
  here.

## 3. In what order

| # | Item | Size | What it buys |
|---|---|---|---|
| 1 | App: shared `Session` phase machine and `Link` trait (1.6) | medium | resilience in one place; the model drives real code; deletes ~400 lines |
| 2 | Firmware: split `main.rs`, `Hardware` struct, beacon planner to `proto/` (2.1) | medium | the macro goes; the planner gets walked; each module has one job |
| 3 | One command channel to the serve loop (2.2) | small | four helpers and a convention go; the model's glue shrinks |
| 4 | Retire `hop_channels = 0` (1.4) | small | one scheduler; seven branches go |
| 5 | Delete the rail and the link's NAK vocabulary; rename WIO (1.2) | small | less to read, nothing to keep in step |
| 6 | One field table for the settings (1.1) | medium | five hand-kept layouts become one |
| 7 | `u64` time (2.3) | small | one clock |
| 8 | `Dedup` to `proto/`, walked (2.4) | small | the last untested radio logic |
| 9 | Generated wire constants for the tools (1.8) | small | drift becomes a build failure |
| 10 | Key table for the config (1.7) | medium | the example file, the parser and the editor agree by construction |
| 11 | `parks_missed` counter and one retry (2.5) | small | the one silent power failure becomes a number |
| 12 | Docs consolidation (1.9) | small, tedious | a README that is read |
| 13 | `BoardLink` in the app (2.7) | medium | the UI half of the model becomes real |
| 14 | The vendored patch as a patch (2.6) | small | upgrades stop being archaeology |

Items 1 through 5 are the ones that change how the next feature is
written; the rest are cleanup that can go one at a time.
