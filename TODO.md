# TODO

Done items move to `TODO_complete.md`.

## Before the beacon transmits

Everything below is written and builds; almost none of it has been run.
That is now the whole of the remaining risk, and the radio is where it
concentrates.

**Run the radio against an existing node.** The air format did not change,
so a ported board has to talk to an unported one - that is the test that
says the port is real. The beacon exists now, so the DIO2/DIO3 fix is
finally exercisable: a wrong value there transmits into an isolated port
and destroys the module, and no amount of reading the code substitutes for
watching the first transmission on a spectrum analyzer or a dummy load.

Note the sync word is a flag day. Nodes on different sync words cannot hear
each other at all, so reflash every node before testing - a partially
updated fleet looks exactly like a range problem.

**Confirm the eFuse state on a real module**, particularly `VDD_SPI_FORCE`:
three of the four SD lines sit on ESP32-S3 strapping pins.

**Watch the first OTA on a board you can reach.** The write path is guarded
twice against targeting the running slot, but `partitions.csv` and the
rollback behavior of espflash's bundled bootloader are both unverified.
Keep a USB cable on the first one.

## The two measurements the mode work is waiting on

The mode work (its plan is retired into `TODO_complete.md`) is implemented
except for its measurements, and they are what decide whether the Stored
mode is worth anything. Both readings need a board on a meter:

1. **The GPS in backup, on its own.** `--features iso-gps-backup` puts the
   receiver into a timed PMREQ 20 s after boot; read the meter across the
   gap. That is the number the whole mode hangs on, because `V_BCKP` is
   unfed here and backup-on-`VCC`-alone has never been priced.
2. **The Stored floor.** Set a cadence and let it sleep: `board-set sleep 60`,
   then `board-set mode stored`. The park path now takes the receiver into
   backup and the radio into cold sleep, and holds
   both NSS and UART TX across the sleep - so this reading is the floor
   itself rather than the old ~30 mA of ungated GPS. Low single-digit
   milliamps means storage life in weeks; tens of milliamps means the plan
   shrinks to Idle plus promotion, and it is a board finding for
   `docs/BOARD-V1-ISSUES.md`.

Two more that are now worth taking while a meter is attached: what **Idle**
costs (predicted ~90 mA, BLE-dominated, never measured) and how long a
**wake-check boot** takes, since every check pays a full init and the
cadence cannot be tuned against a number nobody has.

Also unproven on hardware: whether the receiver comes back at all after a
park, and what TTFF costs when it does.

Two things the first bench run of the modes changed (2026-08-31):

- The receiver measures **~10 mA**, not the 25-31 the documents carry -
  taken by toggling `gps_sleep`, so confirm it with `--features
  iso-gps-backup` before rewriting the floor around it. Everything
  downstream of "the GPS is ~30 mA of the floor" is now suspect.
- `UBX-RXM-PMREQ` was missing the `force` flag the MAX-M10N requires for
  software standby, so parks held only sometimes. Fixed; the park path now
  prints `gps: talking at park - the last park did not hold` when it
  catches one. **Any floor reading taken before this is one of two
  numbers**, so retake them.

## Bench work

**Watch two boards learn each other's names.** Name one board and leave
the other unnamed (`pixi run board-set name sky-1`), and the named one
should announce with its first transmission: the other's console says
`node N is sky-1` once, its ping and position lines carry `N (sky-1)`
from then on, and the compass screen says `sky-1` in place of `nN`. Then
the two things only a bench shows: a rename while both are running
reaches the other board on the next transmission, and the announcement
that costs a beacon every twentieth transmission is not enough to be
visible as a gap in the position stream. A board that is never named must
put nothing extra on the air at all.


**Flash the watchdog, the panic path and the event log, and provoke
each once.** None of it has run on a board. `docs/STATESPACE.md` bench
items 8 and 9 are the recipe: a clean boot's `board-log`, a deliberate
`panic!` after the attribute server attaches with a phone connecting, a
deliberate `loop {}` in the hardware loop's pass. Each must come back as a
reset within its bound with the right record - `panic` with file and line,
`stall` with `hardware loop` and the phase - and never as a board that
stays down. Then the readings that matter: a tracker left beaconing with
a phone connecting now and then shows one `boot` record a day, and no
`stall` in `card` or `ble init` at the same uptime on every boot (a
bound too short for what the hardware takes). Finally the freeze itself:
a phone connecting to a beaconing tracker, repeatedly, until the log says
what it was. The freeze is not diagnosed - the log is what diagnoses it.

**Flash the state space work and the audit's rework.** `docs/STATESPACE.md`
lists the changes the exhaustive models forced and `docs/SYSTEM-AUDIT.md`
the fourteen items that followed, none of them run on a board: the serve
loop and the hardware task drive `session::Serve` and `posture::Posture`,
the beacon is planned by `beacon::Planner`, the serve loop takes its
commands from one channel, the settings are one knob table, the hardware
loop is a struct in `hardware.rs`, and time is 64-bit throughout. Watch
for: beacons landing in the node's own turn every second (`beacon
position` on the verbose console, `rx` climbing by one a second on the
other board); `serve: command queue full` never printing; `parks missed 0`
on every wake line; `board-set` and `board-config` still answering. Then: `tracking: node N (leaf), gps up, radio up` after a
`CFG_MODE tracking` - and `gps in backup` or `radio standby` in that line
when the matching override flag is set; `radio: standby` / `gps: backup
mode` only while tracking; a wake check that a phone connects to coming
up `promoted to idle`; and `sleep: park did not finish in time` never
printing on a commanded store.

**Every state, on a bench.** The models say what the firmware decides;
what they cannot say is what the hardware does with it. The three places
to look are the ones where the model abstracts: a deep sleep entered while
a transmit is in flight (the park waits it out - check the meter afterwards
for a radio left in receive), a config push while the radio is in standby
(the apply re-inits the radio and the posture puts it back), and a
`CFG_GPS_SLEEP=0` with the radio in standby (the receiver should be polled
and report a fix).


**Measure the BLE modem sleep.** It is implemented now - a port of
ESP-IDF's sequence into the vendored esp-radio, since upstream ships the
callbacks as `todo!()` - and not a milliamp of it has been read. Flash the
default build and `--features iso-ble-no-modem-sleep`, and take both
advertising and with a phone connected and idle. The difference is the
answer to the largest open question in `docs/POWER.md`, it is the number
Idle's ~90 mA estimate rests on, and it is the one lever that works while a
phone is attached. Watch that a connection survives it, too: the wake path
that hands the controller an HCI packet is the part with the least margin.
The console prints `ble modem sleep on` at the first window if the
controller really took it, so a run that says `off` is a finding rather
than a measurement.

Still open on the advertising window, and not fixable from the firmware
side: a connect attempt that is *in flight* when the window expires.
`with_timeout(left, advertiser.accept())` cancels the accept, and
trouble-host reports that as "nobody came" rather than "someone was halfway
in", so the board goes dark on a phone that was seconds from connecting. A
returned-but-failed handshake is now held open for the retry
(`Window::after_connect_attempt`); this one needs the host stack to say
that a connection is being made.

Then work the levers list at the end of `docs/POWER.md`, which is ordered
by what it is worth. The board is measured - ~126 mA awake, a 60 mA floor with BLE dark -
so the open items are levers, not unknowns. The next one is restoring the
Wi-Fi clock and power-down bits after the BLE connector drops.

Soak the BLE duty cycle. `esp_radio::init` and `BleConnector::new` now run
once per window rather than once at boot, thousands of times a day at a
45 s cycle, and both are `expect`s on a heap that the controller allocates
from every cycle. Leave a board running overnight with `mode tracking` and
`ble-off 30`, and check the wake counter and the free heap.

Soak the modes: a week of Stored on a cell with the wake counter and heap
checked, then a tracked walk that ends in `mode stored` from the phone. The
promotion is the part to watch - a wake check that is connected to has to
come up idle and stay there for its timeout, and a misfire is the failure
that costs battery rather than reachability.

Check whether an OTA over BLE survives its own flash writes. Each sector
takes tens of milliseconds with interrupts off, which should cost a
connection event rather than the connection - "should" being the word doing
the work.

## Radio

The slot clock is in and untested on hardware. `hop_channels` now defaults
to `1`, so the default build runs the clock, the turns and the sync word on
one carrier and never retunes - see the addendum in `docs/RADIO-AUDIT.md`
for why. The first thing to watch is the console: `hop: clock on gps time`
on a board with a fix, then `hop: clock from node N` on one without, and
two nodes landing in different halves of the second.

Then run it again with `hop_channels = 50` pushed, which is the path that
actually retunes and the one the fifty-channel numbers in the audit
describe. Watch the channel index move in the status line, and time a base
station with no fix from boot to first frame - the model says 61 s there
against 4 s at the default, and that gap is the whole argument for the
default being what it is.

Price hopping against real interference. It is the one thing the new
default gives up and the simulator has nothing to say about it: no
interferers are modelled. Two boards in a band with a live 900 MHz talker
nearby, at `hop_channels` 1 and 50, is the experiment.

Faster join for a node with no clock, if the fifty-channel plan is ever
the one deployed. An RSSI sweep (retune, settle, read, about a millisecond
a channel) finds a strong signal in one pass where the blind receiver
waits a cycle per coincidence; it cannot see a signal under the noise
floor, so it complements the wait rather than replacing it. The other
reference is the phone: its GPS time over BLE would make the board on the
desk stratum 0 without a fix, and that one helps at any channel count.

Hop statistics in telemetry: frames heard per channel, clock corrections
applied and their size. Without them a receiver that is a little out of
step looks like a range problem.

Wake-on-radio: `SetRxDutyCycle` (0x94). The radio cycles sleep/RX on its own
and only wakes the MCU when a real preamble arrives, instead of holding
continuous RX. Biggest battery win available on a leaf that mostly listens.
Needs the receive loop restructured and the sleep/RX ratio picked against
the beacon interval: too long asleep and a whole broadcast passes unheard,
so the two have to be chosen together.

CAD auto-transitions: `SetCadParams` (0x88) with ExitMode. Detect a preamble
and drop straight into RX to catch the payload, or find the channel clear
and go straight to TX. Cheaper than a full RX window for listen-before-talk,
and it would give repeaters a real collision check before forwarding rather
than the current random jitter.

Reduce packet size by encoding GPS position as an offset from a reference
point (given in the TOML or over BLE) at a chosen precision. Worth doing now
that payloads go out at their true length rather than padded to 32 bytes.

## GPS

Ultra-high sensitivity mode. Navigation input filters. More low power.

Dynamic model switching: Airborne <2g to Stationary when on the ground, on a
timer.

## Ideas

Wi-Fi server, hotspot style. The C6 had Wi-Fi too, but with one MCU and 8 MB
of PSRAM it no longer competes with the link task for room. It would want a
partition of its own, which is a `partitions.csv` change and so a reflash
rather than an OTA - worth deciding before a fleet is deployed.

A listener dongle, possibly with a USB bridge.

Beeper.

## Open questions

Does flashing the firmware keep the node address? (It should: the config
record lives in `nvs`, which the runner does not erase - `--erase-parts
otadata` is the whole erase list. Untested on hardware.)

**The SD card, later.** Removed 2026-09-11 (`docs/HARDWARE.md` says why);
the driver is in the history before that date. If it comes back, its mount
and its FAT walks have to beat for the hardware loop from inside the bus,
or run somewhere the loop does not wait on.

Will a sleeping board ever be connected to if an awake board is nearby?

The `PMode::Boost` value: the WIO-E5 build wrote 0x97 to the RX gain
register, which its HAL documented as best sensitivity, but Semtech's
datasheet documents only 0x94 and 0x96. The S3 port writes the register
directly and uses the documented 0x96. With the WIO firmware gone there is
no longer an A/B to run, so this is settled unless RM0453 says otherwise.

## Board changes

These need a respin, not a flash.

Smaller?
Probably need to drop a module.

Software toggle shunt? Or charging IC that handles all of this.

Battery bypass LDO? (Just esp?) USB must not.

Add power switch? 

Add current monitor? (INA219/226?)

Add an LP-GPIO wake button so a deep sleep can be interrupted. Deep sleep
is timer-only, so the 5 min clamp on 0x13 is the only thing keeping the
board reachable - and with the modes in, that clamp is also the worst case
for reaching a *stored* board, which is now the state it spends its life in.

Move SD CS off GPIO44. It is outside the S3's RTC range (0-21), so unlike
NSS and UART TX it cannot be pad-held through a deep sleep and floats for
the whole interval. Whether that costs anything is one of the meter
questions above.

Route the module's Wi-Fi/BT RF port to an antenna. It reaches test point
BLE1 and stops there on the board as drawn, so the 2.4 GHz side has no
antenna despite the module bringing the port out.

Battery sense divider. There is none, so telemetry cannot report cell
voltage without a board change.

Route GPS EXTINT to the MCU, and TIMEPULSE for PPS discipline. Neither is
connected today; backup mode still wakes on UART traffic, but PPS is simply
unavailable.


Add `Keep connected button` constantly attempts to reconnect if device disconnects during active connection. 

Test feature display state to OLED display. The panel now keeps refreshing
in idle but says nothing about which mode the board is in, which is the one
thing a board sitting on a desk doing nothing needs to be able to tell you.

Check bluetooth docs for lower power state management. Wake without advertising?

Confirm in app for stored mode.

- Boot into idle.
- Quick double flash on rx.
- Log gps data to flash. Can load from usb or over BLE.
- Link in status should reflect current BLE status.