# The BLE service

What the board serves, what an app writes to it, the four modes and how the
board sleeps, and the bulk transfer that carries a config or a firmware
image. `ARCHITECTURE.md` draws a session end to end; `docs/POWER.md` prices
the settings.

## The service

Same service UUID as the ESP32-C3 beacon this firmware grew out of, so
gps-gui-rs discovers it unchanged - the app filters scans by service UUID,
so a board's name is display text and renaming one cannot lose it. On top
of the gps-proto position / config / ack characteristics the firmware adds
telemetry (LoRa RSSI/SNR, counters, SD and fix flags, parks missed, the BLE
link's own RSSI), remote
node positions and pings, a status/log characteristic (notify + read), the
board's name (read + notify), the current radio config (read + notify), and
a bulk write characteristic carrying either a TOML config or a firmware
image.

Everything declared `read` is written into the attribute table as well as
notified, so a central that never subscribes reads the current value rather
than the zeros the table was built with. That matters most for telemetry,
where the all-zero blob is not an obviously empty one: `secs_since_rx` of 0
means "heard from just now", and the value for never is 0xFFFF.

The settings characteristic (`c3a10009-...`, read + notify) carries the
device's current power and sleep configuration as one blob
(`midair_proto::ble::Settings`), so an app can populate its controls on
connect rather than assuming defaults. It is republished after every
config write - including values the device changed itself, such as a
clamped interval.

The radio-config characteristic (`c3a1000a-...`, read + notify) does the
same for the radio configuration - the `RADIO.CFG` settings as a
fixed-size `midair_proto::radiocfg::RadioConfig` snapshot. Otherwise the
config only ever travels *to* the board, so this is the one way to see what
a board is running. It is refreshed on connect and after every apply, and
reads back all-zero (which decodes to nothing) until the radio has been
initialized.

### Remote nodes

What other nodes report arrives on two characteristics: positions on
`c3a10007-...` (`[src, rssi i16le, 20-byte packet, age_s u16le]`) and pings
from nodes without a fix on `c3a1000b-...` (`[src, rssi i16le, flags,
uptime_s u16le, age_s u16le]`).

Both are notified when a report arrives rather than on the position notify
tick, and the firmware keeps the newest report from each of up to 8 nodes
rather than one report in total. That combination is what makes the stream
lossless: sampling one cached value on a timer delivered only whichever
node reported last before the tick, and silently dropped the rest. A node
holds one slot, so its newer report replaces its older one - including
replacing a position with a ping when it loses its fix, and the other way
round - and no node can crowd the others out by beaconing fast.

`age_s` is how long ago the board heard the report, on its own clock. A
live report reads 0; anything higher means the value was replayed rather
than just heard. On connect the board hands over every node it has heard
from in the last 30 minutes, so an app opens on the whole roster instead of
waiting for each node's next beacon; nodes quiet for longer than that are
forgotten rather than replayed as if they were still there. The age is
measured on arrival because the sender chooses which fields to spend air
time on and `time` is not one of the defaults.

What a node calls itself arrives on a third, `c3a1000d-...` (`[src, label
zero-padded to 16]`). It is the label that node carries in its own flash,
announced over LoRa on its own slow cadence, so the same board reads as
`sky-1` in a fleet list and `ws3gps-sky-1` in a scan list instead of as
`node 3`. It is notified when a name is first heard and when it changes -
never on a re-announcement of a name that has not - and replayed with the
roster on connect, since the next announcement may be twenty of the
sender's transmissions away. A node with no name reported has simply not
announced one yet, or has never been named; there is no placeholder value.

### Status lines

The firmware writes human-readable status lines to the USB console on
notable events - boot, GPS presence, fix acquired or lost, radio standby
and wake, config applied, a radio that restarted underneath the firmware,
the hop clock changing hands, a ping heard from another node - and notifies
the same text on the status/log characteristic, so an app sees the live
log. Lines are ASCII, up to `link::LOG_MAX` (128) bytes.

A periodic status line every 10 s carries the radio's chip mode and latched
device errors, the packet counters, the hop channel and stratum, GPS
sentence counts and fix state, the idle rate of the core and the free heap.
It exists because a quiet radio and a quiet GPS look identical
otherwise, and because the radio's status byte reports the mode it is in,
not whether it got there intact.

## Config writes

Config command ids (config characteristic, `[id, len, value]`). The five
durations are one table in the firmware (`midair_proto::session::KNOBS`),
which is also what the settings blob, the flash record and the `[power]`
section of the file are laid out from:

| Id | Value | Effect |
|-|-|-|
| `0x01` | u32 ms | position notify interval (gps-proto) |
| `0x10` | - | reserved: the rail switch of the two-MCU board this replaced; refused |
| `0x11` | u8 0/1 | radio to standby / back to receive |
| `0x12` | u8 0/1 | GPS backup mode (UBX-RXM-PMREQ / UART wake) |
| `0x13` | u32 s | deep-sleep wake-check interval, 5 s..5 min, 0 = off (the default) |
| `0x14` | u32 s | advertising window per wake check, 1 s..60 s (default 15 s). Stored's alone |
| `0x15` | u32 s | deep sleep **now** for this long, 5 s..5 min; 0 = use `0x13`. A command, not a setting |
| `0x16` | u32 s | BLE controller down between windows while tracking, 5 s..5 min, 0 = off |
| `0x17` | u8 | mode: 0 stored, 1 idle, 2 tracking, 3 listening. The one an app actually means |
| `0x18` | u32 s | how long idle lasts before the board stores itself, 10 s..1 h, 0 = never (the default) |
| `0x19` | ASCII | board name label, up to 15 bytes; empty clears it |
| `0x1A` | u32 s | BLE up between off periods while tracking, 1 s..60 s (default 15 s). Tracking's alone |

The board clamps and the ack carries the value it stored, so an app reports
what the board is running rather than what it was asked for. The same ids
are the USB console's `board-set` commands.

### Board names

A board advertises as `ws3gps-<label>`, and one that has never been named
falls back to `ws3gps-<xxxx>` from the last two octets of its BLE address -
so two boards out of the same box are already told apart in a scan list,
and a fleet reads as `ws3gps-ground-1`, `ws3gps-sky-1` without anything
having to be configured first.

```
pixi run board-set name sky-1     # over USB, at the bench
pixi run board-set name ""        # back to the address-derived name
pixi run board-info               # what this board is called, and its address
```

The prefix is a firmware constant rather than part of the label, so a board
cannot be named something unrecognizable: whatever it is called, a generic
scanner can be searched by `ws3gps`. Labels take ASCII letters, digits, `-`
and `_`; anything else is rejected rather than sanitized.

The label is stored with the settings that decide reachability - RTC RAM,
mirrored to the `nvs` partition - rather than with the radio config,
because a wake check advertises before anything has read that. It survives
a deep sleep,
a reflash and a flat cell, and a board updated from firmware that predates
names reads back as unnamed rather than as unreadable. `pixi run board-wipe`
is what removes it, along with the rest of the settings; see below.

Four surfaces carry it, and they catch up at different speeds:

| Surface | When it updates |
|-|-|
| scan response (`CompleteLocalName`) | the next advertising window - the one on the air was handed to the controller before the write |
| name characteristic (`c3a1000c-...`, read + notify) | immediately, on the connection that renamed the board |
| GAP device name (`0x2A00`) | the next boot; the attribute table is built once per power cycle |
| the LoRa network | the board's next transmission; other boards then call it that on their own consoles, panels and node-name characteristic |

The ack for `0x19` carries the stored *length*, not the label: an ack has
four value bytes and no name fits in one. What a board is actually called
comes back on `c3a1000c-...`.

## Modes

The board is in one of four, and `0x17` is how it moves between them.

| Mode | What is up | Its knob | Persisted |
|-|-|-|-|
| **stored** | nothing, bar a wake check on a cadence: chip asleep, GPS in backup, radio in cold sleep, card unmounted, panel dark | `0x13` cadence, `0x14` window | yes |
| **idle** | BLE only - connectable, but the GPS stays in backup and the radio stays down | `0x18` timeout, off by default | no, deliberately |
| **tracking** | everything: GPS acquiring, beacons out, receiver listening, card logging | `0x16` modem off, `0x1A` modem on | yes |
| **listening** | the node beside the phone: GPS acquiring, receiver listening, card logging, BLE up throughout - and nothing transmitted | none | yes |

A cold boot lands in **idle** unless nvs says tracking or listening. That is
the rescue window: a board recovered from a flat cell, or one just flashed,
is reachable - and stays reachable, unless `0x18` has been set - and only an
explicit stored `tracking` puts a board back on the air by itself. Tracking
and listening are the modes that survive a brownout on the object, which is
the one time they must.

Idle never reaches flash - a board that came back from a reset still
believing it was idle would sit at awake current with nobody coming - so
what an app reads back as "idle" is the live RTC copy, and the record behind
it says stored.

**A connect during a wake check is a doorbell, not a leash.** The connect
*attempt* promotes the board to idle with the timeout armed, so the app can
take its time instead of having to catch the window, connect, and hold on.
A misfire costs one idle timeout of awake current - or, with the timeout
off, a board that stays idle until it is told to store itself again.

Each duty-cycle knob belongs to exactly one mode, which is what stops them
competing: a wake check wants the shortest window a phone can still catch,
a tracker one long enough to connect, read the roster and let go.

The two override flags (`0x11`, `0x12`) only mean anything inside a
tracking posture. A mode commanded on top of one lands on the flag - a
tracker with `0x12` set comes up with its receiver in backup - and outside
tracking they are stored and honored when tracking is next commanded.

What every one of these does in every order is `midair_proto::session`
and `midair_proto::posture`, walked exhaustively (`docs/STATESPACE.md`).

## Sleep

Sleep is off by default (`0x13` = 0), which is what an unconfigured board
does: land in idle at boot and stay there, advertising continuously. So is
the idle timeout (`0x18` = 0), and storing out of idle needs both: with no
timeout nothing fires, and with no cadence to sleep on the timeout has
nowhere to send the board. `0x13 = 0` is therefore also "never store this
board *on its own*", and it is the bench setting. Being told `0x17 = 0`
still stores it: somebody asked for that one, so it borrows the 5 min
ceiling rather than reading the missing cadence as a refusal.

`0x13` is the stored cadence. While the board is stored it deep-sleeps for
that long, wakes into a check, advertises for `0x14` seconds (one long D2
blink), and goes back down if nobody came. Both persist until changed - a
connect does not clear them, so an unattended board holds its cadence
indefinitely. A board that is *tracking* ignores `0x13` entirely: deep
sleep stops the beacon, the logging and the listening, and a tracker doing
that is not tracking.

The window is the more useful knob of the two, because shortening it does
not make the board any slower to reach - a 5 s window at a 60 s interval is
still four times the battery life of the 15 s default, and still gets you a
wake every minute. What it costs is margin: the window has to overlap a
phone's scan. `0x14` has no "off" - a 0 clamps up to 1 s. A window changed
over BLE applies from the next wake, not the current one.

### Sleeping on command

`0x13` and `0x14` describe when the board sleeps *on its own*, and both only
fire when an advertising window expires with nobody connected. `0x15` is
the direct one, and `0x17 = 0` (stored) is the same path with the mode
written first, so the board comes back into a wake check rather than into
whatever it was doing. The board acks, finishes paying out what it owes the
connection, and goes. `0x15` stores nothing, does not touch `0x13`, and
comes back to exactly what it was configured for. A value of 0 means "for
the `0x13` interval", falling back to 60 s when sleep is off, and the ack
carries the resolved seconds.

**The disconnect is the command working.** The board stops being contactable
the moment it sleeps; the app's Bluetooth page says so before the link
drops and again after it does. The same command is on the USB console
(`pixi run board-sleep`, optionally `--seconds N`), where the serial port
disappearing and coming back is a whole sleep cycle observed without a
phone.

Every command that ends a wait - a nap, a moved mode, from either transport
- reaches the serve loop on one channel, whichever wait it is in: the wait
for a central, the connected session, or the modem's off period.

### What a sleep does

Before it sleeps the board parks everything it can reach. The radio goes to
cold sleep (5.5 mA of continuous RX against 9.3 uA), the panel is blanked,
and the receiver is sent into PMREQ backup
- re-issued on every park, because after a reset the firmware's belief about
the module is worth nothing. The hardware loop declines to start a beacon
while a sleep is pending, and the sleep path waits for the park for the
running config's own worst-case transmit plus a margin, twice, before it
sleeps over one that did not finish. A sleep that did is counted in RTC
RAM: `parks missed N` on the boot line, and `parks_missed` in the
telemetry. Each one is an interval spent with the receiver or the radio
still drawing, which nothing else could show.

Two pads are held across the sleep. NSS (GPIO21), because the SX1262 leaves
cold sleep on a falling edge and the S3 releases every pad it is not holding.
UART TX (GPIO2), because a floating edge there is UART traffic to a receiver
that wakes on it, which would undo the backup the park just asked for. SD CS
(GPIO44) has the same problem and no fix in firmware - the S3's RTC pins stop
at 21.

A wake from deep sleep prints `woke from deep sleep #N (slept M s, parks
missed K)`, counted in RTC RAM since the last cold boot. That line is the
difference between a board on its cadence and a board resetting in a loop:
deep sleep is a full reset, so without the counter the two produce an
identical boot banner.

The mode, the five durations, the two flags and the name are held in RTC
fast RAM and mirrored to the `nvs` flash partition, so they survive deep
sleep *and* a flat battery. Wake checks run from the RTC RAM copy; every
other boot reads flash and takes what it finds.

That second half matters more than it sounds. RTC RAM survives every reset
short of a power cycle - the reset button, a panic, the reset a flashing
tool issues - so an earlier firmware that trusted the copy whenever it was
there came back from `espflash erase-flash` still named, and wrote the
name straight back into the flash that had just been erased. Now only a
deep-sleep wake keeps the copy. A boot after an erase prints `nvs: nothing
stored, settings are defaults (an rtc copy from before the reset is
dropped)`, which is the erase having stuck.

### Wiping a board

`pixi run board-wipe` sends the USB console's `WIPE` command: the board
erases its settings record, its name and its stored radio config, drops the
RTC RAM copy, acks, and restarts on its defaults. The firmware and the OTA
slots are untouched. `pixi run board-wipe --flash` erases every byte of
flash instead and rebuilds and reflashes the firmware - the reset for a
board that has been through several firmwares.

### What the board wrote down

`pixi run board-log` sends the USB console's `EVLOG` command, one record
per round trip, newest first. The board keeps a ring of records about
itself in its own flash: every boot with its reset reason and what the
boot before it left, every panic with its message and location, every
task the monitor found past its heartbeat bound with the phase it stopped
in, and the faults worth a line - a radio that restarted underneath the
firmware, a BLE controller that would not come up, a park that did not
finish before a sleep. A board found dark in a field is plugged in and
asked. The log outlives a reflash and a wipe; `--clear` erases it.

**Deep sleep has no wake source but the timer.** Nothing over the air can
interrupt it: the radio is off, and there is no GPIO or button wake. The
5 min ceiling on `0x13` is what bounds that - the longest the board can
ever be unreachable, short of a physical reset, which lands it in idle for a
whole `0x18` timeout. The wake is timed by the uncalibrated RC slow clock,
so the interval drifts - it paces a wake check, not a schedule.

## Bulk transfer: a config or a firmware image

The bulk characteristic (`c3a10006-...`) carries a transfer of either kind,
op by op - begin, data chunks, end - each acked on the ack characteristic
with id `ACK_ID_BULK` before the next is sent, so a write burst cannot
outrun the transfer buffer on the far side. One transfer at a time,
whichever transport opened it; a transfer whose host walks away is expired
by the board, and a transfer a phone was midway through does not outlive
its connection. The USB console carries the same ops in its framing.

A config (`kind = 1`) is parsed on the board at the end and, if it parses,
applied live and written to both stores (`docs/RADIO.md`).

A firmware image (`kind = 3`) goes into whichever of the two application
slots the board is not running from, streamed a flash sector at a time.
Nothing is pointed at it until the whole transfer has arrived and its CRC
matched; then `otadata` is moved, the board reboots, and the new firmware
marks itself confirmed once it has booted far enough to run `main`. A
bootloader built with rollback enabled reverts to the previous slot if that
never happens, so an image that cannot start costs a reboot rather than a
board.

```sh
cd tools && pixi run board-ota          # builds ../firmware and pushes the image
pixi run board-ota --image firmware.bin # or send one you already have
```

An image must be an ESP-IDF *application image*, not the ELF - `board-ota`
checks the 0xE9 magic and refuses the ELF rather than letting the board
write something it cannot boot. With no `--image` it runs the conversion
(`espflash save-image`) itself. Each sector write holds interrupts off for
tens of milliseconds, which a BLE connection rides out but does notice; USB
is the smoother path and the one the tool takes. `cargo run --release` over
USB still works and is what puts a board onto the two-slot partition table
in the first place.

Kind 2 was the two-MCU board's STM32 image and is retired rather than
reused, so an old tool pushing one at this firmware is rejected instead of
misread into an app slot.
