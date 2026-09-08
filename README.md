# telemetry-in-midair-rs
[Kicad Board](https://github.com/tmpk13/telemetry-in-midair) https://github.com/tmpk13/telemetry-in-midair

GPS tracker board firmware: one Seeed Wio-S3 module (ESP32-S3R8 + SX1262)
reads a MAX-M10 GPS, transmits positions over 915 MHz LoRa, logs to SD, and
serves everything over BLE to the gps-gui-rs app. See `PLAN.md` for the
intent and `ARCHITECTURE.md` for the UML views.

## Layout

| Directory | What | Target |
|-|-|-|
| `proto/` | Shared no_std protocol crate: LoRa payloads, BLE extensions, `RADIO.CFG` parser, USB bulk framing. Host-testable (`cargo test`). | any |
| `firmware/` | Wio-S3 firmware (embassy + trouble BLE): radio, GPS, SD and the GATT service. | `xtensa-esp32s3-none-elf` (`esp` channel) |
| `tools/` | Host tools (Python/pixi): push a radio config, push a firmware image, read a board's BLE address, simulate the radio network. | host |
| `docs/` | Deep dives and history: the power investigation and its audit, the port record, the module datasheet, the V1 board's issues. | - |

Depends on the sibling repo `../gps-proto` for the BLE position protocol
and NMEA parsing (shared with `../esp32c3-gps` and `../gps-gui-rs`).

## Build and flash

The ESP32-S3 is an Xtensa part, not RISC-V, so it needs the `esp` toolchain
channel rather than `stable`:

```sh
# once per machine - provides the `esp` channel and the Xtensa target
espup install

# protocol tests (host)
cd proto && cargo test

# firmware: builds, flashes over USB Serial/JTAG, and stays on the console
cd firmware && cargo run --release
```

The console is on the USB Serial/JTAG port (GPIO19/20 to the USB-C
connector), not UART0 - GPIO43 is UART0_TX on this board and drives the D5
LED, so expect the ROM bootloader's own log to flicker it on every reset.

`cargo run` flashes through `firmware/partitions.csv`, which has two application
slots rather than one factory app - that is what OTA needs somewhere to
write. It also erases `otadata` on every flash, so the image just written is
the one that boots; without that, a board that had taken an over-the-air
update would keep booting the other slot and a fresh flash would look like
it had not taken.

To give a board a fixed BLE address instead of the per-chip one it derives
from its eFuse MAC:

```sh
cd tools && pixi run gen-ble-address        # prints e.g. FF:C6:A1:53:50:47
cd ../firmware && BLE_ADDRESS=FF:C6:A1:53:50:47 cargo run --release
```

`build.rs` rejects anything that is not a static-random address, so a bad
one fails the build rather than flashing a radio that will not advertise.

## Configuring a board

`RADIO.example.toml` documents every setting. It is a reference, not a card
file - the firmware reads at most 1024 bytes of config and the descriptions
put it well over that, so the tool strips them before sending.

```sh
cd tools
pixi run wio-config --address 3                  # applied live, saved to SD
pixi run wio-config --set role=rx_only           # any key, repeatable
pixi run wio-config --set verbose=false          # quiet the console
pixi run wio-config --address 3 --dry-run --save ../RADIO.CFG   # card file
```

A push replaces the whole config: keys absent from what is sent revert to
their defaults rather than keeping the board's current values. The
`wio-config` tool cannot read a config back off a board, so start from a
file holding your settings (`--file`) if the board is not on stock ones.
Over BLE the board *does* report its current config (see below), so the
gps-gui-rs app can read it back - its Radio page has a "Load from board"
that fills the editor from the board itself.

A pushed config is written back to the card as `RADIO.CFG`, which is where
it survives a power cycle - so editing that file on a computer and pushing
over USB are the same thing arriving two ways. It also goes into a backup
record in the board's own flash, so a board with no card (or a failed one)
keeps what it was told. The card wins at boot, and a boot that reads one
refreshes the backup, so the two cannot drift apart while a card is in the
slot. A push that reached neither store says so on the status line, and
`wio-config` exits non-zero when that happens.

The GPS and SD sit directly on +3V3 on this board, so there is no rail to
raise before they answer.

## Radio configuration

The firmware loads `RADIO.CFG` from the SD card at boot; the same file can be
pushed over BLE (bulk characteristic) at runtime, which also rewrites the SD
copy and the flash backup (see [Where a config
lives](#where-a-config-lives)). All keys are optional; defaults in
parentheses:

```toml
[radio]
frequency_hz = 915000000   # (915 MHz)
spreading_factor = 12      # 5-12 (12)
bandwidth_khz = 500        # 62|125|250|500 (500)
coding_rate = 5            # 4/5..4/8 (5)
power_dbm = 22             # -9..22 (22)
rx_boost = true            # boosted RX gain (true)
hop_channels = 1           # channels in the plan; 1 = slot clock, no hop (1)
hop_step_khz = 500         # channel spacing, 25-5000 (500)
hop_dwell_ms = 1000        # time on each channel, 100-10000 (1000)
dcdc_enabled = true        # internal DC-DC instead of LDO (true)
tcxo_volts = "3.3"         # TCXO supply; board hardware, not a tuning knob
                           #   (also the antenna switch VDD - floored at 2.7)
tcxo_startup_ms = 10       # TCXO settling wait, 1-1000 (10)

[network]
address = 1                # 1-255 (1)
role = "leaf"              # leaf | repeater | tx_only | rx_only (leaf)
max_hops = 1               # retransmissions allowed, 0-8 (1)
dedup_ttl_s = 3            # how long a (sender, id) pair is remembered (3)

[beacon]
interval_s = 1             # position period with a fix, 0 = silent (1)
ping_interval_s = 5        # no-fix ping period, 0 = no pings (5)
fields = "lat,lon"         # what each broadcast carries (lat,lon); also
                           #   altitude|speed|course|sats|time

[sd]
sd_enabled = true          # use the SD card at all (true)

[debug]
verbose = true             # detailed console logging (true)

[gps]                       # MAX-M10 receiver (UBX-CFG-VALSET, RAM layer)
gps_enabled = true         # (true)
glonass_enabled = false    # (false); M10 tracks a limited concurrent set
galileo_enabled = true     # (true)
beidou_enabled = true      # (true)
qzss_enabled = true        # (true)
sbas_enabled = true        # (true)
power_mode = "full"        # full|psmoo|psmct (full)
meas_rate_ms = 1000        # measurement/nav period, 25-10000 (1000)
dynamic_model = "portable" # portable|stationary|pedestrian|automotive|
                           #   sea|airborne1g|airborne2g|airborne4g (portable)
```

### Where a config lives

Two stores hold the same text, and the card is the one people edit:

| Store | What it is | Written when |
|-|-|-|
| `RADIO.CFG` on the card | the file, in the card root, 8.3 name, at most 1024 bytes | a config is pushed |
| a record in the `nvs` partition | the same text behind a length and a crc, one sector past the settings record | a config is pushed, and at every boot that reads the card |

At boot the card wins - pulling it to edit `RADIO.CFG` on a computer has to
do what it looks like - and a boot that reads one refreshes the backup, so a
card edited offline is what both stores hold from then on. A board with no
card, or with a card that has failed or gone unreadable, comes up on the
backup instead of on firmware defaults. That is what the backup is for: the
node address is the one setting nothing can guess back, since two senders
sharing an address are mutually deaf.

An invalid `RADIO.CFG` falls through to the backup rather than to defaults,
and says so on the console. A record that fails its crc reads as nothing
stored, which is also what an interrupted write leaves behind - the length
and the crc sit in front of the text precisely so that a half-written record
cannot be read as a whole one.

The backup is not erased by `cargo run` - only `otadata` is - so a reflash
keeps a board's address. Erasing the whole chip does take it, along with the
duty cycle and the board's name, since both records live in `nvs`.

### Modulation

The default is SF12 at 500 kHz on a single carrier at 915 MHz, with the
network's slot clock running under it.

That band gives a transmitter two ways to be legal, and the default takes
the simpler one. A 500 kHz signal counts as a digital modulation and may
hold one carrier indefinitely, with no dwell or duty cycle ceiling, so
`interval_s = 1` is legal without hopping anywhere. Anything narrower has
to hop: at least 50 channels, no more than 0.4 s on any one of them per
20 s, and receivers hopping in step with the sender.

Hopping is therefore a setting rather than the default. Raising
`hop_channels` buys two things - the narrower modulations, and diversity,
since a fade or an interferer parked on one carrier then costs one beacon
in `hop_channels` rather than every beacon. It does not buy link budget:
the 0.4 s dwell caps time on air, time on air is what buys sensitivity,
and the best a legal hopped plan manages against the default is about a
decibel (SF10 at 125 kHz, 330 ms on air). What it costs is below.

SF12 at 500 kHz keeps the frame inside a slot. The default beacon is 289 ms
on air, and the same frame at BW125 is 1.15 s - past the 0.4 s a visit may
occupy a channel, and past the 800 ms window a 1 s slot leaves after its
guards. The narrower modulations are still config keys, and the estimate on
the app's Radio page says whether a frame fits; a frame that does not is
still sent, since receivers hold their hop for a frame in progress, but it
is one channel held longer than a hop is meant to be.

Somewhere the band rules differ - EU 868, say, where there is no minimum
bandwidth and the constraint is a duty cycle - `spreading_factor = 9`,
`bandwidth_khz = 62` and `power_dbm = 14` get the narrow modulation back,
and `hop_channels = 50` makes it legal where hopping is what is asked for.

### The slot clock

Every node keeps a slot clock: time cut into `hop_dwell_ms` slots, and in
slot `s` every node is on the `s mod n`-th channel of a permutation of all
`n`, reshuffled every cycle of `n` slots from the cycle number. At the
default `hop_channels = 1` that permutation has one entry and nothing ever
retunes; what is left is the clock, which is what gives each node its own
turn to transmit in. `hop_channels = 0` is the other setting, and it is
not the same one: it takes the clock away with the plan, leaving each node
on its own interval with random jitter - enough for one transmitter, not
for two.

With `hop_channels` raised, a node that beacons every slot uses each
channel once a cycle; one that beacons every twentieth slot still lands
somewhere different each time, because the order under it changes. The
reshuffle is also what lets two nodes that do not agree on the time find
each other at all: their channels coincide in about one slot in `n`, where
a fixed order at a fixed offset never meets.

The clock has three sources, in order of trust, and every frame says which
its sender is on:

- **GPS time**, for a node with a fix: the slot is the second of the day.
  Every node with a fix agrees without hearing anyone. Stratum 0.
- **A heard frame.** A hopped frame carries a four-byte sync word - the
  slot it went out in, how far into the slot, and the sender's stratum. The
  frame's length and modulation fix its time on air, so the receiver knows
  to a few milliseconds when the sender's slot began, and adopts the clock
  if it is better than its own: a lower stratum, or the same stratum from a
  lower address. It then sits one stratum below.
- **Nothing.** A node with neither hops on its own free-running clock at
  stratum 15, so a follower can still be in step with it, and takes the
  first better clock it hears.

A clock nobody has refreshed ages one stratum every ten minutes, so a
network cut off from its GPS reference reorganizes around the lowest
address instead of every node insisting it is still stratum 1, and a node
that gets a fix back outranks everyone again at once. Crystal drift is a
few milliseconds per ten minutes, against a 100 ms guard at each end of the
slot, so an aged clock is still a usable one.

What hopping costs is the join, and it is why the default plan is one
channel wide. A node that knows nobody's clock hears the network only when
its channel happens to coincide, so it waits on average `hop_channels x
interval / nodes transmitting` seconds - 50 s across fifty channels with
one node beaconing every second, 250 s if that node has no fix and is
pinging every five. It goes on paying after the join, too: a follower is a
stratum below and re-anchors on what it hears, and every disagreement about
where a slot began puts it on the wrong channel for a frame. In simulation
a fixless listener loses about a quarter of the traffic that way across
fifty channels and none of it across one. A node with a fix never waits,
and a node that has synced once stays synced through fix loss, a config
push, a standby and a brownout of the radio. The base station on a desk is
the case to know about on a hopping network: give it a fix, or a short
interval on the nodes it is waiting for.

On the air, nodes take turns inside a slot. The window (the slot less a
100 ms guard at each end) is cut into as many lean beacons as fit back to
back - two at the default modulation - and a node's address picks its
turn; with `interval_s` longer than a slot the address picks the slot of
the interval first. So addresses `1` and `2` may both beacon every second
without ever overlapping, and `2 x interval_s` consecutive addresses
never overlap at any interval. Past that, two nodes share a turn and
overlap on every transmission; they cannot hear each other to notice, so
a node that hears both says `hop: node N shares this node's turn`, and the
app's Radio page shows how many addresses a plan carries. A transmission
is planned for a random point inside the turn and made when that instant
arrives. A receiver that has seen a preamble holds its hop until the frame
lands - for the header time until a header follows, since the detector
fires on noise, then for the longest frame the modulation allows - and a
node holds a transmit for the same reason. The console reports `hop: clock
on gps time` and `hop: clock from node N (stratum K)` as the clock changes
hands, the periodic status line carries the channel and stratum, and the
telemetry characteristic reports both to the app's Status page.

`tools/radio_sim.py` simulates a few boards on this plan - the hop clock,
the receiver, the GPS UART, the card and the BLE notifier - and
`docs/RADIO-AUDIT.md` is what it found; `pixi run radio-sim` runs it.

None of this is a certification. At the default the node holds one 500 kHz
carrier, which is the band's digital-modulation route rather than its
hopping one; raised to fifty channels the plan follows the shape of the
hopping rule - each channel used equally on average, receivers in step.
Whether a given board and antenna comply either way is a measurement.

### Beacon payload

`fields` decides what goes on the air. The default is position only: 13
bytes per frame against the 24 a full GPS packet costs, so roughly half the
air time on every broadcast. Altitude, speed, course, satellite count and
time are still recorded in `GPSLOG.CSV` whether or not they are transmitted
- the choice is only about what a *remote* receiver gets.

`lat` and `lon` are required; a config that omits either is rejected. The
selected set travels in the frame as a one-byte mask, so nodes configured
differently interoperate: a receiver decodes whatever each sender chose to
include, and fields nobody sent read back as zero.

Air time is the scarce resource on a shared band, and it grows with the
spreading factor - at SF12 a field costs about 32 times what it does at
SF7. Add fields when a receiver needs them, not by default.

### No-fix ping

A node with no fix has no position to broadcast, and a silent node looks
exactly like one out of range or one that is dead. So the beacon slot goes
out anyway, carrying a 4-byte ping instead: uptime in seconds, whether the
GPS module is talking at all, and whether a fix was ever held. A receiver
then knows the node is up, roughly how long it has been searching, and
whether to look at the sky or at the board - a silent module is usually the
GPS/LoRa rail being off rather than a receiver that cannot see satellites.

It goes out on its own, slower period, `ping_interval_s` - every 5 s
against the position's every second - since a receiver still searching has
nothing new to report between pings, and a ping is smaller than the leanest
position (248 ms against 289 ms on air at the defaults). The moment a fix
lands, the next transmission is a position on the beacon interval rather
than a ping waiting out its own. `ping_interval_s = 0` turns the ping off
by itself; `interval_s = 0` and `role = "rx_only"` silence the node
altogether, pings included.

A node that hears a ping reports it as a status line (`node 3 ping: rssi
-97, up 214s, gps ok`) rather than a position, so it reaches the app
console and the BLE status characteristic without inventing a position
nobody measured. The same ping also goes over the link as data
(`msg::PING`) and out on the node-ping characteristic, so an app can show
the node as alive-without-a-fix instead of having to parse the line.
Nothing is written to `GPSLOG.CSV`, which holds fixes. The RSSI in either
form is what makes a ping useful as a range check: a node left on a bench
with its antenna disconnected from the sky still tells you what the link is
doing.

### Leaves and repeaters

Every transmission is a broadcast and every node listens continuously, so
a fleet of plain leaves already works: each hears whichever others are in
direct range, and the defaults above need no changing. Setting `role =
"repeater"` on one node makes it retransmit what it hears, which is how
you cover ground no pair of leaves can reach across directly. Give a
repeater the elevation and the antenna - that, not the protocol, is where
the range comes from.

`max_hops` belongs to the *sender*: it is the number of retransmissions
that node's own broadcasts are allowed, stamped into each frame as it goes
out. A repeater forwards anything still carrying hops, so dropping one
into an existing fleet works without reconfiguring the nodes already
deployed. Frames are identified by sender and sequence number, so a frame
that arrives twice is handled once and two repeaters cannot bounce one
back and forth.

Each hop is another full transmission of the same frame on a shared
channel. `max_hops = 1` is the setting that pays; past 2 the traffic grows
faster than the coverage.

A frame is 3 header bytes plus the payload at its true length, with no
padding - a 20-byte position costs 24 bytes on air. Spreading factor is the
largest range knob here (SF7 to SF12 is roughly 12 dB) and every byte of
framing is paid for at whichever one is in use, so the framing is kept
small: at the SF12 default a header byte costs 32 times what it would at
SF7.

Nodes transmit on the private LoRa sync word (0x1424), not the public
LoRaWAN one, so a receiver does not lock onto LoRaWAN preambles it can never
decode. It is not configurable, and nodes on different sync words cannot
hear each other at all - firmware from before this change will not link with
firmware after it.

`rx_boost` is the one link-budget key that is not symmetric: it buys
roughly +2 dB of sensitivity on the node it is set on, and does nothing
for what that node transmits. Range is set by the worse of the two
directions, so it only helps where the receiving end is the weak one -
setting it on both nodes is the usual answer, at a few mA each while
listening. Every other radio key has to match across nodes to link at
all; this one does not.

The `[gps]` settings are pushed to the module as a single UBX-CFG-VALSET at
boot and again whenever a new config is applied (constellation, power and
model changes take effect live). Defaults match the M10 factory set, so an
absent section is a no-op.

The same frame turns off the NMEA sentences the firmware does not read (GLL,
GSA, GSV, VTG), leaving RMC and GGA. The link to the module is 9600 baud, or
960 bytes a second, and GSV alone can exceed that in one epoch once several
constellations are enabled - which delays the two sentences that carry the
fix behind sentences nothing parses. The module acknowledges the frame, and a
push that lands before it has finished starting is retried on its first
sentence.

## BLE

Same service UUID as the ESP32-C3 beacon, so gps-gui-rs discovers it
unchanged (the app filters scans by service UUID, so a board's name is
display text and renaming one cannot lose it - see [Board names](#board-names)).
On top of the gps-proto position / config / ack characteristics the firmware
adds telemetry (LoRa RSSI/SNR, counters, SD + fix flags), remote node
positions and pings, a status/log characteristic (notify + read), the board's
name (read + notify), the current radio config (read + notify), and a bulk
write characteristic carrying either a TOML config or a firmware image.

Everything declared `read` is written into the attribute table as well as
notified, so a central that never subscribes reads the current value rather
than the zeros the table was built with. That matters most for telemetry,
where the all-zero blob is not an obviously empty one: `secs_since_rx` of 0
means "heard from just now", and the value for never is 0xFFFF.

### Remote nodes

What other nodes report arrives on two characteristics: positions on
`c3a10007-...` (`[src, rssi i16le, 20-byte packet, age_s u16le]`) and pings
from nodes without a fix on `c3a1000b-...` (`[src, rssi i16le, flags,
uptime_s u16le, age_s u16le]`).

Both are notified when a report arrives rather than on the position notify
tick, and the firmware keeps the newest report from each of up to 8 nodes rather
than one report in total. That combination is what makes the stream
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
time on and `time` is not one of the defaults - there is nothing in a lean
beacon to age it by.

The age field is appended to the position layout, not inserted, so an app
built before it reads the same fields at the same offsets and ignores the
tail.

## Status updates

The firmware writes human-readable status lines to the USB console on
notable events - boot, GPS presence (first NMEA / silent module), GPS fix
acquired/lost, radio standby/wake, config applied, a radio that restarted
underneath the firmware, and a no-fix ping heard from another node - and notifies the same text on the status/log
characteristic, so gps-gui-rs (or any BLE client) sees the live log. Lines
are ASCII, up to `link::LOG_MAX` (128) bytes.

A periodic status line every 10 s carries the radio's chip mode and latched
device errors, GPS sentence and byte counts, fix state, and whether the
card mounted. It exists because a quiet radio and a quiet GPS look
identical otherwise, and because the radio's status byte reports the mode
it is in, not whether it got there intact - a TCXO that never started or a
calibration that failed still reads as a healthy standby.

The settings characteristic (`c3a10009-...`, read + notify) carries the
device's current power/sleep configuration as one 16-byte blob
(`midair_proto::ble::Settings`), so an app can populate its controls on
connect rather than assuming defaults. It is republished after every
config write - including values the device changed itself, such as a
clamped interval.

The radio-config characteristic (`c3a1000a-...`, read + notify) does the
same for the radio configuration - the `RADIO.CFG` settings as a 28-byte
`midair_proto::radiocfg::RadioConfig` snapshot. Otherwise the config only
ever travels *to* the board, so this is the one way to see what a board is
running. It is refreshed on connect and after every apply, and reads back
all-zero (which decodes to nothing) until the radio has been initialized.

Config command ids (config characteristic, `[id, len, value]`):

| Id | Value | Effect |
|-|-|-|
| `0x01` | u32 ms | position notify interval (gps-proto) |
| `0x10` | u8 0/1 | GPS + LoRa power rail - no hardware on this board, logged and ignored |
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

### Board names

A board advertises as `ws3gps-<label>`, and one that has never been named
falls back to `ws3gps-<xxxx>` from the last two octets of its BLE address -
so two boards out of the same box are already told apart in a scan list,
and a fleet reads as `ws3gps-ground-1`, `ws3gps-sky-1` without anything
having to be configured first.

```
pixi run wio-set name sky-1     # over USB, at the bench
pixi run wio-set name ""        # back to the address-derived name
pixi run wio-info               # what this board is called, and its address
```

The prefix is a firmware constant rather than part of the label, so a board
cannot be named something unrecognizable: whatever it is called, a generic
scanner can be searched by `ws3gps`. Labels take ASCII letters, digits, `-`
and `_`; anything else is rejected rather than sanitized, because a board
answering to a name nobody asked for is worse than a write that failed
loudly.

The label is stored with the settings that decide reachability - RTC RAM,
mirrored to the `nvs` partition - rather than on the card, because a wake
check advertises before anything has mounted one. It survives a deep sleep
and a flat cell, and a board updated from firmware that predates names
reads back as unnamed rather than as unreadable.

Three surfaces carry it, and they catch up at different speeds:

| Surface | When it updates |
|-|-|
| scan response (`CompleteLocalName`) | the next advertising window - the one on the air was handed to the controller before the write |
| name characteristic (`c3a1000c-...`, read + notify) | immediately, on the connection that renamed the board |
| GAP device name (`0x2A00`) | the next boot; the attribute table is built once per power cycle |

The ack for `0x19` carries the stored *length*, not the label: an ack has
four value bytes and no name fits in one. What a board is actually called
comes back on `c3a1000c-...`.

### Status display

An optional 0.91" 128x32 SSD1306 on the J5 JST-SH shows the four numbers
that answer "is this board working" without a phone or a serial cable:

```
GPS FIX   12 sat
RSSI  -87 dBm
SEEN     4s
n1   rx23   tx15
```

`GPS ----` means no fix. `RSSI --` and `SEEN never` mean nothing has been
heard over LoRa since boot, which is what separates a quiet channel from a
radio that is not working. The bottom line is this node's address and its
packet counters, so two boards on a bench are told apart without connecting
to either.

It is **detected, not configured**: no I2C ack on J5 and the firmware says
`oled: none on J5` once and never mentions it again. Both 0x3C and 0x3D are
tried, and so are both SDA/SCL orders - the schematic names those two nets
`GPIO10` and `GPIO11` and nothing else, so a reversed cable is a working
display rather than a dead one.

#### The compass

Add a QMC5883L or HMC5883L magnetometer to the same J5 bus (0x0D and 0x1E,
both probed) and the panel switches to a compass whenever there is somewhere
to point: a rose on the left, the numbers on the right.

```
    |     n9 NNE 032
  \ | /   1.24km
   \|/    -87dB 4s
    o     FIX 12sat M
```

Up is the way you are facing; the arrow is the other node. The three-letter
point and the degrees are both relative, and the last character says where
the heading came from - which matters, because the arrow means something
different in each case:

| | |
|-|-|
| `M` | Magnetometer. Works standing still. |
| `G` | GPS course over ground, used when there is no usable magnetic heading and you are moving faster than 0.5 m/s. Relative to the way you are *travelling*. |
| `T` | No heading at all. The arrow is the **true** bearing - north is up, and you supply the rotation. |

The compass screen appears only when this node has a fix and some other node
has reported one, so it is shown exactly when it can be correct; the status
screen above is what you see the rest of the time. It points at the node
heard from most recently, not the nearest - "nearest" makes the arrow jump
between two nodes trading places at similar range, where "newest" only
changes when a different node is actually heard.

**It has to be turned before it works.** The heading is hard-iron corrected
from the extremes seen on each axis, which only mean anything once the board
has been rotated through a full circle - a magnetometer next to a LoRa PA, an
SD card and a battery does not read a field centered on zero. Until it has,
the marker reads `G` or `T` rather than showing a confident heading built
from a quarter turn. It is also **not tilt-compensated**: hold the board
level. Correcting that needs an accelerometer, which is a different part than
the two supported here.

The panel reads the same telemetry the BLE session notifies, so it and the
app cannot disagree. It refreshes twice a second and skips frames identical
to what is already on screen.

**It costs 5-15 mA** depending on how many pixels are lit, which is why the
layout leaves most of the panel dark and why the firmware blanks it (charge
pump off, not just pixels cleared) before every deep sleep - it sits on the
always-on +3V3 and would otherwise hold its last frame, and its current,
for the whole sleep.

### Modes

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
competing: before the modes existed, `0x16` was dead config on any board
that had a wake-check cadence, because deep sleep was tested first and
always won. The tracker's on period (`0x1A`) and the wake check's window
(`0x14`) were one number until they were separated: a wake check wants the
shortest window a phone can still catch, a tracker one long enough to
connect, read the roster and let go.

### Low power

Sleep is off by default (`0x13` = 0), which is what an unconfigured board
does: land in idle at boot and stay there, advertising continuously. So is
the idle timeout (`0x18` = 0), and storing out of idle needs both: with no
timeout nothing fires, and with no cadence to sleep on the timeout has
nowhere to send the board. `0x13 = 0` is therefore also "never store this
board *on its own*", and it is the bench setting. Being told `0x17 = 0` still stores it: somebody asked for
that one, so it borrows the 5 min ceiling rather than reading the missing
cadence as a refusal.
Two board facts shape everything below - there is no rail to cut, and what
the two-MCU board called the WIO's boot time is now nothing at all.

`0x13` is the stored cadence. While the board is stored it deep-sleeps for
that long, wakes into a check, advertises for `0x14` seconds (one long D2
blink), and goes back down if nobody came. Both persist until changed - a
connect does not clear them, so an unattended board holds its cadence
indefinitely and the settings mean the same thing whether or not anyone is
looking. A board that is *tracking* ignores `0x13` entirely: deep sleep
stops the beacon, the logging and the listening, and a tracker doing that is
not tracking.

Together the two set the duty cycle, and so the average current:
advertising costs roughly two orders of magnitude more than deep sleep, so
the draw tracks window/interval almost exactly. The window is the more
useful knob of the two, because shortening it does not make the board any
slower to reach - a 5 s window at a 60 s interval is still four times the
battery life of the 15 s default, and still gets you a wake every minute.
What it costs is margin: the window has to overlap a phone's scan, and a
phone that only scans intermittently can miss several short windows in a
row. Unlike the interval, `0x14` has no "off" - a 0 clamps up to 1 s
rather than being stored as a window nobody could connect in. One second is
a deliberate duty-cycle choice or a bench setting, not a comfortable connect
time.

A window changed over BLE applies from the next wake, not the current one.

#### Sleeping on command

`0x13` and `0x14` describe when the board sleeps *on its own*, and both only
fire when an advertising window expires with nobody connected. That leaves
no way to sleep a board you are looking at: you would have to disconnect and
wait the window out, on a board that has been given a cadence in the first
place.

`0x15` is the direct one, and `0x17 = 0` (stored) is the same path with the
mode written first, so the board comes back into a wake check rather than
into whatever it was doing. The board acks, finishes paying out what it owes
the connection, and goes. `0x15` stores nothing, does not touch `0x13`, and
comes back to exactly what it was configured for - so a board with sleep
mode off takes one nap and resumes advertising continuously. A value of 0
means "for the `0x13` interval", falling back to 60 s when sleep is off, and
the ack carries the resolved seconds so an app reports what the board will
actually do rather than what it was asked for.

**The disconnect is the command working.** The board stops being contactable
the moment it sleeps; the app's Beacon page says so before the link drops
and again after it does.

The same command is on the USB console (`pixi run wio-sleep`, optionally
`--seconds N`), which is the bench answer - the serial port disappearing and
coming back is a whole sleep cycle observed without a phone.

A wake from deep sleep prints `woke from deep sleep #N (slept M s)`, counted
in RTC RAM since the last cold boot. That line is the difference between a
board on its cadence and a board resetting in a loop: deep sleep is a full
reset, so without the counter the two produce an identical boot banner.

The rail policy is **inert on this board.** The GPS `VCC`/`V_IO` and the
SD both sit directly on +3V3, and the only load switch (U3, SiP32431)
feeds the GPS active antenna and is driven by the GPS's own `LNA_EN`, not
by a host GPIO. So `0x10` is accepted and logged with nothing behind it,
and deep sleep leaves a MAX-M10 acquiring beside a sleeping S3 - which is
the dominant draw at 25-31 mA. GPS backup mode (`0x12`) is the only firmware
lever on it, and on this board it is a poor one: `V_BCKP` goes to a
test point and nothing else, so the M10's backup domain - the RTC, the BBR
holding the ephemeris, and the UART-RX wake source itself - has no supply.
Backup mode there means a cold start on every wake rather than a warm one,
which on a short cadence is a board that never gets a fix. That is why the
sleep path does not reach for it on its own: it stays an explicit choice.

**The board is wired for an active antenna, and the firmware does not
configure the antenna at all.** `U5.VCC_RF` -> U3 (SiP32431) -> R15 10R ->
L1 27nH -> the SMA J2 center pin is a populated, unconditional bias tee, and
U3's enable is the GPS's own `LNA_EN` rather than a host GPIO - so DC is on
the antenna port whenever the receiver's RF section is on. `Gps::configure`
writes no `CFG-HW-ANT_*` key, so the antenna supervisor sits at its factory
default and the receiver has never been told which kind of antenna is
fitted.

**The feed cannot be turned off in firmware.** MAX-M10N integration manual
Table 22: `LNA_EN` is high in normal operation and the antenna supervisor
does not gate it - the supervisor can only pull it low on a detected short,
which needs a sense pin this board does not have, and its voltage control is
disabled by default anyway. The pin's polarity is fixed and it also drives
the module's internal LNA, so it is not the firmware's to repurpose. No
config key exists for this because a key that does nothing is worse than
none.

With an active antenna the LNA adds 5-20 mA and cannot be dropped without
parking the receiver. With a passive one it depends on the antenna's DC
path: a wire or any capacitively-coupled feed is DC-open and the bias drives
nothing, while a DC-shorted feed - common on passive patches - puts 3.3 V
across R15's 10 ohm, which is a short rather than a load. One probe across
R15 tells them apart: ~0 V is open.

**For a passive build the fix is hardware:** depopulate R15, the 10 ohm in
the bias tee's DC path. One 0402, and the feed is gone.

Tying `V_BCKP` to +3V3 is a board change worth more than the TTFF it is
usually filed under (see `BOARD-REVIEW.md` in the board repo): it is the
difference between a receiver whose backup domain is guaranteed and one
whose is merely observed to survive on `VCC`. Backup mode is used either
way - the timed-PMREQ experiment proves the domain lives - but what it costs
there has never been measured, and that number is what a stored board's
floor comes down to.

Before it sleeps the board parks everything it can reach. The radio goes to
cold sleep (5.5 mA of continuous RX against 9.3 uA), the card is flushed and
unmounted, the panel is blanked, and the receiver is sent into PMREQ backup
- re-issued on every park, because after a reset the firmware's belief about
the module is worth nothing, and because a bare request to a module already
in backup gets eaten as the wake-up itself and leaves it awake.

Two pads are held across the sleep. NSS (GPIO21), because the SX1262 leaves
cold sleep on a falling edge and the S3 releases every pad it is not holding.
UART TX (GPIO2), because a floating edge there is UART traffic to a receiver
that wakes on it, which would undo the backup the park just asked for. SD CS
(GPIO44) has the same problem and no fix in firmware - the S3's RTC pins stop
at 21.

The mode, the intervals, the window and the `0x10` rail setting are held in
RTC fast RAM and mirrored to the `nvs` flash partition, so they survive deep
sleep *and* a flat battery - a board put away for transport comes back on the
same cadence rather than advertising until the cell dies again. Flash is read
only on a cold boot; wake checks run from the RTC RAM copy. The two sleep
flags are still not mirrored, and the mode is what resolved the awkwardness
there: "a board that cold-boots with its GPS running is the safer failure" is
true of a tracker and drains the cell of a device in a bag, so a cold boot
lands in idle instead - reachable, and with the GPS down.

The wake is timed by the uncalibrated RC slow clock, so the interval
drifts - it paces a wake-check, not a schedule.

**Deep sleep has no wake source but the timer.** Nothing over the air can
interrupt it: the radio is off, and there is no GPIO or button wake
configured. The 5 min ceiling on `0x13` is what bounds that - it is the
longest the board can ever be unreachable, short of a physical reset. A
reset does get you back sooner, and now it buys more than a window of your
own choosing: a cold boot lands in idle, so the board comes up reachable for
a whole `0x18` timeout rather than for one advertising window.

## SD card

`GPSLOG.CSV` gets one line per own/remote fix
(`ms,src,lat_e7,lon_e7,alt_dm,speed_cms,course_cdeg,sats,fix,rssi`);
readable in any spreadsheet. The card is optional and hot-pluggable - when
none is present the driver retries the mount once a minute, so a card
inserted later starts logging within that. Only about the last 20 seconds of
positions are buffered in RAM while no card is mounted; anything older is
dropped. `sd_enabled = false` shuts the card down entirely.

Formatting: **MBR partition table, first partition FAT16 or FAT32**. That is
what a card of 32 GB or less already ships as, so most cards work untouched.
Larger (SDXC) cards ship exFAT, which is not supported and must be
reformatted - use the SD Association's SD Card Formatter, or on Linux make
an MBR partition of type `0c` and `mkfs.vfat -F 32 /dev/sdX1`. Formatting
the whole device (`/dev/sdX`, no partition) produces a card the driver
cannot mount. GPT is not supported either.

Both filenames are MS-DOS 8.3 - eight characters plus a three-character
extension - which is why the config file is `RADIO.CFG` and not
`RADIO.TOML`. The FAT layer converts a name to 8.3 before looking it up, so
a longer name is not a missing file but one that can never be opened.

## Firmware update

```sh
cd tools && pixi run wio-ota          # builds ../firmware and pushes the image
pixi run wio-ota --image firmware.bin # or send one you already have
```

The image goes into whichever of the two application slots the board is not
running from, streamed a flash sector at a time. Nothing is pointed at it
until the whole transfer has arrived and its CRC matched; then `otadata` is
moved, the board reboots, and the new firmware marks itself confirmed once
it has booted far enough to run `main`. A bootloader built with rollback
enabled reverts to the previous slot if that never happens, so an image that
cannot start costs a reboot rather than a board.

An image must be an ESP-IDF *application image*, not the ELF - `wio-ota`
checks the 0xE9 magic and refuses the ELF rather than letting the board
write something it cannot boot. With no `--image` it runs the conversion
(`espflash save-image`) itself.

The same transfer works over BLE, on the bulk characteristic with
`kind = 3`. Each sector write holds interrupts off for tens of milliseconds,
which a BLE connection rides out but does notice; USB is the smoother path
and the one the tool takes.

`cargo run --release` over USB still works and is what puts a board onto the
two-slot partition table in the first place. It erases `otadata`, so it
always wins over whatever an OTA left selected.

What this replaces: the two-MCU board streamed a raw STM32 image over the
UART link into the WIO-E5's DFU partition, where a swap bootloader
installed it power-fail-safely and reverted if the new image never
confirmed boot. Bulk kind 2 carried it, over BLE or the ESP's USB port.
That kind is retired rather than reused - kind 3 is the ESP image - so an
old tool pushing an STM32 image at this firmware is rejected instead of
misread into an app slot.

## Wio-S3 module

`ESP32-S3R8 + SX1262 + 32 MHz TCXO`
`16 MB Flash, 8 MB PSRAM`

**This board requires the -N SKU (100079384, bare RF pads).** The u.FL
versus RF-pad choice is two parts, not a switch - nothing in firmware or on
the board selects it - and the pad names carry the difference:
`LORA_ANT / NC` and `WIFI/BT_ANT / NC` are the RF ports on the bare-pad
part and *not connected* on the IPEX part (100020327), where the u.FL sits
on the module itself. The carrier runs pad 37 straight to the SMA J6, so
an IPEX module leaves that SMA connected to nothing and the PA transmits
into an open. Check the can before powering a new build: two small gold
u.FL connectors on the top face is the IPEX part.

Board wiring (carrier design, `wio-s3-max-gps`):

| Pin | Function |
|-|-|
| GPIO1 | GPS UART RX (from GPS TXD) |
| GPIO2 | GPS UART TX (to GPS RXD) |
| GPIO3 | SD MISO - strapping pin |
| GPIO14 | LED D2, active low |
| GPIO19 / GPIO20 | USB D- / D+ |
| GPIO43 | LED D5, active low; also UART0_TX |
| GPIO44 | SD CS |
| GPIO45 | SD MOSI - strapping pin, R17 DNP as of board V2 |
| GPIO46 | SD SCK - strapping pin |
| GPIO10, GPIO11 | J5 JST SH 4-pin, I2C - status OLED and compass |
| GPIO38-41, GPIO47 | J1 header 1x07 |
| GPIO0 / RST | BOOT / RST test points |

Module-internal wiring (datasheet Table 2), which never reaches a pad:

| SX1262 pin | Connected to |
|-|-|
| NSS | GPIO21 |
| SCK | GPIO4 |
| MOSI | GPIO6 |
| MISO | GPIO5 |
| NRESET | GPIO7 |
| BUSY | GPIO8 |
| DIO1 | GPIO9 |
| DIO2 | SKY13453-385LF VCTL (RF switch) |
| DIO3 | SKY13453-385LF VDD (and the TCXO) |

`GPIO26-32` are the flash interface and `GPIO33-37` are the octal PSRAM the
R8 part uses; neither is available whatever a generic ESP32-S3 pin table
suggests.

**The two RF-path registers are not tunable.** DIO2 must drive the switch
and DIO3 must supply it at 2.5 V or more, or the PA transmits into an
isolated port and the module dies. `RadioConfig` defaults both to this
board's hardware and the driver enforces them; see `ARCHITECTURE.md`.

**Three of the four SD lines sit on strapping pins.** GPIO45 selects
VDD_SPI (low 3.3 V, high 1.8 V, sampled at reset), so a pull-up there stops
the part booting on a module without `VDD_SPI_FORCE` burned - hence R17
DNP. GPIO46 pulled high disables the ROM boot log and GPIO3 pulled high
moves the JTAG source; both are survivable.

## Connectors
#### JST SH
*As of Version 1*

**I2C** *(J6)*

| Pin | Function |
|-|-|
| 4 | SCL |
| 3 | SDA |
| 2 | 3V3 |
| 1 | GND |

**SWD** *(J5)*

| Pin | Function |
|-|-|
| 4 | SWDIO |
| 3 | SWDCLK |
| 2 | 3V3 |
| 1 | GND |


## Charging IC 

`MCP73831T-2ACI/OT`
4.2 V
Adjustable current. 500 mA @ 2k ohm programming resistor.


## Power

`POWER.md` is the reference: every setting that changes what the board
draws, what it costs, and where it is set. `docs/POWER-S3.md` is the
investigation behind those numbers and `docs/POWER-AUDIT.md` is a critical
read of it.

The short version. Awake, BLE advertising, GPS tracking, LoRa listening and
nothing transmitting, the board measures **~126 mA at the 4.2 V input**. The
BLE controller was 71 mA of that and the MAX-M10 is most of what is left, so
`ble_off_s` and the GPS `power_mode` are the two settings that matter. The
regulator is an LDO, so that current passes straight through from the cell.

That 71 mA was measured with the BLE PHY powered continuously, which is
what esp-radio does: it ships the controller's modem sleep unimplemented.
The vendored copy implements it now, so the controller powers its own PHY
down between advertisements and between the connection events of an idle
connection. The board has not been back on a meter since, so treat every
BLE figure here as an upper bound.

The two-MCU board this one replaced measured 66 mA in the same scenario.
Roughly 50 mA of the gap is the part swap - an S3's BLE radio costs about
twice a C6's for the same job - and roughly 20 mA is a hardware feature the
old board had and this one does not: a GPIO under the GPS and LoRa rail.

For reference, the Wio-S3 datasheet quotes 9.3 uA deep sleep, 1.43 mA
standby, 5.5 mA LoRa RX and 125 mA LoRa TX at 22 dBm. The 5.5 mA RX figure
is only reachable with the SX1262's DC-DC, which is how we know the module
carries the SMPS inductor and why `dcdc_enabled` defaults on.

## GPS board v1

![GPS Board v1](images/GPSv1.svg)
