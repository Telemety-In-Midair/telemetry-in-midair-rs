# telemetry-in-midair-rs
[Kicad Board](https://github.com/tmpk13/telemetry-in-midair) https://github.com/tmpk13/telemetry-in-midair

GPS tracker board firmware: one Seeed Wio-S3 module (ESP32-S3R8 + SX1262)
reads a MAX-M10 GPS, transmits positions over 915 MHz LoRa, logs to SD, and
serves everything over BLE to the gps-gui-rs app. See `PLAN.md` for the
intent and `ARCHITECTURE.md` for the UML views.

This replaced a two-MCU board (ESP32-C6 for BLE and power, WIO-E5 for
GPS/LoRa/SD, a framed UART link between them). That firmware is gone from
the tree as of the single-module cleanup; `git log` still has it, and
`PORT-WIO-S3.md` records what the merge deleted and why.

## Layout

| Directory | What | Target |
|-|-|-|
| `proto/` | Shared no_std protocol crate: LoRa payloads, BLE extensions, `RADIO.CFG` parser, USB bulk framing. Host-testable (`cargo test`). | any |
| `s3/` | Wio-S3 firmware (embassy + trouble BLE): radio, GPS, SD and the GATT service. | `xtensa-esp32s3-none-elf` (`esp` channel) |
| `tools/` | Host tools (Python/pixi) for pushing a radio config over USB. | host |

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
cd s3 && cargo run --release
```

The console is on the USB Serial/JTAG port (GPIO19/20 to the USB-C
connector), not UART0 - GPIO43 is UART0_TX on this board and drives the D5
LED, so expect the ROM bootloader's own log to flicker it on every reset.

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

A pushed config is stored twice - `RADIO.CFG` on the card and a backup in
internal flash - so it survives a power cycle on a board with no SD card.
The card wins at boot, so editing `RADIO.CFG` on a computer still works.
The board reports which stores it reached, and `wio-config` exits non-zero
if a config went live but reached neither.

The GPS and SD sit directly on +3V3 on this board, so there is no rail to
raise before they answer.

## Radio configuration

The firmware loads `RADIO.CFG` from the SD card at boot; the same file can be
pushed over BLE (bulk characteristic) at runtime, which also rewrites the
SD copy. All keys are optional; defaults in parentheses:

```toml
[radio]
frequency_hz = 915000000   # (915 MHz)
spreading_factor = 12      # 5-12 (12)
bandwidth_khz = 500        # 62|125|250|500 (500)
coding_rate = 5            # 4/5..4/8 (5)
power_dbm = 22             # -9..22 (22)
rx_boost = true            # boosted RX gain (true)
dcdc_enabled = true        # internal DC-DC instead of LDO (true)
tcxo_volts = "1.8"         # TCXO supply; board hardware, not a tuning knob
tcxo_startup_ms = 10       # TCXO settling wait, 1-1000 (10)

[network]
address = 1                # 1-255 (1)
role = "leaf"              # leaf | repeater | tx_only | rx_only (leaf)
max_hops = 1               # retransmissions allowed, 0-8 (1)
dedup_ttl_s = 3            # how long a (sender, id) pair is remembered (3)

[beacon]
interval_s = 20            # broadcast period, 0 = off (20); also paces the
                           #   no-fix ping
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

### Modulation

The default is SF12 at 500 kHz, which is a deliberately wide signal rather
than the narrow one a range-first reading would pick.

In the 902-928 MHz band a 500 kHz signal counts as a digital modulation and
is allowed to sit on a single channel indefinitely, with no dwell or duty
cycle ceiling. Anything narrower has to qualify as frequency hopping
instead: at least 50 channels, no more than 0.4 s on any one of them per
20 s, and receivers hopping in step with the sender. That last part is what
rules it out here. Hopping needs a clock the whole network agrees on, the
only one available is GPS time, and a node that has never had a fix does not
have it - which is exactly the node the no-fix ping exists to keep audible.

SF12 buys most of the width back. The narrow alternative it replaced,
SF9 at 62.5 kHz, is 1.5 dB more sensitive (-132.5 against -131 dBm), worth
roughly a tenth of the range on real terrain. It is also *longer* on air:
2^12/500 kHz and 2^9/62.5 kHz are the same 8.192 ms symbol, and SF12 needs
fewer symbols per byte, so the default beacon is 289 ms where the narrow one
was 330 ms.

Both are still config keys. If you are somewhere the band rules differ - EU
868, say, where there is no minimum bandwidth and the constraint is a duty
cycle instead - `spreading_factor = 9` and `bandwidth_khz = 62` on the card
gets the narrow modulation back, along with `power_dbm = 14`.

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

It is the same one transmission per `interval_s`, not an extra one, and a
ping is smaller than the leanest position (248 ms against 289 ms on air at
the defaults), so a node that never gets a fix costs the channel less than
one that does. `interval_s = 0` and `role = "rx_only"` turn it off along
with the beacon; there is no separate switch.

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
unchanged (device name `GPS-S3`; the app filters scans by service UUID, so
the rename from `GPS-C6` is display text only). On top of the gps-proto
position / config / ack characteristics the firmware adds telemetry (LoRa
RSSI/SNR, counters, SD + fix flags), remote node positions and pings, a
status/log characteristic (notify + read), the current radio config (read +
notify), and a bulk write characteristic for TOML config.

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
acquired/lost, radio standby/wake, config applied, and a no-fix ping heard
from another node - and notifies the same text on the status/log
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
| `0x13` | u32 s | deep-sleep wake-check interval, 5 s..5 min, 0 = off (not ported yet) |
| `0x14` | u32 s | advertising window per wake check, 3 s..60 s (default 15 s) |

### Low power

**Not ported yet.** Deep sleep and its nvs-backed settings are outstanding
work (`PORT-WIO-S3.md`, step 5); `Stored` sits in a plain static that
resets with the board, and the firmware advertises indefinitely. What
follows is the policy `session::apply` still enforces and clamps - it is
host-tested and unchanged - described as it will behave once the sleep path
exists. Two board facts already change it: there is no rail to cut, and
what the two-MCU board called the WIO's boot time is now nothing at all.

`0x13` turns sleep on. While set, the board deep-sleeps whenever no central
is connected and wakes every interval to advertise for `0x14` seconds (one
long D2 blink). Both persist until changed - a connect does not clear
them, so an unattended board holds its cadence indefinitely and the
settings mean the same thing whether or not anyone is looking.

Together the two set the duty cycle, and so the average current:
advertising costs roughly two orders of magnitude more than deep sleep, so
the draw tracks window/interval almost exactly. The window is the more
useful knob of the two, because shortening it does not make the board any
slower to reach - a 5 s window at a 60 s interval is still four times the
battery life of the 15 s default, and still gets you a wake every minute.
What it costs is margin: the window has to overlap a phone's scan, and a
phone that only scans intermittently can miss several short windows in a
row. Unlike the interval, `0x14` has no "off" - a 0 clamps up to 3 s
rather than being stored as a window nobody could connect in.

A window changed over BLE applies from the next wake, not the current one.

The rail policy is **inert on this board.** The GPS `VCC`/`V_IO` and the
SD both sit directly on +3V3, and the only load switch (U3, SiP32431)
feeds the GPS active antenna and is driven by the GPS's own `LNA_EN`, not
by a host GPIO. So `0x10` is accepted and logged with nothing behind it,
and deep sleep leaves a MAX-M10 acquiring beside a sleeping S3 - which is
the dominant draw. GPS backup mode (`0x12`) is the only real power lever,
and the app should still expect a GPS cold TTFF after a long sleep.

The interval, the window and the `0x10` rail setting will be held in RTC
RAM and mirrored to the `nvs` flash partition, so they survive deep sleep
*and* a flat battery - a board put away for transport comes back on the same
cadence rather than advertising until the cell dies again. Flash is read
only on a cold boot; wake checks run from the RTC RAM copy.

The wake is timed by the uncalibrated RC slow clock, so the interval
drifts - it paces a wake-check, not a schedule.

**Deep sleep has no wake source but the timer.** Nothing over the air can
interrupt it: the radio is off, and there is no GPIO or button wake
configured. The 5 min ceiling on `0x13` is what bounds that - it is the
longest the board can ever be unreachable, short of a physical reset. A
reset does get you back sooner, but a cold boot restores the settings from
flash and the first advertising window is the same `0x14` seconds as any
other, so it buys you a window you chose the timing of rather than an
awake board.

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

Not ported yet. The ESP-IDF bootloader does two-slot OTA with rollback and
`esp-bootloader-esp-idf` is already a dependency, but nothing drives it -
`cargo run --release` over USB is the only path today.

What this replaces: the two-MCU board streamed a raw STM32 image over the
UART link into the WIO-E5's DFU partition, where a swap bootloader
installed it power-fail-safely and reverted if the new image never
confirmed boot. Bulk kind 2 carried it, over BLE or the ESP's USB port.
That kind is retired rather than reused, so an old tool pushing an STM32
image at this firmware is rejected instead of misread.

## Wio-S3 module

`Wio-S3` (SKU 100020327 with IPEX, 100079384 with bare RF pads)
`ESP32-S3R8 + SX1262 + 32 MHz TCXO`
`16 MB Flash, 8 MB PSRAM`

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
| GPIO10, GPIO11 | J5 JST SH 4-pin, I2C-shaped |
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


## Inital power testing 

**Measured on the two-MCU board.** Kept as a baseline to beat, not as a
description of this one - the single module should come in well under
these, and nothing has been measured on it yet.

Everything running (BLE connected, Satalite fix, SD logging)
`75 mA`
Everything running no SD (BLE connected, Satalite fix, pulse on LoRa TX)
`66 mA`

Fully running (BLE connected, Satalite fix, pulse on LoRa TX, SD logging)
`120 mA`

ESP only BLE connected
`46 mA`

For reference, the Wio-S3 datasheet quotes 9.3 uA deep sleep, 1.43 mA
standby, 5.5 mA LoRa RX and 125 mA LoRa TX at 22 dBm. The 5.5 mA RX figure
is only reachable with the SX1262's DC-DC, which is how we know the module
carries the SMPS inductor and why `dcdc_enabled` defaults on.

GPS Board v1
![GPS Board v1 diagram](./images/GPSv1.svg)