# The hardware

The module, the carrier's pin map, the connectors, the optional panel, the
card, and the GPS antenna. `docs/BOARD-V1-ISSUES.md` is what the V1 carrier
gets wrong; the KiCad design is the sibling `telemetry-in-midair` repo.

## Wio-S3 module

`ESP32-S3R8 + SX1262 + 32 MHz TCXO`, `16 MB Flash, 8 MB PSRAM`.

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
| GPIO2 | GPS UART TX (to GPS RXD); pad-held through deep sleep |
| GPIO3 | SD MISO - strapping pin |
| GPIO14 | LED D2, active low; blinks on a transmit |
| GPIO19 / GPIO20 | USB D- / D+ |
| GPIO43 | LED D5, active low; also UART0_TX, so the ROM bootloader's log flickers it; blinks on a receive |
| GPIO44 | SD CS; not an RTC pin, so it floats through a deep sleep |
| GPIO45 | SD MOSI - strapping pin, R17 DNP as of board V2 |
| GPIO46 | SD SCK - strapping pin |
| GPIO10, GPIO11 | J5 JST SH 4-pin, I2C - status OLED and compass, either order |
| GPIO38-41, GPIO47 | J1 header 1x07, parked with pull-downs |
| GPIO0 / RST | BOOT / RST test points |

Module-internal wiring (datasheet Table 2), which never reaches a pad:

| SX1262 pin | Connected to |
|-|-|
| NSS | GPIO21; pad-held through deep sleep |
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
board's hardware and the driver enforces them - a config that says
otherwise is logged and overridden; see `ARCHITECTURE.md`.

**Three of the four SD lines sit on strapping pins.** GPIO45 selects
VDD_SPI (low 3.3 V, high 1.8 V, sampled at reset), so a pull-up there stops
the part booting on a module without `VDD_SPI_FORCE` burned - hence R17
DNP. GPIO46 pulled high disables the ROM boot log and GPIO3 pulled high
moves the JTAG source; both are survivable.

Every pin the board brings out but the firmware does not use is parked with
a pull-down, and both SPI MISO pads are pulled up through a frozen input
signal: a CMOS input left floating sits wherever leakage puts it, which can
be mid-rail with both halves of the buffer partly on, and that is invisible
at 75 mA and most of the budget in deep sleep.

## Connectors

JST SH, as of version 1.

**I2C** *(J5)*

| Pin | Function |
|-|-|
| 4 | SCL |
| 3 | SDA |
| 2 | 3V3 |
| 1 | GND |

**SWD** *(J6)*

| Pin | Function |
|-|-|
| 4 | SWDIO |
| 3 | SWDCLK |
| 2 | 3V3 |
| 1 | GND |

## Power path

The 4.2 V node is the output of the diode-OR (D3 battery / D4 USB, both
`DM3CS-SF` Schottky) and the input to U2, a `TLV75733PDBVR` linear
regulator. An LDO passes its load current straight through, so a current
measured at that node is the +3V3 load itself, and (4.2 - 3.3) V times it
is burned as heat in U2 - about a fifth of everything drawn from the cell.
The diode costs usable cell range too: the LDO needs about 3.35 V in, the
Schottky drops another 0.3-0.4 V, so the rail starts sagging with the cell
still around 3.7 V.

The GPS and the SD sit directly on +3V3. There is no rail to cut, which is
most of why this board's floor is where it is (`docs/POWER.md`).

Charging: `MCP73831T-2ACI/OT`, 4.2 V, current set by the programming
resistor (500 mA at 2 kOhm).

## The GPS antenna

The board is wired for an active antenna - `U5.VCC_RF` -> U3 (SiP32431)
-> R15 10R -> L1 27nH -> the SMA J2 center pin is a bias tee - and U3's
enable is the GPS's own `LNA_EN` rather than a host GPIO. The MAX-M10N
integration manual (Table 22) has `LNA_EN` high in normal operation, and
the antenna supervisor can only pull it low on a detected short, which needs
a sense pin this board does not have; so the feed cannot be turned off in
firmware, and no config key pretends otherwise.

**On the boards built so far U3 is unpopulated and the antenna is a wire**,
so the DC path is open one component upstream of anything `LNA_EN` could
reach and the antenna line of the power budget is zero. Fitting U3 is the
decision that brings any of this back, and it should only be made alongside
an actual active antenna. For a passive build with U3 fitted, the fix is to
depopulate R15.

`V_BCKP` goes to a test point and nothing else, so the M10's backup domain
runs on `VCC`. A timed PMREQ backup does wake on its own timer regardless;
whether the ephemeris survives it - warm starts rather than cold - is the
measurement `docs/POWER.md` is still owed.

`EXTINT` and `TIMEPULSE` are not routed, so backup mode wakes on UART
traffic and there is no PPS discipline for the hop clock; the clock takes
GPS time from the sentence instead, with a guard against a late pass.
There is no battery sense divider, so telemetry cannot report cell voltage.
The Wi-Fi/BT RF port reaches test point BLE1 and stops there, so the
2.4 GHz side has no antenna on the board as drawn - BLE works at bench
range on board parasitics.

## Status display

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

The panel reads the same telemetry the BLE session notifies, so it and the
app cannot disagree. It refreshes twice a second and skips frames identical
to what is already on screen. **It costs 5-15 mA** depending on how many
pixels are lit, which is why the layout leaves most of the panel dark and
why the firmware blanks it (charge pump off, not just pixels cleared)
before every deep sleep - it sits on the always-on +3V3 and would otherwise
hold its last frame, and its current, for the whole sleep.

### The compass

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
has been rotated through a full circle - a magnetometer next to a LoRa PA
and a battery does not read a field centered on zero. Until it has,
the marker reads `G` or `T` rather than showing a confident heading built
from a quarter turn. It is also **not tilt-compensated**: hold the board
level. Correcting that needs an accelerometer, which is a different part than
the two supported here.

## SD card slot

The carrier has a microSD slot on SPI3 (GPIO46 SCK, GPIO45 MOSI, GPIO3
MISO, GPIO44 CS), and the firmware does not drive it. It did until
2026-09-11: a FAT driver logged every fix to `GPSLOG.CSV` and read
`RADIO.CFG` from the card at boot. Its mount and its walks to the end of a
grown log were single synchronous calls on the hardware loop's core that
could run for longer than the loop's heartbeat bound, and the config store
it provided is covered by the board's own flash. The four lines are parked
at boot - three pulled down, the chip select pulled up so a card in the
slot stays deselected. The driver and its documentation are in the history
before that date, and the config file keeps the 8.3 name it was given for
the card.

## GPS board v1

![GPS Board v1](../images/GPSv1.svg)
