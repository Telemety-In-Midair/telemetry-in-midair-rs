# Power settings

Every knob that changes what the Wio-S3 draws, what it costs, and where it
is set. All of them live in `RADIO.CFG` - the TOML file in the root of the
SD card, documented key by key in `RADIO.example.toml`.

This is the reference. `POWER-S3.md` is the investigation behind the
numbers: how they were measured, and what was ruled out.

## The budget these settings act on

Measured at the 4.2 V input, awake, BLE advertising, GPS tracking, LoRa
listening, nothing transmitting:

| | Draw | Set by |
|-|-|-|
| BLE controller | **71 mA** | `ble_off_s` (its lifetime is the only lever) |
| MAX-M10 GPS | ~30 mA | `power_mode`, `meas_rate_ms`, constellations |
| S3 core, never sleeping | ~12 mA | not configurable, see below |
| USB Serial/JTAG PHY | 3-5 mA | not configurable |
| SX1262 listening | ~6 mA | `role`, `rx_boost` |
| SD card, mounted idle | 1-10 mA | `sd_enabled` |
| **Total** | **~126 mA** | |

Plus one 288 ms LoRa transmit per `interval_s` at 127 mA, which averages to
about 1.3 mA at the 20 s default.

Two things to take from that table. The BLE controller is more than half of
everything, and the GPS is most of what is left - so those are the two
settings that matter and the rest is trimming. And the regulator is an LDO,
so this is the current drawn from the cell, not a higher-voltage figure that
divides down.

## The settings

### The two that matter

| Key | Range | Default | Effect |
|-|-|-|-|
| `ble_off_s` | 0, or 5-300 | 0 | Seconds the BLE controller is powered **down** between advertising windows. **Saves ~70 mA while down.** 0 keeps it up continuously. |
| `power_mode` | `full`, `psmoo`, `psmct` | `full` | GPS receiver power mode. `psmct` is cyclic tracking, `psmoo` acquires a fix then powers down until the next update. Saves up to ~25 mA; costs fix latency. |

`ble_off_s` is the largest lever the firmware has, and it is blunt: the
controller draws its 71 mA for as long as it exists, because `esp-radio`
does not implement the controller's modem sleep, so nothing reduces it
short of destroying the connector and rebuilding it. The board drops to
**60 mA** while BLE is down and cannot be connected to until the next
window. LoRa, GPS and SD logging all keep running - it is still a working
tracker throughout, just not a reachable one.

The average is whatever `adv_window_s` and `ble_off_s` make it:

| `adv_window_s` / `ble_off_s` | Average | Worst-case wait to connect |
|-|-|-|
| 15 / 0 (default) | 130 mA | none |
| 15 / 30 | ~83 mA | 30 s |
| 10 / 60 | ~70 mA | 60 s |
| 5 / 60 | ~65 mA | 60 s |
| 5 / 300 | ~61 mA | 5 min |

Past about a minute of off time the returns are gone - the average
asymptotes to the 60 mA dark reading, so the last few milliamps cost
minutes of unreachability. **60/30 is the setting to reach for.**

`power_mode` is untested on this board, because it has never held a fix
indoors. `psmct` is zero code and worth trying first; `psmoo` risks a cold
start on every cycle, which for a tracker is worse than the current it
saves.

### The duty cycle

| Key | Range | Default | Effect |
|-|-|-|-|
| `ble_off_s` | 0, or 5-300 | 0 | above |
| `adv_window_s` | 0, or 1-60 | 0 (= 15 s) | Seconds each advertising window lasts. Only meaningful alongside `ble_off_s` or `sleep_interval_s`; with both at 0 the window never ends. |
| `sleep_interval_s` | 0, or 5-300 | 0 | Seconds between deep-sleep wake checks. 0 never deep-sleeps. |

Deep sleep is the larger saving and the larger cost. It takes the whole chip
down to ~30 mA (the ungated GPS, which deep sleep cannot reach), but the
board stops beaconing, stops logging, and every wake is a full reset. Use it
for a board that is being stored, not one that is tracking.

These three are the only keys in the file the board also keeps its own copy
of - see "Where a setting lives" below.

### The radio

| Key | Range | Default | Effect |
|-|-|-|-|
| `role` | `leaf`, `repeater`, `tx_only`, `rx_only` | `leaf` | `tx_only` never enables the receiver: **saves ~6 mA continuously** and the node hears nobody. `repeater` doubles the traffic it forwards. |
| `interval_s` | 0-3600 | 20 | Beacon period. Each beacon is 288 ms at 127 mA, so this is ~1.3 mA at the default and ~0.4 mA at 60 s. 0 disables the beacon. |
| `power_dbm` | -9 to 22 | 22 | Transmit power. Only paid during those 288 ms, so dropping it buys little and costs range. |
| `rx_boost` | bool | `true` | ~+2 dB of sensitivity for a few mA while listening. Free on a `tx_only` node. |
| `spreading_factor` | 5-12 | 12 | Lower is shorter on air, so less energy per beacon, at less range. |
| `dcdc_enabled` | bool | `true` | The SX1262 internal switcher, roughly halving its RX and TX current. Leave it on - the module has the inductor. |

The beacon is the wrong place to look for current. A transmit is 1% of the
time at the default interval; a receiver that is always on is 100% of it.
`role = "tx_only"` is the real radio saving, and it is only available to a
node nobody needs to track *from*.

### The GPS

| Key | Range | Default | Effect |
|-|-|-|-|
| `power_mode` | enum | `full` | above |
| `meas_rate_ms` | 25-10000 | 1000 | Fix rate. Raising it to 5000 lets the receiver idle between solutions in a power-save mode; it does nothing in `full`. |
| `glonass_enabled` | bool | `false` | Each extra constellation is more correlator work. |
| `galileo_enabled` | bool | `true` | " |
| `beidou_enabled` | bool | `true` | " |
| `qzss_enabled` | bool | `true` | " |
| `sbas_enabled` | bool | `true` | " |

The MAX-M10 is ungated on this board: there is no GPIO under its rail, so
the firmware cannot turn it off, only ask it to use less. Dropping
constellations is worth a few mA and costs time to first fix, which is the
wrong trade for a tracker that struggles for a fix at all.

The receiver *can* be parked with a timed `UBX-RXM-PMREQ` backup, and it
wakes itself on its own timer despite this board leaving `V_BCKP`
unconnected. That is not exposed as a setting yet, because whether the
ephemeris survives the backup is unknown - if every wake is a cold start it
is useless for a tracker.

### The rest

| Key | Range | Default | Effect |
|-|-|-|-|
| `sd_enabled` | bool | `true` | Stops logging and the card's 1-10 mA. Card-dependent and worth measuring on the specific card before relying on it. |
| `verbose` | bool | `true` | Console detail. Costs nothing when nothing is attached. |

## Where a setting lives

Most keys are read from the card at boot and that is the whole story. The
three under `[power]` are different: the board also keeps them in RTC RAM,
so they survive a deep sleep, and in flash, so they survive a flat cell -
and an app can change them live over BLE, which no other key can.

That makes them the one place precedence matters:

```mermaid
flowchart TD
    FILE["RADIO.CFG [power] key"]
    LIVE["BLE write / wio-set"]
    RTC["RTC RAM<br/>survives deep sleep"]
    NVS["nvs flash record<br/>survives a flat cell"]
    RUN(["what the board runs"])

    FILE -->|"cold boot, or a deliberate push"| RTC
    LIVE -->|"immediately"| RTC
    RTC --> RUN
    RTC -->|"mirrored on change"| NVS
    NVS -->|"cold boot, if the file is silent"| RTC

    WAKE["deep-sleep wake"] -.->|"file is NOT re-read"| RTC
```

The rules that come out of it:

- **An absent `[power]` key changes nothing.** Everywhere else in the file
  an absent key means "the default"; here it means "leave the board's live
  value alone". Otherwise pushing an unrelated radio change would silently
  undo a duty cycle somebody set from the app. This is why all three ship
  commented out in `RADIO.example.toml`.
- **An explicit `0` is a request, not an absence.** Writing `ble_off_s = 0`
  is how a file turns a duty cycle off.
- **A cold boot adopts the file.** It is what survives a reflash; the RTC
  copy is not.
- **A deep-sleep wake does not.** Re-reading the card every interval would
  undo a live change once per wake, forever.
- **A live change lasts until the next cold boot**, where an uncommented key
  in the file takes over again.

One lag worth knowing: the first advertising window's length is fixed before
the card is mounted, so an `adv_window_s` from the file takes effect from the
second window on. `ble_off_s` is re-read at the end of every window and has
no such delay.

## Setting them

Push the whole file, which is what a card would carry:

```
cd tools
pixi run wio-config --set ble_off_s=30 --set adv_window_s=10
pixi run wio-config --set power_mode=psmct
pixi run wio-config --dry-run --save ../RADIO.CFG   # write a card instead
```

This sends a *whole file*, not a patch: anything absent from it reverts to
its default. `--file` starts from settings of your own rather than the
reference. The `[power]` keys are the exception - absent still means "leave
alone" for those three.

Change one of the three live, without touching the file:

```
cd tools
pixi run wio-set ble-off 30
pixi run wio-set adv-window 10
```

That is the same path a BLE config write takes, and it lasts until the next
cold boot.

## What is not configurable, and why

- **CPU clock.** Pinned at 80 MHz, the documented floor for the radio, in
  `s3/src/bin/main.rs`. It cannot come from the file: the clock is
  configured before the SPI bus that reads the card exists. Worth ~10-15 mA
  against 160 MHz, which is already taken.
- **BLE modem sleep.** Would save ~60 mA whether or not a phone is
  connected, and is the single largest thing missing. `esp-radio` leaves the
  controller's sleep callbacks as `todo!()` in every published version
  through 1.0.0-beta.0, so there is nothing to switch on. `ble_off_s` exists
  because this does not.
- **Light sleep.** The ~12 mA of S3 core that never halts for long. Needs
  tickless integration between `embassy-time` and the RTC, which `esp-rtos`
  does not provide. Design work, not a setting.
- **The GPS and LoRa rail.** The old two-MCU board had a GPIO under it; this
  one does not. Both parts sit on +3V3 and cannot be switched off, which is
  most of why this board's floor is 60 mA and the old one's was 46.
