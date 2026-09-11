# telemetry-in-midair-rs

Firmware for a GPS tracker board built on one Seeed Wio-S3 module
(ESP32-S3R8 + SX1262): a MAX-M10 GPS in, positions out over 915 MHz LoRa,
logging to SD, and BLE to the [gps-gui-rs](../gps-gui-rs) app. The board
itself is [telemetry-in-midair](https://github.com/tmpk13/telemetry-in-midair).

## Layout

| Directory | What | Target |
|-|-|-|
| `proto/` | Shared no_std protocol crate and every policy the firmware drives. `cargo test` runs on the host. | any |
| `firmware/` | The Wio-S3 firmware: `main.rs` boots, `ble.rs` and `hardware.rs` are the two loops. | `xtensa-esp32s3-none-elf` |
| `explore/` | Exhaustive state-space explorer; the models in `proto/tests/` and the app use it. | host |
| `tools/` | Host tools over USB, as pixi tasks. | host |
| `docs/` | The radio, the BLE service, the hardware, power, the audits. | - |

Depends on the sibling `../gps-proto` for the BLE position protocol and
NMEA parsing.

## Build, flash, test

```sh
espup install                          # once: the esp toolchain channel
cd proto && cargo test                 # protocol tests and the state space models, ~40 s
cd firmware && cargo run --release     # build, flash over USB Serial/JTAG, stay on the console
```

The console is the USB Serial/JTAG port. A reflash erases only `otadata`,
so a board keeps its address, name and settings. For a fixed BLE address:
`cd tools && BLE_ADDRESS=FF:C6:A1:53:50:47 cargo run --release` (`pixi run
gen-ble-address` makes one).

Reset a board: `pixi run board-wipe` clears the settings, the name and the
config backup and restarts; `pixi run board-wipe --flash` erases every byte
of flash and reflashes. The card is never touched, so a `RADIO.CFG` on it
comes back at the next boot.

## Talking to a board

From `tools/`, over USB; the app does the same over BLE.

| Task | What |
|-|-|
| `pixi run board-config --address 3` | Push a whole radio config, applied live and saved to the card and flash. `--set key=value`, `--file`, `--dry-run --save ../RADIO.CFG`. |
| `pixi run board-set mode tracking` | Write one setting. The ack says what the board stored. |
| `pixi run board-info` | Protocol version, BLE address, name. |
| `pixi run board-sleep --seconds 60` | Deep sleep now. |
| `pixi run board-wipe` | Forget the settings, the name and the config backup, then restart. `--flash` erases the whole part and reflashes. |
| `pixi run board-ota` | Build and push a firmware image into the other slot. |
| `pixi run board-log` | What the board wrote down about itself: every boot and why, every panic, every stall and its phase. `--last 10`, `--clear`. |
| `pixi run radio-sim` | Simulate a few boards on the hop plan. |

`RADIO.example.toml` documents every config key. It is generated from the
firmware's key table (`cd proto && cargo run --example radio_example`), so
the file, the parser and the app's editor cannot disagree.

## What a board does

| Mode | What is up |
|-|-|
| **stored** | a wake check on a cadence; otherwise asleep |
| **idle** | BLE only; where a cold boot lands |
| **tracking** | GPS, a beacon every second, receiver, card, BLE on a duty cycle |
| **listening** | tracking without the transmitter, BLE up throughout |

Every node broadcasts and listens; a repeater forwards. Nodes take turns
on a slot clock kept on GPS time or a heard frame, so two nodes beaconing
every second never overlap. SF12 at 500 kHz on one carrier by default;
hopping is a setting. Awake, the board draws about 126 mA, 71 of them BLE.

A board that stops comes back on its own. Both loops report what they are
doing; a monitor feeds a hardware watchdog while they do, and a loop that
goes quiet, or a panic on either core, is written to the board's own flash
with its name and phase before the reset. `pixi run board-log` reads it.

## Read more

| | |
|-|-|
| `ARCHITECTURE.md` | The parts, the RF path, a BLE session, the modes, the slot clock, the states over time, the state space walked |
| `docs/RADIO.md` | Config, modulation, the slot clock, beacons and pings, repeaters |
| `docs/BLE.md` | The service, config ids, names, modes, sleep, bulk transfer and OTA |
| `docs/HARDWARE.md` | Module, pins, connectors, panel and compass, SD card, GPS antenna |
| `docs/POWER.md` | What each setting costs, the measured budget, the levers left |
| `docs/STATESPACE.md` | The exhaustive models: coverage, findings, how to add to them |
| `docs/RADIO-AUDIT.md`, `docs/SYSTEM-AUDIT.md` | The radio simulated; the system read end to end |
| `TODO.md`, `TODO_complete.md` | What is left; what was done |
