# telemetry-in-midair-rs
[Kicad Board](https://github.com/tmpk13/telemetry-in-midair) https://github.com/tmpk13/telemetry-in-midair

GPS tracker board firmware: one Seeed Wio-S3 module (ESP32-S3R8 + SX1262)
reads a MAX-M10 GPS, transmits positions over 915 MHz LoRa, logs to SD, and
serves everything over BLE to the gps-gui-rs app. `ARCHITECTURE.md` has the
UML views; `docs/` has the rest.

## Layout

| Directory | What | Target |
|-|-|-|
| `proto/` | Shared no_std protocol crate: LoRa payloads, BLE extensions, the `RADIO.CFG` key table and parser, USB bulk framing, and every policy the firmware drives - the serve loop, the hardware posture, the beacon planner, the receive gate, the dedup tables. Host-testable (`cargo test`). | any |
| `firmware/` | Wio-S3 firmware (embassy + trouble BLE). `main.rs` is the boot; `ble.rs`, `hardware.rs`, `sleep.rs` are the loops. | `xtensa-esp32s3-none-elf` (`esp` channel) |
| `explore/` | `midair-explore`: an exhaustive state-space explorer, no dependencies. `proto/tests/statespace_*.rs` and the app's models walk every ordering of what can happen to the policies above. See `docs/STATESPACE.md`. | host |
| `tools/` | Host tools (Python/pixi): configure a board over USB, write one setting, put it to sleep, push a firmware image, simulate the radio network. | host |
| `docs/` | The radio, the BLE service, the hardware, the power budget, the audits, the module datasheet. | - |

Depends on the sibling repo `../gps-proto` for the BLE position protocol
and NMEA parsing (shared with `../gps-gui-rs`).

## Build, flash, test

The ESP32-S3 is an Xtensa part, so it needs the `esp` toolchain channel:

```sh
espup install                          # once per machine

cd proto && cargo test                 # the protocol and every state space model, ~40 s
cd firmware && cargo run --release     # build, flash over USB Serial/JTAG, stay on the console
```

The console is the USB Serial/JTAG port, not UART0 (GPIO43 drives the D5
LED). `cargo run` flashes a two-slot partition table so OTA has somewhere to
write, and erases `otadata` so the image just written is the one that boots.
Nothing else is erased: a board keeps its address, its name and its duty
cycle across a reflash.

A fixed BLE address instead of the per-chip one derived from the eFuse MAC:

```sh
cd tools && pixi run gen-ble-address        # prints e.g. FF:C6:A1:53:50:47
cd ../firmware && BLE_ADDRESS=FF:C6:A1:53:50:47 cargo run --release
```

## Talking to a board

Everything below is a pixi task in `tools/`, over the USB console. The app
does the same over BLE.

| Task | What |
|-|-|
| `pixi run board-config --address 3 --set role=repeater` | Push a radio config, applied live and saved to the card and to flash. Sends a whole file: keys absent from it revert to their defaults, so start from `--file` when the board is not on stock settings. `--dry-run --save ../RADIO.CFG` writes a card file instead. |
| `pixi run board-set mode tracking` | Write one setting: `mode`, `name`, `sleep-interval`, `adv-window`, `ble-off`, `ble-on`, `idle-timeout`, `radio-standby`, `gps-sleep`. The ack says what the board stored. |
| `pixi run board-info` | The board's protocol version, BLE address and name. |
| `pixi run board-sleep --seconds 60` | Deep sleep now. The port disappears while it sleeps; that is the command working. |
| `pixi run board-ota` | Build the firmware and push it into the slot the board is not running from. |
| `pixi run radio-sim` | Simulate a few boards on the hop plan; `docs/RADIO-AUDIT.md` is what it found. |

`RADIO.example.toml` documents every config key with its range, its default
and what it does. It is generated from the firmware's own key table
(`cd proto && cargo run --example radio_example > ../RADIO.example.toml`),
so the file, the parser and the app's editor cannot disagree. It is a
reference, not a card file: the board reads at most 1024 bytes of config and
the tool strips the comments before sending.

## What a board does

A board is in one of four modes, set over BLE or with `board-set mode`:

| Mode | What is up |
|-|-|
| **stored** | nothing but a wake check on a cadence: chip asleep, GPS in backup, radio in cold sleep, card unmounted |
| **idle** | BLE only, so a phone can reach it; the rescue window a cold boot lands in |
| **tracking** | GPS acquiring, a position out over LoRa every second (a ping every five without a fix), receiver listening, card logging, BLE on a duty cycle |
| **listening** | the node beside the phone: everything tracking has, nothing transmitted, BLE up throughout |

Every node broadcasts and every node listens, so a fleet of leaves already
works; a `repeater` forwards what it hears. Nodes take turns inside a slot
clock kept on GPS time or on a heard frame, so addresses `1` and `2` beacon
every second without ever overlapping. The default modulation is SF12 at
500 kHz on one carrier; hopping is a setting.

Awake and advertising the board draws about 126 mA, 71 of them the BLE
controller, so the BLE duty cycle (`ble_off_s`) and the GPS power mode are
the two settings that matter.

## Where to read more

| | |
|-|-|
| `ARCHITECTURE.md` | The parts, the RF path and why two registers are not tunable, a BLE session end to end, the modes, the slot clock, the states over time, and the state space walked. |
| `docs/RADIO.md` | The config and where it lives, the modulation, the slot clock, the beacon and the ping, leaves and repeaters, the GPS settings. |
| `docs/BLE.md` | The GATT service, the remote-node roster, the config ids, board names, the four modes, sleep, the bulk transfer and OTA. |
| `docs/HARDWARE.md` | The Wio-S3 module and its SKU, the pin map, the strapping pins, the connectors, the status panel and compass, the SD card, the GPS antenna. |
| `docs/POWER.md` | Every setting that changes what the board draws, the measured budget, the levers not yet pulled. |
| `docs/STATESPACE.md` | The exhaustive models, what they cover, what they found, how to add to them. |
| `docs/RADIO-AUDIT.md` | The radio path rebuilt in a simulator and what it changed. |
| `docs/SYSTEM-AUDIT.md` | The redundancies and the systems that were not what they should be, and the fourteen items that fixed them. |
| `docs/BOARD-V1-ISSUES.md` | What the V1 carrier gets wrong. |
| `TODO.md`, `TODO_complete.md` | What is left, and the shape of what was done. |
