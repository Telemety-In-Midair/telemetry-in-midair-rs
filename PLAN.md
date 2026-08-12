# Plan

One Seeed Wio-S3 module (ESP32-S3R8 + SX1262 + TCXO) does everything.

The module reads a MAX-M10N-10B over UART. Positions go out over LoRa
(915 MHz), are logged to the SD card, and are served over BLE to an external
app for display and configuration.

The radio is configured by a TOML file, stored on the SD card as `RADIO.CFG`
and/or pushed over BLE at runtime, which also rewrites the card copy. Two
of its keys - the antenna switch on DIO2 and the DIO3 supply that powers it
- describe the module rather than a preference, and the firmware enforces
them: a wrong value there transmits into an isolated port and destroys the
module.

The GPS can be put into backup mode over BLE, and the radio into standby.
The board itself can be put to sleep, waking on an interval to advertise for
a window. There is no rail to cut on this board - the GPS and SD sit
directly on +3V3 - so GPS backup mode is the only real power lever, and a
sleeping module sits beside a receiver that is still acquiring.

The SD card is optional to run. The firmware caches the latest data (LoRa
RSSI/SNR, GPS coordinates, counters) and serves it as soon as BLE connects.
SD logs should be readable by a phone or computer.

Firmware updates go through the ESP-IDF bootloader's two-slot OTA with
rollback, so a bad image reverts rather than bricking the board.

D5 (GPIO43) and D2 (GPIO14) are the status LEDs. Both are active low - the
anodes sit on +3V3 - and GPIO43 is also UART0_TX, so the ROM bootloader's
boot log flickers D5 on every reset.

All LoRa traffic is broadcast. A node is a leaf by default and hears every
other node in direct range; one configured as a repeater retransmits what it
hears, extending coverage past a single radio horizon.

## What this replaced

An ESP32-C6 connected over a framed UART link to a WIO-E5: the C6 was the
BLE face and power master, the WIO did GPS, LoRa and SD, and firmware
reached the WIO either over SWD or streamed through the C6 into a DFU
partition. Roughly a third of that firmware existed only to bridge the
split - the link, its heartbeat, the ack/retry around every command, the
`RADIO_BUSY` negotiation, the WIO's soft sleep, and the swap bootloader.
`PORT-WIO-S3.md` records the merge; `git log` still has the code.
