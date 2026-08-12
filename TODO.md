## Port parity (PORT-WIO-S3.md, steps 4b and 5)

The single-module firmware does not yet do everything the two-MCU pair did.

Bulk transfer handler. The BLE characteristic is declared so the service
shape matches, but nothing handles a write, and the USB console it shares a
path with is not up either - which is what `pixi run wio-config` needs.

Deep sleep, with settings that survive it. `Stored` is a plain static that
resets with the board; it needs RTC RAM plus the nvs mirror. Note the board
changes the sums: there is no rail to cut, so a sleeping S3 sits beside a
MAX-M10 that is still acquiring, and GPS backup mode is the only real lever.

Remote-node roster replay on connect.

OTA. `esp-bootloader-esp-idf` is already a dependency and the ESP-IDF
bootloader does two-slot OTA with rollback; nothing drives it. This is what
replaces the WIO's swap bootloader and `fw-upload`.

Per-board BLE addresses. The C6 derived one from its eFuse MAC and let
`--ble-address` override it at build time; `tools/gen_ble_address.py`
survives and has nothing to feed.

## Bench work

Run the radio against an existing node. The air format did not change, so a
ported board has to talk to an unported one - that is the test that says the
port is real. Everything in `s3/` is written and builds; almost none of it
has been run.

Confirm the eFuse state on a real module, particularly `VDD_SPI_FORCE` -
three of the four SD lines sit on ESP32-S3 strapping pins.

Measure power. The numbers in `README.md` are the old board's, kept only as
a baseline to beat.

## Radio

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
of PSRAM it no longer competes with the link task for room. Decide before
the sleep work - it changes the sleep budget and the partition table.

A listener dongle, possibly with a USB bridge.

Beeper.

## Open questions

Will flashing the firmware with a `RADIO.CFG` present overwrite flags such
as the node address?

Will a sleeping board ever be connected to if an awake board is nearby?

The `PMode::Boost` value: the WIO-E5 build wrote 0x97 to the RX gain
register, which its HAL documented as best sensitivity, but Semtech's
datasheet documents only 0x94 and 0x96. The s3 port writes the register
directly and uses the documented 0x96. With the WIO firmware gone there is
no longer an A/B to run, so this is settled unless RM0453 says otherwise.

## Done

Antenna for BLE - the module brings the Wi-Fi/BT RF port out on its own
connector. Note the board as drawn routes it to test point BLE1 and stops
there, so a 2.4 GHz antenna is still a board change.

Swap to Wio-S3. Done; `wio/` and `esp/` are deleted.

Try a slow preset now that nothing caps the listen window - defaults are
SF12/BW500.

Reduce packet size: payloads go out at their true length.

Note the sync word is a flag day. Nodes on different sync words cannot hear
each other at all, so reflash every node before testing - a partially
updated fleet looks exactly like a range problem.
