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

## Bench work

Work the power list in `docs/POWER-AUDIT.md`, which is ordered by what it
is worth. The board is measured - ~126 mA awake, a 60 mA floor with BLE
dark - so the open items are levers, not unknowns. The first three are
clearing `BT_STATE` in `ble_init`, restoring the Wi-Fi clock and power-down
bits after the BLE connector drops, and running `--features iso-gps-backup`
to get the one reading the investigation never took.

Soak the BLE duty cycle. `esp_radio::init` and `BleConnector::new` now run
once per window rather than once at boot, thousands of times a day at a
45 s cycle, and both are `expect`s on a heap that the controller allocates
from every cycle. Leave a board running overnight with `ble-off 30` and
check the wake counter and the free heap.

Check whether an OTA over BLE survives its own flash writes. Each sector
takes tens of milliseconds with interrupts off, which should cost a
connection event rather than the connection - "should" being the word doing
the work.

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
of PSRAM it no longer competes with the link task for room. It would want a
partition of its own, which is a `partitions.csv` change and so a reflash
rather than an OTA - worth deciding before a fleet is deployed.

A listener dongle, possibly with a USB bridge.

Beeper.

## Open questions

Will flashing the firmware with a `RADIO.CFG` present overwrite flags such
as the node address? (It should not: the card is read at boot and the card
wins. Untested.)

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
board reachable.

Route the module's Wi-Fi/BT RF port to an antenna. It reaches test point
BLE1 and stops there on the board as drawn, so the 2.4 GHz side has no
antenna despite the module bringing the port out.

Battery sense divider. There is none, so telemetry cannot report cell
voltage without a board change.

Route GPS EXTINT to the MCU, and TIMEPULSE for PPS discipline. Neither is
connected today; backup mode still wakes on UART traffic, but PPS is simply
unavailable.
