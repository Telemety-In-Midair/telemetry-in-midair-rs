# Completed

Items retired from `TODO.md`. Kept so the shape of the port is legible
without reading the whole log.

## Port to the Wio-S3

Port parity with the two-MCU pair. The bulk transfer handler and the USB
console `wio-config` needs; deep sleep with settings in RTC RAM and an nvs
mirror; the remote-node roster replay on connect; OTA through the ESP-IDF
bootloader's two slots; per-board BLE addresses from the eFuse MAC with a
`BLE_ADDRESS` build override.

The beacon itself, which the port never had: a position on the configured
interval, a ping while there is no fix, `(src, id)` dedup, jittered
repeating, and the radio re-checked before it keys up.

`RADIO.CFG` read from the card at boot and honored, rather than compiled-in
defaults with the file sitting unread.

Antenna for BLE - the module brings the Wi-Fi/BT RF port out on its own
connector. Note the board as drawn routes it to test point BLE1 and stops
there, so a 2.4 GHz antenna is still a board change.

Swap to Wio-S3. Done; `wio/` and `esp/` are deleted.

Try a slow preset now that nothing caps the listen window - defaults are
SF12/BW500.

Reduce packet size: payloads go out at their true length.

## The three modes (docs/STATES-PLAN.md)

Stored / idle / tracking as one `CFG_MODE` setting, persisted in RTC RAM and
mirrored to nvs (record version 5, settings blob version 5), replacing the
pair of independent sleep flags as the thing an app sets.

Three boot flavors from that mode plus the wake cause: a wake check raises
nothing, idle raises BLE and the card, tracking raises everything. A cold
boot lands in idle - reachable, with the GPS down - so a board recovered
from a flat cell has a rescue window, and only a stored `tracking` puts a
board back on the air by itself.

A connect during a wake check promotes the board to idle with the timeout
armed, on the attempt rather than on a completed session: the connection is
a doorbell rather than a leash.

The knobs scoped to one mode each - `sleep_interval_s` as Stored's cadence,
`idle_timeout_s` as Idle's, `ble_off_s` as Tracking's - which un-deadens
`ble_off_s` on any board that had a wake-check cadence.

The park path made real: the SD buffer flushed and the card unmounted (it
was discarding up to 5 s of fixes per sleep), the GPS taken into PMREQ
backup, and UART TX pad-held across the sleep alongside NSS so a floating
edge cannot wake the receiver out of it. Wake checks no longer run
`gps.configure`, which was waking the receiver on every single wake.

`wio-set mode` and `wio-set idle-timeout` on the USB console; `idle_timeout_s`
in the card's `[power]` section.
