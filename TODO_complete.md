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
