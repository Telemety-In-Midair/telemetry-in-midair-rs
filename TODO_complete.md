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

## BLE modem sleep in the vendored esp-radio

The controller powers its own PHY down between advertisements and between
the connection events of an idle connection. Upstream ships this
unimplemented rather than unconfigured - the sleep callbacks are `todo!()`
and `ble_init` runs no enabling sequence - so it is a port of ESP-IDF's
`components/bt/controller/esp32c3/bt.c`: the low power clock setup, the
cycle arithmetic (both conversions were wrong, one by a factor of two),
the callbacks with the in/out pointer signatures they actually have, and a
wake path so that an HCI send and the teardown can talk to a sleeping
controller.

On by default, `--features iso-ble-no-modem-sleep` for the A/B. What it
saves is not measured yet, which is why the bench list starts with it.

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

## Listening, the tracker's own on period, and idle that stays idle

A fourth mode, `listening` (`CFG_MODE` = 3): the node held beside the
phone. GPS acquiring and the receiver up, so everything heard is relayed and
the phone can take this node's fix as its own; nothing transmitted - the
beacon, the ping and the repeater path are all gated on `Mode::transmits`;
BLE up throughout, so the phone connects at once. Persisted like tracking.

The tracker's modem on period is its own setting, `ble_on_s` (`0x1A`,
record version 7, settings blob version 6). It was the advertising window,
and a wake check and a tracker never wanted the same window.

The idle timeout is off by default: `0x18 = 0` means an idle board stays
idle until told otherwise, rather than storing itself ten minutes after a
connect. A version 6 record's 0 now reads as off, which is the change
wanted.

## Board names

Boards advertise as `ws3gps-<label>`, set by config id `0x19` over BLE or
`wio-set name` over USB, with an unnamed board falling back to the tail of
its BLE address so two out of the same box are still told apart. The label
is stored with the settings that decide reachability - RTC RAM mirrored to
`nvs`, record version 6 - rather than on the card, because a wake check
advertises before anything has mounted one. It comes back on the name
characteristic (`c3a1000c-...`) and in the USB `INFO` reply, and the prefix
is a firmware constant so a board can never be named something a scanner
cannot find.

## Config backup in flash

A pushed config now goes into a record in the `nvs` partition as well as onto
the card, and the boot path reads it when the card has nothing to say - so a
board with no card, or with a card that has failed, comes back on the config
it was given rather than on firmware defaults with the node address reset to
1. The card still wins at boot, and a boot that reads one refreshes the
backup from it. The record is `midair_proto::cfgstore`, host-tested: a length
and a crc in front of the config text, so an interrupted write reads as
nothing stored rather than as a config half of which is the previous one. This restores what the two-MCU board kept in
the WIO-E5's flash page 122 and the S3 port had dropped.

## The radio audit (2026-09-07)

A full read of the hop, receive, GPS and BLE paths, rebuilt in a
discrete-event simulator (`tools/radio_sim.py`, self-tested against
`proto`'s hop clock) and written up in `docs/RADIO-AUDIT.md`. The clock
message suspected of colliding with beacons does not exist - the sync word
rides inside every frame - and the causes were elsewhere:

- Nodes now take turns inside a slot by address (two at the default
  modulation), and the slot of a multi-slot interval by address too, so
  `2 x interval_s` consecutive addresses never overlap; the firmware warns
  when it hears a node in its own turn, and the app's Radio page shows the
  capacity.
- The BLE position notifier waits out a transmit instead of skipping its
  tick, which was dropping 23% of updates at a 1 s beacon.
- GPS bytes go through an async pump task into a pipe, so a transmit or a
  card flush no longer overflows the 128-byte UART FIFO.
- A late pass no longer disciplines a set clock; the sync word is stamped
  for the RF start; listening nodes keep the TCXO running between modes; a
  preamble with no header behind it holds the receiver for the header time
  rather than a whole frame; a beacon whose window passed is re-planned
  rather than waited for.
- The hardware loop runs on the S3's second core with its own executor
  (`dual-core`, default on).

Two-node delivery in the model went from 76% to 99.6-100% with zero
overlaps, GPS sentence loss from 11% to 0%, and phone position updates
from 208 to 300 per 300 s.

## State space testing (2026-09-08)

`explore/` is an exhaustive state-space explorer; `proto/` carries the
serve loop, the hardware posture, the request set and the receive gate as
host-testable machines the firmware drives; `proto/tests/statespace_*.rs`
walk the composed board, the gate and the roster. Nine firmware changes
came out of it (`docs/STATESPACE.md`), all unflashed.

The app's Radio page band rule keys on `channels > 1` rather than on a
plan existing, so the one-channel default reads as the single carrier it
is. Done in `gps-gui-rs`.
