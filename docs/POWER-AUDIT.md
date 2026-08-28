# Critical audit: the Wio-S3 power story

A read of the firmware and of the vendored `esp-radio` against the claims in
`POWER-S3.md`. Short version: **126 mA is not normal**, the measurement work
in that document is sound, and its closing conclusion is not. "60 mA is the
floor and past a minute of off time the returns are gone" rests on a floor
that was never decomposed. Three of its four terms have levers nobody has
pulled, and one of them is not mentioned in the document at all.

Separately, reading the vendored crate turned up two things that are code
defects rather than estimates - one of them a correctness bug in the new
duty-cycle path.

## What the existing document gets right

Confirmed by inspection, so do not re-litigate these:

- Modem sleep really is unimplemented. `btdm_sleep_check_duration`,
  `_enter_phase1/2`, `_exit_phase1/2/3` and `btdm_lpcycles_2_hus` are all
  `todo!()` at `firmware/vendor/esp-radio/src/ble/btdm.rs:237-259`, and `ble_init`
  (same file, 305) runs no `btdm_lpclk_select_src` /
  `btdm_controller_set_sleep_mode` / `btdm_controller_enable_sleep`
  sequence. Setting the two config literals could only ever have panicked.
- 71 mA for BLE is arithmetically consistent with the PHY being powered
  100% of the time: it is roughly an S3's RF-working current minus its
  modem-sleep current.
- The `btdm_controller_disable()` patch before `btdm_controller_deinit()`
  is right and matches ESP-IDF's state machine.
- The HCI transport is genuinely waker-driven
  (`ble/controller/mod.rs`, `HciReadyEventFuture`). No busy-poll, so the
  "is the CPU spinning inside trouble-host" theory is dead.
- The application really is ~6 mA. The port's architecture is fine.

## A. Defects in the vendored crate, verifiable from source

### A1. Stale HCI state survives teardown - a bug, not a power item

`vendor/esp-radio/src/ble/mod.rs:56`

```rust
static BT_STATE: NonReentrantMutex<BleState> = NonReentrantMutex::new(BleState {
    rx_queue: VecDeque::new(),
    hci_read_data: Vec::new(),
});
```

`ble_deinit` (`btdm.rs:387`) does not clear it and neither does `ble_init`.
Every controller-to-host packet the window closed on top of - a pending
`Disconnection Complete`, a command-complete for a command the previous
stack sent - is still queued when the next window's fresh `trouble-host`
runner makes its first `Transport::read`. It gets handed a packet for a
connection handle that no longer exists.

Symptom to expect: intermittent `ble host error, restarting` on the first
pass of a window, or an advertise that fails once and retries. Nothing about
it looks like a teardown problem from the console.

Fix is two lines in a crate that is already vendored: clear both fields at
the top of `ble_init`. Do this before spending any more bench time on the
duty cycle, or the noise will be attributed to the wrong thing.

### A2. The modem power domain is never powered back down

`esp_radio::init()` calls `enable_wifi_power_domain()`, which clears
`RTC_CNTL.dig_pwc.wifi_force_pd` (`common_adapter.rs:346`), and
`init_radio_clocks()`, which ORs `SYSTEM_WIFI_CLK_EN` (0x00FB9FCF) into
`APB_CTRL.wifi_clk_en`.

`Controller::drop` (`vendor/esp-radio/src/lib.rs:227`) does
`shutdown_radio_isr()` and nothing else. `wifi_force_pd` is never re-set
anywhere in the crate - grep confirms it appears exactly once. The
`PhyClockGuard` drop does clear the narrower BT/WiFi common mask
(0x78078F), so the PHY clock goes, but the digital power domain stays up
and the wider clock enable stays set for the whole dark period.

**This is the "~5 mA looks like teardown residue" open item in
`POWER-S3.md`, promoted from a guess to a source-level finding.** It is two
register writes at the end of the window, and it is directly testable
against the 60 mA dark reading vs. the 55 mA that reading was predicted to
be.

### A3. Every advertising window pays a full RF calibration

`esp-phy`'s `PhyState` starts with `calibration_data: None`, so
`enable_phy()` runs `PHY_RF_CAL_FULL` rather than `PARTIAL` on every window
open. Nothing in the firmware calls the backup path, and it is public:
`esp_radio::phy_calibration_data()` and
`esp_radio::set_phy_calibration_data()` are re-exported at
`vendor/esp-radio/src/lib.rs:141`.

Energy cost is small at a 45 s cycle. The reason to fix it is that it is
~100 ms of high-current work in front of the first advertisement of every
window, which is latency a phone feels.

### A4. Upstream says out loud that low power is not done

`common_adapter.rs:328-344`: `phy_enable_clock` and `phy_disable_clock` are
both entirely commented out, with

> This might have some low-power issues, but we're not there yet anyway.

Probably benign here because esp-hal's guard covers the same bits, but it
is the maintainer stating the crate's position on power. Worth knowing
before planning around it.

## B. The 60 mA "floor" is three untried levers

By subtraction from the isolation builds: ~30 mA GPS, ~15-20 mA S3 core,
~5 mA SX1262, ~4 mA USB PHY, plus A2's residue. Each one:

### B1. The GPS is at its maximum-power settings by default, and has never been measured

`GpsConfig::default()` is GPS + Galileo + BeiDou + QZSS + SBAS, 1 Hz,
`PowerMode::Full` (`proto/src/radiocfg.rs:310`). Four concurrent
constellations and a 1 Hz solution for a node that beacons every 30 s.

Two config-only reductions, zero code: `power_mode = "psmct"`, and dropping
constellations the fix does not need. Neither has been tried.

More to the point: **`--features iso-gps-backup` has never been run**, so
"25-31 mA" is still a datasheet estimate carrying every downstream
conclusion in `POWER-S3.md` - including the claim that the floor is a
floor. It is the single cheapest measurement left and the document already
identifies it as such, twice, without it having happened.

### B2. The S3 core never sleeps, and the document does not mention this

esp-rtos idles at `waiti 0` with every clock running at 80 MHz. That is
15-20 mA of the dark period spent doing nothing.

`POWER-S3.md` dismisses light sleep on the grounds that entering it with the
BLE controller up is what `sleep_mode`/`sleep_clock` were supposed to
arrange. True - and beside the point, because **the duty cycle already
creates a window in which no BLE controller exists.** During `ble_off_s`
there is nothing to coordinate with, which is exactly when `Rtc::sleep_light`
is legal.

The wake sources are there. `Ext0WakeupSource<P: RtcPin>`,
`Ext1WakeupSource` and `RtcioWakeupSource` are all implemented for the S3
(`esp-hal-1.0.0/src/rtc_cntl/sleep/esp32s3.rs`), and **GPIO9 - the SX1262's
DIO1 - is inside the S3's RTC GPIO range (0-21)**, so a received LoRa packet
can wake the core. The firmware already steals an RTC pin singleton for the
GPIO21 pad hold in `enter_deep_sleep`; this is the same move.

The real cost is not the wake source, it is that embassy's timebase needs
resyncing across a light sleep and the hardware loop has to be restructured
around it. That is a week, not an afternoon. It is also the only item on
this list that gets the board below ~40 mA.

### B3. The SX1262 is in continuous RX for a network with 30 s beacons

`enter_rx` arms `RX_CONTINUOUS` (0x00FF_FFFF, `src/sx1262.rs:170`) and it
persists. `SetRxDutyCycle` (0x94) is the part's own answer and is already
written down in `TODO.md`. ~5 mA down to ~1-2, at the cost of picking a
sleep/RX ratio against the beacon interval.

## C. The headline number is the shipped default

`Stored::new()` gives `sleep_interval_s = 0` and `ble_off_s = 0`. Out of the
box the board sits at 126-130 mA indefinitely, and both levers are opt-in
over BLE.

For deep sleep, "an unconfigured board stays reachable" is the right call
and `POWER-S3.md` defends it well. For the BLE duty cycle the argument is
much weaker: it does not make the board unreachable, only intermittently
connectable, and LoRa and the GPS stay up throughout. A default of
`adv_window 15 / ble_off 30` would take the out-of-box figure from 130 mA to
~83 mA for a 30 s worst-case connect wait.

## D. The duty-cycle path is not soak-tested

Two things at `firmware/src/bin/main.rs:533-536`:

```rust
let radio = esp_radio::init().expect("radio init");
let transport = ...BleConnector::new(&radio, bt, ble_config).expect("ble connector");
```

Both run once per window - thousands of times a day at a 45 s cycle. A
transient failure is a panic and a reset, on a path that used to run exactly
once at boot. These want a log-and-retry, not an `expect`.

And `btdm_controller_mem_init` plus the blob's allocations go through the
64 KiB `esp-alloc` heap on every cycle. Nothing has established that a few
thousand init/deinit rounds do not fragment it. A single-cycle bench reading
does not answer that; leave a board running overnight with `ble-off 30` and
check the wake counter and free heap.

## E. What is left after all of it

None of the above touches the connected case. With a phone attached the
board reads 130 mA and the duty cycle is inert - the radio is on 100% of the
time and no firmware lever in this repo changes that. If connected-mode
runtime matters, there are two honest options and they are both large:

1. Port ESP-IDF's `bt.c` sleep sequence and the six ROM callbacks into the
   vendored crate, including the RTC cycle arithmetic. Real work, bounded,
   and it is the only thing that helps while connected.
2. Move BLE off the S3.

## Order to work in

1. **Clear `BT_STATE` in `ble_init`.** Correctness. Two lines. Do it first
   so the bench readings are not contaminated by phantom host errors.
2. **Re-set `wifi_force_pd` and restore `wifi_clk_en` after the connector
   drops**, then re-measure the dark period. Predicted 60 -> ~55 mA. Two
   register writes, and it settles A2 either way.
3. **Run `--features iso-gps-backup`.** The one measurement the whole
   investigation is missing.
4. **`power_mode = "psmct"`, and drop BeiDou/QZSS/SBAS.** No code.
5. **Cache the PHY calibration data across windows.** Latency, not current.
6. **Light sleep during the BLE-dark period, woken by DIO1 (GPIO9) or the
   beacon timer.** The only path below ~40 mA that does not need a board
   respin.
7. Then the board changes already in `POWER-S3.md`: V_BCKP to +3V3, a load
   switch under the GPS and SX1262, a buck in place of U2.
