# Development Notes

## Crate/version pins (all deliberate, do not bump blindly)

- `esp-hal` is pinned at `~1.0` (locked 1.0.0) because esp-rtos 0.2 /
  esp-radio 0.17 / esp-phy 0.1 fail to compile against esp-hal 1.1 (missing
  `ModemClockController`, `InterruptHandler::new_not_nested`). The whole
  esp-generate 1.2.0 pinned set from esp32c3-gps was kept: esp-rtos 0.2,
  esp-radio 0.17, trouble-host 0.5, bt-hci 0.6.

  **That rationale is now out of date and the pin is a choice, not a
  constraint.** Checked 2026-08-26: esp-radio 0.18.0 (2026-04) wants
  esp-hal `~1.1.0-rc.0` with esp-phy 0.2, esp-radio-rtos-driver 0.3 and
  bt-hci 0.8; esp-radio 1.0.0-beta.0 (2026-06) wants esp-hal `~1.1.0`. The
  matched set exists upstream now, and esp-hal itself is at 1.1.2 with
  1.2.0-rc.0 out. The blocker was real when written and is not any more.

  Upgrading buys a year of fixes but **not** BLE power - `sleep_mode` and
  `sleep_clock` are still literal zeros in 0.18.0 and 1.0.0-beta.0, and
  every modem-sleep callback in `btdm.rs` is still `todo!()`. The bt-hci
  0.6 -> 0.8 step also drags trouble-host, so it is a real port, not a
  version bump.
- `embedded-io` 0.6 on the WIO: `embedded-nano-mesh` 2.1.x bounds
  `Node::update` on embedded-io 0.6 traits (same finding as
  long-range-radio-rs).
- `embedded-sdmmc` 0.8 with `default-features = false`, used only for the
  FAT layer through its transport-agnostic `BlockDevice` trait. Its SdCard
  driver type needs embedded-hal 1.0 which stm32wlxx-hal 0.6 does not
  implement - the SPI-mode SD driver is in-repo instead (adapted from the
  long-range-radio charger board, CSD capacity read added for
  `num_blocks`).
- `trouble-host` needs `default-packet-pool-mtu-251` or the default pool
  MTU (27) caps ATT writes; the bulk characteristic wants ~200-byte
  chunks.

## Layout: three separate builds, no root workspace

Mixed targets (thumbv7em nightly for the WIO, riscv32imac stable for the
C6, host for proto tests) cannot share one cargo workspace; each
directory carries its own `.cargo/config.toml` + `rust-toolchain.toml`
and must be built from within. `wio/` is itself a workspace with the
bootloader (same target); the memory.x-through-OUT_DIR trick from
long-range-radio-rs prevents the app layout shadowing the bootloader's
(verified: bootloader vector table at 0x08000000, app at 0x08004000).

## ESP32-C6 specifics

- UART0 (GPIO16 TX / GPIO17 RX) is the WIO link, so `esp-println` is
  pinned to `jtag-serial` (default `auto` would fall back to UART0 writes
  when USB is unplugged and corrupt the link).
- The `#[ram(reclaimed)]` heap region is exactly 64 KiB on the C6; the
  esp-generate C3 value (66320) overflows the linker region by 784 bytes.
- `#[ram(unstable(rtc_fast, persistent))]` statics must implement
  esp-hal's `Persistable`, which only covers primitives and
  portable-atomic types - hence three `portable_atomic::AtomicU32`s
  (magic/interval/flags) instead of one struct. Magic word gates
  cold-boot garbage since persistent RAM is not zero-initialized.
- Deep sleep: GPIO2 (LDO EN) and GPIO6 (WIO RST, open-drain) are
  pad-held via `RtcPin::rtcio_pad_hold(true)` before `Rtc::sleep_deep`
  so the WIO keeps power while the C6 sleeps; holds are released at boot
  after the Outputs are reconfigured (order matters - reconfigure first,
  then release, or the pins glitch). The pin singletons are `steal()`n
  for the hold calls because the Outputs own them.
- `SleepSource` has no `PartialEq`; use `matches!`.

## WIO-E5 specifics

- USART kernel clock: both UARTs (`Gps` USART1, `EspLink` USART2) select
  `uart::Clk::Hsi16`, but `Uart::new` only sets the source mux - it does
  NOT enable the HSI16 oscillator. `raise_sysclk` only touches the MSI, so
  HSI16 stayed off and the USART clock was dead: RX `poll()` just returned
  WouldBlock, but the first blocking TX (`send_status` at the top of the
  run task) spun forever in `nb::block!(uart.write)` with no watchdog feed,
  so the 6 s IWDG reset the chip into the bootloader. probe-rs reported it
  as `Firmware exited unexpectedly: Exception` with frame 0 at 0x08000400,
  which is the bootloader's `__stext` (reset entry) - i.e. a reset, not an
  in-place fault. Fix: `platform::enable_hsi16` (HSION + wait HSIRDY) in
  init before either UART is created.
- RTIC dispatcher: the PAC names the SPI2 interrupt `SPI2S2`; used `DAC`
  as the (unused) dispatcher instead.
- rtt-target's `rprintln!` expands with a trailing semicolon; on current
  nightly the reference project's `debug_println!` (expression-position
  if block) is a hard error - the macro body needs `rprintln!(...);`
  inside a statement block.
- Radio params are runtime (`RadioConfig`) instead of the compile-time
  LORA_PRESET scheme: `Sx1262Driver::init(&cfg)` can be called again on a
  live radio (config push re-inits; address/listen changes also recreate
  the MeshNode). TX timeouts and LDRO derive from the config; the LDRO
  threshold comparison must be inclusive (SF11/BW125 is exactly 16.384 ms
  = the 1638 fixed-point boundary).
- This board has no SD card-detect switch (unlike the charger board):
  presence = successful init; every FAT/SPI error unmounts + deinits and
  a 10 s retry remounts, so hot-plug works.
- FAT logging opens/appends/closes `GPSLOG.CSV` per flush (5 s or
  half-full buffer) so the directory entry stays consistent on power
  loss; RAM buffer drops oldest half when no card is present.
- FAT timestamps are a fixed 2026-01-01 (GPS time-of-day alone cannot
  build a calendar date; RMC date field is not parsed by gps-proto).
- Firmware update over UART reuses the long-range-radio swap bootloader
  unchanged (pages: boot 0-7, ACTIVE 8-63, DFU 64-120, state 121).
  Stop-and-wait framing (each FW_DATA acked before the next) means page
  erase (~20-40 ms) cannot overrun the UART - there is no flow control.
  CRC32 is verified over the DFU partition before SWAP_PENDING is set;
  the ESP's 5 s FW_END timeout covers the ~112 KB CRC pass.
- The mesh pads payloads with 0x00; position messages (tag 0x50) are
  fixed-size binary and exempted from the null-truncation in
  `MeshNode::receive` (same class of bug as the OTA chunk truncation in
  long-range-radio-rs).

## Radio coordination (PLAN: "avoid using all radios at once")

Soft flags with a 3 s staleness timeout in both directions:
- WIO -> ESP `RADIO_BUSY(1)` around each LoRa beacon (cleared after
  2*listen_ms + 500 ms); the ESP notifier defers its BLE notification
  ticks while set. BLE connection keep-alive cannot be paused - only
  discretionary traffic is deferred.
- ESP -> WIO `RADIO_BUSY(1)` during bulk (config/firmware) transfers,
  refreshed per chunk; the WIO defers beacon broadcasts while set but
  re-checks every 500 ms, so a stale flag cannot stop beaconing.
The hard path is the LDO: BLE `CFG_PWR_EN 0` powers off WIO + GPS
entirely.

## Sleep semantics

- WIO "soft sleep": radio to standby + GPS/SD/mesh processing skipped;
  the USART2 link stays alive for the WAKE frame. Not STOP2 - a true
  stop mode would need UART wakeup plumbing in stm32wlxx-hal that 0.6
  lacks. The ESP falls back to a reset pulse (GPIO6) if the wake frame
  is not acked.
- GPS backup: UBX-RXM-PMREQ v0 (16-byte payload) with wakeupSources =
  UARTRX | EXTINT0; wake pulses EXTINT (PB10) high 5 ms plus 0xFF fill
  bytes on the UART. Needs hardware validation - if the module ignores
  the request while still ack-less, consider setting the force flag
  (bit 2).
- ESP deep sleep: sleeps whenever no central is connected; wakes every
  interval, advertises 15 s (double D2 blink), sleeps again if nobody
  connects. 5 s linger after disconnect before sleeping. Intervals
  persist in RTC RAM (survive sleep, not power cycle).
- One interval, `CFG_ESP_SLEEP_S` (0x13), clamped 5 s..5 min. There used
  to be a second id (0x14, "stow") for a 15 min..24 h storage sleep that
  armed immediately and disarmed itself on the next connect. It was the
  same deep-sleep path with a different clamp, and the auto-disarm cost
  far more than it bought: the app could not auto-connect to a stowed
  board without destroying the setting, which forced a whole apparatus of
  persisted state and overrides on top (see the gps-gui-rs notes).
  Dropping it left `next_sleep_interval_s` a field read.
- **The wake window is a deadline, not a per-attempt timeout.** It used to be
  `with_timeout(15 s, advertiser.accept())` inside the advertise loop, and
  every error path (`advertise` failed, `accept` errored, attribute server
  would not attach) did a bare `continue` back to the top - which built a
  fresh 15 s timeout. A central that repeatedly started and failed a
  connection therefore reset the budget indefinitely: observed on hardware as
  the board waking to its full 46 mA and never sleeping again. The budget is
  now an `Instant` deadline checked at the top of the loop, so no retry can
  extend it. The `accept` error path also got a 200 ms pause; without it a
  repeated failure spins hot.
- **The post-disconnect linger advertises rather than idles.** It was
  `Timer::after(5 s)` then sleep, but the board is not discoverable during
  that - the advertiser was consumed by `accept()` - so the stated purpose
  (let the phone come straight back) could not happen. It now sets the wake
  deadline to `now + SLEEP_LINGER_S` and loops, so the linger is 5 s of real
  advertising.
- **Deep sleep has no wake source but the timer.** `enter_deep_sleep`
  registers only `TimerWakeupSource`; there is no EXT1/LP-GPIO wake and
  no button, and the radio is off, so nothing over the air can interrupt
  a sleep. That is why the clamp ceiling matters so much - it is the
  worst-case time the board is unreachable. 5 min keeps that a wait
  rather than a lockout. A reset does cut it short, but a cold boot
  restores the interval from flash and the first advertising window is
  still the usual 15 s, so it only buys a window you chose the timing of.
  Adding an LP-GPIO wake button would be the real fix and is not done.
- The rail is off for the whole sleep and through the wake-check window.
  `enter_deep_sleep` drives GPIO2 low *before* the pad hold; boot brings
  it up only on a cold boot (`!woke_from_sleep`), and `serve_task` raises
  it when a central actually connects. Consequences: a wake nobody
  answers never powers the WIO/GPS, and a connecting app pays a WIO boot
  plus GPS cold TTFF.
- `drive_pwr` (pin only) is split from `set_pwr_en` (pin + persist) so
  the firmware's own dark states cannot overwrite the rail setting the
  app last asked for; `pwr_configured_on()` reads that intent back.
- GPIO6 (WIO RST) is pad-held *released*, not asserted, during sleep.
  Open-drain low into an unpowered WIO would sink through any always-on
  pull-up; the dead rail already holds the WIO in reset.
- `heartbeat_task` skips while `RAIL_ON` is false - pinging an unpowered
  WIO can only time out, and logging that as "link down" is misleading.
- Settings read-back is its own characteristic, not an ack. The obvious
  route - a query config id answered through the ack characteristic -
  cannot work: `packet::ACK_MAX_LEN` is 6, so an ack carries at most 4
  value bytes, and the blob is 12. Widening `ACK_MAX_LEN` would have
  touched gps-proto, which the C3 beacon also uses. Hence
  `ble::SETTINGS_UUID` (read + notify) carrying `ble::Settings`.
- `Settings::decode` gates on a version byte and tolerates trailing
  bytes, so the layout can grow without breaking an older app; the value
  is seeded in `main` as well as published in `gatt_session`, or a
  central reading straight after discovery could beat the first publish.
- Settings are mirrored to flash so they survive a flat cell, not just
  deep sleep. RTC RAM stays the authority at runtime and caches flash:
  only a boot that finds an invalid RTC magic reads flash, so the wake
  check never touches it. Saves happen on the two points that change a
  persisted value (`CFG_PWR_EN`, `CFG_ESP_SLEEP_S`), which are all
  app-driven and rare.
- The store claims the `nvs` data partition but is NOT ESP-IDF NVS
  format - one fixed 20-byte record (magic/version/sleep/flags/crc32,
  all u32 so the length is a flash-word multiple) at the partition start.
  Nothing else on the board reads that region. `entry.offset()` doubles
  as the "found" flag because no partition can live at flash offset 0.
- `esp-storage` must stay at **0.8.1**: 0.9.0 requires esp-hal
  `~1.1.0-rc.0`, which collides with the esp-hal ~1.0 pin the whole
  esp-rtos/esp-radio set depends on. 0.7.0 would also work (no esp-hal
  dependency at all) if 0.8.1 ever becomes a problem.
- Uses the `NorFlash` (erase + write) impl, not `Storage::write`: the
  latter does read-modify-write through a 4096-byte `FlashSectorBuffer`
  on the *stack*, which is far too much for an embassy task. The
  partition-table buffer (3 KiB) is heap-scoped in `main` for the same
  reason and dropped straight after.
- Risk, unvalidated on hardware: a sector erase is ~20-40 ms with the
  cache disabled, and `nvs_save` can run with a BLE central connected.
  That is well inside a normal supervision timeout (720 ms+) so the link
  should ride it out, but if config writes turn out to drop connections,
  move the save to just before `enter_deep_sleep` instead of doing it in
  the handler.
- 12 h+ sleeps are safe on the LP_TIMER: esp-hal computes ticks in u64
  into a 48-bit alarm (`sleep/esp32c6.rs`), and 12 h at the ~136 kHz slow
  clock is ~5.9e9 ticks against a 2.8e14 limit, so its "maybe add check
  to prevent overflow" TODO does not bite here. The slow clock is the
  uncalibrated RC oscillator though, so multi-hour wakes drift by tens of
  minutes.

## Power budget

Measured at the battery with a multimeter in series, whole board:

| State | Measured | Datasheet estimate had been |
|-|-|-|
| Deep sleep, rail off | **0.11 mA** | ~10 uA |
| Awake advertising, rail off | **46 mA** | ~35 mA |

The sleep figure is ~11x the datasheet estimate, which is the caveat
below coming true: the C6 itself is a small part of what the board draws
asleep. Whatever the ESP's supply regulator and any sense divider are,
they dominate. Chasing firmware sleep current below this is pointless -
the win is in the hardware or in the advertising window.

One wake cycle is ~16.5 s awake (~1.5 s boot including BLE stack init,
then the 15 s window), so the period is `interval + 16.5 s`. Deep sleep
always pays the full boot, so that is a floor per cycle.

Average by interval, from the measured numbers, on a 2000 mAh cell:

| Interval | Awake | Average | Life |
|-|-|-|-|
| 15 s | 52% | ~24 mA | ~3.5 days |
| 90 s | 15% | ~7.2 mA | ~12 days |
| 300 s (0x13 max) | 5.2% | ~2.5 mA | ~33 days |

The 15 s advertising window dominates at every interval - even at the
300 s ceiling it is essentially the whole budget, since 16.5 s at 46 mA
swamps 300 s at 0.11 mA. Cutting the window to 3 s takes the 300 s case
to ~0.79 mA, about **105 days**: a 3x win from one constant, and far more
leverage than anything available in the sleep state. The window is a
latency/battery trade, so it belongs in the config protocol rather than
as the fixed constant it is now.

Note the 5 min clamp puts a floor under this: the old multi-hour stow
reached self-discharge-limited draw (years on a cell) and is gone
deliberately. ~33 days is now the best case. That is the price of never
being locked out of the board, and shortening the advertising window is
how to buy some of it back.

Caveat on the hardware side: the schematic is not in this repo and the
README's "Charging IC" section is empty, so the ESP's own supply
regulator is unknown - and the measured 0.11 mA says it matters. A second
AP2112K (55 uA quiescent) or a 2x100k sense divider (16 uA) would each be
a large share of it. Worth identifying before any further low-power work.

Caveat on the measurement: a multimeter in series has burden voltage,
which at 46 mA can be hundreds of mV. That is a plausible cause of
connection failures seen only while metering or only on battery, so
confirm any connection problem with the meter out of the circuit.

## WIO status lines -> ESP console + BLE

- `msg::LOG` (0x44), ASCII up to `link::LOG_MAX` (128; was 64, which cut the
  verbose radio-drop breakdown mid-word at "malform"). Three buffers follow
  the constant - the WIO's `StatusWriter`, `LOG_CHANNEL`, and the BLE
  characteristic `Vec` (the `const _: () = assert!` in esp/main.rs is there
  because the `#[characteristic]` macro needs a literal). Well under the
  256-byte `MAX_PAYLOAD`, and both directions truncate rather than reject, so
  a WIO and an ESP on different values still interoperate. WIO emits via the
  `status_println!` macro (RTT + `EspLink::send_status`) only on
  transitions/events (boot, fix gained/lost, sleep, config, fw begin) -
  never per loop iteration, since the WIO `send` is blocking and would
  stall the 1 ms RTIC loop / starve the watchdog.
- ESP prints `wio: <text>` and pushes into `LOG_CHANNEL`; a third
  `select3` arm in `gatt_session` notifies the log characteristic. Notify
  errors there are non-fatal (an unsubscribed central must not tear down
  the session). Backlog is drained on connect so only live events show.

## Firmware upload over the ESP USB

- USB Serial/JTAG is shared with the esp-println console. The esp-hal
  `UsbSerialJtag` driver handles RX (host frames) and TX (reply frames)
  while esp-println keeps writing console text to the same port.
- Not corrupted because `write_async` pokes all bytes of a small (<64 B)
  frame synchronously before its first `.await`, and esp-println writes
  are synchronous too - on the cooperative single-thread executor neither
  interleaves mid-frame. The host parser resyncs past console text by sync
  byte + CRC regardless.
- Host <-> ESP uses the link framing with `link::usb` command ids; the
  host wraps the exact BLE bulk ops so the ESP runs them through the same
  `handle_bulk` -> WIO path with no new transfer logic.
- Concurrency: `wio_request` now holds an async `LINK_LOCK` for the whole
  request/response (`ACK_SIGNAL` is one `Signal`, so a USB transfer and a
  BLE config write would otherwise steal each other's ACK).
  `FW_XFER_ACTIVE` (AtomicBool) rejects a second `OP_BEGIN` while a
  transfer owns the link so the FW_DATA seq stream can't interleave;
  cleared on end/abort/error and BLE disconnect, and the USB task
  aborts + clears it after 5 s idle so a vanished host can't wedge it.
- `tools/wio_fw_upload.py` (pixi + pyserial) auto-detects the ESP by USB
  VID 0x303A; its crc8/crc32/framing were checked against the Rust
  `crc32_known_value` + frame roundtrip behavior.

## ESP-link RX must be interrupt-driven (WIO side)

- Symptom: WIO->ESP worked perfectly (positions/status/logs) but ESP->WIO
  frames larger than a few bytes were dropped - `fw-upload` FW_BEGIN
  (15 B) always came back status 0x11 (ACK_WIO_TIMEOUT), while the 5 B
  heartbeat PING got through. Root cause: `EspLink::poll` read USART2 by
  polling, once per main-loop iteration. At 115200 the 8-byte hardware RX
  FIFO fills in ~0.7 ms, but one loop iteration is >=1 ms (the `Mono::delay`
  tick plus GPS/SD work), so any frame bigger than the FIFO overran and the
  parser never completed it. Small frames fit the FIFO and survived - hence
  the misleading "PING works, FW_BEGIN times out".
- Fix: USART2 RX is now interrupt-driven. `EspLink::new` sets `cr1.rxneie`;
  an RTIC `#[task(binds = USART2, priority = 2)]` (`esp_rx`) drains the FIFO
  into a `heapless::spsc` ring buffer (producer in the ISR, consumer in
  `EspLink`); `poll` reads the ring buffer. Priority 2 preempts the
  priority-1 run task so the FIFO is emptied within a byte time. The queue
  producer is an `#[init(local = ...)]` `'static` split.
- The ISR runs from flash, so a flash erase/program (FW_DATA) would stall
  it - but the transfer is stop-and-wait (ESP waits for each ACK before the
  next frame), so no bytes arrive during an erase. GPS (USART1, 9600) still
  polls: its 8-byte FIFO buffers ~8 ms, longer than a normal loop pass, so
  it only drops the odd sentence during long SD writes - acceptable.

## FW_END ack truncated by the reset (flush_tx)

- After a verified image the WIO acks FW_END then immediately
  `SCB::sys_reset()`s into the swap bootloader. The HAL's UART `flush` only
  waits on the BUSY flag (RX-line activity), NOT the TC (transmission
  complete) flag, so the reset fired while the last ack bytes were still in
  the TX shift register - the ESP saw a truncated frame, `wio_request`
  timed out (5 s), and `fw-upload` reported "end/verify failed status 0x11"
  even though the swap was actually committed. Fix: `EspLink::flush_tx`
  spins on USART2 `isr.tc` and is called between `send_ack` and the reset.
- Retry-safety: the ESP OP_END handler now inspects the transfer state
  without consuming it and only finalizes (drops state, clears
  FW_XFER_ACTIVE) once the WIO confirms; a WIO timeout keeps the transfer
  open so a retried OP_END reaches the WIO again. `fw-upload`'s `send_end`
  retries OP_END on a dropped ack and on status 0x11, and treats a 0x12
  (no active transfer) on a *retry* as "already finalized" (the prior ack
  was lost but the swap committed).
- `fw-upload` resilience knobs: `ATTEMPTS = 10`; `read_frame` now returns a
  diagnostic string (bytes read, other frame ids, a console-text sample) so
  a retry says *why* ("no ack from ESP (0 B in 3s)" vs "... console 'wio:
  ...'") instead of a bare timeout.

## GPS presence detection

- The MAX-M10 has no separate presence/health line; it just streams NMEA
  at 9600. `Gps` counts `rx_bytes` (any USART1 traffic) and `rx_sentences`
  (valid parsed NMEA); `present()` = `rx_sentences > 0`. The run loop
  announces the first sentence (`gps: NMEA up`), warns once after a 5 s
  grace period if still silent - `gps: silent on USART1 (power/wiring?)`
  when `rx_bytes == 0`, or `... no NMEA (baud?)` when bytes arrived but
  none parsed - and prints a 5 s RTT aliveness line (`bytes/nmea/fix/sats`).
  The `debug` feature additionally logs every raw NMEA line. Note the GPS
  rail is powered by the ESP LDO (GPIO2); a silent module often means that
  rail is off, not a dead module.

## ESP heartbeat + verbose flag

- `heartbeat_task` pings the WIO (`cmd::PING`) every 3 s via the existing
  `wio_request` (500 ms ack wait) so a dead/crashed link shows on the
  console (`wio link up` / `wio link down`) instead of silence. Up/down
  transitions always log; per-ping detail only under `verbose`. Skipped
  while `FW_XFER_ACTIVE` (a bulk transfer already proves the link). It
  reuses `wio_request`, so `LINK_LOCK` keeps it from stealing a BLE/USB
  transfer's ACK. Deep sleep = full reset, so the task re-spawns and the
  `announced` transition state re-logs on the next boot.
- `verbose` cargo feature (esp) gates a `vprintln!` macro (mirrors the
  WIO's `debug`/`debug_println!`): logs every inbound WIO frame
  (`handle_link_frame`) and each heartbeat ping. Uses `cfg!(...)` (runtime
  const, dead-code-eliminated) not `#[cfg]`, so args stay type-checked and
  no unused-var warnings when off. Build: `cargo run --release --features
  verbose`.

## fw-upload host tool

- `tools/wio_fw_upload.py` (pixi task `fw-upload`) now needs no args: it
  builds the image (`cargo objcopy` in wio/), auto-detects the ESP by USB
  VID, and uploads. `--no-build` skips the rebuild; an explicit `--file`
  is used as-is (never rebuilt). Default image path is resolved relative
  to the script, not the CWD, so it works from anywhere.
- `cargo objcopy` needs `llvm-objcopy`, which lives in the `llvm-tools`
  rustup component - the wio nightly toolchain lacked it ("Could not find
  tool: objcopy"). Added `llvm-tools` to `wio/rust-toolchain.toml`
  components so rustup auto-installs it. The built `.bin` is gitignored.
- Resilience: `bulk_op_retry` retries each op (begin/data/end) on a
  transport hiccup (lost/garbled ack). Safe because the ESP and WIO both
  de-duplicate by seq (a duplicate is re-acked, not re-applied), so a
  resend either re-acks or applies - the transfer resumes exactly where it
  stalled rather than dying on the first dropped frame. A real NAK status
  is still fatal; on final give-up it aborts the transfer and exits cleanly
  (no traceback). The firmware side needed no change - it already deduped.

## Untested on hardware

Bench-validated since: the UART link end-to-end and the USB
firmware-upload path, including the esp-println/HAL TX coexistence that
this list called out (see the USB FIFO note below).

Still unvalidated: PMREQ behavior, pad-hold levels through deep sleep, SD
FAT on real cards, and the full BLE bulk firmware path (the USB route is
proven, the BLE one is not). gps-gui-rs should discover the C6 unchanged
(same service UUID; it filters scans by UUID, name is only a fallback).

## GPS config via [gps] section (RADIO.TOML)

- Rode the existing config-transfer pipeline: RADIO.TOML is opaque bytes to
  the ESP/host tool (KIND_TOML), only the WIO parses it, so a new [gps]
  section needs zero ESP/tools changes. Parsed in proto radiocfg into a
  GpsConfig on RadioConfig; applied in wio gps.rs::configure().
- MAX-M10 (proto 34.x) is VALSET-only (legacy UBX-CFG-GNSS dropped). One
  UBX-CFG-VALSET (class 0x06 id 0x8A) carries all keys. Value width is
  encoded in the key id: 0x10.. = L/1B, 0x20.. = U1/E1/1B, 0x30.. = U2/2B.
  Keys used: CFG-SIGNAL-{GPS 0x1031001f, SBAS ..20, GAL ..21, BDS ..22,
  QZSS ..24, GLO ..25}_ENA; CFG-PM-OPERATEMODE 0x20d00001 (0 full/1 PSMOO/
  2 PSMCT); CFG-RATE-MEAS 0x30210001 (u16 ms); CFG-NAVSPG-DYNMODEL
  0x20110021.
- Wrote RAM layer only (layers=0x01). The WIO owns the GPS power rail, so
  the module resets whenever the WIO boots and configure() re-runs from
  RADIO.TOML then; RAM avoids wearing battery-backed storage. Not yet
  bench-validated - confirm constellation change via NMEA talker IDs
  ($GLGSV/$GNGSV) under the ESP verbose feature.

## fw-upload stalls: console vs ack frames on the USB FIFO

- Symptom: `fw-upload` retried ~10% of chunks with "no ack from ESP (26 B
  in 3s console 'wio frame: cmd=0x81 len=3')". The transfer always
  completed (the retry worked) but took far longer than it should.
- The diagnostic string was the proof. The 26 bytes are *exactly* that one
  console line and nothing else, and the longer samples carried recognizable
  wreckage of the ack frame: seq 87 left `X` = 0x58 = next_seq 88, seq 107
  left `l` = 0x6C = 108, seq 9 left `R` = 0x52 (BULK_ACK cmd) and ` ` = 0x20
  (ACK_ID_BULK). So the ESP built and sent a correct ack every time; only
  1-9 bytes of the 11-byte frame survived. Not a late ack - a shredded one,
  which is why raising the host timeout would have been useless.
- Root cause: esp-println and esp-hal's `UsbSerialJtagTx` both write the
  single 64-byte USB Serial/JTAG IN FIFO with nothing arbitrating between
  them, and `write_async` stages bytes *without* checking
  `serial_in_ep_data_free` - it assumes it owns the FIFO. With a console
  packet still draining, the hardware silently drops whatever no longer
  fits. Under `--features verbose` a console line is emitted per chunk
  (from link_task) immediately before each ack, so the collision window is
  hit on every single chunk.
- Fix, two parts. `send_usb_frame` flushes *before* writing: `flush` is the
  only path that tests `serial_in_ep_data_free`, so the leading flush waits
  for the FIFO to be free. And `console_busy()` (= `FW_XFER_ACTIVE`) gates
  `qprintln!`/`vprintln!` so discretionary console output goes silent for
  the duration of a transfer - that removes the collision instead of racing
  it. WIO status lines still reach the phone; only the console print is
  suppressed (the `LOG_CHANNEL` notify stays outside the gate).
- Also visible in the logs: a phone was connecting/disconnecting throughout
  the upload (`central connected` / `advertising as GPS-C6`), which is why
  the BLE lifecycle prints are gated too. Stalls clustered on those events.
- Bench-validated: `fw-upload` runs clean on hardware after the fix. The
  check was deliberately the harsh case - `--features verbose` built in,
  which is what previously produced a collision on every chunk.
- Generalizes past this bug: any frame written to the USB Serial/JTAG port
  while esp-println can also reach it needs the same treatment. Bytes are
  dropped silently, with no error surfaced by `write_all`, so the failure
  looks like a timeout on the host and not like a TX fault on the device.
  That mis-signal is what makes it worth remembering - the instinct is to
  raise the timeout, which cannot help when the frame was destroyed rather
  than delayed.

## Configurable advertising window (BLE 0x14)

- The window, not the interval, is what the sleep-mode duty cycle turns
  on. Advertising is ~35 mA against a deep sleep in the tens of uA, so the
  average draw is close to `window/interval` times the advertising figure
  and barely sensitive to anything else. Shortening the window is also the
  cheaper of the two knobs for the user: it does not change how often the
  board is reachable, only how much margin each opportunity carries.
- Clamped 3..60 s, default 15 s (the previous hardcoded value, so an
  unconfigured board behaves exactly as before). Unlike the sleep interval,
  0 is *not* an "off" here - a zero window would leave a sleeping board
  unreachable by anything short of a physical reset, since deep sleep has
  no wake source but the timer. It clamps up to the floor instead.
- Stored 0 means "never configured" and resolves to the default through an
  accessor rather than being written out at first boot. That keeps the
  cold-boot path free of a flash write and means the default can move
  later without stale records pinning old boards to 15 s.
- `serve_task` samples the window once per wake, not per loop iteration.
  Reading it inside the loop would let a window shortened over BLE apply
  to a wake already in progress and strand the board with its budget
  retroactively spent. Same reasoning as the existing deadline-not-timeout
  choice, one level up.
- The flash record went to version 3, but `nvs_decode` still accepts
  version 2 rather than discarding it. The change is a pure append, so the
  only difference is where the crc32 sits (offset 16 for v2, 20 for v3) -
  a few lines to branch on, against the alternative of every deployed
  board losing its sleep interval on the update and coming back
  advertising continuously until someone reconnected to it. Worth it for
  an append; a layout change that actually moved fields would not be.

## RxBoost (radio.toml rx_boost)

- The SX126x RxGain register (0x08AC) was never written, so the radio ran
  at its power-up default of power-saving gain. `stm32wlxx-hal` already
  exposes `SubGhz::set_rx_gain(PMode)`; it was simply unused.
- Exposed as a bool, not the HAL's four-level `PMode`. Only two of the
  four settings are documented (power saving 0x94, boosted 0x96) - the
  intermediate 0x95/0x97 have no specified behavior, so putting them in a
  config file would invite choosing a value nothing describes.
- Defaults to false, matching the chip. It costs receive current
  continuously on whichever node is listening, and this rail is switched
  off during sleep on the ESP side, so it is a trade to opt into rather
  than one to inherit from a firmware update.
- Applied inside `Sx1262Driver::init`, deliberately on every call rather
  than once. RxGain is not covered by SX126x warm-start sleep retention.
  That is currently moot - `Sx1262Driver::sleep()` has no callers, and the
  soft-sleep path uses `standby()` (registers retained) then re-runs
  `init` on wake - but writing it in `init` means the setting survives
  whenever `sleep()` does get wired up, since `init` is already documented
  as mandatory after it.
- Unlike every other radio key, this one does not have to match across
  nodes: it affects only reception on the node that sets it. Range is the
  worse of the two directions, so it helps only where the receiving end is
  the weak one.

## Replacing embedded-nano-mesh with leaf/repeater roles

- Topology never set range. A mesh hop and a repeater hop have the same
  link budget; both are store-and-forward relays. What nano-mesh actually
  cost was air time, and air time is what buys spreading factor.
- Its `Packet` was 40 bytes (`CONTENT_SIZE` fixed at 32, padded, plus 8 of
  fields) and the wire adds `PACKET_START_BYTES_COUNT` = 3, so a 21-byte
  position went out as 43 bytes. The new frame is 3 bytes of header over a
  true-length payload: 24 bytes for the same position.
- The padding is why the "shrink the position payload" idea was dead
  weight before this. Delta-encoding a position down to 10 bytes still
  transmitted 43. It pays now.
- The real blocker was `listen_period`. nano-mesh assumes a byte-stream
  transport (UART, nRF24) where packets are sub-millisecond. At SF12 one
  packet is ~2.1 s, so `listen_ms` had to exceed that, and the beacon path
  held RADIO_BUSY for `2 * listen_ms + 500` - about 4.8 s against a 10 s
  interval. The slow presets where the range actually is were unreachable.
  `listen_ms` is gone; the busy window is now `tx_poll_timeout_ms()`,
  which is derived from `airtime_scale()`.
- Frame is `[src, id, hops_left]`. No checksum: the SX126x already
  transmits with its hardware CRC on.
- That CRC was not actually being enforced. `set_irq_cfg` never enabled
  `Irq::Err`, and the SX126x raises RxDone *alongside* Err on a bad CRC
  with the corrupt payload still in the buffer. nano-mesh's own checksum
  had been covering for this; removing it would have started handing
  corrupt frames up, so `poll_recv` now checks Err and drops.
- Repeats are queued with a random delay, not sent from inside the receive
  path. Two repeaters hearing the same broadcast would otherwise transmit
  at the same instant and collide every time. The delay scales with
  `airtime_scale()` rather than being a fixed millisecond count, since
  "long enough for one to win" means one packet time, which varies 32x
  across the SF range.
- Dedup on `(src, id)`, 16 slots, 60 s TTL. TTL has to stay well under the
  time it takes an id to wrap (256 broadcasts) or a node's own sequence
  would collide with its remembered history.
- A node drops frames whose `src` is its own address - otherwise a
  repeater's forward of our beacon comes back and gets reported as a
  remote node sitting exactly where we are.
- `max_hops` is a property of the sender, stamped into the frame, not a
  setting on the repeaters. Dropping a repeater into a deployed fleet
  needs no reconfiguration of the nodes already out there.
- Old `lifetime` is still parsed, mapped as `max_hops = lifetime - 1`: it
  counted transmissions where max_hops counts retransmissions. `listen_ms`
  is simply ignored, since unknown keys already parse. Cards written for
  the old firmware keep working.
- `[mesh]` renamed to `[network]` in the examples only. Section headers
  are cosmetic in this parser, so old files are unaffected.
- Costs 3.5 KB less flash and 672 bytes less RAM than the nano-mesh build,
  and drops two dependencies (embedded-nano-mesh, embedded-io - the latter
  only existed to bridge a packet radio to the byte-stream API nano-mesh
  wanted, which is what `wio/src/io.rs` was).

## GPS fix-derived fields are cleared on fix loss

`PositionPacket` fields written from optional NMEA fields kept their last
values forever once the fix dropped. The module stops carrying them (GGA
sends an empty altitude, RMC empty speed/course), so `if let Some(v)` simply
never fired again and the packet still read as a valid altitude from
whenever the fix was last good.

- Everything that gates on `FLAG_FIX` was already correct; the stale values
  only leaked through paths that do not - the BLE position notify and the
  console dump both publish the packet unconditionally.
- Cleared together (lat, lon, alt, speed, course, sats) rather than altitude
  alone. Zeroing altitude while leaving a stale lat/lon is a worse state to
  read than either all-stale or all-zero.
- `tod_ms` deliberately survives. The receiver keeps decoding time from the
  satellites it tracks, so RMC carries a valid time across a fix loss; it is
  not fix-derived and clearing it would throw away good data.
- `sleep()` clears them too - backup mode is a fix loss like any other.

## SD config file renamed to RADIO.CFG

`RADIO.TOML` could never be opened. FAT short names allow a three-character
extension and `TOML` is four, so `ShortFileName::create_from_str` returned
`NameTooLong` and the name was rejected before any directory lookup ran.

- `read_config` therefore always returned `None`: every boot fell through to
  "no SD config, using defaults". SD-loaded radio config had never worked.
- `write_config` always failed, and its failure path calls `unmount()` - so
  pushing a config over BLE also tore down the mount and stopped position
  logging until the next retry.
- Creating the file on a PC does not help: FAT stores `RADIO.TOML` as a long
  name with a short alias like `RADIO~1.TOM`, which no 8.3 lookup finds.
- The repo file stays `RADIO.example.toml` so editors still highlight it as
  TOML; only the name on the card changed.

## Beacon payload is a field mask, not a fixed packet

`fields` in the config picks what each broadcast carries; the mask is a byte
in the frame rather than firmware-wide knowledge.

- Self-describing beats agreed-in-advance here. Nodes are configured one at
  a time from separate cards, so any layout both ends must agree on is a
  layout that breaks the first time one card differs.
- Default `lat,lon` is 13 bytes on air against 24 for the old fixed packet.
  The saving multiplies with spreading factor - at SF12 a symbol costs ~32x
  what it does at SF7 - which is what makes the slow presets reachable.
- Tag moved 0x50 -> 0x51. Old firmware rejects the new tag and new firmware
  rejects the old, so a mixed fleet drops frames instead of reading a mask
  byte out of a latitude.
- FLAG_FIX is reconstructed on decode rather than transmitted: a node only
  beacons while it has a fix, so the frame's existence carries the bit.

## SX1262 feature list, checked against STM32WLE5 silicon

Only three of the advertised SX1262 features were missing config surface;
the rest either already ran, do not exist on this chip, or are not settings.

- SetDio3AsTcxoCtrl and SetRegulatorMode were already in `radio.rs`, just
  hardcoded. The STM32WL calls the first `SetTcxoMode` - same opcode 0x97 -
  because the radio is on-die and there is no external DIO3 to name.
- SetDio2AsRfSwitchCtrl (SX1262 opcode 0x9D) is absent from the STM32WL
  opcode table entirely. The die has no bonded DIO2; RF switching here is an
  MCU GPIO job, so it can never become a config key.
- FIFO watermark streaming past 255 bytes does not exist on SX126x. The IRQ
  set has no FIFO threshold line - that is an SX127x feature. The 256-byte
  buffer is a hard ceiling.
- LR-FHSS is not in the STM32WL packet types (LoRa, (G)FSK, (G)MSK, BPSK).
- SetRxDutyCycle (0x94) and SetCadParams (0x88) are both real and exposed by
  the HAL. They are worth having, but they change how the receive loop
  behaves rather than adding a knob, so they are features, not keys.
- AES/PKA/TRNG are real STM32WLE5 peripherals but nothing in this project
  encrypts anything, so a key for them would configure nothing.

## tx_only / rx_only roles

Added as `Role` variants rather than a separate `traffic = ...` key, so the
contradictory combinations (`repeater` that never receives, `rx_only` that
forwards) are unrepresentable instead of validated.

- The RX gate had to go in `radio.rs`, not `node.rs`. `send()` re-armed
  continuous RX unconditionally after every TxDone, so gating `Node::poll`
  alone would have left a tx_only node listening from its first beacon
  onward - i.e. the mode would have saved nothing, which is its only point.
  The driver now carries a `listen` flag set from `cfg.role.receives()` in
  `init()`, and drops to standby after TX when it is false.
- The beacon is gated in `main.rs` on `cfg.role.transmits()` rather than
  inside `Node::broadcast`, so an rx_only node does not send the ESP a
  RADIO_BUSY for a transmission it was never going to make. `broadcast`
  still refuses with `TxError::Muted` so a future caller is told, not
  silently ignored.
- gps-gui-rs needed no change: it builds the dropdown from the
  `role_type = "enum:..."` string in the file itself.

The self-echo drop in `Node::poll` had to become conditional on
`role.transmits()`. A node that never transmits cannot hear itself, so
`src == self.address` on an rx_only node is always a genuine remote frame -
dropping it made a base station blind to whichever tracker shared its
address, which with both left at the default is the likely first setup
anyone tries.

## wio_config.py

`pixi run wio-config --address 3` pushes a config over the same USB bulk
path the firmware upload uses (`kind` 1 = TOML vs 2 = firmware), applied
live and saved to the SD card.

- It sends a whole file, never a patch, because `radiocfg::parse` starts
  from `RadioConfig::default()` - a key absent from what is pushed reverts
  to its default rather than keeping the board's value. There is no
  read-back path for the TOML, so the source file is the whole truth about
  the result. Default source is RADIO.example.toml, which a proto test pins
  to the firmware defaults; `--file` starts from tuned settings instead.
- Key matching is on the text before `=`, so `address` does not also rewrite
  the `address_description` line next to it. The quoting of the replaced
  value is preserved so a string stays valid TOML for the GUI.
- A misspelled key cannot be detected on the board (unknown keys are ignored
  by design), so the tool warns when a key was not already in the reference
  file - the only signal available.
- Config is parsed at CFG_END, so a bad value NAKs at the end of the
  transfer and looks like a link fault; the tool spells that out.
- Transport moved to `wio_link.py` shared with the firmware uploader.

## Config size ceiling: 1024 bytes

RADIO.example.toml was 6150 bytes against a 1024-byte limit enforced in three
independent places - the ESP's OP_BEGIN check, wio/src/cfgxfer.rs CONFIG_MAX,
and the buffer the WIO reads RADIO.CFG into at boot. So the file could not be
pushed *or* used on a card, despite its header saying to copy it there: the
SD read truncated it mid-line and the parse failed back to defaults.

Descriptions and comments are ~92% of it. wio_config.py strips `#` lines and
the `_description`/`_type` keys before sending, which lands at ~500 bytes.
The documented file stays the human/GUI reference; `--dry-run --save` writes
a card-ready RADIO.CFG from it.

Raising CONFIG_MAX was the alternative and was rejected: it costs several KB
of RAM on the WIO to carry text the firmware already ignores.

## ESP console verbosity

`verbose` in RADIO.CFG, default true, with the cargo feature now default-on
too. The two gates mean different things: the feature decides whether the
call sites are compiled in (a code-size choice), the config key switches them
at runtime on a deployed board.

The ESP never parses the config - it streams the bytes through to the WIO
unread - so the setting travels back as TELEM_FLAG_VERBOSE in the periodic
telemetry the WIO already sends. Riding an existing periodic message rather
than adding a command means a lost bit self-corrects on the next heartbeat
and neither side tracks delivery. The ESP's static starts true so the window
before the first heartbeat, when a board that fails to come up most needs to
be talking, is verbose.

## Config persistence without an SD card

The SD card was the only store, so a board with no card (or a failed one)
came back from every power cycle on firmware defaults - losing its address,
the one setting that cannot be guessed back.

Worse, the failure was invisible. `write_config` failing only logged over
RTT, while the CFG_END ack went out unconditionally, so a push reported
success and the board worked perfectly until the next reboot silently
restored the defaults.

- `wio/src/cfgstore.rs` adds a backup copy in flash page 122 (0x0803_D000).
  Pages 122-127 were already excluded from the app's linker region and
  unused by the bootloader, whose map ends at the boot state on page 121.
- Boot order is SD, then flash, then defaults. The card wins deliberately:
  pulling it to edit RADIO.CFG on a computer has to do what it looks like.
- Text is programmed before the header, so a power loss mid-write leaves an
  erased page that reads as "nothing stored" rather than a valid-looking
  header over half-written text. A CRC-32 catches the rest.
- An unchanged config is not rewritten - the page is erased in full for each
  write, and the endurance budget belongs to real changes.
- The apply path now reports which stores it reached ("saved to SD and
  flash" / "NOT SAVED - lost on reboot") over the link rather than RTT, and
  wio-config exits 2 on the last of those.

Flash programming stalls instruction fetch for ~25 ms per page erase, which
would cost UART bytes - but this write happens at CFG_END with the host
waiting on the ack, so nothing is in flight. App flash use is 73 KB of 112.

Note: sd_enabled is still only honored at boot, so a runtime push that sets
it false still writes that config to the card.

## BLE address: eFuse-derived per board, env override, USB read-back

The ESP BLE address is no longer a hardcoded array. Default: ble_address()
in main.rs derives it from the chip's factory MAC (Efuse::
read_base_mac_address, MSB-first), reversing to the LSB-first array
Address::random wants and setting 0xC0 on the MSB (static random). Every
board is then unique with no config, which is what avoids collisions across
multiple boards - a single pinned address would have handed them all the
same one.

Override: build.rs validates an optional BLE_ADDRESS env var (MSB-first
"FF:C6:A1:53:50:47", 6 octets, first & 0xC0 == 0xC0) and passes it through
cargo:rustc-env; main.rs reads it with option_env!() (None -> eFuse path).
build.rs emits nothing when unset. rerun-if-env-changed makes it rebuild.

Read-back: only printing at boot ("BLE-ADDR <addr>") is fragile (the C6's
native USB Serial/JTAG does not reset on DTR/RTS, so you cannot reliably
retrigger the line). Instead the firmware answers a USB info query
(link::usb::INFO 0x53) any time with [INFO, addr MSB-first]; the address is
published to a static at boot before usb_task can be asked. pixi run
esp-address reads it on demand.

Loader: tools/esp_upload.py (pixi run esp-upload) just sets BLE_ADDRESS for
--ble-address and runs cargo run, inheriting stdio so espflash's interactive
monitor renders cleanly (piping it to parse output garbled the monitor). No
pin file - the eFuse default makes per-board uniqueness automatic.

tools/gen_ble_address.py (pixi run gen-ble-address) generates a random one.
It forces the two MSBs to 1 (static random) and by default also sets the
locally-administered (0x02) and clears the multicast (0x01) bit of the MSB.
Those two are IEEE-802 MAC bits, not required by BLE for a random address,
but harmless and keep it out of real OUI space / from looking multicast if a
host shows the address as a MAC. --no-local skips them.

## Reading the WIO radio config back over BLE

The radio config only ever travelled to the board; nothing could read it
back. Added a read-back that mirrors the existing power/sleep Settings
characteristic:

- A versioned 28-byte RadioConfig snapshot (radiocfg encode/decode) rather
  than raw TOML text. Chosen over text because it needs no chunked transport
  (fits one link frame and one BLE read), reflects the live effective config
  even on a board with no stored file, and the app already depends on
  midair-proto so decode is shared - no per-consumer schema copy.
- The WIO owns the parse, so it encodes its live cfg and pushes msg::CONFIG
  to the ESP at boot, after every apply, and on cmd::CFG_READ. The ESP never
  parses config, so it just caches the bytes and relays them on the
  characteristic. cmd::CFG_READ is fire-and-forget (no ACK) because the
  config blob IS the reply; sending it through the wio_request ACK path would
  hang waiting for an ACK that never comes.
- ESP requests CFG_READ on connect and on link-up so the cache fills even
  when the ESP restarted under an already-running WIO. Characteristic stays
  all-zero (version 0 -> decodes to None) until the WIO reports one; the app
  treats all-zero as "not yet", a non-zero undecodable blob as a version
  mismatch.
- GUI bridges the binary blob back into its TOML editor by overlaying the
  decoded values onto the doc (RadioDoc::apply_config), keyed by the editor's
  own field list, so comments/descriptions/dropdowns survive and re-pushing
  sends the same values back. Enum/mask fields round-trip through as_str
  helpers the parser accepts.

## No-fix ping (lora::Ping, tag 0x52)

A node without a fix used to transmit nothing, so "searching", "out of
range" and "dead" all looked the same to a receiver. The beacon slot now
goes out either way: a position when there is a fix, a 4-byte ping when
there is not.

- Payload is `[0x52][flags][uptime_s u16le]`. Flags are GPS_PRESENT (any
  NMEA parsed since boot) and HAD_FIX (a fix held at some point), which is
  what separates a receiver that cannot see the sky from a module that never
  came up, and a fix lost from one never acquired. Uptime saturates at
  0xFFFF (18 h) and also reveals a node that rebooted between pings.
- Deliberately *one* deadline, not two: the ping shares `next_beacon` rather
  than getting an interval of its own. Two deadlines would let a fix that
  comes and goes put a position and a ping on the air inside one interval,
  which breaks the duty-cycle budget the SF9 default was chosen against
  (330 ms position, 248 ms ping, 400 ms per 20 s). The cost is that a fix
  acquired just after a ping waits out the rest of the interval before its
  first position - the old code re-checked every 500 ms, which was free only
  because an unfixed node was silent. A receiver already knows the node is
  alive from the ping, so the trade is worth the bounded air time.
- Ping tag 0x52 is checked before the fall-through to LORA_RX, so a ping is
  not forwarded as an opaque payload as well. Ping::decode requires the tag
  and the full 4 bytes, and unknown flag bits are ignored so a later sender
  can add one.
- Received pings become a status line (`node 3 ping: rssi -97, up 214s, gps
  ok`) rather than a POSITION frame: nothing zeroed out gets logged into
  GPSLOG.CSV, and it reaches the ESP console and the BLE status
  characteristic through the existing msg::LOG path - no new BLE surface and
  no gps-gui-rs change. RSSI is in the line because a range check is the
  main thing a ping is good for. Telemetry already updates rx_count,
  last_rssi and secs_since_rx from node.poll, so the structured side is
  covered without decoding the ping on the ESP.
- Off switches are the ones that already existed: `interval_s = 0` and
  `role = "rx_only"`. No new config key, so nothing had to move into the
  28-byte read-back blob (which has one spare byte left).
- A GPS put into backup mode over BLE (0x12) still pings - it has no fix, so
  the node keeps announcing itself while the receiver is off. That is
  intentional; the interval is the knob if the TX energy matters.

## SubGHz register audit (the range fixes)

A full pass over what `Sx1262Driver::init` writes and what it left alone,
against the SX126x sequence and ST's own STM32WL radio driver.

### The antenna switch was never driven

PA4/PA5 are the module's internal RF switch control lines. Nothing
configured them, so they sat in their reset state as analog inputs and the
switch floated. Transmit energy and received signal reached the antenna only
through the switch's off-state isolation - tens of dB down - which is a radio
that works across a bench and nowhere else.

- The earlier note in this file ("RF switching here is an MCU GPIO job")
  reached the right conclusion and stopped there; the GPIO code was never
  written. A conclusion recorded is not a fix applied.
- Mapping: both low = isolated, control 1 high = RX, control 2 high = TX via
  the high-power PA. The low-power PA output exists (both high) but this
  board never selects it.
- The path is set before each `SetTx` and each `SetRx`, never after. A PA
  ramping into an isolated switch is a transmission that goes nowhere, and
  the ramp starts the instant `SetTx` lands.
- `Speed::Low` on both pins: they are DC logic levels sitting next to an RF
  path, so there is nothing to gain from fast edges.

### Calibrate(0x7F) after SetTcxoMode

The automatic calibration at power-up runs before the TCXO is enabled, so its
RC64k, RC13M, PLL, ADC and image results all came off a clock that was not
running. Nothing reports this - the radio comes up, answers commands, and is
simply less sensitive and off frequency. ST's driver issues the full
calibration immediately after `SetTcxoMode` for exactly this reason.

### RX payload length is a ceiling, not a length

In explicit-header mode the packet params' payload length is the largest
frame the receiver will accept; the header carries the actual length. Every
transmit narrowed it to the frame being sent and nothing put it back.

- Frames here run 7 to 35 bytes, so from the first beacon a node could only
  hear frames no longer than its own last transmission.
- The no-fix ping made it acute: 7 bytes on air is shorter than every
  position frame on the network, so a node that lost its fix went deaf to its
  peers as well - two symptoms of one bug that read as unrelated.
- Semtech's own drivers write 255 on every entry to receive. Entering RX is
  now one function that restores the ceiling, drops to standby to take the
  params (the caller may be re-arming out of continuous RX after an oversize
  drop), and arms.

### Registers reached around the HAL

`stm32wlxx-hal` keeps its register table `pub(crate)` and lists only the
addresses it has methods for, so 0x08D8 and 0x0889 have no route through it.
Rather than fork the crate, `subghz_xfer` runs the same raw SUBGHZSPI
transaction the HAL performs for its own register access (wait out BUSY, NSS
low, shift bytes, NSS high) via the PAC - the same idiom as `clear_errors` in
the GPS driver, and exclusive for the same reason.

- TX clamp 0x08D8 |= 0x1E: PA tolerance of an antenna mismatch. Applied
  after the PA is configured. Relevant here rather than theoretical - the
  antenna is a connector and a short wire.
- TX modulation 0x0889 bit 2: clear at BW500, set otherwise. The reset value
  has it set, so it only matters for a config that selects 500 kHz, which the
  parser accepts. Follows `SetModulationParams`, which is what carries the
  bandwidth.
- Byte-wide volatile accesses to the data register at 0x5801_000C. A 32-bit
  write would shift out four bytes.

### Smaller items from the same pass

- Image calibration picked a band from two thresholds (900 and 860 MHz),
  with everything below falling through to 433. The parser accepts 150-960
  MHz, so 880 calibrated as 863-870 and 490 as 433. Calibrating for a band
  the radio is not in throws away image rejection, which is sensitivity.
- SMPS clock detection has to be enabled *before* the SMPS is selected. It is
  written unconditionally, since it costs nothing on a board running the LDO.
- GetError was never read. It is the only call that separates a radio that is
  idle from one that is idle because its TCXO never started or a calibration,
  PLL lock or PA ramp failed - the status byte reads identically either way.
  Checked and cleared alongside the status line.
- `PMode::Boost` writes 0x97, not the 0x96 an earlier note in this file
  claimed. The Semtech datasheet documents only 0x94 (power saving) and 0x96
  (boosted); the HAL exposes four levels and calls 0x97 the best sensitivity.
  Left as `Boost` - it is what the HAL documents as maximum - but the
  earlier note was wrong about the value being written.
- `rx_boost` has defaulted to true since it was added. The comment on the
  default and the README both still described it as off.

### Checked and correctly left alone

Standby before every config write; `SetPacketType` before the param commands;
PA config 0x04/0x07/HP identical to the HAL's `HP_22` preset; OCP 140 mA set
*after* `SetPaConfig`, which resets it; RxGain rewritten on every `init`
since warm start does not retain it; `0x00FF_FFFF` really is the continuous
RX value; an 8-symbol preamble is right when the receiver never duty-cycles;
`SetLoRaSymbNumTimeout` left at 0 and `SetRxTimeoutStop` untouched for the
same reason; SMPS drive (reset 100 mA) and power control (reset 0x50) left
where ST leaves them; HSE32 trim is meaningless with a TCXO; the FSK-only
registers and the warm-start retention list do not apply.

### GPS, same pass

- The module sends GLL, GSA, GSV and VTG by default and nothing parses any of
  them. 9600 baud is 960 bytes a second, and GSV alone can exceed that in one
  epoch with several constellations enabled - which pushes RMC and GGA behind
  sentences that get discarded, and surfaces as the USART overruns `poll`
  already treats as routine. The same CFG-VALSET now turns the four off.
- CFG-VALSET was fire-and-forget. A rejected frame left the module on its own
  settings while the firmware reported the ones it asked for. Acknowledging
  it exposed a second case: at boot the push often lands before the module
  has finished starting, so it is retried when the first sentence proves the
  module is listening. The ack scan has to tolerate NMEA interleaved with the
  reply, since the module does not stop talking to answer.

### Sync word moved to private

0x1424 instead of the public LoRaWAN 0x3444. Not a link-budget change - it
does not move a single dB - but on the public word the receiver locks onto
every LoRaWAN preamble in earshot, and the time spent failing to decode a
frame that was never ours is time not spent hearing the network. It also
inflates the CRC-error count the status line asks an operator to read as
signal quality. Left hardcoded rather than made a key: nodes on different
sync words cannot hear each other at all, so a config key here is a way to
split a fleet by editing one card. Reflash every node together.

### SMPS clock detection was clobbering the rest of the register

Symptom: the module ran quite warm after the register fixes above went in,
and none of them could account for it - the antenna switch fix should make
transmit *cooler* (a PA ramping into an isolated switch reflects its power
back into itself and dissipates all of it), and at a 20 s beacon the
transmit duty cycle is 1.65% either way. Warmth that constant meant
something drawing tens of mA continuously, which pointed at the only change
touching a permanently powered subsystem.

`SubGhz::set_smps_clock_det_en` looked like a field setter and is not: it
expands to `write_register(SMPSC0, (en as u8) << 6)`, a blind whole-register
write. Enabling clock detection through it therefore zeroed every other bit
of the radio's regulator configuration - on a board where the SMPS supplies
the whole radio. ST's driver read-modify-writes the same register, which is
what 0x0916 gets now via the raw register access added for the errata
workarounds. Confirmed: the read-modify-write took the heat away.

Worth remembering for the rest of this HAL: `set_rx_gain` and `set_pa_ocp`
write whole-byte values and are fine, but any setter that names a single bit
or field (`set_smps_drv`, `set_pwr_ctrl`, `set_smps_clock_det_en`) writes the
whole register underneath.

## Default modulation moved to SF12/BW500

The SF9/BW62.5 default was picked as the longest-range modulation whose
beacon fit a duty-cycle budget. The budget was the wrong rule: 2% duty is an
EU 868 constraint, and the 400 ms figure it was reconciled against is the
FCC *frequency-hopping* dwell limit, which assumes hopping across channels.
On one fixed 915 MHz channel neither applied.

- 15.247 offers two routes in 902-928. Digital modulation needs a 6 dB
  bandwidth of at least 500 kHz and then carries no dwell or duty limit at
  all. Anything narrower has to qualify as frequency hopping: 50+ channels,
  0.4 s per channel per 20 s, and receivers hopping in step.
- The receiver side is what rules hopping out, not the channel count. A hop
  schedule needs a clock the network agrees on; the only one here is GPS
  time; a node that has never had a fix does not have it - and that is
  precisely the node the no-fix ping exists to keep audible. Transmitting
  each frame on all 50 channels instead is 50x the air time.
- SF12/BW500 costs 1.5 dB against SF9/BW62.5 (-131 vs -132.5 dBm, about a
  tenth of the range on real terrain) and is *shorter* on air: 289 ms vs
  330 ms for the default beacon. 2^12/500 kHz and 2^9/62.5 kHz are both
  8.192 ms symbols, and SF12 needs fewer of them per byte.
- `airtime_scale()` is 8 either way, so the TX timeouts and the repeater's
  jitter window did not move.
- The BW500 modulation-quality errata write at 0x0889 stops being dormant.
  It was added blind during the register audit and is now load-bearing.
- Two tests had quietly depended on the default being a fast modulation -
  one bumped SF to 12 expecting a longer air time, one to 10. Both now build
  an explicit fast config rather than deriving from the default, which is
  the slowest spreading factor the parser accepts and has nothing above it.

## Remote reports: a table, pushed on arrival

The C6 kept one 23-byte cache of "the last remote position", written by the
link task and read by the BLE notifier on its own timer. Both halves lost
data. Two nodes reporting between one tick and the next left only whichever
arrived second - the first was never notified at all - and with the notify
interval settable up to 60 s that window is not small. What did go out was
then re-sent every tick forever, so a node that had gone off the air was
indistinguishable from one still reporting.

Now `midair_proto::roster::Roster`: one slot per node (8 of them), newest
report wins within a node, handed out once each. The notifier stopped
sampling it; the link task signals and a session arm drains.

- Per-node table, not a FIFO. A queue is lossless for a burst but lets one
  fast-beaconing node evict everyone else, and re-sends positions a newer
  one has already superseded. Position reports supersede; they do not
  queue.
- A node holds one slot whichever kind of report it sends, so a ping
  replaces that node's position when it loses its fix, rather than the
  stale position outliving it.
- Eviction expires entries past the TTL before it picks a victim, so a full
  table gives up a node long off the air ahead of a live one that happens to
  beacon slowly.
- The logic lives in the shared crate rather than `esp/src/bin/main.rs`
  because a `no_std` binary has nowhere to run tests. It takes `now_ms: u64`
  instead of an `Instant` so nothing pulls embassy-time into a host test.
  proto is edition 2021, so no let-chains in there - the firmware crates are
  2024 and use them freely.

## Age of a remote report, and where it comes from

`age_s` is measured on the receiver's clock from when the report arrived,
not from anything in the report. The sender picks which fields to spend air
time on and `time` is not in `FIELDS_DEFAULT`, so `tod_ms` is usually zero -
a receiver has nothing in a lean beacon to age it by.

Appended to the position layout rather than inserted, and `ble::REMOTE_LEN`
deliberately keeps its old value as the length a reader must *accept*, with
`REMOTE_LEN_V2` as what the board sends. gps-gui-rs takes midair-proto by
path, so bumping `REMOTE_LEN` itself would have made a rebuilt app reject an
un-reflashed board: its `remote_event` checks `len < REMOTE_LEN`. As it
stands both directions work, since `PositionPacket::decode` already tolerates
trailing bytes.

Truncating seconds, not `div_ceil`: rounding up would report every live
notification as 1 s old the moment the drain took a millisecond, and 0
meaning "less than a second" is the conventional reading. This matches
`Telemetry::secs_since_rx`.

## Bug sweep: nine fixes

A pass over the whole system (proto, wio app + bootloader, esp, host tools).
Grouped by what actually went wrong, worst first.

### A long RADIO.CFG was silently adopted as defaults

`SdLog::read_config` filled the caller's buffer and returned its length, which
is indistinguishable from a short file. The TOML parser skips comments and
blanks, so any prefix ending on a line boundary parses clean - it is simply a
file with fewer keys, and every key past the cut comes back as its default.

The reference file is the worst case of that. `RADIO.example.toml` is 7892
bytes and its first 1024 - the ceiling both transfer paths enforce - are all
header comment, so a card holding it parses to *every* setting at its default,
address included, while the board logs "RADIO.CFG loaded (address 1)" and sets
`TELEM_FLAG_CFG_LOADED`. Copying the reference file onto a card, which the
name invites, produced a board that reported a config it was not running.

`read_config` now asks `file_length` first and refuses a file it cannot hold
whole, so boot falls through to the flash backup instead of adopting a phantom.
The size check has to live at the reader: a parse failure cannot be relied on,
which is what `a_truncated_file_parses_as_a_shorter_one` in radiocfg.rs pins.

### The watchdog was shorter than the WIO's own deadlines

`watchdog::start(6_000)` against `tx_poll_timeout_ms = 150 * airtime_scale +
100`, which is a function of the configured modulation:

| Config | poll deadline | beacon + repeat in one pass |
|-|-|-|
| SF12/BW500 (default) | 1300 ms | 2.6 s, safe |
| SF12/BW125 | 4900 ms | 9.8 s, reset |
| SF12/BW62.5 | 9700 ms | 19.4 s, reset |

The beacon and the repeat forward both run between two of the main loop's
feeds, so a stalled transmit at any SF11+/BW125-or-narrower setting reset the
board. LSI tolerance makes it worse: at the 47 kHz end of the STM32WL spec the
real timeout is ~4.1 s, not 6.

Rather than stretch the timeout (which would blunt it everywhere), the bounded
wait loops now feed themselves via `watchdog::feed_now`, a raw write of the
reload key. It is safe from anywhere - the key register is write-only and the
reload is idempotent - and it says "still waiting on something that will time
out", not "trust me". Three callers: the radio TX poll loop, each SD block
transfer in the `BlockDevice` impl (a failing card spends its full per-block
timeout on each of many blocks in one FAT operation), and the ACMD41 loop in
card init. `wait_on_busy` is deliberately *not* fed - a radio that never
releases BUSY is exactly what the watchdog is for.

`CFG_END` also gained an ordinary `feed`, which `FW_DATA`/`FW_END` already had:
applying a config is a radio re-init, a GPS push that waits 250 ms for an ack,
an SD write and a flash page erase, all before the loop's own feed.

### Waking the GPS lost its configuration

`configure` writes the RAM layer only, justified by "the WIO controls the power
rail, so it re-runs at every boot". Backup mode breaks that premise: it cuts
power to the receiver core, and RAM config does not survive. The module came
back on its BBR/flash defaults - including the four NMEA sentences the firmware
silences, which at 9600 baud is the difference between RMC/GGA arriving and
GSV crowding them out.

Nothing re-pushed them. The existing retry hung off `gps_nmea_seen`, already
true by then, and `configured` was still set. Now `wake` clears `configured`
(and returns early if the module was not asleep), and the retry is driven off
`configured` instead of off the first sentence, which covers both this and the
original case of a boot-time push landing before the receiver had started.
Capped at 5 tries, 2 s apart, re-armed on a wake or a config push, so a module
that keeps refusing does not talk over the console forever.

`sleep` also returns early when already asleep: UART RX is one of the wake
sources, so re-sending PMREQ woke the module just to put it back.

### An abandoned firmware transfer took the node off the air for good

An active transfer owns the loop - no GPS, no radio, no SD - and only an
explicit `FW_ABORT` cleared it. The ESP sends one after 5 s idle, but an ESP
that *resets* mid-upload never does, and the WIO then sat there feeding its
watchdog and doing nothing until someone power-cycled it. `FwUpdate` now stamps
`last_ms` on every frame and `expire` drops a transfer after 30 s of silence,
far longer than any legitimate gap between chunks.

### CFG_END / FW_END destroyed the transfer, so the documented retry failed

`wio_link.py` retries `OP_END` on a WIO timeout, reasoning that "the ESP keeps
the transfer open, so re-sending OP_END is safe". It was not safe on the WIO
side: `CfgTransfer::end` cleared `active` before doing the work and
`FwUpdate::end` took the state before even checking the CRC.

So: the WIO applies and saves a config, the apply outlasts the ESP's 2 s
deadline, the host retries, the WIO answers `INVALID_STATE`, and `wio_config.py`
exits non-zero on a push that fully succeeded.

`CfgTransfer` now remembers the CRC of the transfer that verified plus whether
the caller went on to apply it (`mark_applied`, called from the main loop after
the stores). A repeat END carrying that same CRC returns the new `CfgEvent::Done`
and is simply re-acked. The CRC is what distinguishes a retry from a stray
frame - it names the transfer. A config that verified but would not *parse* is
never marked, so retrying that one keeps failing, which is the truth.

`FwUpdate::end` now leaves the transfer in place on every failure path and
consumes it only on success, so a retry re-reports the real error
(`CRC_MISMATCH`, `BAD_SIZE`) instead of `INVALID_STATE` for something this call
had quietly thrown away. Safe to leave state behind now that the idle deadline
above exists to collect it.

### The ESP reported sleep states it had not restored

`PFLAG_WIO_SLEEP` / `PFLAG_GPS_SLEEP` were written on a BLE config write and
read back only to fill the settings characteristic. Nothing re-applied them and
nothing cleared them, while every deep sleep cuts the rail - so the WIO came
back awake with its GPS running and the app was still told both were asleep.
The heartbeat's link-up transition now re-sends whichever flag is set, next to
the `CFG_READ` already there. Only a set flag is sent; a clear one already
matches what a freshly booted WIO does.

### Smaller

- **First-beacon stagger scaled with the node address**, so node 200 sat silent
  for 3 min 22 s after boot with nothing on the console to say why. Folded into
  eight slots (`address % 8`); a second apart is already wide against a beacon's
  air time, and the post-transmit jitter keeps them apart from there.
- **The bootloader read u32s through an unaligned pointer.** `flash_program`
  did `ptr::read(src as *const u32)`; `copy_page` passes an `align(8)` buffer,
  but `write_state` passes a bare `[u8; 8]` stack local with alignment 1. UB,
  and the compiler is free to answer it with an `ldrd`, which faults - in the
  code that writes the boot state, with no way back. Now assembled with
  `u32::from_le_bytes`, which costs nothing where LLVM can prove the alignment.
- **`platform::random` could return below `min`.** `(s as i32).unsigned_abs()`
  is 2^31 for exactly one state value, which casts back negative, and `%` keeps
  the sign of its left operand. One call in 2^32, and the effect was only a
  repeat jitter that read as already due, but the cast was wrong. Reduced in
  unsigned space instead.

### Checked and found sound

Link framing and CRC on both sides, the LoRa frame/position/ping codecs and
their tag disjointness, the roster's eviction and replay, every fixed-buffer
index in the link and bulk paths, the bulk sequence/duplicate logic across ESP
and WIO, the swap bootloader's resume-after-power-loss state machine, the SD
CSD capacity math, and the NMEA parser.

### SNR read back unsigned

Reported SNR was +48 dB, which no LoRa link produces (the demodulator tops out
near +13 dB). `LoRaPacketStatus::snr_pkt()` in stm32wlxx-hal builds its ratio
with `i16::from(buf[2])`, zero-extending the byte, but SnrPkt is two's
complement in quarter-dB steps. Every negative SNR therefore came back as a
large positive: 0xC0 is -64 quarter-dB = -16.0 dB, and read unsigned it is 192
quarter-dB = +48 dB. `radio.rs` now casts the numerator back through `i8`
before scaling to centibels. RSSI is unaffected - RssiPkt really is unsigned
(-RssiPkt/2 dBm), which is why it stayed plausible while SNR did not.

## BLE session policy moved to midair-proto (host tests)

The settings, the config-write handling and the sleep/advertise cycle used to
live inline in `esp/src/bin/main.rs`, where nothing can test them: the crate is
`no_std`/`no_main` for riscv32imac, so `cargo test` cannot build it and there
is no host target to run it on. They are now `proto/src/session.rs`, following
the precedent `roster.rs` set, and `main.rs` keeps only the effects (pins, the
WIO link, flash, the console).

The split is decision vs effect:

- `Stored` is the persisted settings themselves (RTC RAM words + the flash
  record), including the flag encoding, the "0 = never configured" window
  default, and `rail_at_boot`.
- `session::apply` takes a config-characteristic write and the current
  settings, returns the new settings, the ack, whether the change has to reach
  flash, and an `Action` the firmware carries out (drive the rail, ask the WIO,
  set the notify interval). The two WIO-facing actions carry a success ack that
  the firmware replaces if the link fails - that is the only place an ack is
  built outside the module.
- `session::Window` is the advertising budget for one wake, as a deadline. The
  interval is passed to `next()` per call (so enabling sleep takes effect
  within the window) while the window length is sampled once at construction
  (so shortening it cannot strand a board mid-window).

What the tests are actually for: every value here decides whether an unattended
board can still be reached. A window that clamps to 0, a budget a failed
connect can restart, a record that reads back as "unconfigured" after an update
- each one is either a board that never sleeps or a board nobody can connect
to, and neither is visible without hardware and a wait. 26 tests cover the
clamps and their echoed acks, the flags (including `PFLAG_PWR_OFF` being stored
inverted so an all-zero record means the rail is on), version 2 records, crc
and magic rejection, and the wake/advertise/linger/sleep sequence.

Not covered: the bulk transfer state machine (`handle_bulk`) still lives in
`main.rs`, because splitting it means threading the WIO's per-chunk answers
back into a state machine and the sequencing is shared with the USB path.

## Wio-S3 board port: what the module decides for us

The Wio-S3 is an ESP32-S3R8 and an SX1262 in one can, so it replaces both
MCUs, not just the WIO-E5. Findings that cost a search and are easy to get
wrong:

- Do not trust results for "Wio-SX1262 with XIAO ESP32S3". That is a
  different product - two boards over a B2B connector, with the radio on
  GPIO39-42. On the Wio-S3 those four GPIOs are free user pads, so the
  internal SX1262 must sit on GPIOs the module does not bring out
  (GPIO4-10 is the only gap large enough for SPI + NSS + BUSY + DIO1 +
  NRST). That is inference, not documentation - the introduction page does
  not publish the internal map, and it is the one fact the radio port
  blocks on.
- GPIO33-37 are absent from the pad list because the R8 part's 8 MB PSRAM
  is octal and takes them; GPIO26-32 are the flash interface. A generic
  ESP32-S3 pin table will offer both.
- The ESP32-S3 is Xtensa. The `stable` toolchain that builds the C6 crate
  does not target it - it needs the `esp` channel from `espup`.

The radio driver ports better than it looks. The STM32WLE5's SubGHz block
is an SX1262 die on an internal SPI and takes the same opcodes, so
`Sx1262Driver` needs a new transport (NSS, BUSY-wait, SPI transfer) rather
than a new driver, and the typed `stm32wlxx_hal::subghz` structs become the
byte sequences they already compile to. Every tuned value - PA config, OCP,
sync word, the SNR sign extension - carries over. `lora-phy` would mean
re-deriving the whole config surface `RadioConfig` drives.

## DIO2 as RF switch: the one WL entry that the Wio-S3 reopens

The SX1262 feature list above concluded that SetDio2AsRfSwitchCtrl (0x9D)
"can never become a config key" because the STM32WLE5's die has no bonded
DIO2 and the opcode is absent from its table. That holds for the WL and
only for the WL. The Wio-S3 carries a discrete SX1262, DIO2 is a real
pin, and the opcode is back - so it is now a key, `dio2_rf_switch`,
default off (the chip's power-up state).

It belongs with `dcdc_enabled` and `tcxo_volts` as a board-description
key rather than a tuning knob: on for a module whose RF port feeds an
external switch or front-end, off for one wired to a connector directly.
Get it wrong and there is no bad-but-working middle - the antenna is
simply never joined to the PA, so every transmission goes into a
disconnected port and the node looks like a range problem.

The u.FL-vs-RF-pad choice on the Wio-S3 itself is *not* this, and is not
software at all: Seeed sells two SKUs, 100020327 with the IPEX connector
and 100079384 with bare RF pads. There is no switch between them to
drive. On the 2.4 GHz side there is no equivalent register either - the
ESP32-S3 has no antenna switch, so that port is whatever the board wired.

Encoding cost: none. Flag bit 4 of `b[1]` was free, so `RADIO_CONFIG_LEN`
stays 28 and `RADIO_CONFIG_VERSION` stays 1. An older reader decodes the
blob unchanged and just does not see the bit.

## s3 crate: the Xtensa link step rejects the C6 crate's build.rs

Copying `esp/build.rs` across to `s3/` fails at link with

    xtensa-esp-elf-gcc: error: unrecognized command-line option
    '--error-handling-script=.../build-script-build'

`linker_be_nice()` installs that flag so undefined-symbol errors get
translated into "you forgot esp-alloc" and friends. It is an lld flag.
RISC-V targets link with rust-lld and accept it; the Xtensa target links
through `xtensa-esp32s3-elf-gcc`, which does not. `s3/build.rs` therefore
keeps only the `-Tlinkall.x` link arg and drops the helper.

Everything else in the C6 dependency set retargets by swapping the chip
feature: esp-hal ~1.0, esp-rtos 0.2 (embassy), esp-alloc 0.9,
esp-bootloader-esp-idf 0.4, esp-backtrace 0.18, esp-println 0.16 all have
an `esp32s3` feature and compiled first try. `esp_rtos::start` takes only
the timer on this chip - no software interrupt argument like the C6 call.

The console stays on USB Serial/JTAG rather than UART0 for a board reason
rather than a preference: GPIO43 is UART0_TX and the carrier wires it to
the D5 LED, so the ROM bootloader's log flickers the LED at every reset.

## The port answers the PMode::Boost question by writing the datasheet value

TODO.md asked whether `PMode::Boost` writing 0x97 to the RxGain register
was right, since `stm32wlxx-hal` documents it as best sensitivity but
Semtech documents only 0x94 (power saving) and 0x96 (boosted). The WL
firmware went through the HAL and got 0x97; the S3 driver writes the
register directly, so the value had to be chosen rather than inherited.

It writes 0x96. An undocumented register value is not something to carry
across a rewrite on the strength of "the old one seemed to work" - if
0x97 turns out to matter, the two builds are now a clean A/B on one
constant (`reg::RX_GAIN_BOOSTED` in `s3/src/sx1262.rs`).

## Two things the port gains from a discrete radio

- **DIO1 is a real pin.** On the WL the die had no bonded DIO1 and every
  receive poll was an SPI `GetIrqStatus`. Here `poll_recv` reads a GPIO
  first, which is what an idle listening node does thousands of times
  between packets. It also means a mis-wired DIO1 presents as a radio
  that answers SPI but never receives - the diagnostics check the two
  separately for that reason.
- **The transmit wait can await.** The WL spun on the IRQ and had to feed
  the watchdog from inside the loop, because a SF12/BW62.5 beacon takes
  9.7 s against a 6 s watchdog. The S3 driver is async, so the executor
  keeps running BLE, GPS and the link through a slow transmission and the
  watchdog is not involved at all.

## GPS backup wake does not need EXTINT after all

I flagged the unrouted `/EXT_INT_GPS` on the wio-s3-max-gps board as a
blocker for GPS sleep, on the grounds that an EXTINT edge is the only way
out of UBX-RXM-PMREQ backup. That is wrong, and the WIO-E5 firmware
already showed it: the PMREQ payload sets wakeupSources to
`(1 << 3) | (1 << 5)` - uartrx *and* extint0 - and `wake()` pulses EXTINT
and then writes two 0xFF bytes. UART activity is a wake source on the M10.

So on a board without EXTINT, `sleep()` and `wake()` both work over the
UART; the first bytes are consumed as the wake-up rather than parsed,
which is what those 0xFF bytes are for. EXTINT is the more deterministic
edge and worth routing on a respin, but it is not something the firmware
is blocked on, and it was not worth a board change on its own.

## The S3 port deletes the in-repo SD driver

`wio/src/sdcard.rs` is 348 lines of SPI-mode SD - init, capacity, single
block read/write - written because `stm32wlxx-hal` 0.6 only implements
embedded-hal 0.2, which ruled out `embedded-sdmmc`'s own driver types.
esp-hal implements embedded-hal 1.0, so `s3/` uses
`embedded_sdmmc::SdCard` over an `embedded_hal_bus::spi::ExclusiveDevice`
and the whole file goes away. Only `sdlog.rs`'s logic - the pending
buffer, the flush cadence, the remount retry, the config read/write -
needed porting.

The SD bus stays at 400 kHz rather than being raised after init. Cards
require <=400 kHz to initialize, and raising it afterwards means reaching
through the `ExclusiveDevice` to the underlying bus; a flush is about a
kilobyte every five seconds, so the 25 ms it costs is not worth it.

## Phase 4: what the single MCU actually deleted

The link protocol did not get ported to something simpler - it stopped
existing. Three concrete cases:

- A config write used to be `session::apply` building an ack, then a link
  frame to the WIO, then a wait, then *replacing* the ack if the link
  failed. Now the action is a signal the hardware loop picks up, so the
  ack the policy built is the ack that gets sent. The "success ack the
  firmware replaces if the link fails" path has nothing to guard.
- `RADIO_BUSY` was two link messages bracketing every transmission so the
  ESP could hold BLE notifications. It is now `state::radio_busy()`, a
  bool. The reason survives - a 22 dBm LoRa transmit beside a 2.4 GHz
  radio is a supply problem - but the protocol around it does not.
- The 3 s heartbeat PING proved the link was alive. There is no link.

`state.rs` is a snapshot, not a channel, and deliberately: every consumer
wants the latest position and never a backlog, so dropping intermediate
values is correct rather than a lossy compromise. It exists at all for the
same reason the ESP cached link data - a central can connect between GPS
epochs and should not have to wait for the next one.

`Stored` sits in a plain static for now. On the C6 it lives in RTC RAM
with an nvs backup because it has to survive deep sleep and a flat cell;
neither applies until phase 5 ports sleep, and a static that resets with
the board is the honest placeholder rather than a half-built persistence
layer.

## A guessed pin map destroyed a board: output-vs-output contention

The Wio-S3's internal SX1262 wiring is not in the module introduction, so
the first hardware build guessed it from which GPIOs the module does not
bring out to a pad. The real map, from the datasheet:

    NSS GPIO21   SCK GPIO4   MOSI GPIO6   MISO GPIO5
    NRESET GPIO7   BUSY GPIO8   DIO1 GPIO9
    DIO2 -> SKY13453 RF switch VCTL (never reaches an ESP pin)

Three of the guesses put an ESP32-S3 push-pull output on a line the
SX1262 also drives:

- GPIO5 is MISO. It was configured as SCK, an output. The radio drives
  MISO whenever NSS is low, so the two fought during every transaction.
- GPIO8 is BUSY. It was configured as NSS and held high. BUSY idles low,
  so this was a *continuous* fight, not a transient one.
- GPIO9 is DIO1. It was configured as NRST and held high. DIO1 idles low.
  Continuous as well.

Two pads driving opposite levels is a short through both output stages.
At roughly 25-40 ohm per pad that is ~50 mA per pin against an absolute
maximum near 40 mA, and two of the three were fighting the whole time the
firmware ran rather than only during SPI traffic. The board got warm and
then stopped working, and now loads its supply without a measurable
short at the connector - which is what a damaged pad or a latched-up IO
rail looks like from outside.

The lesson is not "check the pin map" - it is that an *unverified* pin
map must never be driven. A probe that does not know which end owns a
line has to configure every candidate as an input, read-only, and infer
direction from what moves. The brute-force scanner written before the
datasheet turned up had exactly the same defect and has been deleted:
it drove each candidate as SCK, MOSI and NSS in turn, which is the same
contention on purpose.

Also note `dio2_rf_switch` must be **true** on this board. DIO2 drives
the SKY13453's VCTL, so the radio owns its own antenna path; left false
the PA ramps into an isolated switch on every transmission.

## The RF switch is on DIO2 and DIO3, so neither is a tunable

The pin-map disaster above was output-vs-output contention, and fixing it
did not make the radio safe to transmit. The same Table 2 that gives the
SPI wiring gives two more lines that never reach an ESP pad:

    DIO2 -> SKY13453-385LF VCTL   (which RF path is connected)
    DIO3 -> SKY13453-385LF VDD    (and the 32 MHz TCXO supply)

Both were still carrying Wio-E5 values, because `RadioConfig`'s defaults
were written for that module:

- `dio2_rf_switch` defaulted **false**. Correct on the WL, whose radio is
  on-die with no bonded DIO2 - `SetDio2AsRfSwitchCtrl` is not even in its
  opcode table. Here it means VCTL never rises, so the switch stays on the
  path that is not the PA and every transmit goes into an isolated port.
- `tcxo_volts` defaulted to **1.8 V**, which is what the WL's TCXO wants.
  The SKY13453-385LF datasheet specifies VDD 2.5 - 3.5 V, and its truth
  table has only two rows, both requiring VDD high, followed by: "Any state
  other than described in this table places the switch into an undefined
  state." At 1.8 V the part is out of spec whatever DIO2 does.

So there were two independent ways to transmit +22 dBm into nothing, and
the defaults picked both. `main.rs` patched `dio2_rf_switch = true` on its
own copy of the config, which protected the boot path and nothing else - a
config arriving over BLE or off the card starts from `Default` again.

The fix is in three places, and the third is the one that matters. The
defaults are now this board's hardware; `RADIO.example.toml` says what the
keys mean here; and `Sx1262Driver::init` stops reading them for the two
destructive values - DIO2 switching is unconditional, the DIO3 trim is
floored at 2.7 V, and each logs when it overrides. The keys stay in the
file because they document the board, but no value a user can reach should
be able to ask for a dead PA.

Worth stating what does *not* need a floor: `power_dbm` is range-checked by
the TOML parser, but `RadioConfig::decode` reads the byte straight into an
i8, so the driver clamps it to the -9..+22 the HP PA documents at the point
it reaches `SetTxParams`.

### The rest of the register audit came out clean

Checked against the SX1262 datasheet and its errata chapter, all correct as
written: `SetPaConfig` 0x04/0x07 with deviceSel 0 (SX1262, not SX1261) and
paLut 1; OCP written 0x38 (140 mA) *after* `SetPaConfig`, which resets it;
the TX clamp workaround `0x08D8 |= 0x1E`; the 500 kHz modulation-quality
bit in 0x0889; RX gain 0x96/0x94; the private sync word 0x1424 in
0x0740/0x0741; every `CalibrateImage` band pair; the LoRa bandwidth and
coding-rate codes; `Calibrate(0x7F)` issued after `SetDio3AsTcxoCtrl` and
not before; and `SetDioIrqParams` leaving the DIO2 and DIO3 masks at zero,
which matters here - routing an IRQ to either would fight the antenna
switch for the pin.

The two errata that do not apply are worth naming so nobody adds them: the
inverted-IQ workaround (0x0736) is for `invert_iq`, and this firmware
transmits standard IQ; the implicit-header timeout workaround (0x0902 /
0x0944) is for implicit headers, and this firmware uses explicit ones.

One register is documented by ST rather than Semtech: `SMPS_C0` at 0x0916,
bit 6, clock detection enabled before selecting the SMPS. That is
`SUBGHZ_SMPSC0R` in RM0453. The WL's radio is the same die, so the register
is the same silicon, and the driver only read-modify-writes one bit - but
it is not in the SX1262 datasheet, so treat it as inherited rather than
verified.

`dcdc_enabled` stays **true**, and the module datasheet is why: it quotes
5.5 mA for LoRa RX including the MCU. The SX1262 draws about 4.6 mA in RX
with the DC-DC and about 10.1 mA on the LDO, so 5.5 mA total is only
reachable with the SMPS - which means the module carries the inductor.

## An unbounded BUSY wait hides the fault it should report

`wait_on_busy` was `while self.busy.is_high() {}`. BUSY is an input with no
pull, so a radio that is absent, held in reset or wired to the wrong pad
floats the line and that loop never exits. It runs before every
transaction, including the first one in `init`, so the firmware hangs
before `print_diagnostics` - the one thing written specifically to say "the
radio is not answering, check the pin map" - ever gets to run.

That is precisely the failure mode a mis-wired module produces, and it was
the one case the diagnostic could not report. It now gives up after 50 ms,
comfortably past the ~3.5 ms of a full `Calibrate` or a startup after
NRESET, and lets the transaction return the floating status byte that
`print_diagnostics` already recognizes as 0x00 or 0xFF.

## Floating pins, and what "switching is correct" actually rests on

A sweep of every pin the board brings out, against what the firmware
claims. esp-hal's defaults matter here: `InputConfig::default()` is
`Pull::None` and `OutputConfig::default()` is push-pull at
`DriveStrength::_20mA`.

Found floating, now parked:

- **GPIO14 (LED D2) was never claimed at all.** Its cathode is on the pin
  with the anode on +3V3 through R20, so an unconfigured input leaves the
  LED biased just under its forward voltage rather than held off. Claimed
  and parked at LED_OFF. It is still not *driven* by anything - the
  firmware only blinks D5 - which is a functional gap, not an electrical
  one.
- **GPIO10/11 (J5 JST-SH) and GPIO38-41/47 (J1 header)** were floating
  inputs on pins that leave the board.
- **GPIO12, 13, 15-18, 42, 48** are module pads with nothing routed.

All are now inputs with a pull-down. Pulled rather than driven, because
seven of them go to connectors: a pull is a defined state that still yields
to whatever a user wires up, where an output would fight it. The cost of
leaving them was a mid-rail input buffer with both halves partly on - not
destructive, invisible at 75 mA, and most of the budget once deep sleep
lands.

GPIO0 is deliberately left alone (BOOT, board pull-up and a test point;
driving it would fight whoever holds it low for the ROM loader), and
GPIO19/20 belong to the USB Serial/JTAG peripheral.

BUSY and DIO1 gained pull-downs. Both idle low, so the pull is the level
they already hold - but it is also what makes an absent radio diagnosable.
With no pull, an unpowered or mis-wired SX1262 floats BUSY, which reads
high as often as not and burns the driver's 50 ms busy timeout on every
transaction. Pulled down it reads "not busy", the transfer proceeds, and
the status byte comes back 0x00, which is what `print_diagnostics` looks
for.

### The switch is interlocked with the TCXO, and that is the whole design

Worth writing down because it is not obvious and it is what makes Seeed's
wiring safe: DIO3 powers the TCXO *and* the antenna switch's VDD, and the
radio cannot transmit or receive without the TCXO. So the switch has its
supply exactly when the antenna is in use, and the PA cannot ramp before
the switch is powered - the chip waits `tcxo_startup_ms` first. The
SKY13453 switches in 650 ns, so it is never the constraint.

`init` now writes DIO3 before DIO2, supply before control. Ordering barely
matters in practice - DIO2 is low in STDBY_RC, so the switch sees both
lines at 0 either way, which its datasheet calls a leakage condition and
not a damaging one - but VCTL is only specified at or below VDD, and there
is no reason to write the commands in the order that needs that argument.

### The gap this leaves, for whoever writes the beacon

Nothing in the firmware transmits yet. When it does, note that the radio
does not reset with the MCU and the firmware does not re-check it: if the
SX1262 browns out and restarts on its own, it comes back with DIO2 and
DIO3 at their power-up defaults - switch unpowered, switching disabled -
and the next `send()` would ramp the PA into an isolated port with nothing
having gone visibly wrong.

The existing periodic status line is the detector, now that `init` clears
the boot latch: a reappearing `XOSC_START_ERR` (0x0020) means the radio
restarted underneath us. A beacon should check the radio's mode and error
word before keying up and re-run `init` if either looks like a fresh
power-up.

## The u.FL-versus-pad choice is a SKU, and it is a third way to kill a PA

Asked whether the IPEX or the pin breakout is configured by default. It is
neither - there is nothing to configure. The choice is two part numbers:

    100020327  Wio-S3     u.FL connectors on the module; pads 18/37 are NC
    100079384  Wio-S3-N   no connectors; RF on pads 18 and 37

Table 1 of the module datasheet carries it in the pin names themselves:
`LORA_ANT / NC` and `WIFI / BT_ANT / NC`. The `/ NC` is not a footnote, it
is the other SKU.

Nothing selects between them - not a register, not DIO2, not a board
jumper. Table 2 documents exactly one switch, the SKY13453-385LF on DIO2
and DIO3, and DIO2's level is fixed silicon behavior (high in TX), so that
part is a TX/RX switch and cannot be a connector selector. The module's
block diagram draws an "RF switch" on the Wi-Fi/BT path too, but the
datasheet documents no control for it and the ESP32-S3 exposes none, so
there is no firmware surface there either.

**The consequence for this carrier is the part worth remembering.** The
board runs pad 37 straight to the SMA J6 and pad 18 to test point BLE1, so
it only works with the -N part. Fit an IPEX module and pad 37 is NC: the
SMA is connected to nothing, and the PA transmits into an open. That is
the same failure as `dio2_rf_switch = false` reached by a completely
different route, and unlike that one, no firmware check can see it - the
radio reports a healthy standby and a clean error word either way.

The trade runs opposite for the two bands, which is worth knowing before
ordering:

- **-N part:** LoRa works into the SMA. BLE is a stub - pad 18 stops at a
  test point, so there is no 2.4 GHz antenna on the board as drawn.
- **IPEX part:** BLE gains a real connector on the module itself. LoRa is
  destroyed on the first transmit.

So the -N part is the only safe choice today, and the BLE antenna stays a
board problem. `BOARD-REVIEW.md` in the `wio-s3-max-gps` repo proposes the
respin that makes all three states reachable: a 50R trace to a junction
pad with two 0402 0R jumpers per port, exactly one populated, and neither
populated for an IPEX module.

---

## Completing the Wio-S3 port (2026-08-21)

Finished everything `TODO.md` listed as port parity, plus the beacon, which
turned out to be missing outright rather than deferred - the firmware could
hear the network but had no `send()` call anywhere, so the DIO2/DIO3
antenna-switch fix had never been exercised on air.

### The bulk transfer belongs in `proto/`

On the two-MCU board the transfer was split down the middle: the C6 parsed
the ops and forwarded each over the UART link, the WIO reassembled the bytes
and checked the CRC. Neither half was host-testable. Putting the whole state
machine in `proto/src/bulk.rs` made it one object that `cargo test` drives
(22 new tests), and made "one transfer at a time" a property of the object -
it records which transport began it, so a BLE write and a USB frame cannot
interleave into one buffer.

Firmware images stream through a `Sink` trait rather than being buffered; an
ESP app image is ~380 KB and `CONFIG_MAX` is 1 KB. The running CRC had to be
split out of `link::crc32` so the pre/post inversion happens once around the
whole stream rather than once per chunk - there is a test pinning the two to
the same value.

### Two OTA hazards worth remembering

**`OtaUpdater::next_partition()` returns the *running* slot when `otadata`
is erased.** With both sequence numbers uninitialized, `current_app_partition()`
answers `Factory`; `next_ota_part()` then computes
`(Factory.ota_app_number() + 2) % 2`, and `Factory.ota_app_number()` is
`0u8 - OTA_SUBTYPE_OFFSET`, which underflows to 240 in a release build. That
lands back on ota_0 - the slot a freshly USB-flashed board is executing
from. Writing an image there does not fail, it destroys the running firmware
mid-transfer.

Fixed twice over: `normalize_otadata()` at boot writes the first real
sequence number when `otadata` names no slot, so every later transition is
an ordinary slot-to-slot move; and `OtaSink::begin` compares the chosen slot
against `booted_partition()` (read from the MMU, not from `otadata`) and
refuses if they match, or if the booted slot cannot be determined at all.

**Erased `otadata` is also what makes a USB flash authoritative.** The flash
runner passes `--erase-parts otadata`, because a board that had taken an OTA
is booting ota_1, and `cargo run` writes ota_0 - without the erase the board
keeps running the old image and the flash looks like it did not take.

### `embedded_storage::Storage::write` erases per sector

It is read-modify-erase-write of the whole 4 KB sector around whatever you
write. Streaming a firmware image at the transfer's 192-byte chunk size
would erase every sector 21 times: ~70 s for one update, against ~4 s with a
sector staged in RAM first. The staging buffer is why `Flash` is 7 KB of
.bss.

### One flash peripheral, two users, and no critical section

The settings mirror and the OTA writer both need the flash. It lives behind
an `embassy_sync::mutex::Mutex` rather than a critical section: an erase is
tens of milliseconds, and holding interrupts off that long would drop the
BLE connection carrying the update. Lock order is always transfer-then-flash
and nothing takes them the other way round.

### Clock skew between tasks

The roster is written by the hardware loop and read by the BLE session, and
the ages it stamps in are only meaningful if both use the same clock. The
hardware task originally measured from its own `start = Instant::now()`,
which is offset from the BLE task's by however long boot took. Both now use
`Instant::now().as_millis()` directly, which is uptime.

### Small things found while reviewing

- `poll_recv` returned without clearing a pending IRQ that was not RxDone,
  leaving DIO1 asserted and costing an SPI read on every subsequent poll.
- The notify deadline (`next_notify += interval`) could fall into the past
  after a slow tick, and `Timer::at` on a past instant returns immediately -
  a spin, not a catch-up. Clamped to `Instant::now()`.
- The beacon interval was measured from the top of the loop pass, before an
  `await` that can take 9.7 s at the slowest settings the config accepts.
- A config that sets `sd_enabled = false` has to be written to the card
  *before* the card is disabled, or the next boot reads nothing and comes up
  with the card enabled again.
- The README documented `tcxo_volts = "1.8"`, which is the Wio-E5's value
  and the one that leaves this board's antenna switch undefined while the PA
  transmits. It is 3.3 in the defaults and floored at 2.7 in `init`.
- `ESP_ADV_MIN_S` is 1, not the 3 the README claimed.

## S3 power: where 180 mA comes from, and why "deep sleep" is 30 mA

Full write-up in `POWER-S3.md`. The findings that cost a search:

- **esp-radio 0.17 hardcodes BLE modem sleep off.** `create_ble_config` in
  `src/ble/os_adapter_esp32c3_s3.rs` writes `sleep_mode: 0, sleep_clock: 0`
  as literals, with a source comment saying "ideally _some_ of these values
  should be configurable". ESP-IDF defaults `CONFIG_BT_CTRL_MODEM_SLEEP` on,
  so the module datasheet's ~31 mA "BLE advertising" figure (derived: the
  158 mA BLE+LoRa-TX row minus the 127 mA LoRa-TX row) is not reachable from
  this crate. The part sits in RF-working current, ~90-95 mA, continuously.
  There is no config key; the only lever is `BleConnector`'s `Drop`, which
  calls `ble_deinit()` and releases the `PhyInitGuard` - i.e. build the BLE
  stack per advertising window instead of once at boot.
- **esp-radio's BLE `TxPower` defaults to `P9` (+9 dBm).** `Default::default()`
  on the connector config picks it. Worst-case spike to have beside the LoRa
  PA, which is the thing `radio_busy` exists to avoid.
- **The S3's deep sleep releases every pad that is not explicitly held**
  (esp-hal clears `dg_pad_force_unhold` in the sleep prep). The SX1262 leaves
  sleep on an NSS *falling* edge, so a floating GPIO21 wakes the radio back
  to STDBY_RC for the whole sleep interval. GPIO21 is an RTC pin (S3 range is
  0-21) so `rtcio_pad_hold` covers it; SD CS on GPIO44 is not, and needs the
  digital pad-hold register esp-hal 1.0 does not expose.
- **`Spi::with_miso` applies `Pull::None`** and enables the input buffer. MISO
  is only driven while CS is low, so GPIO5 and GPIO3 float almost always -
  the same defect the pin sweep fixed everywhere except the pins the
  peripherals had already claimed.
- The 10 ms hardware-loop poll is *not* a suspect: esp-rtos's Xtensa idle hook
  is `waiti 0`, the second core is never started, and the BLE HCI read is
  event-driven (`HciReadyEventFuture`), not a spin. Ruled out, do not
  re-investigate.

## Sleep on command, and the two things that made "deep sleep" 30 mA

- **V_BCKP is unconnected on this board** (`wio-s3-max-gps/BOARD-REVIEW.md`:
  goes to test point BCKP1 only). That kills the obvious deep-sleep fix. The
  M10's backup domain - RTC, the BBR holding the ephemeris, *and the UART-RX
  wake source* - is supplied by V_BCKP, so `Gps::sleep` (RXM-PMREQ backup)
  on this board means a cold start on every wake and an unproven wake path.
  On a 60 s cadence a tracker that backs its GPS up never gets a fix at all.
  So the ~30 mA the GPS draws through deep sleep is a board fact, exactly as
  the `enter_deep_sleep` comment says, and it stays behind the explicit
  `CFG_GPS_SLEEP` lever rather than becoming automatic. Fix is the board's.
- **The SX1262 was waking itself back up.** The S3 releases every pad that is
  not explicitly held when the digital domain drops, and the radio leaves
  sleep on a *falling* NSS edge - so the radio `PrepareSleep` had just put to
  sleep came back to STDBY_RC for the whole interval. GPIO21 is inside the
  S3's RTC GPIO range (0-21), so `RtcPin::rtcio_pad_hold` reaches it; the
  C6's "reconfigure the Output first, then release the hold" ordering applies
  unchanged. SD CS (GPIO44) has the same problem and cannot be fixed the same
  way - RTC pins stop at 21, and `RTC_CNTL_DIG_PAD_HOLD` is not exposed by
  esp-hal 1.0.
- **`PrepareSleep`'s one-second park budget was shorter than a beacon.** A
  transmit awaits the frame's time on air, up to ~9.7 s at the slowest
  settings `RadioConfig` accepts, so a sleep landing mid-beacon timed out and
  slept with the receiver still on. Now waits the running config's own
  `tx_poll_timeout_ms`, published through `state`, and the loop will not
  start a beacon with a sleep pending.
- **`note_sleep` writes the settings, not just the magic word.** Persistent
  RTC RAM is not zero-initialized, so stamping the magic in front of
  never-written interval/flags words would hand the next boot cold-boot
  garbage as a stored config. `set(get())` resolves to the RTC copy or to
  `Stored::new` and writes whichever it was.
- **A pull on an SPI pin needs `InputSignal::freeze()`.** `with_miso` calls
  `apply_input_config(&InputConfig::default())` unconditionally, so anything
  configured beforehand is overwritten; a frozen signal makes that call a
  no-op. Same for `with_rx`/`with_tx` if the UART pins ever need it.
- **`CFG_SLEEP_NOW` ordering.** `apply_config` runs inside the GATT session
  with the ack unbuilt, so it signals rather than sleeping. The session's arm
  waits 400 ms after the signal so the ack and the settings republish leave,
  and `serve` does the sleeping once the link is down. The app remembers that
  it asked, so the disconnect gets rewritten into the answer instead of
  standing as a fault.

## The 180 mA closes once the measurement point is known

Measured at the **4.2 V regulator input**, i.e. after the D3/D4 diode-OR
(`DM3CS-SF` Schottky) and before **U2, a `TLV75733PDBVR`**.

- **U2 is an LDO, not a switcher, so input current = output current.** The
  180 mA is the +3V3 load itself, not a higher-voltage number that divides
  down. An earlier estimate assumed the measurement path explained a
  ~30-50 mA gap; it does not, and cannot.
- The two loads actually missing from that estimate were **the GPS active
  antenna** (the module's `VCC_RF` -> U3 SiP32431 -> R15 -> L1 bias-tee ->
  SMA; the M10's 25-31 mA datasheet figure is the receiver alone) and **BLE
  advertising at +9 dBm** rather than the 0 dBm the datasheet figures assume.
  With both counted the budget brackets 136-183 mA.
- U3's enable is the GPS's own `LNA_EN`, so nothing in firmware can drop the
  antenna without parking the receiver. That makes the V_BCKP board fix worth
  30-50 mA on a sleeping board rather than 25-31.
- Topology costs that are nobody's firmware bug: U2 burns
  (4.2-3.3) x 0.18 = **162 mW**, about a fifth of everything taken from the
  cell, and ~30 C rise in a SOT-23-5. The Schottky drop plus the LDO's
  dropout means the rail starts sagging with the cell still near 3.7 V, so a
  chunk of LiPo capacity is unreachable. A buck in U2's place recovers both.

## The antenna type is nowhere in the firmware, and the bias tee is unconditional

The carrier is built for an **active** GPS antenna: `U5.VCC_RF` -> U3
(SiP32431) -> R15 10R -> L1 27nH -> SMA J2 center pin, populated and with no
option fitted. U3's enable is the GPS's own `LNA_EN`, not a host GPIO, so DC
is on the antenna port whenever the receiver's RF section is on.

`Gps::configure` writes six `CFG-SIGNAL-*`, `CFG-PM-OPERATEMODE`,
`CFG-RATE-MEAS`, `CFG-NAVSPG-DYNMODEL` and six `CFG-MSGOUT-*`. **No
`CFG-HW-ANT_*` key at all**, so the antenna supervisor is at factory default
and nothing has ever told the receiver what is on the cable.

Consequences with a passive antenna fitted:

- The 5-20 mA LNA line comes out of the power budget, which then leaves
  17-49 mA of the measured 180 mA unaccounted for. Do not quietly reshuffle
  other estimates to cover it.
- A DC-open feed (series cap, whips) ignores the bias and is harmless. A
  DC-shorted feed - common on passive patches with a shorted-stub feed - puts
  3.3 V across R15's 10 ohm, i.e. a ~330 mA demand into a short, clamped by
  U3's limit and the M10's `VCC_RF` regulator. That is a fault, not a line
  item, and it would land on +3V3 as tens of mA.
- One probe settles it: DC across R15. ~0 V = nothing drawn. Volts across it
  = current flowing, which is right for active and a short for passive.

If passive, the firmware should write `CFG-HW-ANT_CFG_VOLTCTRL = 0` as a new
`GpsConfig` key. **Not written yet on purpose** - the key id needs checking
against the M10 interface description, and a guessed UBX key id still writes
something.

## LNA_EN is not gated by the antenna supervisor, so "passive mode" is hardware

MAX-M10N integration manual (UBXDOC-304424225 R03), Table 22, section
3.3.2.4:

| Mode | LNA_EN |
|-|-|
| Normal operation | **High** |
| Software standby | Low |
| Hardware backup | Low |
| LEAP mode | Duty cycling |
| Antenna supervisor, power down on short detect | Low |

So `CFG-HW-ANT_CFG_VOLTCTRL = 0` does **not** turn the antenna feed off -
the supervisor only ever pulls LNA_EN *low*, and only on a detected short,
which needs a sense circuit on `CFG-HW-ANT_SUP_SHORT_PIN` this board does
not have. VOLTCTRL is disabled by default anyway (an unconfigured receiver
reports antenna status "DON'T KNOW"). Polarity is fixed, and the pin also
drives the module's internal LNA, so it is not the firmware's to repurpose.

No config key was added: a setting that does nothing is worse than none.
For a passive build the fix is **depopulating R15**, the 10 ohm in the bias
tee's DC path. With a wire antenna the SMA center pin is DC-open anyway, so
the feed drives nothing and the cost is zero - tidiness, not a saving.

`CFG-PM-OPERATEMODE` set to a power-save mode duty-cycles LNA_EN as a side
effect (the LEAP row). That is already a config key (`power_mode`).

## Status OLED

- Lives in `hardware_task` rather than its own task **because of sleep
  ordering**: the panel is on the always-on +3V3 and holds its frame and its
  current through a deep sleep, so it has to be blanked before `SLEEP_READY`
  is signalled. Owning it where `PrepareSleep` is answered makes that a
  function call instead of a negotiation between tasks.
- Blanking sends `0xAE` *and* `0x8D 0x10` - display off alone clears pixels
  but leaves the charge pump running, which is where the current is.
- J5's two nets are named `GPIO10`/`GPIO11` in the schematic and nothing
  else, so SDA/SCL is firmware's choice. `probe_oled` tries both orders and
  both addresses (0x3C, 0x3D), stealing the pin singletons between attempts -
  sound only because it runs once during init with one live borrow at a time.
- esp-hal's `I2c` has an inherent blocking `write` that shadows the
  embedded-hal-async trait method; use `write_async` explicitly or the
  `.await` fails to compile with "Result is not a future".
- 128x32 needs multiplex 0x1F and COM pins 0x02. The 128x64 values (0x3F,
  0x12) render doubled or interlaced rather than blank, which looks like a
  framebuffer bug and is not one.
- **SDA=GPIO10 / SCL=GPIO11 is the order tried first** (bench wiring). The
  reverse is kept only as the fallback. A wrong order cannot false-positive
  the probe: the panel acks on the master's SDA line, so with the lines
  swapped the master reads its own released SDA and sees NACK.
- **Page addressing, not horizontal.** Horizontal addressing (`0x20 0x00` +
  the `0x21`/`0x22` window) is SSD1306-only; an SH1106 ignores those
  commands and dumps the whole 512-byte run into page 0, which is what a
  screen full of scrambled noise looks like. Setting the page and column
  per page (`0xB0|page`, `col & 0x0F`, `0x10 | col>>4`) works on both. An
  SH1106 also centers 128 columns in 132 columns of RAM: if the image is
  right but shifted two pixels, set `COL_OFFSET` to 2.
- **Init clears all 8 pages of RAM before `0xAF`.** Display RAM is undefined
  at power-up, and a controller with more pages than the panel shows keeps
  garbage in the ones the flush never writes.

## Compass on the OLED

- **The bus is owned by the hardware task, not by the display.** Two devices
  on one I2C controller with no arbitration is only safe because there is
  exactly one caller; `Oled`'s methods take `&mut I2c` rather than holding it.
- **HMC5883L's data registers are X, Z, Y** - not X, Y, Z - and big-endian,
  where the QMC5883L is X, Y, Z little-endian. Getting the axis order wrong
  produces a heading that looks plausible and is wrong.
- **An all-zero magnetometer read is discarded.** It is what a part returns
  before its first conversion completes, and feeding it to the running
  min/max would drag both axes' minima to zero and poison the hard-iron
  calibration for the rest of the session.
- Heading is magnetic, no declination applied. That is correct here and not
  laziness: the bearing and the heading are both off by the same declination
  and it cancels out of the *relative* bearing, which is the only thing drawn.
- Not tilt-compensated, and it cannot be with these parts - a magnetometer
  alone measures in the board's frame. Documented rather than hidden.
- Screen choice is compass-when-possible rather than alternating pages: a
  0.91" panel is read at a glance, and a glance landing on the wrong half of
  a rotation is worse than a screen that only changes when the situation does.
- `geo::bearing_deg`/`distance_m` must do the longitude subtraction in i64.
  Two fixes either side of the antimeridian are ~3.6e9 apart in 1e-7 degrees,
  past i32::MAX; in release that wraps silently and puts the arrow exactly
  180 degrees out. A test pins it.

## Why PowerConfig fields are Option, not plain values

RADIO.CFG's contract everywhere else is "an absent key means the default",
which works because the file is the only source of those settings. The three
duty-cycle settings have a second source: RTC RAM, mirrored to nvs, writable
live over BLE. Under the usual contract, pushing a config to change the
beacon interval would carry ble_off_s = 0 by omission and silently kill a
duty cycle set from the app.

So PowerConfig is Option field by field: present means "adopt this", absent
means "leave the live value alone". An explicit 0 is therefore still a
request, which is what makes turning a duty cycle off expressible at all.

The file is adopted on a cold boot only. A deep-sleep wake keeps its RTC
copy, because a wake check that re-read the card would undo a live change
once per interval forever. A deliberate push over BLE or USB adopts
unconditionally, since somebody is sending the file right now.

Consequence to remember: the first advertising window's length is decided
before the SD card is mounted, so adv_window_s from the file lands from the
second window on. ble_off_s is re-read at the end of every window.

## Repo layout: why the firmware is not at the root (2026-08-28)

The tree was `esp32c6-gps/{s3,proto,tools}` plus a root full of the
two-MCU board's paperwork. It is now
`telemetry-in-midair-rs/{firmware,proto,tools,docs}`, matching the git
remote's name, since no ESP32-C6 remains in it.

The obvious tidy - hoist the firmware crate to the repo root - was tried
and rejected for a concrete reason. `firmware/.cargo/config.toml` sets
`build.target = "xtensa-esp32s3-none-elf"` and `build-std`, and cargo
applies `build.target` to every crate beneath the config. A nearer config
cannot cancel it; both escapes were tested:

- child `target = []` against a parent string: `failed to merge key
  target ... expected array, but found string`.
- child `target = ["x86_64-unknown-linux-gnu"]` against a parent array:
  cargo concatenates them and builds for both, so the embedded one still
  fails.

So a root config would capture `proto/`, and `cd proto && cargo test`
would fail with `can't find crate for 'std'` and `#[panic_handler]
function required` - a crate that had not changed, failing because of a
file two directories up. Only a command-line `--target` overrides it,
which is machine-specific and easy to forget. The firmware keeps its own
directory so the config and `rust-toolchain.toml` (channel `esp`) stay
scoped to it.

The directory rename has a cross-repo cost worth remembering: `gps-gui-rs`
depends on `../telemetry-in-midair-rs/proto` by path and pulls
`RADIO.example.toml` through `include_str!`, and
`wio-s3-max-gps/BOARD-REVIEW.md` cites the port document. All were
repointed; `proto/` and `RADIO.example.toml` therefore cannot move without
touching that repo too.

## The two duty cycles are mutually exclusive (2026-08-28)

Drawing the state timeline surfaced this: `serve` tests `window.next(..)`
for deep sleep *before* the `ble_off_s` return, so a board with
`sleep_interval_s` set never takes the BLE-down branch. Setting both is not
"take the modem down between wakes" - it is deep sleep, and `ble_off_s` is
dead config. Only the BLE-down period's own `SLEEP_NOW` wait can mix them,
and that is a commanded sleep rather than the cadence.

This is not a bug (deep sleep saves strictly more) but it is not written
down anywhere the settings are documented either. ARCHITECTURE.md now says
it above the two gantt charts.

## Checking mermaid without a browser

`mermaid.parse()` runs under bun + jsdom with no Chromium, which catches
every syntax error in a doc's diagrams in about a second:

```
bun add mermaid jsdom
# stub globalThis.window/document/navigator from a JSDOM instance, and
# DOMPurify as a passthrough, then import mermaid and parse each block
```

`mermaid.render()` does *not* work this way - gantt layout calls `getBBox`,
which jsdom does not implement. Parsing is the part that catches real
mistakes; layout has to be eyeballed on GitHub.

## Buffered log lines are lost on every deep sleep (2026-08-28)

Found auditing the state gantt, not from a symptom. `log_position` appends
to a 1 KB RAM buffer; the only thing that writes it out is `sdlog.poll`,
called once per hardware-loop pass, which flushes on a 5 s timer or at
512 bytes (~8-10 lines), whichever comes first. At 1 Hz the timer always
wins, so the buffer holds 0-5 fixes at any instant.

`Request::PrepareSleep` parks the radio and blanks the panel but does not
flush, and it sets `standby = true`, whose `continue` skips `sdlog.poll`
for every pass after it. Deep sleep is a full reset, so that RAM is gone.

Cost: 0-5 fixes per sleep cycle, average ~2.5. A board on
`adv_window_s = 15` / `sleep_interval_s = 45` logs ~15 fixes per wake, so
up to a third of each wake's track never reaches the card - and the gap is
always at the end of the window, which is the part a reader would use to
work out where the board was when it went down.

Fix is one line in the `PrepareSleep` arm (`sdlog.poll(now)` before
signalling `SLEEP_READY`, or a `flush`-equivalent), but the flush opens,
appends and closes the file, so it has to be inside the parking budget
rather than after it. `Request::Reboot` has the same hole with a 500 ms
delay it could easily flush inside of.

Not done here - this was a docs pass, and it wants a card on the bench to
confirm the gap and then confirm it closed.

## Three modes, and what that resolved (2026-08-29)

Implemented `docs/STATES-PLAN.md`. The mode is `ble::Mode` (stored / idle /
tracking), kept in `session::Stored` alongside the duty-cycle settings, so it
rides the existing RTC-RAM-plus-nvs mirror rather than needing a store of its
own. Record version 4 -> 5 and settings blob 4 -> 5, both pure appends; the
blob's version byte is an exact match, so **gps-gui-rs has to be rebuilt
against this proto or it stops decoding the settings characteristic**.

Three things that were not obvious going in:

**Phase and mode are the same type.** A first cut had a separate `Phase`
enum for what the board is doing (WakeCheck / Idle / Tracking) against
`Mode` for what it was told to be. They collapse: a board that is awake with
mode `Stored` *is* a wake check, because that is the only way to be awake in
that mode. One enum, and `budget_s`/`at_expiry` switch on it.

**Idle is live-only, and the normalization belongs at the flash boundary.**
`Stored::encode_record` writes `mode.persisted()`, which turns idle into
stored; the RTC copy keeps idle, which is what the settings characteristic
reports and what the serve loop budgets on. Doing it any earlier means an
app that asks for idle reads back "stored" and looks broken.

**A bare PMREQ to a sleeping module wakes it.** The M10 consumes the first
bytes sent to it in backup as the wake-up itself, so `Gps::sleep` on a
module that is already down would have its request eaten and the wake would
stand - a board sleeping with its receiver acquiring, which is the whole
load the mode work exists to remove. `Gps::park` therefore sends two
throwaway bytes, waits 5 ms and then sends the request. It costs a few
milliseconds of receiver time per park and is unconditional, because after a
reset the driver's belief about the module is worth nothing.

The corollary: at a wake-check boot `gps.sleeping` is set to true by hand.
Nothing else would say so, and a promotion straight to tracking would find
`wake()` a no-op, leaving the receiver in backup with nothing to notice -
the settings retry is gated on a sentence having been seen, and in backup
there are none.

Dead config resolved: `ble_off_s` is Tracking's knob, `sleep_interval_s` is
Stored's cadence, and `at_expiry` asks the mode which applies. The 2026-08-28
note below ("The two duty cycles are mutually exclusive") described the bug
this closes.

`sleep_interval_s = 0` now also means "never store this board": with no
cadence, an idle timeout has nowhere to send it. That is what makes an
unconfigured board behave exactly as it did before - reachable forever -
rather than storing itself ten minutes after a flash.

## The firmware does not build with -C force-frame-pointers (2026-08-29)

Pre-existing, found while verifying the mode work:

```
rustc-LLVM ERROR: Error while trying to spill A2 from class AR:
Cannot scavenge register without an emergency spill slot!
error: could not compile `btuuid` (lib)
```

`btuuid 0.1.1` (pulled in by `bt-hci 0.6`) miscompiles on the Xtensa backend
under the `-C force-frame-pointers` that `firmware/.cargo/config.toml` sets
for esp-backtrace. It is not a debuginfo interaction - `debuginfo=2` and
`line-tables-only` both fail - and `cargo check` passes, because the crash is
in codegen.

**Fixed** (2026-08-30) with a per-package profile override:

```toml
[profile.dev.package.btuuid]
opt-level = 0
```

Rustflags cannot be set per package, but `opt-level` can, and it was the
only knob that helped. Measured on `cargo build -p btuuid`, which reproduces
the crash in seconds:

| Override | Result |
|-|-|
| `opt-level = 0` | **builds** |
| `opt-level = 1` / `2` / `3` / `"z"` | crashes |
| `debug-assertions = false` | crashes |
| `debug = 0` | crashes |
| `codegen-units = 1` | crashes |
| `overflow-checks = false` | crashes |

So it is optimization plus the reserved frame pointer, not debuginfo and not
the assertions. Release was never affected - it compiles this crate clean at
`opt-level = "s"`, presumably because `lto = "fat"` moves the codegen - so
the override is dev-only rather than applied to both. The cost is nil:
btuuid is UUID constants and comparisons, and its generics are instantiated
in the crates that call it, at their optimization level rather than its own.

`RUSTFLAGS="" cargo build` also worked and is what verified the mode work
before this fix landed. The reasoning for preferring the override was that
dropping the flag takes esp-backtrace's backtraces with it - **that premise
was wrong on this chip**, and the override is gone. See below.

## A commanded store has to borrow a cadence (2026-08-29)

Caught on review, not from a symptom. `at_expiry` first treated
`sleep_interval_s == 0` the same way in both stored and idle: keep
advertising, because there is nowhere to sleep to. For idle that is right -
nobody asked, so a board with no cadence stays reachable, which is the bench
case and the unconfigured one. For stored it is backwards: `CFG_MODE stored`
would ack, take one 300 s nap (`sleep_cadence` borrows the ceiling for the
command itself), wake into a check, and then sit at ~90 mA forever, having
been explicitly told to go away.

So `Mode::Stored`'s expiry uses `sleep_cadence()` and idle's uses the raw
field. The rule is the asymmetry between a command and a timeout: an
explicit instruction borrows a cadence it was not given; a timeout running
out on a board nobody configured does not.

## BLE modem sleep, ported rather than configured (2026-08-30)

`sleep_mode`/`sleep_clock` in the controller config were never the switch.
Setting them was tried and measured at zero, and reading the crate said why:
the callbacks the controller uses to sleep were `todo!()` in
`vendor/esp-radio/src/ble/btdm.rs`, and `ble_init` ran none of the enabling
sequence. A controller that took the config at its word would have panicked
rather than saved anything.

So it is a port, from ESP-IDF v5.5.3's
`components/bt/controller/esp32c3/bt.c`. That file serves the S3 as well as
the C3 - `components/bt/CMakeLists.txt` maps `CONFIG_IDF_TARGET_ESP32S3` to
`target_name esp32c3` - so it is written against the same `libbtdm_app.a`
the crate links.

What went in:

- **The clock.** `btdm_lpclk_select_src(XTAL)` plus
  `btdm_lpclk_set_div(xtal_mhz)` gives a 1 MHz reference, so one low power
  cycle is one microsecond, carried as `1 << 19` in the fixed point
  ESP-IDF's `RTC_CLK_CAL_FRACT` sets. Both conversions were wrong before:
  the one that existed treated a half-microsecond as a microsecond, out by
  a factor of two, and it was harmless only because nothing called it.
- **Three signatures.** `btdm_sleep_check_duration`, `btdm_lpcycles_2_hus`
  and `btdm_sleep_enter_phase1` take in/out pointers that were declared as
  plain integers. A check that shortens the sleep it is handed could not
  have written the shortened value back.
- **The PHY.** `enter_phase2` drops the reference `ble_init` took, so the
  count reaches zero and the modem really powers down; `exit_phase3` puts
  it back and forgets the guard, because the guard that has to survive to
  `ble_deinit` is the connector's. `ble_deinit` re-takes the reference if
  the controller was asleep when teardown began, or the connector's guard
  would drop the count below zero and panic.
- **The wake path.** A sleeping controller cannot take an HCI packet, so
  `send_hci` posts `btdm_sleep_exit_phase0` to the controller task through
  `r_btdm_vnd_offload_post` and blocks on a semaphore until it has run.
  `ble_deinit` does the same before `btdm_controller_disable`, because a
  sleeping controller cannot be disabled either.

Two callbacks needed nothing: `btdm_sleep_exit_phase1` and `_phase2` are
null in ESP-IDF too, so the table entries are `None` rather than stubs.

Deliberately not ported: the power management locks (no DFS and no light
sleep here, which is also why `enter_phase1` has an empty body) and MAC/BB
power down, which wants deep sleep memory the crate never sets up.
ESP-IDF's `sdk_config_extend_set_pll_track(false)` is in, because it ships
in the same function and the sleep has only ever been validated beside it.

Why the per-event PHY cycling is affordable: `esp-phy` calibrates once per
boot (`PhyState::calibrated` is a global that outlives the connector) and
every later `enable_phy` is `phy_wakeup_init` plus a digital register
restore, which is what ESP-IDF does around its own modem sleep.

`Config::with_modem_sleep(true)` turns it on; it defaults off, matching
ESP-IDF's `BT_CTRL_MODEM_SLEEP` being `default n`. The firmware sets it
unless `--features iso-ble-no-modem-sleep`, which is the control half of
the A/B.

**Unmeasured.** It builds, it links against every symbol it needs, and the
callbacks that run with interrupts off land in IRAM (`nm` shows
`btdm_sleep_check_duration` and both conversions at `0x4037xxxx`). What it
is worth on a meter, and whether a connection survives it, is the next
thing to find out.

The console reports it, and reports the controller's answer rather than the
build's: `ble::modem_sleep_active()` is true only if `low_power_mode_init`
kept the sleep *and* `btdm_controller_get_sleep_mode()` returns mode 1. The
two can disagree - the config byte and the enabling sequence are separate
paths out of the same flag - and a board that armed one without the other
would advertise at full current while looking configured.

## The GPS park was missing one bit (2026-08-31)

Symptom from the bench: the receiver slept through some deep-sleep cycles and
not others, a consistent ~10 mA either way, with nothing in the firmware to
explain the difference.

`UBX-RXM-PMREQ` was going out with `flags = 0x02` - `backup` alone. The
MAX-M10N integration manual, 3.7.4.2:

> The "force" flag must be set in UBX-RXM-PMREQ to enter software standby
> mode.

`force` is bit 2, so the payload wanted `0x06`. The reason it was left clear
is that on u-blox 8 the flag means "back up even though USB is attached",
which is plainly irrelevant to a UART-only board; on M10 it is a precondition
for standby at all.

Three things made it invisible rather than obvious:

- PMREQ is not acknowledged, so a receiver that ignored the request is
  indistinguishable from one that took it.
- The driver sets its own `sleeping` flag when it sends the message, so the
  firmware's view agreed with the firmware.
- It works *sometimes* without the flag, which reads as a race in the park
  path rather than a missing bit.

The observation that would have caught it is now in the park path, and costs
nothing: nothing polls the GPS UART while the board is in standby, so any
bytes in the RX FIFO at the next park came from a receiver that was awake for
the whole wake check. It prints `gps: talking at park - the last park did not
hold`.

Also from that bench run: the receiver's contribution measures ~10 mA
(toggling `gps_sleep` with a meter attached), against the 25-31 mA the power
documents carry as a datasheet estimate. The floor decomposition in
`docs/POWER-S3.md` rests on the larger number.

## A fizzled handshake could cost a whole duty cycle (2026-08-31)

Also from the bench: a phone that tries to connect during an advertising
window sometimes has to wait and try again.

`serve` treated a returned-but-failed `accept()` as an event that consumed
budget like any other - `qprintln`, 200 ms, `continue` - and the loop then
re-checked a window that a 15 s `adv_window_s` had very likely just spent. So
a handshake that fizzled near the end of a window sent the board dark for
`ble_off_s`, or to sleep for a whole cadence, exactly when the phone was
about to retry. Phones fizzle first handshakes routinely; the app's own
reconnect loop exists for it.

`Window::after_connect_attempt` holds the window open for the linger period.
It extends rather than sets, so an attempt early in a ten-minute idle timeout
cannot shorten it - which is the difference from `Window::linger`, and what
the second of the two new tests pins.

Still open, and not fixable from this side: an attempt that is *in flight*
when the window expires. `with_timeout(left, advertiser.accept())` cancels
the accept, and trouble-host reports that as "nobody came" rather than
"someone was halfway in". The controller knows; the host API does not expose
it.

## What a BLE audit turned up after the modem sleep landed (2026-08-31)

Five things, none of which had fired, all of which were one step away.

**Five `todo!()` stubs sat in the table the controller calls.** The osi
struct in the S3 adapter has the same slot order as ESP-IDF's, so
`interrupt_off`/`interrupt_clear` are its `_interrupt_disable`/
`_interrupt_free`, and `esp_hw_power_down`/`_up`/`ets_backup_dma_copy` are
its MAC/BB power-down trio. ESP-IDF installs all five unconditionally; the
MAC/BB three have bodies wrapped in `#if CONFIG_MAC_BB_PD`, so in the
default build they are *empty functions* rather than absent ones -
Espressif does not rely on the controller never calling them. Ours
panicked, which on this firmware is a reset with no message, and modem
sleep is the first thing here that puts the controller through a power
transition at all. They are no-ops now, which for the interrupt pair is
also what `interrupt_on` already was.

**The HCI out collector was smaller than the transport that fills it.**
`Transport::write` builds a `[u8; 259]`; the collector held 256 and pushed
into it with a bare `copy_from_slice`, so an overlong packet was a slice
panic rather than a dropped frame. 259 is the real bound (an HCI command is
1 + 3 + 255); an ACL packet is 1 + 4 + whatever
`le_acl_data_packet_length` the controller reports, and trouble-host
fragments to exactly that - at the usual 251 it lands on 256 with nothing
to spare. Buffer is 259 now, with a check that drops and logs.

**Both spin loops in `send_hci` now yield.** `while !can_send {}` and
`while !PACKET_SENT {}` never gave the CPU up, and what frees a controller
buffer is the controller task. With modem sleep it was worse than a hang:
the wakeup request is held across both loops, so the modem stayed powered
for the duration.

**The GATT write buffer was a round number.** 200 bytes, against a longest
protocol write of 195 (`BULK_DATA_MAX` 192 behind a three byte header) -
five bytes of margin, and `BULK_DATA_MAX`'s own doc comment invites raising
it to fit a 251-byte ATT payload. The buffer is `ble::WRITE_MAX` now,
defined next to the constant it depends on, and a write that does not fit
is refused and logged rather than silently clipped. A clipped chunk fails
as a CRC error at the end of a transfer, which is about as far from the
cause as a symptom can get.

**One more window hole**, the same shape as the fizzled handshake: a
central that connects and then fails `with_attribute_server` fell through
to a `continue` on a possibly spent budget. Held open now.

Checked and found sound, so as not to re-audit them: the HCI read waker
registers before it tests (no missed wakeup), `Transport::read` takes one
packet at a time rather than concatenating into a buffer that
`from_hci_bytes_complete` would then reject, `log_line` drops the oldest
line rather than blocking when nobody is draining, and the duty cycle drops
the connector before the radio.

## Two characteristic sizes that were literals (2026-09-01)

Same class as the write buffer, one layer up, and the second round of the
audit found them by comparing every `#[characteristic]` declaration against
the constant that feeds it.

`bulk` was `heapless::Vec<u8, 200>` against a longest protocol write of 195
(`WRITE_MAX`). That 200 is where the copy buffer's 200 came from, and
tightening only the copy would have made a 196-200 byte write worse than it
was: the attribute layer would accept it, the copy would refuse it, and the
central would hear nothing where it used to get a NAK from the bulk
handler. Sizing the characteristic itself to `WRITE_MAX` puts the rejection
where it belongs - the ATT layer answers an over-length write with an
error code, and the copy guard behind it becomes unreachable rather than
load-bearing.

`log` was `heapless::Vec<u8, 128>` against `link::LOG_MAX`, also 128. Equal
today, and the failure if they ever diverge is not truncation: `notify`
fails on an over-length value, the logger arm ignores the error because a
central that never subscribed is not an error, and every status line stops
reaching the app with nothing said anywhere.

Checked in the same pass and sound, so they do not need re-auditing: the
BLE address (build.rs rejects an override without the static-random bits,
and the eFuse path ORs them in), `radio_busy` set/clear pairing around a
beacon, transfer ownership and the abort ordering out of a session, the
absence of any panicking construct reachable from a BLE write, the idle
hook, `take_remote` termination, and the main task's stack - which is the
linker's CPU0 stack, not a task arena, so `HostResources` on it is fine.

One thing deliberately not copied from ESP-IDF: `IRAM_ATTR` on
`btdm_sleep_exit_phase0`. IDF marks it, but its own phase 0 calls a
semaphore give that is not in IRAM, so the attribute cannot be about
running with the cache off - copying it would spend IRAM for nothing.

Free bench check while measuring modem sleep: the 10 s status line's
`idle N Hz`. Modem sleep trades radio current for CPU work - the PHY's
digital registers are backed up and restored around every sleep - so a
noticeable drop in the idle rate while advertising is that cost showing up,
and it needs no meter.

## Round three: the wake counter, a flash erase per write, and a park race

Past BLE now, into what BLE reaches.

**The wake counter could print garbage exactly once**, on the first wake
after a cold boot - and it is the number the console tells a reader to
trust first, because a board that sleeps and a board that resets in a loop
produce the same banner otherwise. `note_sleep` stamps the magic word with
`set(get())`, whose comment shows the author already reasoned about
un-initialized RTC RAM for the settings words; it writes six of them and
not the two instrumentation words beside them. So the first sleep after a
cold boot left `WAKE_COUNT` as whatever was in the die while the magic word
now said the block was ours. `set` zeroes both when it stamps.

**Every accepted settings write erased a sector**, whether or not the value
moved. `session::apply` returns `save: true` on the write, not on a change,
which is the right layer for it to be ignorant - but the firmware then
spent an erase and something like 40 ms with interrupts off, inside a BLE
session, on an app pushing a value the board already had. `save_settings`
now reads the record back and skips an identical one. The record is
deterministic - fixed fields and a crc, no timestamps - so the comparison
is exact rather than a heuristic, and a read costs nothing next to an
erase.

**A park that runs long can be slept over.** `enter_deep_sleep` waits
`tx_worst_case_ms + 500` for `SLEEP_READY`, and that signal comes at the
*end* of a sequence that flushes and unmounts the card, takes the receiver
into backup, sleeps the radio and blanks the panel. The budget is sized for
the longest single item, a transmit in flight; a card that stalls on wear
levelling is enough to expire it, and then the sleep happens over whatever
the hardware task had not reached. If that is the GPS park, the receiver
tracks through the whole interval at ~10 mA - the same symptom the missing
`force` flag produced, from a different cause, which is worth knowing
before the meter says the flag fix did not work.

Left alone at the time, on the grounds that nobody had demonstrated the race
and the constant sat on the path being measured. Closed on 2026-09-01, after
the `force` flag fix had removed the other cause of the same symptom and the
symptom was still worth protecting against. Two changes, and the ordering is
the one that matters:

- **The card goes last.** `PrepareSleep` used to flush and unmount it first,
  on the reasoning that it is the only item holding data a reset destroys.
  True, but it is also the only item that can stall arbitrarily, and putting
  it in front meant a stall was paid for by whatever came after it - the
  receiver at ~10 mA, the radio at 5.7, the panel. The receiver, radio and
  panel are all bounded work (a couple of UART bytes, an SPI command, an I2C
  frame), so they go first now and the card takes the risk it creates. Under
  a timeout the loss is up to five buffered fixes instead of a whole
  interval of sleep current.
- **The budget is `tx_worst_case_ms + 1500`**, up from `+ 500`. A timeout
  rather than a delay, so the ordinary path does not pay for it.

What can still expire it is a hardware loop inside a transmit that outran
the budget, and that one reaches none of the sequence - which is what the
expiry message now says instead of blaming the radio.

Checked and sound this round: `Storage::write` really does read-modify-
erase-write per sector, so the settings record cannot rot into a
bit-ANDed mess; the OTA sink stages sector-aligned, refuses a slot that is
running or unknown, refuses an image larger than the slot, and re-derives
the target before activating; and the image crc is checked in
`Transfer::end` *before* `sink.finish()` activates anything.

## Following the rx counter out to the app (2026-09-01)

The counter itself is sound end to end: `rx_count` increments on every frame
`Node::poll` delivers, rides the telemetry struct into bytes 6..10 of the
16-byte blob, and comes back out in the GUI through the *same* `midair-proto`
crate - the app depends on this repo's `proto/` by path, so the two cannot
drift apart the way two hand-written layouts would. Both BLE backends
subscribe to the characteristic; the desktop one only ever *reads* settings
and radio config.

**But `telemetry` and `position` are declared `read` and were never `set`.**
The notifier pushed both and stopped there, so the attribute table still held
the zeros it was built with, and a central that read instead of subscribing
got those. That is worse than an empty answer because an all-zero telemetry
decodes as a plausible one: `secs_since_rx` of 0 renders as "just heard from",
not as "never", which is what 0xFFFF means. The remote and node-ping path
already did `set` first and says why in a comment; this is the same fix in
the two places that had been missed. The table is built once for the life of
the board, so the value written also survives into the next BLE window.

Two things that make the counter *look* broken and are not:

- `last_rssi` and `last_rx_ms` are stamped for any packet that passes the
  hardware CRC, before the frame is parsed - while `rx_count` only counts
  what survives the malformed, own-echo and duplicate filters. So "Last RX:
  2 s ago, RSSI -80" beside "RX: 0" is a repeater echoing this node's own
  beacons back at it, which is the drop counters' job to explain and the
  reason the verbose line prints them.
- The counter is a plain local, not RTC-persistent, so it restarts at 0 on
  every wake of a duty-cycled board. The app clears its telemetry on connect,
  so both ends agree it is a per-wake count rather than a lifetime one.

**Do not run `cargo fmt` here.** The committed formatting is not what the
rustfmt on PATH produces - it rewrites 13 files, reordering imports and
rewrapping expressions in crates nothing touched. Format by hand, matching
the surrounding code.

## force-frame-pointers was never needed on Xtensa (2026-09-01)

The same LLVM crash came back, this time taking the whole binary rather than
one dependency:

```
rustc-LLVM ERROR: Error while trying to spill A6 from class AR:
Cannot scavenge register without an emergency spill slot!
error: could not compile `wio-s3-gps` (bin "wio-s3-gps")
```

The btuuid trick has no equivalent here. The failing crate is the firmware
itself, and with `lto = "fat"` its rustc invocation is where the whole
program gets codegened, so the only per-package lever - `opt-level = 0` -
would deoptimize all of it.

The premise behind that trick turned out to be false anyway. The flag was
kept because esp-backtrace was believed to need it. It does not, on this
chip: **`force-frame-pointers` is a RISC-V requirement**, and this is an
Xtensa part. esp-backtrace's own README says so, and its `build.rs` only
inspects the flag when the chip is not Xtensa:

```rust
if !chip.is_xtensa()
    && !std::env::var("CARGO_ENCODED_RUSTFLAGS")
        .unwrap_or_default()
        .contains("force-frame-pointers")
```

`src/xtensa.rs` walks the windowed-ABI spill area instead - it reads the
caller's stack pointer out of `sp - 12` after forcing a register-window
spill with `rotw`, which the hardware ABI maintains whether or not a frame
pointer was reserved.

The flag was copied in with the S3 crate scaffold from the esp-hal template,
which is right for the RISC-V parts and for the retired ESP32-C6 firmware
this was ported from. It has bought nothing since.

**Fixed** by deleting it from `firmware/.cargo/config.toml`. Both profiles
build, backtraces are unaffected, and `[profile.dev.package.btuuid]` went
with it - verified in a clean target dir that btuuid now compiles at
`opt-level = "s"`, since the frame pointer was the whole cause of its crash
too.

Worth remembering as a shape: an LLVM register-scavenger crash blamed on a
dependency is really a crash caused by a *flag*, and the flag is worth
re-reading before the crate is.

---

# Board names: where the label lives, and the three ceilings that shaped it

Boards had one hardcoded BLE name (`GPS-S3`), which made a bench with two of
them a guessing game. They now advertise `ws3gps-<label>`, with an unnamed
board falling back to `ws3gps-<xxxx>` from the tail of its BLE address.

Three separate ceilings decided the sizes, and only one of them is the air:

- **The GAP device name is 22 bytes.** trouble-host builds it into a fixed
  `String<22>` in `gap.rs` and `push_str` *fails the server build* past it -
  which the firmware turns into `.expect("gatt server")`, i.e. a panic at
  boot on a board with a long name. This is the binding constraint: 22 less
  `ws3gps-` leaves 15 bytes of label.
- **The scan response is 29 bytes of name** (31 less the AD header). Not
  binding at 22, but it is why the name goes in the scan response rather
  than the advertisement, which is already spending 21 of its 31 bytes on
  the flags and the 128-bit service UUID.
- **The config characteristic was 8 bytes**, so `[id, len, value]` had six
  value bytes and no name fit. Grown to `2 + NAME_LABEL_MAX`. Growing a
  characteristic only raises the write length the attribute layer accepts,
  so every existing 3-6 byte config write is unaffected.

The stored field is 16 bytes for a 15-byte label. That keeps the flash
record a multiple of the write word *and* guarantees a full-length label is
still followed by a zero, so the reader that stops at the padding always has
padding to stop at.

**The label is stored with the settings, not with the radio config.** The
tempting home was `RADIO.CFG`, next to the LoRa `address` that already
distinguishes nodes - a fleet push would carry it. That is wrong here for a
timing reason: a wake check advertises before anything has mounted the card,
so a name on the card is a name a sleeping board cannot tell anyone. It goes
in `Stored` (RTC RAM, mirrored to `nvs`, record version 6), which is exactly
the set of things that survive both a deep sleep and a flat cell.

**Three surfaces, three different update speeds**, which is worth knowing
before chasing a "stale name" bug:

| Surface | Catches up |
|-|-|
| scan response | next advertising window - the one on the air was handed to the controller before the write landed |
| name characteristic | immediately, on the connection that did the renaming |
| GAP `0x2A00` | next boot - the attribute table is built once per power cycle and cannot be rebuilt (its `StaticCell`s panic on a second `Server::new_with_config`) |

The scan response is therefore built inside the serve loop rather than once
at the top of `serve`, which is the whole cost of making a rename visible
without a reboot.

The ack cannot carry the name - `ACK_MAX_LEN` is 6, so four value bytes -
so `0x19` acks the stored *length* and the characteristic carries the truth.

The prefix is a firmware constant rather than part of the label on purpose:
the app filters scans by service UUID, so the prefix buys nothing there, but
it means a board can never be named something that a generic scanner
(nRF Connect, a phone's settings screen) cannot be searched for.

## Config backup in nvs: what came back from the E5, and what changed

The two-MCU board kept the config in flash page 122 (`wio/src/cfgstore.rs`);
the S3 port dropped it and left the card as the only store, so a card-less
board came back on defaults and lost its address. `RADIO.example.toml` and
the README still described the backup, which is how the gap surfaced.

**Where.** The `nvs` partition, one sector past the settings record
(`CONFIG_AT = SECTOR`). Not a partition of its own: `partitions.csv` is
flashed over USB, so a new partition would be a reflash rather than an OTA
and would strand every board already in the field. `nvs` is 0x4000, and the
two records need 0x1410 of it.

Its own sector, not a neighboring offset. `embedded_storage::Storage::write`
is a read-modify-erase-write of the whole 4 KiB sector, so sharing one would
put every config push through an erase of the settings record - and a power
loss mid-write would take the duty cycle with it.

**The record** is `proto/src/cfgstore.rs`: magic, version, text length,
crc32 of the text, then the text. The text and not the parsed form, so a
firmware that later learns a key still has the bytes somebody wrote.

Crc *in front of* the text, and the record goes down in **one** write. The
E5's trick - program the text, then the header, so an interrupted write
leaves a header that does not vouch for what is behind it - buys nothing
here, because `Storage::write` erases the whole sector around whatever it is
handed: two writes are two erases, and the first has already destroyed the
record being replaced. So the crc is what makes an interrupted write safe,
and the single write halves both the wear and the time spent with interrupts
off (an erase is tens of ms, and the BLE side is usually mid-something).
The cost is assembling the record in a `RECORD_MAX` stack buffer, next to
the 4 KiB sector buffer esp-storage puts on the same stack anyway.

**No-op writes are skipped**, because every cold boot with a card asks for a
save so the backup keeps up with a card edited on a computer. The comparison
is byte-for-byte against the region in 64-byte chunks rather than a header
compare: an interrupted write leaves a header that still matches the config
it was replacing, and re-pushing that config is exactly when a header-only
compare would report success over a record that does not load.

**Empty text is a record**, deliberately. An empty config file is valid and
means "every setting at its default", so refusing to store one would have a
board restore the previous config at the next boot - silently undoing a push.
`Header::for_text(b"")` is `Some`; only a missing magic means nothing stored.

**Precedence** is card, then flash, then defaults - the E5's order, for the
E5's reason: pulling the card to edit `RADIO.CFG` has to do what it looks
like. An *invalid* `RADIO.CFG` now falls through to the backup rather than to
defaults, which is new; defaults would throw away the address as well.

A reflash keeps the backup: the runner's erase list is `--erase-parts
otadata` and nothing else.


## Frequency hopping (2026-09-03)

The modulation note above said hopping was ruled out because a node with
no fix has no clock. It has one now: the frames.

**The plan.** `hop_channels` x `hop_step_khz` about `frequency_hz`, one
slot of `hop_dwell_ms` per channel; defaults 50 x 500 kHz around 915 MHz,
1 s, which fills 902-928. Slot `s` uses entry `s mod n` of a permutation
reshuffled every cycle of `n` slots from the cycle number
(`hop::permutation`, Fisher-Yates over a xorshift seeded from a mixed
cycle count). Not a fixed order: a node that beacons every k-th slot would
visit only `n/gcd(k,n)` channels of a fixed order - five of fifty at the old
20 s interval - and two free-running nodes at a fixed offset would never
coincide. With a fresh order per cycle both are one slot in n.

**The clock** (`hop::Clock`) is a local ms origin plus the slot number that
began there; slots are 20-bit and wrap. Three sources, ranked by stratum:
GPS time of day from an RMC *with a fix* (stratum 0; a receiver still
searching reports a time too, but its error is the unknown range to the
satellites), a heard frame (adopt if lower stratum, or equal from a lower
address, and sit one below), or nothing (15, free-running from a seed of
the address). A node on its own GPS never adopts. Stratum ages +1 per 10
min unrefreshed so a network that lost GPS reorganizes around the lowest
address. The lowest-address rule is also what makes a Free-vs-Free pair
stable: the lower one keeps its own clock, the higher one follows and
re-adopts on every frame.

**The sync word** is 4 bytes behind the 3-byte header, flagged by bit 7 of
the hops byte: 20-bit slot, 4-bit stratum, 8-bit phase in 1/256ths of the
dwell. It describes the transmission, so a repeater re-stamps it with its
own clock; the origin's src/id are untouched and the dedup still works. At
SF12/BW500 it rides in the same symbol block as the 13-byte beacon, so the
default frame is still 288.8 ms; at SF7/BW125 it costs 5 ms. `FRAME_MAX`
grew 35 -> 39 and the slowest-frame test values moved with it.

**Receive timing.** The poll timestamps RxDone; TX start = that minus
`time_on_air_us(len)`. Error is the 10 ms poll period plus SPI, against a
100 ms guard. GPS phase is the RMC parse instant: ~200-300 ms behind the
true second (9600 baud, GGA ahead of RMC) but the same on every board, and
only consistency matters. The one thing that would break it is boards with
different sentence sets or baud rates.

**Why the beacon is two-stage in main.rs.** `send()` waiting for the
window would hold the hardware loop - and so the receiver - for up to a
slot, half the time at a 1 s interval. So the loop asks
`tx_window_start()` for the planned instant when the interval runs out
(`beacon_at`), keeps polling, and calls `send()` when it arrives. `send()`
re-checks the window and waits only if the clock moved in between (a
better frame heard), which is rare and bounded by a slot. Repeats get the
same treatment at queue time. `tx_worst_case_ms()` adds a dwell for the
sleep path's parking budget.

**Holding a hop for a frame.** PREAMBLE_DETECTED / HEADER_VALID / HEADER_ERR
are now in the IRQ mask. A preamble marks `rx_busy_since`; `hop_tick` will
not retune while it is fresh (bounded by min(longest frame, one dwell), a
preamble can be noise), and the beacon and repeat gates in main.rs will
not key up over it. That is also a listen-before-talk the single-channel
mode never had - the gates apply either way, the hold only when hopping.
The non-RxDone IRQ path now clears only the bits it read rather than ALL:
an RxDone landing between the read and the clear was being wiped with the
packet still in the buffer, at about the poll period over the frame time.

**The join cost is the real limit.** A blind receiver on one channel sees
the network's transmissions with density (traffic)/n, whatever it does, so
expected acquisition = n x interval / nodes_transmitting: 500 s at 20 s and
two nodes, 50 s at 1 s and one node. GPS nodes never pay it. The base
station on a desk does, every boot - which is the case to remember. Two
ways out are not implemented: an RSSI sweep (50 channels in ~50 ms, catches
strong signals only) and the phone's GPS time over BLE.

**Read-back blob** grew 28 -> 32 bytes with the plan at bytes 27-31. Byte
27 was reserved zero, so `decode` accepts a 28-byte blob from an older
board as hopping off (`RADIO_CONFIG_LEN_V1`), which is the truth about that
board; no version bump. Telemetry grew 16 -> 18 with a hop byte
(`TELEM_HOP_ON` | stratum) and the channel index.

**Air format is a flag day again**: hoppers and non-hoppers cannot hear
each other except by the 1/n coincidence, and the sync flag in the hops
byte would read as hops_left >= 128 on the old firmware.

## Two intervals: a position every second, a ping every five (2026-09-03)

`interval_s` is the position period with a fix (default 20 -> 1) and a new
`ping_interval_s` (default 5) paces the no-fix ping. `interval_s = 0` still
silences the node outright, pings included - a card that says 0 has always
meant silent, and a ping continuing behind it would surprise whoever wrote
it; `ping_interval_s = 0` turns off only the ping.

**Which interval applies is decided per pass by the fix state**, not fixed
at the last transmission: a fix gained goes out as soon as the beacon
interval allows rather than after the slower ping interval it was last
scheduled under. The loop measures from `last_beacon`, the `(start, end)`
the driver timed, and asks `beacon_due(last, interval_ms, now)`.

**Hopping, the interval is a count of slots** (`Clock::slots_for`,
`interval_elapsed`): every second at a 1 s dwell is every slot, decided at
the slot boundary, then a random start in that slot's window. Measured as
time from the last transmit plus jitter, a 1 s interval landed past the
window's random target about half the time and skipped a slot, for an
effective rate near 1.5 s. Single-channel, the interval runs from the end
of the last transmit as before, with the jitter now capped at half the
interval (was a flat 2 s, which would have doubled a 1 s interval).

**One transmission per slot, whatever it carries**: `tx_window_start` moves
to the next slot when the last transmission (beacon or repeat) started in
this one. A beacon and a repeat sharing a slot would be two visits' worth
of air on one channel.

**Read-back blob 32 -> 34** with the ping interval at bytes 32-33; a blob
without it reads as pinging on the beacon interval, which is what that
firmware did. 34 bytes crossed the array `Default` limit (32) that
trouble's `gatt_service` macro relies on, so `radio_config` became a
`heapless::Vec<u8, RADIO_CONFIG_LEN>` like the log and name
characteristics already were.

**Power**: a 288 ms beacon every second is ~37 mA average, against ~1.3 mA
at the old 20 s - the largest config-driven load on the board now, and
`docs/POWER.md` says so. Two nodes at 1 s collide on most slots (each
frame is 29% of the window); interval in seconds ~= transmitting nodes
keeps the shared air near 30%.

## Listening mode, ble_on_s, and an idle timeout that is off by default (2026-09-06)

Three app-driven changes to the modes, made together because they touch the
same three files (`proto/src/ble.rs`, `proto/src/session.rs`, `main.rs`).

**Listening is a mode, not a role.** `role = rx_only` in `RADIO.CFG` would
also keep a node off the air, but a role is what a card says a node *is*,
and the node beside the phone is a tracker that has been told to be quiet
for now - from the app, over BLE, and back to tracking the same way. So it
is `Mode::Listening` (wire 3) with `Mode::tracks()` true and a new
`Mode::transmits()` false, and the only firmware difference from tracking is
the `live.transmits()` term on the beacon gate and on the repeat-forwarding
gate. Everything else - the boot raise, `Request::Mode`, the standby flag -
matches on `tracks()`. It persists like tracking: the node in the pocket
must not go dark on the phone over a brownout. Its budget never bites
(`at_expiry` is always `Advertise`), because the whole point is that the
phone can come back whenever it likes.

**`ble_on_s` is the tracker's window.** `budget_s()` handed Tracking the
advertising window, so a bench setting of `adv-window 1` to watch a wake
cycle go by also gave a tracker one-second windows nobody could connect in.
`CFG_BLE_ON_S` (0x1A) is clamped like the window (1-60, 0 = default 15),
appended to the settings blob (version 6, 28 bytes) and to the flash record
after the name (version 7; `V6_CRC_AT` is where version 6 ended). The
`[power]` section of the card takes `ble_on_s` too.

**Idle off by default was a semantic change to an existing id.** `0x18 = 0`
used to read as "the default, 600 s" and now reads as "never". The version
6 record needs no migration: a stored 0 was a board nobody had set a
timeout on, and that board now stays idle, which is the behavior asked for.
`at_expiry` for Idle is `Advertise` when either the timeout or the cadence
is 0. `IDLE_TIMEOUT_DEFAULT_S` stays as the value an app offers when the
timeout is switched on.

## The radio audit: turns, a notifier that skipped, and a FIFO that overflowed (2026-09-07)

The complaint was that hopping made both radios erratic, and the suspicion
was a clock message colliding with the beacons. There is no clock message
- the sync word is four bytes inside every frame - but the audit found the
real causes, and a simulator (`tools/radio_sim.py`, standard library, hop
clock ported line for line and self-tested against `proto`'s own vectors)
put numbers on each. `docs/RADIO-AUDIT.md` is the report. What is worth
keeping in mind from it:

- **Random starts in a shared window collide far more than intuition
  says.** Two nodes at 1 s with 289 ms frames in a 512 ms start range
  overlapped in 20% of slots, and the listen-before-talk gate cannot help
  when both commit within the ~60 ms it takes a preamble to be seen. On a
  two-node link every overlap loses both frames, since each node is
  transmitting through the other's. The schedule is turns by address now
  (`Plan::sub_slots`, `start_range_for`, `Clock::turn_due`), slot of the
  interval first, then turn of the slot - slot first because an unsynced
  listener joins on a coincidence with a *busy* slot, and bunching a fleet
  into one slot of five halves its chances. Half of each turn's slack is a
  guard, because two GPS nodes disagree by their sentence-latency spread.
- **Capacity is a hard number.** Two turns a slot at SF12: four trackers
  at 1 Hz are over it (4 x 289 > 800 ms) under any schedule, and a shared
  turn is worse than random - 0% between the sharing pair - so the
  firmware warns when it hears a node in its own turn and the app shows
  `turns x interval_s`. There is no adaptive fallback on purpose: the
  sharing pair cannot hear each other, so nothing they could measure
  converges.
- **`continue` past a busy flag is a phase-dependent drop.** The notifier
  skipped its tick while the radio transmitted; at a 29% transmit duty
  that was 23% of position updates on average and anywhere from 0% to 57%
  per session, invisible in any counter. It waits now. The supply-noise
  argument for the gate is thin either way - the controller keeps sending
  connection events through the transmit regardless of what the notifier
  does - but waiting costs nothing.
- **A 128-byte FIFO against a 289 ms transmit.** The loop that drained
  the GPS UART is held for every transmit, and the burst of a second's
  sentences (150 bytes) overflowed the FIFO one time in seven at 1 Hz.
  Bytes go through an async pump task on the BLE core into a 512-byte
  `Pipe` now; a full pipe drops and flags rather than blocking, since
  blocking would only move the loss into the FIFO.
- **Late timestamps must not set a clock.** A pass after a transmit or a
  card flush timestamped a GPS mark or a packet end with the pass time,
  and a stratum-0 node handed the resulting 250-450 ms error to every
  follower for a second. `LATE_PASS_MS` / `LATE_POLL_MS` (40 ms) gate the
  GPS mark and the frame offer; an unsynced clock still takes a late
  frame, since a rough slot finds the network and the next well-timed
  frame corrects it.
- **The oscillator lead is a per-stratum error.** From `STDBY_RC` the
  SX1262 waits `tcxo_startup_ms` (10 ms) before every transmit and every
  receive entry, and the sync word was stamped before it. Listening nodes
  now park in `STDBY_XOSC` (fallback mode too) and the stamp is for the
  RF start; a transmit-only node keeps the cold standby, since its radio
  idles for whole seconds, and compensates the stamp instead.
- **Two-stage receive hold.** A preamble detection held the hop and the
  transmit gate for 452 ms; the detector fires on noise. Until a valid
  header lands the hold is the header time (`time_on_air_us(0)`, which is
  exactly preamble plus header) plus a poll period.
- **Second core.** `esp_rtos::start_second_core` with a second
  `esp_rtos::embassy::Executor` runs `hardware_task`; the BLE host, USB and
  the GPS pump stay on core 0 with the controller. Three things to know:
  esp-hal's `I2c` is not `Send` (a raw pointer, not an affinity), so the
  panel crosses in a wrapper; a Rust 2021 closure that destructures the
  wrapper captures its fields individually and the wrapper's `Send` covers
  none of them - consume it through a method (`into_inner`); and
  esp-storage's `multicore_auto_park` parks the other core before taking
  its own lock, which is the deadlock-free order, so flash writes from
  either core are safe but freeze the other for tens of milliseconds.
  `--no-default-features` puts everything back on one core.
- **Untested on hardware.** All of it: `cargo check` passes in both
  configurations and the proto and app tests pass, but no board has run
  this. The bench checklist is at the end of the audit.

## The hop default is one channel, and 0 is not the same as 1

`hop_channels` carries two decisions in one number, which is the trap:

- `0` - no plan, and so no slot clock, no turns, no sync word. Beacons go
  out on the node's own interval with random jitter. Two nodes at 1 Hz
  then collide in ~20% of slots and lose both frames.
- `1` - the default. Full clock, turns by address, sync word, one carrier.
- `50` - the above plus frequency hopping.

Chose `1` because at SF12/BW500 the band lets a single carrier be held
with no dwell or duty limit, so hopping bought no air time, and the 0.4 s
dwell caps time on air hard enough that it bought no link budget either
(best legal hopped plan is ~1 dB better; the 3 dB modes are illegal both
ways). It cost a fixless node a 61 s join and ~25% of frames thereafter,
from re-anchor disagreements putting a follower on the wrong channel.

Two things that made this non-obvious:

- `Radio::hopping()` returned true for any plan, so a one-channel default
  would have read as "hopping" everywhere. Renamed `scheduled()`.
- `proto/examples/hop_vectors.rs` built its vectors from
  `RadioConfig::default()`. Left alone, the simulator's selftest would
  have started checking the permutation against a column of zeroes and
  still passed. It now pins 50 channels explicitly.

## The state space, walked (2026-09-08)

"Serious BLE and LoRa problems" with no symptom attached, and a request
for exhaustive state space testing of everything around the radios and
whatever can interrupt them. The simulator in `tools/radio_sim.py`
already covers the timing; what nothing covered was the discrete
interleavings - a mode written while the loop is inside a transmit, a
sleep commanded between the park and the chip going down, a request
landing on a full queue. Those need every ordering, not a scenario.

**The harness.** `explore/` is `midair-explore`: a `Machine` trait
(initial states, enabled events, step, invariants), a breadth-first walk
that stops at the first violation with the shortest trace to it,
liveness (`assert_always_reachable`: from every state some predicate
state is reachable), coverage (`assert_some`: an invariant cannot pass
vacuously), and a mermaid gantt of any trace. Zero dependencies, host
only, a dev-dependency of `proto/` and of the app. Two things learned
building it:

- A transition-check violation whose target state was already reached by
  another path printed that path's trace, which does not end in the
  offending step. The violation now carries the step (event and
  resulting state) separately and the report appends it.
- A `Machine` needs a two-valued clock, not a counter. The composed
  firmware model drives `Serve` with `now = 0` for every event except
  the budget's expiry, which uses `now = ends_ms`; the deadline is then
  one of a handful of values the settings decide, and the state is
  finite. A millisecond counter in the state is an infinite space.

**The machines.** The policy that was inline in `main.rs` is now a value
in `proto/` that the firmware drives, and the models are built from the
same values - so a new request, mode or effect that is not handled fails
to compile before it fails to explore:

- `session::Serve` is the serve loop: `pass` (re-budget on a moved mode,
  then advertise / sleep / drop the modem), `on_accept` (connect, fizzled
  handshake, expiry, nap, moved mode - and the wake-check promotion),
  `on_session_end`. `session::dispatch` is what a config write sets in
  motion (a request, a nap, the mode signal).
- `posture::Posture` is what the hardware task has up (mode, radio, GPS,
  card) and `on(Request)` the `Effect`s the task carries out; `consistent`
  is the rule. `posture::Requests` replaces the four-deep channel: one
  slot per kind, a newer one replaces an older, a fixed drain order (mode,
  overrides, config, park, reboot).
- `rxgate::RxGate` is the receiver's hold: preamble held a header time, a
  header a whole frame, a hop refused inside the hold and for at most a
  slot.

**What the walk found**, and what changed for it (the firmware still has
never been flashed with any of this - a bench pass is owed):

1. The GPS was polled while the *radio* was up (`if !standby`). A
   `CFG_GPS_SLEEP=0` with the radio in standby woke a receiver that
   nothing drained: ~10 mA and no position, and the settings retry never
   ran. Polling is gated on the receiver's own state now.
2. A mode commanded on top of an override flag ignored the flag:
   `CFG_GPS_SLEEP=1`, then `CFG_MODE tracking`, and the app showed "GPS
   in backup" beside a receiver that was acquiring. `Posture::on(Mode)`
   lands on the flags the way the boot path always did; the overrides are
   no-ops outside a tracking posture (the flag is stored and honored when
   tracking is next commanded).
3. A request could be dropped on a full queue - `Channel<_, 4>` with
   `try_send` - and the droppable one could be the park before a deep
   sleep, which then happened over a radio in continuous receive.
   `Requests` cannot overflow.
4. A `CFG_MODE tracking` over the console in the window between the park
   and the chip going down raised the receiver and the radio for the
   sleep to happen over. A parked posture ignores everything but another
   park or a reboot.
5. The one-channel default retuned at every slot boundary: `hop_tick`
   took the receiver through standby to the frequency it was already on,
   a millisecond deaf a second, and a preamble landing in it was a frame
   lost. `Plan::retunes` says whether the carrier changes; the boundary
   is only noted when it does not.
6. `Clock::tx_start` planned "the next slot" on a multi-slot interval,
   which is another node's slot, where `turn_due` says nothing is owed -
   so the plan was dropped and the beacon waited a whole interval. Any
   pass that came late at the start of the node's slot did this. It now
   plans the node's own next slot; a test walks every address, interval
   and phase.
7. A valid header seen after a stale preamble's hold had lapsed inherited
   the stale start, so its hold could already be spent. Reachable at fast
   modulations where preamble and header land in one poll. `RxGate`
   starts a fresh hold.
8. A config push during a wake check wrote to a card that was never
   mounted. The apply mounts it first.
9. Repeats were not gated on a pending sleep the way beacons are.

**What the walk says is sound**, over 692,936 board states and 9.8
million transitions (every persisted mode and flag set, bench and
deployed settings): a sleeping board is always fully parked, the hardware
is in the mode the settings report once the loop has drained, a listening
node never transmits, nothing transmits into a transfer or over a pending
sleep, and from every state the board can be advertised, tracked and
slept again. The receive gate over every interrupt at every phase of a
hold (18,828 states), and the roster over every record/take/replay/age
sequence (471 states for three nodes, 31,974 from a full table with a
ninth), likewise.

**Roster tie-break.** With every node equally old the roster evicts the
first slot and the model's ledger evicted the lowest address; both are
"the quietest". The ledger follows the roster's choice and asserts only
that the evicted node was among the quietest. `expire` uses `>= TTL_MS`
and `newest_position` uses `> TTL_MS`; a one-millisecond disagreement,
left alone.

**Runtime.** The composed model is ~20 s; the rest is under a second. If
it grows past a minute, the levers are the write set (nine writes on two
transports is most of the branching) and the initial states (24).

## The system audit, worked (2026-09-08)

Fourteen items, the choices that were not obvious:

- **The beacon planner is in `proto`, and the radio exposes its clock and
  plan for it.** `Sx1262Driver::schedule()` hands out `(&mut Clock,
  &Plan)` so `beacon::Planner::pass` can be a pure function of the loop's
  inputs and walked on the host. The alternative - keeping `beacon_due`,
  `tx_window_start` and `tx_wait_ms` on the driver - left the "keep a plan
  across other nodes' slots" rule in the loop, which is exactly the rule
  the first walk found broken. The driver keeps `repeat_start` for the
  repeat path, which plans a one-off into the same turns.
- **The hop is never an `Option`, and a `configured` flag decides whether
  the clock survives a re-init.** `Sx1262Driver::new` builds a clock from
  the default plan so `hop` has a value before `init`; that clock is seeded
  from nothing and anchored at zero, so `init` only keeps the existing clock
  when a previous `init` made it - a config push, a standby or a brownout
  keep the network's time, the first init does not keep the placeholder.
- **One command channel of four, and `SLEEP_ASKED` is never cleared.**
  Every `SleepNow` ends in `enter_deep_sleep`, which is a reset, so there
  is no path on which the flag is stale. The old cell-plus-signal pair
  needed four helpers to keep coherent because it could be cleared on one
  side and read on the other. A full queue is logged rather than blocked:
  the loop drains it inside a millisecond of any wait ending, so a full one
  means the loop is inside a park and the chip's RAM is about to go.
- **The park miss is counted on the second timeout, not the first.** A
  first expiry is most often the card mid-flush, and a second budget is
  what it needs; counting it would make `parks_missed` a card statistic.
  The count is stamped with the settings (`set(get())`) for the reason the
  wake counter is: a value in front of no magic word reads as garbage on
  the next boot.
- **The knob table indexes the RTC array by `Knob as usize`.** The five
  durations live in one `[AtomicU32; KNOBS.len()]` rather than five named
  statics, so a sixth knob is one row in `session.rs` and nothing in
  `settings.rs`.
- **`u64` time and no `due()`.** Every `wrapping_add`/`wrapping_sub` pair
  in the loop, the card driver and the blinkers was a 49-day wrap the
  hardware would never reach but every reader had to reason about.
  `embassy_time::Instant::as_millis()` is already 64-bit.
- **The vendored crate as a patch.** `vendor/esp-radio-patch.sh make`
  diffs the registry copy against the tree; `check` applies the patch to a
  fresh registry copy and compares. The registry's own bookkeeping files
  (`.cargo-ok`, `.cargo_vcs_info.json`, `Cargo.toml.orig`) are deleted
  from the staging copy rather than excluded from the diff, so the patch
  header carries no `--exclude` noise and applies with plain `patch -p1`.
- **Time in the beacon model is a tick of 50 ms over two and a half
  intervals**, and the "never twice a slot" check needs only the last
  sent slot since slots are monotonic. The first draft kept four sent
  slots and a fixed 60-tick horizon; at a three-slot interval two beacons
  did not fit and the coverage assertion caught it.
- **The tools are `board-*`.** The `wio_` prefix named the WIO-E5 the old
  board talked to; nothing here talks to one.

## The second core's stack guard at boot (2026-09-08)

The first flash of the split firmware panicked with a write to the stack
guard on the app core, inside a `memcpy` under the executor's spawn, before
the hardware loop printed anything; the main core then faulted in the BLE
controller's `r_rwip_init`, which was the overflow having run into the
controller's memory. Cause: `Hardware::boot` was an `async fn` taking the
peripherals and returning `Self`, so its future held the state twice (the
arguments and the value under construction) beside the task future's own
`hw`, and embassy builds a task's future on the spawning stack and then
memcpy's it into the arena - so the whole three-fold state crossed a
32 KiB stack that also held the `carried` tuple of peripherals. Fix: a
synchronous `Hardware::new` returning the struct literal, `boot(&mut self)`
for the effects, and a 48 KiB stack. The boot line prints
`size_of::<Hardware>()` so the margin can be read off the console. Rule:
never write an `async fn` that owns a large value and returns it; build
it synchronously and borrow it in the async part.

## RTC RAM outlives a flash erase (2026-09-09)

Reported as "neither `espflash erase-parts nvs` nor `espflash erase-flash`
erase the name". The settings sit in RTC fast RAM behind a magic word and
`settings::restore` read flash only when the word was absent - and the word
is absent only after a power cycle. Every other reset (EN, a panic, the
reset espflash issues after a flash or an erase) keeps RTC RAM, so the
erased board came up on its copy and the first save wrote the name back
into the flash that had just been cleared.

Choice: the copy is trusted only when the wake cause says deep-sleep timer.
Any other boot reads flash and takes what it finds; nothing found drops the
copy. Nothing that matters is lost - every setting that decides reachability
is mirrored to flash on the write that changes it, and the live-only state
(idle by promotion, the notify interval) is what a reset should discard.
`is_cold()` is private now; the boot prints which of the three happened.

`board-wipe` (`link::usb::WIPE`) is the reset without a reflash: RTC copy
dropped, settings record and config backup written as `0xFF`, restart.
Written as erased bytes rather than erased sectors because `write` is the
one operation the partition region offers and `0xFF` is what both loaders
refuse by magic word. The card is deliberately untouched: `RADIO.CFG` is
the user's file and the boot reads it as at any cold boot, `[power]`
included, so the tool says so.

## The link's own RSSI (2026-09-09)

A connected board stops advertising, so no scan on the phone can measure
it; the one reading there is the controller's `HCI Read RSSI` for the
connection. trouble-host 0.5 has `Connection::rssi(&stack)`, which needs
the `Stack` threaded into `serve`/`gatt_session` and a
`ControllerCmdSync<ReadRssi>` bound (`ExternalController` implements every
sync command). Read once per notify tick, appended to telemetry as a
trailing `i8` (127, the controller's "no reading", becomes 0). The decoder
accepts the 19-byte blob older firmware sends, so a mixed fleet still
reports.

## The connect freeze, the watchdog and the event log (2026-09-11)

The report: tracking, beaconing every second, a phone connecting; D2
stopped blinking and stayed lit, the phone never connected, the board
stayed that way. D2 is lit at the start of a transmit and put out at the
top of the next pass (289 ms transmit, 20 ms blink), so a lit D2 is the
hardware loop stopped inside `send()`, which it cannot do on its own - the
wait is a 1 ms `Timer::after` loop with a deadline. A loop stopped inside
a sleep is its core stopped.

Why both cores: esp-backtrace's panic handler, without `halt-cores`,
ends in `interrupt_free(|| loop {})` on the panicking core. esp-hal's
critical section on the S3 is a spinlock with an owner (esp-sync
`RawMutex`), so a core that panics inside one - and the controller's OS
adapter, the timer queue, every `state::` accessor is one - never
releases it, and the other core spins at its next `critical_section::with`
with interrupts off. Milliseconds later both are stopped and the LEDs hold
whatever they had. `halt-cores` would have stalled both explicitly, which
is the same outcome. The HAL's `__user_exception` (LoadProhibited, illegal
instruction, the stack guard) is a `panic!`, so faults end the same way.

Which panic or fault a connect reaches is *not* established. Candidates,
none confirmed: the controller allocating from a heap fragmented by the
duty cycle's build-and-tear-down of the whole BLE stack every window (the
status line now carries `heap N B free` for this); a trouble-host
`panic!("unexpected refcount")` in the channel manager on a teardown
race; the vendored btdm glue. The event log is what will say.

What was built, and the shape of it:

- `proto/src/supervise.rs`: the policy. Two tasks, `Loop` (15 s) and
  `Serve` (20 s); a flat `Phase` set for both; `Supervisor::check` names
  the first task past its bound. `WDT_TIMEOUT_MS` 30 s covers the boot
  before the monitor task runs.
- `firmware/src/watchdog.rs`: heartbeats are two relaxed stores (a u32
  ms word and a phase byte; the age is a wrapping difference, no 64-bit
  atomics, no lock). `guarded(task, phase, fut)` selects the future
  against a 1 s ticker for the waits that may last: `accept`, the
  BLE-down timer, the session as a whole, the park before sleep. The
  session ticker skips its beat while a notify or a write has been in
  flight over 5 s (`session_busy`/`session_free`), which is how a stack
  that stopped answering under a connection the controller still thinks
  is up gets found. TIMG1's MWDT, stage 0 `ResetSystem`; esp-hal's init
  disables every watchdog, `arm` re-enables this one. TIMG0 belongs to
  esp-rtos. The MWDT is in the digital domain, so deep sleep stops it and
  the wake re-arms it.
- Order on a stall: `crumb::record_stall` (RTC RAM, atomics), the console
  line, the direct flash write, reset. Either of the last three may block
  on the spinlock the dead core holds; the unfed watchdog then resets with
  the crumb intact, and the boot logs it. The supervision model checks
  exactly this ordering.
- `firmware/src/crumb.rs`: `#[ram(rtc_fast, persistent)]` atomics with a
  magic word and a checksum; `take()` at boot clears it. `mark_reset`
  records a reason for the resets that leave no crumb (OTA reboot, wipe)
  so the boot record says `reset: software (wipe)`.
- `firmware/src/panic.rs`: own `#[panic_handler]` (esp-backtrace's
  `panic-handler` feature off; its `println` feature must stay on or its
  build script panics). Crumb first, then print, then a 250 ms spin on the
  systimer for the USB FIFO, then `software_reset`. A panic inside the
  handler resets at once.
- `firmware/src/evlog.rs` + `flash.rs`: a `coredump` data partition at
  0x800000, 64 KiB, 512 records of 128 bytes. Appends are one NorFlash
  program of a blank slot; the sector is erased when the ring enters it
  (32 records go at once); a slot that is not blank when reached - an
  interrupted program - is skipped to the next boundary. The head is
  found at boot by the highest seq. Events are queued from wherever they
  happen (`event!` = `status_println!` + a queue push) and the monitor
  writes the queue between checks, so no task does a flash write for a
  log line; `sleep.rs` and the OTA reboot flush explicitly because the
  monitor will not get there first. `usb::EVLOG` reads one record per
  round trip, newest first; index 0xFFFF erases.
- The flash-park deadlock, found by reading esp-storage 0.8.1's
  `internal_write`: `pre_write` parks the other core (`SW_CPU_STALL`, a
  hardware stall at a random instruction) *before* `maybe_with_critical_
  section` and `post_write` unparks *after* it returns. In both gaps an
  interrupt on the writing core runs; if it needs the critical-section
  spinlock and the parked core holds it, it spins forever. Every program
  and erase is now inside `critical_section::with` (`flash::exclusive`),
  which also covers the OTA data writes. Reentrant with esp-storage's own
  lock on the same core. Not the connect freeze - a plain connect writes
  no flash - but a bulk transfer is a hundred windows.
- Isolation builds `unwatch` the loop they do not start, or they would
  reset every 15 s.

Bounds are the unverified part. A card mount that retries for longer than
15 s, or a controller init over 20 s, is a board that resets itself while
working; the log would show the same `stall` phase at the same uptime on
every boot. Raise the bound in `Task::bound_ms` if it does.

## The bench, the false stall, and the card's exit (2026-09-11, later)

The first flash of the watchdog reset the board every 15 s: `stall:
hardware loop silent 15 s in boot`. Two causes, both mine. The monitor's
first act was to beat every task at `Phase::Boot`, which overwrote the
loop's own first beat and left the phase reading `boot`; and the loop's
boot really was longer than 15 s on that board, inside the SD card's
mount - a single synchronous call the loop cannot beat from. The user's
answer was to take the card out of the project for now, which is what
made the loop's boot half a second. If the card comes back, its bus has
to beat (the `BeatingSpi`/`BeatingDelay` wrappers in the history at
`98f73dc^` did that) or it has to live off the loop.

Then a finding the log made on its own: node 1 (`ws3gps-LN4`) came back
with `boot: reset: hardware watchdog` and no crumb - the monitor itself
never ran for 30 s. It coincided with the laptop's first BLE scans of the
two boards, and it did not reproduce under four USB open-and-ping cycles
or four BLE connect cycles against node 0. A first core that stops is
worse than it sounds on this chip: the embassy time driver's alarm fires
on the first core, so the second core's timers stop with it - the
hardware loop freezes wherever it was, LED included. That is a second
route to the original "D2 stuck on, nothing answers", beside a panic
inside a critical section, and it needs no lock at all.

What was added for it: the TIMG1 watchdog now runs two stages - a warning
interrupt at 25 s, bound to the *second* core from its start closure
(`watchdog::warn_on_this_core`; the interrupt matrix is per core, so an
`InterruptConfigurable::set_interrupt_handler` call from core 1 lands
there), whose handler writes what the heartbeats say into RTC RAM with
atomics (`crumb::record_watchdog`: the serve loop's last phase and age,
the hardware loop's, the monitor's silence) - then the reset stage at
30 s. `MwdtStage::Stage0` Interrupt + `Stage1` ResetSystem; a feed puts
the count back to stage 0; the peripheral's own `int_ena.wdt` bit has to
be set beside the CPU-side enable or nothing fires. What it cannot cover:
a second core spinning on the critical-section lock with its interrupts
masked, since a Rust handler runs at level 3 at most. `bench-hang` is the
cargo feature that stops the first core's executor to prove the path.

Bench results (node 0 = `TN2`, node 1 = `LN4`, basement, no fix): boot
to beacon 0.5 s; pings every 5 s; node 1 hears every one at -24 dBm and
takes the hop clock from it; `board-config` over USB while beaconing, the
flash write under the critical section included; four BLE
connect/read/disconnect cycles from the laptop, each seen as `central
connected` on the console with beacons continuing; rename, idle, tracking,
a 20 s nap with `reset: deep sleep wake` logged. The `board-config` ping
has no retry: a port that was just closed answered nothing to the first
open on node 1 once and worked on the next. `hop: clock from node 1
(stratum 15)` prints on every ping a listening node hears, which is one
log-characteristic line per beacon - worth a look.

OTA over USB to node 1 while it ran, after the card's removal and the
two-stage watchdog: 505,952 bytes in 141 s - about 124 sector erases and
programs, every one under the critical section - with the hardware loop
beating throughout, then a reboot into `ota_1` logged as `reset: software
(ota reboot)` behind the transfer record. Node 1 runs from `ota_1` now;
node 0 from `ota_0`, named `TN2`, address 1; node 1 is `LN4`, address 2,
in the listening mode it had stored.


## Naming a remote node

A frame carries the originator's address and nothing else about who sent
it, so a receiver could only ever call a node "node 3". The name a board
keeps in its own flash (the BLE label, `Stored::label`) now travels on the
air as its own message, `MSG_NAME` (0x53) - the tag and 1 to 15 label
bytes, at their true length.

Three designs were weighed. A new bit in the position message's field mask
was rejected outright: `fields_len` computes offsets from the mask, so a
variable-length field breaks it, and a fixed 16-byte name field puts the
full position message past `PAYLOAD_MAX`. Carrying the name on every beacon
was rejected on air time - fifteen bytes on a ten-byte default beacon. What
is left is a separate message taking a whole turn, which the schedule
allows one of per slot, so a name announcement costs exactly one beacon.

The cadence is counted in transmissions (`NAME_EVERY_TX = 20`), not
seconds, and there is deliberately no config key for it. Counting
transmissions makes the cost a fixed 5% of whatever air time the node was
already configured to spend, at any beacon interval, which is the property
a shared channel cares about; a key in seconds would have to be validated
against the beacon interval to keep that. A rename is noticed without any
plumbing: the hardware loop keeps the label it last announced and compares
it against RTC RAM each transmission, so the BLE write on the other core
needs no signal. The same comparison makes the first transmission after
boot a name, since the remembered label starts empty.

On the receiving side names sit beside the roster's reports rather than
being a third kind of report. A name is not something a node "last said" -
it survives the change from a position to a ping, may arrive before that
node's first report, and must not re-notify when a node re-announces what
it is already called. It therefore has its own dirty flags and its own
hand-out (`take_dirty_name`). The one subtlety is expiry: a name ages by
when its node was last *heard from at all*, refreshed by every report,
because a name aged by its own announcement would expire on any node whose
beacon interval put twenty transmissions past the 30-minute TTL.


## Naming a remote node

A frame carries the originator's LoRa address and nothing else about who
sent it, so a receiver can only say "node 3" about a board its operator
calls sky-1. The first attempt at this put the name on the air as its own
message on a slow cadence, and it was the wrong place: a name changes
about once in a board's life, a shared channel pays for every turn spent
on one, and every receiver ends up holding a table of other boards' names
that only a phone ever reads.

What ships instead is a pairing on the BLE link. `c3a1000d` carries
`[address, label]` - which node this board is and what it is called - so an
app that connects to a board once can name that address wherever it later
hears it, through whichever board it is connected to at the time. A fleet
is named by connecting to each board once, or by giving the app the same
pairs as a file. Nothing is added to the air.

Two details that are not obvious:

- It is one value, not two that an app joins. The address is a field of
  the radio-config blob and the label is on the name characteristic, and
  they change at different moments (a rename notifies one, a config push
  the other), so an app pairing them itself would eventually file a name
  under the address the board had before a push. The pair is republished
  whole on connect, after a rename and after a config apply.
- The address is read back out of the config the radio is running
  (`state::radio_config()`), not from a copy kept beside the name, so the
  two cannot disagree. That makes address 0 - not an assignable address -
  the honest answer during a wake check, where the config has never been
  read. The real pair follows by notification once a promotion loads it.

## What wake-on-LoRa turns out to hinge on (2026-09-13)

Design only; nothing built. Three choices that were not the obvious ones,
recorded because the obvious ones are what a later reading would reach for
again.

- **A second LoRa sync word is the whole design, not a detail.** The first
  shape of this was "DIO1 wakes the chip, the chip reads the frame and
  decides" - which is unusable in a fleet: every other node's beacon is a
  wake, and a stored board next to a tracker beaconing once a second never
  sleeps. Writing a different word to `LORA_SYNC_WORD_MSB/LSB` for the
  sentry means the chip does not detect ordinary traffic at all, in
  hardware, with the MCU still gone. Everything else - the address in the
  payload, the fast reject - is belt and braces behind that one register
  pair.
- **The TCXO picks the spreading factor, not the duty cycle.** The 10 ms
  `tcxo_startup_ms` is paid on every RX window of a `SetRxDutyCycle`,
  because DIO3 drops the oscillator during the radio's own sleep. So a
  shorter symbol time does not scale the sentry current the way it scales
  the air time: at SF12/BW500 a four-symbol window is 33 ms against 10 ms
  of startup, at SF7 it is 1 ms against the same 10 ms, and the sentry is
  about 4x cheaper at SF7 for that reason alone. It is still the wrong
  default - SF7 is ~10 dB less sensitive, so the board could be heard and
  not woken - but the reason it is tempting is the fixed startup, not the
  symbol rate.
- **The frame that woke the board is inside the radio, and `init` erases
  it.** After `RxDone` the chip sits in `STDBY_RC` with its configuration
  and RX buffer intact, but `Sx1262Driver::init` pulses NRST. So the
  address check that makes a false wake cheap has to read the buffer
  before anything touches the reset line - a peek that takes the existing
  driver and only reads, with `init` on the yes branch only.

## Why the sleeping board must not be the one that transmits (2026-09-14)

Two alternatives to wake-on-LoRa, analyzed and rejected. The reasoning is
worth keeping because both are the natural first idea.

- **The sleeper chirping on each wake, so a listener learns its schedule.**
  Killed by a silicon fact, not by arithmetic: `SetRxDutyCycle` is the only
  self-timed command the SX1262 has, and there is no timer-driven transmit.
  So a sentry that receives leaves the S3 in deep sleep for the whole cycle,
  and a sentry that transmits costs a chip wake every cycle forever. Add
  that TX is 127 mA against RX's 6 mA and that the shortest frame this
  firmware sends is already 248 ms - a 4-byte ping, because preamble and
  header dominate - and the chirp is ~43 mAs a cycle against the sentry's
  0.26. Shortening the chirp does not help: at SF7 the S3 wake is 90% of
  what is left, so the term being optimized has stopped mattering.
  The deeper point is that a chirp pays per cycle and a wake burst pays per
  wake event, and wake events are rare.
- **A train of short wake packets instead of one long preamble.** Same
  family as the plan - the waker still pays - but a duty-cycled receiver
  samples for a *preamble*, and a short packet's preamble is a small slice
  of the train period. Worse, the sentry cycle and the train period are two
  free-running periodic processes: at 1000 ms against 250 ms the ratio is
  exactly 4 and the RX window samples the same slice of every train period
  forever, so the board either wakes immediately or never, decided by a
  phase nobody chose. A lucky bench phase passes. Its one real advantage is
  that 248 ms fits the FHSS 400 ms/20 s rule and a 2 s preamble does not, so
  it is the shape a hopping plan would need.

Both roads end at the same hardware gap: GPIO15/16 are the S3's
XTAL_32K_P/N and this module leaves them unrouted, so RTC_SLOW_CLK is the
150 kHz RC, free-running and uncorrected through a sleep at percent-level
drift. A schedule agreed before a sleep is good for under 2 s. That is what
makes "announce the next window in the wake ack" - which would otherwise be
the obvious refinement - worthless here, and it is why the crystal is now an
HW-TODO item rather than a firmware one.

## Measuring a duty-cycled receiver with no instruments (2026-09-14)

The wake-on-LoRa design turns on one number: what a receive window costs
beyond the symbols it is meant to hear. The chip restarts its oscillator on
every window, and whether that 10 ms is *added* to the commanded window or
taken *out* of it decides how long every window in the design has to be.

The instrument is the radio. With a second board keying
`SetTxInfinitePreamble`, a signal is always present, so every window that
opens should detect - and DIO1 becomes a readout of the receiver's own
schedule. No scope, no current probe, and the whole thing is two cargo
features.

Three things that were not obvious while writing it:

- **The cadence half and the sweep half want opposite treatment of the
  arming.** The sweep must re-arm per trial, because each trial is an
  independent question ("does a window this long catch a signal already
  there?"). The cadence half must *not*, because `SetRxDutyCycle` starts
  with the receive phase - re-arming after each detection would measure the
  window's own length instead of the cycle. Doing both the same way makes
  one of them silently measure the wrong quantity, and the number still
  looks reasonable.
- **Whether detections keep coming is itself a result.** If the chip leaves
  the cycle on a reception, a sleeping board has to re-arm on every wake or
  it is deaf from the second one on. The cadence half reports that rather
  than treating an early stop as a failed run.
- **The sweep ladder is in overhead, not in microseconds.** Each step is
  `detect_symbols * t_sym + headroom`, so the same ladder runs at any
  spreading factor and the shortest passing step *is* the overhead. An
  absolute ladder would have to be rewritten for every modulation and would
  not report the quantity wanted.

`sweep_floor` only accepts a step whose longer neighbors all passed, which
is the one place a plausible wrong answer was easy to get: a short window
that catches while a longer one misses is a run to repeat, and taking the
shortest passing step would adopt the luckier of two runs and size every
preamble in the system against a window that is not reliable. Host-tested
for exactly that case.

## The wake preamble is bounded above, and that was nearly missed (2026-09-15)

SX1261/2 datasheet DS.SX1261-2.W.APP rev 1.2, section 13.1.7. Four things
about `SetRxDutyCycle` that no amount of bench iteration would have produced,
and one of them inverts the arithmetic the plan was built on.

- **The sniff loop leaves on `RX_DONE`, not on preamble detection.** A
  preamble merely stops the window timer and restarts it at
  `2 * rxPeriod + sleepPeriod` while the chip hunts for a header. This is
  why the first bench run measured nothing: the source keyed an infinite
  preamble, which never becomes a packet, so the probe was watching for an
  event the mode cannot produce on that signal. The test was invalid, not
  the mechanism.
- **`Tpreamble + Theader <= 2 * rxPeriod + sleepPeriod`.** The preamble has
  an upper bound, not just the lower one the sampling requirement gives. The
  draft plan asked for `2 * sleepPeriod + rxPeriod` - 2.08 s at a one-second
  sleep against a ceiling of 1.10 s - so every wake frame would have been
  abandoned mid-preamble with the receiver awake and nothing in any counter
  to say why. A longer preamble is not the safe direction.
- **Subtracting the bounds cancels the sleep**, leaving
  `margin = 2 * rxPeriod - (Theader + Tdetect + Ttcxo)`. So the receive
  window sets the timing margin and the sleep period sets the current, and
  they are independent. There is also a floor - `rxPeriod` at least half of
  `Theader + Tdetect + Ttcxo`, which is 54 ms at SF12/BW500 - below which no
  sentry works at any sleep or preamble. The draft's 43 ms window was under
  it.
- **The TCXO startup is added between the sleep and receive phases**, not
  taken out of the window. So the cycle is `rx + sleep + tcxo` and the
  window listens for as long as it was told to.

Separately, and easy to ship without noticing: the Rx Gain register is not
in the warm-start retention memory, and the datasheet calls the fix
mandatory for `SetRxDutyCycle` - write 0x01/0x08/0xAC to 0x029F/0x02A0/0x02A1.
This firmware boosts Rx gain in `init` and defaults `rx_boost` on, so every
sentry window after the first would run ~2 dB deaf on the one link where
sensitivity is the entire point.

## The symbol timeout was the whole problem (2026-09-17)

A week of two-percent wake rates, seven parameters swept to no effect, and
a final finding that the configuration did not survive the sleep - all of
it was `SetLoRaSymbNumTimeout(8)`. RM0461 says what the datasheet does not:
with SymbNum set, the modem counts chirps from the first one it sees and
times out unless the *end* of the preamble arrives within that many
symbols. A window opened mid-way through a 140-symbol preamble can never
satisfy that with eight, so only a window opening in the last few symbols
ever completed a reception: about three percent, independent of preamble
length. At zero the sniff loop woke on 29 of 29, 30 of 30, 29 of 29 frames
in three arms; at eight, 2 of about 30.

The "configuration lost" finding was the same setting: it persists in the
chip across every mode change, the control listen had run at zero, and the
after-warm-start listen inherited the eight from the duty-cycle arm before
it. `XOSC_START_ERR` (0x0020) is latched by every warm start with a TCXO
and means nothing; it reads after every successful trial too. Every plain
receive arm now resets both SymbNum and StopTimerOnPreamble.

The corrected bounds: window = rxPeriod; a preamble is caught if it spans
`sleep + tcxo + 2 * detect` and its header lands inside `2 * rx + sleep`
(measured to bite: 62% with the bound exceeded by 120 ms). The sleep
cancels out of the margin, so the window buys drift tolerance and the sleep
buys current - two config keys, not one.

Choices in the implementation that were not the obvious ones:

- **The sync word goes in the retention list.** It is a register write,
  and the warm start between windows restores commands, not registers -
  the same shape as the Rx gain, and the list holds four.
- **DIO1's pad has to be un-muxed at boot.** The EXT0 wake source routes
  it to the RTC mux; that register is in the RTC domain and survives the
  reset, and esp-hal's `Input::new` never clears it. Without the un-mux the
  receive poll would never see an interrupt after the first LoRa wake.
- **NRESET is pad-held through the sleep** beside NSS: a reset line
  drifting low would reset the sentry into its power-up state silently.
- **The fast reject sleeps from inside the boot**, before the event log,
  the config or any task, using the periods and the address the park left
  in RTC RAM - so a frame for another node costs a fraction of a second
  and no flash read. `go_down` is the sleep's tail split out for it.
- **A call is not gated by the mode.** A listening node never beacons and
  the burst first inherited that gate, and waited silently forever. It is
  gated on the radio being up and the role transmitting.
- **The posture decides on the sentry before the config it needs is
  read**, on a wake check: `PrepareSleep` in one pass pushes LoadConfig,
  RadioInit and RadioSentry, and the arm effect re-checks the config and
  sleeps the radio cold if it says no. The posture's rule accepts a sentry
  as a parked radio; the firmware model holds that a board asleep has a
  sentry exactly when its config asked, via what the arm actually did.
- **The status line no longer asks a cold radio its health.** The NSS edge
  of the asking woke it into standby, where it stayed at half a milliamp
  on every idle board - a pre-existing cost, cheap to stop once seen.
- **A console line printed ahead of an ack is one the tool discards** (its
  frame reader parses past text), so the hardware task says "calling" a
  pass later rather than the config path saying it before the ack.
- The wake-check ceiling is an hour now; a store with no cadence still
  borrows five minutes (`STORE_DEFAULT_S`).

Bench: node 5 called node 3 with a 403-symbol preamble; node 3 woke 14 s
into a 60 s interval, booted idle and answered 860 ms after the frame
ended. Pings every 5 s from node 5 across several cadences did not wake
the sentry.

A second bench finding the same evening: the caller's own beacon, 0.8 s
before its wake frame, landed in the sentry's window. With SymbNum at zero
the modem locks on any LoRa symbol, the sniff loop then holds the receiver
for `2 rx + sleep` hunting a header on the wrong sync word, and then sleeps
a full period - about seven seconds blind, and the wake frame fell inside
it. The call was answered on its second try. So a caller holds its beacon
for the length of a call, and the sentry has its own carrier
(`wake_frequency_hz`, 0 = the network's): a fleet beaconing every second on
the sentry's carrier would blind it most of the time, and 927 MHz measured
ten times quieter than the rest of the band here.

## A stored-mode reading of 0.02 mA (2026-09-17, user's meter)

Reported: about 0.02 mA in stored mode with 0.03 mA spikes on an interval.
Which board and which build was on the meter is not recorded. Two things
that reading can be:

- **The deep-sleep floor without a sentry.** S3 deep sleep plus the SX1262
  in cold sleep plus the M10 in backup on VCC comes to about 20-25 uA on
  paper, which is what this looks like, and it answers the stored-floor
  question the TODO carried: storage life is years on a cell, not weeks.
- **Not a sentry.** A sentry armed at the defaults is 300 ms of receive at
  about 5 mA every 3.3 s - about 0.5 mA averaged, and 5 mA spikes a meter
  cannot miss. A board reading 0.02 mA is not cycling its radio: an older
  build, a config with wake_enabled off or hopping, or a meter whose
  range cannot resolve a 300 ms pulse. The check is a `board-wake` from the
  other board while the meter is attached: an answer means the sentry was
  armed, and the meter should show the windows.

The 0.03 mA spikes on an interval are unexplained; the M10's backup and
the S3's RTC domain are both steady, and a wake check is a hundred
milliamps for fifteen seconds, not thirty microamps.
