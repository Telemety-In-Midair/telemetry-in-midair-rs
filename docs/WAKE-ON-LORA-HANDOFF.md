# Wake on LoRa: where this is, for whoever picks it up

A briefing rather than a design. The design is `WAKE-ON-LORA.md`; this says
what is proven, what is open, how to run the bench without losing an
afternoon to it, and which mistakes have already been paid for.

## The question

Can a board asleep in deep sleep be woken over LoRa, so that a stored node
is reachable on demand rather than on a timer cadence? The plan is for the
SX1262 to duty-cycle its own receiver while the S3 sleeps, and to pull DIO1
when a frame arrives.

## Where it stands in one line

The mechanism works, and a duty-cycled receiver completes about 2% of the
frames a continuously listening one completes 100% of. Nobody knows why yet.

## What is proven, with the evidence

Every line here was measured on hardware, most of it twice.

| | |
|-|-|
| A duty-cycled receiver **can** be woken by a real frame, via `RxDone` | reproducible across runs |
| **The receive window is `SymbNum x symbol time`**, not `rxPeriod` | 4/8/16/32 symbols tracked to within 8 us |
| `rxPeriod` is only a ceiling on that | same sweep |
| The sleep is exactly what is commanded | 994 ms, seven runs |
| The oscillator startup is *added* between sleep and receive | datasheet, and BUSY maxima at 10.7 ms |
| `SetLoRaSymbNumTimeout` must not be zero | zero woke on nothing in 150 s |
| Symbol counts above ~63 need the 0x0706 mantissa/exponent encoding | 64 and 128 misbehave without it |
| **Continuous receive completes these frames perfectly** | 18 of 18, 0 CRC errors, preambles to 320 symbols |
| So the frame, the preamble length and the link are all fine | same run, back to back with the duty cycle |
| Rx gain is **not** retained across a warm start | datasheet calls the fix mandatory; it is applied in `arm_duty_cycle` |
| The chip will not answer SPI while duty cycling, and asking ends the cycle | status reads return 0xFFFF; NSS falling wakes it |
| `BUSY` is readable from the host and costs nothing | it is an ESP input; this is the only free observation of the part |
| `BUSY` marks only the transitions, not the phases | its low periods alternate window/sleep |
| `BUSY` glitches for a few microseconds at its own edges | filter anything under ~1 ms |
| RC64k runs +10 680 ppm with a ~50 ppm spread | stable across runs; the offset divides out, the spread needs margin |
| 927 MHz is about ten times quieter than 903-923 | band survey, both gain settings |
| DIO2/DIO3 never leave the module; DIO1/BUSY/SPI are internal too | module datasheet pin list |

## What has been ruled out, with evidence

Do not re-test these without a reason:

- **The re-arm after a reception** - every arm verified `rx` in its first
  window, 100%.
- **The symbol timeout value** - 4 and 8 behave identically; 0 is fatal.
- **The receive window width** - 100 ms and 200 ms give identical results,
  which the window measurement later explained: both truncate to the same
  symbol count.
- **The noise floor** - moving to the quietest carrier in the band made it
  *worse*.
- **The oscillator settling** - 10 ms against 100 ms, no difference.
- **The preamble length** - swept 60 to 320 symbols, four frames each. No
  length works reliably and the ones that do wake are not reproducible
  between runs.
- **The frame and the link** - continuous receive completes 100% of the same
  frames.

## The open question

A duty-cycled receiver completes ~2% of what a continuous one completes,
with the window measured as exactly what was commanded, the timing
predicting ~96%, and the frames themselves provably receivable.

So something about **sleeping and warm-starting** leaves the receiver less
capable than it is from a cold configuration. The `RX_GAIN` retention fix is
already applied and was not enough.

The test that was running when this was written - `receives_after_warm_start`
in `firmware/src/sentry.rs` - pushes the chip through a sleep and restore
and then listens continuously without re-initializing. If it still receives,
the restore is fine and the cost is genuinely the window being open 6% of
the time. If it does not, the restore leaves something behind and no amount
of window or preamble arithmetic will ever find it.

After that, the honest next step is a supply-current measurement across a
window: `BUSY` says the chip is awake, not that its receiver works.

## The hardware

Two Wio-S3 boards and an RP2040-Zero.

```
E2:D4   ws3gps-TN2     usually the source
E4:EC   ws3gps-e4ec    usually the probe
RP2040-Zero            independent monitor, ~/gps/sentry-monitor
```

Address boards by `/dev/serial/by-id/` and never by `ttyACMn` - the numbers
move between sessions and have already caused one wrong-board flash.

RP2040 wiring, both 3.3 V so direct:

| RP2040-Zero | Wio-S3 J1 | signal |
|-|-|-|
| GP2 | GPIO38 | BUSY |
| GP3 | GPIO39 | DIO1 |
| GP4 | GPIO40 | armed marker |
| GND | GND | required |

The mirror build pulses each pin a different number of times at startup
(BUSY once, DIO1 twice, MARK three times) so a monitor can prove not just
continuity but identity. Use it - DIO1 only moves on a reception, so a dead
wire and a good one are otherwise indistinguishable.

## Running the bench

The isolation builds, in `firmware/`:

```
iso-sentry-probe     arms a sentry and measures it; the main instrument
iso-sentry-source    sends wake frames, sweeping preamble length
iso-sentry-carrier   keys a continuous preamble (for detection tests only)
iso-sentry-mirror    times BUSY and mirrors it to J1; needs no transmitter
```

**Order matters and getting it wrong silently wastes the run:**

```
1. flash NORMAL firmware to both boards
2. board-set mode listening
3. board-config --address N --set frequency_hz=927000000 --set power_dbm=0
   -> must print "config applied". "config accepted" means it did NOT.
4. flash the isolation builds
5. read the SOURCE console and confirm it is transmitting
6. only then read the probe
```

Steps 3 and 5 are not optional. Both have produced whole runs of null
results that looked like radio findings.

## Traps already paid for

Each of these cost at least one run, and most have a skill written for them.

- **Config pushed to a board running an isolation build is silently
  dropped.** The build never returns to the loop that applies it. The
  transport acks, the tool says "accepted", the board keeps its old
  settings. A source stuck at 22 dBm then refuses to transmit and the probe
  reports a silent channel. `config-push-dropped-by-blocked-loop`.
- **`cargo build` caches per feature set and the artifact path never
  changes.** Always rebuild with the intended feature immediately before
  flashing, or you will flash the previous build. Done twice here.
- **`espflash reset` on a port another process holds** drives the chip into
  ROM download mode before failing, and the board can leave the USB bus
  entirely. Never overlap a capture with an espflash command.
  `espflash-reset-busy-port`.
- **A tight poll loop starves other tasks.** On the dual-core S3 a 1 ms
  `Timer::after` starved the watchdog monitor on the other core and reset
  the board with no stall record. On the RP2040 a loop with no `await` at
  all meant the USB task never ran and nothing was ever reported.
  `embassy-poll-starves-other-core-watchdog`.
- **`grep -c` returning 0 exits non-zero** and breaks `&&` chains, so a
  build step after it never runs.
- **Do not leave a variable set between experiments.** A `tcxo_startup_ms`
  left at 100 from a previous test, with only the firmware constant
  reverted, invalidated a whole sweep.

## Reading the history

The commit messages carry the reasoning, including three findings that were
later withdrawn. Withdrawals are recorded rather than quietly deleted,
because the reason a measurement was wrong is usually the useful part: in
every case the instrument was measuring something other than what it
appeared to, and that is the standing hazard of this particular chip.

`proto/src/sentry.rs` holds the arithmetic and is host-tested - 294 tests.
If a model here disagrees with hardware, the model is wrong; it has been
three times.
