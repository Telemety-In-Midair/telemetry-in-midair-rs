# Architecture

UML views of the telemetry-in-midair board: what the parts are, how the RF
path is wired and why that wiring is a safety constraint, what a BLE session
does, and the modes the radio runs in.

The board is one Seeed Wio-S3 module (ESP32-S3R8 + SX1262 + TCXO in one can)
on the `wio-s3-max-gps` carrier, with a u-blox MAX-M10 GPS and a microSD
card. It replaced a two-MCU design - an ESP32-C6 for BLE and power, a
WIO-E5 for GPS/LoRa/SD, a framed UART link between them - and roughly a
third of the old firmware existed only to bridge that split.

## Structure

One MCU holds every role. Everything host-testable - the session policy,
the roster, the LoRa codec, the radio config parser, the USB framing - lives
in the shared `midair-proto` crate and runs under `cargo test`.

```mermaid
classDiagram
    direction LR

    class GpsGuiApp {
        <<BLE central>>
        scans by service uuid
        subscribes to notifications
        writes config ids and bulk ops
    }
    class HostTools {
        <<pixi, USB serial>>
        cargo run flashes
        wio-config pushes a radio config
        wio-ota pushes a firmware image
        wio-info reads the BLE address
    }
    class RemoteNode {
        <<other board, 915 MHz>>
        broadcasts position or ping
        repeater forwards hops
    }

    class Firmware {
        <<Wio-S3, embassy, one binary>>
        BLE, LoRa, GPS, SD
    }
    class ServeTask {
        advertise()
        accept one central()
        enter_deep_sleep()
    }
    class GattSession {
        publish settings and radio config
        replay the roster on connect
        notify position, telemetry, remotes, log
        apply_config() bulk writes
    }
    class UsbTask {
        <<USB Serial/JTAG>>
        PING INFO BULK
    }
    class HardwareTask {
        owns radio, gps, card and panel
        beacon() poll() repeat() log()
        applies a pushed config
        blanks the panel for sleep
    }
    class StatusOled {
        <<optional, SSD1306 on J5>>
        fix, sats, RSSI, time since
        compass to the newest node
        detected not configured
    }
    class Magnetometer {
        <<optional, QMC5883L or HMC5883L>>
        shares the J5 bus
        heading, hard-iron corrected
    }
    class Geo {
        bearing_deg() distance_m()
        relative_bearing_deg()
    }
    class State {
        <<snapshot, not a channel>>
        set_position() take_position()
        radio_busy() transfer_active()
        roster, log lines, request queue
    }
    class Xfer {
        <<one transfer, either transport>>
        handle(owner, op)
    }
    class FlashStore {
        <<one peripheral, two users>>
        nvs settings record
        OtaSink into the idle slot
    }
    class Settings {
        <<RTC RAM + nvs mirror>>
        survives deep sleep and a flat cell
    }

    class MidairProto {
        <<no_std, cargo test on host>>
    }
    class SessionPolicy {
        apply(write) Outcome
    }
    class Roster {
        newest report per node
    }
    class LoraCodec {
        encode_position() Ping
    }
    class RadioConfig {
        parse_bytes() encode()
    }
    class LinkCodec {
        USB bulk framing only
    }
    class BulkTransfer {
        ops, sequencing, crc
        Sink for firmware images
    }

    class Node {
        address, role, max hops
        dedup by (src, id)
        jittered repeat queue
    }
    class Sx1262Driver {
        init() send() poll_recv()
        looks_reset()
    }
    class Sx1262Cmds {
        <<SPI + NSS + BUSY + DIO1 + NRST>>
    }
    class RfSwitch {
        <<SKY13453-385LF, in module>>
        VCTL from DIO2
        VDD from DIO3
    }
    class MaxM10 {
        <<GPS receiver, UART>>
    }
    class SdCardHw {
        <<FAT16 or FAT32, SPI, optional>>
    }

    GpsGuiApp ..> GattSession : GATT
    HostTools ..> Firmware : USB serial

    Firmware *-- ServeTask
    Firmware *-- GattSession
    Firmware *-- UsbTask
    Firmware *-- HardwareTask
    Firmware *-- FlashStore
    ServeTask --> GattSession
    ServeTask --> Settings : sleep interval, window
    GattSession <--> State
    HardwareTask <--> State
    HardwareTask --> StatusOled : render(telemetry)
    HardwareTask --> Magnetometer : sample()
    Magnetometer --> StatusOled : heading, else GPS course
    StatusOled ..> Geo : bearing and distance<br/>to the roster's newest node
    StatusOled ..> State : reads the same snapshot<br/>the app is notified
    UsbTask --> Xfer
    GattSession --> Xfer
    Xfer --> BulkTransfer
    Xfer --> FlashStore : OtaSink
    Xfer ..> State : request(ApplyConfig)
    Settings --> FlashStore : nvs mirror
    GattSession --> Settings

    MidairProto *-- SessionPolicy
    MidairProto *-- Roster
    MidairProto *-- LoraCodec
    MidairProto *-- RadioConfig
    MidairProto *-- LinkCodec
    MidairProto *-- BulkTransfer

    GattSession --> SessionPolicy
    State --> Roster
    HardwareTask --> Node
    HardwareTask --> MaxM10
    HardwareTask --> SdCardHw
    Node --> Sx1262Driver
    Node ..> LoraCodec
    Sx1262Driver --> Sx1262Cmds
    Sx1262Driver --> RadioConfig : live settings
    SdCardHw ..> RadioConfig : RADIO.CFG
    Sx1262Cmds --> RfSwitch : DIO2, DIO3
    RfSwitch <..> RemoteNode : broadcasts and hops
```

`State` is a snapshot, not a channel, and deliberately: every consumer wants
the latest position and never a backlog. It is also what removed the link
protocol - the BLE session never touches hardware, it reads what the
hardware task published and signals back through a `Request`.

## The RF path, and why two registers are not tunable

This is the part of the design where a wrong value destroys hardware rather
than degrading a link, so it gets its own view. Everything below is from
the Wio-S3 module datasheet, Table 2 (SX1262 pin mapping) and Table 3
(SKY13453-385LF truth table).

```mermaid
flowchart LR
    subgraph MOD["Wio-S3 module"]
        ESP["ESP32-S3R8"]
        SX["SX1262"]
        SW["SKY13453-385LF<br/>antenna switch"]
        TCXO["32 MHz TCXO"]
    end
    ANT(["LoRa port<br/>pad 37 to SMA"])

    ESP -- "SPI: GPIO4 SCK, GPIO6 MOSI,<br/>GPIO5 MISO, GPIO21 NSS" --> SX
    ESP -- "GPIO7 NRESET" --> SX
    SX -- "GPIO8 BUSY, GPIO9 DIO1" --> ESP
    SX -- "DIO2 = VCTL<br/>high in TX" --> SW
    SX -- "DIO3 = VDD<br/>2.5 - 3.5 V required" --> SW
    SX -- "DIO3 supply" --> TCXO
    TCXO -- "clock" --> SX
    SX <--> SW
    SW <--> ANT
```

Two consequences the firmware must not let a config override:

- **`SetDio2AsRfSwitchCtrl` (0x9D) must be enabled.** DIO2 is the switch's
  VCTL. Left off, VCTL never rises, the switch parks on the path that is not
  the PA, and every transmission ramps +22 dBm into an isolated port.
- **`SetDio3AsTcxoCtrl` (0x97) must name at least 2.7 V.** DIO3 is the
  switch's VDD as well as the TCXO supply, and the switch is specified
  2.5 - 3.5 V. Its truth table calls anything outside that *undefined*, so a
  lower setting leaves the switch in no known state while the PA transmits
  into it.

Both were inherited from the WIO-E5, whose radio is on-die with no bonded
DIO2 and whose TCXO is a 1.8 V part fed by nothing else. `RadioConfig`
now defaults them to this board's hardware, and `Sx1262Driver::init`
enforces them regardless of what a config asks for, logging when it
overrides. The keys remain in `RADIO.CFG` to be read, not retuned.

## A BLE session end to end

```mermaid
sequenceDiagram
    participant App as gps-gui-rs
    participant Serve as ServeTask
    participant Gatt as GattSession
    participant St as State
    participant Hw as HardwareTask

    Serve->>Serve: advertise (service uuid + name)
    App->>Serve: connect
    Serve->>Gatt: hand over the connection
    Gatt->>App: notify settings, radio config
    Gatt->>St: replay_remotes()
    St->>App: every node still inside the TTL, aged

    loop every notify interval
        Gatt->>St: radio_busy()?
        alt radio has the air
            Gatt-->>Gatt: skip this tick
        else
            Gatt->>St: take_position(), telemetry()
            Gatt->>App: notify position, telemetry
        end
    end

    Hw->>St: record_remote() when a node is heard
    St->>Gatt: REMOTE_SIGNAL
    Gatt->>App: notify remote or node_ping, once per report

    App->>Gatt: write config id
    Gatt->>Gatt: session::apply (host-tested policy)
    Gatt->>St: request(GpsSleep | RadioStandby)
    Gatt->>App: notify ack, then fresh settings
    St->>Hw: take_request() on the next loop
    Note over Gatt,Hw: the ack always holds - there is no<br/>second chip that can fail to answer

    App->>Gatt: bulk ops (a radio config)
    Gatt->>Gatt: reassemble, check the crc, parse
    Gatt->>App: ack per op
    Gatt->>St: request(ApplyConfig)
    St->>Hw: re-init the radio, rewrite RADIO.CFG
    Hw->>App: status line, then the new radio config

    App->>Serve: disconnect
    Serve->>Serve: advertise again, then sleep if the window is spent
```

Notifications hold while the radio has the air. A LoRa transmit at 22 dBm
beside a 2.4 GHz radio is a supply problem, not a protocol one; on the
two-MCU board this was a `RADIO_BUSY` link message negotiated in both
directions, and here it is a bool.

## Radio modes

```mermaid
stateDiagram-v2
    [*] --> Reset : NRST pulse

    Reset --> StandbyRc : init()
    note right of Reset
        BUSY high through startup.
        The wait is bounded at 50 ms,
        so an absent or mis-wired radio
        is reported, not a hang.
    end note

    StandbyRc --> StandbyRc : configure<br/>regulator, DIO2, DIO3,<br/>calibrate, PA, sync word

    StandbyRc --> Rx : enter_rx() if the role listens
    Rx --> Rx : RxDone, packet kept
    Rx --> Rx : CrcErr, counted and dropped
    Rx --> StandbyRc : send() takes the air

    StandbyRc --> Tx : set_tx()
    Tx --> Rx : TxDone, re-arm at once
    Tx --> StandbyRc : TxDone on a tx-only node
    Tx --> Rx : timeout

    StandbyRc --> Sleep : sleep()
    Sleep --> Reset : init() again
    note left of Sleep
        Cold sleep: configuration is lost
        on purpose, since init() rewrites
        all of it anyway.
    end note
```

A transmit-only node never arms the receiver - idling in standby instead of
continuous RX is the whole reason that role exists. A listening node
re-enters RX immediately after `TxDone`, because every millisecond out of RX
is a chance to miss someone else's broadcast.

Receive polling checks DIO1 as a GPIO before paying for an SPI round trip,
which is most of what an idle node does. On the WIO-E5 there was no such
pin - DIO1 was an internal NVIC vector - so every poll cost a transaction.

## Where the config and the firmware live

Both arrive the same way - a bulk transfer over BLE or the USB console -
and the shared `midair_proto::bulk` state machine is what makes "one at a
time" a property of the object rather than a flag someone has to check.
Where they end up is what differs: a config is small, has to be parsed
whole, and belongs on the card; an image is hundreds of kilobytes and goes
straight to flash as it arrives.

```mermaid
flowchart TB
    App["gps-gui-rs<br/>(BLE)"] --> Xfer
    Host["wio-config / wio-ota<br/>(USB)"] --> Xfer

    Xfer{{"bulk::Transfer<br/>ops, sequencing, crc32<br/>owned by one transport"}}

    Xfer -->|KIND_TOML| Parse["radiocfg::parse_bytes"]
    Parse -->|Err| Nak["NAK the op<br/>(the host hears about it)"]
    Parse -->|Ok| Pending["pending config"]
    Pending --> Hw["HardwareTask"]
    Hw --> Radio["re-init the radio<br/>reconfigure the node<br/>re-push GPS settings"]
    Hw --> Card["write RADIO.CFG<br/>(the only copy that survives a reboot)"]

    Xfer -->|KIND_OTA| Sink["OtaSink<br/>stage a sector, write it"]
    Sink --> Slot["the app slot that is NOT running"]
    Slot --> Activate["crc matched:<br/>point ota-data at it, mark New"]
    Activate --> Boot["reboot"]
    Boot --> Confirm["boots, reaches main:<br/>mark Valid"]
    Boot -.->|never gets there| Rollback["bootloader reverts<br/>to the previous slot"]
```

Two things about the OTA path are load-bearing rather than incidental. The
destination is checked against the slot the code is *executing* from, read
from the MMU rather than from ota-data - the two disagree on every board
just flashed over USB, and writing an image over the running slot destroys
the firmware mid-transfer rather than failing. And ota-data is normalized at
boot when it names no slot at all, which is the state a USB flash leaves it
in, because the arithmetic that picks the *other* slot has nothing to work
from otherwise.

## The wake / advertise / sleep cycle

```mermaid
stateDiagram-v2
    [*] --> Boot
    Boot --> Advertise : window = adv_window_s

    Advertise --> Connected : a central accepts
    Connected --> Advertise : disconnect (linger 5 s)

    Advertise --> Advertise : sleep_interval_s = 0<br/>(the default: never sleep)
    Advertise --> Park : window spent and sleep_interval_s > 0

    Advertise --> Park : CFG_SLEEP_NOW / USB SLEEP
    Connected --> Park : CFG_SLEEP_NOW<br/>(after the ack has left)

    Park --> DeepSleep : radio in cold sleep,<br/>NSS pad-held
    DeepSleep --> Boot : timer wake, wake count += 1

    note right of Park
        There is no rail to cut on this
        board, so the radio is the one load
        the firmware can drop - and holding
        NSS is what keeps it dropped, since
        the S3 releases unheld pads and the
        SX1262 wakes on a falling NSS edge.
        The MAX-M10 keeps acquiring: V_BCKP
        is unconnected, so backup mode costs
        a cold start on every wake.
    end note

    note right of Connected
        A commanded sleep ends the session
        rather than sleeping under it: the
        board stops being contactable, and
        a connection left open would show
        the phone a timeout instead of a
        disconnect.
    end note

    note left of Advertise
        The budget is a deadline, not a
        per-attempt timeout. A central that
        keeps failing to connect cannot
        restart it, which is what kept a
        board awake at full current forever.
    end note
```

The two commanded transitions exist because the timed ones only fire when a
window expires with nobody connected - so without them the only way to sleep
a board in front of you is to disconnect and wait. Sleeping is asked for
through `state::SLEEP_NOW_SIGNAL` rather than done where it is requested:
`apply_config` runs inside the GATT session with the ack still unbuilt, and
the loop that owns the `Rtc` is the one that can wait for the link to finish.

Settings live in RTC fast RAM so a wake check costs no flash read, and are
mirrored into the `nvs` partition so they also survive a flat cell. Only the
settings that decide whether a board is reachable at all are mirrored; the
GPS and radio sleep flags are not, because a board that cold-boots with its
GPS running is the safer of the two failures.

## The states over time

The diagram above says what follows what. This says what the board can
still *do* while it is in each state, and for how long.

Two charts, because the two duty cycles never run together. `serve` checks
deep sleep first, so a board with `sleep_interval_s` set never reaches the
`ble_off_s` branch at all - the awake duty cycle only exists on a board that
has deep sleep off.

Read the axis with one caveat. The window, off-period and sleep-interval
lengths are the config keys and are exact; the beacon spacing is the
interval without its jitter; and **boot, Park and the session timings are
illustrative** - no boot or wake time has ever been measured on this board,
and the connect, transfer and disconnect instants in the third chart are one
plausible session rather than a recorded one.

### The awake duty cycle

`adv_window_s = 15`, `ble_off_s = 30`, `sleep_interval_s = 0`. The modem
goes down; the tracker does not.

```mermaid
gantt
    title Awake duty cycle - only BLE is duty-cycled
    dateFormat X
    axisFormat %M:%S

    section Board state
    Boot (cold)             :done,   s1, 0, 4s
    Advertise               :active, s2, 4, 5s
    Connected               :crit,   s3, 9, 25s
    Linger 5 s              :active, s4, 34, 5s
    BLE down 30 s           :done,   s5, 39, 30s
    Advertise               :active, s6, 69, 15s
    BLE down 30 s           :done,   s7, 84, 30s

    section BLE
    stack built             :milestone, b0, 4, 0s
    advertising - anyone may connect :active, b1, 4, 5s
    in session - CONNECTIONS_MAX is 1 :crit, b2, 9, 25s
    advertising - the phone may return :active, b3, 34, 5s
    stack dropped - 71 mA goes :milestone, b4, 39, 0s
    advertising - anyone may connect :active, b5, 69, 15s
    stack dropped           :milestone, b6, 84, 0s

    section LoRa
    continuous RX           :active, r1, 3, 111s
    beacon                  :milestone, r2, 5, 0s
    beacon                  :milestone, r3, 25, 0s
    beacon                  :milestone, r4, 45, 0s
    beacon                  :milestone, r5, 65, 0s
    beacon                  :milestone, r6, 85, 0s
    beacon                  :milestone, r7, 105, 0s

    section GPS
    tracking - never gated  :active, g1, 1, 113s

    section SD card
    logging every fix       :active, d1, 1, 113s

    section USB console
    alive throughout        :active, u1, 0, 114s

    section J5 panel
    refreshing              :active, o1, 4, 110s
```

Nothing below the BLE lane changes shape. That is the whole point of this
cycle: a board in a BLE-down period is still beaconing, still logging and
still answering the USB console - it is only unreachable from a phone,
for at most `ble_off_s`.

Two timings worth reading off the chart. A **session is not bounded by the
window** - the deadline is only consulted at the top of the serve loop, so a
phone that stays connected keeps the board up indefinitely. And the
**linger** after a disconnect is spent advertising rather than merely awake,
so the phone can come straight back.

### The deep-sleep duty cycle

`adv_window_s = 15`, `sleep_interval_s = 45`. The chip goes away; the GPS
does not.

```mermaid
gantt
    title Deep-sleep duty cycle - everything stops except the ungated GPS
    dateFormat X
    axisFormat %M:%S

    section Board state
    Boot (cold)             :done,   t1, 0, 4s
    Advertise               :active, t2, 4, 15s
    Park                    :crit,   t3, 19, 1s
    Deep sleep 45 s         :done,   t4, 20, 45s
    Boot (wake check)       :done,   t5, 65, 4s
    Advertise               :active, t6, 69, 3s
    Connected               :crit,   t7, 72, 28s
    Linger 5 s              :active, t8, 100, 5s
    Park                    :crit,   t9, 105, 1s
    Deep sleep 45 s         :done,   t10, 106, 45s

    section BLE
    reachable by a phone    :active, c1, 4, 15s
    unreachable             :done,   c2, 19, 50s
    reachable by a phone    :active, c3, 69, 36s
    unreachable             :done,   c4, 105, 46s

    section LoRa
    continuous RX           :active, q1, 3, 16s
    beacon                  :milestone, q2, 5, 0s
    cold sleep - 9.3 uA     :done,  q3, 19, 46s
    continuous RX           :active, q4, 68, 37s
    beacon                  :milestone, q5, 70, 0s
    beacon                  :milestone, q6, 90, 0s
    cold sleep - 9.3 uA     :done,  q7, 105, 46s

    section GPS
    acquiring - ~30 mA even asleep :active, p1, 1, 150s

    section SD card
    logging every fix       :active, e1, 1, 18s
    powered but idle        :done,   e2, 19, 46s
    logging every fix       :active, e3, 66, 39s
    powered but idle        :done,   e4, 105, 46s

    section USB console
    alive                   :active, v1, 0, 19s
    dead - the chip is off  :done,   v2, 19, 46s
    alive                   :active, v3, 65, 40s
    dead - the chip is off  :done,   v4, 105, 46s

    section J5 panel
    refreshing              :active, n1, 4, 15s
    blanked by Park         :done,   n2, 19, 46s
    refreshing              :active, n3, 69, 36s
    blanked by Park         :done,   n4, 105, 46s
```

The GPS lane running unbroken through both sleeps is the board fact that
makes deep sleep worth ~30 mA and not less - the MAX-M10 has no rail to cut.

`Park` is drawn a second wide to stay legible and is normally one 10 ms pass
of the hardware loop. It only becomes long when a beacon is already in
flight, which it then waits out: 289 ms at the defaults, and up to 9.7 s at
the slowest settings the config accepts.

The SD lane is drawn stopping at `Park`, and that is worse than it looks.
`log_position` buffers into RAM and only `sdlog.poll` writes it out, on a
5 s cadence - but `PrepareSleep` does not flush, and the `standby` gate it
sets skips the poll for the rest of the pass. So every deep sleep discards
whatever had not reached the card: 0-5 s of fixes, once per cycle. On a
15 s window at 1 Hz that is up to a third of each wake's log.

### Inside one connected window

What a session can be doing, and what each thing suspends while it runs.

```mermaid
gantt
    title One session - notifications transfers and the interlocks between them
    dateFormat X
    axisFormat %Ss

    section Session
    connected               :crit, x1, 0, 52s
    roster replayed - settings and radio config published :milestone, x2, 0, 0s
    disconnect              :milestone, x3, 52, 0s

    section BLE out
    position notify every 1000 ms :active, y1, 0, 52s
    remote reports pushed as heard :active, y2, 0, 52s
    status lines - a transfer does not gate these :active, y3, 0, 52s

    section USB console
    firmware text            :active, y4, 0, 20s
    quiet - the transfer owns the IN FIFO :done, y5, 20, 15s
    firmware text            :active, y6, 35, 17s

    section LoRa
    beacon - notifications hold :milestone, z1, 10, 0s
    beacon declined - a transfer owns the board :milestone, z2, 30, 0s
    beacon                   :milestone, z3, 40, 0s

    section Bulk transfer
    config push - console quiet beacon held :crit, w1, 20, 15s
    ApplyConfig - radio re-init GPS re-pushed RADIO.CFG rewritten :active, w2, 35, 2s

    section Commanded sleep
    CFG_SLEEP_NOW written    :milestone, k1, 50, 0s
    ack leaves then the session ends :active, k2, 50, 2s
    Park then deep sleep     :crit, k3, 52, 1s
```

Three interlocks show up here, and all three exist for the same reason -
one board, one supply, one USB FIFO:

- A beacon in flight holds BLE notifications (`state::radio_busy`), because
  22 dBm of LoRa PA beside a 2.4 GHz radio is a supply problem.
- A bulk transfer holds the beacon *and* silences the console
  (`state::transfer_active`), because the console and the transfer's ack
  frames share the USB Serial/JTAG IN FIFO with no arbitration. Only the
  console half is gated: `status_println!` still queues every line to the
  log characteristic, so a phone watching the log sees what the console
  cannot show it.
- A commanded sleep is taken after the session ends, not under it, so the
  ack has left before the board disappears.

### Every state, and what is possible in it

| State | Entered by | Connectable | LoRa | GPS | SD log | USB console | Leaves when | Draw |
|-|-|-|-|-|-|-|-|-|
| **Boot** | power-on, reset, OTA reboot | no | init | configured | mounted | yes | init done (never measured) | ~126 mA |
| **Boot (wake check)** | deep-sleep timer | no | init | configured | mounted | yes | init done (never measured) | ~126 mA |
| **Advertise** | boot, or a spent BLE-down period | yes | beacon + RX | tracking | yes | yes | a central connects, the window expires, or `SLEEP_NOW` | ~126-130 mA |
| **Connected** | a central accepts | in session | beacon + RX | tracking | yes | yes | disconnect, or `CFG_SLEEP_NOW` | ~130 mA |
| **Linger** | disconnect | yes | beacon + RX | tracking | yes | yes | 5 s, or the phone returns | ~126-130 mA |
| **BLE down** | window spent with `ble_off_s` set | **no** | beacon + RX | tracking | yes | yes | `ble_off_s` elapses, or USB `SLEEP` | **60 mA** |
| **Park** | any path into deep sleep | no | going to cold sleep | tracking | **not flushed** | yes | radio parked, or the TX budget expires | ~126 mA |
| **Deep sleep** | window spent with `sleep_interval_s` set, or a commanded sleep | no | cold sleep | **still acquiring** | no | **no** | the timer fires - a full reset | **~30 mA** |

Three of these are modal rather than positional - they overlay whichever
state the board is in:

| Overlay | Set by | What it changes |
|-|-|-|
| **Radio standby** (`PFLAG_WIO_SLEEP`) | `CFG_WIO_SLEEP`, survives deep sleep | The hardware loop skips GPS, beacon and telemetry entirely and polls at 20 Hz. Only BLE and the console stay alive. |
| **GPS backup** (`PFLAG_GPS_SLEEP`) | `CFG_GPS_SLEEP`, survives deep sleep | The receiver is parked with `UBX-RXM-PMREQ`. It loses its settings, so the loop re-pushes them when sentences resume. |
| **Transfer active** | a bulk op over BLE or USB | Beacon held off, console quiet, and the transfer is bounded so a host that walks away cannot hold the board. |

And one that does nothing here: `PFLAG_PWR_OFF` and `Action::Rail` are the
old board's GPS/LoRa rail switch. This carrier has no such rail, so the
firmware logs the request and honors nothing.

## What the board forces on the firmware

Board facts that firmware cannot work around, from the carrier design:
there is no rail to cut (GPS and SD sit on +3V3, so `Action::Rail` logs and
does nothing), GPS `EXTINT` is not routed (backup mode wakes on UART traffic
instead), `TIMEPULSE` is unconnected so there is no PPS discipline, and
there is no battery sense divider so telemetry cannot report cell voltage.

The Wi-Fi/BT RF port reaches test point BLE1 and stops there, so the 2.4 GHz
side has no antenna on the board as drawn - BLE works at bench range on
board parasitics.
