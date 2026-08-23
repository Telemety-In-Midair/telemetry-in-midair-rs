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
        detected not configured
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

## What the board forces on the firmware

Board facts that firmware cannot work around, from the carrier design:
there is no rail to cut (GPS and SD sit on +3V3, so `Action::Rail` logs and
does nothing), GPS `EXTINT` is not routed (backup mode wakes on UART traffic
instead), `TIMEPULSE` is unconnected so there is no PPS discipline, and
there is no battery sense divider so telemetry cannot report cell voltage.

The Wi-Fi/BT RF port reaches test point BLE1 and stops there, so the 2.4 GHz
side has no antenna on the board as drawn - BLE works at bench range on
board parasitics.
