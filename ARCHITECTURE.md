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
    }
    class GattSession {
        publish settings on connect
        notify position and telemetry
        apply_config()
    }
    class HardwareTask {
        owns radio, gps and card
        poll_recv() beacon() log()
    }
    class State {
        <<snapshot, not a channel>>
        set_position() take_position()
        radio_busy()
        request() take_request()
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

    class Sx1262Driver {
        init() send() poll_recv()
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
    Firmware *-- HardwareTask
    ServeTask --> GattSession
    GattSession <--> State
    HardwareTask <--> State

    MidairProto *-- SessionPolicy
    MidairProto *-- Roster
    MidairProto *-- LoraCodec
    MidairProto *-- RadioConfig
    MidairProto *-- LinkCodec

    GattSession --> SessionPolicy
    GattSession --> Roster
    HardwareTask --> Sx1262Driver
    HardwareTask --> MaxM10
    HardwareTask --> SdCardHw
    Sx1262Driver ..> LoraCodec
    Sx1262Driver --> Sx1262Cmds
    Sx1262Driver --> RadioConfig : live settings
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
    Gatt->>App: notify settings (so controls populate)

    loop every NOTIFY_INTERVAL_MS
        Gatt->>St: radio_busy()?
        alt radio has the air
            Gatt-->>Gatt: skip this tick
        else
            Gatt->>St: take_position(), telemetry()
            Gatt->>App: notify position, telemetry
        end
    end

    App->>Gatt: write config id
    Gatt->>Gatt: session::apply (host-tested policy)
    Gatt->>St: request(GpsSleep | RadioStandby)
    Gatt->>App: notify ack, then fresh settings
    St->>Hw: take_request() on the next loop
    Note over Gatt,Hw: the ack always holds - there is no<br/>second chip that can fail to answer

    App->>Serve: disconnect
    Serve->>Serve: advertise again
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

## What is not ported yet

The single-MCU firmware is not yet at parity with what the two-MCU pair did.
Outstanding, from `PORT-WIO-S3.md`:

- **Bulk transfer.** The characteristic is declared so the service shape
  matches, but the handler is not written, and neither is the USB console
  that `wio-config` needs.
- **Deep sleep**, with its nvs-backed settings. `Stored` sits in a plain
  static that resets with the board.
- **The remote-node roster replay** on connect.
- **OTA.** The ESP-IDF bootloader does two-slot OTA with rollback and
  `esp-bootloader-esp-idf` is already a dependency; nothing drives it.
- **Per-board BLE addresses.** The C6 derived one from its eFuse MAC.

Board facts that firmware cannot work around, from the carrier design:
there is no rail to cut (GPS and SD sit on +3V3, so `Action::Rail` logs and
does nothing), GPS `EXTINT` is not routed (backup mode wakes on UART traffic
instead), `TIMEPULSE` is unconnected so there is no PPS discipline, and
there is no battery sense divider so telemetry cannot report cell voltage.
