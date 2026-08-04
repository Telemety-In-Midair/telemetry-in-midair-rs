# Architecture

UML views of the telemetry-in-midair board: what the parts are, how the
ESP32-C6 moves between deep sleep, advertising and a BLE connection, what a
session actually does, and the modes the WIO-E5 runs in.

## Structure

Two MCUs on one board. The ESP32-C6 is the BLE face and the power master
(it owns the GPS/LoRa rail); the WIO-E5 does GPS, LoRa and SD. They talk
over a framed UART link. Everything host-testable - the session policy, the
roster, the link and LoRa codecs, the radio config parser - lives in the
shared `midair-proto` crate.

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
        esp-upload
        wio-upload
        wio-config
        fw-upload
    }
    class RemoteNode {
        <<other board, 915 MHz>>
        broadcasts position or ping
        repeater forwards hops
    }

    class EspFirmware {
        <<ESP32-C6, embassy>>
        BLE face and power master
        deep sleeps when unattended
    }
    class ServeTask {
        advertise()
        accept one central()
        enter_deep_sleep()
    }
    class GattSession {
        notify tick position and telemetry
        push remote reports on arrival
        stream WIO log lines
        apply_config()
        handle_bulk()
    }
    class GpsService {
        <<GATT service>>
        position read notify
        config write
        ack notify
        telemetry read notify
        bulk write
        remote read notify
        node_ping read notify
        log read notify
        settings read notify
        radio_config read notify
    }
    class LinkTask {
        drive UART0 to the WIO
        FrameParser feed()
        handle_link_frame()
    }
    class HeartbeatTask {
        cmd PING every 3 s
        log link up and down
        re-apply sleep flags on link up
    }
    class UsbTask {
        usb PING INFO BULK
        console shares the port
    }
    class LedTask {
        wake blink, info burst, fw toggle
    }
    class PersistedSettings {
        RTC RAM copy survives deep sleep
        nvs flash record survives a flat cell
        read from flash only on a cold boot
    }

    class MidairProto {
        <<no_std, cargo test on host>>
    }
    class SessionPolicy {
        apply(write) Outcome
        clamp and ack
        decide save to flash
    }
    class Stored {
        sleep_interval_s
        flags PWR_OFF WIO_SLEEP GPS_SLEEP
        adv_window_s
        rail_at_boot(woke_from_sleep)
        settings()
    }
    class Window {
        ends_ms
        next(now, interval) Next
        remaining_ms(now)
        linger(now)
    }
    class Roster {
        newest report per node, up to 8
        record() take_dirty() replay()
        forget after 30 min
    }
    class LinkCodec {
        sync byte, crc, cmd and msg ids
        FrameParser FrameBuf
        Telemetry
    }
    class LoraCodec {
        encode_position(fields mask)
        Ping encode decode
    }
    class RadioConfig {
        radio network beacon sd gps keys
        parse_bytes() encode()
    }

    class WioFirmware {
        <<WIO-E5, RTIC>>
        one priority 1 run task
        USART2 RX ISR fills a ring buffer
    }
    class RunTask {
        drain link frames
        poll GPS, beacon, receive, log
        periodic telemetry
        feed the watchdog
    }
    class EspLink {
        send() send_ack() send_nak()
        peer_busy()
    }
    class Node {
        broadcast()
        poll() dedup and repeat
        role leaf repeater tx_only rx_only
    }
    class GpsDriver {
        NMEA parse RMC and GGA
        configure() UBX-CFG-VALSET
        sleep() UBX-RXM-PMREQ
        wake() EXTINT pulse
    }
    class SdLog {
        GPSLOG.CSV append
        RADIO.CFG read and write
        remount retry once a minute
    }
    class FwUpdate {
        stream image into DFU
        verify crc then reset
    }
    class CfgTransfer {
        collect TOML, verify crc
    }
    class Bootloader {
        <<separate binary>>
        page swap ACTIVE and DFU
        revert if boot is never confirmed
    }

    class PowerRail {
        <<AP2112K on GPIO2>>
        GPS and LoRa supply
    }
    class WioReset {
        <<open drain GPIO6>>
        wake fallback pulse
    }
    class MaxM10 {
        <<GPS receiver>>
    }
    class Sx1262 {
        <<SubGHz radio in module>>
    }
    class SdCardHw {
        <<FAT16 or FAT32, optional>>
    }

    GpsGuiApp ..> GpsService : GATT
    HostTools ..> UsbTask : framed bulk over USB
    HostTools ..> GpsGuiApp : same bulk ops over BLE

    EspFirmware *-- ServeTask
    EspFirmware *-- LinkTask
    EspFirmware *-- HeartbeatTask
    EspFirmware *-- UsbTask
    EspFirmware *-- LedTask
    EspFirmware *-- PersistedSettings
    ServeTask --> Window : one budget per wake
    ServeTask --> GattSession : one central at a time
    ServeTask --> PowerRail : raise on connect, cut before sleep
    ServeTask --> LedTask : wake blink
    GattSession --> GpsService : set and notify
    GattSession --> SessionPolicy : config writes
    GattSession --> Roster : replay on connect, push on arrival
    GattSession ..> LinkTask : queue frames and await acks
    HeartbeatTask ..> LinkTask
    UsbTask ..> LinkTask
    PersistedSettings --> Stored : holds

    MidairProto *-- SessionPolicy
    MidairProto *-- Roster
    MidairProto *-- LinkCodec
    MidairProto *-- LoraCodec
    MidairProto *-- RadioConfig
    SessionPolicy *-- Stored
    SessionPolicy *-- Window

    LinkTask <..> EspLink : UART 115200, crc framed
    LinkTask ..> LinkCodec
    EspLink ..> LinkCodec

    WioFirmware *-- RunTask
    WioFirmware *-- EspLink
    WioFirmware *-- Node
    WioFirmware *-- GpsDriver
    WioFirmware *-- SdLog
    WioFirmware *-- FwUpdate
    WioFirmware *-- CfgTransfer
    RunTask --> RadioConfig : live settings
    Node ..> LoraCodec
    FwUpdate ..> Bootloader : reset into the swap
    CfgTransfer --> SdLog : write RADIO.CFG
    CfgTransfer --> RunTask : apply live

    PowerRail --> WioFirmware : powers
    PowerRail --> MaxM10 : powers
    WioReset --> WioFirmware : hard reset
    GpsDriver --> MaxM10
    Node --> Sx1262
    SdLog --> SdCardHw
    Sx1262 <..> RemoteNode : broadcasts and hops
```

## ESP32-C6 power and BLE lifecycle

The whole sleep story. Sleep is off until an app writes `0x13`; while it is
set the board wakes on the timer, advertises for the `0x14` window and goes
back down. The window is a deadline for the wake, not a per-attempt
timeout - no retry can push it out, only a disconnect (which buys a 5 s
linger).

```mermaid
stateDiagram-v2
    direction TB

    [*] --> ColdBoot

    ColdBoot: Cold boot
    ColdBoot: RTC RAM magic absent, so read the nvs record
    ColdBoot: rail follows the stored pwr_en setting

    TimerWake: Timer wake
    TimerWake: RTC RAM still valid, no flash read
    TimerWake: rail stays dark whatever pwr_en says
    TimerWake: one long D2 blink

    state Awake {
        direction TB

        Advertising: Advertising
        Advertising: connectable, name GPS-C6
        Advertising: budget is the wake deadline

        state Connected {
            direction TB
            RaiseRail: Raise the rail if pwr_en
            RaiseRail: WIO boot time plus GPS cold TTFF from here
            Publish: Publish settings and radio_config, replay the roster
            Streaming: Notify position and telemetry on the interval
            Streaming: push remote reports and log lines as they arrive
            Streaming: defer while the WIO flags its radio busy
            Writing: Config write, clamp, ack, save, republish settings
            Bulk: Bulk transfer, TOML or WIO firmware, into the link

            [*] --> RaiseRail
            RaiseRail --> Publish
            Publish --> Streaming
            Streaming --> Writing : write to config char
            Writing --> Streaming : ack notified
            Streaming --> Bulk : OP_BEGIN
            Bulk --> Streaming : OP_END or OP_ABORT
        }

        Linger: Linger
        Linger: keep advertising 5 s so the phone can come straight back

        [*] --> Advertising
        Advertising --> Connected : central accepted
        Connected --> Linger : disconnected
        Linger --> Connected : central returns
        Linger --> Advertising : budget still open
    }

    DeepSleep: Deep sleep
    DeepSleep: rail off, GPIO2 and GPIO6 pad held
    DeepSleep: about 1000x cheaper than advertising

    ColdBoot --> Awake
    TimerWake --> Awake
    Advertising --> Advertising : advertise failed or connect attempt failed, 200 ms pause, deadline unchanged
    Advertising --> DeepSleep : window spent and sleep_interval_s greater than 0
    Linger --> DeepSleep : linger spent and sleep is on
    DeepSleep --> TimerWake : after sleep_interval_s, 5 s to 5 min

    note right of DeepSleep
        The timer is the only wake source.
        No radio, no GPIO, no button, so
        sleep_interval_s bounds how long
        the board can be unreachable.
        A connection outlives the window,
        so sleep is only ever entered from
        Advertising or Linger.
    end note

    note right of Awake
        With sleep_interval_s = 0 the
        board never leaves this state:
        it advertises indefinitely and
        the window means nothing.
    end note
```

## A BLE session end to end

One wake that somebody answers. Note where the rail comes up: not on the
wake, only on the connect, which is why an app should expect the WIO's boot
and a GPS cold fix after connecting to a sleeping board.

```mermaid
sequenceDiagram
    autonumber
    actor Phone as gps-gui-rs
    participant ESP as ESP32-C6
    participant Rail as GPS and LoRa rail
    participant WIO as WIO-E5
    participant GPS as MAX-M10
    participant Air as 915 MHz LoRa

    Note over ESP: Timer wake. RTC RAM holds the settings,<br/>rail stays dark, long D2 blink.
    ESP->>ESP: Window::new(now, adv_window_s)

    alt nobody connects before the window ends
        ESP-->>ESP: enter_deep_sleep(interval_s)
        Note over ESP,Rail: rail already off, pads held, timer armed
    else a central connects
        Phone->>ESP: connect
        ESP->>Rail: drive high if pwr_en
        Rail->>WIO: power
        Rail->>GPS: power
        ESP->>Phone: notify settings and radio_config
        ESP->>Phone: replay roster, each report with its age_s

        WIO->>ESP: msg LOG "wio vN up, node A"
        WIO->>ESP: msg CONFIG, the live RADIO.CFG snapshot
        ESP->>Phone: notify radio_config

        loop every 3 s
            ESP->>WIO: cmd PING
            WIO-->>ESP: ACK with firmware version
        end

        loop every notify interval
            WIO->>ESP: msg POSITION src 0 and msg STATUS
            ESP->>Phone: notify position and telemetry
        end

        Note over WIO,Air: beacon slot, a position with a fix,<br/>otherwise a 4 byte ping
        WIO->>ESP: msg RADIO_BUSY 1
        ESP->>ESP: hold BLE notifications
        WIO->>Air: broadcast
        WIO->>ESP: msg RADIO_BUSY 0

        Air->>WIO: remote position or ping
        WIO->>ESP: msg POSITION src N or msg PING
        ESP->>Phone: notify remote or node_ping on arrival

        Phone->>ESP: write config 0x13 sleep interval, 0x14 window
        ESP->>ESP: session::apply, clamp, store in RTC RAM, nvs_save
        ESP->>Phone: notify ack with the applied value
        ESP->>Phone: notify settings

        Phone->>ESP: write config 0x11 WIO soft sleep
        ESP->>WIO: cmd WIO_SLEEP 1
        WIO-->>ESP: ACK
        ESP->>Phone: notify ack
        Note over ESP,WIO: a wake that times out falls back to<br/>a reset pulse on GPIO6

        opt firmware or radio config push
            Phone->>ESP: bulk OP_BEGIN, OP_DATA chunks, OP_END
            ESP->>WIO: cmd FW or CFG frames, one transfer at a time
            WIO-->>ESP: ACK per chunk with the next expected seq
            ESP->>Phone: notify ack per op
        end

        Phone->>ESP: disconnect
        ESP->>ESP: window.linger(now), 5 s of advertising
        ESP-->>ESP: enter_deep_sleep(interval_s)
        ESP->>Rail: drive low
    end
```

## WIO-E5 modes

The WIO has no deep sleep of its own - the ESP cutting the rail is what
makes it cheap. What it does have is soft sleep (radio to standby, link
still answering), GPS backup mode, and two modes that take over the loop.

```mermaid
stateDiagram-v2
    direction TB

    [*] --> Init

    Init: Init
    Init: watchdog first, confirm the boot image
    Init: RADIO.CFG from SD, else the flash backup, else defaults
    Init: push GPS settings, init the radio, announce over the link

    Running: Running
    Running: poll GPS, beacon on the interval, receive and repeat
    Running: log fixes to GPSLOG.CSV, telemetry every 5 s
    Running: feed the watchdog every millisecond

    SoftSleep: Soft sleep
    SoftSleep: radio in standby, only the ESP link is served

    Transfer: Bulk transfer
    Transfer: firmware or config owns the loop
    Transfer: no GPS, no beacon, no SD work

    Swap: Bootloader swap
    Swap: page by page ACTIVE and DFU, power fail safe

    Init --> Running
    Running --> SoftSleep : cmd WIO_SLEEP 1
    SoftSleep --> Running : cmd WIO_SLEEP 0 or a reset pulse
    Running --> Transfer : cmd FW_BEGIN or CFG_BEGIN
    Transfer --> Running : CFG_END applied, FW_ABORT, or a 5 s stall
    Transfer --> Swap : FW_END verified, then reset
    Swap --> Init : new image, reverts if the boot is never confirmed
    Running --> [*] : rail cut by the ESP

    state GpsPower {
        direction LR
        Full: Tracking
        Backup: Backup mode
        Full --> Backup : cmd GPS_SLEEP 1, UBX-RXM-PMREQ
        Backup --> Full : cmd GPS_SLEEP 0, EXTINT pulse, settings re-pushed
    }

    note right of GpsPower
        Independent of the modes above.
        Backup loses the RAM configuration
        layer, so the run loop retries the
        settings push once the module talks.
    end note
```
