//! Radio configuration: the `RADIO.CFG` file format and its parser.
//!
//! The file is TOML-shaped but the name is not `.toml`: it lives in the root
//! of a FAT card, where 8.3 short names allow only a three-character
//! extension.
//!
//! The firmware loads this from the SD card at boot (`RADIO.CFG`) and/or
//! receives it over BLE. No TOML crate runs on the target, so this is a
//! small no_std
//! parser for the subset the file needs: `key = value` pairs with integer,
//! boolean and quoted-string values, `#` comments, and `[section]` headers
//! (accepted and ignored - keys are unique across sections).
//!
//! Example file:
//!
//! ```toml
//! [radio]
//! frequency_hz = 915000000
//! spreading_factor = 7      # 5-12
//! bandwidth_khz = 125       # 62, 125, 250, 500
//! coding_rate = 5           # 4/5 .. 4/8
//! power_dbm = 22            # -9 .. 22
//! rx_boost = true           # boosted RX gain, ~+2 dB for more RX current
//! dio2_rf_switch = true     # radio drives its own antenna switch on DIO2
//!
//! [mesh]
//! address = 1               # 1-255
//! role = "leaf"             # leaf | repeater | tx_only | rx_only
//! max_hops = 1              # retransmissions allowed per broadcast
//!
//! [beacon]
//! interval_s = 10           # position broadcast period
//! ```

use crate::ble;
use crate::lora;

/// Which halves of the air interface a node uses.
///
/// Position reporting is one-way traffic, so a node does not have to do
/// both halves. A tracker that is only ever reported *on* can leave its
/// receiver off; a base station that only collects reports never needs to
/// transmit. Each saves the power the unused half costs, and on a tracker
/// that is the larger saving by far: continuous RX draws current every
/// second between beacons, while a beacon is milliseconds of TX.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// Originates its own broadcasts and receives everyone else's, but
    /// never retransmits. A network of nothing but leaves works: every
    /// node hears every other one that is in direct range.
    Leaf,
    /// A leaf that additionally retransmits frames still carrying hops,
    /// extending the network past one radio horizon.
    Repeater,
    /// Beacons its own position and nothing else. The receiver is never
    /// enabled, so this node hears no one, repeats nothing, and reports no
    /// peers - it exists only in other nodes' logs.
    TxOnly,
    /// Listens and reports what it hears, never transmitting. Its own
    /// beacon is off whatever `beacon_interval_s` says, since a beacon is a
    /// transmission.
    RxOnly,
}

impl Role {
    /// Whether this node ever puts a frame on the air.
    pub fn transmits(self) -> bool {
        !matches!(self, Role::RxOnly)
    }

    /// Whether this node enables its receiver at all.
    pub fn receives(self) -> bool {
        !matches!(self, Role::TxOnly)
    }

    /// Whether this node retransmits other nodes' frames.
    pub fn repeats(self) -> bool {
        matches!(self, Role::Repeater)
    }

    /// Compact value used in the [`RadioConfig`] read-back blob.
    pub fn to_wire(self) -> u8 {
        match self {
            Role::Leaf => 0,
            Role::Repeater => 1,
            Role::TxOnly => 2,
            Role::RxOnly => 3,
        }
    }

    /// Inverse of [`to_wire`](Self::to_wire); `None` for an unknown value.
    pub fn from_wire(v: u8) -> Option<Self> {
        Some(match v {
            0 => Role::Leaf,
            1 => Role::Repeater,
            2 => Role::TxOnly,
            3 => Role::RxOnly,
            _ => return None,
        })
    }

    /// The `role` config-file spelling, so a reader can render it back into
    /// TOML that the parser accepts unchanged.
    pub fn as_str(self) -> &'static str {
        match self {
            Role::Leaf => "leaf",
            Role::Repeater => "repeater",
            Role::TxOnly => "tx_only",
            Role::RxOnly => "rx_only",
        }
    }
}

/// Supply voltage the radio drives the TCXO at (`SetTcxoMode` trim field).
///
/// This is a property of the board, not a preference. Setting a value the
/// hardware does not expect stops the oscillator starting, which takes the
/// radio with it.
///
/// On the Wio-S3 the pin does double duty and the floor is not the TCXO's.
/// DIO3 also supplies the SKY13453-385LF antenna switch, whose VDD is
/// specified 2.5 - 3.5 V, so only "2.7", "3.0" and "3.3" are usable at all;
/// below 2.5 V the switch's own truth table calls the part undefined. See
/// [`RadioConfig::dio2_rf_switch`] for what an undefined switch does to a
/// transmitting PA.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TcxoVolts {
    V1_6,
    V1_7,
    V1_8,
    V2_2,
    V2_4,
    V2_7,
    V3_0,
    V3_3,
}

impl TcxoVolts {
    /// `SetTcxoMode` trim enum value.
    pub fn trim(self) -> u8 {
        match self {
            TcxoVolts::V1_6 => 0x0,
            TcxoVolts::V1_7 => 0x1,
            TcxoVolts::V1_8 => 0x2,
            TcxoVolts::V2_2 => 0x3,
            TcxoVolts::V2_4 => 0x4,
            TcxoVolts::V2_7 => 0x5,
            TcxoVolts::V3_0 => 0x6,
            TcxoVolts::V3_3 => 0x7,
        }
    }

    /// Inverse of [`trim`](Self::trim); `None` for an unknown value. Doubles as
    /// the read-back blob decoder, since the blob carries the trim value.
    pub fn from_trim(v: u8) -> Option<Self> {
        Some(match v {
            0x0 => TcxoVolts::V1_6,
            0x1 => TcxoVolts::V1_7,
            0x2 => TcxoVolts::V1_8,
            0x3 => TcxoVolts::V2_2,
            0x4 => TcxoVolts::V2_4,
            0x5 => TcxoVolts::V2_7,
            0x6 => TcxoVolts::V3_0,
            0x7 => TcxoVolts::V3_3,
            _ => return None,
        })
    }

    /// The `tcxo_volts` config-file spelling.
    pub fn as_str(self) -> &'static str {
        match self {
            TcxoVolts::V1_6 => "1.6",
            TcxoVolts::V1_7 => "1.7",
            TcxoVolts::V1_8 => "1.8",
            TcxoVolts::V2_2 => "2.2",
            TcxoVolts::V2_4 => "2.4",
            TcxoVolts::V2_7 => "2.7",
            TcxoVolts::V3_0 => "3.0",
            TcxoVolts::V3_3 => "3.3",
        }
    }
}

/// GPS receiver power mode (u-blox M10 `CFG-PM-OPERATEMODE`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerMode {
    /// Continuous tracking, lowest fix latency.
    Full,
    /// Power-save on/off: acquire a fix, then power down until the next
    /// update period.
    PsmOnOff,
    /// Power-save cyclic tracking: stays in a reduced-power tracking loop.
    PsmCyclic,
}

impl PowerMode {
    /// `CFG-PM-OPERATEMODE` enum value.
    pub fn operate_mode(self) -> u8 {
        match self {
            PowerMode::Full => 0,
            PowerMode::PsmOnOff => 1,
            PowerMode::PsmCyclic => 2,
        }
    }

    /// Inverse of [`operate_mode`](Self::operate_mode); `None` for an unknown
    /// value. Also the read-back blob decoder.
    pub fn from_operate_mode(v: u8) -> Option<Self> {
        Some(match v {
            0 => PowerMode::Full,
            1 => PowerMode::PsmOnOff,
            2 => PowerMode::PsmCyclic,
            _ => return None,
        })
    }

    /// The `power_mode` config-file spelling.
    pub fn as_str(self) -> &'static str {
        match self {
            PowerMode::Full => "full",
            PowerMode::PsmOnOff => "psmoo",
            PowerMode::PsmCyclic => "psmct",
        }
    }
}

/// GPS navigation dynamic model (u-blox M10 `CFG-NAVSPG-DYNMODEL`). Only the
/// subset useful for this tracker is exposed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynModel {
    Portable,
    Stationary,
    Pedestrian,
    Automotive,
    Sea,
    /// Airborne with < 1 g acceleration.
    Airborne1g,
    /// Airborne with < 2 g acceleration.
    Airborne2g,
    /// Airborne with < 4 g acceleration.
    Airborne4g,
}

impl DynModel {
    /// `CFG-NAVSPG-DYNMODEL` enum value.
    pub fn dynmodel(self) -> u8 {
        match self {
            DynModel::Portable => 0,
            DynModel::Stationary => 2,
            DynModel::Pedestrian => 3,
            DynModel::Automotive => 4,
            DynModel::Sea => 5,
            DynModel::Airborne1g => 6,
            DynModel::Airborne2g => 7,
            DynModel::Airborne4g => 8,
        }
    }

    /// Inverse of [`dynmodel`](Self::dynmodel); `None` for an unknown value.
    /// Also the read-back blob decoder. Note 1 is unused by the u-blox map.
    pub fn from_dynmodel(v: u8) -> Option<Self> {
        Some(match v {
            0 => DynModel::Portable,
            2 => DynModel::Stationary,
            3 => DynModel::Pedestrian,
            4 => DynModel::Automotive,
            5 => DynModel::Sea,
            6 => DynModel::Airborne1g,
            7 => DynModel::Airborne2g,
            8 => DynModel::Airborne4g,
            _ => return None,
        })
    }

    /// The `dynamic_model` config-file spelling.
    pub fn as_str(self) -> &'static str {
        match self {
            DynModel::Portable => "portable",
            DynModel::Stationary => "stationary",
            DynModel::Pedestrian => "pedestrian",
            DynModel::Automotive => "automotive",
            DynModel::Sea => "sea",
            DynModel::Airborne1g => "airborne1g",
            DynModel::Airborne2g => "airborne2g",
            DynModel::Airborne4g => "airborne4g",
        }
    }
}

/// GPS receiver configuration (u-blox M10, applied via UBX-CFG-VALSET).
///
/// The constellation and power defaults match the M10 factory set, so an
/// absent `[gps]` section leaves the module at its out-of-box behavior.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GpsConfig {
    pub gps_enabled: bool,
    pub glonass_enabled: bool,
    pub galileo_enabled: bool,
    pub beidou_enabled: bool,
    pub qzss_enabled: bool,
    pub sbas_enabled: bool,
    /// Receiver power mode.
    pub power_mode: PowerMode,
    /// Measurement period in milliseconds (25-10000, i.e. 40 Hz down to
    /// 0.1 Hz). The nav solution runs at the same rate.
    pub meas_rate_ms: u16,
    /// Navigation dynamic model.
    pub dyn_model: DynModel,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            gps_enabled: true,
            // GLONASS is off in the M10 default concurrent set (GPS +
            // Galileo + BeiDou + QZSS + SBAS).
            glonass_enabled: false,
            galileo_enabled: true,
            beidou_enabled: true,
            qzss_enabled: true,
            sbas_enabled: true,
            power_mode: PowerMode::Full,
            meas_rate_ms: 1000,
            dyn_model: DynModel::Portable,
        }
    }
}

/// The duty cycle: how long the board is reachable, and how long it is not.
///
/// These are the only settings in this file that the board also keeps a copy
/// of - in RTC RAM, so they survive a deep sleep, and in flash, so they
/// survive a flat cell (see [`crate::session::Stored`]). An app can change
/// them live over BLE, which nothing else here can do.
///
/// That is why every field is an `Option`. Elsewhere in this file an absent
/// key means "the default"; here it means "leave the board's current value
/// alone", so that pushing an unrelated radio change does not silently undo a
/// duty cycle somebody set from the app. A key that *is* present wins at the
/// next boot, because the file is what survives a reflash and the RTC copy is
/// not.
///
/// The ranges are the ones [`crate::ble`] clamps a live write to, but a file
/// is edited by hand and read once, so an out-of-range value is reported as
/// [`ConfigError::OutOfRange`] rather than quietly clamped.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PowerConfig {
    /// Seconds the BLE controller stays powered down between advertising
    /// windows; 0 never takes it down. This is the largest lever the
    /// firmware has - the controller is most of the board's current and its
    /// lifetime is the only thing that changes it.
    pub ble_off_s: Option<u32>,
    /// Seconds each advertising window lasts; 0 means the firmware default.
    pub adv_window_s: Option<u32>,
    /// Seconds between deep-sleep wake checks; 0 never deep-sleeps.
    pub sleep_interval_s: Option<u32>,
}

/// Parsed and validated radio configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadioConfig {
    /// RF frequency in Hz.
    pub frequency_hz: u32,
    /// LoRa spreading factor (5-12).
    pub spreading_factor: u8,
    /// LoRa bandwidth in kHz (62 means 62.5).
    pub bandwidth_khz: u16,
    /// LoRa coding rate denominator: 5-8 for 4/5..4/8.
    pub coding_rate: u8,
    /// TX power in dBm (-9..22 on the SX1262 high-power PA).
    pub power_dbm: i8,
    /// Receiver boosted gain (SX126x `RxGain` register). Roughly +2 dB of
    /// sensitivity for a few mA more while listening; the chip powers up
    /// with it off.
    ///
    /// Only two of the register's four settings are documented (power
    /// saving and boosted), so this is a bool rather than an enum - the
    /// intermediate values have no specified behavior to expose.
    pub rx_boost: bool,
    /// This node's address (1-255). It travels in every frame this node
    /// originates and is how receivers tell one sender's positions from
    /// another's, so it must be unique among the nodes that transmit: two
    /// senders sharing one are mutually deaf, each dropping the other's
    /// frames as an echo of its own.
    ///
    /// A [`Role::RxOnly`] node originates nothing, so its address never
    /// reaches the air and cannot collide with anyone's. Leaving it at the
    /// default is fine.
    pub address: u8,
    /// Which halves of the air interface this node uses, and whether it
    /// retransmits other nodes' frames.
    pub role: Role,
    /// Retransmissions allowed for a broadcast this node originates, i.e.
    /// the `hops_left` it stamps into the frame. 0 means no repeater will
    /// forward it; 1 covers the usual single-repeater deployment.
    ///
    /// This is a property of the sender, not of the repeaters, so raising
    /// it on one node does not require touching any other.
    pub max_hops: u8,
    /// How long a `(src, id)` pair is remembered for deduplication, in
    /// seconds. A frame seen again inside this window is neither delivered
    /// again nor repeated again, which is what stops a frame reaching a node
    /// by two paths from being handled twice and a repeater pair from
    /// bouncing one between themselves.
    ///
    /// Ids wrap every 256 transmissions, so this has to stay well under the time
    /// that takes at the beacon interval in use, or a node's own sequence
    /// would eventually collide with its remembered history and be suppressed
    /// as a duplicate. At the 20 s default interval that wrap is ~85 min.
    pub dedup_ttl_s: u16,
    /// Broadcast interval in seconds (0 disables the beacon).
    ///
    /// One transmission per interval, carrying a position while the sender
    /// has a fix and a [`crate::lora::Ping`] while it does not - so this is
    /// the period a node is heard on, not the period it has a position on.
    pub beacon_interval_s: u16,
    /// Which [`PositionPacket`](gps_proto::packet::PositionPacket) fields the
    /// beacon puts on the air, as a mask of the `FIELD_*` bits in
    /// [`crate::lora`]. Every extra field is airtime paid on every
    /// transmission, so the default carries position and nothing else.
    ///
    /// The mask travels in the frame, so nodes disagreeing about it is fine:
    /// a receiver decodes whatever the sender chose to include.
    pub beacon_fields: u8,
    /// Whether the SD card is used at all. False stops logging, config
    /// read-back and the card's power draw.
    pub sd_enabled: bool,
    /// Verbose console logging on the ESP: per-frame link traffic and
    /// per-heartbeat detail, on top of the events that are always logged.
    ///
    /// On by default, since the console costs nothing when nobody is
    /// attached to it. Turn it off for a quiet log, or when a tool is
    /// parsing the console and the extra lines are in the way.
    ///
    /// The ESP does not parse this file, so the setting reaches it as
    /// [`crate::link::TELEM_FLAG_VERBOSE`] in the WIO's periodic telemetry.
    pub verbose: bool,
    /// Use the internal DC-DC (SMPS) rather than the LDO, roughly halving
    /// RX and TX current at 3.3 V.
    ///
    /// The SMPS needs the module's inductor fitted, so this is only safe to
    /// leave on for boards that have one - the Wio-S3 does. Turning it off
    /// costs current but is safe anywhere.
    pub dcdc_enabled: bool,
    /// Let the radio drive its own antenna switch from DIO2
    /// (`SetDio2AsRfSwitchCtrl`, opcode 0x9D).
    ///
    /// This is a property of how the module's RF port is wired, not a
    /// tuning choice, and on the Wio-S3 there is only one correct answer.
    /// DIO2 is the VCTL of an SKY13453-385LF between the PA and the LoRa
    /// antenna port; left off, VCTL never rises, the switch parks on the
    /// path that is not the PA, and every transmission ramps +22 dBm into
    /// an isolated port. That is a PA-destroying condition, not a range
    /// problem, which is why [`crate::radiocfg`] defaults it on and the
    /// firmware refuses to honor an off.
    pub dio2_rf_switch: bool,
    /// Supply the radio drives DIO3 at.
    ///
    /// Named for the TCXO because that is the command
    /// (`SetDio3AsTcxoCtrl`), but on the Wio-S3 the same pin is also the
    /// antenna switch's VDD - see [`TcxoVolts`] for the floor that puts
    /// under it.
    pub tcxo_volts: TcxoVolts,
    /// How long the radio waits for the TCXO to stabilize before it will
    /// use the clock, in milliseconds.
    ///
    /// Paid on every wake from sleep, so it is a real duty-cycle cost - but
    /// too short and the radio runs off an oscillator that has not settled,
    /// which shows up as a receiver that works warm and fails cold.
    pub tcxo_startup_ms: u16,
    /// GPS receiver configuration.
    pub gps: GpsConfig,
    /// Duty cycle, i.e. how much of the time the board is reachable.
    pub power: PowerConfig,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 915_000_000,
            // SF12 at BW500 kHz, CR 4/5.
            //
            // The bandwidth is the deliberate part. A 500 kHz signal is wide
            // enough to count as a digital modulation in the 902-928 MHz
            // band, which is the difference between being allowed to sit on
            // one channel and having to hop across fifty. Hopping needs a
            // clock every node agrees on, and the only one here is GPS time -
            // which a node that has never had a fix does not have, and that
            // is precisely the node the no-fix ping exists to keep audible.
            //
            // SF12 buys most of that width back. Against the SF9/BW62.5 this
            // replaces it costs 1.5 dB of sensitivity (-131 against -132.5
            // dBm), and the beacon is shorter on air rather than longer -
            // 289 ms against 330 ms - because the symbol time works out
            // identical at 8.192 ms.
            spreading_factor: 12,
            bandwidth_khz: 500,
            coding_rate: 5,
            power_dbm: 22,
            // On, unlike the chip's power-up state: a couple of dB of
            // sensitivity is worth a few mA on a board whose receiver is
            // already listening continuously, and range is set by the worse
            // of the two directions.
            rx_boost: true,
            address: 1,
            // Leaf by default: repeating is a job you give one well-placed
            // node, not something every node should do to every frame.
            role: Role::Leaf,
            // Allow one repeat, so dropping a repeater into an existing
            // fleet works without reconfiguring the nodes already deployed.
            max_hops: 1,
            dedup_ttl_s: 3,
            // 20 s. The wideband default carries no dwell or duty-cycle
            // ceiling, so this is no longer a budget the modulation has to
            // fit inside - it is a plain trade of battery life and shared air
            // time against how stale a position is allowed to get.
            beacon_interval_s: 20,
            // Position only. Everything else a fix produces is written to
            // the SD log, where a byte costs nothing, rather than spent on
            // air time that has to be paid on every single transmission.
            beacon_fields: crate::lora::FIELDS_DEFAULT,
            sd_enabled: true,
            // The console is free when nothing is reading it, so the
            // detailed build is the one to ship and quieting it is the
            // deliberate choice.
            verbose: true,
            // The three below are the Wio-S3's hardware, not a tuning
            // choice. The module carries the SMPS inductor, wires SX1262
            // DIO2 to the antenna switch's VCTL, and wires DIO3 to that
            // same switch's VDD.
            dcdc_enabled: true,
            dio2_rf_switch: true,
            // 3.3 V, not the 1.8 V a TCXO alone would want: DIO3 is the
            // switch's supply and its specified minimum is 2.5 V. The
            // Wio-E5 default this replaces would have left the switch
            // undefined on every transmission.
            tcxo_volts: TcxoVolts::V3_3,
            tcxo_startup_ms: 10,
            gps: GpsConfig::default(),
            // All-absent: a file that says nothing about the duty cycle
            // leaves whatever the board is running untouched.
            power: PowerConfig::default(),
        }
    }
}

impl RadioConfig {
    /// Low-data-rate optimization is required when the LoRa symbol time
    /// exceeds 16.38 ms (SF11/SF12 at BW125, SF12 at BW62.5).
    pub fn ldro(&self) -> bool {
        // symbol time ms = 2^sf / bw_khz; SF11/BW125 is exactly 16.384 ms,
        // so the 0.01 ms-resolution comparison must be inclusive.
        let num = 1u32 << self.spreading_factor;
        let bw = if self.bandwidth_khz == 62 { 62 } else { self.bandwidth_khz as u32 };
        num * 100 / bw >= 1638
    }

    /// Approximate scale of the on-air time relative to SF7/BW125,
    /// used to derive TX timeouts and safe listen periods.
    pub fn airtime_scale(&self) -> u32 {
        let sf = self.spreading_factor.clamp(5, 12);
        let shift = sf.saturating_sub(7) as u32;
        let base = 1u32 << shift; // 2^(sf-7), 1/4 floor for sf < 7
        let bw = if self.bandwidth_khz == 62 { 62 } else { self.bandwidth_khz as u32 };
        (base * 125 / bw).max(1)
    }

    /// Software poll deadline for TxDone (ms).
    pub fn tx_poll_timeout_ms(&self) -> u32 {
        150 * self.airtime_scale() + 100
    }

    /// Chip-level TX timeout (ms); longer than the poll deadline so the
    /// polling loop always exits first.
    pub fn tx_chip_timeout_ms(&self) -> u32 {
        self.tx_poll_timeout_ms() + 500
    }

    /// Upper bound of the random delay a repeater waits before forwarding
    /// a frame (ms).
    ///
    /// Two repeaters that hear the same transmission would otherwise answer it
    /// at the same instant and collide every time, so the wait has to span
    /// enough air time for one of them to win outright - hence scaling with
    /// the modulation rather than a fixed number of milliseconds.
    pub fn repeat_jitter_ms(&self) -> u32 {
        100 * self.airtime_scale()
    }

    /// Exact LoRa time-on-air for a PHY payload of `payload_len` bytes, in
    /// microseconds.
    ///
    /// This is the Semtech time-on-air formula, evaluated with the fixed PHY
    /// parameters the firmware always transmits under (see the WIO radio
    /// setup): an 8-symbol preamble, an explicit header, and the hardware CRC
    /// on. Low-data-rate optimization tracks [`ldro`](Self::ldro).
    ///
    /// Unlike [`airtime_scale`](Self::airtime_scale) - a coarse ratio used for
    /// timeouts - this is the real airtime, for reporting to a user. The four
    /// legal bandwidths all divide 1 MHz evenly, so the symbol time is a whole
    /// number of microseconds and the whole calculation stays in integer math:
    /// no float, so it is usable on the no_std targets too.
    pub fn time_on_air_us(&self, payload_len: usize) -> u32 {
        let sf = self.spreading_factor.clamp(5, 12) as u32;
        // Bandwidth in Hz; 62 is the config's shorthand for 62.5 kHz.
        let bw_hz = if self.bandwidth_khz == 62 {
            62_500
        } else {
            self.bandwidth_khz as u32 * 1_000
        };
        // 1_000_000 / bw_hz is exact for every legal bandwidth (16, 8, 4 or 2),
        // so the symbol time comes out an exact microsecond count.
        let t_sym_us = (1u32 << sf) * (1_000_000 / bw_hz);

        // Preamble is (n + 4.25) symbols with n = 8. 4.25 = 17/4, so scale the
        // symbol count by 4 and divide once to keep the quarter-symbol exact.
        let t_preamble_us = (4 * 8 + 17) * t_sym_us / 4;

        // Payload symbol count. `cr` is coding_rate's 1..4 offset over 4, `de`
        // the low-data-rate flag, the header is explicit (IH = 0) and the CRC
        // is on (the +16 term). The bracket can only go non-positive for a
        // payload far shorter than any real frame, and then no symbols are
        // added beyond the fixed 8.
        let cr = self.coding_rate.clamp(5, 8) as u32 - 4;
        let de = self.ldro() as i32;
        let num = 8 * payload_len as i32 - 4 * sf as i32 + 28 + 16;
        let den = 4 * (sf as i32 - 2 * de); // sf >= 5 and de <= 1, so den >= 12
        let payload_syms = if num <= 0 {
            8
        } else {
            8 + ((num + den - 1) / den) as u32 * (cr + 4)
        };
        let t_payload_us = payload_syms * t_sym_us;

        t_preamble_us + t_payload_us
    }

    /// Time-on-air of one beacon transmission at the current settings, in
    /// microseconds: the frame header plus whichever position fields
    /// [`beacon_fields`](Self::beacon_fields) selects.
    pub fn beacon_airtime_us(&self) -> u32 {
        let payload = crate::lora::HEADER_LEN + crate::lora::position_msg_len(self.beacon_fields);
        self.time_on_air_us(payload)
    }
}

/// Ceiling on [`RadioConfig::max_hops`]. Each hop costs another full
/// transmission of the same frame on a shared channel, so the useful range
/// is small and the limit exists to keep a typo from flooding the band.
pub const MAX_HOPS_LIMIT: u8 = 8;

// -- Read-back blob ---------------------------------------------------------
//
// A fixed, versioned binary snapshot of a [`RadioConfig`], so a board can
// report the settings it is actually running - which it otherwise never
// does: the config only ever travels *to* the board. The WIO encodes its
// live config, the ESP relays the bytes over BLE
// ([`crate::ble::RADIO_CONFIG_UUID`]) without parsing them, and the app
// decodes them here (the same crate both firmwares and the app build
// against, so there is one schema, not a copy per consumer).
//
// This is not the config file: it is a snapshot of the parsed result, so it
// reflects defaults and clamping and is available even on a board running
// with no stored file at all.

/// Wire length of the [`RadioConfig`] read-back blob.
pub const RADIO_CONFIG_LEN: usize = 28;

/// Layout version in byte 0, so an app meeting a newer firmware can reject
/// the blob rather than misread it.
pub const RADIO_CONFIG_VERSION: u8 = 1;

// byte 1 (misc bools)
const RCFG_RX_BOOST: u8 = 1 << 0;
const RCFG_SD_ENABLED: u8 = 1 << 1;
const RCFG_VERBOSE: u8 = 1 << 2;
const RCFG_DCDC: u8 = 1 << 3;
const RCFG_DIO2_RF_SWITCH: u8 = 1 << 4;
// byte 2 (GPS constellations)
const RCFG_GPS: u8 = 1 << 0;
const RCFG_GLONASS: u8 = 1 << 1;
const RCFG_GALILEO: u8 = 1 << 2;
const RCFG_BEIDOU: u8 = 1 << 3;
const RCFG_QZSS: u8 = 1 << 4;
const RCFG_SBAS: u8 = 1 << 5;

impl RadioConfig {
    /// Encode the config as the fixed [`RADIO_CONFIG_LEN`]-byte read-back
    /// blob (little-endian).
    pub fn encode(&self) -> [u8; RADIO_CONFIG_LEN] {
        let mut b = [0u8; RADIO_CONFIG_LEN];
        b[0] = RADIO_CONFIG_VERSION;
        let mut flags = 0u8;
        if self.rx_boost {
            flags |= RCFG_RX_BOOST;
        }
        if self.sd_enabled {
            flags |= RCFG_SD_ENABLED;
        }
        if self.verbose {
            flags |= RCFG_VERBOSE;
        }
        if self.dcdc_enabled {
            flags |= RCFG_DCDC;
        }
        if self.dio2_rf_switch {
            flags |= RCFG_DIO2_RF_SWITCH;
        }
        b[1] = flags;
        let mut g = 0u8;
        if self.gps.gps_enabled {
            g |= RCFG_GPS;
        }
        if self.gps.glonass_enabled {
            g |= RCFG_GLONASS;
        }
        if self.gps.galileo_enabled {
            g |= RCFG_GALILEO;
        }
        if self.gps.beidou_enabled {
            g |= RCFG_BEIDOU;
        }
        if self.gps.qzss_enabled {
            g |= RCFG_QZSS;
        }
        if self.gps.sbas_enabled {
            g |= RCFG_SBAS;
        }
        b[2] = g;
        b[3] = self.role.to_wire();
        b[4..8].copy_from_slice(&self.frequency_hz.to_le_bytes());
        b[8] = self.spreading_factor;
        b[9..11].copy_from_slice(&self.bandwidth_khz.to_le_bytes());
        b[11] = self.coding_rate;
        b[12] = self.power_dbm as u8;
        b[13] = self.address;
        b[14] = self.max_hops;
        b[15..17].copy_from_slice(&self.dedup_ttl_s.to_le_bytes());
        b[17..19].copy_from_slice(&self.beacon_interval_s.to_le_bytes());
        b[19] = self.beacon_fields;
        b[20] = self.tcxo_volts.trim();
        b[21..23].copy_from_slice(&self.tcxo_startup_ms.to_le_bytes());
        b[23] = self.gps.power_mode.operate_mode();
        b[24..26].copy_from_slice(&self.gps.meas_rate_ms.to_le_bytes());
        b[26] = self.gps.dyn_model.dynmodel();
        // b[27] reserved, kept zero.
        b
    }

    /// Decode a read-back blob. `None` for a short buffer, an unknown layout
    /// version, or an enum byte this build does not recognize - the caller
    /// then keeps its own values rather than acting on a half-read config.
    /// Trailing bytes are tolerated so a future layout can only grow.
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() < RADIO_CONFIG_LEN || b[0] != RADIO_CONFIG_VERSION {
            return None;
        }
        let flags = b[1];
        let g = b[2];
        let u16at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        Some(Self {
            frequency_hz: u32::from_le_bytes([b[4], b[5], b[6], b[7]]),
            spreading_factor: b[8],
            bandwidth_khz: u16at(9),
            coding_rate: b[11],
            power_dbm: b[12] as i8,
            rx_boost: flags & RCFG_RX_BOOST != 0,
            address: b[13],
            role: Role::from_wire(b[3])?,
            max_hops: b[14],
            dedup_ttl_s: u16at(15),
            beacon_interval_s: u16at(17),
            beacon_fields: b[19],
            sd_enabled: flags & RCFG_SD_ENABLED != 0,
            verbose: flags & RCFG_VERBOSE != 0,
            dcdc_enabled: flags & RCFG_DCDC != 0,
            dio2_rf_switch: flags & RCFG_DIO2_RF_SWITCH != 0,
            tcxo_volts: TcxoVolts::from_trim(b[20])?,
            tcxo_startup_ms: u16at(21),
            gps: GpsConfig {
                gps_enabled: g & RCFG_GPS != 0,
                glonass_enabled: g & RCFG_GLONASS != 0,
                galileo_enabled: g & RCFG_GALILEO != 0,
                beidou_enabled: g & RCFG_BEIDOU != 0,
                qzss_enabled: g & RCFG_QZSS != 0,
                sbas_enabled: g & RCFG_SBAS != 0,
                power_mode: PowerMode::from_operate_mode(b[23])?,
                meas_rate_ms: u16at(24),
                dyn_model: DynModel::from_dynmodel(b[26])?,
            },
            // Not carried in the blob, and it would be the wrong place for
            // it: this reports the radio config the board parsed, while the
            // duty cycle the board is actually running is reported live on
            // the settings characteristic ([`crate::ble::Settings`]). Two
            // read-backs of the same three numbers could disagree.
            power: PowerConfig::default(),
        })
    }
}

/// Config parse/validation errors. The u32 is the offending line number
/// (1-based) where one applies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// Line is not `key = value`, a comment, a blank or a `[section]`.
    Syntax(u32),
    /// Value does not parse as the expected type.
    BadValue(u32),
    /// A recognized key holds an out-of-range value.
    OutOfRange(u32),
    /// The file is not valid UTF-8.
    Utf8,
}

/// Parse TOML text into a [`RadioConfig`], starting from the defaults so a
/// partial file is fine. Unknown keys are ignored (forward compatibility).
pub fn parse(text: &str) -> Result<RadioConfig, ConfigError> {
    let mut cfg = RadioConfig::default();
    for (idx, raw_line) in text.lines().enumerate() {
        let lineno = idx as u32 + 1;
        let line = match raw_line.split_once('#') {
            Some((before, _)) => before.trim(),
            None => raw_line.trim(),
        };
        if line.is_empty() {
            continue;
        }
        if line.starts_with('[') {
            if line.ends_with(']') {
                continue; // section headers are accepted and ignored
            }
            return Err(ConfigError::Syntax(lineno));
        }
        let (key, value) = line.split_once('=').ok_or(ConfigError::Syntax(lineno))?;
        let (key, value) = (key.trim(), value.trim());
        if key.is_empty() || value.is_empty() {
            return Err(ConfigError::Syntax(lineno));
        }

        match key {
            "frequency_hz" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                // Sub-GHz ISM range the SX126x covers.
                if !(150_000_000..=960_000_000).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.frequency_hz = v as u32;
            }
            "spreading_factor" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(5..=12).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.spreading_factor = v as u8;
            }
            "bandwidth_khz" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !matches!(v, 62 | 125 | 250 | 500) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.bandwidth_khz = v as u16;
            }
            "coding_rate" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(5..=8).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.coding_rate = v as u8;
            }
            "power_dbm" => {
                let v = parse_i64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(-9..=22).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.power_dbm = v as i8;
            }
            "rx_boost" => cfg.rx_boost = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "address" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(1..=255).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.address = v as u8;
            }
            "role" => {
                cfg.role = match unquote(value) {
                    "leaf" => Role::Leaf,
                    "repeater" => Role::Repeater,
                    "tx_only" => Role::TxOnly,
                    "rx_only" => Role::RxOnly,
                    _ => return Err(ConfigError::BadValue(lineno)),
                };
            }
            "max_hops" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if v > MAX_HOPS_LIMIT as u64 {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.max_hops = v as u8;
            }
            "dedup_ttl_s" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                // Below 1 s dedup is effectively off; above the beacon-id
                // wrap it starts suppressing a node's own later frames. The
                // ceiling matches beacon_interval_s so neither can be set to
                // a value the other makes nonsensical on its own.
                if !(1..=3600).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.dedup_ttl_s = v as u16;
            }
            // Accepted so cards written for the old mesh keep the hop count
            // their author intended: it counted transmissions, where
            // max_hops counts only the retransmissions after the first.
            "lifetime" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(1..=16).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.max_hops = ((v - 1) as u8).min(MAX_HOPS_LIMIT);
            }
            "interval_s" | "beacon_interval_s" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if v > 3600 {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.beacon_interval_s = v as u16;
            }
            "fields" | "beacon_fields" => {
                cfg.beacon_fields = parse_fields(value).ok_or(ConfigError::BadValue(lineno))?;
                // Position is the whole point of the broadcast, and a
                // receiver has nothing to plot without it.
                if cfg.beacon_fields & lora::FIELDS_REQUIRED != lora::FIELDS_REQUIRED {
                    return Err(ConfigError::OutOfRange(lineno));
                }
            }
            "dcdc_enabled" => {
                cfg.dcdc_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?
            }
            "dio2_rf_switch" => {
                cfg.dio2_rf_switch = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?
            }
            "tcxo_volts" => {
                cfg.tcxo_volts = match unquote(value) {
                    "1.6" => TcxoVolts::V1_6,
                    "1.7" => TcxoVolts::V1_7,
                    "1.8" => TcxoVolts::V1_8,
                    "2.2" => TcxoVolts::V2_2,
                    "2.4" => TcxoVolts::V2_4,
                    "2.7" => TcxoVolts::V2_7,
                    "3.0" => TcxoVolts::V3_0,
                    "3.3" => TcxoVolts::V3_3,
                    _ => return Err(ConfigError::BadValue(lineno)),
                };
            }
            "tcxo_startup_ms" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                // The ceiling is a stuck-oscillator guard: past this the
                // radio is not slow to start, it is not starting.
                if !(1..=1_000).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.tcxo_startup_ms = v as u16;
            }
            // -- [sd] -------------------------------------------------------
            "sd_enabled" => cfg.sd_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            // -- [debug] ----------------------------------------------------
            "verbose" => cfg.verbose = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            // -- [gps] ------------------------------------------------------
            "gps_enabled" => cfg.gps.gps_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "glonass_enabled" => cfg.gps.glonass_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "galileo_enabled" => cfg.gps.galileo_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "beidou_enabled" => cfg.gps.beidou_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "qzss_enabled" => cfg.gps.qzss_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "sbas_enabled" => cfg.gps.sbas_enabled = parse_bool(value).ok_or(ConfigError::BadValue(lineno))?,
            "power_mode" => {
                cfg.gps.power_mode = match unquote(value) {
                    "full" => PowerMode::Full,
                    "psmoo" | "psm_onoff" => PowerMode::PsmOnOff,
                    "psmct" | "psm_cyclic" => PowerMode::PsmCyclic,
                    _ => return Err(ConfigError::BadValue(lineno)),
                };
            }
            "meas_rate_ms" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if !(25..=10_000).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.gps.meas_rate_ms = v as u16;
            }
            "dynamic_model" | "dyn_model" => {
                cfg.gps.dyn_model = match unquote(value) {
                    "portable" => DynModel::Portable,
                    "stationary" => DynModel::Stationary,
                    "pedestrian" => DynModel::Pedestrian,
                    "automotive" => DynModel::Automotive,
                    "sea" => DynModel::Sea,
                    "airborne1g" => DynModel::Airborne1g,
                    "airborne2g" => DynModel::Airborne2g,
                    "airborne4g" => DynModel::Airborne4g,
                    _ => return Err(ConfigError::BadValue(lineno)),
                };
            }
            // -- [power] ----------------------------------------------------
            // Each of these is stored as `Some` even when the value equals
            // the firmware default, because what the board does with them
            // turns on whether the key was written at all, not on what it
            // says. See [`PowerConfig`].
            "ble_off_s" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if v != 0 && !(ble::BLE_OFF_MIN_S as u64..=ble::BLE_OFF_MAX_S as u64).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.power.ble_off_s = Some(v as u32);
            }
            "adv_window_s" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                // 0 is legal and means "the firmware default", which is why
                // the floor is not `ESP_ADV_MIN_S`.
                if v != 0 && !(ble::ESP_ADV_MIN_S as u64..=ble::ESP_ADV_MAX_S as u64).contains(&v) {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.power.adv_window_s = Some(v as u32);
            }
            "sleep_interval_s" => {
                let v = parse_u64(value).ok_or(ConfigError::BadValue(lineno))?;
                if v != 0
                    && !(ble::ESP_SLEEP_MIN_S as u64..=ble::ESP_SLEEP_MAX_S as u64).contains(&v)
                {
                    return Err(ConfigError::OutOfRange(lineno));
                }
                cfg.power.sleep_interval_s = Some(v as u32);
            }
            _ => {} // unknown key: ignore
        }
    }
    Ok(cfg)
}

/// Parse raw file bytes (validates UTF-8 first).
pub fn parse_bytes(bytes: &[u8]) -> Result<RadioConfig, ConfigError> {
    parse(core::str::from_utf8(bytes).map_err(|_| ConfigError::Utf8)?)
}

fn parse_u64(s: &str) -> Option<u64> {
    // Allow underscores as digit separators, as TOML does.
    let mut n: u64 = 0;
    let mut any = false;
    for b in s.bytes() {
        match b {
            b'0'..=b'9' => {
                n = n.checked_mul(10)?.checked_add((b - b'0') as u64)?;
                any = true;
            }
            b'_' if any => {}
            _ => return None,
        }
    }
    any.then_some(n)
}

fn parse_i64(s: &str) -> Option<i64> {
    let (neg, digits) = match s.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, s),
    };
    let v = parse_u64(digits)? as i64;
    Some(if neg { -v } else { v })
}

/// Parse a comma-separated beacon field list ("lat,lon,altitude") into a
/// [`crate::lora`] field mask. An empty list is rejected: writing `""` reads
/// like "send nothing", which is not a thing a beacon can do, so it is a
/// mistake worth reporting rather than silently accepting.
fn parse_fields(value: &str) -> Option<u8> {
    let mut mask = 0u8;
    for name in unquote(value).split(',') {
        mask |= match name.trim() {
            "lat" | "latitude" => lora::FIELD_LAT,
            "lon" | "longitude" => lora::FIELD_LON,
            "alt" | "altitude" => lora::FIELD_ALT,
            "speed" => lora::FIELD_SPEED,
            "course" => lora::FIELD_COURSE,
            "sats" => lora::FIELD_SATS,
            "time" => lora::FIELD_TIME,
            _ => return None,
        };
    }
    Some(mask)
}

fn parse_bool(s: &str) -> Option<bool> {
    match s {
        "true" => Some(true),
        "false" => Some(false),
        _ => None,
    }
}

/// Strip matching single or double quotes from a TOML string value.
fn unquote(s: &str) -> &str {
    let b = s.as_bytes();
    if b.len() >= 2 && (b[0] == b'"' || b[0] == b'\'') && b[b.len() - 1] == b[0] {
        &s[1..s.len() - 1]
    } else {
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_when_empty() {
        assert_eq!(parse("").unwrap(), RadioConfig::default());
        assert_eq!(parse("# just a comment\n\n").unwrap(), RadioConfig::default());
    }

    /// RADIO.example.toml states in its header that its values are the
    /// firmware defaults, and it is written by hand, so nothing but this
    /// makes that true. It is also what gps-gui-rs lays down for a board with
    /// no config yet: since the app writes every key explicitly, a stale value
    /// here is not corrected by the default it drifted from - it silently
    /// becomes the board's setting.
    #[test]
    fn example_file_documents_the_real_defaults() {
        let example = include_str!("../../RADIO.example.toml");
        assert_eq!(parse(example).unwrap(), RadioConfig::default());
    }

    /// Why a reader must never hand this parser a truncated file.
    ///
    /// Comments and blank lines are skipped, so any prefix that happens to end
    /// on a line boundary parses clean - it is simply a file with fewer keys,
    /// and every key it lost comes back as its default. There is nothing in the
    /// result to say it was cut. The example file is the worst case of that:
    /// its first 1024 bytes, the ceiling both transfer paths enforce, are all
    /// header comment, so a truncated read of it succeeds and yields *every*
    /// setting at its default - address included.
    ///
    /// So the size check belongs at the reader (`SdLog::read_config` refuses a
    /// file it cannot hold whole) and cannot be delegated to a parse failure.
    #[test]
    fn a_truncated_file_parses_as_a_shorter_one() {
        let example = include_str!("../../RADIO.example.toml");
        let head = &example.as_bytes()[..1024];
        assert_eq!(parse_bytes(head), Ok(RadioConfig::default()));
        // Not a quirk of that one offset: a config cut after its first key
        // keeps that key and defaults the rest, silently.
        let cut = parse("address = 7\nspreading_factor = 9").unwrap();
        assert_eq!((cut.address, cut.spreading_factor), (7, 9));
        let truncated = parse("address = 7\n").unwrap();
        assert_eq!(truncated.address, 7);
        assert_eq!(truncated.spreading_factor, RadioConfig::default().spreading_factor);
    }

    #[test]
    fn full_file() {
        let toml = r#"
            # telemetry-in-midair radio config
            [radio]
            frequency_hz = 915_000_000
            spreading_factor = 10
            bandwidth_khz = 125
            coding_rate = 8
            power_dbm = 14
            rx_boost = true

            [mesh]
            address = 3
            role = "repeater"
            max_hops = 2

            [beacon]
            interval_s = 30
        "#;
        let cfg = parse(toml).unwrap();
        assert_eq!(cfg.frequency_hz, 915_000_000);
        assert_eq!(cfg.spreading_factor, 10);
        assert_eq!(cfg.coding_rate, 8);
        assert_eq!(cfg.power_dbm, 14);
        assert!(cfg.rx_boost);
        assert_eq!(cfg.address, 3);
        assert_eq!(cfg.role, Role::Repeater);
        assert_eq!(cfg.max_hops, 2);
        assert_eq!(cfg.beacon_interval_s, 30);
        assert!(!cfg.ldro());
    }

    #[test]
    fn role_defaults_to_leaf_with_one_repeat_allowed() {
        let cfg = RadioConfig::default();
        assert_eq!(cfg.role, Role::Leaf);
        assert_eq!(cfg.max_hops, 1);
        assert_eq!(parse("role = \"leaf\"").unwrap().role, Role::Leaf);
        assert_eq!(parse("max_hops = 0").unwrap().max_hops, 0);
        assert_eq!(parse("role = repeater").unwrap().role, Role::Repeater);
        assert_eq!(parse("role = \"gateway\""), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("max_hops = 9"), Err(ConfigError::OutOfRange(1)));
    }

    #[test]
    fn one_way_roles_parse() {
        assert_eq!(parse("role = \"tx_only\"").unwrap().role, Role::TxOnly);
        assert_eq!(parse("role = \"rx_only\"").unwrap().role, Role::RxOnly);
        // Near misses are typos, not a mode to guess at.
        assert_eq!(parse("role = \"tx\""), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("role = \"txonly\""), Err(ConfigError::BadValue(1)));
    }

    /// Each role uses exactly the halves of the air interface it names, and
    /// only a repeater forwards. These drive the radio's idle state and the
    /// beacon gate, so getting one wrong is a node that is silently deaf or
    /// silently mute.
    #[test]
    fn roles_use_the_halves_they_name() {
        for (role, tx, rx, repeats) in [
            (Role::Leaf, true, true, false),
            (Role::Repeater, true, true, true),
            (Role::TxOnly, true, false, false),
            (Role::RxOnly, false, true, false),
        ] {
            assert_eq!(role.transmits(), tx, "{role:?} transmits");
            assert_eq!(role.receives(), rx, "{role:?} receives");
            assert_eq!(role.repeats(), repeats, "{role:?} repeats");
        }
    }

    /// A repeater has to hear a frame before it can forward one, so no role
    /// may repeat without receiving.
    #[test]
    fn repeating_implies_receiving() {
        for role in [Role::Leaf, Role::Repeater, Role::TxOnly, Role::RxOnly] {
            assert!(!role.repeats() || role.receives(), "{role:?}");
        }
    }

    #[test]
    fn legacy_lifetime_maps_to_hop_count() {
        // The old key counted transmissions; 2 meant "one repeat".
        assert_eq!(parse("lifetime = 2").unwrap().max_hops, 1);
        assert_eq!(parse("lifetime = 1").unwrap().max_hops, 0);
        assert_eq!(parse("lifetime = 16").unwrap().max_hops, MAX_HOPS_LIMIT);
        assert_eq!(parse("lifetime = 0"), Err(ConfigError::OutOfRange(1)));
        // listen_ms is gone; an old card carrying it still parses.
        assert_eq!(parse("listen_ms = 900").unwrap(), RadioConfig::default());
    }

    #[test]
    fn dedup_ttl_parses_and_is_bounded() {
        assert_eq!(RadioConfig::default().dedup_ttl_s, 3);
        assert_eq!(parse("dedup_ttl_s = 120").unwrap().dedup_ttl_s, 120);
        assert_eq!(parse("dedup_ttl_s = 0"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("dedup_ttl_s = 3601"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("dedup_ttl_s = often"), Err(ConfigError::BadValue(1)));
    }

    /// Built from explicit modulation rather than from the default, which is
    /// itself the slowest spreading factor and so has nothing to grow into.
    #[test]
    fn repeat_jitter_tracks_air_time() {
        let mut cfg = RadioConfig {
            spreading_factor: 7,
            bandwidth_khz: 125,
            ..RadioConfig::default()
        };
        let fast = cfg.repeat_jitter_ms();
        cfg.spreading_factor = 12;
        assert!(cfg.repeat_jitter_ms() > fast);
    }

    #[test]
    fn ldro_and_timeouts_scale() {
        // SF7/BW125 is the scale's own reference point, so pin it here rather
        // than lean on the default (which is a slower modulation).
        let mut cfg = RadioConfig {
            spreading_factor: 7,
            bandwidth_khz: 125,
            ..RadioConfig::default()
        };
        assert!(!cfg.ldro());
        assert_eq!(cfg.airtime_scale(), 1);

        cfg.spreading_factor = 12;
        assert!(cfg.ldro());
        assert_eq!(cfg.airtime_scale(), 32);
        assert!(cfg.tx_poll_timeout_ms() > 4_000);
        assert!(cfg.tx_chip_timeout_ms() > cfg.tx_poll_timeout_ms());

        cfg.bandwidth_khz = 62;
        assert!(cfg.ldro());
        assert_eq!(cfg.airtime_scale(), 64);

        cfg.spreading_factor = 11;
        cfg.bandwidth_khz = 125;
        assert!(cfg.ldro());
        cfg.spreading_factor = 10;
        assert!(!cfg.ldro());
    }

    /// Time-on-air against values worked out by hand from the Semtech formula,
    /// so a change to the arithmetic that shifts the result is caught. The
    /// default beacon is the 46.3 ms figure the docs quote.
    #[test]
    fn time_on_air_matches_hand_calc() {
        // SF7, BW125, CR 4/5, with a header (3) + position lat/lon (10) =
        // 13-byte PHY payload: the 46.3 ms figure the docs quote.
        let mut cfg = RadioConfig {
            spreading_factor: 7,
            bandwidth_khz: 125,
            ..RadioConfig::default()
        };
        assert_eq!(cfg.time_on_air_us(13), 46_336);
        assert_eq!(cfg.beacon_airtime_us(), 46_336);

        // The shipped default (SF12/BW500) beacon: ~289 ms. Shorter than the
        // ~330 ms of the SF9/BW62.5 it replaced, despite the far higher
        // spreading factor, because 2^12/500 kHz and 2^9/62.5 kHz are the
        // same 8.192 ms symbol and SF12 needs fewer of them per byte.
        assert_eq!(RadioConfig::default().beacon_airtime_us(), 288_768);

        // The no-fix ping that goes out in the same slot: header + 4 bytes,
        // ~248 ms, so a node reporting a missing fix spends less air time
        // than one reporting a position.
        let ping = crate::lora::HEADER_LEN + crate::lora::PING_MSG_LEN;
        assert_eq!(RadioConfig::default().time_on_air_us(ping), 247_808);

        // Slowest modulation the parser accepts, largest frame the firmware
        // sends: SF12 / BW62.5 / CR 4/8 with LDRO on, 35 bytes -> ~5 s.
        cfg.spreading_factor = 12;
        cfg.bandwidth_khz = 62;
        cfg.coding_rate = 8;
        assert!(cfg.ldro());
        assert_eq!(cfg.time_on_air_us(crate::lora::FRAME_MAX), 4_997_120);

        // Same frame at BW125 is exactly half the symbol time, so ~2.5 s.
        cfg.bandwidth_khz = 125;
        assert_eq!(cfg.time_on_air_us(crate::lora::FRAME_MAX), 2_498_560);
    }

    /// A longer beacon payload costs more airtime, and airtime climbs steeply
    /// with the spreading factor - the two levers a range-vs-limit trade pulls.
    #[test]
    fn beacon_airtime_grows_with_fields_and_sf() {
        // Fields are measured against the shipped default, since that is the
        // payload choice the default actually makes.
        let base = RadioConfig::default();
        let mut richer = base;
        richer.beacon_fields = lora::FIELDS_ALL;
        assert!(richer.beacon_airtime_us() > base.beacon_airtime_us());

        // The spreading factor needs a base with room above it: the default
        // is SF12, the slowest the parser accepts.
        let fast = RadioConfig {
            spreading_factor: 9,
            ..base
        };
        let mut slower = fast;
        slower.spreading_factor = 10;
        assert!(slower.beacon_airtime_us() > fast.beacon_airtime_us());
    }

    #[test]
    fn beacon_fields_default_to_position_only() {
        let cfg = RadioConfig::default();
        assert_eq!(cfg.beacon_fields, lora::FIELD_LAT | lora::FIELD_LON);
        assert_eq!(lora::position_msg_len(cfg.beacon_fields), 10);
    }

    #[test]
    fn beacon_fields_parse() {
        let mask = parse("fields = \"lat,lon,altitude,time\"").unwrap().beacon_fields;
        assert_eq!(
            mask,
            lora::FIELD_LAT | lora::FIELD_LON | lora::FIELD_ALT | lora::FIELD_TIME
        );
        // Whitespace, the short spellings and the aliased key all work.
        assert_eq!(
            parse("beacon_fields = \"lat, lon, alt\"").unwrap().beacon_fields,
            lora::FIELD_LAT | lora::FIELD_LON | lora::FIELD_ALT
        );
        assert_eq!(parse("fields = \"lat,lon\"").unwrap(), RadioConfig::default());
    }

    #[test]
    fn beacon_fields_reject_nonsense() {
        // Unknown field name.
        assert_eq!(parse("fields = \"lat,lon,heading\""), Err(ConfigError::BadValue(1)));
        // A transmission without a position is not a position transmission.
        assert_eq!(parse("fields = \"altitude\""), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("fields = \"lat\""), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("fields = \"\""), Err(ConfigError::BadValue(1)));
    }

    #[test]
    /// The defaults are the Wio-S3's hardware. `tcxo_volts` in particular
    /// is not free: DIO3 supplies the SKY13453-385LF antenna switch as well
    /// as the TCXO, and that part is specified 2.5 - 3.5 V, so the 1.8 V
    /// this used to carry from the Wio-E5 left the switch undefined with
    /// the PA transmitting into it.
    fn tcxo_and_regulator_defaults_match_the_wio_s3() {
        let cfg = RadioConfig::default();
        assert!(cfg.dcdc_enabled);
        assert_eq!(cfg.tcxo_volts, TcxoVolts::V3_3);
        assert_eq!(cfg.tcxo_volts.trim(), 0x7);
        assert_eq!(cfg.tcxo_startup_ms, 10);
        // 0x5 is 2.7 V, the lowest trim at or above the switch's minimum.
        assert!(cfg.tcxo_volts.trim() >= 0x5);
    }

    #[test]
    fn tcxo_and_regulator_parse() {
        assert!(!parse("dcdc_enabled = false").unwrap().dcdc_enabled);
        assert_eq!(parse("tcxo_volts = \"3.3\"").unwrap().tcxo_volts, TcxoVolts::V3_3);
        assert_eq!(parse("tcxo_volts = \"1.6\"").unwrap().tcxo_volts.trim(), 0x0);
        assert_eq!(parse("tcxo_startup_ms = 50").unwrap().tcxo_startup_ms, 50);
        // A voltage the chip has no trim setting for is a typo, not a
        // value to round to the nearest one.
        assert_eq!(parse("tcxo_volts = \"3.0V\""), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("tcxo_volts = \"5.0\""), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("tcxo_startup_ms = 0"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("tcxo_startup_ms = 2000"), Err(ConfigError::OutOfRange(1)));
    }

    /// The ESP has no copy of the config, so this key only reaches it as a
    /// telemetry flag bit - see [`crate::link::TELEM_FLAG_VERBOSE`].
    #[test]
    fn verbose_defaults_on_and_can_be_turned_off() {
        assert!(RadioConfig::default().verbose);
        assert!(!parse("verbose = false").unwrap().verbose);
        assert!(parse("verbose = true").unwrap().verbose);
        assert_eq!(parse("verbose = quiet"), Err(ConfigError::BadValue(1)));
    }

    #[test]
    fn sd_can_be_disabled() {
        assert!(RadioConfig::default().sd_enabled);
        assert!(!parse("sd_enabled = false").unwrap().sd_enabled);
        assert_eq!(parse("sd_enabled = yes"), Err(ConfigError::BadValue(1)));
    }

    #[test]
    fn gps_defaults_when_section_absent() {
        assert_eq!(parse("frequency_hz = 915000000").unwrap().gps, GpsConfig::default());
    }

    #[test]
    fn gps_section() {
        let toml = r#"
            [gps]
            gps_enabled = true
            glonass_enabled = true
            galileo_enabled = false
            beidou_enabled = false
            qzss_enabled = false
            sbas_enabled = false
            power_mode = "psmoo"
            meas_rate_ms = 500
            dynamic_model = "airborne2g"
        "#;
        let g = parse(toml).unwrap().gps;
        assert!(g.gps_enabled);
        assert!(g.glonass_enabled);
        assert!(!g.galileo_enabled);
        assert!(!g.sbas_enabled);
        assert_eq!(g.power_mode, PowerMode::PsmOnOff);
        assert_eq!(g.power_mode.operate_mode(), 1);
        assert_eq!(g.meas_rate_ms, 500);
        assert_eq!(g.dyn_model, DynModel::Airborne2g);
        assert_eq!(g.dyn_model.dynmodel(), 7);
    }

    #[test]
    fn rejects_bad_gps_input() {
        assert_eq!(parse("gps_enabled = maybe"), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("power_mode = \"turbo\""), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("dynamic_model = spaceship"), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("meas_rate_ms = 10"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("meas_rate_ms = 20000"), Err(ConfigError::OutOfRange(1)));
    }

    #[test]
    fn rejects_bad_input() {
        assert_eq!(parse("frequency_hz = maybe"), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("spreading_factor = 13"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("bandwidth_khz = 100"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("power_dbm = 23"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("address = 0"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("rx_boost = 1"), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("\nnot a kv line"), Err(ConfigError::Syntax(2)));
        assert_eq!(parse_bytes(&[0xFF, 0xFE]), Err(ConfigError::Utf8));
        // Unknown keys pass through untouched.
        assert!(parse("future_knob = 42").is_ok());
    }

    #[test]
    fn radio_config_blob_roundtrips() {
        // A config that differs from the defaults in every field type: an
        // integer, a signed value, each enum, the field mask and bools on
        // both flag bytes, so a swapped byte cannot pass unnoticed.
        let cfg = RadioConfig {
            frequency_hz: 868_100_000,
            spreading_factor: 12,
            bandwidth_khz: 250,
            coding_rate: 8,
            power_dbm: -9,
            rx_boost: false,
            address: 200,
            role: Role::Repeater,
            max_hops: 3,
            dedup_ttl_s: 120,
            beacon_interval_s: 30,
            beacon_fields: crate::lora::FIELD_LAT | crate::lora::FIELD_LON | crate::lora::FIELD_ALT,
            sd_enabled: false,
            verbose: false,
            dcdc_enabled: false,
            dio2_rf_switch: true,
            tcxo_volts: TcxoVolts::V3_3,
            tcxo_startup_ms: 250,
            gps: GpsConfig {
                gps_enabled: true,
                glonass_enabled: true,
                galileo_enabled: false,
                beidou_enabled: false,
                qzss_enabled: true,
                sbas_enabled: false,
                power_mode: PowerMode::PsmCyclic,
                meas_rate_ms: 500,
                dyn_model: DynModel::Airborne4g,
            },
            // Left at the default: the blob does not carry the duty cycle,
            // so a non-default here would fail the round trip by design.
            power: PowerConfig::default(),
        };
        let bytes = cfg.encode();
        assert_eq!(bytes.len(), RADIO_CONFIG_LEN);
        assert_eq!(bytes[0], RADIO_CONFIG_VERSION);
        assert_eq!(RadioConfig::decode(&bytes), Some(cfg));
    }

    /// The antenna switch is a board property, and on the Wio-S3 it is
    /// wired, so a card that says nothing about it must leave it on. The
    /// key must also not ride on any of the flag byte's other bits: a board
    /// that stops driving DIO2 ramps its PA into an isolated switch on
    /// every transmission.
    #[test]
    fn dio2_rf_switch_is_on_unless_refused() {
        assert!(RadioConfig::default().dio2_rf_switch);
        let quiet = parse("rx_boost = true\ndcdc_enabled = true\n").unwrap();
        assert!(quiet.dio2_rf_switch);

        let off = parse("dio2_rf_switch = false\n").unwrap();
        assert!(!off.dio2_rf_switch);
        // and it did not disturb the other bools sharing the flag byte
        let d = RadioConfig::default();
        assert_eq!(off.rx_boost, d.rx_boost);
        assert_eq!(off.sd_enabled, d.sd_enabled);
        assert_eq!(off.verbose, d.verbose);
        assert_eq!(off.dcdc_enabled, d.dcdc_enabled);

        assert!(!RadioConfig::decode(&off.encode()).unwrap().dio2_rf_switch);
        assert!(RadioConfig::decode(&d.encode()).unwrap().dio2_rf_switch);
    }

    #[test]
    fn radio_config_blob_defaults_roundtrip() {
        let cfg = RadioConfig::default();
        assert_eq!(RadioConfig::decode(&cfg.encode()), Some(cfg));
    }

    #[test]
    fn radio_config_blob_rejects_short_and_wrong_version() {
        let good = RadioConfig::default().encode();
        assert_eq!(RadioConfig::decode(&good[..RADIO_CONFIG_LEN - 1]), None);
        let mut bad = good;
        bad[0] = RADIO_CONFIG_VERSION + 1;
        assert_eq!(RadioConfig::decode(&bad), None);
    }

    /// A longer buffer must still decode: a future layout can only grow, and
    /// byte 0 is what gates compatibility.
    #[test]
    fn radio_config_blob_tolerates_trailing_bytes() {
        let good = RadioConfig::default().encode();
        let mut longer = [0u8; RADIO_CONFIG_LEN + 4];
        longer[..RADIO_CONFIG_LEN].copy_from_slice(&good);
        assert!(RadioConfig::decode(&longer).is_some());
    }

    /// The blob has to survive both transports it rides: one UART link frame
    /// (see [`crate::link::MAX_PAYLOAD`]) and a single BLE read.
    #[test]
    fn radio_config_blob_fits_its_transports() {
        assert!(RADIO_CONFIG_LEN <= crate::link::MAX_PAYLOAD);
        assert!(RADIO_CONFIG_LEN <= 244); // conservative ATT_MTU-3 floor
    }

    /// Every enum's string form has to parse back to the same variant, since
    /// that round-trip is what lets a reader render the blob into TOML the
    /// firmware then accepts.
    #[test]
    fn enum_strings_parse_back() {
        for role in [Role::Leaf, Role::Repeater, Role::TxOnly, Role::RxOnly] {
            assert_eq!(Role::from_wire(role.to_wire()), Some(role));
            let toml = format!("role = \"{}\"", role.as_str());
            assert_eq!(parse(&toml).unwrap().role, role);
        }
        for v in [TcxoVolts::V1_6, TcxoVolts::V1_8, TcxoVolts::V3_3] {
            assert_eq!(TcxoVolts::from_trim(v.trim()), Some(v));
            let toml = format!("tcxo_volts = \"{}\"", v.as_str());
            assert_eq!(parse(&toml).unwrap().tcxo_volts, v);
        }
        for pm in [PowerMode::Full, PowerMode::PsmOnOff, PowerMode::PsmCyclic] {
            assert_eq!(PowerMode::from_operate_mode(pm.operate_mode()), Some(pm));
            let toml = format!("power_mode = \"{}\"", pm.as_str());
            assert_eq!(parse(&toml).unwrap().gps.power_mode, pm);
        }
        for dm in [DynModel::Portable, DynModel::Sea, DynModel::Airborne4g] {
            assert_eq!(DynModel::from_dynmodel(dm.dynmodel()), Some(dm));
            let toml = format!("dynamic_model = \"{}\"", dm.as_str());
            assert_eq!(parse(&toml).unwrap().gps.dyn_model, dm);
        }
    }
    /// An absent `[power]` section must leave every field `None`, because
    /// `None` is what the firmware reads as "do not touch the board's live
    /// duty cycle". A default that was `Some(0)` would make every config
    /// push silently disable the BLE off period.
    #[test]
    fn a_file_with_no_power_section_asks_for_nothing() {
        let cfg = parse("frequency_hz = 915000000").unwrap();
        assert_eq!(cfg.power, PowerConfig::default());
        assert_eq!(cfg.power.ble_off_s, None);
    }

    /// A zero is a value, not an absence: writing `ble_off_s = 0` is how a
    /// file says "keep BLE up", and it has to override a board that was left
    /// duty-cycling.
    #[test]
    fn an_explicit_zero_is_still_a_request() {
        let cfg = parse("ble_off_s = 0\nsleep_interval_s = 0\nadv_window_s = 0").unwrap();
        assert_eq!(cfg.power.ble_off_s, Some(0));
        assert_eq!(cfg.power.sleep_interval_s, Some(0));
        assert_eq!(cfg.power.adv_window_s, Some(0));
    }

    #[test]
    fn power_values_parse_and_range_check() {
        let cfg = parse("[power]\nble_off_s = 30\nadv_window_s = 10\nsleep_interval_s = 120")
            .unwrap();
        assert_eq!(cfg.power.ble_off_s, Some(30));
        assert_eq!(cfg.power.adv_window_s, Some(10));
        assert_eq!(cfg.power.sleep_interval_s, Some(120));

        // Below the floor but not zero, and above the ceiling, on each key.
        for bad in [
            "ble_off_s = 1",
            "ble_off_s = 301",
            "adv_window_s = 61",
            "sleep_interval_s = 1",
            "sleep_interval_s = 301",
        ] {
            assert_eq!(
                parse(bad),
                Err(ConfigError::OutOfRange(1)),
                "{bad} should have been rejected"
            );
        }
    }
}

