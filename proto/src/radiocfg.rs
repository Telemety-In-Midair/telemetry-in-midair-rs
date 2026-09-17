//! Radio configuration: the `RADIO.CFG` file format and its parser.
//!
//! The file is TOML-shaped but the name is not `.toml`: it was written for
//! the root of a FAT card, where 8.3 short names allow only a
//! three-character extension, and the name outlived the card.
//!
//! The firmware receives this over USB or BLE and keeps the text in its own
//! flash, where the next boot reads it. No TOML crate runs on the target,
//! so this is a small no_std
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
//! interval_s = 1            # position broadcast period
//! ping_interval_s = 5       # no-fix ping period
//! ```

use core::fmt::{self, Write};

use crate::lora;
use crate::session::{Knob, KNOBS};

/// Which halves of the air interface a node uses.
///
/// Position reporting is one-way traffic, so a node does not have to do
/// both halves. A tracker that is only ever reported *on* can leave its
/// receiver off; a base station that only collects reports never needs to
/// transmit. Each saves the power the unused half costs, and on a tracker
/// that is the larger saving by far: continuous RX draws current every
/// second between beacons, while a beacon is milliseconds of TX.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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

/// The `[power]` section: the duty cycle a file asks for, knob by knob.
///
/// Every other key in the file means "this, or the default if absent".
/// These are the only keys the board also keeps its own copy of - in RTC
/// RAM so they survive a deep sleep, in flash so they survive a flat cell,
/// changed live over BLE (see [`crate::session::Stored`]) - so an absent
/// key has to mean "leave the board's live value alone": under the usual
/// rule a push that changed only the beacon interval would carry
/// `ble_off_s = 0` by omission and silently kill a duty cycle set from the
/// app. Present means adopt, absent means untouched, and an explicit `0`
/// is still a request. A key that is present wins at the next boot,
/// because the file is what survives a reflash and the RTC copy is not.
///
/// The ranges are the ones [`KNOBS`] clamp a live write to, but a file is
/// edited by hand and read once, so an out-of-range value is reported as
/// [`ConfigError::OutOfRange`] rather than clamped.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct PowerConfig {
    asked: [Option<u32>; KNOBS.len()],
}

impl PowerConfig {
    /// What the file asked for `knob`, if it mentioned it.
    pub fn get(&self, knob: Knob) -> Option<u32> {
        self.asked[knob as usize]
    }

    pub fn set(&mut self, knob: Knob, secs: u32) {
        self.asked[knob as usize] = Some(secs);
    }

    /// The same, for building one in place.
    pub fn with(mut self, knob: Knob, secs: u32) -> Self {
        self.set(knob, secs);
        self
    }

    /// Whether the file mentioned no knob at all.
    pub fn is_empty(&self) -> bool {
        self.asked.iter().all(Option::is_none)
    }
}

/// Parsed and validated radio configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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
    /// Channels in the plan: this many, [`hop_step_khz`](Self::hop_step_khz)
    /// apart, centered on [`frequency_hz`](Self::frequency_hz), visited one
    /// per [`hop_dwell_ms`](Self::hop_dwell_ms) on a clock the frames
    /// themselves keep in step (see [`crate::hop`]).
    ///
    /// The value carries two decisions rather than one, and 0 and 1 are not
    /// the same setting:
    ///
    /// - **0** turns the slot clock off with the plan. Nothing schedules
    ///   transmissions: a node beacons on its own interval with random
    ///   jitter, and two nodes at the same rate collide whenever their
    ///   starts land closer together than a preamble takes to detect.
    /// - **1** keeps the clock on a single carrier at `frequency_hz`.
    ///   Nodes still share slots and still take turns inside them by
    ///   address, which is what keeps them off each other's transmissions;
    ///   there is simply nowhere to hop to. This is the default.
    /// - **more** adds channel diversity on top: a fade or an interferer
    ///   sitting on one carrier costs one beacon in `hop_channels` rather
    ///   than every beacon. It costs the join, since a node that knows
    ///   nobody's clock has to wait for a slot where its free-running
    ///   channel coincides with the network's.
    ///
    /// A plan wide enough to be legal below 500 kHz of bandwidth needs at
    /// least 50 channels in the 902-928 MHz band; see the README for when
    /// that is the plan to be on.
    pub hop_channels: u8,
    /// Spacing between hop channels, kHz. At least the signal bandwidth,
    /// or adjacent channels overlap.
    pub hop_step_khz: u16,
    /// How long the network stays on each channel, ms. One transmission per
    /// node per slot, which has to fit inside it with a guard at each end
    /// for clock error - see [`crate::hop::Plan::window_ms`].
    pub hop_dwell_ms: u16,
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
    /// as a duplicate. At the 1 s default interval that wrap is about four
    /// minutes; at 20 s it is 85.
    pub dedup_ttl_s: u16,
    /// Position broadcast interval in seconds, while the node has a fix.
    ///
    /// 0 silences the node's own transmissions altogether - positions and
    /// the no-fix ping both - which is what a card that says 0 has always
    /// meant.
    pub beacon_interval_s: u16,
    /// How often a node with no fix says so, in seconds: the period of the
    /// [`crate::lora::Ping`] that goes out in place of a position. Slower
    /// than the beacon by default, since a node searching for the sky has
    /// nothing new to report between pings. 0 sends no pings, and so does
    /// a `beacon_interval_s` of 0.
    pub ping_interval_s: u16,
    /// Which [`PositionPacket`](gps_proto::packet::PositionPacket) fields the
    /// beacon puts on the air, as a mask of the `FIELD_*` bits in
    /// [`crate::lora`]. Every extra field is airtime paid on every
    /// transmission, so the default carries position and nothing else.
    ///
    /// The mask travels in the frame, so nodes disagreeing about it is fine:
    /// a receiver decodes whatever the sender chose to include.
    pub beacon_fields: u8,
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
    /// Whether a stored board leaves its radio listening for a wake frame
    /// while the chip sleeps, so it can be reached on demand rather than
    /// on its wake-check cadence. See [`crate::sentry`].
    pub wake_enabled: bool,
    /// The sentry's receive window, ms: how long each listen lasts. What
    /// buys margin against the drift of the radio's own timer.
    pub wake_rx_ms: u16,
    /// The sentry's sleep between windows, ms. What buys current: the
    /// receiver is off for this long out of every cycle.
    pub wake_sleep_ms: u16,
    /// The carrier the sentry listens on and a wake frame is sent on, Hz;
    /// 0 for [`frequency_hz`](Self::frequency_hz). A sentry locks on any
    /// LoRa symbol in its window and a foreign preamble costs it a whole
    /// cycle, so a carrier the fleet's beacons are not on is what keeps a
    /// stored board reachable beside a busy one.
    pub wake_frequency_hz: u32,
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
            // band, so with hopping turned off it may sit on one channel
            // indefinitely; anything narrower has to hop there. With hopping
            // on, 500 kHz also keeps the beacon well inside a one-second
            // slot: 289 ms, against the 1.15 s the same frame costs at
            // BW125, which is longer than the band's 400 ms occupancy cap.
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
            // One channel, and the slot clock that would hop across many.
            //
            // A 500 kHz signal is a digital modulation in the 902-928 MHz
            // band, which may hold one carrier with no dwell or duty
            // ceiling, so at the default modulation hopping buys no air
            // time that this does not already have. What is worth keeping
            // is the clock: it is what cuts a slot into turns and gives
            // each address one, and without it two nodes beaconing at the
            // same rate pick random starts and lose both frames whenever
            // those land within a preamble of each other. So the plan
            // stays, one channel wide.
            //
            // Raise this to hop for real - 50 channels is the band's floor
            // below 500 kHz of bandwidth, and the answer to a carrier
            // somebody else is sitting on. It is not free: a node that
            // knows nobody's clock then coincides with the network on
            // about one slot in fifty, so it waits, on average, fifty
            // beacon intervals divided by the number of nodes
            // transmitting, before it hears anything at all.
            hop_channels: 1,
            hop_step_khz: 500,
            hop_dwell_ms: 1_000,
            address: 1,
            // Leaf by default: repeating is a job you give one well-placed
            // node, not something every node should do to every frame.
            role: Role::Leaf,
            // Allow one repeat, so dropping a repeater into an existing
            // fleet works without reconfiguring the nodes already deployed.
            max_hops: 1,
            dedup_ttl_s: 3,
            // Every second: every hop slot at the default dwell. Hopping
            // caps one visit to a channel, not how often a node transmits,
            // so the interval is a trade of battery life and shared air time
            // against how stale a position is allowed to get - and a node
            // being followed in flight wants the freshest one. The ping is
            // slower: a node with no fix has nothing new to say every second.
            beacon_interval_s: 1,
            ping_interval_s: 5,
            // Position only. Everything else a fix produces is air time
            // that has to be paid on every single transmission.
            beacon_fields: crate::lora::FIELDS_DEFAULT,
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
            // On: a stored board that can be called is the point of
            // storing one. The two periods are the measured shape - a
            // 300 ms window every 3 s listens 9% of the time and tolerates
            // several percent of timer drift either way.
            wake_enabled: true,
            wake_rx_ms: 300,
            wake_sleep_ms: 3_000,
            wake_frequency_hz: 0,
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

    /// The longest a transmit can hold the hardware loop, ms: the TxDone
    /// deadline, plus a whole hop slot for a send that finds its clock has
    /// moved and waits for the next window. What a sleep that arrives
    /// mid-beacon has to budget for.
    pub fn tx_worst_case_ms(&self) -> u32 {
        self.tx_poll_timeout_ms() + u32::from(self.hop_dwell_ms)
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
        self.time_on_air_preamble_us(payload_len, PREAMBLE_SYMBOLS)
    }

    /// One LoRa symbol at the current spreading factor and bandwidth, in
    /// microseconds.
    ///
    /// The four legal bandwidths all divide 1 MHz evenly, so this is exact
    /// in integer math - which is what keeps every airtime below off the
    /// floating point a no_std target would rather not carry.
    pub fn symbol_time_us(&self) -> u32 {
        let sf = self.spreading_factor.clamp(5, 12) as u32;
        // Bandwidth in Hz; 62 is the config's shorthand for 62.5 kHz.
        let bw_hz = if self.bandwidth_khz == 62 {
            62_500
        } else {
            self.bandwidth_khz as u32 * 1_000
        };
        (1u32 << sf) * (1_000_000 / bw_hz)
    }

    /// [`time_on_air_us`](Self::time_on_air_us) with the preamble length
    /// given rather than assumed.
    ///
    /// Everything this firmware sends on the network uses
    /// [`PREAMBLE_SYMBOLS`]. A transmission meant to be caught by a receiver
    /// that is only listening part of the time does not: its preamble has to
    /// span that receiver's whole cycle, which is thousands of symbols
    /// rather than eight, and is most of what such a frame costs.
    pub fn time_on_air_preamble_us(&self, payload_len: usize, preamble_syms: u32) -> u32 {
        let sf = self.spreading_factor.clamp(5, 12) as u32;
        let t_sym_us = self.symbol_time_us();

        // Preamble is (n + 4.25) symbols. 4.25 = 17/4, so scale the symbol
        // count by 4 and divide once to keep the quarter-symbol exact.
        let t_preamble_us = (4 * preamble_syms + 17) * t_sym_us / 4;

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

    /// Bytes every frame this node sends carries ahead of its payload: the
    /// header and the slot clock's sync word, which every frame carries
    /// since every node is on the schedule.
    pub fn frame_overhead(&self) -> usize {
        crate::lora::HEADER_SYNC_LEN
    }

    /// Time-on-air of one beacon transmission at the current settings, in
    /// microseconds: the frame header (and sync word, when hopping) plus
    /// whichever position fields [`beacon_fields`](Self::beacon_fields)
    /// selects.
    pub fn beacon_airtime_us(&self) -> u32 {
        let payload = self.frame_overhead() + crate::lora::position_msg_len(self.beacon_fields);
        self.time_on_air_us(payload)
    }

    /// Time-on-air of one no-fix ping at the current settings, in
    /// microseconds.
    pub fn ping_airtime_us(&self) -> u32 {
        self.time_on_air_us(self.frame_overhead() + crate::lora::PING_MSG_LEN)
    }

    /// Time-on-air of the lean beacon - header, sync word and position only
    /// - in microseconds: the unit a hop slot is cut into turns by. Taken
    /// from the modulation alone rather than from this node's own
    /// [`beacon_fields`](Self::beacon_fields), so every node on a network
    /// cuts the slot the same way whatever each of them chooses to send.
    pub fn hop_unit_airtime_us(&self) -> u32 {
        self.time_on_air_us(
            self.frame_overhead() + crate::lora::position_msg_len(crate::lora::FIELDS_DEFAULT),
        )
    }

    /// Time a receiver needs from the start of a preamble to the end of the
    /// explicit header, in microseconds: the preamble plus the eight
    /// symbols the header occupies. A preamble detection that has not
    /// become a valid header by then was noise.
    pub fn header_time_us(&self) -> u32 {
        self.time_on_air_us(0)
    }
}

/// Preamble length every frame on the network is sent with, in symbols.
///
/// The SX126x power-up default, and all a receiver held in continuous
/// receive needs: it is already listening when the preamble starts, so the
/// preamble only has to be long enough to lock onto. A receiver that is
/// only listening part of the time is the case this does not cover.
pub const PREAMBLE_SYMBOLS: u32 = 8;

/// Ceiling on [`RadioConfig::hop_dwell_ms`]. Past ten seconds a slot is no
/// longer a hop, it is a channel with a schedule.
pub const HOP_DWELL_MAX_MS: u16 = 10_000;
/// Floor on [`RadioConfig::hop_dwell_ms`]: below this the guards leave no
/// room for even the fastest beacon.
pub const HOP_DWELL_MIN_MS: u16 = 100;
/// Bounds on [`RadioConfig::hop_step_khz`]. 25 kHz is the narrowest spacing
/// the band rules recognize as separate channels; 5 MHz steps with more than
/// a handful of channels would not fit any band the radio covers.
pub const HOP_STEP_MIN_KHZ: u16 = 25;
pub const HOP_STEP_MAX_KHZ: u16 = 5_000;

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
pub const RADIO_CONFIG_LEN: usize = 42;

/// Length of the blob before the hop plan was appended. A board on that
/// firmware sends this much, and its byte 27 - now `hop_channels` - was a
/// reserved zero, which reads back as a one-channel plan.
/// [`RadioConfig::decode`] accepts any length from this one up, so a
/// newer app can still read an older board; the ping interval, appended
/// after the plan, reads as the beacon interval when absent, which is the
/// period such a board pings on.
pub const RADIO_CONFIG_LEN_V1: usize = 28;
/// Length with the hop plan but before the ping interval.
const RADIO_CONFIG_LEN_HOP: usize = 32;
/// Length with the ping interval but before the wake sentry's periods.
const RADIO_CONFIG_LEN_PING: usize = 34;

/// Layout version in byte 0, so an app meeting a newer firmware can reject
/// the blob rather than misread it.
pub const RADIO_CONFIG_VERSION: u8 = 1;

// byte 1 (misc bools)
const RCFG_RX_BOOST: u8 = 1 << 0;
// Bit 1 was the SD card's enable. The firmware no longer drives a card,
// so the bit is written clear and ignored on read; it stays reserved so
// the blob's other flags keep their places.
const RCFG_VERBOSE: u8 = 1 << 2;
const RCFG_DCDC: u8 = 1 << 3;
const RCFG_DIO2_RF_SWITCH: u8 = 1 << 4;
const RCFG_WAKE: u8 = 1 << 5;
// byte 2 (GPS constellations)
const RCFG_GPS: u8 = 1 << 0;
const RCFG_GLONASS: u8 = 1 << 1;
const RCFG_GALILEO: u8 = 1 << 2;
const RCFG_BEIDOU: u8 = 1 << 3;
const RCFG_QZSS: u8 = 1 << 4;
const RCFG_SBAS: u8 = 1 << 5;

impl RadioConfig {
    /// The carrier a sentry listens on and a wake frame goes out on.
    pub fn wake_carrier_hz(&self) -> u32 {
        if self.wake_frequency_hz == 0 {
            self.frequency_hz
        } else {
            self.wake_frequency_hz
        }
    }

    /// Encode the config as the fixed [`RADIO_CONFIG_LEN`]-byte read-back
    /// blob (little-endian).
    pub fn encode(&self) -> [u8; RADIO_CONFIG_LEN] {
        let mut b = [0u8; RADIO_CONFIG_LEN];
        b[0] = RADIO_CONFIG_VERSION;
        let mut flags = 0u8;
        if self.rx_boost {
            flags |= RCFG_RX_BOOST;
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
        if self.wake_enabled {
            flags |= RCFG_WAKE;
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
        b[27] = self.hop_channels;
        b[28..30].copy_from_slice(&self.hop_step_khz.to_le_bytes());
        b[30..32].copy_from_slice(&self.hop_dwell_ms.to_le_bytes());
        b[32..34].copy_from_slice(&self.ping_interval_s.to_le_bytes());
        b[34..36].copy_from_slice(&self.wake_rx_ms.to_le_bytes());
        b[36..38].copy_from_slice(&self.wake_sleep_ms.to_le_bytes());
        b[38..42].copy_from_slice(&self.wake_frequency_hz.to_le_bytes());
        b
    }

    /// Decode a read-back blob. `None` for a short buffer, an unknown layout
    /// version, or an enum byte this build does not recognize - the caller
    /// then keeps its own values rather than acting on a half-read config.
    /// Trailing bytes are tolerated so a future layout can only grow, and a
    /// [`RADIO_CONFIG_LEN_V1`] blob from a board that predates hopping reads
    /// as a config with hopping off.
    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() < RADIO_CONFIG_LEN_V1 || b[0] != RADIO_CONFIG_VERSION {
            return None;
        }
        let flags = b[1];
        let g = b[2];
        let u16at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let hopping = b.len() >= RADIO_CONFIG_LEN_HOP;
        let has_ping = b.len() >= RADIO_CONFIG_LEN_PING;
        let has_wake = b.len() >= RADIO_CONFIG_LEN;
        let defaults = Self::default();
        Some(Self {
            frequency_hz: u32::from_le_bytes([b[4], b[5], b[6], b[7]]),
            spreading_factor: b[8],
            bandwidth_khz: u16at(9),
            coding_rate: b[11],
            power_dbm: b[12] as i8,
            rx_boost: flags & RCFG_RX_BOOST != 0,
            // A board from before the schedule was always on sent a zero
            // here, and a zero has meant "one channel" since: the clock on
            // a single carrier.
            hop_channels: if hopping { b[27].max(1) } else { 1 },
            hop_step_khz: if hopping { u16at(28) } else { defaults.hop_step_khz },
            hop_dwell_ms: if hopping { u16at(30) } else { defaults.hop_dwell_ms },
            address: b[13],
            role: Role::from_wire(b[3])?,
            max_hops: b[14],
            dedup_ttl_s: u16at(15),
            beacon_interval_s: u16at(17),
            ping_interval_s: if has_ping { u16at(32) } else { u16at(17) },
            beacon_fields: b[19],
            verbose: flags & RCFG_VERBOSE != 0,
            dcdc_enabled: flags & RCFG_DCDC != 0,
            dio2_rf_switch: flags & RCFG_DIO2_RF_SWITCH != 0,
            tcxo_volts: TcxoVolts::from_trim(b[20])?,
            tcxo_startup_ms: u16at(21),
            // A board from before the sentry existed reads as one without:
            // its flag byte never had the bit, and the periods default.
            wake_enabled: has_wake && flags & RCFG_WAKE != 0,
            wake_rx_ms: if has_wake { u16at(34) } else { defaults.wake_rx_ms },
            wake_sleep_ms: if has_wake { u16at(36) } else { defaults.wake_sleep_ms },
            wake_frequency_hz: if has_wake {
                u32::from_le_bytes([b[38], b[39], b[40], b[41]])
            } else {
                0
            },
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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

// ---------------------------------------------------------------------------
// The keys
// ---------------------------------------------------------------------------

/// What a key takes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Kind {
    /// An integer inside inclusive bounds.
    Int { min: i64, max: i64 },
    /// Zero, or an integer inside inclusive bounds: a duration whose zero
    /// is a setting of its own.
    IntOrZero { min: i64, max: i64 },
    /// One of a fixed set of integers.
    IntChoice(&'static [i64]),
    Bool,
    /// One of a fixed set of words, quoted in the file.
    Choice(&'static [&'static str]),
    /// A comma-separated list of position field names, quoted.
    Fields,
}

/// One key of the config file: where it goes, what it takes, what it is
/// for. This table is what the parser checks a value against, what the
/// example file is printed from, and what the app's editor shows beside
/// each field - the one description of each key.
pub struct Key {
    pub section: &'static str,
    pub name: &'static str,
    pub kind: Kind,
    /// Printed into the example commented out: a fact about the board
    /// that is not to be retuned, or a duty cycle that a push must not
    /// undo.
    pub commented: bool,
    pub doc: &'static str,
    show: fn(&RadioConfig, &mut dyn Write) -> fmt::Result,
}

impl Key {
    /// Write the value `cfg` holds for this key, as the file spells it.
    pub fn show(&self, cfg: &RadioConfig, w: &mut dyn Write) -> fmt::Result {
        (self.show)(cfg, w)
    }
}

/// A `[section]` of the file and what it groups.
pub struct Section {
    pub name: &'static str,
    pub title: &'static str,
    pub doc: &'static str,
}

pub const SECTIONS: &[Section] = &[
    Section {
        name: "radio",
        title: "Radio",
        doc: "frequency, spreading factor, bandwidth, coding rate and power are the LoRa link-budget knobs: together they set sensitivity, on-air time and radiated power - i.e. range. The three hop_* keys are the channel plan and the slot clock that runs it. The default plan is one channel: a 500 kHz signal may hold a single carrier in the 902-928 MHz band as often as it likes, so hopping buys no air time there that the default modulation does not already have. What the plan is kept for is the clock, which cuts every second into turns and gives each address one - without it two nodes beaconing at the same rate talk over each other. Raise hop_channels to hop for real; see the notes on that key. The four commented keys describe the board the firmware is running on, not a preference: the defaults are the Wio-S3's own hardware, and inside the module SX1262 DIO2 is the VCTL of an SKY13453-385LF antenna switch and DIO3 is that same switch's VDD. Turning the switch off, or supplying it below the 2.5 V its datasheet specifies, leaves the PA transmitting +22 dBm into an isolated port, so the firmware refuses both, logs that it did, and runs the board's values instead.",
    },
    Section {
        name: "network",
        title: "Network",
        doc: "Everything is sent. Leaves hear each other directly, so a fleet of nothing but leaves is a working network and these defaults need no changing. A repeater is for covering ground no pair of leaves can reach across on their own. tx_only and rx_only split a leaf in half, for a deployment where the reporting only ever runs one way.",
    },
    Section {
        name: "beacon",
        title: "Beacon",
        doc: "What goes out, and how often.",
    },
    Section {
        name: "wake",
        title: "Wake on LoRa",
        doc: "How a stored board is reached without waiting for its wake check. With wake_enabled on, the park before every deep sleep leaves the SX1262 in its own sniff loop - listening for wake_rx_ms, sleeping for wake_sleep_ms, on frequency_hz and a sync word ordinary traffic never uses - and the S3 sleeps until the radio pulls DIO1 with a completed wake frame. A board that wants another woken sends a burst of wake frames behind a preamble sized to the target's cycle: cd tools && pixi run board-wake --target 3. The woken board comes up idle, answers with a ping, and can then be connected to over BLE. The timer wake-check stays armed as a backstop. The window sets the tolerance for the radio's timer drift (half of 2*rx - 10 ms - 24 symbols, either way) and the sleep sets the current: at the defaults the receiver is on 9% of the time, and a wake frame is about 3.4 s on air. Both boards must share these values, because the preamble one sends is computed from them. Refused on a hopping plan, since a wake frame would hold one channel far longer than hopping allows.",
    },
    Section {
        name: "debug",
        title: "Debug",
        doc: "",
    },
    Section {
        name: "gps",
        title: "GPS",
        doc: "MAX-M10 receiver, applied as one UBX-CFG-VALSET to the RAM layer. An absent [gps] section leaves the module at its factory concurrent set.",
    },
    Section {
        name: "power",
        title: "Power",
        doc: "The duty cycle: how much of the time the board is reachable. These five are the only keys in this file the board also keeps its own copy of, in RTC RAM so they survive a deep sleep and in flash so they survive a flat cell, and they are the only ones an app can change live over BLE. So they follow a different rule to every other key here: an absent one leaves the board's current value alone rather than resetting it to the default. That is why all five are commented out - a push of this file should not silently undo a duty cycle somebody set from the app. Uncomment one and it wins at the next boot, because the file is what survives a reflash and the RTC copy is not. Writing an explicit 0 is a request, not an absence: it is how a file turns a duty cycle off. What is NOT here is the mode. The board has four - stored, idle, tracking and listening - and which one it is in is a command from the app or the console, never a file: a card that said tracking would put every board it was ever copied into onto the air. Each key belongs to one mode: stored reads sleep_interval_s and adv_window_s, idle reads idle_timeout_s, tracking reads ble_off_s and ble_on_s, and listening reads none of them.",
    },
];

/// The `role` choices, in the order [`Role::as_str`] names them.
const ROLES: [Role; 4] = [Role::Leaf, Role::Repeater, Role::TxOnly, Role::RxOnly];
const ROLE_NAMES: [&str; 4] = ["leaf", "repeater", "tx_only", "rx_only"];
const TCXO: [TcxoVolts; 8] = [
    TcxoVolts::V1_6,
    TcxoVolts::V1_7,
    TcxoVolts::V1_8,
    TcxoVolts::V2_2,
    TcxoVolts::V2_4,
    TcxoVolts::V2_7,
    TcxoVolts::V3_0,
    TcxoVolts::V3_3,
];
const TCXO_NAMES: [&str; 8] = ["1.6", "1.7", "1.8", "2.2", "2.4", "2.7", "3.0", "3.3"];
const POWER_MODES: [PowerMode; 3] = [PowerMode::Full, PowerMode::PsmOnOff, PowerMode::PsmCyclic];
const POWER_MODE_NAMES: [&str; 3] = ["full", "psmoo", "psmct"];
const DYN_MODELS: [DynModel; 8] = [
    DynModel::Portable,
    DynModel::Stationary,
    DynModel::Pedestrian,
    DynModel::Automotive,
    DynModel::Sea,
    DynModel::Airborne1g,
    DynModel::Airborne2g,
    DynModel::Airborne4g,
];
const DYN_MODEL_NAMES: [&str; 8] = [
    "portable",
    "stationary",
    "pedestrian",
    "automotive",
    "sea",
    "airborne1g",
    "airborne2g",
    "airborne4g",
];
const BANDWIDTHS: [i64; 4] = [62, 125, 250, 500];

/// Every key the file takes. Aliases the parser also accepts
/// (`beacon_interval_s`, `beacon_fields`, `dyn_model`, the old mesh's
/// `lifetime`) are not keys of their own.
pub const KEYS: &[Key] = &[
    Key {
        section: "radio",
        name: "frequency_hz",
        kind: Kind::Int { min: RF_MIN_HZ as i64, max: RF_MAX_HZ as i64 },
        commented: false,
        doc: "RF center frequency in Hz, 150000000-960000000. With more than one hop channel this is the center of the plan and the channels straddle it; with one it is simply the carrier.",
        show: |c, w| write!(w, "{}", c.frequency_hz),
    },
    Key {
        section: "radio",
        name: "hop_channels",
        kind: Kind::Int { min: 0, max: 255 },
        commented: false,
        doc: "Channels in the plan, 1-255 (0 is read as 1). 1, the default, is the slot clock on the single carrier at frequency_hz: nodes share slots and take turns inside them by address, which is what keeps two nodes beaconing at the same rate from talking over each other, and it is the right answer at 500 kHz bandwidth, where the band lets one carrier be held as often as you like. More than 1 adds channel diversity: a fade or an interferer parked on one carrier costs one beacon in hop_channels rather than every beacon, and below 500 kHz of bandwidth 50 channels is what the 902-928 MHz band requires. Hopping is not free - every node hops on one shared clock, from GPS where a node has a fix and from the frames it hears where it does not, so a node that knows nobody's clock spends roughly hop_channels times the beacon interval, divided by the number of nodes transmitting, before it hears the network at all.",
        show: |c, w| write!(w, "{}", c.hop_channels),
    },
    Key {
        section: "radio",
        name: "hop_step_khz",
        kind: Kind::Int { min: HOP_STEP_MIN_KHZ as i64, max: HOP_STEP_MAX_KHZ as i64 },
        commented: false,
        doc: "Spacing between hop channels in kHz, 25-5000. At least the bandwidth, or adjacent channels overlap. Unused with one channel, since there is nothing to space. 500 kHz for 50 channels spans 902.75-927.25 MHz around the 915 MHz center.",
        show: |c, w| write!(w, "{}", c.hop_step_khz),
    },
    Key {
        section: "radio",
        name: "hop_dwell_ms",
        kind: Kind::Int { min: HOP_DWELL_MIN_MS as i64, max: HOP_DWELL_MAX_MS as i64 },
        commented: false,
        doc: "Slot length in ms, 100-10000, and so how long every node stays on each channel. One transmission per node per slot, which has to fit the slot with a 100 ms guard at each end, so at 1000 ms a beacon may be up to 800 ms on air. The band caps one hop visit at 400 ms; the default beacon is 289 ms. The 800 ms window is cut into as many default beacons as fit - two - and nodes take those turns by address, so two nodes may beacon every second without overlapping.",
        show: |c, w| write!(w, "{}", c.hop_dwell_ms),
    },
    Key {
        section: "radio",
        name: "spreading_factor",
        kind: Kind::Int { min: 5, max: 12 },
        commented: false,
        doc: "LoRa spreading factor, 5-12. Higher gives longer range at a lower data rate.",
        show: |c, w| write!(w, "{}", c.spreading_factor),
    },
    Key {
        section: "radio",
        name: "bandwidth_khz",
        kind: Kind::IntChoice(&BANDWIDTHS),
        commented: false,
        doc: "LoRa bandwidth in kHz: 62, 125, 250 or 500 (62 means 62.5). Narrower gives longer range at a lower data rate, and a longer frame: the default beacon is 289 ms at 500 and 1150 ms at 125, past the 400 ms one hop visit may occupy a channel. In the 902-928 MHz band only 500 is wide enough to hold a single carrier; anything narrower has to hop, so it needs hop_channels raised to 50 as well.",
        show: |c, w| write!(w, "{}", c.bandwidth_khz),
    },
    Key {
        section: "radio",
        name: "coding_rate",
        kind: Kind::Int { min: 5, max: 8 },
        commented: false,
        doc: "Coding rate denominator 5-8 for 4/5..4/8. Higher adds forward error correction at a lower data rate.",
        show: |c, w| write!(w, "{}", c.coding_rate),
    },
    Key {
        section: "radio",
        name: "power_dbm",
        kind: Kind::Int { min: -9, max: 22 },
        commented: false,
        doc: "Transmit power in dBm, -9 to 22 on the high-power PA.",
        show: |c, w| write!(w, "{}", c.power_dbm),
    },
    Key {
        section: "radio",
        name: "rx_boost",
        kind: Kind::Bool,
        commented: false,
        doc: "Boosted receiver gain: roughly +2 dB of sensitivity for a few mA more current whenever the node is listening.",
        show: |c, w| write!(w, "{}", c.rx_boost),
    },
    Key {
        section: "radio",
        name: "dcdc_enabled",
        kind: Kind::Bool,
        commented: true,
        doc: "Use the internal DC-DC (SMPS). The Wio-S3 carries the inductor it needs.",
        show: |c, w| write!(w, "{}", c.dcdc_enabled),
    },
    Key {
        section: "radio",
        name: "dio2_rf_switch",
        kind: Kind::Bool,
        commented: true,
        doc: "Let the radio drive its own antenna switch from DIO2 (SetDio2AsRfSwitchCtrl). The Wio-S3 wires DIO2 to the SKY13453-385LF between the PA and the LoRa port, so this must stay on; the firmware ignores an off.",
        show: |c, w| write!(w, "{}", c.dio2_rf_switch),
    },
    Key {
        section: "radio",
        name: "tcxo_volts",
        kind: Kind::Choice(&TCXO_NAMES),
        commented: true,
        doc: "Supply the radio drives DIO3 at. Named for the TCXO, but on the Wio-S3 DIO3 is also the antenna switch's VDD, specified 2.5-3.5 V - so only 2.7, 3.0 and 3.3 are usable and the firmware raises anything lower.",
        show: |c, w| write!(w, "\"{}\"", c.tcxo_volts.as_str()),
    },
    Key {
        section: "radio",
        name: "tcxo_startup_ms",
        kind: Kind::Int { min: 1, max: 1_000 },
        commented: true,
        doc: "How long the radio waits for the TCXO to settle before using the clock, 1-1000 ms.",
        show: |c, w| write!(w, "{}", c.tcxo_startup_ms),
    },
    Key {
        section: "wake",
        name: "wake_enabled",
        kind: Kind::Bool,
        commented: false,
        doc: "Leave the radio listening for a wake frame through every deep sleep. Off, a stored board is reachable only during its wake checks.",
        show: |c, w| write!(w, "{}", c.wake_enabled),
    },
    Key {
        section: "wake",
        name: "wake_rx_ms",
        kind: Kind::Int { min: 50, max: 5_000 },
        commented: false,
        doc: "The sentry's receive window, 50-5000 ms. Longer tolerates more drift in the radio's own timer and costs proportionally more current; under about 105 ms at the default modulation no wake preamble fits at all and the sentry is refused.",
        show: |c, w| write!(w, "{}", c.wake_rx_ms),
    },
    Key {
        section: "wake",
        name: "wake_sleep_ms",
        kind: Kind::Int { min: 200, max: 30_000 },
        commented: false,
        doc: "The sentry's sleep between windows, 200-30000 ms. Longer is cheaper and makes every wake frame longer by the same amount, so a waker spends more air time per attempt.",
        show: |c, w| write!(w, "{}", c.wake_sleep_ms),
    },
    Key {
        section: "wake",
        name: "wake_frequency_hz",
        kind: Kind::Int { min: 0, max: RF_MAX_HZ as i64 },
        commented: false,
        doc: "The carrier the sentry listens on and wake frames go out on, Hz; 0 means frequency_hz. A sentry locks on any LoRa symbol that lands in its window, and a preamble that is not a wake frame's - a beacon, another network, noise - holds it for a restarted timer and then costs it a whole sleep, during which a wake frame is lost. So put the sentry where the beacons are not: a fleet that beacons every second on frequency_hz will blind a sentry on the same carrier most of the time. Every board that calls or is called must share this value. 927 MHz measured about ten times quieter than the rest of the 902-928 band on one bench.",
        show: |c, w| write!(w, "{}", c.wake_frequency_hz),
    },
    Key {
        section: "network",
        name: "address",
        kind: Kind::Int { min: 1, max: 255 },
        commented: false,
        doc: "This node's address, 1-255. Must be unique among the nodes that transmit: two senders sharing an address are mutually deaf, each dropping the other's broadcasts as an echo of its own, so neither ever sees the other. An rx_only node never puts its address on the air, so it can keep the default. The address also picks the node's turn inside a slot: number a fleet 1, 2, 3... and the first 2 x interval_s of them never overlap on the air.",
        show: |c, w| write!(w, "{}", c.address),
    },
    Key {
        section: "network",
        name: "role",
        kind: Kind::Choice(&ROLE_NAMES),
        commented: false,
        doc: "leaf transmits its own position and receives everyone else's. repeater also retransmits other nodes' broadcasts, extending range past one radio horizon at the cost of doubling the traffic it forwards. tx_only beacons without ever switching the receiver on, which is the largest power saving available to a node nobody needs to track from. rx_only listens without ever transmitting, for a base station that only collects.",
        show: |c, w| write!(w, "\"{}\"", c.role.as_str()),
    },
    Key {
        section: "network",
        name: "max_hops",
        kind: Kind::Int { min: 0, max: MAX_HOPS_LIMIT as i64 },
        commented: false,
        doc: "Retransmissions allowed for a broadcast this node sends, 0-8. 0 means no repeater forwards it.",
        show: |c, w| write!(w, "{}", c.max_hops),
    },
    Key {
        section: "network",
        name: "dedup_ttl_s",
        kind: Kind::Int { min: 1, max: 3600 },
        commented: false,
        doc: "How long a broadcast is remembered so a repeat of it is dropped rather than delivered or forwarded again, 1-3600 s. Must stay under 200 x interval_s: the beacon id wraps every 256 frames, and past that a node starts suppressing its own later broadcasts as duplicates. The file is refused otherwise.",
        show: |c, w| write!(w, "{}", c.dedup_ttl_s),
    },
    Key {
        section: "beacon",
        name: "interval_s",
        kind: Kind::Int { min: 0, max: 3600 },
        commented: false,
        doc: "Position broadcast period in seconds while the node has a fix, 0-3600. A node transmits at most once per slot, so 1 is every slot at the default dwell. It also sets how many nodes fit: a slot holds two default beacons, so 2 x interval_s addresses beacon without overlapping - two every second, four every two seconds, ten every five. 0 silences the node's own transmissions altogether, pings included, as does a role of rx_only.",
        show: |c, w| write!(w, "{}", c.beacon_interval_s),
    },
    Key {
        section: "beacon",
        name: "ping_interval_s",
        kind: Kind::Int { min: 0, max: 3600 },
        commented: false,
        doc: "How often a node with no fix says so, 0-3600 s: a small ping goes out in place of the position, so a node that cannot see the sky is still heard. Slower than the beacon by default, since a searching receiver has nothing new to report every second. 0 sends no pings.",
        show: |c, w| write!(w, "{}", c.ping_interval_s),
    },
    Key {
        section: "beacon",
        name: "fields",
        kind: Kind::Fields,
        commented: false,
        doc: "Which GPS fields each broadcast carries: lat, lon, altitude, speed, course, sats, time. lat and lon are required. More fields cost more air time.",
        show: |c, w| lora::write_fields(c.beacon_fields, w),
    },
    Key {
        section: "debug",
        name: "verbose",
        kind: Kind::Bool,
        commented: false,
        doc: "Log every event on the USB console, on top of the events that are always logged. Costs nothing when nothing is attached; turn it off for a quiet console or when a tool is parsing it.",
        show: |c, w| write!(w, "{}", c.verbose),
    },
    Key {
        section: "gps",
        name: "gps_enabled",
        kind: Kind::Bool,
        commented: false,
        doc: "Enable the GPS (US) constellation.",
        show: |c, w| write!(w, "{}", c.gps.gps_enabled),
    },
    Key {
        section: "gps",
        name: "glonass_enabled",
        kind: Kind::Bool,
        commented: false,
        doc: "Enable the GLONASS (Russia) constellation. The M10 tracks a limited concurrent set.",
        show: |c, w| write!(w, "{}", c.gps.glonass_enabled),
    },
    Key {
        section: "gps",
        name: "galileo_enabled",
        kind: Kind::Bool,
        commented: false,
        doc: "Enable the Galileo (EU) constellation.",
        show: |c, w| write!(w, "{}", c.gps.galileo_enabled),
    },
    Key {
        section: "gps",
        name: "beidou_enabled",
        kind: Kind::Bool,
        commented: false,
        doc: "Enable the BeiDou (China) constellation.",
        show: |c, w| write!(w, "{}", c.gps.beidou_enabled),
    },
    Key {
        section: "gps",
        name: "qzss_enabled",
        kind: Kind::Bool,
        commented: false,
        doc: "Enable the QZSS (Japan) augmentation.",
        show: |c, w| write!(w, "{}", c.gps.qzss_enabled),
    },
    Key {
        section: "gps",
        name: "sbas_enabled",
        kind: Kind::Bool,
        commented: false,
        doc: "Enable SBAS augmentation (WAAS, EGNOS and similar).",
        show: |c, w| write!(w, "{}", c.gps.sbas_enabled),
    },
    Key {
        section: "gps",
        name: "power_mode",
        kind: Kind::Choice(&POWER_MODE_NAMES),
        commented: false,
        doc: "GPS power mode: full, psmoo (power-save on/off) or psmct (power-save cyclic tracking).",
        show: |c, w| write!(w, "\"{}\"", c.gps.power_mode.as_str()),
    },
    Key {
        section: "gps",
        name: "meas_rate_ms",
        kind: Kind::Int { min: 25, max: 10_000 },
        commented: false,
        doc: "GPS measurement and navigation period in ms, 25-10000.",
        show: |c, w| write!(w, "{}", c.gps.meas_rate_ms),
    },
    Key {
        section: "gps",
        name: "dynamic_model",
        kind: Kind::Choice(&DYN_MODEL_NAMES),
        commented: false,
        doc: "GPS motion model: portable, stationary, pedestrian, automotive, sea, airborne1g, airborne2g or airborne4g.",
        show: |c, w| write!(w, "\"{}\"", c.gps.dyn_model.as_str()),
    },
    Key {
        section: "power",
        name: KNOBS[Knob::BleOff as usize].name,
        kind: Kind::IntOrZero { min: KNOBS[Knob::BleOff as usize].min as i64, max: KNOBS[Knob::BleOff as usize].max as i64 },
        commented: true,
        doc: KNOBS[Knob::BleOff as usize].doc,
        show: |_, w| write!(w, "0"),
    },
    Key {
        section: "power",
        name: KNOBS[Knob::AdvWindow as usize].name,
        kind: Kind::IntOrZero { min: KNOBS[Knob::AdvWindow as usize].min as i64, max: KNOBS[Knob::AdvWindow as usize].max as i64 },
        commented: true,
        doc: KNOBS[Knob::AdvWindow as usize].doc,
        show: |_, w| write!(w, "0"),
    },
    Key {
        section: "power",
        name: KNOBS[Knob::BleOn as usize].name,
        kind: Kind::IntOrZero { min: KNOBS[Knob::BleOn as usize].min as i64, max: KNOBS[Knob::BleOn as usize].max as i64 },
        commented: true,
        doc: KNOBS[Knob::BleOn as usize].doc,
        show: |_, w| write!(w, "0"),
    },
    Key {
        section: "power",
        name: KNOBS[Knob::SleepInterval as usize].name,
        kind: Kind::IntOrZero { min: KNOBS[Knob::SleepInterval as usize].min as i64, max: KNOBS[Knob::SleepInterval as usize].max as i64 },
        commented: true,
        doc: KNOBS[Knob::SleepInterval as usize].doc,
        show: |_, w| write!(w, "0"),
    },
    Key {
        section: "power",
        name: KNOBS[Knob::IdleTimeout as usize].name,
        kind: Kind::IntOrZero { min: KNOBS[Knob::IdleTimeout as usize].min as i64, max: KNOBS[Knob::IdleTimeout as usize].max as i64 },
        commented: true,
        doc: KNOBS[Knob::IdleTimeout as usize].doc,
        show: |_, w| write!(w, "0"),
    },
];

/// The key called `name`, if there is one. Aliases are resolved first.
pub fn key(name: &str) -> Option<&'static Key> {
    let name = canonical(name);
    KEYS.iter().find(|k| k.name == name)
}

/// The spelling the table uses for a key that has more than one.
fn canonical(name: &str) -> &str {
    match name {
        "beacon_interval_s" => "interval_s",
        "beacon_fields" => "fields",
        "dyn_model" => "dynamic_model",
        other => other,
    }
}

/// The header every generated example starts with.
const EXAMPLE_HEADER: &str = "RADIO.CFG - telemetry-in-midair radio configuration reference. Generated from the key table in the protocol crate (cargo run --example radio_example in proto/); every value below is the firmware default. The firmware reads at most 1024 bytes of config, which the comments here put this file well over, so it is a reference rather than a file to push: strip the comments and what remains fits. The tools do that (cd tools && pixi run board-config --address 3 pushes it; add --dry-run --save ../RADIO.CFG for the stripped file). A config pushed over USB or BLE is applied live and kept in the board's own flash, so it survives a power cycle. Every key is optional - an absent key keeps its default, and an empty file is valid. Section headers are cosmetic: keys are unique across sections, so a key works regardless of which [section] it sits under.";

/// Column the example's comments wrap at.
const EXAMPLE_WIDTH: usize = 76;

/// Write the reference config file: every key at its default, with its
/// doc as a comment, the board facts and the duty cycle commented out.
/// What `RADIO.example.toml` is generated from.
pub fn write_example(w: &mut dyn Write) -> fmt::Result {
    let cfg = RadioConfig::default();
    write_wrapped(w, "# ", EXAMPLE_HEADER, EXAMPLE_WIDTH)?;
    for section in SECTIONS {
        w.write_str("\n")?;
        write!(w, "# -- {} ", section.title)?;
        for _ in 0..EXAMPLE_WIDTH.saturating_sub(6 + section.title.len()) {
            w.write_str("-")?;
        }
        w.write_str("\n")?;
        if !section.doc.is_empty() {
            write_wrapped(w, "# ", section.doc, EXAMPLE_WIDTH)?;
        }
        write!(w, "[{}]\n", section.name)?;
        for key in KEYS.iter().filter(|k| k.section == section.name) {
            w.write_str("\n")?;
            write_wrapped(w, "# ", key.doc, EXAMPLE_WIDTH)?;
            if key.commented {
                w.write_str("# ")?;
            }
            write!(w, "{} = ", key.name)?;
            key.show(&cfg, w)?;
            w.write_str("\n")?;
        }
    }
    Ok(())
}

/// Write `text` word-wrapped at `width`, every line starting with
/// `prefix`.
fn write_wrapped(w: &mut dyn Write, prefix: &str, text: &str, width: usize) -> fmt::Result {
    let mut col = 0usize;
    for word in text.split_whitespace() {
        if col == 0 {
            w.write_str(prefix)?;
            col = prefix.len();
        } else if col + 1 + word.len() > width {
            w.write_str("\n")?;
            w.write_str(prefix)?;
            col = prefix.len();
        } else {
            w.write_str(" ")?;
            col += 1;
        }
        w.write_str(word)?;
        col += word.len();
    }
    if col > 0 {
        w.write_str("\n")?;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// The parser
// ---------------------------------------------------------------------------

/// The integer `name` takes, checked against its row of the table.
fn int_of(name: &str, value: &str, lineno: u32) -> Result<i64, ConfigError> {
    let k = key(name).ok_or(ConfigError::Syntax(lineno))?;
    let v = parse_i64(value).ok_or(ConfigError::BadValue(lineno))?;
    let ok = match k.kind {
        Kind::Int { min, max } => (min..=max).contains(&v),
        Kind::IntOrZero { min, max } => v == 0 || (min..=max).contains(&v),
        Kind::IntChoice(list) => list.contains(&v),
        _ => return Err(ConfigError::BadValue(lineno)),
    };
    if !ok {
        return Err(ConfigError::OutOfRange(lineno));
    }
    Ok(v)
}

/// Which of `name`'s choices `value` is.
fn choice_of(name: &str, value: &str, lineno: u32) -> Result<usize, ConfigError> {
    let k = key(name).ok_or(ConfigError::Syntax(lineno))?;
    let Kind::Choice(names) = k.kind else {
        return Err(ConfigError::BadValue(lineno));
    };
    let value = unquote(value);
    names
        .iter()
        .position(|n| *n == value)
        .ok_or(ConfigError::BadValue(lineno))
}

fn bool_of(value: &str, lineno: u32) -> Result<bool, ConfigError> {
    parse_bool(value).ok_or(ConfigError::BadValue(lineno))
}

/// Parse TOML text into a [`RadioConfig`], starting from the defaults so a
/// partial file is fine. Unknown keys are ignored (forward compatibility).
pub fn parse(text: &str) -> Result<RadioConfig, ConfigError> {
    let mut cfg = RadioConfig::default();
    // The last line that shaped the hop plan, so a plan that does not fit
    // the radio's range is reported against something the author wrote;
    // likewise the lines that set the dedup window and the interval it
    // has to fit.
    let mut hop_line = 0u32;
    let mut ttl_line = 0u32;
    let mut interval_line = 0u32;
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

        let name = canonical(key);
        match name {
            "frequency_hz" => {
                cfg.frequency_hz = int_of(name, value, lineno)? as u32;
                hop_line = lineno;
            }
            // 0 was "no plan" once; the schedule is always on now, and a
            // plan with nowhere to hop to is the one-channel plan.
            "hop_channels" => {
                cfg.hop_channels = (int_of(name, value, lineno)? as u8).max(1);
                hop_line = lineno;
            }
            "hop_step_khz" => {
                cfg.hop_step_khz = int_of(name, value, lineno)? as u16;
                hop_line = lineno;
            }
            "hop_dwell_ms" => cfg.hop_dwell_ms = int_of(name, value, lineno)? as u16,
            "spreading_factor" => cfg.spreading_factor = int_of(name, value, lineno)? as u8,
            "bandwidth_khz" => cfg.bandwidth_khz = int_of(name, value, lineno)? as u16,
            "coding_rate" => cfg.coding_rate = int_of(name, value, lineno)? as u8,
            "power_dbm" => cfg.power_dbm = int_of(name, value, lineno)? as i8,
            "rx_boost" => cfg.rx_boost = bool_of(value, lineno)?,
            "address" => cfg.address = int_of(name, value, lineno)? as u8,
            "role" => cfg.role = ROLES[choice_of(name, value, lineno)?],
            "max_hops" => cfg.max_hops = int_of(name, value, lineno)? as u8,
            "dedup_ttl_s" => {
                cfg.dedup_ttl_s = int_of(name, value, lineno)? as u16;
                ttl_line = lineno;
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
            "interval_s" => {
                cfg.beacon_interval_s = int_of(name, value, lineno)? as u16;
                interval_line = lineno;
            }
            "ping_interval_s" => cfg.ping_interval_s = int_of(name, value, lineno)? as u16,
            "fields" => {
                cfg.beacon_fields = parse_fields(value).ok_or(ConfigError::BadValue(lineno))?;
                // Position is the whole point of the broadcast, and a
                // receiver has nothing to plot without it.
                if cfg.beacon_fields & lora::FIELDS_REQUIRED != lora::FIELDS_REQUIRED {
                    return Err(ConfigError::OutOfRange(lineno));
                }
            }
            "dcdc_enabled" => cfg.dcdc_enabled = bool_of(value, lineno)?,
            "dio2_rf_switch" => cfg.dio2_rf_switch = bool_of(value, lineno)?,
            "tcxo_volts" => cfg.tcxo_volts = TCXO[choice_of(name, value, lineno)?],
            "tcxo_startup_ms" => cfg.tcxo_startup_ms = int_of(name, value, lineno)? as u16,
            "wake_enabled" => cfg.wake_enabled = bool_of(value, lineno)?,
            "wake_rx_ms" => cfg.wake_rx_ms = int_of(name, value, lineno)? as u16,
            "wake_sleep_ms" => cfg.wake_sleep_ms = int_of(name, value, lineno)? as u16,
            "wake_frequency_hz" => cfg.wake_frequency_hz = int_of(name, value, lineno)? as u32,
            // The card is gone; a file that still carries its key is not
            // wrong, only out of date, and the value is what it always
            // would have been.
            "sd_enabled" => {
                bool_of(value, lineno)?;
            }
            "verbose" => cfg.verbose = bool_of(value, lineno)?,
            "gps_enabled" => cfg.gps.gps_enabled = bool_of(value, lineno)?,
            "glonass_enabled" => cfg.gps.glonass_enabled = bool_of(value, lineno)?,
            "galileo_enabled" => cfg.gps.galileo_enabled = bool_of(value, lineno)?,
            "beidou_enabled" => cfg.gps.beidou_enabled = bool_of(value, lineno)?,
            "qzss_enabled" => cfg.gps.qzss_enabled = bool_of(value, lineno)?,
            "sbas_enabled" => cfg.gps.sbas_enabled = bool_of(value, lineno)?,
            "power_mode" => {
                // The long spellings are accepted for the cards that carry
                // them; the table names the short ones.
                let value = match unquote(value) {
                    "psm_onoff" => "psmoo",
                    "psm_cyclic" => "psmct",
                    v => v,
                };
                cfg.gps.power_mode = POWER_MODES[choice_of(name, value, lineno)?];
            }
            "meas_rate_ms" => cfg.gps.meas_rate_ms = int_of(name, value, lineno)? as u16,
            "dynamic_model" => cfg.gps.dyn_model = DYN_MODELS[choice_of(name, value, lineno)?],
            other => {
                // The `[power]` knobs, each stored as asked - zero
                // included - because what the board does with them turns
                // on whether the key was written at all. See
                // [`PowerConfig`].
                if let Some(knob) = Knob::from_name(other) {
                    let v = int_of(other, value, lineno)?;
                    cfg.power.set(knob, v as u32);
                }
                // Anything else is a key this build does not know: ignored.
            }
        }
    }
    // The plan as a whole: every channel has to be somewhere the radio can
    // tune. Checked after the loop because it depends on three keys that
    // can arrive in any order.
    let plan = crate::hop::Plan::from_config(&cfg);
    let (lo, hi) = plan.span_hz();
    if u64::from(lo) < RF_MIN_HZ || u64::from(hi) > RF_MAX_HZ || lo > hi {
        return Err(ConfigError::OutOfRange(hop_line));
    }
    // The dedup window against the id it keys on: eight bits, one per
    // transmission, so past 256 beacons a node starts dropping its own
    // later frames as duplicates. Refused with margin.
    if cfg.beacon_interval_s > 0
        && u32::from(cfg.dedup_ttl_s) > 200 * u32::from(cfg.beacon_interval_s)
    {
        return Err(ConfigError::OutOfRange(if ttl_line > 0 { ttl_line } else { interval_line }));
    }
    Ok(cfg)
}

/// The carrier range the SX126x covers, Hz: the bounds on `frequency_hz`
/// and on every channel of a hop plan.
pub const RF_MIN_HZ: u64 = 150_000_000;
pub const RF_MAX_HZ: u64 = 960_000_000;

/// Parse from bytes, reporting non-UTF-8 as an error.
pub fn parse_bytes(bytes: &[u8]) -> Result<RadioConfig, ConfigError> {
    parse(core::str::from_utf8(bytes).map_err(|_| ConfigError::Utf8)?)
}

fn parse_u64(s: &str) -> Option<u64> {
    let s = unquote(s);
    let mut v: u64 = 0;
    let mut any = false;
    for c in s.chars() {
        match c {
            '0'..='9' => {
                v = v.checked_mul(10)?.checked_add((c as u8 - b'0') as u64)?;
                any = true;
            }
            '_' => {}
            _ => return None,
        }
    }
    any.then_some(v)
}

fn parse_i64(s: &str) -> Option<i64> {
    let s = unquote(s);
    let (neg, digits) = match s.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, s.strip_prefix('+').unwrap_or(s)),
    };
    let v = parse_u64(digits)?;
    if v > i64::MAX as u64 {
        return None;
    }
    Some(if neg { -(v as i64) } else { v as i64 })
}

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
    /// So the size check belongs at the reader - the flash record refuses a
    /// text it cannot hold whole - and cannot be delegated to a parse failure.
    #[test]
    fn a_truncated_file_parses_as_a_shorter_one() {
        let example = include_str!("../../RADIO.example.toml");
        // The head of the file up to its first key: comments and a section
        // header, which is the shape a 1024-byte cut of it takes.
        let first_key = example.find("\nfrequency_hz").expect("the example starts at the radio");
        let head = &example.as_bytes()[..first_key + 1];
        assert!(head.len() >= 1024, "the header is shorter than a config read");
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

    /// The default is one channel with the slot clock running, and every
    /// key of the plan is bounded. One channel is a plan, not the absence
    /// of one: it still cuts slots into turns. Zero is the absence of one.
    #[test]
    fn the_default_is_one_channel_with_a_clock() {
        let cfg = RadioConfig::default();
        assert_eq!((cfg.hop_channels, cfg.hop_step_khz, cfg.hop_dwell_ms), (1, 500, 1000));
        let plan = crate::hop::Plan::from_config(&cfg);
        // Nowhere to hop to: every slot is frequency_hz itself.
        assert_eq!(plan.span_hz(), (cfg.frequency_hz, cfg.frequency_hz));
        for slot in 0..8 {
            assert_eq!(plan.frequency_for_slot(slot), cfg.frequency_hz);
        }
        // But the turns are there, which is the point of keeping it.
        assert!(plan.sub_slots >= 2);

        // 0 was "no plan" once; the schedule is always on, so it reads as
        // the one-channel plan it is.
        assert_eq!(parse("hop_channels = 0").unwrap().hop_channels, 1);
        assert_eq!(parse("hop_channels = 0").unwrap(), RadioConfig::default());
        assert_eq!(parse("hop_channels = 50").unwrap().hop_channels, 50);

        assert_eq!(parse("hop_channels = 64").unwrap().hop_channels, 64);
        assert_eq!(parse("hop_channels = 256"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("hop_channels = many"), Err(ConfigError::BadValue(1)));
        assert_eq!(parse("hop_step_khz = 200").unwrap().hop_step_khz, 200);
        assert_eq!(parse("hop_step_khz = 24"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("hop_step_khz = 5001"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("hop_dwell_ms = 400").unwrap().hop_dwell_ms, 400);
        assert_eq!(parse("hop_dwell_ms = 99"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("hop_dwell_ms = 10001"), Err(ConfigError::OutOfRange(1)));
    }

    /// A plan has to fit the radio: fifty 5 MHz channels around 915 MHz
    /// would put channels below 150 MHz and above 960. The error names the
    /// last of the keys that shaped the plan, whatever order they came in.
    #[test]
    fn a_hop_plan_must_fit_the_radio() {
        assert_eq!(
            parse("hop_step_khz = 5000\nhop_channels = 200"),
            Err(ConfigError::OutOfRange(2))
        );
        assert_eq!(
            parse("hop_channels = 200\nhop_step_khz = 5000"),
            Err(ConfigError::OutOfRange(2))
        );
        // A plan at the band edge fits only while every carrier is in range:
        // 41 channels about 160 MHz put the lowest exactly on 150 MHz, and
        // one more puts it below.
        assert!(parse("frequency_hz = 160_000_000\nhop_channels = 41").is_ok());
        assert_eq!(
            parse("frequency_hz = 160_000_000\nhop_channels = 42"),
            Err(ConfigError::OutOfRange(2))
        );
        // One channel has no span to fail with, whatever the step.
        assert!(parse("hop_channels = 1\nhop_step_khz = 5000").is_ok());
    }

    /// The sync word is four bytes on every frame - every node is on the
    /// schedule - so the frame overhead is the header and the word, on a
    /// one-channel plan as much as on fifty.
    #[test]
    fn every_frame_carries_the_sync_word() {
        let on = RadioConfig::default();
        let wide = RadioConfig { hop_channels: 50, ..on };
        assert_eq!(on.frame_overhead(), lora::HEADER_SYNC_LEN);
        assert_eq!(wide.frame_overhead(), lora::HEADER_SYNC_LEN);
        assert_eq!(on.beacon_airtime_us(), wide.beacon_airtime_us());
        assert!(on.ping_airtime_us() < on.beacon_airtime_us());
        // At SF12/BW500 the four bytes ride in the same symbol block, so the
        // default beacon stays under 300 ms and well inside a 1 s slot.
        assert!(on.beacon_airtime_us() < 300_000, "{} us", on.beacon_airtime_us());
        let plan = crate::hop::Plan::from_config(&on);
        assert!(plan.fits(on.beacon_airtime_us().div_ceil(1000)));
    }

    /// A position every second and a ping every five: the defaults, each
    /// on its own key with the same bounds as the beacon interval.
    #[test]
    fn ping_has_its_own_slower_interval() {
        let cfg = RadioConfig::default();
        assert_eq!(cfg.beacon_interval_s, 1);
        assert_eq!(cfg.ping_interval_s, 5);
        assert_eq!(parse("ping_interval_s = 30").unwrap().ping_interval_s, 30);
        assert_eq!(parse("ping_interval_s = 0").unwrap().ping_interval_s, 0);
        assert_eq!(parse("ping_interval_s = 3601"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("ping_interval_s = soon"), Err(ConfigError::BadValue(1)));
        // The beacon key alone does not move the ping.
        assert_eq!(parse("interval_s = 20").unwrap().ping_interval_s, 5);
        // At a 1 s interval the id wraps in about four minutes, and the
        // dedup window still sits well inside that.
        assert!(u32::from(cfg.dedup_ttl_s) * 10 < 256 * u32::from(cfg.beacon_interval_s));
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
        // 13-byte PHY payload: the 46.3 ms figure the docs quote. Every
        // frame carries the sync word as well, which makes the beacon 17
        // bytes and one more symbol block, 5.1 ms at this modulation.
        let mut cfg = RadioConfig {
            spreading_factor: 7,
            bandwidth_khz: 125,
            ..RadioConfig::default()
        };
        assert_eq!(cfg.time_on_air_us(13), 46_336);
        assert_eq!(cfg.time_on_air_us(17), 51_456);
        assert_eq!(cfg.beacon_airtime_us(), 51_456);

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

        // The same ping with the sync word (11 bytes) rides in the same
        // symbol block, so hopping costs it nothing at this modulation.
        assert_eq!(RadioConfig::default().ping_airtime_us(), 247_808);

        // Slowest modulation the parser accepts, largest frame the firmware
        // sends: SF12 / BW62.5 / CR 4/8 with LDRO on, 39 bytes (header,
        // sync word and a full payload) -> ~5.5 s.
        cfg.spreading_factor = 12;
        cfg.bandwidth_khz = 62;
        cfg.coding_rate = 8;
        assert!(cfg.ldro());
        assert_eq!(cfg.time_on_air_us(crate::lora::FRAME_MAX), 5_521_408);

        // Same frame at BW125 is exactly half the symbol time, so ~2.8 s.
        cfg.bandwidth_khz = 125;
        assert_eq!(cfg.time_on_air_us(crate::lora::FRAME_MAX), 2_760_704);
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

    /// A file written when the firmware still drove a card parses, and
    /// the key changes nothing.
    #[test]
    fn the_retired_sd_key_is_accepted_and_ignored() {
        assert_eq!(parse("sd_enabled = false").unwrap(), RadioConfig::default());
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
            hop_channels: 64,
            hop_step_khz: 200,
            hop_dwell_ms: 400,
            address: 200,
            role: Role::Repeater,
            max_hops: 3,
            dedup_ttl_s: 120,
            beacon_interval_s: 30,
            ping_interval_s: 7,
            beacon_fields: crate::lora::FIELD_LAT | crate::lora::FIELD_LON | crate::lora::FIELD_ALT,
            verbose: false,
            dcdc_enabled: false,
            dio2_rf_switch: true,
            tcxo_volts: TcxoVolts::V3_3,
            tcxo_startup_ms: 250,
            wake_enabled: false,
            wake_rx_ms: 450,
            wake_sleep_ms: 7_500,
            wake_frequency_hz: 927_000_000,
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
        assert_eq!(RadioConfig::decode(&good[..RADIO_CONFIG_LEN_V1 - 1]), None);
        let mut bad = good;
        bad[0] = RADIO_CONFIG_VERSION + 1;
        assert_eq!(RadioConfig::decode(&bad), None);
    }

    /// A blob from a board that predates the hop plan is the first 28 bytes
    /// of this one with a zero where `hop_channels` now sits. It decodes,
    /// and it decodes as a board that does not hop - which is the truth
    /// about that board. A blob cut anywhere inside the hop fields reads
    /// the same way rather than half a plan.
    #[test]
    fn radio_config_blob_from_before_hopping_reads_as_one_channel() {
        let good = RadioConfig { beacon_interval_s: 20, ..RadioConfig::default() }.encode();
        let old = RadioConfig::decode(&good[..RADIO_CONFIG_LEN_V1]).unwrap();
        assert_eq!(old.hop_channels, 1);
        assert_eq!(old.hop_step_khz, RadioConfig::default().hop_step_khz);
        assert_eq!(old.hop_dwell_ms, RadioConfig::default().hop_dwell_ms);
        // Such a board pinged on its beacon interval, so that is what it
        // reads back as pinging on.
        assert_eq!(old.ping_interval_s, 20);
        // And it had no sentry: the flag byte's bit was never set, so it
        // reads as off rather than as the default that came later.
        assert!(!old.wake_enabled);
        assert_eq!(old.wake_rx_ms, RadioConfig::default().wake_rx_ms);
        assert_eq!(
            RadioConfig { ping_interval_s: 5, wake_enabled: true, ..old },
            RadioConfig { beacon_interval_s: 20, ..RadioConfig::default() }
        );
        // A blob with the plan but not the ping interval: hopping as sent,
        // ping on the beacon interval.
        let hop_only = RadioConfig::decode(&good[..RADIO_CONFIG_LEN_HOP]).unwrap();
        assert_eq!(hop_only.hop_channels, 1);
        assert_eq!(hop_only.ping_interval_s, 20);
        // Cut inside a field, the field is absent rather than half-read.
        let cut = RadioConfig::decode(&good[..RADIO_CONFIG_LEN - 1]).unwrap();
        assert_eq!(cut.wake_sleep_ms, RadioConfig::default().wake_sleep_ms);
        assert!(!cut.wake_enabled);
        assert_eq!(RadioConfig::decode(&good[..RADIO_CONFIG_LEN_PING - 1]).unwrap().ping_interval_s, 20);
        assert_eq!(RadioConfig::decode(&good[..RADIO_CONFIG_LEN_HOP - 1]).unwrap().hop_channels, 1);
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
        const { assert!(RADIO_CONFIG_LEN <= crate::link::MAX_PAYLOAD) };
        // Conservative ATT_MTU-3 floor.
        const { assert!(RADIO_CONFIG_LEN <= 244) };
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
        assert!(cfg.power.is_empty());
        assert_eq!(cfg.power.get(Knob::BleOff), None);
    }

    /// A zero is a value, not an absence: writing `ble_off_s = 0` is how a
    /// file says "keep BLE up", and it has to override a board that was left
    /// duty-cycling.
    #[test]
    fn an_explicit_zero_is_still_a_request() {
        let cfg = parse("ble_off_s = 0\nsleep_interval_s = 0\nadv_window_s = 0").unwrap();
        assert_eq!(cfg.power.get(Knob::BleOff), Some(0));
        assert_eq!(cfg.power.get(Knob::SleepInterval), Some(0));
        assert_eq!(cfg.power.get(Knob::AdvWindow), Some(0));
        assert_eq!(cfg.power.get(Knob::BleOn), None);
    }

    #[test]
    fn power_values_parse_and_range_check() {
        let cfg = parse(
            "[power]\nble_off_s = 30\nadv_window_s = 10\nsleep_interval_s = 120\nidle_timeout_s = 900\nble_on_s = 20",
        )
        .unwrap();
        assert_eq!(cfg.power.get(Knob::BleOff), Some(30));
        assert_eq!(cfg.power.get(Knob::AdvWindow), Some(10));
        assert_eq!(cfg.power.get(Knob::SleepInterval), Some(120));
        assert_eq!(cfg.power.get(Knob::IdleTimeout), Some(900));
        assert_eq!(cfg.power.get(Knob::BleOn), Some(20));

        // Below the floor but not zero, and above the ceiling, on each key.
        for bad in [
            "ble_off_s = 1",
            "ble_off_s = 301",
            "adv_window_s = 61",
            "sleep_interval_s = 1",
            "sleep_interval_s = 3601",
            "idle_timeout_s = 9",
            "idle_timeout_s = 3601",
            "ble_on_s = 61",
        ] {
            assert_eq!(
                parse(bad),
                Err(ConfigError::OutOfRange(1)),
                "{bad} should have been rejected"
            );
        }
    }

    // -- the key table -----------------------------------------------------

    /// The shipped example is this crate's output, so the file, the
    /// parser's ranges and the app's field help cannot drift apart.
    /// Regenerate with `cargo run --example radio_example >
    /// ../RADIO.example.toml`.
    #[test]
    fn the_shipped_example_is_generated() {
        let mut out = String::new();
        write_example(&mut out).unwrap();
        let shipped = include_str!("../../RADIO.example.toml");
        assert!(
            shipped == out,
            "RADIO.example.toml is out of date - regenerate it with cargo run --example radio_example"
        );
        // No line runs past the wrap, and every key's doc is there.
        for line in out.lines() {
            assert!(line.len() <= 80, "{line}");
        }
        for k in KEYS {
            assert!(out.contains(&format!("{} = ", k.name)), "{} missing", k.name);
        }
    }

    /// Every key in the table is one the parser accepts at its own default,
    /// commented or not, and every section it names exists.
    #[test]
    fn every_key_in_the_table_parses_at_its_default() {
        let cfg = RadioConfig::default();
        for k in KEYS {
            assert!(SECTIONS.iter().any(|s| s.name == k.section), "{} in no section", k.name);
            assert_eq!(key(k.name).map(|x| x.name), Some(k.name));
            let mut line = String::new();
            write!(line, "{} = ", k.name).unwrap();
            k.show(&cfg, &mut line).unwrap();
            let parsed = parse(&line).unwrap_or_else(|e| panic!("{line}: {e:?}"));
            // A power key at 0 is a request, so the config differs by
            // exactly that; everything else is the default it printed.
            match Knob::from_name(k.name) {
                Some(knob) => assert_eq!(parsed.power.get(knob), Some(0)),
                None => assert_eq!(parsed, cfg, "{line}"),
            }
            // And a value past the bounds is refused with a line number.
            match k.kind {
                Kind::Int { max, .. } | Kind::IntOrZero { max, .. } => {
                    let over = format!("{} = {}", k.name, max + 1);
                    assert_eq!(parse(&over), Err(ConfigError::OutOfRange(1)), "{over}");
                }
                Kind::IntChoice(_) => {
                    assert_eq!(parse(&format!("{} = 1", k.name)), Err(ConfigError::OutOfRange(1)));
                }
                Kind::Choice(_) => {
                    let bad = format!("{} = \"nonsense\"", k.name);
                    assert_eq!(parse(&bad), Err(ConfigError::BadValue(1)), "{bad}");
                }
                Kind::Bool => {
                    assert_eq!(parse(&format!("{} = maybe", k.name)), Err(ConfigError::BadValue(1)));
                }
                Kind::Fields => {}
            }
        }
        // The aliases resolve to table keys.
        for alias in ["beacon_interval_s", "beacon_fields", "dyn_model"] {
            assert!(key(alias).is_some(), "{alias}");
        }
        assert!(key("no_such_key").is_none());
    }

    /// The dedup window has to sit inside the beacon id's wrap, and the
    /// file is refused otherwise - against whichever line set it up.
    #[test]
    fn the_dedup_window_must_fit_the_id_wrap() {
        assert!(parse("dedup_ttl_s = 200").is_ok());
        assert_eq!(parse("dedup_ttl_s = 201"), Err(ConfigError::OutOfRange(1)));
        assert_eq!(parse("interval_s = 1\ndedup_ttl_s = 201"), Err(ConfigError::OutOfRange(2)));
        assert!(parse("interval_s = 2\ndedup_ttl_s = 400").is_ok());
        // A silenced node has no ids to wrap.
        assert!(parse("interval_s = 0\ndedup_ttl_s = 3600").is_ok());
        // The interval line takes the blame when the window was left at
        // its default and the interval moved under it.
        assert!(parse("dedup_ttl_s = 3").is_ok());
    }
}
