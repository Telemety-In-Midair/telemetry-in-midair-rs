//! Frequency hopping: the channel plan, the slot clock the network keeps in
//! step, and the sync word every hopped frame carries so a receiver can
//! join that clock from nothing but the frames it hears.
//!
//! Time is cut into slots of [`Plan::dwell_ms`]. In slot `s` every node
//! listens on, and transmits on, one channel: the `s mod n`-th entry of a
//! permutation of all `n` channels, reshuffled every cycle of `n` slots
//! from the cycle number. Each channel is therefore used exactly once per
//! cycle by a node that transmits every slot, and a node that transmits
//! every k-th slot still lands on a different channel each time, since the
//! order changes underneath it. That is what the band rules ask of a
//! hopping transmitter, and it is also what lets two nodes that do not
//! agree on the time find each other: their channels coincide in about one
//! slot in `n`, where a fixed order with a fixed offset would never meet.
//!
//! The clock itself is nothing but a local millisecond counter and the two
//! numbers that map it onto slots - when a slot began, and which slot it
//! was. Three things set those numbers, in order of trust:
//!
//! 1. **GPS time.** A node with a fix takes its slot from the time of day,
//!    so every node with a fix agrees without hearing anyone. Stratum 0.
//! 2. **A heard frame.** Every hopped frame carries the slot and the phase
//!    its sender transmitted at, and the sender's stratum. A receiver that
//!    hears a better clock than its own adopts it and sits one stratum
//!    below. A frame's length and modulation fix its time on air, so the
//!    receiver knows to the millisecond when the sender's slot began.
//! 3. **Nothing.** A node that has neither free-runs from boot at
//!    [`STRATUM_MAX`], hopping on its own clock so a follower can still be
//!    in step with it, and adopts the first better clock it hears.
//!
//! A clock nobody has refreshed ages one stratum every [`AGE_STEP_MS`], so
//! a network cut off from its GPS reference reorganizes on its own: two
//! nodes at the same stratum settle on the lower address as the reference,
//! and a node that gets a fix back outranks everyone again at once.
//!
//! Everything here is integer arithmetic on caller-supplied millisecond
//! timestamps, so it is `no_std` and tests on the host.

use crate::radiocfg::RadioConfig;

/// Stratum of a clock disciplined by the node's own GPS receiver.
pub const STRATUM_GPS: u8 = 0;
/// Stratum of a clock nothing has disciplined: the node is hopping on its
/// own free-running counter. Also the ceiling a follower's stratum ages to.
pub const STRATUM_MAX: u8 = 15;

/// How long a clock goes without a fresh reference before it counts as one
/// stratum worse. Crystals on two boards drift apart at tens of parts per
/// million, a few milliseconds per ten minutes, so a clock this old is still
/// good - the point of aging is the ordering, not the accuracy: it lets a
/// network that lost its reference pick a new one instead of every node
/// insisting it is still stratum 1.
pub const AGE_STEP_MS: u64 = 10 * 60 * 1_000;

/// Width of the slot number as it travels in a frame. 2^20 slots is twelve
/// days at the default dwell; a GPS-derived slot (seconds into the day) is
/// well inside it.
pub const SLOT_BITS: u32 = 20;
/// Mask for a slot number: every slot value in this module is reduced by it.
pub const SLOT_MASK: u32 = (1 << SLOT_BITS) - 1;

/// Upper bound on the guard either side of a slot, in milliseconds. A slot
/// of the default dwell gives up this much at each end to clock error
/// between nodes; a shorter slot gives up a fifth of itself instead.
pub const GUARD_MAX_MS: u32 = 100;

/// The channel plan: which frequencies, and how long each slot lasts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    /// Channels in the plan, at least 1.
    pub channels: u8,
    /// Spacing between adjacent channels, kHz.
    pub step_khz: u16,
    /// Center of the plan, Hz. The channels straddle it symmetrically.
    pub center_hz: u32,
    /// Slot length, ms.
    pub dwell_ms: u16,
}

impl Plan {
    /// The plan `cfg` describes, or `None` when hopping is off.
    pub fn from_config(cfg: &RadioConfig) -> Option<Self> {
        (cfg.hop_channels > 0).then_some(Self {
            channels: cfg.hop_channels,
            step_khz: cfg.hop_step_khz,
            center_hz: cfg.frequency_hz,
            dwell_ms: cfg.hop_dwell_ms,
        })
    }

    /// Carrier of channel `index`, Hz. Channels are spread evenly about the
    /// center, so with an even count the center itself is between two of
    /// them. Indices past the plan are clamped to its last channel.
    pub fn channel_hz(&self, index: u8) -> u32 {
        let n = i64::from(self.channels.max(1));
        let i = i64::from(index).min(n - 1);
        // Half a step, so an even count stays symmetric without fractions:
        // step_khz * 1000 / 2 is a whole number of hertz for any step.
        let half_step = i64::from(self.step_khz) * 500;
        let off = (2 * i - (n - 1)) * half_step;
        (i64::from(self.center_hz) + off).clamp(0, i64::from(u32::MAX)) as u32
    }

    /// The lowest and highest carriers in the plan, Hz.
    pub fn span_hz(&self) -> (u32, u32) {
        (self.channel_hz(0), self.channel_hz(self.channels.saturating_sub(1)))
    }

    /// Slots in one cycle, i.e. the channel count.
    pub fn cycle(&self) -> u32 {
        u32::from(self.channels.max(1))
    }

    /// Which channel slot `slot` uses: the `slot mod n`-th entry of the
    /// permutation drawn for cycle `slot div n`.
    pub fn index_for_slot(&self, slot: u32) -> u8 {
        let n = self.cycle();
        let slot = slot & SLOT_MASK;
        let mut perm = [0u8; 256];
        permutation(slot / n, self.channels.max(1), &mut perm);
        perm[(slot % n) as usize]
    }

    /// Carrier for slot `slot`, Hz.
    pub fn frequency_for_slot(&self, slot: u32) -> u32 {
        self.channel_hz(self.index_for_slot(slot))
    }

    /// Time given up at each end of a slot to disagreement between clocks,
    /// ms: a transmission neither starts before it nor is planned to run
    /// into it.
    pub fn guard_ms(&self) -> u32 {
        (u32::from(self.dwell_ms) / 5).min(GUARD_MAX_MS)
    }

    /// The longest transmission that fits a slot with its guards, ms.
    pub fn window_ms(&self) -> u32 {
        u32::from(self.dwell_ms).saturating_sub(2 * self.guard_ms())
    }

    /// Whether a transmission of `airtime_ms` ends inside the slot it
    /// starts in. One that does not is still sent - a receiver holds its
    /// hop while a frame is arriving - but it is the sender's occupancy of
    /// one channel, which is what the band rules cap.
    pub fn fits(&self, airtime_ms: u32) -> bool {
        airtime_ms <= self.window_ms()
    }

    /// The range of slot phases a transmission of `airtime_ms` may start at,
    /// `(earliest, latest)` in ms from the slot's start. A frame too long for
    /// the window collapses the range to the guard, so it at least starts as
    /// early as a receiver can be trusted to be listening.
    pub fn start_range_ms(&self, airtime_ms: u32) -> (u32, u32) {
        let guard = self.guard_ms();
        let latest = u32::from(self.dwell_ms)
            .saturating_sub(guard + airtime_ms)
            .max(guard);
        (guard, latest)
    }
}

/// Fill `out[..n]` with the channel order for `cycle`: a Fisher-Yates
/// shuffle of `0..n` driven by a generator seeded from the cycle number.
/// Deterministic, so every node that agrees on the cycle agrees on the
/// order, and a fresh order each cycle.
pub fn permutation(cycle: u32, n: u8, out: &mut [u8; 256]) {
    let n = n.max(1) as usize;
    for (i, slot) in out.iter_mut().enumerate().take(n) {
        *slot = i as u8;
    }
    let mut s = mix(cycle);
    for i in (1..n).rev() {
        s = xorshift(s);
        let j = (s % (i as u32 + 1)) as usize;
        out.swap(i, j);
    }
}

/// Spread a cycle number into a generator seed, so neighboring cycles give
/// unrelated orders. The finalizer from MurmurHash3, forced odd because a
/// xorshift state of zero never leaves zero.
fn mix(x: u32) -> u32 {
    let mut h = x ^ 0x5bd1_e995;
    h ^= h >> 16;
    h = h.wrapping_mul(0x85eb_ca6b);
    h ^= h >> 13;
    h = h.wrapping_mul(0xc2b2_ae35);
    h ^= h >> 16;
    h | 1
}

fn xorshift(mut s: u32) -> u32 {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    s
}

/// What a hopped frame says about its sender's clock, packed into four
/// bytes: the slot the transmission started in, how far into that slot it
/// started, and how good the sender thinks its clock is.
///
/// Layout, as a little-endian `u32`: bits 0-7 phase, bits 8-11 stratum,
/// bits 12-31 slot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SyncWord {
    /// Slot number, [`SLOT_BITS`] wide.
    pub slot: u32,
    /// Sender's stratum, 0-15.
    pub stratum: u8,
    /// Start of the transmission within the slot, in 1/256ths of the dwell.
    pub phase: u8,
}

/// Wire size of a [`SyncWord`].
pub const SYNC_LEN: usize = 4;

impl SyncWord {
    pub fn to_u32(self) -> u32 {
        ((self.slot & SLOT_MASK) << 12) | (u32::from(self.stratum & 0x0F) << 8) | u32::from(self.phase)
    }

    pub fn from_u32(v: u32) -> Self {
        Self {
            slot: v >> 12,
            stratum: ((v >> 8) & 0x0F) as u8,
            phase: (v & 0xFF) as u8,
        }
    }

    pub fn to_bytes(self) -> [u8; SYNC_LEN] {
        self.to_u32().to_le_bytes()
    }

    pub fn from_bytes(b: [u8; SYNC_LEN]) -> Self {
        Self::from_u32(u32::from_le_bytes(b))
    }
}

/// What [`Clock::offer`] decided about a heard frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Offer {
    /// The node's own clock is as good or better; nothing changed.
    Kept,
    /// The heard clock was adopted. Carries the stratum this node now has.
    Adopted(u8),
}

/// This node's slot clock: a mapping from local milliseconds to slot
/// numbers, and how much to trust it.
#[derive(Clone, Copy, Debug)]
pub struct Clock {
    dwell_ms: u32,
    /// Local time at which slot `origin_slot` began. Signed, because a
    /// slot that was already under way at boot began before local time
    /// zero.
    origin_ms: i64,
    origin_slot: u32,
    /// Stratum as of the last discipline; ages from there.
    base_stratum: u8,
    /// When the clock was last disciplined. `None` since boot.
    synced_ms: Option<u64>,
    rng: u32,
}

impl Clock {
    /// A free-running clock started at `now_ms`. `seed` picks the starting
    /// slot and the transmit jitter; give it something that differs
    /// between nodes (the address), so two nodes booted together do not
    /// start on the same count.
    pub fn new(dwell_ms: u16, now_ms: u64, seed: u32) -> Self {
        let s = mix(seed);
        Self {
            dwell_ms: u32::from(dwell_ms.max(1)),
            origin_ms: now_ms as i64,
            origin_slot: s & SLOT_MASK,
            base_stratum: STRATUM_MAX,
            synced_ms: None,
            rng: xorshift(s),
        }
    }

    /// Slot length, ms.
    pub fn dwell_ms(&self) -> u32 {
        self.dwell_ms
    }

    /// The slot `now_ms` falls in.
    pub fn slot(&self, now_ms: u64) -> u32 {
        let elapsed = (now_ms as i64 - self.origin_ms).div_euclid(i64::from(self.dwell_ms));
        (self.origin_slot.wrapping_add(elapsed as u32)) & SLOT_MASK
    }

    /// Milliseconds from the start of the current slot to `now_ms`.
    pub fn phase_ms(&self, now_ms: u64) -> u32 {
        (now_ms as i64 - self.origin_ms).rem_euclid(i64::from(self.dwell_ms)) as u32
    }

    /// Stratum as of `now_ms`, aging included.
    pub fn stratum(&self, now_ms: u64) -> u8 {
        match self.synced_ms {
            None => STRATUM_MAX,
            Some(t) => {
                let aged = u64::from(self.base_stratum) + now_ms.saturating_sub(t) / AGE_STEP_MS;
                aged.min(u64::from(STRATUM_MAX)) as u8
            }
        }
    }

    /// Whether something has disciplined this clock at all.
    pub fn synced(&self) -> bool {
        self.synced_ms.is_some()
    }

    /// Whether the clock is still on the node's own GPS time.
    pub fn from_gps(&self, now_ms: u64) -> bool {
        self.stratum(now_ms) == STRATUM_GPS
    }

    /// Take the slot from GPS time: `tod_ms` is the time of day the receiver
    /// reported, and `at_ms` the local time that report was parsed. Every
    /// node with a fix keeps the same output latency, so what matters is
    /// only that they all take it the same way.
    pub fn discipline_gps(&mut self, tod_ms: u32, at_ms: u64) {
        let slot = tod_ms / self.dwell_ms;
        let phase = tod_ms % self.dwell_ms;
        self.set_origin(at_ms, phase, slot);
        self.base_stratum = STRATUM_GPS;
        self.synced_ms = Some(at_ms);
    }

    /// Consider a heard frame's clock. `word` is what the frame carried,
    /// `src` its sender, `tx_start_ms` the local time the transmission
    /// began (the receive instant less the frame's time on air), and
    /// `my_addr` this node's address, which settles a tie between two
    /// clocks of the same stratum in favor of the lower address.
    ///
    /// A clock on its own GPS never adopts anything: the receiver in the
    /// next box is not a better reference than the one on this board.
    pub fn offer(
        &mut self,
        word: SyncWord,
        src: u8,
        my_addr: u8,
        tx_start_ms: u64,
        now_ms: u64,
    ) -> Offer {
        let mine = self.stratum(now_ms);
        if mine == STRATUM_GPS {
            return Offer::Kept;
        }
        let theirs = word.stratum.min(STRATUM_MAX);
        if !(theirs < mine || (theirs == mine && src < my_addr)) {
            return Offer::Kept;
        }
        let phase = u32::from(word.phase) * self.dwell_ms / 256;
        self.set_origin(tx_start_ms, phase, word.slot);
        self.base_stratum = theirs.saturating_add(1).min(STRATUM_MAX);
        self.synced_ms = Some(now_ms);
        Offer::Adopted(self.base_stratum)
    }

    /// Re-anchor the mapping: at local time `at_ms` the clock was
    /// `phase_ms` into slot `slot`.
    fn set_origin(&mut self, at_ms: u64, phase_ms: u32, slot: u32) {
        self.origin_ms = at_ms as i64 - i64::from(phase_ms);
        self.origin_slot = slot & SLOT_MASK;
    }

    /// Slots an interval of `interval_ms` spans, at least one. A beacon
    /// interval on a hopping network is a count of slots, so "every
    /// second" at the default dwell means every slot rather than every
    /// slot and a bit, which would skip one every few.
    pub fn slots_for(&self, interval_ms: u32) -> u32 {
        interval_ms.div_ceil(self.dwell_ms).max(1)
    }

    /// Whether `now_ms` is in, or past, the slot `interval_ms` after the
    /// one `last_ms` fell in. Slot numbers wrap, so "past" is the nearer
    /// half of the ring.
    pub fn interval_elapsed(&self, last_ms: u64, interval_ms: u32, now_ms: u64) -> bool {
        let due = self.slot(last_ms).wrapping_add(self.slots_for(interval_ms)) & SLOT_MASK;
        let ahead = self.slot(now_ms).wrapping_sub(due) & SLOT_MASK;
        ahead < (1 << (SLOT_BITS - 1))
    }

    /// Local time the slot after the one `now_ms` is in begins.
    pub fn next_slot_start_ms(&self, now_ms: u64) -> u64 {
        now_ms + u64::from(self.dwell_ms - self.phase_ms(now_ms))
    }

    /// The sync word for a transmission starting at local time
    /// `tx_start_ms`, with the stratum as of the same instant.
    pub fn word_at(&self, tx_start_ms: u64) -> SyncWord {
        SyncWord {
            slot: self.slot(tx_start_ms),
            phase: (self.phase_ms(tx_start_ms) * 256 / self.dwell_ms).min(255) as u8,
            stratum: self.stratum(tx_start_ms),
        }
    }

    /// The local time a transmission of `airtime_ms` should start, at or
    /// after `now_ms`: a random point inside the slot's window, in this
    /// slot if that point is still ahead and otherwise in the next. The
    /// randomness is what keeps two nodes that beacon on the same interval
    /// from colliding every slot.
    pub fn tx_start(&mut self, plan: &Plan, now_ms: u64, airtime_ms: u32) -> u64 {
        let (lo, hi) = plan.start_range_ms(airtime_ms);
        self.rng = xorshift(self.rng);
        let target = lo + self.rng % (hi - lo + 1);
        let phase = self.phase_ms(now_ms);
        if phase <= target {
            now_ms + u64::from(target - phase)
        } else {
            now_ms + u64::from(self.dwell_ms - phase) + u64::from(target)
        }
    }

    /// How long a transmission of `airtime_ms` that wants to start at
    /// `now_ms` has to wait to be inside a window, or 0 if it already is.
    /// The check a sender makes at the last moment, in case its clock moved
    /// between planning the transmission and making it.
    pub fn wait_for_window_ms(&self, plan: &Plan, now_ms: u64, airtime_ms: u32) -> u32 {
        let (lo, hi) = plan.start_range_ms(airtime_ms);
        let phase = self.phase_ms(now_ms);
        if phase < lo {
            lo - phase
        } else if phase > hi {
            self.dwell_ms - phase + lo
        } else {
            0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan() -> Plan {
        Plan::from_config(&RadioConfig::default()).expect("hopping is the default")
    }

    /// The default plan fills the 902-928 MHz band with fifty 500 kHz
    /// channels about 915 MHz, every carrier a whole channel inside the
    /// band edges.
    #[test]
    fn default_plan_covers_the_us_band() {
        let p = plan();
        assert_eq!(p.channels, 50);
        assert_eq!(p.dwell_ms, 1000);
        let (lo, hi) = p.span_hz();
        assert_eq!(lo, 902_750_000);
        assert_eq!(hi, 927_250_000);
        // Symmetric about the center, and evenly spaced.
        assert_eq!(hi - 915_000_000, 915_000_000 - lo);
        for i in 1..p.channels {
            assert_eq!(p.channel_hz(i) - p.channel_hz(i - 1), 500_000);
        }
        // An index past the end is the last channel, never past the band.
        assert_eq!(p.channel_hz(200), hi);
    }

    /// An odd count puts a channel on the center itself.
    #[test]
    fn odd_count_has_a_channel_on_the_center() {
        let p = Plan { channels: 51, ..plan() };
        assert_eq!(p.channel_hz(25), 915_000_000);
    }

    /// Every cycle uses every channel exactly once, and consecutive cycles
    /// use them in different orders.
    #[test]
    fn each_cycle_is_a_fresh_permutation() {
        let p = plan();
        let n = p.cycle();
        let mut orders = alloc_orders(&p, 0..4);
        for order in &orders {
            let mut seen = [false; 256];
            for &c in order {
                assert!(!seen[c as usize], "channel {c} twice in a cycle");
                seen[c as usize] = true;
            }
            assert_eq!(order.len() as u32, n);
        }
        let first = orders.remove(0);
        assert!(orders.iter().all(|o| *o != first));
    }

    fn alloc_orders(p: &Plan, cycles: core::ops::Range<u32>) -> std::vec::Vec<std::vec::Vec<u8>> {
        cycles
            .map(|c| (0..p.cycle()).map(|i| p.index_for_slot(c * p.cycle() + i)).collect())
            .collect()
    }

    /// A node that transmits every k-th slot must still spread across the
    /// channels over time. With a fixed order and k sharing a factor with n
    /// it would cycle through a handful; reshuffling each cycle fixes that.
    #[test]
    fn a_periodic_sender_uses_every_channel() {
        let p = plan();
        let mut hit = [false; 256];
        for k in 0..2_000u32 {
            hit[p.index_for_slot(k * 20) as usize] = true;
        }
        assert!(hit[..p.channels as usize].iter().all(|&h| h));
    }

    /// The order is a function of the cycle alone, so two nodes that agree
    /// on the slot number agree on the channel.
    #[test]
    fn permutation_is_deterministic() {
        let mut a = [0u8; 256];
        let mut b = [0u8; 256];
        permutation(1234, 50, &mut a);
        permutation(1234, 50, &mut b);
        assert_eq!(a, b);
        permutation(1235, 50, &mut b);
        assert_ne!(a, b);
    }

    #[test]
    fn sync_word_roundtrip() {
        let w = SyncWord { slot: 0xABCDE, stratum: 9, phase: 200 };
        assert_eq!(SyncWord::from_bytes(w.to_bytes()), w);
        // The slot is masked to its width on the way in.
        let wide = SyncWord { slot: 0xFFF_FFFF, stratum: 3, phase: 0 };
        assert_eq!(SyncWord::from_u32(wide.to_u32()).slot, SLOT_MASK);
        assert_eq!(SyncWord::from_u32(0), SyncWord::default());
    }

    /// The guard is a fifth of a short slot and capped for a long one, and
    /// the default beacon fits the default slot with room to jitter.
    #[test]
    fn guard_and_window() {
        let p = plan();
        assert_eq!(p.guard_ms(), 100);
        assert_eq!(p.window_ms(), 800);
        assert!(p.fits(289));
        assert!(!p.fits(801));
        let short = Plan { dwell_ms: 200, ..p };
        assert_eq!(short.guard_ms(), 40);
        // The start range leaves the frame ending before the far guard...
        assert_eq!(p.start_range_ms(300), (100, 600));
        // ...and collapses to the near guard for a frame that cannot.
        assert_eq!(p.start_range_ms(900), (100, 100));
    }

    /// GPS time maps straight onto slots: seconds into the day at the
    /// default dwell, with the phase taken from the fraction.
    #[test]
    fn gps_disciplines_to_time_of_day() {
        let mut c = Clock::new(1000, 5_000, 7);
        assert_eq!(c.stratum(5_000), STRATUM_MAX);
        assert!(!c.synced());
        // 12:34:56.250, parsed at local 5000 ms.
        c.discipline_gps(45_296_250, 5_000);
        assert_eq!(c.slot(5_000), 45_296);
        assert_eq!(c.phase_ms(5_000), 250);
        assert_eq!(c.slot(5_750), 45_297);
        assert_eq!(c.stratum(5_000), STRATUM_GPS);
        assert!(c.from_gps(5_000));
        // A report parsed before the slot began, in local time, still lands.
        let mut early = Clock::new(1000, 0, 1);
        early.discipline_gps(900, 100);
        assert_eq!(early.slot(100), 0);
        assert_eq!(early.phase_ms(100), 900);
        assert_eq!(early.slot(200), 1);
    }

    /// A follower that hears a frame ends up on the sender's slot and phase
    /// to within the sync word's resolution, one stratum below.
    #[test]
    fn a_heard_frame_aligns_a_follower() {
        let mut a = Clock::new(1000, 0, 1);
        a.discipline_gps(1_000_000, 10_000);
        // A transmits 300 ms into some slot.
        let tx = 20_300;
        let word = a.word_at(tx);
        assert_eq!(word.stratum, 0);
        assert_eq!(word.slot, a.slot(tx));

        // B free-runs on an unrelated count until it hears A. Its own local
        // clock is offset from A's - the frame started at B's 77_300.
        let mut b = Clock::new(1000, 3, 2);
        let rx_start = 77_300;
        assert_eq!(b.offer(word, 1, 2, rx_start, rx_start + 289), Offer::Adopted(1));
        assert_eq!(b.slot(rx_start), a.slot(tx));
        let err = b.phase_ms(rx_start) as i32 - a.phase_ms(tx) as i32;
        assert!(err.abs() <= 1000 / 256 + 1, "phase error {err} ms");
        assert_eq!(b.stratum(rx_start + 289), 1);
        // And B's own word now says so.
        assert_eq!(b.word_at(rx_start + 500).stratum, 1);
    }

    /// Who adopts whom: a better stratum always, an equal one only from a
    /// lower address, and a GPS clock never.
    #[test]
    fn adoption_rules() {
        let word = |stratum| SyncWord { slot: 100, stratum, phase: 0 };
        let mut c = Clock::new(1000, 0, 9);
        // Free-running (15) adopts an equal 15 from a lower address...
        assert_eq!(c.offer(word(15), 3, 9, 1_000, 1_000), Offer::Adopted(15));
        // ...but not from a higher one.
        let mut d = Clock::new(1000, 0, 2);
        assert_eq!(d.offer(word(15), 3, 2, 1_000, 1_000), Offer::Kept);
        // A follower at 1 ignores a 1 from a higher address and a 2.
        let mut f = Clock::new(1000, 0, 5);
        assert_eq!(f.offer(word(0), 1, 5, 1_000, 1_000), Offer::Adopted(1));
        assert_eq!(f.offer(word(1), 7, 5, 2_000, 2_000), Offer::Kept);
        assert_eq!(f.offer(word(2), 1, 5, 2_000, 2_000), Offer::Kept);
        // ...and takes a 1 from a lower address, becoming 2.
        assert_eq!(f.offer(word(1), 4, 5, 3_000, 3_000), Offer::Adopted(2));
        // Own GPS outranks everything.
        let mut g = Clock::new(1000, 0, 200);
        g.discipline_gps(0, 1_000);
        assert_eq!(g.offer(word(0), 1, 200, 1_500, 1_500), Offer::Kept);
    }

    /// A clock nobody refreshes gets worse one stratum per step, so a lost
    /// reference is eventually replaceable - and a GPS clock that lost its
    /// fix stops outranking the network.
    #[test]
    fn stratum_ages_without_a_reference() {
        let mut c = Clock::new(1000, 0, 1);
        c.discipline_gps(0, 1_000);
        assert_eq!(c.stratum(1_000 + AGE_STEP_MS - 1), 0);
        assert_eq!(c.stratum(1_000 + AGE_STEP_MS), 1);
        assert_eq!(c.stratum(1_000 + 3 * AGE_STEP_MS), 3);
        assert_eq!(c.stratum(1_000 + 100 * AGE_STEP_MS), STRATUM_MAX);
        // Aged past 0, it will take a frame from a stratum-0 node again.
        let later = 1_000 + AGE_STEP_MS;
        let word = SyncWord { slot: 5, stratum: 0, phase: 0 };
        assert_eq!(c.offer(word, 2, 1, later, later), Offer::Adopted(1));
        // Aging does not move the mapping, only the trust in it.
        assert_eq!(c.slot(later), 5);
    }

    /// Planned transmissions land inside the window of a slot, spread
    /// across it, and never before the guard.
    #[test]
    fn tx_start_lands_in_the_window() {
        let p = plan();
        let mut c = Clock::new(1000, 0, 4);
        c.discipline_gps(0, 0);
        let mut phases = std::vec::Vec::new();
        for i in 0..200u64 {
            let now = 10_000 + i * 37;
            let start = c.tx_start(&p, now, 289);
            assert!(start >= now);
            let phase = c.phase_ms(start);
            assert!((100..=611).contains(&phase), "phase {phase}");
            phases.push(phase);
        }
        let min = *phases.iter().min().unwrap();
        let max = *phases.iter().max().unwrap();
        assert!(max - min > 300, "no spread: {min}..{max}");
        // Already inside the window, ready to go: no wait.
        assert_eq!(c.wait_for_window_ms(&p, 10_300, 289), 0);
        // Before the guard: wait for it.
        assert_eq!(c.wait_for_window_ms(&p, 10_020, 289), 80);
        // Too late for the frame to end in this slot: wait for the next.
        assert_eq!(c.wait_for_window_ms(&p, 10_700, 289), 400);
    }

    /// An interval is a count of slots: one second at the default dwell is
    /// every slot, five seconds every fifth, and the interval is up at the
    /// slot boundary rather than a phase later.
    #[test]
    fn intervals_are_counted_in_slots() {
        let mut c = Clock::new(1000, 0, 1);
        c.discipline_gps(0, 0);
        assert_eq!(c.slots_for(1000), 1);
        assert_eq!(c.slots_for(1001), 2);
        assert_eq!(c.slots_for(5000), 5);
        assert_eq!(c.slots_for(0), 1);
        // Last transmission 300 ms into slot 10.
        let last = 10_300;
        assert!(!c.interval_elapsed(last, 1000, 10_900));
        assert!(c.interval_elapsed(last, 1000, 11_000));
        assert!(c.interval_elapsed(last, 1000, 11_050));
        assert!(!c.interval_elapsed(last, 5000, 14_999));
        assert!(c.interval_elapsed(last, 5000, 15_000));
        // Long past is still elapsed, not wrapped around to "not yet".
        assert!(c.interval_elapsed(last, 1000, 400_000));
        assert_eq!(c.next_slot_start_ms(10_300), 11_000);
        assert_eq!(c.next_slot_start_ms(11_000), 12_000);
    }

    /// The channel a slot maps to is the same on every node that agrees
    /// on the slot, whatever their local clocks read.
    #[test]
    fn synced_nodes_share_a_channel() {
        let p = plan();
        let mut a = Clock::new(1000, 0, 1);
        a.discipline_gps(3_600_000, 500);
        let mut b = Clock::new(1000, 0, 2);
        b.discipline_gps(3_600_000, 90_500);
        for t in 0..50u64 {
            let sa = a.slot(500 + t * 1000);
            let sb = b.slot(90_500 + t * 1000);
            assert_eq!(sa, sb);
            assert_eq!(p.frequency_for_slot(sa), p.frequency_for_slot(sb));
        }
    }

    /// Two free-running clocks with an unrelated offset still coincide on
    /// a channel now and then - which is how they find each other at all.
    #[test]
    fn unsynced_nodes_coincide_sometimes() {
        let p = plan();
        let a = Clock::new(1000, 0, 11);
        let b = Clock::new(1000, 0, 12);
        let hits = (0..5_000u64)
            .filter(|&t| {
                let now = t * 1000 + 500;
                p.index_for_slot(a.slot(now)) == p.index_for_slot(b.slot(now))
            })
            .count();
        // About one slot in fifty; anything in a wide band around that.
        assert!((30..300).contains(&hits), "{hits} coincidences in 5000 slots");
    }
}
