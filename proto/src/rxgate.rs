//! What the receiver has seen of a frame that is arriving, and whether the
//! radio may be disturbed.
//!
//! The SX1262 raises a preamble-detected interrupt before it knows whether
//! a frame is behind the preamble, a header-valid interrupt once one is,
//! and a packet-done interrupt at the end. Between the first and the last
//! a transmit, or a retune to the next hop channel, would trample a frame
//! that is mid-air - so both wait. How long they may wait is the question
//! this answers, and it is a different answer at each stage: a preamble is
//! a weak claim (the detector fires on noise now and then) and is held only
//! for as long as a header would take to follow it; a valid header is a
//! frame, held for the longest one the modulation allows.
//!
//! Every input is a millisecond timestamp and a set of interrupt bits, so
//! the whole of it runs on the host: every sequence of interrupts a poll
//! can read, at every phase of the hold, is checked there against the two
//! things that must always be true - a hold ends, and a valid header is
//! never left unprotected because a stale preamble was still on the books.

/// The longest gap between two receive polls after which what the second
/// one reads is not trusted to set a clock, ms. The hardware loop polls
/// every 10 ms; a transmit holds it for the frame, a card flush for tens
/// of milliseconds and occasionally hundreds, and the packet end a poll
/// timestamps could then be anywhere inside that gap.
pub const LATE_POLL_MS: u64 = 40;

/// How much of an arriving frame has been seen.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Stage {
    /// A preamble, which may be noise.
    Preamble,
    /// A header that passed its check: a frame is on its way.
    Header,
}

/// The interrupt bits a poll read, as the gate cares about them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Irq {
    pub preamble: bool,
    pub header_valid: bool,
    pub header_err: bool,
    pub rx_done: bool,
    pub crc_err: bool,
}

/// What a poll's interrupts amounted to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Seen {
    /// Nothing is arriving.
    Nothing,
    /// A frame may be arriving; the hold is on.
    Arriving,
    /// A packet landed. Its payload is in the radio's buffer whether or
    /// not the CRC passed, so the caller drops a failed one there.
    Packet { crc_ok: bool },
}

/// The receiver's view of the air.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RxGate {
    /// Preamble plus explicit header at the modulation, ms, with the slack
    /// the caller adds: how long a preamble that never became a header is
    /// held.
    header_ms: u32,
    /// The longest frame the modulation allows, ms: how long a header that
    /// never became a packet is held.
    max_frame_ms: u32,
    /// When a preamble or header was first seen with no packet end since.
    busy_since: Option<u64>,
    stage: Option<Stage>,
    prev_poll_ms: u64,
    poll_late: bool,
}

impl RxGate {
    pub const fn new(header_ms: u32, max_frame_ms: u32) -> Self {
        Self {
            header_ms,
            max_frame_ms,
            busy_since: None,
            stage: None,
            prev_poll_ms: 0,
            poll_late: false,
        }
    }

    /// New timings after a re-init: the modulation may have changed, and
    /// whatever was arriving on the old one is gone with it.
    pub fn retime(&mut self, header_ms: u32, max_frame_ms: u32) {
        self.header_ms = header_ms;
        self.max_frame_ms = max_frame_ms;
        self.clear();
    }

    /// Note a poll at `now_ms`, and say whether it came so long after the
    /// last that its timestamps cannot place anything.
    pub fn begin_poll(&mut self, now_ms: u64) -> bool {
        self.poll_late = now_ms.saturating_sub(self.prev_poll_ms) > LATE_POLL_MS;
        self.prev_poll_ms = now_ms;
        self.poll_late
    }

    /// Whether the latest poll was late.
    pub fn poll_late(&self) -> bool {
        self.poll_late
    }

    /// Fold the interrupts a poll read at `now_ms` into what is known.
    ///
    /// The time a frame was first seen is what its hold runs from - a
    /// preamble detector re-firing on noise must not extend it - and a
    /// header lifts the hold from the header time to a whole frame. A
    /// header that failed its check means the radio has given the frame
    /// up. A preamble or header seen after the previous hold has lapsed is
    /// a new frame, and its hold starts now: a valid header must never be
    /// left with a hold that a stale preamble had already spent.
    pub fn observe(&mut self, now_ms: u64, irq: Irq) -> Seen {
        if irq.rx_done {
            self.clear();
            return Seen::Packet {
                crc_ok: !irq.crc_err,
            };
        }
        if irq.header_valid {
            if !self.in_progress(now_ms) {
                self.busy_since = Some(now_ms);
            }
            self.stage = Some(Stage::Header);
        } else if irq.preamble && !self.in_progress(now_ms) {
            self.busy_since = Some(now_ms);
            self.stage = Some(Stage::Preamble);
        }
        if irq.header_err {
            self.clear();
        }
        if self.in_progress(now_ms) {
            Seen::Arriving
        } else {
            Seen::Nothing
        }
    }

    /// How long the receiver is held for the frame it is mid-way through.
    pub fn hold_ms(&self) -> u32 {
        match self.stage {
            Some(Stage::Preamble) => self.header_ms,
            _ => self.max_frame_ms,
        }
    }

    /// Whether a frame may be arriving: something was seen and its hold
    /// has not lapsed. A transmit started now would trample it, and on a
    /// hopping network the retune waits for it too.
    pub fn in_progress(&self, now_ms: u64) -> bool {
        self.busy_since
            .is_some_and(|since| now_ms.saturating_sub(since) < u64::from(self.hold_ms()))
    }

    /// Whether the receiver may leave the channel it is on, for a hop:
    /// yes if nothing is arriving or the hold has run out, bounded by
    /// `cap_ms` (one slot) so a preamble that was noise cannot pin the
    /// receiver on a channel the network has left. Clears the hold on the
    /// way out.
    pub fn may_leave(&mut self, now_ms: u64, cap_ms: u32) -> bool {
        if let Some(since) = self.busy_since {
            let cap = u64::from(self.hold_ms().min(cap_ms));
            if now_ms.saturating_sub(since) < cap {
                return false;
            }
            self.clear();
        }
        true
    }

    /// What has been seen, if anything.
    pub fn stage(&self) -> Option<Stage> {
        self.stage
    }

    /// Forget whatever was arriving: a packet landed, a transmit went out,
    /// the radio was re-initialized.
    pub fn clear(&mut self) {
        self.busy_since = None;
        self.stage = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The default modulation's numbers, as the driver computes them:
    /// 166 ms preamble plus header with 20 ms of slack, 452 ms longest
    /// frame.
    fn gate() -> RxGate {
        RxGate::new(186, 452)
    }

    fn preamble() -> Irq {
        Irq {
            preamble: true,
            ..Irq::default()
        }
    }

    fn header() -> Irq {
        Irq {
            header_valid: true,
            ..Irq::default()
        }
    }

    fn done(crc_ok: bool) -> Irq {
        Irq {
            rx_done: true,
            crc_err: !crc_ok,
            ..Irq::default()
        }
    }

    #[test]
    fn a_preamble_is_held_for_a_header_time_and_no_longer() {
        let mut g = gate();
        assert_eq!(g.observe(1_000, preamble()), Seen::Arriving);
        assert_eq!(g.stage(), Some(Stage::Preamble));
        assert!(g.in_progress(1_185));
        assert!(!g.in_progress(1_186));
        // A re-fire on noise does not extend the hold.
        assert_eq!(g.observe(1_100, preamble()), Seen::Arriving);
        assert!(!g.in_progress(1_186));
        // After the lapse, nothing is arriving.
        assert_eq!(g.observe(1_200, Irq::default()), Seen::Nothing);
    }

    #[test]
    fn a_header_lifts_the_hold_to_a_frame_from_the_preamble() {
        let mut g = gate();
        g.observe(1_000, preamble());
        assert_eq!(g.observe(1_170, header()), Seen::Arriving);
        assert_eq!(g.stage(), Some(Stage::Header));
        assert!(g.in_progress(1_451));
        assert!(!g.in_progress(1_452));
    }

    /// The trap: a preamble that was noise, then a real frame whose
    /// preamble and header bits arrive in one read. The hold must run
    /// from the header, not from the stale preamble.
    #[test]
    fn a_header_after_a_lapsed_preamble_starts_a_fresh_hold() {
        let mut g = gate();
        g.observe(1_000, preamble());
        let both = Irq {
            preamble: true,
            header_valid: true,
            ..Irq::default()
        };
        assert_eq!(g.observe(1_400, both), Seen::Arriving);
        assert!(g.in_progress(1_400 + 451));
        assert!(!g.in_progress(1_400 + 452));
    }

    #[test]
    fn a_packet_end_clears_the_hold_and_says_whether_the_crc_passed() {
        let mut g = gate();
        g.observe(1_000, preamble());
        g.observe(1_170, header());
        assert_eq!(g.observe(1_300, done(true)), Seen::Packet { crc_ok: true });
        assert!(!g.in_progress(1_300));
        assert_eq!(g.stage(), None);
        g.observe(2_000, header());
        assert_eq!(g.observe(2_100, done(false)), Seen::Packet { crc_ok: false });
        assert!(!g.in_progress(2_100));
    }

    #[test]
    fn a_header_error_gives_the_frame_up() {
        let mut g = gate();
        g.observe(1_000, preamble());
        let err = Irq {
            header_err: true,
            ..Irq::default()
        };
        assert_eq!(g.observe(1_100, err), Seen::Nothing);
        assert!(!g.in_progress(1_100));
    }

    /// The hop waits for a frame in progress and no longer than a slot,
    /// and leaving clears what it was waiting for.
    #[test]
    fn a_hop_waits_for_the_hold_and_at_most_a_slot() {
        let mut g = gate();
        g.observe(1_000, header());
        assert!(!g.may_leave(1_100, 1_000));
        assert!(g.may_leave(1_452, 1_000));
        assert_eq!(g.stage(), None);
        // A hold longer than the slot is cut to the slot.
        let mut g = RxGate::new(186, 3_000);
        g.observe(1_000, header());
        assert!(!g.may_leave(1_999, 1_000));
        assert!(g.may_leave(2_000, 1_000));
        // Nothing arriving: free to go.
        assert!(gate().may_leave(0, 1_000));
    }

    #[test]
    fn a_late_poll_is_noticed() {
        let mut g = gate();
        assert!(!g.begin_poll(0));
        assert!(!g.begin_poll(LATE_POLL_MS));
        assert!(g.begin_poll(2 * LATE_POLL_MS + 1));
        assert!(g.poll_late());
        assert!(!g.begin_poll(2 * LATE_POLL_MS + 11));
    }

    #[test]
    fn a_retime_forgets_the_old_modulation_s_frame() {
        let mut g = gate();
        g.observe(1_000, header());
        g.retime(100, 200);
        assert!(!g.in_progress(1_000));
        g.observe(2_000, preamble());
        assert!(g.in_progress(2_099));
        assert!(!g.in_progress(2_100));
    }
}
