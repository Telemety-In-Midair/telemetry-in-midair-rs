//! Broadcast LoRa node: originates frames, receives everyone else's, and
//! optionally repeats them.
//!
//! Ported from the WIO-E5 firmware's `node.rs`. The logic is unchanged -
//! this is the layer the air format lives in, and the air format did not
//! move - but the transmits are `.await` now rather than blocking spins,
//! and the radio is owned concretely instead of through a trait: there was
//! one implementation of that trait and there still is.
//!
//! The network has no routing and no join procedure. Every node transmits
//! [`midair_proto::lora::Frame`] broadcasts and listens continuously, so a
//! fleet of nothing but [`Role::Leaf`] nodes already works - each hears
//! whichever others are in direct range. A [`Role::Repeater`] is a node
//! placed to extend that range: it retransmits frames that still carry
//! hops, and is the only thing that has to be configured differently.
//!
//! [`Role::TxOnly`] and [`Role::RxOnly`] drop one half of that. Position
//! reporting is one-way, so a node that is only ever tracked can keep its
//! receiver off and a node that only collects can stay off the air - each
//! saving the power the unused half would cost.
//!
//! Two mechanisms keep repeating from turning into a broadcast storm:
//!
//! - **Deduplication.** A frame is identified by `(src, id)`. One that has
//!   been seen recently is neither delivered again nor repeated again, so a
//!   frame that reaches a node by two paths is handled once and a repeater
//!   pair cannot bounce a frame between themselves.
//! - **Jittered forwarding.** A repeat is queued with a random delay rather
//!   than sent from inside the receive path. Two repeaters that heard the
//!   same broadcast would otherwise transmit simultaneously and collide
//!   every single time; the delay also keeps the radio out of a transmit
//!   while more of the same burst is still arriving.

use embassy_time::Instant;
use midair_proto::hop::{Offer, SyncWord};
use midair_proto::lora::{Frame, FLAG_SYNC, FRAME_MAX, HEADER_LEN};
use midair_proto::radiocfg::{RadioConfig, Role};

use crate::radio::{Sx1262Driver, Sx1262Error};

/// Recently seen frames tracked for deduplication. Sized for more nodes
/// than a shared 915 MHz channel can carry beacons for.
const SEEN_SLOTS: usize = 16;

/// Frames that can be waiting to be repeated at once. A burst deeper than
/// this means the channel is already saturated, so dropping is the honest
/// response.
const REPEAT_SLOTS: usize = 4;

/// Running totals of packets the node dropped after the radio handed them
/// up but before delivery, for diagnostics. Each saturates and is cleared
/// only by reboot. Radio-layer drops (bad CRC, oversize) are counted
/// separately on the radio, since a packet dropped there never reaches here.
#[derive(Clone, Copy, Default)]
pub struct RxDrops {
    /// This node's own broadcast, echoed back by a repeater.
    pub own_echo: u32,
    /// A frame already seen inside the dedup window.
    pub duplicate: u32,
    /// A packet that did not decode as a frame.
    pub malformed: u32,
    /// A frame a repeater could not queue because the queue was full.
    pub repeat_full: u32,
}

/// A frame received from another node.
pub struct Received<'a> {
    /// Address of the node that originated it (not the repeater that may
    /// have forwarded it).
    pub src: u8,
    /// RSSI of the transmission actually heard, in dBm.
    pub rssi: i16,
    /// Application payload.
    pub payload: &'a [u8],
}

/// Transmit failures.
#[derive(Debug)]
pub enum TxError {
    /// Payload is empty or longer than [`midair_proto::lora::PAYLOAD_MAX`].
    Payload,
    /// This node's role does not transmit.
    Muted,
    /// The radio rejected the transmission.
    Radio(Sx1262Error),
}

#[derive(Clone, Copy)]
struct Seen {
    src: u8,
    id: u8,
    at_ms: u32,
    valid: bool,
}

#[derive(Clone, Copy)]
struct Repeat {
    buf: [u8; FRAME_MAX],
    len: usize,
    due_ms: u32,
    valid: bool,
}

/// A node on the broadcast network, owning the radio it speaks through.
pub struct Node<'d> {
    radio: Sx1262Driver<'d>,
    address: u8,
    role: Role,
    max_hops: u8,
    jitter_ms: u32,
    /// How long a `(src, id)` pair stays in `seen`, in milliseconds. From
    /// [`RadioConfig::dedup_ttl_s`]; must stay well under the time the 8-bit
    /// id takes to wrap at the beacon interval or a node suppresses its own
    /// later frames as duplicates.
    seen_ttl_ms: u32,
    /// Sequence number for the next frame this node originates.
    next_id: u8,
    seen: [Seen; SEEN_SLOTS],
    repeats: [Repeat; REPEAT_SLOTS],
    rx_buf: [u8; FRAME_MAX],
    last_rssi: i16,
    last_rx_ms: u32,
    have_rx: bool,
    drops: RxDrops,
    rng: u32,
    /// A hop clock adopted from a heard frame since the last time anyone
    /// asked: `(sender, this node's new stratum)`. For the log.
    sync_note: Option<(u8, u8)>,
}

impl<'d> Node<'d> {
    /// Wrap an initialized radio.
    pub fn new(radio: Sx1262Driver<'d>, cfg: &RadioConfig) -> Self {
        Self {
            radio,
            address: cfg.address,
            role: cfg.role,
            max_hops: cfg.max_hops,
            jitter_ms: cfg.repeat_jitter_ms(),
            seen_ttl_ms: cfg.dedup_ttl_s as u32 * 1000,
            next_id: 0,
            seen: [Seen {
                src: 0,
                id: 0,
                at_ms: 0,
                valid: false,
            }; SEEN_SLOTS],
            repeats: [Repeat {
                buf: [0; FRAME_MAX],
                len: 0,
                due_ms: 0,
                valid: false,
            }; REPEAT_SLOTS],
            rx_buf: [0; FRAME_MAX],
            last_rssi: 0,
            last_rx_ms: 0,
            have_rx: false,
            drops: RxDrops::default(),
            sync_note: None,
            // Seeded from the address, which is what has to differ: the
            // jitter exists to separate two repeaters that heard the same
            // frame, and two repeaters cannot share an address. Never zero,
            // which is the one state a xorshift cannot leave.
            rng: (cfg.address as u32).wrapping_mul(2_654_435_761) | 1,
        }
    }

    /// Apply a new configuration to the node itself. The caller re-inits the
    /// radio separately, since a config push does not always change one.
    ///
    /// Both the duplicate history and any queued repeats are dropped: under
    /// a new address this node's own past frames would look like a remote
    /// node's, and forwarding a frame admitted under the old settings is not
    /// something the new ones asked for.
    pub fn reconfigure(&mut self, cfg: &RadioConfig) {
        self.address = cfg.address;
        self.role = cfg.role;
        self.max_hops = cfg.max_hops;
        self.jitter_ms = cfg.repeat_jitter_ms();
        self.seen_ttl_ms = cfg.dedup_ttl_s as u32 * 1000;
        self.seen.iter_mut().for_each(|s| s.valid = false);
        self.repeats.iter_mut().for_each(|r| r.valid = false);
    }

    /// This node's address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// This node's role. Ask it directly what the node does on the air:
    /// [`Role::transmits`], [`Role::receives`], [`Role::repeats`].
    pub fn role(&self) -> Role {
        self.role
    }

    /// The radio, for diagnostics and power state.
    pub fn radio(&self) -> &Sx1262Driver<'d> {
        &self.radio
    }

    /// The radio, mutably (re-init, standby, sleep).
    pub fn radio_mut(&mut self) -> &mut Sx1262Driver<'d> {
        &mut self.radio
    }

    /// RSSI of the last packet received, in dBm.
    pub fn last_rssi(&self) -> i16 {
        self.last_rssi
    }

    /// Millisecond timestamp of the last packet received, or `None` if
    /// nothing has been heard yet.
    pub fn last_rx_ms(&self) -> Option<u32> {
        self.have_rx.then_some(self.last_rx_ms)
    }

    /// Cumulative counts of packets dropped after reaching the node, by
    /// reason. Read alongside the radio's CRC/oversize counts for the full
    /// picture of what is not being delivered.
    pub fn rx_drops(&self) -> RxDrops {
        self.drops
    }

    /// The last hop clock adopted from a heard frame, once: who it came
    /// from and the stratum this node has because of it.
    pub fn take_sync_note(&mut self) -> Option<(u8, u8)> {
        self.sync_note.take()
    }

    /// The sync word a frame this node sends carries: a placeholder the
    /// radio overwrites at the instant of transmission on a scheduled
    /// network, nothing otherwise. Sent on a one-channel plan too - it
    /// carries the clock, which is needed wherever the schedule runs.
    fn sync_placeholder(&self) -> Option<SyncWord> {
        self.radio.scheduled().then_some(SyncWord::default())
    }

    /// Broadcast a payload as a new frame from this node, one sent every
    /// `interval_ms` - which on a scheduled network is what picks the turn
    /// it goes out in.
    ///
    /// Fails with [`TxError::Muted`] on a receive-only node rather than
    /// reporting a success nothing heard.
    pub async fn broadcast(&mut self, payload: &[u8], interval_ms: u32) -> Result<(), TxError> {
        if !self.role.transmits() {
            return Err(TxError::Muted);
        }
        let frame = Frame {
            src: self.address,
            id: self.next_id,
            hops_left: self.max_hops,
            sync: self.sync_placeholder(),
            payload,
        };
        let mut buf = [0u8; FRAME_MAX];
        let n = frame.encode(&mut buf).ok_or(TxError::Payload)?;
        // Claim the id even if the transmission fails, so a retry is a new
        // frame rather than one receivers have already discarded.
        self.next_id = self.next_id.wrapping_add(1);
        self.radio
            .send(&mut buf[..n], frame.sync_offset(), interval_ms)
            .await
            .map_err(TxError::Radio)
    }

    /// Bytes ahead of the payload in a frame this node sends.
    pub fn frame_overhead(&self) -> usize {
        HEADER_LEN + if self.radio.scheduled() { midair_proto::hop::SYNC_LEN } else { 0 }
    }

    /// Poll the radio for one frame.
    ///
    /// Duplicates, this node's own frames echoed back by a repeater, and
    /// packets that are not frames at all return `None`. When acting as a
    /// repeater, a frame with hops remaining is queued for forwarding here;
    /// [`send_due_repeat`](Self::send_due_repeat) is what puts it on the air.
    pub fn poll(&mut self, now: u32) -> Option<Received<'_>> {
        let (len, rssi) = self.radio.poll_recv(&mut self.rx_buf)?;
        // Record radio liveness for any packet that passed the hardware
        // CRC, whether or not it turns out to be one of ours.
        self.last_rssi = rssi;
        self.last_rx_ms = now;
        self.have_rx = true;

        // Take the header out as values first: the bookkeeping below needs
        // `&mut self`, so the payload is borrowed back from `rx_buf` only
        // once that is done.
        let (src, id, hops_left, sync, hlen) = match Frame::decode(&self.rx_buf[..len]) {
            Some(f) => (f.src, f.id, f.hops_left, f.sync, f.header_len()),
            None => {
                self.drops.malformed = self.drops.malformed.saturating_add(1);
                return None;
            }
        };
        // Every frame that decoded is a real transmission with a real clock
        // behind it, so all of them are offered - the echo of this node's
        // own broadcast included, since what it carries is the repeater's
        // clock, not this node's. What is dropped below is the *message*.
        if let Some(word) = sync
            && let Offer::Adopted(stratum) = self.radio.hop_heard(word, src, self.address, len)
        {
            self.sync_note = Some((src, stratum));
        }
        // Our own transmission, forwarded back to us by a repeater.
        //
        // Only a node that transmits can hear itself. One that does not has
        // nothing of its own on the air, so a frame carrying its address
        // really did come from someone else and dropping it would blind a
        // receive-only base station to exactly one node - most likely the
        // one that, like the base station, was left at the default address.
        if self.role.transmits() && src == self.address {
            self.drops.own_echo = self.drops.own_echo.saturating_add(1);
            return None;
        }
        if self.mark_seen(src, id, now) {
            self.drops.duplicate = self.drops.duplicate.saturating_add(1);
            return None;
        }
        if self.role.repeats() && hops_left > 0 {
            let jitter = self.random(self.jitter_ms);
            let onward = Frame {
                src,
                id,
                hops_left: hops_left - 1,
                // This node's clock, not the sender's: the radio writes it
                // when the repeat actually goes out.
                sync: self.sync_placeholder(),
                payload: &self.rx_buf[hlen..len],
            };
            // The jitter picks a moment; on a hopping network the moment
            // is then moved into a slot's window, which the same clock the
            // poll runs on decides. Kept in the caller's 32-bit domain.
            let wanted = Instant::now().as_millis() + u64::from(jitter);
            let start = self.radio.tx_window_start(wanted, onward.encoded_len(), 0);
            let due = now.wrapping_add((start - Instant::now().as_millis().min(start)) as u32);
            if !queue_repeat(&mut self.repeats, &onward, due) {
                self.drops.repeat_full = self.drops.repeat_full.saturating_add(1);
            }
        }
        Some(Received {
            src,
            rssi,
            payload: &self.rx_buf[hlen..len],
        })
    }

    /// Whether a queued repeat is ready to transmit.
    pub fn repeat_due(&self, now: u32) -> bool {
        self.repeats
            .iter()
            .any(|r| r.valid && now.wrapping_sub(r.due_ms) < 0x8000_0000)
    }

    /// Transmit one due repeat, returning whether anything went out.
    ///
    /// The slot is released before the transmit, so a radio error drops the
    /// frame rather than retrying it: by the time the radio is working
    /// again the position it carries is stale, and the node that sent it
    /// has almost certainly beaconed a newer one.
    pub async fn send_due_repeat(&mut self, now: u32) -> bool {
        let Some(idx) = self
            .repeats
            .iter()
            .position(|r| r.valid && now.wrapping_sub(r.due_ms) < 0x8000_0000)
        else {
            return false;
        };
        self.repeats[idx].valid = false;
        let len = self.repeats[idx].len;
        let mut buf = [0u8; FRAME_MAX];
        buf[..len].copy_from_slice(&self.repeats[idx].buf[..len]);
        // The queued copy carries the flag that says where the sync word
        // sits, which is all the radio needs to stamp it afresh.
        let sync_at = (buf[2] & FLAG_SYNC != 0).then_some(HEADER_LEN);
        self.radio.send(&mut buf[..len], sync_at, 0).await.is_ok()
    }

    /// Record a `(src, id)` pair, returning whether it had already been
    /// seen inside [`seen_ttl_ms`](Self::seen_ttl_ms).
    fn mark_seen(&mut self, src: u8, id: u8, now: u32) -> bool {
        let mut free: Option<usize> = None;
        let mut oldest = 0usize;
        for i in 0..SEEN_SLOTS {
            let s = self.seen[i];
            if s.valid && now.wrapping_sub(s.at_ms) >= self.seen_ttl_ms {
                self.seen[i].valid = false;
            }
            if !self.seen[i].valid {
                free.get_or_insert(i);
            } else {
                if s.src == src && s.id == id {
                    // Refresh, so a frame arriving repeatedly by several
                    // paths stays suppressed for a full TTL after the last
                    // copy rather than the first.
                    self.seen[i].at_ms = now;
                    return true;
                }
                if now.wrapping_sub(s.at_ms) > now.wrapping_sub(self.seen[oldest].at_ms) {
                    oldest = i;
                }
            }
        }
        let slot = free.unwrap_or(oldest);
        self.seen[slot] = Seen {
            src,
            id,
            at_ms: now,
            valid: true,
        };
        false
    }

    /// A pseudo-random value in `0..=max`.
    ///
    /// The WIO drew this from its DWT cycle counter, which the Xtensa core
    /// has no equivalent of. A xorshift is enough: nothing here is a
    /// security decision, and what the jitter has to do is decorrelate two
    /// repeaters - which the per-address seed already guarantees.
    pub fn random(&mut self, max: u32) -> u32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        if max == 0 {
            0
        } else {
            self.rng % (max + 1)
        }
    }
}

/// Queue a frame for forwarding, preferring a free slot and otherwise
/// dropping it - overwriting one already waiting would starve whichever
/// node's frame it belonged to. Returns whether it was queued.
fn queue_repeat(slots: &mut [Repeat; REPEAT_SLOTS], frame: &Frame<'_>, due_ms: u32) -> bool {
    let Some(slot) = slots.iter_mut().find(|r| !r.valid) else {
        return false;
    };
    let mut buf = [0u8; FRAME_MAX];
    match frame.encode(&mut buf) {
        Some(n) => {
            slot.buf = buf;
            slot.len = n;
            slot.due_ms = due_ms;
            slot.valid = true;
            true
        }
        None => false,
    }
}
