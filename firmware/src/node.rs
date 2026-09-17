//! Broadcast LoRa node: originates frames, receives everyone else's, and
//! optionally repeats them.
//!
//! This is the layer the air format lives in. The transmits are `.await`
//! rather than blocking spins, and the radio is owned concretely instead
//! of through a trait: there was one implementation of that trait and
//! there still is.
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
//! Two mechanisms keep repeating from turning into a broadcast storm, both
//! in [`midair_proto::dedup`] where the state space tests can walk them:
//!
//! - **Deduplication.** A frame is identified by `(src, id)`. One that has
//!   been seen recently is neither delivered again nor repeated again, so a
//!   frame that reaches a node by two paths is handled once and a repeater
//!   pair cannot bounce a frame between themselves.
//! - **Jittered forwarding.** A repeat is queued with a random delay rather
//!   than sent from inside the receive path, then moved into this node's
//!   turn of a slot. Two repeaters that heard the same broadcast would
//!   otherwise transmit simultaneously and collide every single time; the
//!   delay also keeps the radio out of a transmit while more of the same
//!   burst is still arriving.

use midair_proto::dedup::{RepeatQueue, SeenTable};
use midair_proto::hop::{Offer, SyncWord};
use midair_proto::lora::{self, Frame, FLAG_SYNC, FRAME_MAX, HEADER_LEN};
use midair_proto::radiocfg::{RadioConfig, Role};

use crate::radio::{Sx1262Driver, Sx1262Error};

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

/// A node on the broadcast network, owning the radio it speaks through.
pub struct Node<'d> {
    radio: Sx1262Driver<'d>,
    address: u8,
    role: Role,
    max_hops: u8,
    jitter_ms: u32,
    /// Sequence number for the next frame this node originates.
    next_id: u8,
    /// `(src, id)` pairs heard inside [`RadioConfig::dedup_ttl_s`]. The
    /// TTL must stay well under the time the 8-bit id takes to wrap at the
    /// beacon interval or a node suppresses its own later frames as
    /// duplicates; the config parser refuses one that does not.
    seen: SeenTable,
    repeats: RepeatQueue,
    rx_buf: [u8; FRAME_MAX],
    last_rssi: i16,
    last_rx_ms: Option<u64>,
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
            next_id: 0,
            seen: SeenTable::new(u64::from(cfg.dedup_ttl_s) * 1000),
            repeats: RepeatQueue::new(),
            rx_buf: [0; FRAME_MAX],
            last_rssi: 0,
            last_rx_ms: None,
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
        self.seen.reset(u64::from(cfg.dedup_ttl_s) * 1000);
        self.repeats.clear();
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

    /// The radio, mutably (re-init, standby, sleep, the schedule).
    pub fn radio_mut(&mut self) -> &mut Sx1262Driver<'d> {
        &mut self.radio
    }

    /// RSSI of the last packet received, in dBm.
    pub fn last_rssi(&self) -> i16 {
        self.last_rssi
    }

    /// Millisecond timestamp of the last packet received, or `None` if
    /// nothing has been heard yet.
    pub fn last_rx_ms(&self) -> Option<u64> {
        self.last_rx_ms
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

    /// Broadcast a payload as a new frame from this node, one sent every
    /// `interval_ms` - which is what picks the turn it goes out in.
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
            // A placeholder the radio overwrites with its clock at the
            // instant of transmission.
            sync: Some(SyncWord::default()),
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

    /// Call a sleeping node: one wake frame, from this node, behind a
    /// preamble of `preamble_syms` symbols, on the wake sync word.
    ///
    /// Not a broadcast in the network's sense - no hop clock, no turn, no
    /// repeat - because the receiver it is for has no clock and is not
    /// listening for the network at all. The frame still carries this
    /// node's address and a fresh id, so the woken node knows who called.
    pub async fn send_wake(
        &mut self,
        wake: &lora::Wake,
        preamble_syms: u16,
    ) -> Result<(), TxError> {
        if !self.role.transmits() {
            return Err(TxError::Muted);
        }
        let payload = wake.encode();
        let frame = Frame {
            src: self.address,
            id: self.next_id,
            hops_left: 0,
            sync: None,
            payload: &payload,
        };
        let mut buf = [0u8; FRAME_MAX];
        let n = frame.encode(&mut buf).ok_or(TxError::Payload)?;
        self.next_id = self.next_id.wrapping_add(1);
        self.radio
            .send_wake_frame(&buf[..n], preamble_syms)
            .await
            .map_err(TxError::Radio)
    }

    /// Poll the radio for one frame.
    ///
    /// Duplicates, this node's own frames echoed back by a repeater, and
    /// packets that are not frames at all return `None`. When acting as a
    /// repeater, a frame with hops remaining is queued for forwarding here;
    /// [`send_due_repeat`](Self::send_due_repeat) is what puts it on the air.
    pub fn poll(&mut self, now_ms: u64) -> Option<Received<'_>> {
        let (len, rssi) = self.radio.poll_recv(&mut self.rx_buf)?;
        // Record radio liveness for any packet that passed the hardware
        // CRC, whether or not it turns out to be one of ours.
        self.last_rssi = rssi;
        self.last_rx_ms = Some(now_ms);

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
        if self.seen.mark(src, id, now_ms) {
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
                sync: Some(SyncWord::default()),
                payload: &self.rx_buf[hlen..len],
            };
            let mut buf = [0u8; FRAME_MAX];
            let queued = match onward.encode(&mut buf) {
                Some(n) => {
                    // The jitter picks a moment; the moment is then moved
                    // into this node's turn of a slot, which the same clock
                    // the poll runs on decides.
                    let due = self.radio.repeat_start(now_ms + u64::from(jitter), n);
                    self.repeats.push(&buf[..n], due)
                }
                None => false,
            };
            if !queued {
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
    pub fn repeat_due(&self, now_ms: u64) -> bool {
        self.repeats.due(now_ms)
    }

    /// Transmit one due repeat, returning whether anything went out.
    ///
    /// The slot is released before the transmit, so a radio error drops the
    /// frame rather than retrying it: by the time the radio is working
    /// again the position it carries is stale, and the node that sent it
    /// has almost certainly beaconed a newer one.
    pub async fn send_due_repeat(&mut self, now_ms: u64) -> bool {
        let Some((mut buf, len)) = self.repeats.take_due(now_ms) else {
            return false;
        };
        // The queued copy carries the flag that says where the sync word
        // sits, which is all the radio needs to stamp it afresh.
        let sync_at = (buf[2] & FLAG_SYNC != 0).then_some(HEADER_LEN);
        self.radio.send(&mut buf[..len], sync_at, 0).await.is_ok()
    }

    /// A pseudo-random value in `0..=max`.
    ///
    /// A xorshift is enough: nothing here is a security decision, and what
    /// the jitter has to do is decorrelate two repeaters - which the
    /// per-address seed already guarantees.
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
