//! Broadcast LoRa node: originates frames, receives everyone else's, and
//! optionally repeats them.
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
//!   every single time; the delay also keeps the radio out of a blocking
//!   transmit while more of the same burst is still arriving.

use midair_proto::lora::{Frame, FRAME_MAX, HEADER_LEN};
use midair_proto::radiocfg::{RadioConfig, Role};

use crate::platform;
use crate::radio::PacketRadio;

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
pub enum TxError<E> {
    /// Payload is empty or longer than [`midair_proto::lora::PAYLOAD_MAX`].
    Payload,
    /// This node's role does not transmit.
    Muted,
    /// The radio rejected the transmission.
    Radio(E),
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
pub struct Node<R: PacketRadio> {
    radio: R,
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
}

impl<R: PacketRadio> Node<R> {
    /// Wrap an initialized radio.
    pub fn new(radio: R, cfg: &RadioConfig) -> Self {
        Self {
            radio,
            address: cfg.address,
            role: cfg.role,
            max_hops: cfg.max_hops,
            jitter_ms: cfg.repeat_jitter_ms(),
            seen_ttl_ms: cfg.dedup_ttl_s as u32 * 1000,
            next_id: 0,
            seen: [Seen { src: 0, id: 0, at_ms: 0, valid: false }; SEEN_SLOTS],
            repeats: [Repeat { buf: [0; FRAME_MAX], len: 0, due_ms: 0, valid: false }; REPEAT_SLOTS],
            rx_buf: [0; FRAME_MAX],
            last_rssi: 0,
            last_rx_ms: 0,
            have_rx: false,
            drops: RxDrops::default(),
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
        self.seen = [Seen { src: 0, id: 0, at_ms: 0, valid: false }; SEEN_SLOTS];
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
    pub fn radio(&self) -> &R {
        &self.radio
    }

    /// The radio, mutably (re-init, standby, sleep).
    pub fn radio_mut(&mut self) -> &mut R {
        &mut self.radio
    }

    /// RSSI of the last packet received, in dBm.
    pub fn last_rssi(&self) -> i16 {
        self.last_rssi
    }

    /// [`platform::millis`] timestamp of the last packet received, or
    /// `None` if nothing has been heard yet.
    pub fn last_rx_ms(&self) -> Option<u32> {
        self.have_rx.then_some(self.last_rx_ms)
    }

    /// Cumulative counts of packets dropped after reaching the node, by
    /// reason. Read alongside the radio's CRC/oversize counts for the full
    /// picture of what is not being delivered.
    pub fn rx_drops(&self) -> RxDrops {
        self.drops
    }

    /// Broadcast a payload as a new frame from this node.
    ///
    /// Fails with [`TxError::Muted`] on a receive-only node rather than
    /// reporting a success nothing heard.
    pub fn broadcast(&mut self, payload: &[u8]) -> Result<(), TxError<R::Error>> {
        if !self.role.transmits() {
            return Err(TxError::Muted);
        }
        let frame = Frame {
            src: self.address,
            id: self.next_id,
            hops_left: self.max_hops,
            payload,
        };
        let mut buf = [0u8; FRAME_MAX];
        let n = frame.encode(&mut buf).ok_or(TxError::Payload)?;
        // Claim the id even if the transmission fails, so a retry is a new
        // frame rather than one receivers have already discarded.
        self.next_id = self.next_id.wrapping_add(1);
        self.radio.send(&buf[..n]).map_err(TxError::Radio)
    }

    /// Poll the radio for one frame.
    ///
    /// Duplicates, this node's own frames echoed back by a repeater, and
    /// packets that are not frames at all return `None`. When acting as a
    /// repeater, a frame with hops remaining is queued for forwarding here;
    /// [`send_due_repeat`](Self::send_due_repeat) is what puts it on the air.
    pub fn poll(&mut self, now: u32) -> Option<Received<'_>> {
        let (len, rssi) = match self.radio.poll_recv(&mut self.rx_buf) {
            Ok(Some(v)) => v,
            _ => return None,
        };
        // Record radio liveness for any packet that passed the hardware
        // CRC, whether or not it turns out to be one of ours.
        self.last_rssi = rssi;
        self.last_rx_ms = platform::millis();
        self.have_rx = true;

        // Take the header out as values first: the bookkeeping below needs
        // `&mut self`, so the payload is borrowed back from `rx_buf` only
        // once that is done.
        let (src, id, hops_left) = match Frame::decode(&self.rx_buf[..len]) {
            Some(f) => (f.src, f.id, f.hops_left),
            None => {
                self.drops.malformed = self.drops.malformed.saturating_add(1);
                return None;
            }
        };
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
            let due = now.wrapping_add(platform::random(0, self.jitter_ms as i32) as u32);
            let onward = Frame {
                src,
                id,
                hops_left: hops_left - 1,
                payload: &self.rx_buf[HEADER_LEN..len],
            };
            queue_repeat(&mut self.repeats, &onward, due);
        }
        Some(Received {
            src,
            rssi,
            payload: &self.rx_buf[HEADER_LEN..len],
        })
    }

    /// Whether a queued repeat is ready to transmit.
    ///
    /// Split from [`send_due_repeat`](Self::send_due_repeat) so the caller
    /// can coordinate with the ESP's radio before the blocking transmit.
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
    pub fn send_due_repeat(&mut self, now: u32) -> bool {
        let Some(idx) = self
            .repeats
            .iter()
            .position(|r| r.valid && now.wrapping_sub(r.due_ms) < 0x8000_0000)
        else {
            return false;
        };
        self.repeats[idx].valid = false;
        let len = self.repeats[idx].len;
        self.radio.send(&self.repeats[idx].buf[..len]).is_ok()
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
        self.seen[slot] = Seen { src, id, at_ms: now, valid: true };
        false
    }
}

/// Queue a frame for forwarding, preferring a free slot and otherwise
/// dropping it - overwriting one already waiting would starve whichever
/// node's frame it belonged to.
fn queue_repeat(slots: &mut [Repeat; REPEAT_SLOTS], frame: &Frame<'_>, due_ms: u32) {
    let Some(slot) = slots.iter_mut().find(|r| !r.valid) else {
        debug_println!("Repeat queue full, dropping frame from {}", frame.src);
        return;
    };
    let mut buf = [0u8; FRAME_MAX];
    if let Some(n) = frame.encode(&mut buf) {
        slot.buf = buf;
        slot.len = n;
        slot.due_ms = due_ms;
        slot.valid = true;
    }
}
