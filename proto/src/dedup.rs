//! The frames a node has seen, and the ones it is waiting to repeat.
//!
//! Two mechanisms keep repeating from turning into a broadcast storm, and
//! both are tables of fixed size on a node with no allocator:
//!
//! - **Deduplication.** A frame is identified by `(src, id)`. One that has
//!   been seen inside the TTL is neither delivered again nor repeated
//!   again, so a frame that reaches a node by two paths is handled once and
//!   a repeater pair cannot bounce a frame between themselves. Seeing a
//!   frame again refreshes its entry, so a frame arriving repeatedly by
//!   several paths stays suppressed for a full TTL after the last copy.
//! - **Jittered forwarding.** A repeat is queued with a due time rather
//!   than sent from inside the receive path. Two repeaters that heard the
//!   same broadcast would otherwise transmit simultaneously and collide
//!   every single time; the delay also keeps the radio out of a transmit
//!   while more of the same burst is still arriving.
//!
//! The id is eight bits and wraps every 256 frames - four minutes at the 1 s
//! default - so the TTL has to stay well inside that or a node starts
//! dropping its own later frames as duplicates. The config parser refuses
//! a TTL that does not.
//!
//! Both tables are pure functions of what they were handed and the clock
//! they were handed it on, which is what lets the state space tests walk
//! every order of arrivals, expiries and wraps on the host.

use crate::lora::FRAME_MAX;

/// Recently seen frames tracked for deduplication. Sized for more nodes
/// than a shared 915 MHz channel can carry beacons for.
pub const SEEN_SLOTS: usize = 16;

/// Frames that can be waiting to be repeated at once. A burst deeper than
/// this means the channel is already saturated, so dropping is the honest
/// response.
pub const REPEAT_SLOTS: usize = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Seen {
    src: u8,
    id: u8,
    at_ms: u64,
}

/// The `(src, id)` pairs seen inside the TTL, `N` of them at most. The
/// firmware uses [`SEEN_SLOTS`]; the state space tests use a table small
/// enough to fill.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SeenTable<const N: usize = SEEN_SLOTS> {
    slots: [Option<Seen>; N],
    ttl_ms: u64,
}

impl<const N: usize> SeenTable<N> {
    pub const fn new(ttl_ms: u64) -> Self {
        Self {
            slots: [None; N],
            ttl_ms,
        }
    }

    /// How long a pair stays remembered.
    pub fn ttl_ms(&self) -> u64 {
        self.ttl_ms
    }

    /// A new TTL, and a fresh table: under a new address this node's own
    /// past frames would look like a remote node's.
    pub fn reset(&mut self, ttl_ms: u64) {
        self.ttl_ms = ttl_ms;
        self.slots = [None; N];
    }

    /// Record `(src, id)` at `now_ms`, returning whether it had already
    /// been seen inside the TTL. A seen pair is refreshed; a new one takes
    /// a free slot, an expired slot, or the oldest.
    pub fn mark(&mut self, src: u8, id: u8, now_ms: u64) -> bool {
        let mut free: Option<usize> = None;
        let mut oldest: Option<(usize, u64)> = None;
        for i in 0..N {
            if let Some(s) = self.slots[i] {
                if now_ms.saturating_sub(s.at_ms) >= self.ttl_ms {
                    self.slots[i] = None;
                }
            }
            match self.slots[i] {
                None => {
                    free.get_or_insert(i);
                }
                Some(s) => {
                    if s.src == src && s.id == id {
                        self.slots[i] = Some(Seen { src, id, at_ms: now_ms });
                        return true;
                    }
                    if oldest.is_none_or(|(_, at)| s.at_ms < at) {
                        oldest = Some((i, s.at_ms));
                    }
                }
            }
        }
        let slot = free.unwrap_or_else(|| oldest.map(|(i, _)| i).unwrap_or(0));
        self.slots[slot] = Some(Seen { src, id, at_ms: now_ms });
        false
    }

    /// Pairs currently remembered, expired ones included until the next
    /// [`mark`](Self::mark) sweeps them.
    pub fn len(&self) -> usize {
        self.slots.iter().flatten().count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Whether `(src, id)` is remembered and inside the TTL at `now_ms`,
    /// without recording anything.
    pub fn contains(&self, src: u8, id: u8, now_ms: u64) -> bool {
        self.slots.iter().flatten().any(|s| {
            s.src == src && s.id == id && now_ms.saturating_sub(s.at_ms) < self.ttl_ms
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Repeat {
    buf: [u8; FRAME_MAX],
    len: usize,
    due_ms: u64,
}

/// Frames waiting to be repeated, each with the instant it is due, `N` of
/// them at most. The firmware uses [`REPEAT_SLOTS`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RepeatQueue<const N: usize = REPEAT_SLOTS> {
    slots: [Option<Repeat>; N],
}

impl<const N: usize> Default for RepeatQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> RepeatQueue<N> {
    pub const fn new() -> Self {
        Self { slots: [None; N] }
    }

    /// Queue `frame` for `due_ms`, preferring a free slot and otherwise
    /// dropping it - overwriting one already waiting would starve whichever
    /// node's frame it belonged to. Returns whether it was queued; a frame
    /// too long for a slot is never queued.
    pub fn push(&mut self, frame: &[u8], due_ms: u64) -> bool {
        if frame.is_empty() || frame.len() > FRAME_MAX {
            return false;
        }
        let Some(slot) = self.slots.iter_mut().find(|r| r.is_none()) else {
            return false;
        };
        let mut buf = [0u8; FRAME_MAX];
        buf[..frame.len()].copy_from_slice(frame);
        *slot = Some(Repeat {
            buf,
            len: frame.len(),
            due_ms,
        });
        true
    }

    /// Whether a queued repeat is due at `now_ms`.
    pub fn due(&self, now_ms: u64) -> bool {
        self.slots.iter().flatten().any(|r| now_ms >= r.due_ms)
    }

    /// Take one due repeat, the earliest due first. The slot is released
    /// on the way out, so a radio error drops the frame rather than
    /// retrying it: by the time the radio is working again the position it
    /// carries is stale, and the node that sent it has almost certainly
    /// beaconed a newer one.
    pub fn take_due(&mut self, now_ms: u64) -> Option<([u8; FRAME_MAX], usize)> {
        let idx = self
            .slots
            .iter()
            .enumerate()
            .filter_map(|(i, r)| r.map(|r| (i, r.due_ms)))
            .filter(|&(_, due)| now_ms >= due)
            .min_by_key(|&(_, due)| due)
            .map(|(i, _)| i)?;
        let r = self.slots[idx].take()?;
        Some((r.buf, r.len))
    }

    /// Frames waiting.
    pub fn len(&self) -> usize {
        self.slots.iter().flatten().count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Drop everything waiting: a new config is not a request to forward
    /// what was admitted under the old one.
    pub fn clear(&mut self) {
        self.slots = [None; N];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_is_new_once_and_seen_until_the_ttl() {
        let mut t: SeenTable = SeenTable::new(3_000);
        assert!(!t.mark(1, 7, 1_000));
        assert!(t.mark(1, 7, 1_500));
        assert!(t.contains(1, 7, 1_500));
        // Seen again at 1500, so the TTL runs from there.
        assert!(t.mark(1, 7, 4_400));
        assert!(!t.mark(1, 7, 7_400));
        // Another id from the same node is another frame.
        assert!(!t.mark(1, 8, 7_400));
        assert_eq!(t.len(), 2);
    }

    #[test]
    fn a_full_table_gives_up_the_oldest() {
        let mut t: SeenTable = SeenTable::new(60_000);
        for i in 0..SEEN_SLOTS as u8 {
            assert!(!t.mark(i, 0, u64::from(i) * 100));
        }
        assert_eq!(t.len(), SEEN_SLOTS);
        // Node 0 at t=0 is the oldest and makes way.
        assert!(!t.mark(200, 0, 10_000));
        assert!(!t.contains(0, 0, 10_000));
        assert!(t.contains(1, 0, 10_000));
        assert!(t.contains(200, 0, 10_000));
    }

    #[test]
    fn a_reset_forgets_everything() {
        let mut t: SeenTable = SeenTable::new(3_000);
        t.mark(1, 1, 0);
        t.reset(5_000);
        assert!(t.is_empty());
        assert_eq!(t.ttl_ms(), 5_000);
        assert!(!t.mark(1, 1, 0));
    }

    #[test]
    fn repeats_come_out_earliest_due_first_and_only_when_due() {
        let mut q: RepeatQueue = RepeatQueue::new();
        assert!(q.push(&[1, 2, 3], 500));
        assert!(q.push(&[4, 5], 200));
        assert!(!q.due(100));
        assert!(q.due(200));
        let (buf, len) = q.take_due(300).unwrap();
        assert_eq!(&buf[..len], &[4, 5]);
        assert!(q.take_due(300).is_none());
        let (buf, len) = q.take_due(500).unwrap();
        assert_eq!(&buf[..len], &[1, 2, 3]);
        assert!(q.is_empty());
    }

    #[test]
    fn a_full_queue_drops_rather_than_overwrites() {
        let mut q: RepeatQueue = RepeatQueue::new();
        for i in 0..REPEAT_SLOTS as u8 {
            assert!(q.push(&[i], 0));
        }
        assert!(!q.push(&[99], 0));
        assert_eq!(q.len(), REPEAT_SLOTS);
        assert!(!q.push(&[], 0), "an empty frame is not a frame");
        assert!(!q.push(&[0; FRAME_MAX + 1], 0), "nor is one too long");
        q.clear();
        assert!(q.is_empty());
    }
}
