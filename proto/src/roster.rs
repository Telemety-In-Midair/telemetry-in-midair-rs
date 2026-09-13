//! The latest report heard from each remote node, and the BLE values it
//! hands out.
//!
//! One slot per node, newest report wins. That is the whole idea: a node
//! says one thing at a time - a position while it has a fix, a
//! [`crate::lora::Ping`] while it does not - and what it said before is
//! superseded, not queued. A single cached report instead loses a node
//! whenever two report between one notification and the next, and a plain
//! queue lets one fast-beaconing node push every other node out of it.
//!
//! Reports are handed out once each ([`Roster::take_dirty`]), so a value on
//! the air means something was actually heard rather than that a timer
//! fired. Ages are stamped in on the way out, from when the report arrived
//! rather than from anything inside it - the sender chooses which fields to
//! spend air time on and `tod_ms` is not among the defaults, so there is
//! nothing in a beacon to age it by.
//!
//! Names sit beside the reports rather than inside them. A node announces
//! what it is called on its own slow cadence (see
//! [`crate::lora::MSG_NAME`]), so a name is not something a node "last
//! said" - it is what the node is, true of it between reports, before its
//! first one, and across the change from a position to a ping and back. A
//! name is therefore handed out on its own ([`Roster::take_dirty_name`])
//! and only when it is news: first heard, or changed.
//!
//! This lives in the shared crate rather than the firmware because it
//! emits the exact BLE byte layouts (see [`crate::ble`]) and can be tested
//! on the host, which a `no_std` binary cannot be.

use crate::ble;
use crate::link;
use gps_proto::packet;

/// Nodes tracked at once. A shared LoRa channel saturates well before this
/// many nodes are beaconing on it, so the table is not the limit.
pub const SLOTS: usize = 8;

/// A node not heard from in this long is forgotten rather than handed to the
/// next central that connects. Long against the 20 s default beacon, so
/// falling out takes a node genuinely off the air rather than a missed
/// transmission or two.
pub const TTL_MS: u64 = 30 * 60 * 1000;

/// What a node last told us, in the bytes the link delivered.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Report {
    /// Position report: `[src, rssi, packet]`.
    Position([u8; ble::REMOTE_LEN]),
    /// Node ping: `[src, rssi, flags, uptime]`.
    Ping([u8; link::PING_LEN]),
}

impl Report {
    /// Originating node address, which is byte 0 of either payload.
    pub fn src(&self) -> u8 {
        match self {
            Report::Position(b) => b[0],
            Report::Ping(b) => b[0],
        }
    }
}

/// One report as a BLE characteristic value, age stamped in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Value {
    /// For [`ble::REMOTE_UUID`].
    Position([u8; ble::REMOTE_LEN_V2]),
    /// For [`ble::NODE_PING_UUID`].
    Ping([u8; ble::NODE_PING_LEN]),
}

impl Value {
    /// The bytes to notify.
    pub fn bytes(&self) -> &[u8] {
        match self {
            Value::Position(b) => b,
            Value::Ping(b) => b,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Slot {
    report: Report,
    /// When the report arrived, on the caller's monotonic millisecond clock.
    at_ms: u64,
    /// Set on arrival, cleared once handed out, so one report produces one
    /// notification rather than a value resent on every tick.
    dirty: bool,
}

/// What a node calls itself, and whether that is still news.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct NameSlot {
    /// The node that announced it.
    src: u8,
    /// The label, zero-padded - the shape the BLE value wants, and the
    /// shape the sender keeps it in on its own flash.
    label: [u8; ble::NAME_FIELD_LEN],
    /// When this node was last heard from at all, not when it last
    /// announced its name: a name announcement is one transmission in
    /// twenty, and a name that expired between two of them would be
    /// forgotten and re-learned for a node that never went off the air.
    at_ms: u64,
    /// Set when the name is first heard and when it changes, cleared once
    /// handed out. A node re-announcing the name it already had is not
    /// news and does not notify.
    dirty: bool,
}

/// Per-node table of the latest report from each remote node, and of what
/// each node calls itself.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Roster {
    slots: [Option<Slot>; SLOTS],
    names: [Option<NameSlot>; SLOTS],
}

impl Roster {
    pub const fn new() -> Self {
        Self { slots: [None; SLOTS], names: [None; SLOTS] }
    }

    /// Record a node's newest report, replacing whatever that node last said.
    ///
    /// A full table gives up the node that has gone quietest. Entries past
    /// [`TTL_MS`] are dropped first, so it is a node long off the air that
    /// makes way rather than a live one that happens to beacon slowly.
    pub fn record(&mut self, now_ms: u64, report: Report) {
        let src = report.src();
        self.expire(now_ms);
        let idx = self
            .slots
            .iter()
            .position(|s| matches!(s, Some(s) if s.report.src() == src))
            .or_else(|| self.slots.iter().position(Option::is_none))
            .unwrap_or_else(|| self.oldest());
        self.slots[idx] = Some(Slot { report, at_ms: now_ms, dirty: true });
        // Hearing from a node keeps its name alive, whatever the report
        // was: the name rides a far slower cadence than the beacon, and
        // aging it by its own last announcement would drop the name of a
        // node that is reporting every second.
        if let Some(n) = self.names.iter_mut().flatten().find(|n| n.src == src) {
            n.at_ms = now_ms;
        }
    }

    /// Record what a node calls itself, returning whether this is news -
    /// a node named for the first time, or one that has been renamed.
    ///
    /// A label that is not one a board would store is ignored rather than
    /// recorded: the caller has already refused it on the way off the air
    /// (see [`crate::lora::decode_name`]), and this is the second place a
    /// byte string would have to pass to reach a display.
    ///
    /// A name arriving for a node nothing has been heard from is kept.
    /// That is the ordinary case rather than an edge one: a node announces
    /// its name as its first transmission after boot, so the name usually
    /// arrives before the first position.
    pub fn record_name(&mut self, now_ms: u64, src: u8, label: &str) -> bool {
        if src == 0 || !ble::valid_label(label.as_bytes()) {
            return false;
        }
        self.expire(now_ms);
        let mut padded = [0u8; ble::NAME_FIELD_LEN];
        padded[..label.len()].copy_from_slice(label.as_bytes());
        if let Some(n) = self.names.iter_mut().flatten().find(|n| n.src == src) {
            n.at_ms = now_ms;
            if n.label == padded {
                return false;
            }
            n.label = padded;
            n.dirty = true;
            return true;
        }
        let idx = self
            .names
            .iter()
            .position(Option::is_none)
            .unwrap_or_else(|| self.oldest_name());
        self.names[idx] = Some(NameSlot { src, label: padded, at_ms: now_ms, dirty: true });
        true
    }

    /// Take the oldest report still waiting to go out, as the value to
    /// notify. `None` once every node's current report has been handed out.
    ///
    /// Oldest first, so a central connecting to a board that has heard
    /// several nodes receives them in the order they were heard.
    pub fn take_dirty(&mut self, now_ms: u64) -> Option<Value> {
        let mut pick: Option<(usize, u64)> = None;
        for (i, slot) in self.slots.iter().enumerate() {
            let Some(s) = *slot else { continue };
            let older = match pick {
                Some((_, at)) => s.at_ms < at,
                None => true,
            };
            if s.dirty && older {
                pick = Some((i, s.at_ms));
            }
        }
        let (idx, at_ms) = pick?;
        let slot = self.slots[idx].as_mut()?;
        slot.dirty = false;
        let age = (now_ms.saturating_sub(at_ms) / 1000).min(ble::AGE_MAX_S as u64) as u16;
        Some(match slot.report {
            Report::Position(b) => {
                let mut v = [0u8; ble::REMOTE_LEN_V2];
                v[..ble::REMOTE_LEN].copy_from_slice(&b);
                v[ble::REMOTE_AGE_OFF..].copy_from_slice(&age.to_le_bytes());
                Value::Position(v)
            }
            Report::Ping(b) => {
                let mut v = [0u8; ble::NODE_PING_LEN];
                v[..link::PING_LEN].copy_from_slice(&b);
                v[ble::NODE_PING_AGE_OFF..].copy_from_slice(&age.to_le_bytes());
                Value::Ping(v)
            }
        })
    }

    /// Take the oldest name still waiting to go out, as the value to
    /// notify on [`ble::NODE_NAME_UUID`]: `[src, label zero-padded]`.
    /// `None` once every name has been handed out.
    ///
    /// Separate from [`Roster::take_dirty`] because a name is not a report:
    /// it is not superseded by the next thing the node says, and a node
    /// reporting every second must not re-notify a name that has not
    /// changed since it booted.
    ///
    /// No age travels with it, unlike a report: a name is not a
    /// measurement that goes stale, and the age of the node it belongs to
    /// is already on that node's own value.
    pub fn take_dirty_name(&mut self) -> Option<[u8; ble::NODE_NAME_LEN]> {
        let mut pick: Option<(usize, u64)> = None;
        for (i, slot) in self.names.iter().enumerate() {
            let Some(n) = *slot else { continue };
            let older = match pick {
                Some((_, at)) => n.at_ms < at,
                None => true,
            };
            if n.dirty && older {
                pick = Some((i, n.at_ms));
            }
        }
        let (idx, _) = pick?;
        let slot = self.names[idx].as_mut()?;
        slot.dirty = false;
        let mut v = [0u8; ble::NODE_NAME_LEN];
        v[0] = slot.src;
        v[1..].copy_from_slice(&slot.label);
        Some(v)
    }

    /// What a node calls itself, or `None` for one that has not said.
    ///
    /// For a console line or a display: an address is what the frame
    /// carries, a name is what the operator recognizes.
    pub fn name(&self, src: u8) -> Option<&str> {
        let slot = self.names.iter().flatten().find(|n| n.src == src)?;
        let end = slot
            .label
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(ble::NAME_LABEL_MAX);
        core::str::from_utf8(&slot.label[..end]).ok()
    }

    /// Re-arm every node still inside the TTL, so a central that has just
    /// connected receives the whole roster rather than only the next node to
    /// report.
    ///
    /// Nodes past the TTL are dropped instead of replayed: a position from a
    /// node half an hour off the air would put a marker on a map with
    /// nothing behind it. The ages that go out with the replay are what
    /// separate the rest from live reports.
    pub fn replay(&mut self, now_ms: u64) {
        self.expire(now_ms);
        for slot in self.slots.iter_mut().flatten() {
            slot.dirty = true;
        }
        // Names too: a central that has just connected has no idea what
        // any of these nodes are called, and the next announcement is
        // twenty of the sender's transmissions away.
        for name in self.names.iter_mut().flatten() {
            name.dirty = true;
        }
    }

    /// The most recently heard node that reported a position, as
    /// `(src, packet bytes, age in seconds, rssi)`.
    ///
    /// This is what the compass points at. Newest rather than nearest,
    /// deliberately: "nearest" needs a distance to every node on every
    /// refresh and, worse, makes the arrow jump between nodes as two of
    /// them trade places at similar range. Newest changes only when a
    /// different node is actually heard from, which is a change the
    /// operator can see a reason for.
    ///
    /// Ping-only nodes are skipped - they carry no position, so there is
    /// nothing to point at.
    pub fn newest_position(
        &self,
        now_ms: u64,
    ) -> Option<(u8, [u8; packet::POSITION_PACKET_LEN], u16, i16)> {
        let mut pick: Option<(&Slot, u64)> = None;
        for slot in self.slots.iter().flatten() {
            if !matches!(slot.report, Report::Position(_)) {
                continue;
            }
            if now_ms.saturating_sub(slot.at_ms) > TTL_MS {
                continue;
            }
            if pick.is_none_or(|(_, at)| slot.at_ms > at) {
                pick = Some((slot, slot.at_ms));
            }
        }
        let (slot, at_ms) = pick?;
        let Report::Position(b) = slot.report else {
            return None;
        };
        let mut pkt = [0u8; packet::POSITION_PACKET_LEN];
        // Layout is [src, rssi u16, packet]; see `ble::REMOTE_LEN`.
        pkt.copy_from_slice(&b[3..]);
        let age = (now_ms.saturating_sub(at_ms) / 1000).min(ble::AGE_MAX_S as u64) as u16;
        Some((b[0], pkt, age, i16::from_le_bytes([b[1], b[2]])))
    }

    /// Nodes currently remembered.
    pub fn len(&self) -> usize {
        self.slots.iter().flatten().count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Forget nodes not heard from within [`TTL_MS`], names included.
    fn expire(&mut self, now_ms: u64) {
        for slot in self.slots.iter_mut() {
            let stale = match *slot {
                Some(s) => now_ms.saturating_sub(s.at_ms) >= TTL_MS,
                None => false,
            };
            if stale {
                *slot = None;
            }
        }
        for name in self.names.iter_mut() {
            let stale = match *name {
                Some(n) => now_ms.saturating_sub(n.at_ms) >= TTL_MS,
                None => false,
            };
            if stale {
                *name = None;
            }
        }
    }

    /// Index of the name slot for the node longest unheard, for the same
    /// reason [`Roster::oldest`] exists: a full table gives up the
    /// quietest node.
    fn oldest_name(&self) -> usize {
        let mut oldest = 0;
        for i in 1..SLOTS {
            let older = match (self.names[i], self.names[oldest]) {
                (Some(a), Some(b)) => a.at_ms < b.at_ms,
                (None, _) => true,
                _ => false,
            };
            if older {
                oldest = i;
            }
        }
        oldest
    }

    /// Index of the slot holding the oldest report; empty slots count as
    /// infinitely old, so this only matters on a full table.
    fn oldest(&self) -> usize {
        let mut oldest = 0;
        for i in 1..SLOTS {
            let older = match (self.slots[i], self.slots[oldest]) {
                (Some(a), Some(b)) => a.at_ms < b.at_ms,
                (None, _) => true,
                _ => false,
            };
            if older {
                oldest = i;
            }
        }
        oldest
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn position(src: u8, rssi: i16) -> Report {
        let mut b = [0u8; ble::REMOTE_LEN];
        b[0] = src;
        b[1..3].copy_from_slice(&rssi.to_le_bytes());
        // A recognizable packet body: lat_e7 = src, so a value can be traced
        // back to the node it came from.
        b[3..7].copy_from_slice(&(src as i32).to_le_bytes());
        Report::Position(b)
    }

    fn ping(src: u8, uptime_s: u16) -> Report {
        let mut b = [0u8; link::PING_LEN];
        b[0] = src;
        b[4..6].copy_from_slice(&uptime_s.to_le_bytes());
        Report::Ping(b)
    }

    fn drain(r: &mut Roster, now_ms: u64) -> Vec<Value> {
        let mut out = Vec::new();
        while let Some(v) = r.take_dirty(now_ms) {
            out.push(v);
        }
        out
    }

    /// Age of a value, whichever kind it is.
    fn age(v: &Value) -> u16 {
        let b = v.bytes();
        let off = match v {
            Value::Position(_) => ble::REMOTE_AGE_OFF,
            Value::Ping(_) => ble::NODE_PING_AGE_OFF,
        };
        u16::from_le_bytes([b[off], b[off + 1]])
    }

    /// The bug this table exists for: two nodes reporting between one
    /// notification and the next must both be delivered, not just the
    /// second.
    #[test]
    fn two_nodes_in_one_window_both_survive() {
        let mut r = Roster::new();
        r.record(1_000, position(3, -80));
        r.record(1_100, position(7, -95));
        let got = drain(&mut r, 1_200);
        assert_eq!(got.len(), 2);
        // Oldest first: node 3 was heard before node 7.
        assert_eq!(got[0].bytes()[0], 3);
        assert_eq!(got[1].bytes()[0], 7);
        assert_eq!(r.len(), 2);
    }

    /// A node's newer report supersedes its older one instead of queueing
    /// behind it, so a fast beacon costs one slot and one notification.
    #[test]
    fn newest_report_per_node_wins() {
        let mut r = Roster::new();
        r.record(1_000, position(3, -80));
        r.record(2_000, position(3, -60));
        assert_eq!(r.len(), 1);
        let got = drain(&mut r, 2_000);
        assert_eq!(got.len(), 1);
        assert_eq!(i16::from_le_bytes([got[0].bytes()[1], got[0].bytes()[2]]), -60);
    }

    /// A node that loses its fix stops presenting the position it can no
    /// longer stand behind.
    #[test]
    fn a_ping_replaces_that_nodes_position() {
        let mut r = Roster::new();
        r.record(1_000, position(3, -80));
        r.record(2_000, ping(3, 120));
        assert_eq!(r.len(), 1);
        let got = drain(&mut r, 2_000);
        assert!(matches!(got.as_slice(), [Value::Ping(_)]));

        // And back again once it has a fix.
        r.record(3_000, position(3, -80));
        assert!(matches!(drain(&mut r, 3_000).as_slice(), [Value::Position(_)]));
    }

    /// Each report goes out once. A value resent every tick cannot be told
    /// apart from a node still reporting.
    #[test]
    fn a_report_is_handed_out_once() {
        let mut r = Roster::new();
        r.record(1_000, position(3, -80));
        assert!(r.take_dirty(1_000).is_some());
        assert!(r.take_dirty(1_000).is_none());
        assert!(r.take_dirty(9_000).is_none());
        // The node is still known, just not news.
        assert_eq!(r.len(), 1);
    }

    /// Age is measured from arrival on the receiver's clock, which is the
    /// only clock both a lean beacon and a rich one leave available.
    /// Age is measured from arrival on the receiver's clock, which is the
    /// only clock both a lean beacon and a rich one leave available.
    #[test]
    fn age_counts_from_arrival() {
        let mut r = Roster::new();
        r.record(1_000, position(3, -80));
        let v = r.take_dirty(1_000 + 42_000).unwrap();
        assert_eq!(v.bytes().len(), ble::REMOTE_LEN_V2);
        assert_eq!(age(&v), 42);

        // A report handed out as it arrives is aged zero: a live report and
        // a replayed one are not the same thing.
        r.record(100_000, ping(4, 7));
        let v = r.take_dirty(100_000).unwrap();
        assert_eq!(v.bytes().len(), ble::NODE_PING_LEN);
        assert_eq!(age(&v), 0);
    }

    /// Whole seconds, so a long-idle node cannot saturate the field into
    /// looking recent.
    #[test]
    fn age_truncates_and_saturates() {
        let mut r = Roster::new();
        r.record(0, position(3, -80));
        assert_eq!(age(&r.take_dirty(1_999).unwrap()), 1);

        // Past what the field can hold it pins at the maximum rather than
        // wrapping. Only reachable with a TTL longer than this build's.
        r.record(0, position(4, -80));
        assert_eq!(age(&r.take_dirty(u64::MAX).unwrap()), ble::AGE_MAX_S);
    }

    /// Everything a reader that predates the age field knows about stays
    /// where it was, which is what lets an old app read a new board.
    #[test]
    fn the_value_keeps_the_link_bytes_in_place() {
        let mut r = Roster::new();
        let Report::Position(sent) = position(9, -101) else {
            unreachable!()
        };
        r.record(1_000, Report::Position(sent));
        let v = r.take_dirty(1_000).unwrap();
        assert_eq!(&v.bytes()[..ble::REMOTE_LEN], &sent);

        let Report::Ping(sent) = ping(9, 4_242) else {
            unreachable!()
        };
        r.record(1_000, Report::Ping(sent));
        let v = r.take_dirty(1_000).unwrap();
        assert_eq!(&v.bytes()[..link::PING_LEN], &sent);
    }

    /// A table with no room drops the node that has gone quietest, not the
    /// one that reported first.
    #[test]
    fn a_full_table_evicts_the_quietest_node() {
        let mut r = Roster::new();
        for i in 0..SLOTS {
            r.record(1_000 + i as u64, position(i as u8 + 1, -80));
        }
        assert_eq!(r.len(), SLOTS);
        // Node 1 is the oldest; refreshing it makes node 2 the quietest.
        r.record(5_000, position(1, -70));
        r.record(6_000, position(99, -80));
        assert_eq!(r.len(), SLOTS);

        let srcs: Vec<u8> = drain(&mut r, 6_000).iter().map(|v| v.bytes()[0]).collect();
        assert!(srcs.contains(&99), "the new node must be admitted");
        assert!(srcs.contains(&1), "a refreshed node must not be evicted");
        assert!(!srcs.contains(&2), "the quietest node makes way");
    }

    /// An expired node makes way before a live one does.
    #[test]
    fn expired_nodes_go_first() {
        let mut r = Roster::new();
        r.record(0, position(1, -80));
        for i in 1..SLOTS {
            r.record(TTL_MS + i as u64, position(i as u8 + 1, -80));
        }
        // Node 1 is now past the TTL, so the table has room without
        // evicting any of the live nodes.
        r.record(TTL_MS + 100, position(99, -80));
        assert_eq!(r.len(), SLOTS);
        let srcs: Vec<u8> = drain(&mut r, TTL_MS + 100).iter().map(|v| v.bytes()[0]).collect();
        assert!(!srcs.contains(&1));
        assert!(srcs.contains(&99));
        assert!(srcs.contains(&2));
    }

    /// A connect hands over every node still inside the TTL, with ages, and
    /// forgets the ones that are not.
    #[test]
    fn replay_covers_the_live_roster_only() {
        let mut r = Roster::new();
        r.record(1_000, position(3, -80));
        r.record(2_000, ping(4, 30));
        // Both already delivered to whoever was connected at the time.
        assert_eq!(drain(&mut r, 2_000).len(), 2);

        let now = 2_000 + TTL_MS - 1_000;
        r.record(now, position(5, -90));
        r.replay(now);
        let got = drain(&mut r, now);
        // Node 3 aged out; nodes 4 and 5 are replayed, oldest first.
        assert_eq!(got.len(), 2);
        assert_eq!(got[0].bytes()[0], 4);
        assert_eq!(got[1].bytes()[0], 5);
        // The replayed node carries its real age, so it cannot be read as
        // having just reported.
        assert_eq!(age(&got[0]), ((TTL_MS - 1_000) / 1000) as u16);
        assert_eq!(age(&got[1]), 0);
    }

    #[test]
    fn an_empty_roster_hands_out_nothing() {
        let mut r = Roster::new();
        assert!(r.is_empty());
        assert!(r.take_dirty(0).is_none());
        r.replay(0);
        assert!(r.take_dirty(0).is_none());
    }

    // -- what a node calls itself -----------------------------------------

    /// The label out of a name value, read to its padding.
    fn label(v: &[u8; ble::NODE_NAME_LEN]) -> &str {
        let end = v[1..].iter().position(|&b| b == 0).unwrap_or(ble::NAME_LABEL_MAX);
        core::str::from_utf8(&v[1..1 + end]).unwrap()
    }

    #[test]
    fn a_name_is_handed_out_and_read_back() {
        let mut r = Roster::new();
        assert!(r.record_name(1_000, 3, "sky-1"));
        assert_eq!(r.name(3), Some("sky-1"));
        assert_eq!(r.name(4), None, "a node that has not said");

        let v = r.take_dirty_name().expect("a name to hand out");
        assert_eq!(v[0], 3);
        assert_eq!(label(&v), "sky-1");
        // Once each, like a report.
        assert!(r.take_dirty_name().is_none());
    }

    /// A name is what the node is, not what it last said: the position and
    /// ping values keep coming and the name is not resent with them.
    #[test]
    fn a_name_is_not_news_twice() {
        let mut r = Roster::new();
        assert!(r.record_name(1_000, 3, "sky-1"));
        assert!(r.take_dirty_name().is_some());

        assert!(!r.record_name(2_000, 3, "sky-1"), "the same name is not news");
        assert!(r.take_dirty_name().is_none());

        // A rename is.
        assert!(r.record_name(3_000, 3, "sky-2"));
        assert_eq!(label(&r.take_dirty_name().unwrap()), "sky-2");
        assert_eq!(r.name(3), Some("sky-2"));
    }

    /// Reports and names are independent: a node going from a position to
    /// a ping keeps its name, and a name does not displace a position.
    #[test]
    fn a_name_and_a_report_do_not_displace_each_other() {
        let mut r = Roster::new();
        r.record(1_000, position(3, -80));
        r.record_name(1_100, 3, "sky-1");
        assert_eq!(r.len(), 1, "a name is not a second node");
        assert!(r.newest_position(1_100).is_some(), "the position survived");

        r.record(2_000, ping(3, 60));
        assert_eq!(r.name(3), Some("sky-1"), "and the name survived the ping");

        // Draining the reports leaves the name to be handed out on its own.
        assert_eq!(drain(&mut r, 2_000).len(), 1);
        assert!(r.take_dirty_name().is_some());
    }

    /// The usual order on the air: a node announces its name as its first
    /// transmission, so the name arrives before anything has been heard
    /// from that node at all.
    #[test]
    fn a_name_can_arrive_before_the_first_report() {
        let mut r = Roster::new();
        assert!(r.record_name(1_000, 7, "ground-1"));
        assert_eq!(r.len(), 0, "still nothing reported");
        assert_eq!(r.name(7), Some("ground-1"));

        r.record(2_000, position(7, -70));
        assert_eq!(r.newest_position(2_000).unwrap().0, 7);
        assert_eq!(r.name(7), Some("ground-1"));
    }

    /// A byte string that is not a label never reaches the table, whatever
    /// arrived on the air.
    #[test]
    fn a_name_that_is_not_a_label_is_refused() {
        let mut r = Roster::new();
        assert!(!r.record_name(1_000, 3, "sky 1"));
        assert!(!r.record_name(1_000, 3, ""));
        assert!(!r.record_name(1_000, 3, &"a".repeat(ble::NAME_LABEL_MAX + 1)));
        // Address 0 is not assignable, so nothing can be named by it.
        assert!(!r.record_name(1_000, 0, "sky-1"));
        assert!(r.take_dirty_name().is_none());
        assert_eq!(r.name(3), None);
    }

    /// A name lives as long as the node is heard from, not as long as its
    /// last announcement - which is twenty of the sender's transmissions
    /// behind.
    #[test]
    fn a_report_keeps_a_name_alive() {
        let mut r = Roster::new();
        r.record_name(0, 3, "sky-1");
        for t in (0..TTL_MS * 3).step_by((TTL_MS / 2) as usize) {
            r.record(t, position(3, -80));
            assert_eq!(r.name(3), Some("sky-1"), "at {t}");
        }
        // A node that does go off the air is forgotten, name and all.
        r.record(TTL_MS * 4, position(4, -80));
        assert_eq!(r.name(3), None);
    }

    /// A central that has just connected is told what every node it is
    /// about to hear about is called.
    #[test]
    fn replay_covers_the_names() {
        let mut r = Roster::new();
        r.record_name(1_000, 3, "sky-1");
        r.record_name(2_000, 4, "ground-1");
        assert!(r.take_dirty_name().is_some());
        assert!(r.take_dirty_name().is_some());
        assert!(r.take_dirty_name().is_none());

        r.replay(2_000);
        // Oldest first, like the reports.
        assert_eq!(label(&r.take_dirty_name().unwrap()), "sky-1");
        assert_eq!(label(&r.take_dirty_name().unwrap()), "ground-1");
        assert!(r.take_dirty_name().is_none());

        // An expired node is not replayed.
        r.record_name(3_000, 5, "sky-2");
        r.replay(3_000 + TTL_MS);
        assert!(r.take_dirty_name().is_none());
        assert_eq!(r.name(5), None);
    }

    /// A full name table gives up the node longest unheard, the way the
    /// report table does.
    #[test]
    fn a_full_name_table_evicts_the_quietest_node() {
        let mut r = Roster::new();
        for i in 0..SLOTS {
            r.record_name(1_000 + i as u64, i as u8 + 1, "sky-1");
        }
        // Node 1 is the oldest; hearing a report from it makes node 2 the
        // quietest.
        r.record(5_000, position(1, -70));
        assert!(r.record_name(6_000, 99, "new-1"));
        assert_eq!(r.name(99), Some("new-1"));
        assert_eq!(r.name(1), Some("sky-1"), "a node heard from is not evicted");
        assert_eq!(r.name(2), None, "the quietest made way");
    }

    /// The longest label a board stores survives the round trip, padding
    /// and all - the field is one byte longer than the label so a reader
    /// always finds a zero to stop at.
    #[test]
    fn the_longest_label_round_trips() {
        let long = "a".repeat(ble::NAME_LABEL_MAX);
        let mut r = Roster::new();
        assert!(r.record_name(1_000, 3, &long));
        let v = r.take_dirty_name().unwrap();
        assert_eq!(v.len(), ble::NODE_NAME_LEN);
        assert_eq!(label(&v), long);
        assert_eq!(v[ble::NODE_NAME_LEN - 1], 0, "the padding is always there");
        assert_eq!(r.name(3).map(str::to_owned), Some(long));
    }

    // -- what the compass points at ---------------------------------------

    /// Newest, not first-seen and not nearest: the arrow follows whichever
    /// node was heard from last.
    #[test]
    fn the_compass_target_is_the_newest_position() {
        let mut r = Roster::new();
        r.record(1_000, position(7, -80));
        r.record(2_000, position(9, -95));
        let (src, pkt, _, rssi) = r.newest_position(2_500).expect("a target");
        assert_eq!(rssi, -95, "the rssi of that node's report, not another's");
        assert_eq!(src, 9);
        assert_eq!(i32::from_le_bytes(pkt[0..4].try_into().unwrap()), 9);

        // An older node reporting again takes the arrow back.
        r.record(3_000, position(7, -80));
        assert_eq!(r.newest_position(3_100).unwrap().0, 7);
    }

    /// A ping carries no position, so a fleet of nodes that have never had
    /// a fix leaves nothing to point at rather than pointing at nothing.
    #[test]
    fn pings_are_not_compass_targets() {
        let mut r = Roster::new();
        r.record(1_000, ping(4, 60));
        assert!(r.newest_position(1_100).is_none());

        // A position from another node is picked even though the ping is
        // newer, because the ping was never a candidate.
        r.record(2_000, position(5, -70));
        r.record(3_000, ping(4, 90));
        assert_eq!(r.newest_position(3_100).unwrap().0, 5);
    }

    /// An expired node must not be walked towards. Its last position is
    /// half an hour old and it is the one target where being confidently
    /// wrong costs the operator a walk.
    #[test]
    fn an_expired_node_is_not_a_target() {
        let mut r = Roster::new();
        r.record(1_000, position(3, -70));
        assert!(r.newest_position(1_000 + TTL_MS).is_some(), "inside the ttl");
        assert!(r.newest_position(1_001 + TTL_MS).is_none(), "past it");
    }

    /// The age and signal strength come back with the target, so a display
    /// can say how stale the bearing it is drawing actually is and how well
    /// the node it points at is being heard.
    #[test]
    fn the_target_carries_its_age() {
        let mut r = Roster::new();
        r.record(1_000, position(2, -60));
        assert_eq!(r.newest_position(1_000).unwrap().2, 0);
        assert_eq!(r.newest_position(46_000).unwrap().2, 45);
        assert_eq!(r.newest_position(1_000).unwrap().3, -60, "rssi rides along");
    }

    #[test]
    fn an_empty_roster_has_no_target() {
        assert!(Roster::new().newest_position(1_000).is_none());
    }
}
