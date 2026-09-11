//! The internal flash, and the three things that share it.
//!
//! One peripheral, three unrelated users - the settings mirror, the radio
//! config's backup copy and the OTA writer - so they live in one object
//! rather than three that would each need their own claim on it. Nothing
//! here is on a hot path: the settings are written when an app changes one,
//! a config when somebody pushes one, and an image only during an update.
//!
//! **Settings.** RTC RAM is lost when the cell goes flat, so a sleeping
//! board would come back with nothing configured and advertise at full
//! current until it died again. The same record is therefore mirrored into
//! the `nvs` partition, which RTC RAM then caches (see [`crate::settings`]);
//! only a cold boot reads flash, so the wake-check path stays free of it.
//! This claims the partition but does *not* use the ESP-IDF NVS key/value
//! format - it is two fixed records at fixed offsets, and nothing else on
//! this board reads the region.
//!
//! **Radio config.** The card is the store, and a board with no card had
//! nowhere to keep a pushed config: it ran until the next power cycle and
//! then came back on firmware defaults, losing the node address, which is
//! the one setting nothing can guess back. A copy of the config text
//! therefore goes into the sector after the settings record, and the boot
//! path reads it when the card has nothing to say (see
//! [`midair_proto::cfgstore`] for the record and why it is framed that way).
//!
//! **OTA.** [`OtaSink`] writes an application image into whichever slot is
//! not running and hands it to the bootloader, replacing the WIO-E5's
//! page-swap bootloader and `fw-upload`. Writes are staged a sector at a
//! time: [`embedded_storage::Storage`] does a read-modify-erase-write per
//! sector, so writing straight through at the 192-byte transfer chunk size
//! would erase every sector twenty-one times over and turn a four-second
//! update into a seventy-second one.
//!
//! A board flashed with a single-app partition table has no OTA slots. That
//! is not an error - the firmware runs identically - so every OTA entry
//! point degrades to "not available" and the bulk transfer refuses the kind.
//!
//! **Event log.** The fourth user, and the fourth region: the `coredump`
//! data partition holds the ring of [`midair_proto::evlog`] records the
//! board keeps about itself. Unlike the other three it is appended to, a
//! record at a time, with a sector erased only as the ring enters it -
//! see [`Flash::evlog_append`]. A board whose table has no such partition
//! keeps its events on the console only.
//!
//! **Every program and erase holds this core's critical section.** The
//! flash driver parks the other core for the duration of a write, because
//! that core would otherwise fetch instructions through a cache the write
//! has to disable. It parks it *before* taking its own lock and unparks it
//! *after* releasing, which leaves two gaps in which an interrupt on this
//! core can run while the other core is stalled. If that interrupt needs
//! the critical-section spinlock and the stalled core was holding it - it
//! holds one every time it touches the shared state, and it is stalled at
//! a random instruction - the interrupt spins on a lock that will never be
//! released, this core never reaches the unpark, and the board is two
//! stopped cores. Holding the critical section around the whole operation
//! closes both gaps: the other core cannot hold the spinlock when it is
//! parked, since this core holds it, and no interrupt runs on this core
//! until the other core is running again.

use esp_bootloader_esp_idf::ota::{Ota, OtaImageState};
use esp_bootloader_esp_idf::ota_updater::OtaUpdater;
use esp_bootloader_esp_idf::partitions::{
    self, AppPartitionSubType, DataPartitionSubType, PartitionType,
};
use embedded_storage::{ReadStorage, Storage};
use esp_println::println;
use esp_storage::FlashStorage;
use midair_proto::bulk::Sink;
use midair_proto::cfgstore;
use midair_proto::evlog::{self, Head, Record, Ring, Slot};
use midair_proto::session::{Stored, RECORD_LEN};

/// Run a flash program or erase with this core's critical section held,
/// so the other core is not parked holding the spinlock and no interrupt
/// on this core runs while it is parked. See the module doc.
///
/// Reads need none of this - they go through the cache like any other
/// access - so the comparisons that decide whether a write is needed at
/// all stay outside, with interrupts on.
fn exclusive<R>(f: impl FnOnce() -> R) -> R {
    critical_section::with(|_| f())
}

/// Where the event log's ring is, once opened.
#[derive(Clone, Copy, Debug)]
struct EventLog {
    ring: Ring,
    head: Head,
}

/// Flash sector size: the unit an erase works in, and so the unit an image
/// is staged in.
const SECTOR: usize = FlashStorage::SECTOR_SIZE as usize;

/// Where the config backup starts inside `nvs`: the sector after the one the
/// settings record sits in.
///
/// A sector of its own rather than a neighboring offset, because a write is
/// a read-modify-erase-write of the whole sector around it - so sharing one
/// would put every config push through an erase of the settings, and a power
/// loss during a config write would take the duty cycle with it.
const CONFIG_AT: u32 = SECTOR as u32;

/// Bytes compared at a time when checking whether a write would change
/// anything. Small enough to be a stack buffer on the boot path, which is
/// the only reason the comparison is chunked at all.
const COMPARE_CHUNK: usize = 64;

pub struct Flash {
    storage: FlashStorage<'static>,
    /// Scratch for the partition table, re-read per operation rather than
    /// cached: it is a 3 KiB read a handful of times per boot, and keeping
    /// a parsed table would mean keeping its borrow of this buffer alive
    /// across every other use of the flash.
    table: [u8; partitions::PARTITION_TABLE_MAX_LEN],
    /// The sector being assembled for the OTA slot.
    staged: [u8; SECTOR],
    /// Offset into the image of `staged[0]`.
    staged_at: u32,
    /// Bytes of `staged` that hold image data.
    staged_len: usize,
    /// The slot the running transfer writes into, chosen once at the start
    /// of it. Held rather than re-derived per sector so the destination
    /// cannot move under an image that is half written.
    target: Option<AppPartitionSubType>,
    /// The event log, once [`evlog_open`](Self::evlog_open) has found it.
    evlog: Option<EventLog>,
}

impl Flash {
    /// Claim the flash. Call once.
    pub fn new(flash: esp_hal::peripherals::FLASH<'static>) -> Self {
        Self {
            // The hardware loop runs on the second core, so every write
            // has to park it: it would otherwise fetch through the cache
            // the write disables. `exclusive` is what makes the park safe.
            storage: FlashStorage::new(flash).multicore_auto_park(),
            table: [0; partitions::PARTITION_TABLE_MAX_LEN],
            staged: [0; SECTOR],
            staged_at: 0,
            staged_len: 0,
            target: None,
            evlog: None,
        }
    }

    /// Read the saved settings, or `None` if there are none to read: no
    /// partition table, no `nvs` partition, erased flash, another program's
    /// bytes, or a record the crc rejects. The board then comes up
    /// unconfigured rather than acting on garbage.
    pub fn load_settings(&mut self) -> Option<Stored> {
        let mut rec = [0u8; RECORD_LEN];
        self.with_nvs(|region| region.read(0, &mut rec).is_ok())?
            .then(|| Stored::decode_record(&rec))?
    }

    /// Persist the settings. Returns whether they landed.
    ///
    /// A failure is not fatal: the RTC RAM copy still drives this power
    /// cycle, and only the survive-a-flat-battery guarantee is lost.
    pub fn save_settings(&mut self, stored: &Stored) -> bool {
        let rec = stored.encode_record();
        self.with_nvs(|region| {
            // An identical record is not written again.
            //
            // `Storage::write` erases and rewrites the whole sector around
            // the record - which is what makes it safe, since there is no
            // separate erase step to get wrong, and also what makes it
            // expensive. Every accepted settings write asks for a save
            // whether or not the value moved, so an app that pushes what
            // the board already has spends a sector erase and something
            // like 40 ms with interrupts disabled on a no-op, inside a BLE
            // session, where that is a dropped connection event or two.
            //
            // The record is deterministic - fixed fields and a crc over
            // them, no timestamps - so comparing it is exact rather than a
            // heuristic. A read costs nothing next to an erase.
            let mut current = [0u8; RECORD_LEN];
            if region.read(0, &mut current).is_ok() && current == rec {
                return true;
            }
            exclusive(|| region.write(0, &rec)).is_ok()
        })
        .unwrap_or(false)
    }

    /// Erase the settings record and the config backup, so the next boot
    /// finds nothing stored. Returns whether both landed.
    ///
    /// Written as erased flash rather than erased as sectors: `write` is
    /// the one operation the region offers, it is an erase-and-rewrite of
    /// the sector underneath, and a record of `0xFF` is exactly what an
    /// erased partition reads as - `load_settings` and `load_config` refuse
    /// it by its magic word, the same way they refuse a blank part.
    pub fn wipe(&mut self) -> bool {
        self.with_nvs(|region| {
            let settings = exclusive(|| region.write(0, &[0xFF; RECORD_LEN])).is_ok();
            let config = !config_fits(region)
                || exclusive(|| region.write(CONFIG_AT, &[0xFF; cfgstore::HEADER_LEN])).is_ok();
            settings && config
        })
        .unwrap_or(false)
    }

    /// Read the backed-up config text into `buf`, returning its length.
    ///
    /// `None` when there is nothing to read, which is every reason a record
    /// can fail to be one: no partition table, no `nvs` partition, erased
    /// flash, another program's bytes, a header this build does not
    /// understand, or a text the crc rejects. The caller then falls back to
    /// the card or to defaults rather than acting on garbage.
    ///
    /// A text longer than `buf` is refused rather than truncated, for the
    /// reason [`crate::sdlog::SdLog::read_config`] refuses one: the parser
    /// accepts any prefix that ends on a line boundary, so a truncated read
    /// would be adopted as whatever fitted and reported as loaded.
    pub fn load_config(&mut self, buf: &mut [u8]) -> Option<usize> {
        self.with_nvs(|region| {
            if !config_fits(region) {
                return None;
            }
            let mut hdr = [0u8; cfgstore::HEADER_LEN];
            region.read(CONFIG_AT, &mut hdr).ok()?;
            let header = cfgstore::Header::decode(&hdr)?;
            if header.len > buf.len() {
                println!("nvs: stored config is {} bytes, too big to read", header.len);
                return None;
            }
            let text = &mut buf[..header.len];
            region
                .read(CONFIG_AT + cfgstore::HEADER_LEN as u32, text)
                .ok()?;
            // Said out loud rather than folded into "none stored", because
            // the two have different causes: a header that decoded and a
            // text that does not match it is an interrupted write or a
            // failing part, not a board that has never been configured.
            if !header.matches(text) {
                println!("nvs: stored config failed its crc, ignoring it");
                return None;
            }
            Some(header.len)
        })
        .flatten()
    }

    /// Back up the config text. Returns whether it landed.
    ///
    /// The header and the text go down in one write, and the record is
    /// assembled here rather than written in two halves: `Storage::write`
    /// erases the whole sector around whatever it is handed, so two writes
    /// would be two erases - and the first of them would already have taken
    /// the record being replaced. Which makes the crc, not the order the
    /// halves are programmed in, the thing that makes an interrupted write
    /// safe: a half-programmed sector fails it and reads as nothing stored.
    ///
    /// An identical record is not written again. Every cold boot with a card
    /// asks for this save so the backup keeps up with a card edited on a
    /// computer, and a card that has not changed is the common case - where
    /// a comparison is a handful of reads and a write is a sector erase,
    /// tens of milliseconds of it with interrupts off, in the middle of
    /// whatever the BLE side is doing.
    pub fn save_config(&mut self, text: &[u8]) -> bool {
        let Some(header) = cfgstore::Header::for_text(text) else {
            return false;
        };
        let mut buf = [0u8; cfgstore::RECORD_MAX];
        let len = cfgstore::HEADER_LEN + text.len();
        buf[..cfgstore::HEADER_LEN].copy_from_slice(&header.encode());
        buf[cfgstore::HEADER_LEN..len].copy_from_slice(text);
        let rec = &buf[..len];
        self.with_nvs(|region| {
            if !config_fits(region) {
                return false;
            }
            if region_holds(region, CONFIG_AT, rec) {
                return true;
            }
            exclusive(|| region.write(CONFIG_AT, rec)).is_ok()
        })
        .unwrap_or(false)
    }

    // -- The event log ------------------------------------------------------

    fn with_evlog<R>(
        &mut self,
        f: impl FnOnce(&mut partitions::FlashRegion<'_, FlashStorage<'static>>) -> R,
    ) -> Option<R> {
        let Self { storage, table, .. } = self;
        let pt = partitions::read_partition_table(storage, table).ok()?;
        let entry = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Coredump))
            .ok()??;
        let mut region = entry.as_embedded_storage(storage);
        Some(f(&mut region))
    }

    /// One slot's bytes.
    fn evlog_slot(
        region: &mut partitions::FlashRegion<'_, FlashStorage<'static>>,
        ring: &Ring,
        slot: usize,
    ) -> Option<[u8; evlog::RECORD_LEN]> {
        let mut buf = [0u8; evlog::RECORD_LEN];
        region.read(ring.offset(slot), &mut buf).ok()?;
        Some(buf)
    }

    /// What a slot holds. A slot that cannot be read counts as junk: not
    /// a record, and not something to program over.
    fn evlog_probe(
        region: &mut partitions::FlashRegion<'_, FlashStorage<'static>>,
        ring: &Ring,
        slot: usize,
    ) -> Slot {
        Self::evlog_slot(region, ring, slot).map_or(Slot::Junk, |b| Record::probe(&b))
    }

    /// Erase the sector a slot begins.
    fn evlog_erase_sector(
        region: &mut partitions::FlashRegion<'_, FlashStorage<'static>>,
        ring: &Ring,
        slot: usize,
    ) -> bool {
        let from = ring.offset(slot);
        exclusive(|| {
            embedded_storage::nor_flash::NorFlash::erase(region, from, from + SECTOR as u32)
        })
        .is_ok()
    }

    /// Find the log and where its next record goes. `None` on a board
    /// whose partition table has no log partition. Called once at boot;
    /// the head is then kept here and moved by every append.
    pub fn evlog_open(&mut self) -> Option<Head> {
        let found = self.with_evlog(|region| {
            let ring = Ring::new(region.capacity(), SECTOR)?;
            let head = ring.locate(|slot| Self::evlog_probe(region, &ring, slot));
            Some(EventLog { ring, head })
        })??;
        self.evlog = Some(found);
        Some(found.head)
    }

    /// Append one record. Returns its sequence number, or `None` if the
    /// log is not open or the write did not land.
    ///
    /// The record's slot is programmed once, on erased flash: the sector
    /// is erased as the ring enters it, which takes the oldest records
    /// with it, and a slot that turns out not to be blank - a program a
    /// reset interrupted - is skipped rather than programmed over, since
    /// flash bits only clear and no crc would pass the result.
    pub fn evlog_append(&mut self, kind: evlog::Kind, uptime_s: u32, text: &str) -> Option<u32> {
        let mut log = self.evlog?;
        let seq = log.head.next_seq;
        let record = Record::new(kind, seq, uptime_s, log.head.next_boot, text).encode();
        let landed = self.with_evlog(|region| {
            let ring = log.ring;
            let mut slot = log.head.slot;
            // Skip forward over anything already programmed, up to the
            // next sector boundary, which is erased on entry.
            while !ring.erase_before(slot) && Self::evlog_probe(region, &ring, slot) != Slot::Blank
            {
                slot = (slot + 1) % ring.slots;
            }
            if ring.erase_before(slot) && !Self::evlog_erase_sector(region, &ring, slot) {
                return false;
            }
            let at = ring.offset(slot);
            if exclusive(|| embedded_storage::nor_flash::NorFlash::write(region, at, &record))
                .is_err()
            {
                return false;
            }
            log.head.slot = (slot + 1) % ring.slots;
            true
        })?;
        if !landed {
            return None;
        }
        log.head.next_seq = seq.wrapping_add(1).max(1);
        log.head.count = (log.head.count + 1).min(log.ring.slots);
        self.evlog = Some(log);
        Some(seq)
    }

    /// The `i`-th newest record: 0 is the last one written.
    pub fn evlog_read(&mut self, i: usize) -> Option<Record> {
        let log = self.evlog?;
        if i >= log.head.count {
            return None;
        }
        let slot = log.ring.newest(log.head.slot, i)?;
        self.with_evlog(|region| {
            Self::evlog_slot(region, &log.ring, slot).and_then(|b| Record::decode(&b))
        })?
    }

    /// How many records the log holds.
    pub fn evlog_count(&mut self) -> usize {
        self.evlog.map_or(0, |log| log.head.count)
    }

    /// Erase the whole log. The next append starts from the first slot.
    pub fn evlog_erase(&mut self) -> bool {
        let Some(mut log) = self.evlog else {
            return false;
        };
        let ok = self
            .with_evlog(|region| {
                let len = region.capacity() as u32;
                exclusive(|| embedded_storage::nor_flash::NorFlash::erase(region, 0, len)).is_ok()
            })
            .unwrap_or(false);
        if ok {
            log.head = Head {
                slot: 0,
                next_seq: 1,
                next_boot: log.head.next_boot,
                count: 0,
            };
            self.evlog = Some(log);
        }
        ok
    }

    fn with_nvs<R>(
        &mut self,
        f: impl FnOnce(&mut partitions::FlashRegion<'_, FlashStorage<'static>>) -> R,
    ) -> Option<R> {
        let Self { storage, table, .. } = self;
        let pt = partitions::read_partition_table(storage, table).ok()?;
        let entry = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
            .ok()??;
        let mut region = entry.as_embedded_storage(storage);
        Some(f(&mut region))
    }

    fn with_ota<R>(
        &mut self,
        f: impl FnOnce(&mut OtaUpdater<'_, FlashStorage<'static>>) -> R,
    ) -> Option<R> {
        let Self { storage, table, .. } = self;
        let mut updater = OtaUpdater::new(storage, table).ok()?;
        Some(f(&mut updater))
    }

    /// Run `f` against the ota-data partition itself.
    ///
    /// [`OtaUpdater`] covers most of what this file needs, but not setting
    /// an arbitrary slot as current, which [`normalize_otadata`] does.
    ///
    /// [`normalize_otadata`]: Self::normalize_otadata
    fn with_ota_data<R>(
        &mut self,
        f: impl FnOnce(&mut Ota<'_, FlashStorage<'static>>) -> R,
    ) -> Option<R> {
        let Self { storage, table, .. } = self;
        let pt = partitions::read_partition_table(storage, table).ok()?;
        let entry = pt
            .find_partition(PartitionType::Data(DataPartitionSubType::Ota))
            .ok()??;
        let mut region = entry.as_embedded_storage(storage);
        let mut ota = Ota::new(&mut region, OTA_SLOTS).ok()?;
        Some(f(&mut ota))
    }

    /// Which app slot the bootloader actually mapped, read from the MMU
    /// rather than from ota-data.
    ///
    /// These two can disagree, and the disagreement is the dangerous case:
    /// ota-data says nothing on a board just flashed over USB, while the
    /// code running is unambiguously in a particular slot. Anything that
    /// decides where an update goes has to believe this one.
    pub fn booted_slot(&mut self) -> Option<AppPartitionSubType> {
        let Self { storage, table, .. } = self;
        let pt = partitions::read_partition_table(storage, table).ok()?;
        let entry = pt.booted_partition().ok()??;
        match entry.partition_type() {
            PartitionType::App(slot) => Some(slot),
            _ => None,
        }
    }

    /// Whether this board has the two app slots and the ota-data partition
    /// an update needs.
    pub fn ota_available(&mut self) -> bool {
        let Self { storage, table, .. } = self;
        let Ok(pt) = partitions::read_partition_table(storage, table) else {
            return false;
        };
        let has = |t: PartitionType| matches!(pt.find_partition(t), Ok(Some(_)));
        has(PartitionType::Data(DataPartitionSubType::Ota))
            && has(PartitionType::App(AppPartitionSubType::Ota0))
            && has(PartitionType::App(AppPartitionSubType::Ota1))
    }

    /// Point ota-data at the slot that is actually running, if it does not
    /// name a slot at all.
    ///
    /// Erased ota-data reads back as "factory selected", which is the state
    /// a board is in after every USB flash - the flash step erases it
    /// deliberately, so that the image it just wrote to ota_0 is the one
    /// that boots. From there the sequence arithmetic that picks the *other*
    /// slot has no slot to work from, and works out to the running one. This
    /// writes the first real sequence number so every later step is an
    /// ordinary slot-to-slot move.
    ///
    /// Returns whether it wrote anything.
    pub fn normalize_otadata(&mut self) -> bool {
        if !self.ota_available() {
            return false;
        }
        let Some(booted) = self.booted_slot() else {
            return false;
        };
        if !matches!(
            booted,
            AppPartitionSubType::Ota0 | AppPartitionSubType::Ota1
        ) {
            return false;
        }
        self.with_ota_data(|ota| {
            match ota.current_app_partition() {
                Ok(AppPartitionSubType::Factory) => {}
                // Already naming a slot, or unreadable - either way this is
                // not the case being fixed.
                _ => return false,
            }
            exclusive(|| ota.set_current_app_partition(booted)).is_ok()
        })
        .unwrap_or(false)
    }

    /// The app slot ota-data currently selects, for the boot line.
    pub fn selected_slot(&mut self) -> Option<AppPartitionSubType> {
        self.with_ota(|u| u.selected_partition().ok())?
    }

    /// Tell the bootloader this image works.
    ///
    /// With rollback enabled the bootloader marks a freshly activated image
    /// `PendingVerify` and reverts to the previous slot unless the image
    /// itself says otherwise before the next reset. Reaching this line means
    /// the firmware got as far as a working console and a claimed set of
    /// pins, which is the most a boot-time check can honestly assert.
    ///
    /// Silent where the state is already `Valid` or where there are no OTA
    /// partitions at all, since neither is news.
    pub fn confirm_boot(&mut self) -> bool {
        self.with_ota(|u| {
            let state = u.current_ota_state().unwrap_or(OtaImageState::Undefined);
            if !matches!(state, OtaImageState::New | OtaImageState::PendingVerify) {
                return false;
            }
            exclusive(|| u.set_current_ota_state(OtaImageState::Valid)).is_ok()
        })
        .unwrap_or(false)
    }

    /// A writer for the slot that is not running.
    pub fn ota_sink(&mut self) -> OtaSink<'_> {
        OtaSink(self)
    }

    /// Run `f` against one app partition's flash region.
    fn with_app<R>(
        &mut self,
        slot: AppPartitionSubType,
        f: impl FnOnce(&mut partitions::FlashRegion<'_, FlashStorage<'static>>) -> R,
    ) -> Option<R> {
        let Self { storage, table, .. } = self;
        let pt = partitions::read_partition_table(storage, table).ok()?;
        let entry = pt.find_partition(PartitionType::App(slot)).ok()??;
        let mut region = entry.as_embedded_storage(storage);
        Some(f(&mut region))
    }

    /// Write one staged sector into the slot the transfer claimed.
    fn flush_staged(&mut self) -> bool {
        if self.staged_len == 0 {
            return true;
        }
        let Some(slot) = self.target else {
            return false;
        };
        let at = self.staged_at;
        let len = self.staged_len;
        // `staged` and the flash are both fields of `self` and `with_app`
        // wants the flash, so the bytes are handed over as a raw pointer's
        // worth of bounds rather than a borrow that would overlap.
        let ok = {
            let Self {
                storage,
                table,
                staged,
                ..
            } = self;
            let pt = match partitions::read_partition_table(storage, table) {
                Ok(pt) => pt,
                Err(_) => return false,
            };
            match pt.find_partition(PartitionType::App(slot)) {
                Ok(Some(entry)) => {
                    let mut region = entry.as_embedded_storage(storage);
                    exclusive(|| region.write(at, &staged[..len])).is_ok()
                }
                _ => false,
            }
        };
        if ok {
            self.staged_at += len as u32;
            self.staged_len = 0;
        }
        ok
    }
}

/// The `nvs` partition as the two config-backup helpers see it.
type NvsRegion<'a> = partitions::FlashRegion<'a, FlashStorage<'static>>;

/// Whether the partition has room for a full-length config behind the
/// settings record.
///
/// The table this firmware flashes gives `nvs` 16 KiB, which is four times
/// what the two records need. A board carrying a smaller one keeps its
/// settings and simply has no backup, rather than writing a record that
/// runs off the end of the partition into whatever follows it.
fn config_fits(region: &mut NvsRegion<'_>) -> bool {
    region.capacity() >= CONFIG_AT as usize + cfgstore::RECORD_MAX
}

/// Whether the region already holds `bytes` at `at`.
///
/// The whole record is compared, byte for byte and in chunks so it costs a
/// small stack buffer rather than a second copy of the config. Comparing
/// only the header would be cheaper and wrong: an interrupted write leaves
/// one that still describes the record it was replacing, and re-pushing that
/// config is exactly when a header-only check would report a save over a
/// record that does not load.
fn region_holds(region: &mut NvsRegion<'_>, at: u32, bytes: &[u8]) -> bool {
    let mut chunk = [0u8; COMPARE_CHUNK];
    for (i, want) in bytes.chunks(COMPARE_CHUNK).enumerate() {
        let off = at + (i * COMPARE_CHUNK) as u32;
        let got = &mut chunk[..want.len()];
        if region.read(off, got).is_err() || got != want {
            return false;
        }
    }
    true
}

/// App slots this build expects, and the count the ota-data arithmetic is
/// done modulo. The partition table has exactly two.
const OTA_SLOTS: usize = 2;

/// The [`Sink`] a firmware transfer writes through.
///
/// Borrows the flash for the length of one bulk op rather than owning it,
/// so the settings mirror is still reachable between chunks.
pub struct OtaSink<'a>(&'a mut Flash);

impl Sink for OtaSink<'_> {
    fn begin(&mut self, total: u32) -> bool {
        self.0.staged_at = 0;
        self.0.staged_len = 0;
        self.0.target = None;
        if !self.0.ota_available() {
            return false;
        }
        // The slot ota-data would move to next, which after
        // `normalize_otadata` is the one that is not running.
        let Some(Some(slot)) = self.0.with_ota(|u| u.next_partition().ok().map(|(_, s)| s)) else {
            return false;
        };
        // Refuse rather than trust it. Writing an image over the slot the
        // code is executing from does not fail, it destroys the running
        // firmware mid-transfer - so the one check worth making twice is
        // that the destination is not where we are. A board that cannot say
        // where it is running from is refused too: "unknown" is not a
        // difference, and this is the wrong question to answer optimistically.
        match self.0.booted_slot() {
            Some(booted) if booted != slot => {}
            _ => return false,
        }
        let Some(Some(room)) = self.0.with_app(slot, |r| Some(r.partition_size())) else {
            return false;
        };
        if total as usize > room {
            return false;
        }
        self.0.target = Some(slot);
        true
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> bool {
        if self.0.target.is_none() {
            return false;
        }
        // The transfer delivers chunks strictly in order, so anything else
        // is a bug on this side rather than something to absorb.
        if offset != self.0.staged_at + self.0.staged_len as u32 {
            return false;
        }
        let mut rest = bytes;
        while !rest.is_empty() {
            let room = SECTOR - self.0.staged_len;
            let take = rest.len().min(room);
            let at = self.0.staged_len;
            self.0.staged[at..at + take].copy_from_slice(&rest[..take]);
            self.0.staged_len += take;
            rest = &rest[take..];
            if self.0.staged_len == SECTOR && !self.0.flush_staged() {
                self.0.target = None;
                return false;
            }
        }
        true
    }

    fn finish(&mut self) -> bool {
        let Some(slot) = self.0.target else {
            return false;
        };
        // The tail sector is short; the rest of it stays erased, which is
        // what the bootloader expects past the end of an image.
        if !self.0.flush_staged() {
            self.0.target = None;
            return false;
        }
        self.0.target = None;
        // Point the bootloader at the slot just written and mark it new, so
        // a bootloader built with rollback watches the first boot of it.
        // The order matters: activating first means a reset between the two
        // calls still lands on the new image rather than on the old one
        // with a state that describes the new.
        self.0
            .with_ota(|u| {
                // Re-derived, and checked against the slot the image
                // actually went into: activating the wrong one would hand
                // the bootloader a partition holding the previous image.
                match u.next_partition() {
                    Ok((_, next)) if next == slot => {}
                    _ => return false,
                }
                exclusive(|| {
                    if u.activate_next_partition().is_err() {
                        return false;
                    }
                    let _ = u.set_current_ota_state(OtaImageState::New);
                    true
                })
            })
            .unwrap_or(false)
    }

    fn cancel(&mut self) {
        // Nothing to undo. The slot holds a partial image, but it is the
        // one the bootloader is not pointed at, and the next attempt
        // overwrites it from the top.
        self.0.target = None;
        self.0.staged_len = 0;
        self.0.staged_at = 0;
    }
}

// ---------------------------------------------------------------------------
// The shared handle
// ---------------------------------------------------------------------------

/// The flash, once `main` has claimed it.
///
/// An async mutex rather than a critical section: an erase takes tens of
/// milliseconds and there are eighty of them in an update, which is far too
/// long to hold interrupts off - the BLE connection carrying the update
/// would be the first thing to drop. Every user is a task, so awaiting the
/// lock is free.
static FLASH: embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    Option<Flash>,
> = embassy_sync::mutex::Mutex::new(None);

/// Hand the flash over to the rest of the firmware. Call once, from `main`.
pub async fn install(flash: Flash) {
    *FLASH.lock().await = Some(flash);
}

/// Run `f` with the flash, or return `None` if it was never installed.
pub async fn with_flash<R>(f: impl FnOnce(&mut Flash) -> R) -> Option<R> {
    let mut guard = FLASH.lock().await;
    guard.as_mut().map(f)
}
