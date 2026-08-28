//! The internal flash, and the two things that share it.
//!
//! One peripheral, two unrelated users - the settings mirror and the OTA
//! writer - so they live in one object rather than two that would each need
//! their own claim on it. Nothing here is on a hot path: the settings are
//! written when an app changes one, and an image only during an update.
//!
//! **Settings.** RTC RAM is lost when the cell goes flat, so a sleeping
//! board would come back with nothing configured and advertise at full
//! current until it died again. The same record is therefore mirrored into
//! the `nvs` partition, which RTC RAM then caches (see [`crate::settings`]);
//! only a cold boot reads flash, so the wake-check path stays free of it.
//! This claims the partition but does *not* use the ESP-IDF NVS key/value
//! format - it is one fixed record at the partition start, and nothing else
//! on this board reads the region.
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

use esp_bootloader_esp_idf::ota::{Ota, OtaImageState};
use esp_bootloader_esp_idf::ota_updater::OtaUpdater;
use esp_bootloader_esp_idf::partitions::{
    self, AppPartitionSubType, DataPartitionSubType, PartitionType,
};
use embedded_storage::{ReadStorage, Storage};
use esp_storage::FlashStorage;
use midair_proto::bulk::Sink;
use midair_proto::session::{Stored, RECORD_LEN};

/// Flash sector size: the unit an erase works in, and so the unit an image
/// is staged in.
const SECTOR: usize = FlashStorage::SECTOR_SIZE as usize;

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
}

impl Flash {
    /// Claim the flash. Call once.
    pub fn new(flash: esp_hal::peripherals::FLASH<'static>) -> Self {
        Self {
            // The S3 is dual core. Only core 0 runs here, so the default
            // strategy (fail while another core is up) would already pass -
            // but a future second core would turn every settings save into
            // a silent failure, and parking is what makes that safe.
            storage: FlashStorage::new(flash).multicore_auto_park(),
            table: [0; partitions::PARTITION_TABLE_MAX_LEN],
            staged: [0; SECTOR],
            staged_at: 0,
            staged_len: 0,
            target: None,
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
        // `Storage::write` erases and rewrites the whole sector around the
        // record, so there is no separate erase step to get wrong.
        self.with_nvs(|region| region.write(0, &rec).is_ok())
            .unwrap_or(false)
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
            ota.set_current_app_partition(booted).is_ok()
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
            u.set_current_ota_state(OtaImageState::Valid).is_ok()
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
                    region.write(at, &staged[..len]).is_ok()
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
                if u.activate_next_partition().is_err() {
                    return false;
                }
                let _ = u.set_current_ota_state(OtaImageState::New);
                true
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
