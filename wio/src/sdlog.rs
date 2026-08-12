//! SD card FAT logging and config storage.
//!
//! The card carries a normal FAT16/FAT32 filesystem (as formatted by a
//! phone or computer), with two files in the root directory:
//!
//! - `GPSLOG.CSV` - appended position log, one line per own/remote fix:
//!   `ms,src,lat_e7,lon_e7,alt_dm,speed_cms,course_cdeg,sats,fix,rssi`
//! - `RADIO.CFG` - the radio configuration (see midair-proto's radiocfg),
//!   read at boot and rewritten when a new config arrives over the link.
//!
//! The card is fully optional: the logger buffers lines in RAM and drops
//! the oldest data when no card is present, and a card inserted after
//! boot (or an SPI error) is picked up by the retry path. Log lines are
//! appended in batches with an open/append/close cycle per flush, so the
//! FAT directory entry stays consistent on power loss (at most one flush
//! interval of data is lost).

use core::cell::RefCell;

use embedded_sdmmc::{
    Block, BlockCount, BlockDevice, BlockIdx, Mode, RawDirectory, RawVolume, TimeSource,
    Timestamp, VolumeIdx, VolumeManager,
};
use gps_proto::packet::{PositionPacket, FLAG_FIX};

use crate::sdcard::{SdCard, SdError};

// Both names must be valid MS-DOS 8.3 - at most eight characters, then at
// most three of extension. The FAT layer converts a &str to a short name
// before it looks anything up, so an over-long name is not a failed lookup
// but a name that can never be opened or created at all.
pub const LOG_FILE: &str = "GPSLOG.CSV";
pub const CONFIG_FILE: &str = "RADIO.CFG";

const LOG_HEADER: &str = "ms,src,lat_e7,lon_e7,alt_dm,speed_cms,course_cdeg,sats,fix,rssi\n";

/// Flush the pending buffer to the card this often.
const FLUSH_MS: u32 = 5_000;
/// Retry card init this often while absent/failing. Each attempt clocks the
/// SPI bus and powers the card through its init sequence, so on a board with
/// no card fitted this is pure standby cost; a minute keeps hot-plug working
/// without paying for it every ten seconds forever.
const RETRY_MS: u32 = 60_000;
/// Pending line buffer; ~8 lines of headroom between flushes.
const PENDING_LEN: usize = 1024;
/// Largest config file we handle.
pub const CONFIG_MAX: usize = 1024;

/// [`BlockDevice`] adapter over the raw SPI driver. The trait takes
/// `&self`, hence the RefCell; all access happens from one RTIC task.
pub struct SdDev(pub RefCell<SdCard>);

impl BlockDevice for SdDev {
    type Error = SdError;

    fn read(&self, blocks: &mut [Block], start: BlockIdx, _reason: &str) -> Result<(), SdError> {
        let mut sd = self.0.borrow_mut();
        for (i, block) in blocks.iter_mut().enumerate() {
            // One FAT operation is many blocks, and a failing card spends the
            // full per-block timeout on each of them - enough of them in a row
            // to outlast the watchdog between two main-loop feeds. Each block
            // is individually bounded, so feeding per block keeps the deadline
            // meaningful while letting a slow card finish.
            crate::watchdog::feed_now();
            sd.read_block(start.0 + i as u32, &mut block.contents)?;
        }
        Ok(())
    }

    fn write(&self, blocks: &[Block], start: BlockIdx) -> Result<(), SdError> {
        let mut sd = self.0.borrow_mut();
        for (i, block) in blocks.iter().enumerate() {
            crate::watchdog::feed_now();
            sd.write_block(start.0 + i as u32, &block.contents)?;
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<BlockCount, SdError> {
        Ok(BlockCount(self.0.borrow().num_blocks()))
    }
}

/// Fixed timestamp for FAT directory entries - the GPS time of day is not
/// enough to build a calendar date, so files get a constant stamp.
struct FixedTime;

impl TimeSource for FixedTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56, // 2026
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

struct Mounted {
    volume: RawVolume,
    root: RawDirectory,
}

pub struct SdLog {
    vm: VolumeManager<SdDev, FixedTime, 2, 2, 1>,
    mounted: Option<Mounted>,
    pending: [u8; PENDING_LEN],
    pending_len: usize,
    header_needed: bool,
    next_flush_ms: u32,
    next_retry_ms: u32,
    /// Cleared by [`disable`](Self::disable) to shut the card down for good.
    enabled: bool,
}

impl SdLog {
    pub fn new(sd: SdCard) -> Self {
        Self {
            vm: VolumeManager::new_with_limits(SdDev(RefCell::new(sd)), FixedTime, 0),
            mounted: None,
            pending: [0; PENDING_LEN],
            pending_len: 0,
            header_needed: false,
            next_flush_ms: 0,
            next_retry_ms: 0,
            enabled: true,
        }
    }

    /// Shut the card down and stop touching it: unmounts, drops whatever is
    /// still buffered, and makes every later call a no-op.
    ///
    /// This is one-way by design. The config that asks for it lives on the
    /// card, so the card has to be mounted and read before the setting is
    /// even known - "disabled" therefore means "stop now", not "never
    /// started", and re-enabling it would mean a reboot anyway.
    pub fn disable(&mut self, now_ms: u32) {
        self.unmount(now_ms);
        self.pending_len = 0;
        self.enabled = false;
    }

    /// Whether a card is mounted and logging.
    pub fn ready(&self) -> bool {
        self.enabled && self.mounted.is_some()
    }

    /// Drop the mount and card state so the retry path starts over.
    fn unmount(&mut self, now_ms: u32) {
        if let Some(m) = self.mounted.take() {
            let _ = self.vm.close_dir(m.root);
            let _ = self.vm.close_volume(m.volume);
        }
        self.vm.device().0.borrow_mut().deinit();
        self.next_retry_ms = now_ms.wrapping_add(RETRY_MS);
    }

    /// Try to init the card and mount the first FAT volume.
    fn try_mount(&mut self, now_ms: u32) {
        self.next_retry_ms = now_ms.wrapping_add(RETRY_MS);
        if self.vm.device().0.borrow_mut().init().is_err() {
            return;
        }
        let volume = match self.vm.open_raw_volume(VolumeIdx(0)) {
            Ok(v) => v,
            Err(_) => {
                self.vm.device().0.borrow_mut().deinit();
                return;
            }
        };
        let root = match self.vm.open_root_dir(volume) {
            Ok(d) => d,
            Err(_) => {
                let _ = self.vm.close_volume(volume);
                self.vm.device().0.borrow_mut().deinit();
                return;
            }
        };
        // Write the CSV header if the log does not exist yet.
        self.header_needed = self.vm.find_directory_entry(root, LOG_FILE).is_err();
        self.mounted = Some(Mounted { volume, root });
        rtt_target::rprintln!("SD: FAT volume mounted");
    }

    /// Periodic driver: mounts/retries the card and flushes the pending
    /// buffer. Call from the main loop; feeds no watchdog itself.
    pub fn poll(&mut self, now_ms: u32) {
        if !self.enabled {
            return;
        }
        if self.mounted.is_none() {
            if now_ms.wrapping_sub(self.next_retry_ms) < 0x8000_0000 {
                self.try_mount(now_ms);
            }
            return;
        }
        if self.pending_len > 0
            && (self.pending_len > PENDING_LEN / 2
                || now_ms.wrapping_sub(self.next_flush_ms) < 0x8000_0000)
        {
            self.flush(now_ms);
        }
    }

    fn flush(&mut self, now_ms: u32) {
        self.next_flush_ms = now_ms.wrapping_add(FLUSH_MS);
        let Some(m) = &self.mounted else { return };
        let root = m.root;

        let result = (|| -> Result<(), ()> {
            let file = self
                .vm
                .open_file_in_dir(root, LOG_FILE, Mode::ReadWriteCreateOrAppend)
                .map_err(|_| ())?;
            let mut ok = true;
            if self.header_needed {
                ok &= self.vm.write(file, LOG_HEADER.as_bytes()).is_ok();
            }
            ok &= self.vm.write(file, &self.pending[..self.pending_len]).is_ok();
            // Close even if a write failed, then report.
            let closed = self.vm.close_file(file).is_ok();
            if ok && closed { Ok(()) } else { Err(()) }
        })();

        match result {
            Ok(()) => {
                self.header_needed = false;
                self.pending_len = 0;
            }
            Err(()) => {
                rtt_target::rprintln!("SD: write failed, remounting");
                self.unmount(now_ms);
            }
        }
    }

    /// Queue a position line. `src` 0 = local GPS; `rssi` is the LoRa RSSI
    /// for remote positions (0 for local).
    pub fn log_position(&mut self, now_ms: u32, src: u8, rssi: i16, p: &PositionPacket) {
        if !self.enabled {
            return;
        }
        use core::fmt::Write as _;
        struct Buf<'a>(&'a mut [u8], usize);
        impl core::fmt::Write for Buf<'_> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                let b = s.as_bytes();
                if self.1 + b.len() > self.0.len() {
                    return Err(core::fmt::Error);
                }
                self.0[self.1..self.1 + b.len()].copy_from_slice(b);
                self.1 += b.len();
                Ok(())
            }
        }

        let mut line = [0u8; 96];
        let mut w = Buf(&mut line, 0);
        let fix = (p.flags & FLAG_FIX != 0) as u8;
        if writeln!(
            w,
            "{},{},{},{},{},{},{},{},{},{}",
            now_ms, src, p.lat_e7, p.lon_e7, p.alt_dm, p.speed_cms, p.course_cdeg, p.sats, fix, rssi
        )
        .is_err()
        {
            return;
        }
        let len = w.1;
        if self.pending_len + len > PENDING_LEN {
            // Buffer full (no card for a while): drop the oldest half so
            // recent history survives until a card shows up.
            self.pending.copy_within(PENDING_LEN / 2..self.pending_len, 0);
            self.pending_len -= PENDING_LEN / 2;
        }
        self.pending[self.pending_len..self.pending_len + len].copy_from_slice(&line[..len]);
        self.pending_len += len;
    }

    /// Read `RADIO.CFG` into `buf`, returning the length read.
    ///
    /// A file too big for `buf` is refused rather than truncated. Filling the
    /// buffer and reporting its length looks exactly like a short file to the
    /// caller, and the TOML parser accepts any prefix that happens to end on a
    /// line boundary - so a card holding a long config would be adopted as
    /// whatever fitted, silently, and reported as loaded. The reference
    /// `RADIO.example.toml` is the worst case: its first 1024 bytes are all
    /// header comment, so it parses clean as *every* setting at its default.
    pub fn read_config(&mut self, buf: &mut [u8]) -> Option<usize> {
        let m = self.mounted.as_ref()?;
        let root = m.root;
        let file = self.vm.open_file_in_dir(root, CONFIG_FILE, Mode::ReadOnly).ok()?;
        let len = self.vm.file_length(file).ok();
        let n = match len {
            Some(len) if len as usize <= buf.len() => self.vm.read(file, buf).ok(),
            Some(len) => {
                rtt_target::rprintln!(
                    "SD: RADIO.CFG is {} bytes, over the {}-byte limit - ignored",
                    len,
                    buf.len()
                );
                None
            }
            None => None,
        };
        let _ = self.vm.close_file(file);
        n
    }

    /// Write (replace) `RADIO.CFG`. Returns `false` when no card is
    /// mounted or the write failed.
    pub fn write_config(&mut self, now_ms: u32, bytes: &[u8]) -> bool {
        if !self.enabled {
            return false;
        }
        let Some(m) = &self.mounted else { return false };
        let root = m.root;
        let ok = (|| -> Result<(), ()> {
            let file = self
                .vm
                .open_file_in_dir(root, CONFIG_FILE, Mode::ReadWriteCreateOrTruncate)
                .map_err(|_| ())?;
            let ok = self.vm.write(file, bytes).is_ok();
            let closed = self.vm.close_file(file).is_ok();
            if ok && closed { Ok(()) } else { Err(()) }
        })()
        .is_ok();
        if !ok {
            self.unmount(now_ms);
        }
        ok
    }
}
