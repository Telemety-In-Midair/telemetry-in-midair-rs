//! SD card FAT logging and config storage.
//!
//! The card carries a normal FAT16/FAT32 filesystem (as formatted by a
//! phone or computer), with two files in the root directory:
//!
//! - `GPSLOG.CSV` - appended position log, one line per own/remote fix:
//!   `ms,src,lat_e7,lon_e7,alt_dm,speed_cms,course_cdeg,sats,fix,rssi`
//! - `RADIO.CFG` - the radio configuration (see midair-proto's radiocfg),
//!   read at boot and rewritten when a new config arrives.
//!
//! The card is fully optional: the logger buffers lines in RAM and drops
//! the oldest data when no card is present, and a card inserted after boot
//! (or an SPI error) is picked up by the retry path. Log lines are appended
//! in batches with an open/append/close cycle per flush, so the FAT
//! directory entry stays consistent on power loss (at most one flush
//! interval of data is lost).
//!
//! Ported from the WIO-E5 firmware, minus its 348-line SPI-mode SD driver:
//! that existed only because `stm32wlxx-hal` stopped at embedded-hal 0.2,
//! which ruled out `embedded-sdmmc`'s own driver. esp-hal implements
//! embedded-hal 1.0, so the upstream driver is usable here and the in-repo
//! one does not come across.

use embedded_sdmmc::{
    Mode, SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager,
};
use esp_hal::delay::Delay;
use esp_println::println;
use gps_proto::packet::{PositionPacket, FLAG_FIX};

pub const LOG_FILE: &str = "GPSLOG.CSV";
pub const CONFIG_FILE: &str = "RADIO.CFG";

const LOG_HEADER: &str = "ms,src,lat_e7,lon_e7,alt_dm,speed_cms,course_cdeg,sats,fix,rssi\n";

/// How often the pending buffer is written out.
const FLUSH_MS: u32 = 5_000;
/// How often a missing or failed card is retried.
const RETRY_MS: u32 = 60_000;
/// RAM buffer for lines waiting on a card.
const PENDING_LEN: usize = 1024;
/// Largest `RADIO.CFG` this firmware will read.
pub const CONFIG_MAX: usize = 1024;

/// The card has no clock and this board has no RTC worth trusting at mount
/// time, so every file gets one fixed timestamp. The log lines carry
/// milliseconds since boot, which is what actually orders them.
pub struct FixedTime;

impl TimeSource for FixedTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 55, // 2025
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// The concrete SPI device the card sits behind.
pub type CardSpi<'d> = embedded_hal_bus::spi::ExclusiveDevice<
    esp_hal::spi::master::Spi<'d, esp_hal::Blocking>,
    esp_hal::gpio::Output<'d>,
    Delay,
>;

type Card<'d> = SdCard<CardSpi<'d>, Delay>;
type Vm<'d> = VolumeManager<Card<'d>, FixedTime, 2, 2, 1>;

struct Mounted {
    volume: embedded_sdmmc::RawVolume,
    root: embedded_sdmmc::RawDirectory,
}

pub struct SdLog<'d> {
    vm: Vm<'d>,
    mounted: Option<Mounted>,
    pending: [u8; PENDING_LEN],
    pending_len: usize,
    header_needed: bool,
    next_flush_ms: u32,
    next_retry_ms: u32,
    /// Cleared by [`disable`](Self::disable) to shut the card down for good.
    enabled: bool,
}

impl<'d> SdLog<'d> {
    pub fn new(card: Card<'d>) -> Self {
        Self {
            vm: VolumeManager::new_with_limits(card, FixedTime, 0),
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
        self.vm.device().mark_card_uninit();
        self.next_retry_ms = now_ms.wrapping_add(RETRY_MS);
    }

    /// Try to init the card and mount the first FAT volume.
    fn try_mount(&mut self, now_ms: u32) {
        self.next_retry_ms = now_ms.wrapping_add(RETRY_MS);
        // Any command forces the init the upstream driver does lazily; a
        // missing card fails here rather than halfway through a mount.
        if self.vm.device().num_bytes().is_err() {
            return;
        }
        let volume = match self.vm.open_raw_volume(VolumeIdx(0)) {
            Ok(v) => v,
            Err(_) => {
                self.vm.device().mark_card_uninit();
                return;
            }
        };
        let root = match self.vm.open_root_dir(volume) {
            Ok(d) => d,
            Err(_) => {
                let _ = self.vm.close_volume(volume);
                self.vm.device().mark_card_uninit();
                return;
            }
        };
        // Write the CSV header if the log does not exist yet.
        self.header_needed = self.vm.find_directory_entry(root, LOG_FILE).is_err();
        self.mounted = Some(Mounted { volume, root });
        println!("SD: FAT volume mounted");
    }

    /// Periodic driver: mounts/retries the card and flushes the pending
    /// buffer.
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
                println!("SD: write failed, remounting");
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
    /// A file too big for `buf` is refused rather than truncated. Filling
    /// the buffer and reporting its length looks exactly like a short file
    /// to the caller, and the TOML parser accepts any prefix that happens
    /// to end on a line boundary - so a card holding a long config would be
    /// adopted as whatever fitted, silently, and reported as loaded. The
    /// reference `RADIO.example.toml` is the worst case: its first 1024
    /// bytes are all header comment, so it parses clean as *every* setting
    /// at its default.
    pub fn read_config(&mut self, buf: &mut [u8]) -> Option<usize> {
        if !self.enabled {
            return None;
        }
        let root = self.mounted.as_ref()?.root;
        let file = self
            .vm
            .open_file_in_dir(root, CONFIG_FILE, Mode::ReadOnly)
            .ok()?;
        let len = self.vm.file_length(file).ok()? as usize;
        let result = if len > buf.len() {
            println!("SD: {} is {} bytes, too big to read", CONFIG_FILE, len);
            None
        } else {
            self.vm.read(file, &mut buf[..len]).ok()
        };
        let _ = self.vm.close_file(file);
        result
    }

    /// Replace `RADIO.CFG` with `bytes`. Returns whether it landed.
    pub fn write_config(&mut self, now_ms: u32, bytes: &[u8]) -> bool {
        if !self.enabled {
            return false;
        }
        let Some(m) = &self.mounted else {
            return false;
        };
        let root = m.root;
        let ok = (|| -> Result<(), ()> {
            let file = self
                .vm
                .open_file_in_dir(root, CONFIG_FILE, Mode::ReadWriteCreateOrTruncate)
                .map_err(|_| ())?;
            let wrote = self.vm.write(file, bytes).is_ok();
            let closed = self.vm.close_file(file).is_ok();
            if wrote && closed { Ok(()) } else { Err(()) }
        })()
        .is_ok();
        if !ok {
            println!("SD: config write failed, remounting");
            self.unmount(now_ms);
        }
        ok
    }
}
