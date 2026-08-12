//! Minimal SD card driver in SPI mode: init, capacity, single-block
//! read/write. SPI1 on PB3 (SCK) / PB4 (MISO) / PB5 (MOSI), chip select
//! on PA0. This board has no card-detect switch - presence is determined
//! by whether [`SdCard::init`] succeeds.
//!
//! The in-repo driver exists because stm32wlxx-hal 0.6 only implements
//! embedded-hal 0.2, which rules out the driver types in embedded-sdmmc
//! (eh 1.0) and older crates wanting `FullDuplex`. The FAT layer talks to
//! this driver through embedded-sdmmc's transport-agnostic `BlockDevice`
//! trait instead (see [`crate::sdlog`]).

use cortex_m::interrupt::CriticalSection;
use stm32wlxx_hal::{
    gpio::{pins, Output, OutputArgs, PinState},
    pac,
    spi::{BaudRate, Spi, Transfer, Write, MODE_0},
};

/// SD card errors.
#[derive(Debug, Clone, Copy)]
pub enum SdError {
    /// Card did not respond in time.
    Timeout,
    /// Unexpected R1/token response (value included).
    Response(u8),
    /// SPI bus error.
    Spi,
    /// Card not initialized (call [`SdCard::init`] first).
    NotInitialized,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdKind {
    /// SDSC v1, byte addressing.
    Sd1,
    /// SDSC v2, byte addressing.
    Sd2,
    /// SDHC/SDXC, block addressing.
    Sdhc,
}

type SdSpi = Spi<pac::SPI1, pins::B3, pins::B4, pins::B5>;

pub const SD_BLOCK_LEN: usize = 512;

pub struct SdCard {
    spi: SdSpi,
    cs: Output<pins::A0>,
    kind: Option<SdKind>,
    /// Card capacity in 512-byte blocks (from the CSD), once initialized.
    num_blocks: u32,
}

impl SdCard {
    pub fn new(
        spi1: pac::SPI1,
        sck: pins::B3,
        miso: pins::B4,
        mosi: pins::B5,
        cs_pin: pins::A0,
        rcc: &mut pac::RCC,
        cs: &CriticalSection,
    ) -> Self {
        // Cards must be initialized below 400 kHz; Div64 gives 250 kHz
        // at the 16 MHz core clock. init() raises it afterwards.
        let spi = Spi::new_spi1_full_duplex(spi1, (sck, miso, mosi), MODE_0, BaudRate::Div64, rcc, cs);
        const CS_ARGS: OutputArgs = OutputArgs {
            level: PinState::High,
            ..OutputArgs::new()
        };
        Self {
            spi,
            cs: Output::new(cs_pin, &CS_ARGS, cs),
            kind: None,
            num_blocks: 0,
        }
    }

    /// Card kind detected by the last successful [`init`](Self::init).
    pub fn kind(&self) -> Option<SdKind> {
        self.kind
    }

    /// Whether the card initialized successfully.
    pub fn ready(&self) -> bool {
        self.kind.is_some()
    }

    /// Capacity in 512-byte blocks (0 before init).
    pub fn num_blocks(&self) -> u32 {
        self.num_blocks
    }

    /// Forget the card (after an IO error, so the retry path re-inits).
    pub fn deinit(&mut self) {
        self.kind = None;
    }

    fn xfer(&mut self, byte: u8) -> Result<u8, SdError> {
        let mut buf = [byte];
        Transfer::transfer(&mut self.spi, &mut buf).map_err(|_| SdError::Spi)?;
        Ok(buf[0])
    }

    /// Send a command and return the R1 response.
    fn cmd(&mut self, cmd: u8, arg: u32) -> Result<u8, SdError> {
        // CRC is only checked for CMD0/CMD8 in SPI mode.
        let crc = match cmd {
            0 => 0x95,
            8 => 0x87,
            _ => 0x01,
        };
        let frame = [
            0x40 | cmd,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            crc,
        ];
        Write::write(&mut self.spi, &frame).map_err(|_| SdError::Spi)?;
        // R1 arrives within 8 bytes (bit 7 clear).
        for _ in 0..8 {
            let r = self.xfer(0xFF)?;
            if r & 0x80 == 0 {
                return Ok(r);
            }
        }
        Err(SdError::Timeout)
    }

    fn with_cs<T>(&mut self, f: impl FnOnce(&mut Self) -> Result<T, SdError>) -> Result<T, SdError> {
        self.cs.set_level_low();
        let result = f(self);
        self.cs.set_level_high();
        // One trailing clock byte releases the card's DO line.
        let _ = self.xfer(0xFF);
        result
    }

    fn wait_not_busy(&mut self, timeout_ms: u32) -> Result<(), SdError> {
        let start = crate::platform::millis();
        loop {
            if self.xfer(0xFF)? == 0xFF {
                return Ok(());
            }
            if crate::platform::millis().wrapping_sub(start) > timeout_ms {
                return Err(SdError::Timeout);
            }
        }
    }

    /// Wait for a data start token (0xFE), tolerating idle 0xFF fill.
    fn wait_start_token(&mut self, timeout_ms: u32) -> Result<(), SdError> {
        let start = crate::platform::millis();
        loop {
            let t = self.xfer(0xFF)?;
            if t == 0xFE {
                return Ok(());
            }
            if t != 0xFF {
                return Err(SdError::Response(t));
            }
            if crate::platform::millis().wrapping_sub(start) > timeout_ms {
                return Err(SdError::Timeout);
            }
        }
    }

    fn set_baud(&mut self, baud: BaudRate) {
        // The HAL exposes no baud setter; we own the peripheral inside
        // `self.spi`, so a direct CR1 update is exclusive.
        unsafe {
            let spi1 = &*pac::SPI1::PTR;
            spi1.cr1.modify(|_, w| w.spe().clear_bit());
            spi1.cr1.modify(|_, w| w.br().bits(baud as u8));
            spi1.cr1.modify(|_, w| w.spe().set_bit());
        }
    }

    /// Initialize the card (CMD0 / CMD8 / ACMD41 / CMD58 / CMD9 sequence).
    pub fn init(&mut self) -> Result<SdKind, SdError> {
        self.kind = None;
        self.set_baud(BaudRate::Div64);

        // At least 74 clocks with CS high to enter native mode.
        self.cs.set_level_high();
        for _ in 0..10 {
            self.xfer(0xFF)?;
        }

        let kind = self.with_cs(|sd| {
            // Software reset into idle state.
            let mut r1 = 0xFF;
            for _ in 0..32 {
                r1 = sd.cmd(0, 0)?;
                if r1 == 0x01 {
                    break;
                }
            }
            if r1 != 0x01 {
                return Err(SdError::Response(r1));
            }

            // Voltage check distinguishes v2 from v1 cards.
            let v2 = match sd.cmd(8, 0x1AA)? {
                0x01 => {
                    let mut r7 = [0u8; 4];
                    for b in &mut r7 {
                        *b = sd.xfer(0xFF)?;
                    }
                    if r7[3] != 0xAA {
                        return Err(SdError::Response(r7[3]));
                    }
                    true
                }
                _ => false, // illegal command: v1 card
            };

            // ACMD41 until the card leaves idle (up to 1 s). Bounded, but the
            // caller reaches here from the main loop's SD housekeeping, so it
            // shares that loop's watchdog interval with everything else.
            let hcs = if v2 { 0x4000_0000 } else { 0 };
            let start = crate::platform::millis();
            loop {
                crate::watchdog::feed_now();
                sd.cmd(55, 0)?;
                if sd.cmd(41, hcs)? == 0x00 {
                    break;
                }
                if crate::platform::millis().wrapping_sub(start) > 1_000 {
                    return Err(SdError::Timeout);
                }
            }

            if v2 {
                // Read OCR: CCS bit selects block addressing.
                if sd.cmd(58, 0)? != 0x00 {
                    return Err(SdError::Spi);
                }
                let mut ocr = [0u8; 4];
                for b in &mut ocr {
                    *b = sd.xfer(0xFF)?;
                }
                if ocr[0] & 0x40 != 0 {
                    return Ok(SdKind::Sdhc);
                }
            }
            // Byte-addressed cards: fix the block length at 512.
            let r1 = sd.cmd(16, SD_BLOCK_LEN as u32)?;
            if r1 != 0x00 {
                return Err(SdError::Response(r1));
            }
            Ok(if v2 { SdKind::Sd2 } else { SdKind::Sd1 })
        })?;

        // Read the CSD for the capacity (the FAT layer wants num_blocks).
        let csd = self.with_cs(|sd| {
            let r1 = sd.cmd(9, 0)?;
            if r1 != 0x00 {
                return Err(SdError::Response(r1));
            }
            sd.wait_start_token(200)?;
            let mut csd = [0u8; 16];
            for b in &mut csd {
                *b = sd.xfer(0xFF)?;
            }
            // Discard the 16-bit CRC.
            sd.xfer(0xFF)?;
            sd.xfer(0xFF)?;
            Ok(csd)
        })?;
        self.num_blocks = csd_capacity_blocks(&csd);

        self.set_baud(BaudRate::Div2); // 8 MHz for data transfers
        self.kind = Some(kind);
        Ok(kind)
    }

    fn block_addr(&self, lba: u32) -> Result<u32, SdError> {
        match self.kind {
            Some(SdKind::Sdhc) => Ok(lba),
            Some(_) => Ok(lba * SD_BLOCK_LEN as u32),
            None => Err(SdError::NotInitialized),
        }
    }

    /// Read one 512-byte block.
    pub fn read_block(&mut self, lba: u32, buf: &mut [u8; SD_BLOCK_LEN]) -> Result<(), SdError> {
        let addr = self.block_addr(lba)?;
        self.with_cs(|sd| {
            let r1 = sd.cmd(17, addr)?;
            if r1 != 0x00 {
                return Err(SdError::Response(r1));
            }
            sd.wait_start_token(200)?;
            buf.fill(0xFF);
            Transfer::transfer(&mut sd.spi, buf).map_err(|_| SdError::Spi)?;
            // Discard the 16-bit CRC.
            sd.xfer(0xFF)?;
            sd.xfer(0xFF)?;
            Ok(())
        })
    }

    /// Write one 512-byte block.
    pub fn write_block(&mut self, lba: u32, buf: &[u8; SD_BLOCK_LEN]) -> Result<(), SdError> {
        let addr = self.block_addr(lba)?;
        self.with_cs(|sd| {
            let r1 = sd.cmd(24, addr)?;
            if r1 != 0x00 {
                return Err(SdError::Response(r1));
            }
            sd.xfer(0xFF)?; // gap before the data token
            sd.xfer(0xFE)?; // start token
            Write::write(&mut sd.spi, buf).map_err(|_| SdError::Spi)?;
            // Dummy CRC.
            sd.xfer(0xFF)?;
            sd.xfer(0xFF)?;
            let resp = sd.xfer(0xFF)? & 0x1F;
            if resp != 0x05 {
                return Err(SdError::Response(resp));
            }
            sd.wait_not_busy(500)
        })
    }
}

/// Capacity in 512-byte blocks from a raw CSD register.
fn csd_capacity_blocks(csd: &[u8; 16]) -> u32 {
    match csd[0] >> 6 {
        1 => {
            // CSD v2 (SDHC/SDXC): C_SIZE is bits [69:48].
            let c_size = ((csd[7] as u32 & 0x3F) << 16) | ((csd[8] as u32) << 8) | csd[9] as u32;
            (c_size + 1) * 1024
        }
        _ => {
            // CSD v1: C_SIZE bits [73:62], C_SIZE_MULT bits [49:47],
            // READ_BL_LEN bits [83:80].
            let read_bl_len = (csd[5] & 0x0F) as u32;
            let c_size =
                (((csd[6] as u32 & 0x03) << 10) | ((csd[7] as u32) << 2) | (csd[8] as u32 >> 6)) + 1;
            let c_size_mult = ((csd[9] as u32 & 0x03) << 1) | (csd[10] as u32 >> 7);
            // blocks = c_size * 2^(mult+2) * 2^read_bl_len / 512
            (c_size << (c_size_mult + 2)) << read_bl_len >> 9
        }
    }
}
