//! What the hardware loop keeps about the receiver between passes.
//!
//! The driver in [`crate::gps`] speaks to the module; this is the loop's
//! side of it - the settings retry, the fix and presence reporting, the
//! self-wake detection - which was a dozen locals threaded through the
//! loop and is one value with a handful of methods here.

use embassy_time::Instant;
use gps_proto::packet::PositionPacket;
use midair_proto::radiocfg::GpsConfig;

use crate::gps::Gps;

/// How many times a settings push is retried before the loop stops asking.
const CFG_TRIES: u8 = 5;

/// Between settings pushes, ms.
const CFG_RETRY_MS: u64 = 2_000;

/// How long after boot a silent receiver is reported, ms.
const GRACE_MS: u64 = 5_000;

/// What one pass of the receiver produced for the rest of the loop.
#[derive(Clone, Copy, Debug, Default)]
pub struct Seen {
    /// A time mark fit to discipline the hop clock with: the time of day
    /// a fix reported and the local time it was parsed. Only from a pass
    /// that followed the previous one promptly, and only with a fix.
    pub time_mark: Option<(u32, u64)>,
    /// A position to publish and log, at most once a second.
    pub position: Option<PositionPacket>,
}

/// The loop's bookkeeping for the receiver.
pub struct GpsWatch {
    cfg_tries: u8,
    next_cfg_ms: u64,
    /// Last seen backup state, so a receiver that wakes on its own timer
    /// is noticed. `sleep_for` ends with nothing sent to the host.
    was_sleeping: bool,
    had_fix: bool,
    /// Whether a fix has ever held since boot, which is what separates a
    /// fix lost from one never acquired in the no-fix ping.
    ever_had_fix: bool,
    nmea_seen: bool,
    checked: bool,
    grace_until_ms: u64,
    next_pos_ms: u64,
}

impl GpsWatch {
    pub fn new(now_ms: u64) -> Self {
        Self {
            cfg_tries: 0,
            next_cfg_ms: now_ms,
            was_sleeping: false,
            had_fix: false,
            ever_had_fix: false,
            nmea_seen: false,
            checked: false,
            grace_until_ms: now_ms + GRACE_MS,
            next_pos_ms: now_ms,
        }
    }

    /// Whether a fix has held at any point since boot.
    pub fn ever_had_fix(&self) -> bool {
        self.ever_had_fix
    }

    /// The receiver was just raised or reconfigured: push the settings
    /// again from `at_ms`. Backup mode loses the RAM layer the settings
    /// live in, and a boot-time push can land before the receiver has
    /// finished starting.
    pub fn rearm(&mut self, at_ms: u64) {
        self.cfg_tries = 0;
        self.next_cfg_ms = at_ms;
        // Already accounted for; keep the self-wake detector from
        // reporting this one a second time.
        self.was_sleeping = false;
    }

    /// The receiver was just parked, or is assumed to be.
    pub fn parked(&mut self) {
        self.was_sleeping = true;
    }

    /// One pass over an awake receiver: drain its sentences, keep its
    /// settings pushed, report what changed.
    pub async fn pass(&mut self, gps: &mut Gps<'_>, cfg: &GpsConfig, now_ms: u64, late_pass: bool) -> Seen {
        let mut seen = Seen::default();
        gps.poll();
        // A timed backup ends on the module's own clock, so the only signal
        // is that sentences started again - the driver clears its own flag
        // on the first one. Re-arm the config retry here, because the
        // settings do not survive backup and the budget may already be
        // spent from an earlier attempt.
        if self.was_sleeping && !gps.sleeping {
            self.cfg_tries = 0;
            self.next_cfg_ms = now_ms;
            status_println!("gps: woke itself from backup, reconfiguring");
        }
        self.was_sleeping = gps.sleeping;
        let fix = gps.has_fix();
        if fix != self.had_fix {
            self.had_fix = fix;
            if fix {
                self.ever_had_fix = true;
                status_println!("gps fix acquired ({} sats)", gps.packet().sats);
            } else {
                status_println!("gps fix lost");
            }
        }
        if !self.nmea_seen && gps.present() {
            self.nmea_seen = true;
            status_println!("gps: NMEA up ({} bytes)", gps.rx_bytes());
        }
        // The hop clock's best reference. Taken whether or not there is a
        // fix, so a stale mark cannot be handed over later as if it were
        // fresh; used only with one, and only from a pass that followed
        // the previous one promptly - the mark's local time is when the
        // sentence was parsed, and after a long gap that is not when it
        // arrived. The next second brings another.
        if let Some(mark) = gps.take_time_mark()
            && fix
            && !late_pass
        {
            seen.time_mark = Some(mark);
        }
        // Settings retry. Two things leave the module running its own
        // defaults while the firmware reports the ones it asked for: a
        // boot-time push that landed before the receiver had finished
        // starting, and a wake from backup mode, which cuts power to the
        // receiver core and takes the whole RAM configuration layer with
        // it - including the four NMEA sentences this firmware silences
        // to fit 9600 baud. Driving the retry off `configured` rather than
        // off the first sentence covers both, since `wake` clears it.
        if !gps.configured
            && !gps.sleeping
            && gps.present()
            && self.cfg_tries < CFG_TRIES
            && now_ms >= self.next_cfg_ms
        {
            self.next_cfg_ms = now_ms + CFG_RETRY_MS;
            self.cfg_tries += 1;
            if gps.configure(cfg).await {
                status_println!("gps: settings applied");
            } else if self.cfg_tries == CFG_TRIES {
                status_println!("gps: settings still not accepted, giving up");
            }
        }
        // Not while the receiver is in a backup this firmware asked for:
        // silence is the request working, and reporting it as a wiring or
        // baud fault sends whoever is measuring the GPS off after a bug
        // that is not there.
        if !self.checked && !gps.sleeping && now_ms >= self.grace_until_ms {
            self.checked = true;
            if !gps.present() {
                if gps.rx_bytes() == 0 {
                    status_println!("gps: silent on UART1 (power/wiring?)");
                } else {
                    status_println!("gps: {} bytes but no NMEA (baud?)", gps.rx_bytes());
                }
            }
        }
        if gps.take_updated() && now_ms >= self.next_pos_ms {
            self.next_pos_ms = now_ms + 1_000;
            seen.position = Some(gps.packet());
        }
        seen
    }

    /// Push the settings and re-arm the retry, for a receiver that was
    /// just woken or whose settings just changed.
    pub async fn configure(&mut self, gps: &mut Gps<'_>, cfg: &GpsConfig) {
        if gps.sleeping {
            gps.wake().await;
            status_println!("gps: woken");
        } else {
            gps.configure(cfg).await;
        }
        self.rearm(Instant::now().as_millis());
    }
}
