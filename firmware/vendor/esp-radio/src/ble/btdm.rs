use alloc::boxed::Box;
use core::ptr::{addr_of, addr_of_mut};

use esp_phy::{PhyController, PhyInitGuard};
use esp_sync::RawMutex;
use esp_wifi_sys::c_types::*;
use portable_atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};

use super::{Config, ReceivedPacket};
use crate::{
    binary::include::*,
    ble::{
        HCI_OUT_COLLECTOR,
        HciOutCollector,
        btdm::ble_os_adapter_chip_specific::{G_OSI_FUNCS, osi_funcs_s},
    },
    compat::common::str_from_c,
    hal::ram,
};

#[cfg_attr(esp32c3, path = "os_adapter_esp32c3_s3.rs")]
#[cfg_attr(esp32s3, path = "os_adapter_esp32c3_s3.rs")]
#[cfg_attr(esp32, path = "os_adapter_esp32.rs")]
pub(crate) mod ble_os_adapter_chip_specific;

static PACKET_SENT: AtomicBool = AtomicBool::new(true);

#[repr(C)]
struct VhciHostCallbacks {
    // callback used to notify that the host can
    // send packet to controller
    notify_host_send_available: extern "C" fn(),
    // callback used to notify that the
    // controller has a packet to send to
    // the host
    notify_host_recv: extern "C" fn(*mut u8, u16) -> i32,
}

unsafe extern "C" {
    fn btdm_osi_funcs_register(osi_funcs: *const osi_funcs_s) -> i32;
    fn btdm_controller_get_compile_version() -> *const c_char;

    #[cfg(any(esp32c3, esp32s3))]
    fn btdm_controller_init(config_opts: *const esp_bt_controller_config_t) -> i32;

    #[cfg(esp32)]
    fn btdm_controller_init(
        config_mask: u32,
        config_opts: *const esp_bt_controller_config_t,
    ) -> i32;

    fn btdm_controller_enable(mode: esp_bt_mode_t);

    fn API_vhci_host_check_send_available() -> bool;
    fn API_vhci_host_send_packet(data: *const u8, len: u16);
    fn API_vhci_host_register_callback(vhci_host_callbac: *const VhciHostCallbacks) -> i32;

    #[cfg(not(esp32))]
    fn coex_pti_v2();
}

static VHCI_HOST_CALLBACK: VhciHostCallbacks = VhciHostCallbacks {
    notify_host_send_available,
    notify_host_recv,
};

extern "C" fn notify_host_send_available() {
    trace!("notify_host_send_available");

    PACKET_SENT.store(true, Ordering::Relaxed);
}

extern "C" fn notify_host_recv(data: *mut u8, len: u16) -> i32 {
    trace!("notify_host_recv {:?} {}", data, len);

    let data = unsafe { core::slice::from_raw_parts(data, len as usize) };

    let packet = ReceivedPacket {
        data: Box::from(data),
    };

    super::BT_STATE.with(|state| state.rx_queue.push_back(packet));

    super::dump_packet_info(data);

    crate::ble::controller::hci_read_data_available();

    0
}

// This is fine, we're only accessing it inside a critical section (protected by INTERRUPT_LOCK).
static mut G_INTER_FLAGS: heapless::Vec<esp_sync::RestoreState, 10> = heapless::Vec::new();

static INTERRUPT_LOCK: RawMutex = RawMutex::new();

#[ram]
unsafe extern "C" fn interrupt_enable() {
    #[allow(static_mut_refs)]
    unsafe {
        let flags = unwrap!(
            G_INTER_FLAGS.pop(),
            "interrupt_enable called without prior interrupt_disable"
        );
        trace!("interrupt_enable {:?}", flags);
        INTERRUPT_LOCK.release(flags);
    }
}

#[ram]
unsafe extern "C" fn interrupt_disable() {
    trace!("interrupt_disable");
    #[allow(static_mut_refs)]
    unsafe {
        let flags = INTERRUPT_LOCK.acquire();
        unwrap!(
            G_INTER_FLAGS.push(flags),
            "interrupt_disable was called too many times"
        );
        trace!("interrupt_disable {:?}", flags);
    }
}

#[ram]
unsafe extern "C" fn task_yield() {
    crate::preempt::yield_task();
}

unsafe extern "C" fn task_yield_from_isr() {
    // This is not called because we never set xHigherPriorityTaskWoken = true in the `_from_isr`
    // functions. This should be revisited if a scheduler needs it.
    crate::preempt::yield_task_from_isr();
}

unsafe extern "C" fn mutex_create() -> *const () {
    todo!();
}

unsafe extern "C" fn mutex_delete(_mutex: *const ()) {
    todo!();
}

unsafe extern "C" fn mutex_lock(_mutex: *const ()) -> i32 {
    todo!();
}

unsafe extern "C" fn mutex_unlock(_mutex: *const ()) -> i32 {
    todo!();
}

unsafe extern "C" fn task_create(
    func: *mut crate::binary::c_types::c_void,
    name_ptr: *const c_char,
    stack_depth: u32,
    param: *mut crate::binary::c_types::c_void,
    prio: u32,
    handle: *mut crate::binary::c_types::c_void,
    core_id: u32,
) -> i32 {
    let name = unsafe { str_from_c(name_ptr) };
    trace!(
        "task_create {:?} {:?} {} {} {:?} {} {:?} {}",
        func, name_ptr, name, stack_depth, param, prio, handle, core_id
    );

    unsafe {
        let task_func = core::mem::transmute::<
            *mut crate::binary::c_types::c_void,
            extern "C" fn(*mut esp_wifi_sys::c_types::c_void),
        >(func);

        let task = crate::preempt::task_create(
            name,
            task_func,
            param,
            prio,
            if core_id < 2 { Some(core_id) } else { None },
            stack_depth as usize,
        );
        *(handle as *mut usize) = task as usize;
    }

    1
}

unsafe extern "C" fn task_delete(task: *mut ()) {
    trace!("task delete called for {:?}", task);

    unsafe {
        crate::preempt::schedule_task_deletion(task.cast());
    }
}

#[ram]
unsafe extern "C" fn is_in_isr() -> i32 {
    crate::is_interrupts_disabled() as i32
}

#[cfg(esp32)]
#[ram]
unsafe extern "C" fn cause_sw_intr_to_core(_core: i32, _intr_no: i32) -> i32 {
    trace!("cause_sw_intr_to_core {} {}", _core, _intr_no);
    unsafe { xtensa_lx_rt::xtensa_lx::interrupt::set(1 << _intr_no) };
    0
}

#[allow(unused)]
#[ram]
unsafe extern "C" fn srand(seed: u32) {
    debug!("!!!! unimplemented srand {}", seed);
}

#[allow(unused)]
#[ram]
unsafe extern "C" fn rand() -> i32 {
    trace!("rand");
    unsafe { crate::common_adapter::random() as i32 }
}

// -- BLE modem sleep -------------------------------------------------------
//
// LOCAL PATCH. Upstream leaves the controller's modem sleep unimplemented -
// every callback below was a `todo!()`, and nothing ever asked the
// controller to sleep, so its PHY stayed powered for as long as the
// connector lived. What follows is ESP-IDF's sequence and its callbacks,
// ported from `components/bt/controller/esp32c3/bt.c`, which is the file
// that serves the ESP32-S3 as well as the C3.
//
// With it the controller powers the PHY down between advertisements, and
// between the connection events of an idle connection, keeping time on the
// low power clock `Config::sleep_clock` picks. The host sees none of it,
// with one exception: a sleeping controller cannot take an HCI packet, so
// `send_hci` has to wake it and wait.
//
// Two parts of ESP-IDF's version are deliberately absent, because there is
// nothing here for them to talk to: the power management locks (this crate
// has neither DFS nor light sleep, which is also why
// `btdm_sleep_enter_phase1` has nothing to do) and MAC/BB power down, which
// needs deep sleep memory the crate never sets up.

/// `ESP_BT_SLEEP_MODE_1`, the only sleep mode the controller implements.
#[cfg(any(esp32c3, esp32s3))]
const BTDM_SLEEP_MODE_1: u8 = esp_bt_sleep_mode_t_ESP_BT_SLEEP_MODE_1 as u8;

// Low power clock sources as `btdm_lpclk_select_src` numbers them, which is
// not how `esp_bt_sleep_clock_t` numbers them in the controller config.
#[cfg(any(esp32c3, esp32s3))]
const BTDM_LPCLK_SEL_XTAL: u32 = 0;
#[cfg(any(esp32c3, esp32s3))]
const BTDM_LPCLK_SEL_XTAL32K: u32 = 1;
#[cfg(any(esp32c3, esp32s3))]
const BTDM_LPCLK_SEL_RTC_SLOW: u32 = 2;

/// Fractional bits in [`LP_CYCLE_US`] - ESP-IDF's `RTC_CLK_CAL_FRACT`.
#[cfg(any(esp32c3, esp32s3))]
const LP_CYCLE_US_FRACT_BITS: u32 = 19;

/// Shortest gap worth sleeping through, in half slots of 312.5 us.
#[cfg(any(esp32c3, esp32s3))]
const BTDM_MIN_SLEEP_DURATION: i32 = 24;

/// How far ahead of the next event to wake, in half slots, so that the PHY
/// is back up by the time it starts.
#[cfg(any(esp32c3, esp32s3))]
const BTDM_MODEM_WAKE_UP_DELAY: i32 = 8;

/// The controller task signal that carries the wake work.
#[cfg(any(esp32c3, esp32s3))]
const BTDM_VND_OL_SIG_WAKEUP_TMR: u32 = 0;

// Wakeup request sources. Only the two a task can raise are here; the
// controller's own coexistence path is `coex_bt_wakeup_request` in the
// chip-specific adapter.
#[cfg(any(esp32c3, esp32s3))]
const BTDM_ASYNC_WAKEUP_SRC_VHCI: i32 = 0;
#[cfg(any(esp32c3, esp32s3))]
const BTDM_ASYNC_WAKEUP_SRC_DISA: i32 = 2;

/// Whether the controller currently up was told to sleep.
#[cfg(any(esp32c3, esp32s3))]
static LP_ENABLED: AtomicBool = AtomicBool::new(false);

/// Length of one low power clock cycle in microseconds, as a fixed point
/// number with [`LP_CYCLE_US_FRACT_BITS`] fractional bits.
#[cfg(any(esp32c3, esp32s3))]
static LP_CYCLE_US: AtomicU32 = AtomicU32::new(0);

/// Whether the PHY reference this crate holds for BLE is currently taken.
///
/// ESP-IDF's `s_lp_stat.phy_enabled`. Written by the controller task as it
/// sleeps and wakes, and by `ble_init`/`ble_deinit` at the ends of the
/// controller's life.
#[cfg(any(esp32c3, esp32s3))]
static PHY_ENABLED: AtomicBool = AtomicBool::new(false);

/// Signalled by the controller task once a requested wake has happened.
#[cfg(any(esp32c3, esp32s3))]
static WAKEUP_REQ_SEM: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());

#[cfg(any(esp32c3, esp32s3))]
unsafe extern "C" {
    /// Arms the controller's sleep. Nothing sleeps until this is called
    /// with `true`, whatever the controller config asked for.
    fn btdm_controller_enable_sleep(enable: bool);
    fn btdm_controller_get_sleep_mode() -> u8;
    fn btdm_power_state_active() -> bool;
    fn btdm_wakeup_request();
    fn btdm_in_wakeup_requesting_set(in_wakeup_requesting: bool);
    /// Non-zero while the controller's sleep state is still settling.
    fn btdm_sleep_clock_sync() -> u8;
    fn btdm_lpclk_select_src(sel: u32) -> bool;
    fn btdm_lpclk_set_div(div: u32) -> bool;
    /// Background re-tracking of the BBPLL. ESP-IDF disables it on this
    /// controller whether or not it sleeps.
    fn sdk_config_extend_set_pll_track(enable: bool);
    /// Hands the controller task a function to run when a signal is posted
    /// to it. The wake path needs one because `btdm_wakeup_request` belongs
    /// to that task.
    fn btdm_vnd_offload_task_register(sig: u32, func: unsafe extern "C" fn(*mut c_void)) -> i32;
    fn btdm_vnd_offload_task_deregister(sig: u32) -> i32;
    fn r_btdm_vnd_offload_post(sig: u32, param: *mut c_void) -> i32;
}

/// Low power clock cycles to half microseconds.
///
/// `error_corr` carries the fraction this truncates into the next call, so
/// that a run of conversions does not drift; it is null when the caller
/// does not care.
#[cfg(any(esp32c3, esp32s3))]
#[ram]
unsafe extern "C" fn btdm_lpcycles_2_hus(cycles: u32, error_corr: *mut u32) -> u32 {
    let mut acc = if error_corr.is_null() {
        0u64
    } else {
        unsafe { *error_corr as u64 }
    };

    acc += LP_CYCLE_US.load(Ordering::Relaxed) as u64 * cycles as u64 * 2;
    let hus = acc >> LP_CYCLE_US_FRACT_BITS;
    acc -= hus << LP_CYCLE_US_FRACT_BITS;

    if !error_corr.is_null() {
        unsafe { *error_corr = acc as u32 };
    }

    hus as u32
}

/// Half microseconds to low power clock cycles.
#[cfg(any(esp32c3, esp32s3))]
#[ram]
unsafe extern "C" fn btdm_hus_2_lpcycles(hus: u32) -> u32 {
    let lpcycle_us = LP_CYCLE_US.load(Ordering::Relaxed) as u64;
    if lpcycle_us == 0 {
        // No clock selected, so the controller is either not up yet or
        // already torn down. Zero is a poor answer, but this runs with
        // interrupts disabled and a division by zero there is a worse one.
        return 0;
    }

    let cycles = ((hus as u64) << LP_CYCLE_US_FRACT_BITS) / lpcycle_us;
    (cycles >> 1) as u32
}

/// Decides whether a gap is worth sleeping through, and shortens the ones
/// that are so that the wake has time to bring the PHY back.
///
/// Called with interrupts disabled, so it stays in RAM and touches nothing
/// but its argument.
#[cfg(any(esp32c3, esp32s3))]
#[ram]
unsafe extern "C" fn btdm_sleep_check_duration(half_slot_cnt: *mut i32) -> bool {
    let half_slots = unsafe { *half_slot_cnt };
    if half_slots < BTDM_MIN_SLEEP_DURATION {
        return false;
    }

    unsafe { *half_slot_cnt = half_slots - BTDM_MODEM_WAKE_UP_DELAY };
    true
}

/// Called with interrupts disabled as the controller commits to a sleep.
#[cfg(any(esp32c3, esp32s3))]
unsafe extern "C" fn btdm_sleep_enter_phase1(_lpcycles: u32) {
    // ESP-IDF starts a timer here to reclaim its power management lock
    // before the controller wakes itself. There is no such lock under this
    // crate - no dynamic frequency scaling and no light sleep - so the
    // sleep needs nothing arranged in advance.
}

/// Called from the controller task once the sleep is committed.
#[cfg(any(esp32c3, esp32s3))]
unsafe extern "C" fn btdm_sleep_enter_phase2() {
    if unsafe { btdm_controller_get_sleep_mode() } != BTDM_SLEEP_MODE_1 {
        return;
    }

    // The reference dropped here is the one `ble_init` took, so the count
    // reaches zero and the PHY genuinely powers down - which is the whole
    // point. `esp-phy` backs the digital registers up on the way out, so
    // the wake below is a `phy_wakeup_init` rather than a recalibration.
    if PHY_ENABLED.swap(false, Ordering::Relaxed) {
        unsafe { esp_hal::peripherals::BT::steal() }.decrease_phy_ref_count();
    }
}

/// Called from the controller task after a wake, before anything is
/// transmitted or received.
#[cfg(any(esp32c3, esp32s3))]
unsafe extern "C" fn btdm_sleep_exit_phase3() {
    if unsafe { btdm_controller_get_sleep_mode() } == BTDM_SLEEP_MODE_1
        && !PHY_ENABLED.swap(true, Ordering::Relaxed)
    {
        // Balances phase 2. Forgotten rather than kept, because the guard
        // that has to survive until `ble_deinit` is the connector's, and
        // this one only exists to put the reference count back.
        core::mem::forget(unsafe { esp_hal::peripherals::BT::steal() }.enable_phy());
    }

    // The controller's sleep state trails the wake by a few microseconds.
    while unsafe { btdm_sleep_clock_sync() } != 0 {}
}

/// The wake itself, run in the controller task because that is where
/// `btdm_wakeup_request` belongs.
#[cfg(any(esp32c3, esp32s3))]
unsafe extern "C" fn btdm_sleep_exit_phase0(param: *mut c_void) {
    let event = param as usize as i32;
    if event == BTDM_ASYNC_WAKEUP_SRC_VHCI || event == BTDM_ASYNC_WAKEUP_SRC_DISA {
        unsafe { btdm_wakeup_request() };
        crate::compat::semaphore::sem_give(WAKEUP_REQ_SEM.load(Ordering::Relaxed));
    }
}

/// Wake the controller for a task that needs to talk to it, and hold it
/// awake until [`async_wakeup_request_end`].
///
/// Returns whether it had to be woken.
///
/// Blocks while it wakes, with no timeout, which is what ESP-IDF does: a
/// controller that answers half a wake is worse than one that answers none.
/// The cost of that choice is worth knowing - the caller here is usually
/// the executor's own task, so a controller that never runs its offload
/// signal stalls everything else on that executor, not just BLE.
#[cfg(any(esp32c3, esp32s3))]
fn async_wakeup_request(event: i32) -> bool {
    if !LP_ENABLED.load(Ordering::Relaxed) {
        return false;
    }

    unsafe { btdm_in_wakeup_requesting_set(true) };
    if unsafe { btdm_power_state_active() } {
        return false;
    }

    unsafe {
        r_btdm_vnd_offload_post(BTDM_VND_OL_SIG_WAKEUP_TMR, event as usize as *mut c_void);
    }
    crate::compat::semaphore::sem_take(
        WAKEUP_REQ_SEM.load(Ordering::Relaxed),
        crate::compat::OSI_FUNCS_TIME_BLOCKING,
    );

    true
}

/// Let the controller sleep again.
#[cfg(any(esp32c3, esp32s3))]
fn async_wakeup_request_end() {
    if LP_ENABLED.load(Ordering::Relaxed) {
        unsafe { btdm_in_wakeup_requesting_set(false) };
    }
}

/// Selects the low power clock and, when the controller is going to sleep,
/// gives it a way to be woken.
///
/// ESP-IDF's `btdm_low_power_mode_init`, and called where ESP-IDF calls it:
/// before `btdm_controller_init`, which reads the clock this leaves
/// selected.
#[cfg(any(esp32c3, esp32s3))]
fn low_power_mode_init(config: &Config) {
    use crate::{
        ble::btdm::ble_os_adapter_chip_specific::SleepClock,
        hal::clock::{Clock, RtcClock, RtcSlowClock},
    };

    let mut enabled = config.modem_sleep();

    // Selected whether or not the controller will sleep, as in ESP-IDF: the
    // divider and the cycle length are the units every duration the
    // controller quotes is measured in.
    let mut clock = if enabled {
        config.sleep_clock()
    } else {
        SleepClock::MainXtal
    };
    if clock == SleepClock::External32kXtal
        && !matches!(RtcClock::slow_freq(), RtcSlowClock::_32kXtal)
    {
        warn!("no 32.768 kHz crystal on the RTC slow clock - using the main crystal instead");
        clock = SleepClock::MainXtal;
    }

    let selected = match clock {
        SleepClock::MainXtal => {
            // Divided to 1 MHz, so a cycle is a microsecond.
            let selected = unsafe {
                btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL)
                    && btdm_lpclk_set_div(RtcClock::xtal_freq().mhz())
            };
            LP_CYCLE_US.store(1 << LP_CYCLE_US_FRACT_BITS, Ordering::Relaxed);
            selected
        }
        SleepClock::External32kXtal => {
            // Undivided, so a cycle is 1e6/32768 us. The shift is that
            // division, held in the same fixed point.
            let selected = unsafe {
                btdm_lpclk_select_src(BTDM_LPCLK_SEL_XTAL32K) && btdm_lpclk_set_div(0)
            };
            LP_CYCLE_US.store(1_000_000 << (LP_CYCLE_US_FRACT_BITS - 15), Ordering::Relaxed);
            selected
        }
    };

    // ESP-IDF asserts here. A controller that cannot be given a clock it can
    // count sleeps for the wrong length of time and misses events, which is
    // worse than not sleeping - and this is reached once per advertising
    // window on a duty-cycled board, where a panic costs the whole node. So
    // the sleep is dropped instead: nothing calls
    // `btdm_controller_enable_sleep`, and the controller stays awake.
    if !selected {
        warn!("BLE low power clock could not be selected - modem sleep is off");
        enabled = false;
    }

    if enabled {
        WAKEUP_REQ_SEM.store(
            crate::compat::semaphore::sem_create(1, 0),
            Ordering::Relaxed,
        );
        unsafe {
            btdm_vnd_offload_task_register(BTDM_VND_OL_SIG_WAKEUP_TMR, btdm_sleep_exit_phase0);
        }
    }

    LP_ENABLED.store(enabled, Ordering::Relaxed);
    debug!(
        "BLE modem sleep {}",
        if enabled { "enabled" } else { "disabled" }
    );
}

/// Whether the controller that is currently up is sleeping between events.
///
/// LOCAL PATCH, and the only way to answer that question from outside. It
/// asks both ends of the setup rather than repeating what the config said:
/// [`low_power_mode_init`] drops modem sleep if it cannot select a low
/// power clock, and the controller reports the mode it took from its own
/// config struct. Both have to agree, because either one alone is a way
/// for this to be half wired and look connected.
#[cfg(any(esp32c3, esp32s3))]
pub(crate) fn modem_sleep_active() -> bool {
    LP_ENABLED.load(Ordering::Relaxed)
        && unsafe { btdm_controller_get_sleep_mode() } == BTDM_SLEEP_MODE_1
}

/// Gives back everything [`low_power_mode_init`] took.
///
/// A duty-cycled board builds and drops the connector once per advertising
/// window - thousands of times a day - so a semaphore left behind here is a
/// leak that shows up as a shrinking heap rather than as a failure.
#[cfg(any(esp32c3, esp32s3))]
fn low_power_mode_deinit() {
    if LP_ENABLED.swap(false, Ordering::Relaxed) {
        unsafe { btdm_vnd_offload_task_deregister(BTDM_VND_OL_SIG_WAKEUP_TMR) };

        let sem = WAKEUP_REQ_SEM.swap(core::ptr::null_mut(), Ordering::Relaxed);
        if !sem.is_null() {
            crate::compat::semaphore::sem_delete(sem);
        }
    }

    unsafe {
        btdm_lpclk_select_src(BTDM_LPCLK_SEL_RTC_SLOW);
        btdm_lpclk_set_div(0);
    }
    LP_CYCLE_US.store(0, Ordering::Relaxed);
}

// -- The classic ESP32's controller, whose sleep is a different interface
// and is still not implemented here.

#[cfg(esp32)]
#[ram]
unsafe extern "C" fn btdm_lpcycles_2_hus(_cycles: u32, _error_corr: u32) -> u32 {
    todo!();
}

#[cfg(esp32)]
#[ram]
unsafe extern "C" fn btdm_hus_2_lpcycles(us: u32) -> u32 {
    const RTC_CLK_CAL_FRACT: u32 = 19;
    let g_btdm_lpcycle_us_frac = RTC_CLK_CAL_FRACT;
    let g_btdm_lpcycle_us = 2 << (g_btdm_lpcycle_us_frac);

    // Converts a duration in half us into a number of low power clock cycles.
    let cycles: u64 = (us as u64) << (g_btdm_lpcycle_us_frac as u64 / g_btdm_lpcycle_us as u64);
    trace!("btdm_hus_2_lpcycles {} {}", us, cycles);

    cycles as u32
}

#[cfg(esp32)]
unsafe extern "C" fn btdm_sleep_check_duration(_slot_cnt: i32) -> i32 {
    todo!();
}

#[cfg(esp32)]
unsafe extern "C" fn btdm_sleep_enter_phase1(_lpcycles: i32) {
    todo!();
}

#[cfg(esp32)]
unsafe extern "C" fn btdm_sleep_enter_phase2() {
    todo!();
}

#[cfg(esp32)]
unsafe extern "C" fn btdm_sleep_exit_phase1() {
    todo!();
}

#[cfg(esp32)]
unsafe extern "C" fn btdm_sleep_exit_phase2() {
    todo!();
}

#[cfg(esp32)]
unsafe extern "C" fn btdm_sleep_exit_phase3() {
    todo!();
}

unsafe extern "C" fn coex_schm_status_bit_set(_typ: i32, status: i32) {
    trace!("coex_schm_status_bit_set {} {}", _typ, status);
    #[cfg(coex)]
    unsafe {
        crate::binary::include::coex_schm_status_bit_set(_typ as u32, status as u32)
    };
}

unsafe extern "C" fn coex_schm_status_bit_clear(_typ: i32, status: i32) {
    trace!("coex_schm_status_bit_clear {} {}", _typ, status);
    #[cfg(coex)]
    unsafe {
        crate::binary::include::coex_schm_status_bit_clear(_typ as u32, status as u32)
    };
}

#[ram]
unsafe extern "C" fn read_efuse_mac(mac: *const ()) -> i32 {
    unsafe { crate::common_adapter::read_mac(mac as *mut _, 2) }
}

#[cfg(esp32)]
unsafe extern "C" fn set_isr13(n: i32, handler: unsafe extern "C" fn(), arg: *const ()) -> i32 {
    unsafe { ble_os_adapter_chip_specific::set_isr(n, handler, arg) }
}

#[cfg(esp32)]
unsafe extern "C" fn interrupt_l3_disable() {
    // info!("unimplemented interrupt_l3_disable");
}

#[cfg(esp32)]
unsafe extern "C" fn interrupt_l3_restore() {
    //  info!("unimplemented interrupt_l3_restore");
}

#[cfg(esp32)]
unsafe extern "C" fn custom_queue_create(
    _len: u32,
    _item_size: u32,
) -> *mut crate::binary::c_types::c_void {
    todo!();
}

pub(crate) fn ble_init(config: &Config) -> PhyInitGuard<'static> {
    let phy_init_guard;
    // LOCAL PATCH: a previous connector's queued packets must not reach
    // this one's host. See `super::reset_hci_state`.
    super::reset_hci_state();
    unsafe {
        (*addr_of_mut!(HCI_OUT_COLLECTOR)).write(HciOutCollector::new());
        // turn on logging
        #[allow(static_mut_refs)]
        #[cfg(feature = "sys-logs")]
        {
            unsafe extern "C" {
                static mut g_bt_plf_log_level: u32;
            }

            debug!("g_bt_plf_log_level = {}", g_bt_plf_log_level);
            g_bt_plf_log_level = 10;
        }

        // esp32_bt_controller_init
        ble_os_adapter_chip_specific::btdm_controller_mem_init();

        let mut cfg = ble_os_adapter_chip_specific::create_ble_config(config);

        let res = btdm_osi_funcs_register(addr_of!(G_OSI_FUNCS));
        assert!(res == 0, "btdm_osi_funcs_register returned {}", res);

        // LOCAL PATCH: the modem sleep setup, in ESP-IDF's order - the
        // controller reads the low power clock when it initializes.
        #[cfg(any(esp32c3, esp32s3))]
        low_power_mode_init(config);

        #[cfg(coex)]
        {
            let res = crate::wifi::coex_init();
            assert!(res == 0, "coex_init failed");
        }

        let version = btdm_controller_get_compile_version();
        let version_str = str_from_c(version);
        debug!("BT controller compile version {}", version_str);

        ble_os_adapter_chip_specific::bt_periph_module_enable();

        ble_os_adapter_chip_specific::disable_sleep_mode();

        #[cfg(any(esp32c3, esp32s3))]
        let res = btdm_controller_init(&mut cfg as *mut esp_bt_controller_config_t);

        #[cfg(esp32)]
        let res = btdm_controller_init(
            (1 << 3) | (1 << 4),
            &mut cfg as *mut esp_bt_controller_config_t,
        ); // see btdm_config_mask_load for mask

        assert!(res == 0, "btdm_controller_init returned {}", res);

        debug!("The btdm_controller_init was initialized");

        #[cfg(coex)]
        crate::binary::include::coex_enable();

        phy_init_guard = esp_hal::peripherals::BT::steal().enable_phy();
        // LOCAL PATCH: the reference modem sleep hands back and forth.
        #[cfg(any(esp32c3, esp32s3))]
        PHY_ENABLED.store(true, Ordering::Relaxed);

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                unsafe extern "C" {
                    fn btdm_rf_bb_init_phase2();
                }

                btdm_rf_bb_init_phase2();
                coex_bt_high_prio();
            } else {
                coex_pti_v2();
            }
        }

        #[cfg(coex)]
        coex_enable();

        // LOCAL PATCH: sleep is armed before the controller is enabled, as
        // in ESP-IDF. After this the controller may power the PHY down on
        // its own, through the callbacks above.
        #[cfg(any(esp32c3, esp32s3))]
        {
            // ESP-IDF turns the controller's PLL tracking off here, on
            // every C3 and S3 build - so the sleep this arms below has only
            // ever been shipped alongside it.
            sdk_config_extend_set_pll_track(false);

            if LP_ENABLED.load(Ordering::Relaxed) {
                btdm_controller_enable_sleep(true);
            }
        }

        btdm_controller_enable(esp_bt_mode_t_ESP_BT_MODE_BLE);

        API_vhci_host_register_callback(&VHCI_HOST_CALLBACK);
    }

    // At some point the "High-speed ADC" entropy source became available.
    unsafe { esp_hal::rng::TrngSource::increase_entropy_source_counter() };
    phy_init_guard
}

pub(crate) fn ble_deinit() {
    esp_hal::rng::TrngSource::decrease_entropy_source_counter(unsafe {
        esp_hal::Internal::conjure()
    });

    // LOCAL PATCH: hand nothing forward to the next connector, and give the
    // queued packets' allocations back now rather than at the next init.
    super::reset_hci_state();

    unsafe extern "C" {
        fn btdm_controller_deinit();
        // LOCAL PATCH: not declared upstream, because upstream never calls
        // it. ESP-IDF's teardown is esp_bt_controller_disable() and then
        // esp_bt_controller_deinit(), and the second is only valid from the
        // INITED state - deinit on a controller that is still ENABLED is
        // outside the state machine. `ble_init` ends with
        // btdm_controller_enable, so without this the controller is still
        // enabled here and the deinit does not bring it down.
        //
        // Measured on the Wio-S3: dropping BleConnector left the board at
        // 130 mA rather than the 55 the teardown was for.
        fn btdm_controller_disable();
    }

    unsafe {
        // LOCAL PATCH: a sleeping controller cannot be disabled, so wake it
        // and wait until it really is awake, as ESP-IDF does.
        #[cfg(any(esp32c3, esp32s3))]
        {
            async_wakeup_request(BTDM_ASYNC_WAKEUP_SRC_DISA);
            while !btdm_power_state_active() {}
        }

        btdm_controller_disable();

        #[cfg(any(esp32c3, esp32s3))]
        async_wakeup_request_end();

        btdm_controller_deinit();
    }

    // LOCAL PATCH: the connector drops its `PhyInitGuard` as soon as this
    // returns, so the reference that drop decrements has to be there. If
    // the controller had powered the PHY down, the reference it gave up is
    // that one, and this puts it back rather than letting the count go
    // negative.
    #[cfg(any(esp32c3, esp32s3))]
    if !PHY_ENABLED.swap(true, Ordering::Relaxed) {
        core::mem::forget(unsafe { esp_hal::peripherals::BT::steal() }.enable_phy());
    }

    #[cfg(any(esp32c3, esp32s3))]
    low_power_mode_deinit();

    // Disabling the PHY happens automatically, when the BLEController gets dropped.
}
/// Sends HCI data to the BLE controller.
#[instability::unstable]
pub fn send_hci(data: &[u8]) {
    let hci_out = unsafe { (*addr_of_mut!(HCI_OUT_COLLECTOR)).assume_init_mut() };
    hci_out.push(data);

    if hci_out.is_ready() {
        let packet = hci_out.packet();

        unsafe {
            // LOCAL PATCH: a sleeping controller takes no packets, and this
            // also keeps it awake until the matching call below - the
            // buffer it is handed has to outlive the send.
            #[cfg(any(esp32c3, esp32s3))]
            async_wakeup_request(BTDM_ASYNC_WAKEUP_SRC_VHCI);

            loop {
                let can_send = API_vhci_host_check_send_available();

                if !can_send {
                    trace!("can_send is false");
                    // LOCAL PATCH: yield rather than spin. What frees a
                    // controller buffer is the controller task, and this
                    // loop can otherwise deny it the CPU it needs to do
                    // that. With modem sleep the spin is worse than a hang:
                    // the wakeup request above is still held, so the modem
                    // stays powered for as long as it lasts.
                    crate::preempt::yield_task();
                    continue;
                }

                PACKET_SENT.store(false, Ordering::Relaxed);

                #[cfg(all(esp32, coex))]
                ble_os_adapter_chip_specific::async_wakeup_request(
                    ble_os_adapter_chip_specific::BTDM_ASYNC_WAKEUP_REQ_HCI,
                );

                API_vhci_host_send_packet(packet.as_ptr(), packet.len() as u16);

                #[cfg(all(esp32, coex))]
                ble_os_adapter_chip_specific::async_wakeup_request_end(
                    ble_os_adapter_chip_specific::BTDM_ASYNC_WAKEUP_REQ_HCI,
                );

                trace!("sent vhci host packet");

                super::dump_packet_info(packet);

                break;
            }

            // make sure the packet buffer doesn't get touched until sent
            while !PACKET_SENT.load(Ordering::Relaxed) {
                // LOCAL PATCH: as above - the notification that sets this
                // comes from the controller.
                crate::preempt::yield_task();
            }

            #[cfg(any(esp32c3, esp32s3))]
            async_wakeup_request_end();
        }

        hci_out.reset();
    }
}
