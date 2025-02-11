//! SysTick-based time driver.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use core::{mem, ptr};
use critical_section::Mutex as CsMutex;

static SYSTICK_WAKER: CsMutex<RefCell<Option<core::task::Waker>>> =
    CsMutex::new(RefCell::new(None));

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::Driver;
use pac::systick::vals;
use qingke::interrupt::Priority;
use qingke_rt::interrupt;

use crate::pac;

pub const ALARM_COUNT: usize = 1;

struct AlarmState {
    timestamp: Cell<u64>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

pub struct SystickDriver {
    alarm_count: AtomicU8,
    alarms: Mutex<[AlarmState; ALARM_COUNT]>,
    period: AtomicU32,
}

const ALARM_STATE_NEW: AlarmState = AlarmState::new();
embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver = SystickDriver {
    period: AtomicU32::new(1), // avoid div by zero
    alarm_count: AtomicU8::new(0),
    alarms: Mutex::new([ALARM_STATE_NEW; ALARM_COUNT]),
});

impl SystickDriver {
    fn init(&'static self) {
        let rb = &crate::pac::SYSTICK;
        let hclk = crate::rcc::clocks().hclk.0 as u64;

        let cnt_per_second = hclk / 8; // HCLK/8
        let cnt_per_tick = cnt_per_second / embassy_time_driver::TICK_HZ;

        self.period.store(cnt_per_tick as u32, Ordering::Relaxed);

        // UNDOCUMENTED:  Avoid initial interrupt
        rb.cmp().write(|w| *w = u64::MAX - 1);
        rb.cmp().write_value(0);
        critical_section::with(|_| {
            rb.sr().write(|w| w.set_cntif(false)); // clear

            // Configration: Upcount, No reload, HCLK as clock source
            rb.ctlr().modify(|w| {
                //  w.set_init(true);
                w.set_mode(vals::Mode::UPCOUNT);
                w.set_stre(false);
                w.set_stclk(vals::Stclk::HCLK_DIV8);
                w.set_ste(true);
            });
        })
    }

    #[inline(always)]
    fn on_interrupt(&self) {
        let rb = &crate::pac::SYSTICK;
        rb.sr().write(|w| w.set_cntif(false)); // clear IF

        let period = self.period.load(Ordering::Relaxed) as u64;

        let next_timestamp = critical_section::with(|cs| {
            let next = self.alarms.borrow(cs)[0].timestamp.get();
            if next > self.now() + 1 {
                return next;
            }
            self.trigger_alarm(cs);
            return u64::MAX;
        });

        let new_cmp = u64::min(next_timestamp * period, self.raw_cnt().wrapping_add(period));
        rb.cmp().write_value(new_cmp + 1);
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        self.alarms.borrow(cs)[0].timestamp.set(u64::MAX);
        if let Some(w) = SYSTICK_WAKER.borrow(cs).take() {
            w.wake();
        }
    }


    #[inline]
    fn raw_cnt(&self) -> u64 {
        let rb = crate::pac::SYSTICK;
        rb.cnt().read()
    }
}

impl Driver for SystickDriver {
    fn now(&self) -> u64 {
        let rb = crate::pac::SYSTICK;
        let period = self.period.load(Ordering::Relaxed) as u64;
        rb.cnt().read() / period
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            // Store (a clone of) the provided waker in our module-local variable.
            SYSTICK_WAKER.borrow(cs).replace(Some(waker.clone()));
            // Use the alarm at index 0 (the only one)
            self.alarms.borrow(cs)[0].timestamp.set(at);
            let rb = &crate::pac::SYSTICK;
            // Ensure the SysTick interrupt is enabled
            rb.ctlr().modify(|w| w.set_stie(true));
            // Calculate the compare register value from the new target timestamp
            let period = self.period.load(Ordering::Relaxed) as u64;
            let t = self.raw_cnt();
            // Use the smaller of (at * period) or (t + period)
            let cmp_val = u64::min(at * period, t.wrapping_add(period));
            rb.cmp().write_value(cmp_val + 1);
        });
    }
}

#[interrupt(core)]
fn SysTick() {
    DRIVER.on_interrupt();
}

pub(crate) fn init() {
    DRIVER.init();
    use qingke_rt::CoreInterrupt;

    // enable interrupt
    unsafe {
        qingke::pfic::set_priority(CoreInterrupt::SysTick as u8, Priority::P15 as u8);
        qingke::pfic::enable_interrupt(CoreInterrupt::SysTick as u8);
    }
}
