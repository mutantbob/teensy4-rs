//! Runtime support
//!
//! The useful bits are written in assembly. See `start.s` for the
//! reset handler implementation. The reset handler eventually calls
//! `t4_init()`, which finishes setup, and calls the `cortex-m-rt`
//! `Reset()` handler.
//!
//! Code that's in this module is running before `.data` is initialized,
//! and before `.bss` is zeroed. This code should only touch ARM and
//! peripheral memory.

pub use cortex_m_rt::*;

/// System entrypoint, invoked by reset handler
///
/// # Safety
///
/// The function is unsafe since it directly modifies registers, and invokes
/// other functions that do the same. It does not touch any memory that needs
/// to first be initialized by cortex-m-rt.
#[no_mangle]
unsafe extern "C" fn t4_init() {
    // Remain in 'run' when transitioning to low power mode.
    // Do not transition to wait mode.
    //
    // This addresses an edge case identified when reprogramming
    // the Teensy, and then immediately executing WFI.
    // See https://github.com/mciantyre/teensy4-rs/issues/76 for
    // more details.
    //
    // It's also useful for users who expect SYSTICK exceptions
    // to unblock WFI, which isn't the default behavior.
    //
    // Ideally, we could figure out an approach that keeps the processor
    // as close to the reset state as possible, while avoiding #76.
    // For now, we'll go with this, which simplifies some downstream
    // code.
    const CCM_CLPCR: *mut u32 = 0x400F_C054 as *mut _;
    CCM_CLPCR.write_volatile(CCM_CLPCR.read_volatile() & !0b11);

    extern "C" {
        fn Reset();
    }
    Reset(); // cortex-m-rt entrypoint
}
