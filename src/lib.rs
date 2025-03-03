//! A Rust board support package (BSP) for the Teensy 4. Supports the Teensy 4.0 and
//! 4.1 boards.
//!
//! Peripherals are re-exported from the [`imxrt-hal`](imxrt_hal)
//! hardware abstraction layer. See the HAL's documentation for more information on creating
//! and using peripherals.
//!
//! # Runtime
//!
//! The `teensy4-bsp` use the [`cortex-m-rt`] crate to provide a runtime. Enable runtime support using
//! the `"rt"` feature. To properly link your program, you **must link with the `t4link.x` linker
//! script**.
//!
//! [`cortex-m-rt`]: https://crates.io/crates/cortex-m-rt
//!
//! The `teensy4-bsp` emits a `cortex-m-rt`-compatible linker script. The layout supports the i.MX RT's boot requirements
//! and advanced memory features. Specifically, the layout includes the image vector table (IVT) and boot data
//! that's necessary for i.MX RT processors. Additionally, the layout specifies the FlexSPI configuration block's (FCB)
//! FLASH location. Finally, the linker script places all instructions in ITCM, and all data in DTCM.
//!
//! The `teensy4-bsp` provides its own reset hander. The custom reset handler
//!
//! - initalizes TCM regions
//! - copies instructions into ITCM
//! - copies the vector table into DTCM, and sets VTOR
//! - set's the CCM's low-power setting
//!
//! The reset handler then calls the `cortex-m-rt` entrypoint to finish memory initialization and invoke your program's
//! `main()`.
//!
//! # Features
//!
//! The `teensy4-bsp` supports these features:
//!
//! | Flag            |         Description                                                   | Default? |
//! | --------------- | --------------------------------------------------------------------- | -------- |
//! | `"systick"`     | Registers a SYSTICK exception handler that's called every millisecond | ✓        |
//! | `"usb-logging"` | Adds support for logging over USB with the `log` crate                | ✓        |
//! | `"rt"`          | Adds runtime support using `cortex-m-rt`                              |          |
//! | `"rtic"`        | Adds support for using the BSP peripherals with RTIC                  |          |
//!
//! Notes:
//!
//! - Combining `"rtic"` with `"systick"` is not supported. RTIC may try to register a SYSTICK exception handler, which
//!   conflicts with the BSP's definition.
//!
//! Proper RTIC support requires that you disable the BSP's default features. You may combine `"rtic"` with
//! either `"rt"` and `"usb-logging"`.
//!
//! ```toml
//! [dependencies.teensy4-bsp]
//! default-features = false
//! features = ["rtic",  "rt"]
//! version = # ...
//! ```
//!
//! # Pins
//!
//! The BSP helps you convert all the i.MX RT processor pads into your Teensy 4's pins.
//! From these pins, you may construct peripherals and perform I/O. The Teensy 4.0 and 4.1
//! boards support many of the same pins; see the [`common`] module
//! for those similar pins. To construct Teensy 4.0 or 4.1 pins, see the
//! [`t40::into_pins`](t40::into_pins()) and [`t41::into_pins`](t41::into_pins())
//! for more information.
//!
//! # Examples
//!
//! Turn on a Teensy 4.0's LED:
//!
//! ```no_run
//! use cortex_m::asm::wfi;
//! use teensy4_bsp as bsp;
//!
//! use embedded_hal::digital::v2::OutputPin;
//!
//! let peripherals = bsp::Peripherals::take().unwrap();
//! let pins = bsp::t40::into_pins(peripherals.iomuxc);
//! let mut led = bsp::configure_led(pins.p13);
//!
//! loop {
//!     led.set_high().unwrap();
//!     wfi();
//! }
//! ```
//!
//! See more examples in the project's repository.

#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]

// Need to reference this so that it doesn't get stripped out
#[cfg(target_arch = "arm")]
extern crate teensy4_fcb;

pub use teensy4_pins::common;
pub use teensy4_pins::t40;
pub use teensy4_pins::t41;

#[cfg(all(target_arch = "arm", feature = "rt"))]
mod rt;
#[cfg(feature = "systick")]
mod systick;
#[cfg(feature = "usb-logging")]
#[cfg_attr(docsrs, doc(cfg(feature = "usb-logging")))]
pub mod usb;

#[cfg(feature = "systick")]
pub use systick::SysTick;

pub use hal::ral::interrupt;
// `rtic` expects these in the root.
#[doc(hidden)]
#[cfg(feature = "rtic")]
pub use hal::ral::{interrupt as Interrupt, NVIC_PRIO_BITS};

pub use hal::Peripherals;
pub use imxrt_hal as hal;

/// The LED
///
/// See [`configure_led`](configure_led()) to prepare the LED.
pub type Led = hal::gpio::GPIO<common::P13, hal::gpio::Output>;

/// Configure the board's LED
///
/// Returns a GPIO that's physically tied to the LED. Use the returned handle
/// to drive the LED.
pub fn configure_led(pad: common::P13) -> Led {
    let mut led = hal::gpio::GPIO::new(pad);
    led.set_fast(true);
    led.output()
}
