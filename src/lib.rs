//! Smartled drivers for the nrf52 family
//!
//! Currently offers an option using the PWM peripheral, and
//! it is possible in the future to also support the use of the
//! i2s peripheral

#![no_std]

#[cfg(feature = "52810")]
use nrf52810_hal as hal;

#[cfg(feature = "52832")]
use nrf52832_hal as hal;

#[cfg(feature = "52840")]
use nrf52840_hal as hal;

// pub mod i2s;
pub mod pwm;

pub use smart_leds_trait::RGB8;
