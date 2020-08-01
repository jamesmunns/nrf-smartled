#[allow(unused)]
use crate::hal;

use crate::hal::{
    gpio::{Level, Output, Pin, PushPull},
    pac,
};

use core::sync::atomic::{compiler_fence, Ordering};

use smart_leds_trait::{SmartLedsWrite, RGB8};

/// Fill a buffer with the DMA representation
///
/// The buffer must be a slice of 24 u16s or more.
pub fn fill_buf(color: &RGB8, buf: &mut [u16]) -> Result<(), ()> {
    if buf.len() < 24 {
        return Err(());
    }

    let red = color.r.reverse_bits();
    let green = color.g.reverse_bits();
    let blue = color.b.reverse_bits();

    for g in 0..8 {
        if ((green >> g) & 0b1) == 0b1 {
            buf[g] = 0x8000 | 13;
        } else {
            buf[g] = 0x8000 | 5;
        }
    }

    for r in 0..8 {
        if ((red >> r) & 0b1) == 0b1 {
            buf[8 + r] = 0x8000 | 13;
        } else {
            buf[8 + r] = 0x8000 | 5;
        }
    }

    for b in 0..8 {
        if ((blue >> b) & 0b1) == 0b1 {
            buf[16 + b] = 0x8000 | 13;
        } else {
            buf[16 + b] = 0x8000 | 5;
        }
    }

    Ok(())
}

/// A PWM peripheral driven Smartled driver
pub struct Pwm<T: sealed::Instance> {
    pwm: T,
    _gpio: Pin<Output<PushPull>>,
}

impl<T> Pwm<T>
where
    T: sealed::Instance,
{
    /// Create a new Smartled driver with a given pin and PWM engine
    pub fn new<Mode>(pwm: T, pin: Pin<Mode>) -> Pwm<T> {
        let pin = pin.into_push_pull_output(Level::Low);

        pwm.psel.out[0].write(|w| {
            #[cfg(feature = "52840")]
            match pin.port() {
                hal::gpio::Port::Port0 => w.port().clear_bit(),
                hal::gpio::Port::Port1 => w.port().set_bit(),
            };
            unsafe {
                w.pin().bits(pin.pin());
            }
            w.connect().connected()
        });

        pwm.enable.write(|w| w.enable().enabled());
        pwm.mode.write(|w| w.updown().up());
        pwm.prescaler.write(|w| w.prescaler().div_1());
        pwm.countertop.write(|w| unsafe { w.countertop().bits(20) });
        pwm.loop_.write(|w| w.cnt().disabled());
        pwm.decoder.write(|w| {
            w.load().common();
            w.mode().refresh_count()
        });
        pwm.seq0.refresh.write(|w| unsafe { w.bits(0) });
        pwm.seq0.enddelay.write(|w| unsafe { w.bits(0) });

        Pwm { pwm, _gpio: pin }
    }

    /// Start sending raw data
    ///
    /// NOTE: You should probably use Pwm::send_full_buf() instead.
    ///
    /// SAFETY: the contents of `buf` must live and be constant until Pwm::is_done_raw()
    /// returns true.
    pub unsafe fn start_send_raw(&mut self, buf: *const [u16]) -> Result<(), ()> {
        // TODO: Check maximum supported len?
        if (*buf).is_empty() {
            return Err(());
        }

        if (((*buf).as_ptr() as usize) < hal::target_constants::SRAM_LOWER)
            || (((*buf).as_ptr() as usize) > hal::target_constants::SRAM_UPPER)
        {
            return Err(());
        }

        compiler_fence(Ordering::SeqCst);

        self.pwm.seq0.ptr.write(|w| w.bits((*buf).as_ptr() as u32));
        self.pwm.seq0.cnt.write(|w| w.bits((*buf).len() as u32));

        self.pwm.events_seqend[0].write(|w| w.bits(0));
        self.pwm.tasks_seqstart[0].write(|w| w.bits(1));

        Ok(())
    }

    /// Is the raw send complete?
    ///
    /// Note: You probably shouldn't use this function unless you
    /// are also using Pwm::start_send_raw().
    pub fn is_done_raw(&self) -> bool {
        self.pwm.events_seqend[0].read().bits() == 0
    }

    /// Send a series of colors and a stop pattern, using a given scratch space
    ///
    /// NOTE: the size of `scratch` must be >= u16s_needed_slice(colors).
    ///
    /// NOTE: You can also use the SmartLedsWrite::write method to avoid the
    /// need for a scratch space (it uses its own)
    pub fn send_full_buf(&mut self, colors: &[RGB8], scratch: &mut [u16]) -> Result<(), ()> {
        if scratch.len() < u16s_needed_slice(colors) {
            return Err(());
        }

        for (color, buf) in colors.iter().zip(scratch.chunks_exact_mut(24)) {
            fill_buf(color, buf)?;
        }

        let start = colors.len() * 24;
        let end = start + 40;

        for by in &mut scratch[start..end] {
            *by = 0x8000;
        }

        // Safety: we block until the DMA transaction is complete
        unsafe {
            self.start_send_raw(&scratch[..end])?;
        }
        while !self.is_done_raw() {}

        Ok(())
    }
}

/// How many u16s are needed to send a given slice
///
/// This number includes space for necessary stop patterns
pub fn u16s_needed_slice(slice: &[RGB8]) -> usize {
    u16s_needed_ct(slice.len())
}

/// How many u16s are needed to send a given number of RGB8s
///
/// This number includes space for necessary stop patterns
pub const fn u16s_needed_ct(leds: usize) -> usize {
    leds * 24 + 40
}

impl<T> SmartLedsWrite for Pwm<T>
where
    T: sealed::Instance,
{
    type Error = ();
    type Color = RGB8;
    /// Write all the items of an iterator to a ws2812 strip
    fn write<Iter, I>(&mut self, mut iterator: Iter) -> Result<(), ()>
    where
        Iter: Iterator<Item = I>,
        I: Into<Self::Color>,
    {
        // Two buffers to ping-pong between
        let mut buf_a = [0u16; 24];
        let mut buf_b = [0u16; 24];

        let mut toggle = false;

        // Start by filling and starting buf_a
        if let Some(c) = iterator.next() {
            fill_buf(&c.into(), &mut buf_a)?;
            unsafe {
                self.start_send_raw(&mut buf_a)?;
            }
        } else {
            return Ok(());
        }

        // Begin ping-ponging
        for c in iterator {
            let buf = if toggle { &mut buf_a } else { &mut buf_b };
            toggle = !toggle;

            // Proactively fill the next buffer
            fill_buf(&c.into(), buf)?;

            // Wait for the last buffer to complete
            while !self.is_done_raw() {}

            // Begin sending the next buffer
            unsafe {
                self.start_send_raw(buf)?;
            }
        }

        let buf = if toggle {
            &mut buf_a[..20]
        } else {
            &mut buf_b[..20]
        };

        // Fill buffers with zeros to force a quiet period
        buf.iter_mut().for_each(|x| *x = 0x8000);

        // Finish sending last pixel
        while !self.is_done_raw() {}

        // Send 1/2 of reset period
        unsafe {
            self.start_send_raw(buf)?;
        }
        while !self.is_done_raw() {}

        // Send 2/2 of reset period
        unsafe {
            self.start_send_raw(buf)?;
        }
        while !self.is_done_raw() {}

        Ok(())
    }
}

// As of now, there is no mainline PWM driver. Implement the parts
// we need to support various configurations

mod sealed {
    use core::ops::Deref;
    pub trait Instance: Deref<Target = crate::hal::pac::pwm0::RegisterBlock> {}
}

impl sealed::Instance for pac::PWM0 {}

#[cfg(not(any(feature = "52810")))]
impl sealed::Instance for pac::PWM1 {}

#[cfg(not(any(feature = "52810")))]
impl sealed::Instance for pac::PWM2 {}

#[cfg(not(any(feature = "52810", feature = "52832")))]
impl sealed::Instance for pac::PWM3 {}
