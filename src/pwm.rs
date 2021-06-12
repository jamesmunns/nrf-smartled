//! Smartleds using the PWM peripheral

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
        pwm.seq1.refresh.write(|w| unsafe { w.bits(0) });
        pwm.seq1.enddelay.write(|w| unsafe { w.bits(0) });

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

    /// Set the seq[1] register's ptr and length
    ///
    /// SAFETY: the contents of `buf` must live and me constant until sequence 1
    /// is completed
    pub unsafe fn set_seq1_raw(&mut self, buf: *const [u16]) -> Result<(), ()> {
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

        self.pwm.seq1.ptr.write(|w| w.bits((*buf).as_ptr() as u32));
        self.pwm.seq1.cnt.write(|w| w.bits((*buf).len() as u32));

        Ok(())
    }

    /// Is the raw send complete?
    ///
    /// Note: You probably shouldn't use this function unless you
    /// are also using Pwm::start_send_raw().
    pub fn is_done_raw(&self) -> bool {
        self.pwm.events_seqend[0].read().bits() == 1
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

        // Disable looping, this is a one-shot
        self.pwm.loop_.write(|w| w.cnt().disabled());

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

        let mut blanks_fed = 0;
        let mut toggle = false;

        match (iterator.next(), iterator.next()) {
            (Some(a), Some(b)) => {
                // Two pixels, fill two buffers
                fill_buf(&a.into(), &mut buf_a)?;
                fill_buf(&b.into(), &mut buf_b)?;
            }
            (Some(a), None) => {
                // One pixel, fill the pixel and a blank
                fill_buf(&a.into(), &mut buf_a)?;
                buf_b.copy_from_slice(&[0x8000u16; 24]);
                blanks_fed = 1;
            }
            (None, Some(_)) => {
                // what? Intermittent iterator?
                return Err(());
            }
            _ => {
                // Empty iterator, nothing completed successfully
                return Ok(());
            }
        }

        unsafe {
            // Set the back half, and set + start the front half
            self.pwm.loop_.write(|w| w.cnt().bits(1));
            self.pwm.events_loopsdone.write(|w| w.bits(0));
            self.set_seq1_raw(&buf_b)?;
            self.start_send_raw(&buf_a)?;
        }

        #[derive(Copy, Clone)]
        enum Data {
            Pixel(RGB8),
            Blank,
        }

        // Create a new iterator that can contain pixels and blanks.
        // We include three blanks to ensure that we always have
        // enough to do a full A/B cycle, even if there are an
        // odd number of LEDs.
        //
        // TODO: In the future, we could have slightly more complex
        // code to skip the "extra" sequence by setting the loop
        // count to zero
        let new_iter = iterator
            .map(|seq| Data::Pixel(seq.into()))
            .chain([Data::Blank; 3].iter().cloned());

        // Begin filling the rest of the LEDs, or any remaining
        // blanks needed
        for seq in new_iter {
            if !toggle {
                // We're currently on the "A" side, about to start the "B" side,
                // and refill the "A" side data.

                // wait until seq_end[0] to refill the data
                while !self.is_done_raw() {}

                // refill seq[0] data
                match seq {
                    Data::Pixel(p) => {
                        fill_buf(&p, &mut buf_a)?;
                    }
                    Data::Blank => {
                        buf_a.copy_from_slice(&[0x8000u16; 24]);
                        blanks_fed += 1;
                    }
                }

                compiler_fence(Ordering::SeqCst);
            } else {
                // We're currently on the "B" side, waiting to restart the
                // sequence, then refill the "B" side data

                // If we are **here**, this means:
                // 0 blanks: We haven't filled ANY blanks, might still
                //   have more LEDs or not
                // 1 blanks: We've filled the A side with the first blank.
                //   we still need to start the seq and fill the B side
                //   with a blank.
                // 2 blanks: We've sent a blank in B before, and loaded
                //   one into the pending "A" side, but we still
                //   need to launch A to fire the last blank.
                // > 2 blanks: Shouldn't happen.

                // wait until the A+B loop is done
                while self.pwm.events_loopsdone.read().bits() == 0 {}

                // start seq[0] as quickly as possible
                unsafe {
                    self.pwm.tasks_seqstart[0].write(|w| w.bits(1));
                    self.pwm.events_seqend[0].write(|w| w.bits(0));
                    self.pwm.events_loopsdone.write(|w| w.bits(0));
                }

                compiler_fence(Ordering::SeqCst);

                // refill seq[1] data
                match seq {
                    Data::Pixel(p) => {
                        fill_buf(&p, &mut buf_b)?;
                    }
                    Data::Blank => {
                        buf_b.copy_from_slice(&[0x8000u16; 24]);
                        blanks_fed += 1;
                    }
                }
                compiler_fence(Ordering::SeqCst);

                // We've filled at least two blanks
                if blanks_fed >= 2 {
                    break;
                }
            }
            toggle = !toggle;
        }

        // Wait until the last loop is done
        while self.pwm.events_loopsdone.read().bits() == 0 {}

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
