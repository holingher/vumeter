#![no_std]
#![no_main]


extern crate panic_halt;
extern crate vumeter_lib as vu;

#[macro_use(block)]
extern crate nb;

// use hal::clock::GenericClockController;
// use hal::pac::{Peripherals};
// use hal::prelude::*;


use biquad::frequency::*;
// use boardlib;

use bsp::hal::ccm;
//use cortex_m::delay;
use rt::entry;
// use vu::protocol::{ParseState,PWM};
use vu::protocol as pkt;
use pkt::Pkt;
use vu::bandpass as bp;
use vu::audio_hw::*;
use vu::shared::*;
use teensy4_bsp as bsp;
use imxrt1062_pac;
use log::info;

use imxrt_rt as rt;
use teensy4_bsp::interrupt;
//use teensy4_bsp::LED;

use rt::interrupt;
use embedded_hal::serial::{Read,Write};
use embedded_hal::digital::v2::OutputPin;
//use embedded_hal::digital::v2::ToggleableOutputPin;
use core::time::Duration;

use boardlib::{spdif,uart,switch,timer};
use cortex_m::asm::wfi;

const PLL1_DIV_SEL: u32 = 100;
const ARM_DIVIDER: u32 = 2;
const AHB_DIVIDER: u32 = 1;
pub const ARM_FREQUENCY: u32 =
    ccm::analog::pll1::frequency(PLL1_DIV_SEL) / ARM_DIVIDER / AHB_DIVIDER;
    
#[interrupt]
fn SPDIF() {
    unsafe{spdif::spdif_isr()};
}

#[interrupt]
fn DMA3_DMA19() {
    unsafe{spdif::spdif_dma_isr()};
}

#[interrupt]
fn LPUART3() {
    unsafe{uart::uart_isr()};
}

#[interrupt]
fn PIT() {
    unsafe{timer::pit_isr()};
}

#[derive(Clone, Copy)]
struct DelayTicks(u32);
fn delay(_ticks: DelayTicks) {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::arch::asm! {
            r#"
                5:
                subs {ticks}, #1 @ ticks--; Z = (0 == ticks);
                bne 5b           @ if (0 == Z) goto 5;
            "#,
            ticks = inout(reg) _ticks.0 => _,
        };
    }
}

#[entry]
fn main() -> ! {
    // WARNING: Must leave this uncommented
    let mut peripherals = cortex_m::Peripherals::take().unwrap();
    let mut led = switch::Led::initialize().unwrap();
    led.set(true);
    let mut spdif = spdif::SPDIF::initialize().unwrap();
    let mut uart = uart::UART::initialize((460800. *  600. / 528.) as u32).unwrap();
    let mut switch = switch::Switch::initialize().unwrap();
    let mut pwr = switch::Pwr::initialize().unwrap();
    let mut sleep_pin = switch::SleepPin::initialize().unwrap();
    const DELAY: DelayTicks = DelayTicks(200);

    pwr.set(true);

    // Enabling logging increases current usage by about 10mA at 528MHz
    // peripherals.log.init(Default::default());
    delay(DELAY);
    
    // Switch from 528 MHz to 600 (increases power consumption not quite linearly)
    // Must add a fudge factor to baud rate and sample rate
    // For some reason this seems to be required to get UART to work. Oh well. Extra speed can't hurt I guess
 //   peripherals.ccm.pll1.set_arm_clock(bsp::hal::ccm::analog::pll1::frequency, &mut peripherals.ccm.handle, &mut peripherals.dcdc);
    const _: () = assert!(ARM_FREQUENCY == 600_000_000);
    // Set up some memory for signal processing
    const MAX_BANDS : usize = 32;
    let mut l = [bp::BandState::default(); MAX_BANDS];
    let mut r = [bp::BandState::default(); MAX_BANDS];
    let mut p = [0.; MAX_BANDS];
    let mut s = [0.; MAX_BANDS];
    let scratch = bp::Scratch::new(&mut l[..],&mut r[..],&mut p[..],&mut s[..]).unwrap();
    // let mut state = ConnectedState::new(AudioState::Disconnected{scratch});
    let mut state = DeviceState::new(scratch);

    // Buffer outgoing serial data
    let mut tx_buf : [u8;512] = [0;512];
    let mut tx_buf = RingBuf::new(&mut tx_buf);

    let mut timer = timer::Timer::initialize(25_000).unwrap(); // 25ms

    loop {
        // Flush major serial buffer to minor buffer until we run out of space in minor buffer
        while let Some(()) = tx_buf.with_front(|byte| uart.write(byte).ok()) {}
        // Check for hardware events
        let switch = switch.check().map(Event::Switch);
        let spdif = spdif.read().map(Event::Audio);
        let timer = timer.elapsed().map(Event::Timer);
        let uart = uart.read().ok().map(Event::Rx);
        
        if !timer.is_none() {

        }
        if switch.is_none() && spdif.is_none() && timer.is_none() && uart.is_none() {
            sleep_pin.set(true);
            wfi();
            sleep_pin.set(false);
        } else {
            for evt in switch.into_iter().chain(spdif.into_iter()).chain(timer.into_iter()).chain(uart.into_iter()) {
                state.step(evt,&mut tx_buf)
            }
        }
    }

}









