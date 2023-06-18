#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio;
use embassy_rp::pio::Pio;
use embassy_time::{Duration, Timer};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0};
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};
use cyw43_pio::PioSpi;
use static_cell::*;

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static, PIN_23>, PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialise Peripherals
    let p = embassy_rp::init(Default::default());

    // Create LED
    let mut led = Output::new(p.PIN_2, Level::Low);

    let fw = include_bytes!("../wifi_firmware/43439A0.bin");
    let clm = include_bytes!("../wifi_firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);


    let state = make_static!(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(_spawner.spawn(wifi_task(runner)));

    // Loop
    loop {
        // Log
        info!("LED On!");

        // Turn LED On
        led.set_high();

        // Wait 100ms
        Timer::after(Duration::from_millis(100)).await;

        // Log
        info!("LED Off!");

        // Turn Led Off
        led.set_low();

        // Wait 100ms
        Timer::after(Duration::from_millis(100)).await;
    }
}
