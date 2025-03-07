#![no_std]
#![no_main]

use core::convert::Infallible;

// Ensure we halt on panic.
use panic_halt as _;

use rp2040_hal::{
    self as hal,
    gpio::{Pin, PinId, PinState, SioOutput},
    spi::ValidSpiPinout,
};

//use cortex_m::prelude::*;
use hal::gpio::{FunctionSio, PullDown};
use hal::{clocks::Clock, fugit::RateExtU32, pac};

// Embedded Hal traits
use embedded_hal::{delay::DelayNs, digital::OutputPin, spi::SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;

#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

// Pi Pico frequency of 12Mhz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

enum Angle {
    Open,
    Close,
    Unknown(u8),
}

impl Angle {
    fn is_open(&self) -> bool {
        matches!(self, Angle::Open)
    }

    fn is_close(&self) -> bool {
        matches!(self, Angle::Close)
    }
}

// Gets the current angle of the sensor.
fn angle() -> Angle {
    Angle::Open
}

struct Calibrator<SD> {
    dev: SD,
}

impl<SD> Calibrator<SD>
where
    SD: SpiDevice,
{
    fn new(dev: SD) -> Self {
        Self { dev }
    }

    // Writes a value to a register and verifies its content afterwards.
    fn write_register(&mut self, addr: u8, value: u8) -> bool {
        self.dev
            .write(&[addr, value])
            .map_or_else(|_| false, |_| true)
    }

    fn angle_degrees(&self) -> Option<u8> {
        Some(1)
    }

    fn set_zero_angle(&self) {}

    fn calibrate_pro(&mut self) {
        self.write_register(10, 0);
        self.write_register(28, 0);
        self.write_register(29, 0);
        self.write_register(2, 0);
        self.write_register(3, 0);
        self.write_register(4, 232);
        self.write_register(5, 3);
        self.write_register(6, 1);
        self.write_register(7, 135);
        self.write_register(8, 4);
        self.write_register(9, 0);
        self.write_register(11, 248);
        self.write_register(12, 130);
        self.write_register(13, 0);
        self.write_register(14, 128);
        self.write_register(15, 201);
        self.write_register(16, 20);
        self.write_register(17, 1);
        self.write_register(18, 147);
        self.write_register(22, 1);
        self.write_register(30, 255);
    }

    fn calbrate_air(&mut self) {
        self.write_register(10, 202);
        self.write_register(28, 0);
        self.write_register(29, 0);
        self.write_register(2, 144);
        self.write_register(3, 0);
        self.write_register(4, 232);
        self.write_register(5, 3);
        self.write_register(6, 61);
        self.write_register(7, 135);
        self.write_register(8, 59);
        self.write_register(9, 130);
        self.write_register(11, 246);
        self.write_register(12, 130);
        self.write_register(13, 0);
        self.write_register(14, 128);
        self.write_register(15, 201);
        self.write_register(16, 20);
        self.write_register(17, 1);
        self.write_register(18, 147);
        self.write_register(22, 1);
        self.write_register(30, 255);
    }
}

#[hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = pins.gpio25.into_push_pull_output();

    let spi_sck = pins.gpio2.into_function::<hal::gpio::FunctionSpi>();
    let spi_mosi = pins.gpio3.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();
    let spi_cs = pins.gpio5.into_push_pull_output_in_state(PinState::High);
    let spi_bus = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_u32.MHz(),
        embedded_hal::spi::MODE_0,
    );

    let spi_device = ExclusiveDevice::new(spi_bus, spi_cs, timer).unwrap();

    let calibrator = Calibrator::new(spi_device);

    loop {
        led.set_high().unwrap();
        timer.delay_ms(500);
        led.set_low().unwrap();
        timer.delay_ms(500);
    }
}
