#![no_std]
#![no_main]

use core::convert::Infallible;

// Ensure we halt on panic.
use panic_halt as _;

use rp2040_hal::{
    self as hal,
    gpio::{Pin, PinId, SioOutput},
    spi::{SpiDevice, ValidSpiPinout},
};

//use cortex_m::prelude::*;
use hal::gpio::{FunctionSio, PullDown};
use hal::{clocks::Clock, fugit::RateExtU32, pac};

// Embedded Hal traits
use embedded_hal::{delay::DelayNs, digital::OutputPin};

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

struct Calibrator<S: hal::spi::State, D: SpiDevice, P: ValidSpiPinout<D>, const DS: u8, I: PinId> {
    spi: hal::spi::Spi<S, D, P, DS>,
    cs_pin: Pin<I, FunctionSio<SioOutput>, PullDown>,
}

impl<S: hal::spi::State, D: SpiDevice, P: ValidSpiPinout<D>, const DS: u8, I: PinId>
    Calibrator<S, D, P, DS, I>
{
    fn new(
        spi: hal::spi::Spi<S, D, P, DS>,
        cs_pin: Pin<I, FunctionSio<SioOutput>, PullDown>,
    ) -> Self {
        Self { spi, cs_pin }
    }

    // Writes a value to a register and verifies its content afterwards.
    fn write_register(addr: u16, value: u16) -> bool {
        false
    }

    fn angle_degrees(&self) -> Option<u8> {
        Some(1)
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
    let cs_pin = pins.gpio5.into_push_pull_output();
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sck)).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_u32.MHz(),
        embedded_hal::spi::MODE_0,
    );

    let calibrator = Calibrator::new(spi, cs_pin);

    loop {
        led.set_high().unwrap();
        timer.delay_ms(500);
        led.set_low().unwrap();
        timer.delay_ms(500);
    }
}
