#![no_std]
#![no_main]

//extern crate alloc;

// Ensure we halt on panic.
use panic_halt as _;

use rp2040_hal as hal;

use core::{f64, fmt::Display};
//use embedded_alloc::LlffHeap as Heap;

use hal::{clocks::Clock, fugit::RateExtU32, gpio::PinState, pac};

// Embedded Hal traits
use embedded_hal::{delay::DelayNs, digital::OutputPin, spi::SpiDevice};
use embedded_hal_bus::spi::ExclusiveDevice;

#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

// Pi Pico frequency of 12Mhz.
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

//#[global_allocator]
//static HEAP: Heap = Heap::empty();

#[derive(thiserror::Error, Debug)]
enum Error<SD>
where
    SD: SpiDevice + ?Sized,
    <SD as embedded_hal::spi::ErrorType>::Error: Display,
{
    SpiDevice(#[source] <SD as embedded_hal::spi::ErrorType>::Error),
}

trait Calibrator
where
    Self: SpiDevice,
    <Self as embedded_hal::spi::ErrorType>::Error: Display,
{
    fn write_register(&mut self, addr: u16, value: u16) -> Result<(), Error<Self>>;

    fn get_angle(&mut self) -> Result<u16, Error<Self>>;

    fn set_zero_angle(&mut self) -> Result<(), Error<Self>>;

    fn store_regs_to_nvm(&mut self) -> Result<(), Error<Self>>;

    fn restore_regs_from_nvm(&mut self) -> Result<(), Error<Self>>;

    fn calibrate_air(&mut self) -> Result<(), Error<Self>>;
    fn calibrate_pro(&mut self) -> Result<(), Error<Self>>;
}

impl<SD> Calibrator for SD
where
    SD: SpiDevice,
    <SD as embedded_hal::spi::ErrorType>::Error: Display,
{
    // Writes a value to a register and verifies its content afterwards.
    #[inline]
    fn write_register(&mut self, addr: u16, value: u16) -> Result<(), Error<SD>> {
        //self.write(&[addr, value]).map_err(Error::SpiDevice)?;
        Ok(())
    }

    fn get_angle(&mut self) -> Result<u16, Error<Self>> {
        Ok(1)
    }

    fn set_zero_angle(&mut self) -> Result<(), Error<Self>> {
        Ok(())
    }

    fn store_regs_to_nvm(&mut self) -> Result<(), Error<Self>> {
        Ok(())
    }

    fn restore_regs_from_nvm(&mut self) -> Result<(), Error<Self>> {
        Ok(())
    }

    fn calibrate_air(&mut self) -> Result<(), Error<Self>> {
        self.write_register(10, 202)?;
        self.write_register(28, 0)?;
        self.write_register(29, 0)?;
        self.write_register(2, 144)?;
        self.write_register(3, 0)?;
        self.write_register(4, 232)?;
        self.write_register(5, 3)?;
        self.write_register(6, 61)?;
        self.write_register(7, 135)?;
        self.write_register(8, 59)?;
        self.write_register(9, 130)?;
        self.write_register(11, 246)?;
        self.write_register(12, 130)?;
        self.write_register(13, 0)?;
        self.write_register(14, 128)?;
        self.write_register(15, 201)?;
        self.write_register(16, 20)?;
        self.write_register(17, 1)?;
        self.write_register(18, 147)?;
        self.write_register(22, 1)?;
        self.write_register(30, 255)?;
        self.set_zero_angle()?;
        //TODO: Delay for 1ms
        self.store_regs_to_nvm()?;
        //TODO: Delay for 1ms
        self.restore_regs_from_nvm()?;
        self.write_register(10, 202)?;
        //TODO: Delay for 100ms
        self.get_angle()?;
        Ok(())
    }

    fn calibrate_pro(&mut self) -> Result<(), Error<Self>> {
        self.write_register(10, 0)?;
        self.write_register(28, 0)?;
        self.write_register(29, 0)?;
        self.write_register(2, 0)?;
        self.write_register(3, 0)?;
        self.write_register(4, 232)?;
        self.write_register(5, 3)?;
        self.write_register(6, 1)?;
        self.write_register(7, 135)?;
        self.write_register(8, 4)?;
        self.write_register(9, 0)?;
        self.write_register(11, 248)?;
        self.write_register(12, 130)?;
        self.write_register(13, 0)?;
        self.write_register(14, 128)?;
        self.write_register(15, 201)?;
        self.write_register(16, 20)?;
        self.write_register(17, 1)?;
        self.write_register(18, 147)?;
        self.write_register(22, 1)?;
        self.write_register(30, 255)?;
        self.set_zero_angle()?;
        //TODO: Delay for 1ms
        self.store_regs_to_nvm()?;
        //TODO: Delay for 1ms
        self.restore_regs_from_nvm()?;
        self.write_register(10, 202)?;
        //TODO: Delay for 100ms
        self.get_angle()?;
        Ok(())
    }
}

#[hal::entry]
fn main() -> ! {
    // Initialize the allocator BEFORE you use it
    //    {
    //        use core::mem::MaybeUninit;
    //        const HEAP_SIZE: usize = 1024;
    //        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    //        unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
    //    }

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

    // Creates SPI Device.
    let sd = ExclusiveDevice::new(spi_bus, spi_cs, timer).unwrap();

    loop {
        led.set_high().unwrap();
        timer.delay_ms(500);
        led.set_low().unwrap();
        timer.delay_ms(500);
    }
}
