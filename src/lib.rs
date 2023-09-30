#![allow(unused_imports)]
#![no_std]
#![no_main]
#![feature(const_maybe_uninit_zeroed)]
/**
Copyright (c) 2023 Todd Stellanova, All Rights Reserved
License: BSD3 (see LICENSE file)
*/

use core::mem::MaybeUninit;

// extern crate alloc;
// use core::sync::atomic::{AtomicPtr, Ordering};
//use core::cell::RefCell;
//use critical_section::Mutex;


use esp32s3_hal as hal;

// debugging
use esp_println::println;
use hal::{
    prelude::*,
    gpio::{self, Floating, GpioPin, Output, OutputPin, Input, PullUp, PushPull},
    Delay,
    clock::ClockControl,
    i2c::I2C,
    IO,
    peripherals::{Peripherals, I2C0, SPI2, TIMG0, UART0},
    timer::{Timer0, TimerGroup},
    Rtc,
    spi::{Spi, SpiMode, FullDuplexMode},
    Timer,
    Uart,
    uart::{TxRxPins, config::*},
};


use shared_bus::{BusManager, BusManagerSimple, NullMutex, I2cProxy, SpiProxy, XtensaMutex};


#[cfg(feature = "emdisplay")]
use embedded_graphics::{
    // mono_font::{ascii::{FONT_9X15_BOLD, FONT_8X13_BOLD},
    //             MonoFont, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Rectangle, PrimitiveStyleBuilder},
    // text::{Text, TextStyle},
};

// Provides the parallel port and display interface builders
#[cfg(feature = "emdisplay")]
use display_interface_spi::{SPIInterface, SPIInterfaceNoCS};

// Provides the Display builder
#[cfg(feature = "emdisplay")]
use mipidsi::{Builder, Display, models::ST7789};

#[cfg(feature = "sdcard")]
use embedded_sdmmc::SdCard;

// LoRa info
#[cfg(feature = "lorawan")]
use sx126x::{SX126x, conf::Config as LoRaConfig};



pub const LILYGO_KB_I2C_ADDRESS: u8 =     0x55;

type SharedBusMutex<T>  = XtensaMutex<T>; // was originally NullMutex for BusManagerSimple
type I2c0RawBusType<'a> = I2C<'a, I2C0>;
//type I2c0MutexBusType<'a> = SharedBusMutex<I2c0RawBusType<'a>>; 
//type I2c0BusType<'a> = BusManager<I2c0MutexBusType<'a>>;  
type I2c0ProxyType<'a> = I2cProxy<'a, SharedBusMutex<I2c0RawBusType<'a>>>;  


// Display dimensions
#[cfg(feature = "emdisplay")]
pub const DISPLAY_W: usize = 320;
#[cfg(feature = "emdisplay")]
pub const DISPLAY_H: usize = 240;
#[cfg(feature = "emdisplay")]
pub const DISPLAY_SIZE: Size = Size::new(DISPLAY_W as u32, DISPLAY_H as u32);


type Spi2RawBusType<'a> = Spi<'a, SPI2, FullDuplexMode>;
//type Spi2BusType<'a> = BusManager<SharedBusMutex<Spi2RawBusType<'a>>>; //Spi<'a, SPI2, FullDuplexMode>>>;
type Spi2ProxyType<'a> = SpiProxy<'a, SharedBusMutex<Spi2RawBusType<'a>>>; //NullMutex<Spi<'a, SPI2, FullDuplexMode>>>;
//type Spi2ProxyType<'a> = Spi<'a, SPI2, FullDuplexMode>;

#[cfg(feature = "sdcard")]
type SdCardType<'a> = SdCard<Spi2ProxyType<'a>, GpioPin<Output<PushPull>, 39>, Delay>;


#[cfg(feature = "lorawan")]
type LoraWrapperType<'a> = crate::lora_utils::LoraWrapper<'a>;





#[cfg(feature = "emdisplay")]
type DisplayType <'a> = Display<
    // SPIInterfaceNoCS<Spi2ProxyType<'a>, GpioPin<Output<esp32s3_hal::gpio::PushPull>, 11>>,
    SPIInterface<Spi2ProxyType<'a>,  GpioPin<Output<PushPull>, 11>, GpioPin<Output<PushPull>, 12>>,
    ST7789,
    GpioPin<Output<esp32s3_hal::gpio::PushPull>, 42>
>;


pub struct Board<'a> {
    // peripherals: Peripherals,
    pub timer0: Timer<Timer0<TIMG0>>,
    pub board_periph_pin: GpioPin<Output<PushPull>, 10>,
    pub delay: Delay,
    pub rtc: Rtc<'a>, // real time clock

    pub uart0: Uart<'a, UART0>,
    // pub spi2_bus: Spi2BusType<'a>,
    pub i2c0_proxy: I2c0ProxyType<'a>, //I2C<'a, I2C0>,

    // Trackball pins
    pub tball_click: GpioPin<Input<PullUp>, 0>,
    pub tball_up: GpioPin<Input<PullUp>, 3>,
    pub tball_right: GpioPin<Input<PullUp>, 2>,
    pub tball_down: GpioPin<Input<PullUp>, 15>,
    pub tball_left: GpioPin<Input<PullUp>, 1>,

    #[cfg(feature = "sdcard")]
    pub sdcard: Option<SdCardType<'a>>,

    #[cfg(feature = "lorawan")]
    pub lora: LoraWrapperType<'a>,

    #[cfg(feature = "emdisplay")]
    pub display: DisplayType<'a>,


//TODO add feature-flag-wrapped fields
}

impl Board<'static> {

    pub fn new() -> Board<'static> {
        let perphs = Peripherals::take();
        let mut system = perphs.SYSTEM.split();
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let timer_group0 = TimerGroup::new(
            perphs.TIMG0,
            &clocks,
            &mut system.peripheral_clock_control,
        );
        let mut wdt0 = timer_group0.wdt;
        let timer0 = timer_group0.timer0;

        let timer_group1 = TimerGroup::new(
            perphs.TIMG1,
            &clocks,
            &mut system.peripheral_clock_control,
        );
        let mut wdt1 = timer_group1.wdt;

        // Disable the RTC and TIMG watchdog timers
        let mut rtc = Rtc::new(perphs.RTC_CNTL);
        rtc.rwdt.disable();
        wdt0.disable();
        wdt1.disable();

        println!("Start setup\r\n\r\n");
        // init_heap();

        let io = IO::new(perphs.GPIO, perphs.IO_MUX);
        let mut delay = Delay::new(&clocks);

        let mut board_periph_pin = io.pins.gpio10.into_push_pull_output();
        board_periph_pin.set_high().unwrap();

        // Setup 100 kHz i2c0 bus
        let i2c0_raw = I2C::new(
            perphs.I2C0,
            io.pins.gpio18, //I2C0 SDA
            io.pins.gpio8, //I2C0 SCL
            100u32.kHz(),
            &mut system.peripheral_clock_control,
            &clocks,
        );
         
        let i2c0_bus: &'static _ = shared_bus::new_xtensa!(I2c0RawBusType = i2c0_raw).unwrap();
        let i2c0_proxy = i2c0_bus.acquire_i2c();

        // TODO setup GT911 capactive touch driver on i2c0 :
        // const GT911_ADDRESS1:u8 =  0x5d;
        // const GT911_ADDRESS2:u8 =  0x14;
        // TODO setup es7210_adc audio input on i2c0 / i2s ?

        let uart_config = Config {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            parity: Parity::ParityNone,
            stop_bits: StopBits::STOP1,
        };

        let uart_pins = TxRxPins::new_tx_rx(
            io.pins.gpio43.into_push_pull_output(), //GPIO 43 U0TXD
            io.pins.gpio44.into_floating_input(), //GPIO 44 U0RXD
        );

        let uart0 = Uart::new_with_config(
            perphs.UART0,
            Some(uart_config),
            Some(uart_pins),
            &clocks,
            &mut system.peripheral_clock_control,
        );

        // Interrupt pins
        // let tdeck_touch_int = io.pins.gpio16; //ESP_TP_INT
        // let tdeck_kb_int = io.pins.gpio46;

        //T-Deck trackball (mouse) pins, reading AN48841
        let tdeck_track_click = io.pins.gpio0.into_pull_up_input();
        let tdeck_track_up = io.pins.gpio3.into_pull_up_input(); // G01  GS1
        let tdeck_track_down = io.pins.gpio15.into_pull_up_input(); //  GS3
        let tdeck_track_right = io.pins.gpio2.into_pull_up_input(); //  GS2
        let tdeck_track_left = io.pins.gpio1.into_pull_up_input(); //   GS4
        //TODO setup trackball interrupt handling on falling edge

        // Note that AW9364 LED driver is used to drive TFT backlight

        // SPI chip select pins
        let mut tdeck_tft_cs = io.pins.gpio12.into_push_pull_output();
        let mut tdeck_lora_cs = io.pins.gpio9.into_push_pull_output();
        let mut tdeck_sdcard_cs = io.pins.gpio39.into_push_pull_output();

        // set all SPI CS pins high initially (this disables them)
        tdeck_tft_cs.set_high().unwrap();
        tdeck_sdcard_cs.set_high().unwrap();
        tdeck_lora_cs.set_high().unwrap();

        let spi2_raw = Spi::new_no_cs(
            perphs.SPI2,
            io.pins.gpio40, //tdeck_sclk,
            io.pins.gpio41, //tdeck_mosi,
            io.pins.gpio38, //tdeck_miso,
            // tdeck_tft_cs, // TODO I guess this is a default CS for the bus?
            60u32.MHz(),
            SpiMode::Mode0,
            &mut system.peripheral_clock_control,
            &clocks,
        );
        //let spi2_bus = spi2_raw;
        // TODO create a shared_bus so we can share SPI among multiple devices
        //let spi2_bus = make_static!(shared_bus::BusManagerSimple::new(spi2_raw));
        let spi2_bus: &'static _ = shared_bus::new_xtensa!(Spi2RawBusType = spi2_raw).unwrap();


        // TODO setup audio output on   ESP_I2S_BCK, ESP_I2S_WS, ESP_I2S_DOUT;
        // TODO setup ES7210 (analog voice ADC from mic) on i2c0_bus? address ES7210_AD1_AD0_00 = 0x40,

        #[cfg(feature = "sdcard")]
        let sdcard_local = {
           let sdcard = embedded_sdmmc::SdCard::new(spi2_bus.acquire_spi(), tdeck_sdcard_cs, delay);
           println!("sdcard {} bytes", sdcard.num_bytes().unwrap());
           sdcard
        };

        // TODO radio setup for lorawan
        #[cfg(feature = "lorawan")]
        let lora =  {
            // #define RADIO_FREQ          433.0
            // SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

            let mut lora_nreset = io.pins.gpio17.into_push_pull_output();
            lora_nreset.set_high().unwrap();
            // let lora_nss = io.pins.gpio9.into_push_pull_output_with_state(State::High);
            let lora_busy = io.pins.gpio13.into_floating_input();
            let lora_fake_ant = io.pins.gpio35.into_push_pull_output(); //No Connection?

            // TODO Configure DIO1 pin as interrupt source and store a reference to it in static
            let lora_dio1 = io.pins.gpio45.into_floating_input();
            // lora_dio1.make_interrupt_source(&mut afio);
            // lora_dio1.trigger_on_edge(&exti, Edge::RISING);
            // lora_dio1.enable_interrupt(&exti);

            let lora_pins = (
                tdeck_lora_cs, //RADIO_CS_PIN, // 9
                lora_nreset, //RADIO_RST_PIN, // 17
                lora_busy, //RADIO_BUSY_PIN, // 13
                lora_fake_ant, // lora_ant ???? no antenna switch on tdeck
                lora_dio1, // RADIO_DIO1_PIN, // 45
                );

            let mut wrappo = crate::lora_utils::LoraWrapper {
                radio: SX126x::new(lora_pins),
                spi_holder: spi2_bus.acquire_spi()
            };
            let conf = lora_utils::build_lora_config();
            wrappo.radio.init(&mut wrappo.spi_holder, &mut delay, conf).unwrap();
            wrappo
        };

        // Setup TFT display
        #[cfg(feature = "emdisplay")]
        let gfx_display = {
            let tdeck_tft_dc = io.pins.gpio11.into_push_pull_output();
            let mut tft_enable_pin =  io.pins.gpio42.into_push_pull_output();//enables backlight?
            tft_enable_pin.set_high().unwrap();
            let spi2_proxy = spi2_bus.acquire_spi();

            // let di = SPIInterfaceNoCS::new(spi2_bus, tdeck_tft_dc);
            let di = SPIInterface::new(spi2_proxy, tdeck_tft_dc, tdeck_tft_cs);
            Builder::st7789(di)
            .with_display_size(DISPLAY_H as u16, DISPLAY_W as u16, )
            .with_orientation(mipidsi::Orientation::Landscape(true))
            .with_invert_colors(mipidsi::ColorInversion::Inverted)
            .with_framebuffer_size(DISPLAY_H as u16, DISPLAY_W as u16, ) //remember this is rotated
            .init(&mut delay, Some(tft_enable_pin))
            .unwrap()
        };

        Board {
            timer0: timer0,
            board_periph_pin: board_periph_pin,
            delay,
            rtc,
            uart0: uart0,
            i2c0_proxy: i2c0_proxy,
            tball_click: tdeck_track_click,
            tball_up: tdeck_track_up,
            tball_right: tdeck_track_right,
            tball_down: tdeck_track_down,
            tball_left: tdeck_track_left,
            #[cfg(feature = "emdisplay")]
            display: gfx_display,
            #[cfg(feature = "lorawan")]
            lora,
            #[cfg(feature = "sdcard")]
            sdcard: Some(sdcard_local),
        }


    }
//TODO setup i2s audio output
// fn setup_audio_output() {
    // I2S audio output on MAX98357A
    // let tdeck_i2s_bck = io.pins.gpio7; //ESP_I2S_BCK
    // let tdeck_i2s_ws = io.pins.gpio5; //ESP_I2S_WS
    // let tdeck_i2c_da = io.pins.gpio6; //ESP_I2S_DA (DOUT)

//     let i2s = I2s::new(
//         peripherals.I2S0,
//         MclkPin::new(io.pins.gpio4),
//         Standard::Philips,
//         DataFormat::Data16Channel16,
//         44100u32.Hz(),
//         dma_channel.configure(
//             false,
//             &mut tx_descriptors,
//             &mut rx_descriptors,
//             DmaPriority::Priority0,
//         ),
//         &mut system.peripheral_clock_control,
//         &clocks,
//     );
// }
}



//TODO move to sd card library file
#[cfg(feature = "sdcard")]
pub mod sdcard_utils {
    use embedded_sdmmc::{File, TimeSource, SdCard, Timestamp, BlockDevice, VolumeManager, Volume};
    use crate::{Delay, SdCardType};

    pub type FileContextType<'a> = FileContextWrapper<SdCardType<'a>, FixedTimeSource>;

    pub struct FixedTimeSource {
        pub base_timestamp: Timestamp,
    }

    impl FixedTimeSource {
        // Timestamp { year_since_1970: 0, zero_indexed_month: 0, zero_indexed_day: 0, hours: 0, minutes: 0, seconds: 0 };
        fn new_with_time_ms(time_ms: u64) -> Self {
            let seconds = time_ms / 1000;
            let minutes = seconds / 60;
            let hours =  minutes / 60;
            return FixedTimeSource {
                base_timestamp: Timestamp {
                    year_since_1970: 53,
                    zero_indexed_month: 8,
                    zero_indexed_day: 2,
                    hours: (hours % 24) as u8,
                    minutes: (minutes % 60) as u8,
                    seconds: (seconds % 60) as u8
                }
            }
        }
    }

    impl TimeSource for FixedTimeSource {
        // #[inline(always)]
        fn get_timestamp(&self) -> Timestamp {
            return self.base_timestamp;
        }
    }

    pub struct FileContextWrapper<D,T>
        where
            D: BlockDevice,
            T: TimeSource,
    {
        pub volume_mgr: VolumeManager<D,T>,
        pub volume: Volume,
        pub file: File,
    }

    impl<D,T> FileContextWrapper<D,T>
        where
            D: BlockDevice,
            T: TimeSource,
    {
        pub fn write(&mut self, bytes: &[u8]) {
            let _ = self.volume_mgr.write(&mut self.volume, &mut self.file, bytes);
        }
    }

    pub fn open_logfile<D: BlockDevice>(sdcard: D, timestamp_ms: u64) -> FileContextWrapper<D,FixedTimeSource> {
        let fake_time_source = FixedTimeSource::new_with_time_ms(timestamp_ms);
        let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, fake_time_source);
        // Try and access Volume 0 (i.e. the first partition).
        // The volume object holds information about the filesystem on that volume.
        // It doesn't hold a reference to the Volume Manager and so must be passed back
        // to every Volume Manager API call. This makes it easier to handle multiple
        // volumes in parallel.
        let mut volume0 = volume_mgr.get_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
        //println!("\r\nVolume 0: {:?}", volume0);
        // Open the root directory (passing in the volume we're using).
        let root_dir = volume_mgr.open_root_dir(&volume0).unwrap();
        // Open a file called "MY_FILE.TXT" in the root directory
        let my_file = volume_mgr.open_file_in_dir(
            &mut volume0,
            &root_dir,
            "LOG.TXT",
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        ).unwrap();
        return FileContextWrapper {
            volume_mgr,
            volume: volume0,
            file: my_file
        }
    }
} // mod filetime

#[cfg(feature = "lorawan")]
pub mod lora_utils {
    use sx126x::conf::Config as LoRaConfig;
    use sx126x::op::status::CommandStatus::{CommandTimeout, CommandTxDone, DataAvailable};
    use sx126x::op::*;
    use sx126x::SX126x;

    use crate::{Delay, Spi2ProxyType};
    use crate::hal:: {
        gpio::{self, Floating, GpioPin, Output, OutputPin, Input, PullUp, PushPull},
    };


    type LoraRadioType<'a> = SX126x<
        Spi2ProxyType<'a>,
        GpioPin<Output<PushPull>, 9>,
        GpioPin<Output<PushPull>, 17>,
        GpioPin<Input<Floating>, 13>,
        GpioPin<Output<PushPull>, 35>,
        GpioPin<Input<Floating>, 45>,
    >;

    pub struct LoraWrapper<'a> {
        pub radio: LoraRadioType<'a>,
        pub(crate) spi_holder: Spi2ProxyType<'a>,
    }

    impl LoraWrapper<'_>{

        pub fn configure_for_receive(&mut self, delay: &mut Delay, timeout_ms: u32) {
            let rx_timeout = RxTxTimeout::from_ms(timeout_ms);
            self.radio.set_rx(&mut self.spi_holder, delay, rx_timeout).unwrap();
        }
    }

    pub(crate) fn build_lora_config() -> LoRaConfig {
        use sx126x::op::{
            irq::IrqMaskBit::*, modulation::lora::*, packet::lora::LoRaPacketParams,
            rxtx::DeviceSel::SX1261, PacketType::LoRa,
        };

        let mod_params = LoraModParams::default().into();
        let tx_params = TxParams::default()
            .set_power_dbm(14)
            .set_ramp_time(RampTime::Ramp200u);
        let pa_config = PaConfig::default()
            .set_device_sel(SX1261)
            .set_pa_duty_cycle(0x04);

        let dio1_irq_mask = IrqMask::none()
            .combine(TxDone)
            .combine(Timeout)
            .combine(RxDone);

        let packet_params = LoRaPacketParams::default().into();

        const F_XTAL: u32 = 32_000_000; // 32MHz
        const RF_FREQUENCY: u32 = 433_000_000; // 433 MHz
        let rf_freq = sx126x::calc_rf_freq(433 as f32, F_XTAL as f32);

        LoRaConfig {
            packet_type: LoRa,
            sync_word: 0x1424, // Private networks
            calib_param: CalibParam::from(0x7F),
            mod_params,
            tx_params,
            pa_config,
            packet_params: Some(packet_params),
            dio1_irq_mask,
            dio2_irq_mask: IrqMask::none(),
            dio3_irq_mask: IrqMask::none(),
            rf_frequency: RF_FREQUENCY,
            rf_freq,
        }
    }
}
