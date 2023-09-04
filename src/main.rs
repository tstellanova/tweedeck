#![no_std]
#![no_main]

extern crate embedded_sdmmc;

/**
Copyright (c) 2023 Todd Stellanova. All rights reserved.
LICENSE: BSD3 (See LICENSE file)

Bare metal application for using the Lilygo T-Deck

*/
// use core::fmt::Write;
use nb::block;

use esp_backtrace as _;
use esp_println::println;
use hal::{
    prelude::*,
    Delay,
    clock::ClockControl,
    i2c::I2C,
    IO,
    peripherals::Peripherals,
    timer::TimerGroup,
    Rtc,
    spi::{Spi, SpiMode},
    Uart,
    uart::{TxRxPins, config::*},
    };

use embedded_graphics::{
    mono_font::{ascii::{FONT_9X15_BOLD, FONT_8X13_BOLD},
                MonoFont, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Rectangle, PrimitiveStyleBuilder},
    text::{Text, TextStyle},
};

// Provides the parallel port and display interface builders
use display_interface_spi::SPIInterfaceNoCS;

// Provides the Display builder
use mipidsi::{Builder, Display};

// LoRa info
use sx126x::conf::Config as LoRaConfig;
use sx126x::SX126x;
use embedded_sdmmc::{File, TimeSource, SdCard, Timestamp, BlockDevice, VolumeManager, Volume};

// LoRa constants
const LORA_RF_FREQUENCY: u32 = 433_000_000; // 433MHz

// Display dimensions
const DISPLAY_W: usize = 320;
const DISPLAY_H: usize = 240;

const DISPLAY_SIZE: Size = Size::new(DISPLAY_W as u32, DISPLAY_H as u32);
const OUT_VIEW_SIZE: Size = Size::new(DISPLAY_SIZE.width, (DISPLAY_SIZE.height*5)/6);
const IN_VIEW_SIZE: Size = Size::new(DISPLAY_SIZE.width, DISPLAY_SIZE.height/6);
const OUT_VIEW_TOP_LEFT: Point = Point::new(0, 0);
const IN_VIEW_TOP_LEFT: Point = Point::new(0, OUT_VIEW_SIZE.height as i32);

// i2c bus address of T-Deck "T-Keyboard" (run by a separate microcontroller -- ESP32-C3)
const LILYGO_KB_I2C_ADDRESS: u8 =     0x55;

const INPUT_TEXT_FONT: MonoFont = FONT_9X15_BOLD;
const OUTPUT_TEXT_FONT: MonoFont = FONT_8X13_BOLD;
const OUT_CHAR_W:u32 = OUTPUT_TEXT_FONT.character_size.width as u32;
const OUT_CHAR_H:u32 = OUTPUT_TEXT_FONT.character_size.height as u32;
const IN_CHAR_W:u32 = INPUT_TEXT_FONT.character_size.width as u32;
const IN_CHAR_H:u32 = INPUT_TEXT_FONT.character_size.height as u32;
const OUT_VIEW_TXT_TOP_LEFT: Point = Point::new(
    OUT_VIEW_TOP_LEFT.x + OUT_CHAR_W as i32,  OUT_VIEW_TOP_LEFT.y + OUT_CHAR_H as i32);
const IN_VIEW_TXT_TOP_LEFT: Point = Point::new(
    IN_VIEW_TOP_LEFT.x + IN_CHAR_W  as i32,  IN_VIEW_TOP_LEFT.y + IN_CHAR_H as i32);
const MAX_OUT_X_IDX: u32 = (OUT_VIEW_SIZE.width / OUT_CHAR_W) -1; //expect 35?
const MAX_OUT_Y_IDX:u32 = (OUT_VIEW_SIZE.height / OUT_CHAR_H)-1; // expect 10?
const MAX_IN_X_IDX:u32 = (IN_VIEW_SIZE.width / IN_CHAR_W) -1; //expect 35?
const MAX_IN_Y_IDX:u32 = (IN_VIEW_SIZE.height / IN_CHAR_H)-1; // expect 2?

// Rules for how to update the cursor position based on a single character input
fn update_cursor_position( x_idx: &mut u32, y_idx: &mut u32, max_x_idx: u32, max_y_idx: u32, eol: bool,) {
    if eol {
        *y_idx += 1;
        *x_idx = 0;
        if *y_idx > max_y_idx {
            *y_idx = 0;
        }
    }
    else {
        *x_idx += 1;
        if *x_idx > max_x_idx {
            *x_idx = 0;
            *y_idx += 1;
            if *y_idx > max_y_idx {
                *y_idx = 0;
            }
        }
    }
}



//TODO setup i2s audio output
// fn setup_audio_output() {
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


struct FixedTimeSource {
    base_timestamp: Timestamp,
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

struct FileContextWrapper<D,T>
where
    D: BlockDevice,
    T: TimeSource,
{
    volume_mgr: VolumeManager<D,T>,
    volume: Volume,
    file: File,
}

impl<D,T> FileContextWrapper<D,T>
where
    D: BlockDevice,
    T: TimeSource,
{
    fn write(&mut self, bytes: &[u8]) {
        let _ = self.volume_mgr.write(&mut self.volume, &mut self.file, bytes);
    }

}

fn open_logfile<D: BlockDevice>(sdcard: D, timestamp_ms: u64) -> FileContextWrapper<D,FixedTimeSource> {
    let fake_time_source = FixedTimeSource::new_with_time_ms(timestamp_ms);
    let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sdcard, fake_time_source);
    // Try and access Volume 0 (i.e. the first partition).
    // The volume object holds information about the filesystem on that volume.
    // It doesn't hold a reference to the Volume Manager and so must be passed back
    // to every Volume Manager API call. This makes it easier to handle multiple
    // volumes in parallel.
    let mut volume0 = volume_mgr.get_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    println!("\r\nVolume 0: {:?}", volume0);
    // Open the root directory (passing in the volume we're using).
    let root_dir = volume_mgr.open_root_dir(&volume0).unwrap();
    // Open a file called "MY_FILE.TXT" in the root directory
    let mut my_file = volume_mgr.open_file_in_dir(
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
#[entry]
fn main() -> ! {
    let perphs = Peripherals::take();
    let mut system = perphs.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(
        perphs.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let mut timer0 = timer_group0.timer0;

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

    println!("Start setup");

    let io = IO::new(perphs.GPIO, perphs.IO_MUX);
    let mut delay = Delay::new(&clocks);

    let mut board_periph_pin = io.pins.gpio10.into_push_pull_output();
    board_periph_pin.set_high().unwrap();

    // Setup 100 kHz i2c0 bus
    let mut i2c0_bus = I2C::new(
        perphs.I2C0,
        io.pins.gpio18, //I2C0 SDA
        io.pins.gpio8, //I2C0 SCL
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
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

    // I2S audio output on MAX98357A
    let tdeck_i2s_bck = io.pins.gpio7; //ESP_I2S_BCK
    let tdeck_i2s_ws = io.pins.gpio5; //ESP_I2S_WS
    let tdeck_i2c_da = io.pins.gpio6; //ESP_I2S_DA (DOUT)


    let tdeck_tft_dc = io.pins.gpio11.into_push_pull_output();
    let mut tft_enable_pin =  io.pins.gpio42.into_push_pull_output();//enables backlight?
    // Note that AW9364 LED driver is used to drive TFT backlight

    // SPI chip select pins
    let mut tdeck_tft_cs = io.pins.gpio12.into_push_pull_output();
    let mut tdeck_lora_cs = io.pins.gpio9.into_push_pull_output();
    let mut tdeck_sdcard_cs = io.pins.gpio39.into_push_pull_output();

    // set CS pins high
    tdeck_sdcard_cs.set_high().unwrap();
    tdeck_lora_cs.set_high().unwrap();
    tdeck_tft_cs.set_high().unwrap();

    let spi2_raw = Spi::new(
        perphs.SPI2,
        io.pins.gpio40, //tdeck_sclk,
        io.pins.gpio41, //tdeck_mosi,
        io.pins.gpio38, //tdeck_miso,
        tdeck_tft_cs, // TODO I guess this is a default CS for the bus?
        60u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );
    // create a shared_bus so we can share SPI among multiple devices
    let spi2_bus = shared_bus::BusManagerSimple::new(spi2_raw);
    let sdcard = embedded_sdmmc::SdCard::new(spi2_bus.acquire_spi(), tdeck_sdcard_cs, delay);
    println!("sdcard {} bytes", sdcard.num_bytes().unwrap());
    let mut log_file_ctx = open_logfile(sdcard,     rtc.get_time_ms() );
    log_file_ctx.write(&[0x54, 0x53, 0x0d, 0x0a]);
    log_file_ctx.volume_mgr.close_file(&log_file_ctx.volume, log_file_ctx.file);

    // TODO setup audio output on   ESP_I2S_BCK, ESP_I2S_WS, ESP_I2S_DOUT;
    // TODO setup ES7210 (analog voice ADC from mic) on i2cs? address ES7210_AD1_AD0_00 = 0x40,

    // Setup TFT display
    tft_enable_pin.set_high().unwrap();
    // let di = SPIInterface::new(spi2_bus, tdeck_tft_dc, tdeck_tft_cs);
    let di = SPIInterfaceNoCS::new(spi2_bus.acquire_spi(), tdeck_tft_dc);
    let mut display = Builder::st7789(di)
        .with_display_size(DISPLAY_H as u16, DISPLAY_W as u16,)
        .with_orientation(mipidsi::Orientation::Landscape(true))
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .with_framebuffer_size(DISPLAY_H as u16, DISPLAY_W as u16,) //remember this is rotated
        .init(&mut delay, Some(tft_enable_pin))
        .unwrap();

    // Draw a box around the total display  area
    let box_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::YELLOW)
        .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();
    Rectangle::new(Point::new(0,0),Size::new(DISPLAY_W as u32,DISPLAY_H as u32))
        .into_styled(box_style)
        .draw(&mut display).unwrap();
    // Draw a box around the output view
    let output_view_box_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::CYAN)
        .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();
    Rectangle::new(OUT_VIEW_TOP_LEFT, OUT_VIEW_SIZE)
        .into_styled(output_view_box_style)
        .draw(&mut display).unwrap();
    // Draw a box around the input view
    let input_view_box_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::MAGENTA)
        .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();
    let input_view_rect = Rectangle::new(IN_VIEW_TOP_LEFT, IN_VIEW_SIZE)
        .into_styled(input_view_box_style);

    input_view_rect.draw(&mut display).unwrap();

    // let text_style = MonoTextStyle::new(&text_font, Rgb565::RED);// no bg fill
    let output_text_style = MonoTextStyleBuilder::new()
        .font(&OUTPUT_TEXT_FONT)
        .text_color(Rgb565::GREEN)
        .background_color(Rgb565::BLACK) //fill with black
        .build();

    let kb_text_style = MonoTextStyleBuilder::new()
        .font(&INPUT_TEXT_FONT)
        .text_color(Rgb565::RED)
        .background_color(Rgb565::BLACK)
        .build();


    // let text = "12345678901234567890123456789012345\r\n";
    let mut output_x_idx: u32 = 0;
    let mut output_y_idx: u32 = 0;
    let mut input_x_idx: u32 = 0;
    let mut input_y_idx: u32 = 0;

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

    let mut uart0 = Uart::new_with_config(
        perphs.UART0,
        Some(uart_config),
        Some(uart_pins),
        &clocks,
        &mut system.peripheral_clock_control,
        );

    timer0.start(50u64.millis());
    loop {
        let mut rbuf:[u8;1] = [0u8];
        let mut action_count = 0;

        // read from the serial port UART0 and display
        {
            // let mut read_count = 0;
            if let Ok(rb) = uart0.read() {
                action_count += 1;
                let eol =  rb == 0x0d;
                rbuf[0] = rb;

                // TODO disable echo -- used for debugging purposes only
                //let _ = uart0.write_bytes(&rbuf);
                // if eol {
                //     //tack on a linefeed
                //     let crlf: [u8;2] = [0x0d, 0x0a];
                //     let _ = uart0.write_bytes(&crlf);
                // }

                let _ = Text::new(core::str::from_utf8(&rbuf).unwrap(),
                                  Point::new(
                                      OUT_VIEW_TXT_TOP_LEFT.x + (output_x_idx * OUT_CHAR_W) as i32,
                                      OUT_VIEW_TXT_TOP_LEFT.y + (output_y_idx * OUT_CHAR_H) as i32),
                                  output_text_style)
                    .draw(&mut display)
                    .unwrap();
                update_cursor_position(&mut output_x_idx, &mut output_y_idx, MAX_OUT_X_IDX, MAX_OUT_Y_IDX, eol);
            }
        }

        //read from the T-Keyboard via i2c
        {
            let kb_res = i2c0_bus.read(LILYGO_KB_I2C_ADDRESS, &mut rbuf);
            match kb_res {
                Ok(..) => {
                    if 0 != rbuf[0] {
                        //println!("\r\n0x{:02x}",rbuf[0]);
                        action_count += 1;
                        let eol = rbuf[0] == 0x0d;
                        let _ = Text::new(core::str::from_utf8(&rbuf).unwrap(),
                                          Point::new(
                                              IN_VIEW_TXT_TOP_LEFT.x + (input_x_idx * OUT_CHAR_W) as i32,
                                              IN_VIEW_TXT_TOP_LEFT.y + (input_y_idx * OUT_CHAR_H) as i32),
                                          kb_text_style)
                            .draw(&mut display)
                            .unwrap();
                        update_cursor_position(&mut input_x_idx, &mut input_y_idx, MAX_IN_X_IDX, MAX_IN_Y_IDX, eol);
                        let _ = uart0.write_bytes(&rbuf);
                        if eol {
                            //tack on a linefeed
                            rbuf[0] = 0x0a;
                            let _ = uart0.write_bytes(&rbuf);
                            // clear input field
                            input_view_rect.draw(&mut display).unwrap();
                            input_x_idx = 0;
                            input_y_idx = 0;
                        }
                    }
                },
                Err(_err) => {
                    println!("kb err: {:?}", _err);
                }
            }
        }

        if 0 == action_count {
            block!(timer0.wait()).unwrap();
        }

    }


}


