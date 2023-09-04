#![no_std]
#![no_main]

extern crate embedded_graphics;

use core::fmt::Write;
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
    mono_font::{ascii::FONT_9X15_BOLD, MonoFont, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Rectangle, PrimitiveStyleBuilder},
    text::{Text, TextStyle},
};

// Provides the parallel port and display interface builders
use display_interface_spi::SPIInterfaceNoCS;

// Provides the Display builder
use mipidsi::{Builder, Display};
// use embedded_graphics_framebuf::FrameBuf;

// LoRa info
use sx126x::conf::Config as LoRaConfig;
use sx126x::SX126x;

// LoRa constants
const LORA_RF_FREQUENCY: u32 = 433_000_000; // 433MHz

// Display dimensions
const DISPLAY_W: usize = 320;
const DISPLAY_H: usize = 240;

const DISPLAY_SIZE: Size = Size::new(DISPLAY_W as u32, DISPLAY_H as u32);
const OUTPUT_VIEW_SIZE: Size = Size::new(DISPLAY_SIZE.width, (DISPLAY_SIZE.height*2)/3);
const INPUT_VIEW_SIZE: Size = Size::new(DISPLAY_SIZE.width, DISPLAY_SIZE.height/3);
const OUTPUT_VIEW_TOP_LEFT: Point = Point::new(0,0);
const INPUT_VIEW_TOP_LEFT: Point = Point::new(0, OUTPUT_VIEW_SIZE.height as i32);

// i2c bus address of T-Deck "T-Keyboard" (run by a separate microcontroller -- ESP32-C3)
const LILYGO_KB_I2C_ADDRESS: u8 =     0x55;

const INPUT_TEXT_FONT: MonoFont = FONT_9X15_BOLD;
const OUTPUT_TEXT_FONT: MonoFont = FONT_9X15_BOLD;
const OUT_CHAR_W:u32 = OUTPUT_TEXT_FONT.character_size.width as u32;
const OUT_CHAR_H:u32 = OUTPUT_TEXT_FONT.character_size.height as u32;
const IN_CHAR_W:u32 = INPUT_TEXT_FONT.character_size.width as u32;
const IN_CHAR_H:u32 = INPUT_TEXT_FONT.character_size.height as u32;
const MAX_OUT_X_IDX: u32 = OUTPUT_VIEW_SIZE.width / OUT_CHAR_W; //expect 35?
const MAX_OUT_Y_IDX:u32 = OUTPUT_VIEW_SIZE.height / OUT_CHAR_H; // expect 10?
const MAX_IN_X_IDX:u32 = INPUT_VIEW_SIZE.width / IN_CHAR_W; //expect 35?
const MAX_IN_Y_IDX:u32 = INPUT_VIEW_SIZE.height / IN_CHAR_H; // expect 2?

// Rules for how to move the cursor position based on a single character input
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

    let mut board_periph_pin = io.pins.gpio10.into_push_pull_output();
    board_periph_pin.set_high().unwrap();

    /*
    #define BOARD_I2C_SDA       18
#define BOARD_I2C_SCL       8
     */
    // Setup 100 kHz i2c0 bus
    let mut i2c0_bus = I2C::new(
        perphs.I2C0,
        io.pins.gpio18,
        io.pins.gpio8,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    // bus_i2c.write_read(LILYGO_KB_I2C_ADDRESS, &[0xaa], &mut kb_data).ok();


    let tdeck_sclk = io.pins.gpio40;
    let tdeck_miso = io.pins.gpio38;
    let tdeck_mosi = io.pins.gpio41;
    let tdeck_tft_cs = io.pins.gpio12.into_push_pull_output();
    let tdeck_tft_dc = io.pins.gpio11.into_push_pull_output();
    let mut tft_enable_pin =  io.pins.gpio42.into_push_pull_output();
    let _tdeck_sdcard_cs = io.pins.gpio39;

    let spi2_bus = Spi::new(
        perphs.SPI2,
        tdeck_sclk,
        tdeck_mosi,
        tdeck_miso,
        tdeck_tft_cs,
        60u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );
    tft_enable_pin.set_high().unwrap();

    // let di = SPIInterface::new(spi_tft, tdeck_tft_dc, tdeck_tft_cs);
    let di = SPIInterfaceNoCS::new(spi2_bus, tdeck_tft_dc);
    let mut delay = Delay::new(&clocks);
    let mut display = Builder::st7789(di)
        .with_display_size(DISPLAY_H as u16, DISPLAY_W as u16,)
        .with_orientation(mipidsi::Orientation::Landscape(true))
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .with_framebuffer_size(400 as u16, 400 as u16,) //TODO tune
        .init(&mut delay, Some(tft_enable_pin))
        .unwrap();

    // Clear the display initially
    // display.clear(Rgb565::BLUE).unwrap();

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
    Rectangle::new(OUTPUT_VIEW_TOP_LEFT,OUTPUT_VIEW_SIZE)
        .into_styled(output_view_box_style)
        .draw(&mut display).unwrap();
    // Draw a box around the input view
    let input_view_box_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::MAGENTA)
        .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();
    Rectangle::new(INPUT_VIEW_TOP_LEFT,INPUT_VIEW_SIZE)
        .into_styled(input_view_box_style)
        .draw(&mut display).unwrap();

    // println!("start text render");


    println!("max_x {} max_y {}", MAX_OUT_X_IDX, MAX_OUT_Y_IDX);

    // let text_style = MonoTextStyle::new(&text_font, Rgb565::RED);// no bg fill
    let output_text_style = MonoTextStyleBuilder::new()
        .font(&OUTPUT_TEXT_FONT)
        .text_color(Rgb565::RED)
        .background_color(Rgb565::BLACK) //fill with black
        .build();

    let kb_text_style = MonoTextStyleBuilder::new()
        .font(&OUTPUT_TEXT_FONT)
        .text_color(Rgb565::GREEN)
        .background_color(Rgb565::BLACK)
        .build();


    let text = "12345678901234567890123456789012345\r\n";
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

    let mut serial_port = Uart::new_with_config(
        perphs.UART0,
        Some(uart_config),
        Some(uart_pins),
        &clocks,
        &mut system.peripheral_clock_control,
        );

    // let _ = serial_port.write_str(&text);
    // let _ = serial_port.flush();

    timer0.start(100u64.millis());
    loop {
        let mut rbuf:[u8;1] = [0u8];

        // read from the serial port and display
        {
            // let mut read_count = 0;
            if let Ok(rb) = serial_port.read() {
                let eol = ( rb == 0x0d);
                rbuf[0] = rb;
                // TODO disable echo -- used for debugging purposes only
                let _ = serial_port.write_bytes(&rbuf);
                if eol {
                    //tack on a linefeed
                    rbuf[0] = 0x0a;
                    let _ = serial_port.write_bytes(&rbuf);
                }

                let _ = Text::new(core::str::from_utf8(&rbuf).unwrap(),
                                  Point::new(
                                      OUTPUT_VIEW_TOP_LEFT.x + (output_x_idx * OUT_CHAR_W) as i32,
                                      OUTPUT_VIEW_TOP_LEFT.y + (output_y_idx * OUT_CHAR_H) as i32),
                                  output_text_style)
                    .draw(&mut display)
                    .unwrap();
                update_cursor_position(&mut output_x_idx, &mut output_y_idx, MAX_OUT_X_IDX, MAX_OUT_Y_IDX, eol);
            }
        }

        //read from the T-Keyboard via i2c
        {
            let mut rbuf = [0u8;1];
            let kb_res = i2c0_bus.read(LILYGO_KB_I2C_ADDRESS, &mut rbuf);
            match kb_res {
                Ok(..) => {
                    if 0 != rbuf[0] {
                        //println!("\r\n0x{:02x}",rbuf[0]);
                        let eol = rbuf[0] == 0x0d;
                        let _ = Text::new(core::str::from_utf8(&rbuf).unwrap(),
                                          Point::new(
                                              INPUT_VIEW_TOP_LEFT.x + (input_x_idx * OUT_CHAR_W) as i32,
                                              INPUT_VIEW_TOP_LEFT.y + (input_y_idx * OUT_CHAR_H) as i32),
                                          kb_text_style)
                            .draw(&mut display)
                            .unwrap();
                        update_cursor_position(&mut input_x_idx, &mut input_y_idx, MAX_IN_X_IDX, MAX_IN_Y_IDX, eol);
                        let _ = serial_port.write_bytes(&rbuf);
                        if eol {
                            //tack on a linefeed
                            rbuf[0] = 0x0a;
                            let _ = serial_port.write_bytes(&rbuf);
                        }
                    }
                },
                Err(_err) => {
                    println!("kb err: {:?}", _err);
                }

            }
        }
        block!(timer0.wait()).unwrap();



    }


}


