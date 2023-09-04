#![no_std]
#![no_main]

use core::fmt::Write;

use esp_backtrace as _;
use esp_println::println;
use hal::{
    prelude::*,
    Delay,
    clock::ClockControl,
    IO,
    peripherals::{Peripherals},
    timer::TimerGroup,
    Rtc,
    spi::{Spi, SpiMode},
    Uart,
    uart::{TxRxPins, config::*},
    };

use embedded_graphics::{
    mono_font::{ascii::FONT_9X15_BOLD, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Rectangle, PrimitiveStyleBuilder},
    text::Text,
};

// Provides the parallel port and display interface builders
use display_interface_spi::SPIInterfaceNoCS;

// Provides the Display builder
use mipidsi::Builder;
use nb::block;
// use embedded_graphics_framebuf::FrameBuf;
use sx126x::conf::Config as LoRaConfig;
use sx126x::SX126x;

// LoRA constants
const LORA_RF_FREQUENCY: u32 = 433_000_000; // 433MHz

// Display dimensions
const DISPLAY_W: usize = 320;
const DISPLAY_H: usize = 240;


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

    let tdeck_sclk = io.pins.gpio40;
    let tdeck_miso = io.pins.gpio38;
    let tdeck_mosi = io.pins.gpio41;
    let tdeck_tft_cs = io.pins.gpio12.into_push_pull_output();
    let tdeck_tft_dc = io.pins.gpio11.into_push_pull_output();
    let mut tft_enable_pin =  io.pins.gpio42.into_push_pull_output();
    let _tdeck_sdcard_cs = io.pins.gpio39;

    let  spi_tft = Spi::new(
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
    let di = SPIInterfaceNoCS::new(spi_tft, tdeck_tft_dc);
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

    // Draw a box around the text area
    let box_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::GREEN)
        .stroke_width(3)
        .fill_color(Rgb565::BLACK)
        .build();
    Rectangle::new(Point::new(0,0),Size::new(DISPLAY_W as u32,DISPLAY_H as u32))
        .into_styled(box_style)
        .draw(&mut display).unwrap();

    // println!("start text render");

    let text_font = FONT_9X15_BOLD;
    let char_w = text_font.character_size.width as usize;
    let char_h = text_font.character_size.height as usize;
    let max_x_index = DISPLAY_W / char_w; //expect 35?
    let max_y_index = DISPLAY_H / char_h; // expect 15?
    println!("max_x {} max_y {}", max_x_index, max_y_index);

    // let text_style = MonoTextStyle::new(&text_font, Rgb565::RED);
    let text_style = MonoTextStyleBuilder::new()
        .font(&text_font)
        .text_color(Rgb565::RED)
        .background_color(Rgb565::BLACK)
        .build();

    let text = "12345678901234567890123456789012345\r\n";
    let mut x_char_index = 0;
    let mut y_char_index = 0;

    // let _ = Text::new(text, Point::new(0, text_y as i32), text_style)
    //     .draw(&mut display)
    //     .unwrap();
    // text_y += char_h;

    // let mut line_count = 0;
    // // display.clear(Rgb565::RED).unwrap();
    // for text_y in (char_h..DISPLAY_H as usize).step_by(char_h) {
    //     println!("text_y: {}", text_y);
    //     let _ = Text::new(text, Point::new(0, text_y as i32), text_style)
    //         .draw(&mut display)
    //         .unwrap();
    //     line_count += 1;
    // }
    //
    // println!("done text render, lines: {}", line_count);


    let uart_config = Config {
        baudrate: 115200,
        data_bits: DataBits::DataBits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::STOP1,
    };

    let uart_pins = TxRxPins::new_tx_rx(
        io.pins.gpio43.into_push_pull_output(), //43 U0TXD
        io.pins.gpio44.into_floating_input(), // 44 U0RXD
        );

    let mut serial_port = Uart::new_with_config(
        perphs.UART0,
        Some(uart_config),
        Some(uart_pins),
        &clocks,
        &mut system.peripheral_clock_control,
        );

    // let mut serial_port = Uart::new(perphs.UART0, &mut system.peripheral_clock_control);
    let _ = serial_port.write_str(&text);
    let _ = serial_port.flush();

    timer0.start(100u64.millis());
    loop {
        let mut rbuf:[u8;1] = [0u8];
        let mut read_count = 0;
        let res = serial_port.read();
        match res {
            Ok(rb) => {
                rbuf[0] = rb;
                read_count += 1;
            },
            Err(_err) => {
            }
        }

        if read_count > 0 {
            let _ = serial_port.write_bytes(&rbuf);
            let eol = rbuf[0] == 0x0d;

            let _ = Text::new(core::str::from_utf8(&rbuf).unwrap(),
                              Point::new(
                                  (x_char_index * char_w) as i32,
                                  (y_char_index * char_h) as i32),
                              text_style)
                .draw(&mut display)
                .unwrap();

            if eol {
                //tack on a linefeed
                rbuf[0] = 0x0a;
                let _ = serial_port.write_bytes(&rbuf);

                y_char_index += 1;
                x_char_index = 0;
                if y_char_index > max_y_index {
                    y_char_index = 0;
                }
            }
            else {
                x_char_index += read_count;
                if x_char_index > max_x_index {
                    x_char_index = 0;
                    y_char_index += 1;
                    if y_char_index > max_y_index {
                        y_char_index = 0;
                    }
                }
            }
        }
        else {
            block!(timer0.wait()).unwrap();
        }

    }


}


