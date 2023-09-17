#![no_std]
#![no_main]

extern crate tweedeck;

/**
Copyright (c) 2023 Todd Stellanova. All rights reserved.
LICENSE: BSD3 (See LICENSE file)

Bare metal app that shows how to open a serial port (UART0),
send and receive data, and display on the embedded graphics display.

*/


use esp32s3_hal as hal;
use hal::{
    prelude::*,
};
use embedded_graphics::{
    mono_font::{ascii::{FONT_9X15_BOLD, FONT_8X13_BOLD},
                MonoFont, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Rectangle, PrimitiveStyleBuilder},
    text::{Text, TextStyle},
};

use nb::block;
use esp_backtrace as _;
use esp_println::println;
use tweedeck::{Board, LILYGO_KB_I2C_ADDRESS, DISPLAY_SIZE,
               sdcard_utils::{self, FileContextType},
};

const OUT_VIEW_SIZE: Size = Size::new(DISPLAY_SIZE.width, (DISPLAY_SIZE.height*5)/6);
const IN_VIEW_SIZE: Size = Size::new(DISPLAY_SIZE.width, DISPLAY_SIZE.height/6);
const OUT_VIEW_TOP_LEFT: Point = Point::new(0, 0);
const IN_VIEW_TOP_LEFT: Point = Point::new(0, OUT_VIEW_SIZE.height as i32);

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






#[entry]
fn main() -> ! {
    let mut board = Board::new();

    //setup a log file
    #[cfg(feature = "sdcard")]
        let mut log_ctx = {
                //TODO board.sdcard may be None if the card isn't inserted?
                let mut ctx =
                    sdcard_utils::open_logfile(board.sdcard.unwrap(), board.rtc.get_time_ms() );
                ctx.write(&[0x54, 0x53, 0x0d, 0x0a]);
                // ctx.volume_mgr.close_file(&log_ctx.volume, log_ctx.file);
                ctx
        };

    // Draw a box around the total display  area
    let box_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::YELLOW)
        .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();
    Rectangle::new(Point::new(0,0),DISPLAY_SIZE)
        .into_styled(box_style)
        .draw(&mut board.display).unwrap();
    // Draw a box around the output view
    let output_view_box_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::CYAN)
        .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();
    Rectangle::new(OUT_VIEW_TOP_LEFT, OUT_VIEW_SIZE)
        .into_styled(output_view_box_style)
        .draw(&mut board.display).unwrap();
    // Draw a box around the input view
    let input_view_box_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::MAGENTA)
        .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();
    let input_view_rect = Rectangle::new(IN_VIEW_TOP_LEFT, IN_VIEW_SIZE)
        .into_styled(input_view_box_style);

    input_view_rect.draw(&mut board.display).unwrap();

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

    let mut output_x_idx: u32 = 0;
    let mut output_y_idx: u32 = 0;
    let mut input_x_idx: u32 = 0;
    let mut input_y_idx: u32 = 0;

    //create a reusable 50 ms timer
    board.timer0.start(50u64.millis());
    loop {
        let mut rbuf:[u8;1] = [0u8];
        let mut action_count = 0;

        // read from the serial port UART0 and display
        {
            if let Ok(rb) = board.uart0.read() {
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
                    .draw(&mut board.display)
                    .unwrap();
                update_cursor_position(&mut output_x_idx, &mut output_y_idx, MAX_OUT_X_IDX, MAX_OUT_Y_IDX, eol);
            }
        }

        //read from the T-Keyboard via i2c and send data to correspondent via uart
        {
            let kb_res = board.i2c0_proxy.read(LILYGO_KB_I2C_ADDRESS, &mut rbuf);
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
                            .draw(&mut board.display)
                            .unwrap();
                        update_cursor_position(&mut input_x_idx, &mut input_y_idx, MAX_IN_X_IDX, MAX_IN_Y_IDX, eol);
                        let _ = board.uart0.write_bytes(&rbuf);
                        #[cfg(feature = "sdcard")]
                        log_ctx.write(&rbuf);

                        if eol {
                            //tack on a linefeed
                            rbuf[0] = 0x0a;
                            let _ = board.uart0.write_bytes(&rbuf);
                            // clear input field
                            input_view_rect.draw(&mut board.display).unwrap();
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
            block!(board.timer0.wait()).unwrap();
        }

        // on exit or whatever:
        // log_ctx.volume_mgr.close_file(&log_ctx.volume, log_ctx.file);

    }



}


