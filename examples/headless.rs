#![no_std]
#![no_main]

/**
Copyright (c) 2023 Todd Stellanova. All rights reserved.
LICENSE: BSD3 (See LICENSE file)

Simplest possible bare-metal app that will run on T-Deck
with no display etc.

*/


use esp32s3_hal as hal;
use hal::{
    prelude::*,
};
use nb::block;
use esp_backtrace as _;
use esp_println::println;
use tweedeck::{Board, LILYGO_KB_I2C_ADDRESS};

#[entry]
fn main() -> ! {
    let mut board = Board::default();

    board.timer0.start(50u64.millis());
    loop {
        let mut rbuf:[u8;1] = [0u8];
        let mut action_count = 0;

        // read from the serial port UART0 and display
        {
            // let mut read_count = 0;
            if let Ok(rb) = board.uart0.read() {
                action_count += 1;
                let eol =  rb == 0x0d;
                rbuf[0] = rb;

                // echo uart data back to sender
                let _ = board.uart0.write_bytes(&rbuf);
                if eol {
                    //tack on a linefeed
                    let crlf: [u8;2] = [0x0d, 0x0a];
                    let _ = board.uart0.write_bytes(&crlf);
                }
            }
        }

        //read from the T-Keyboard via i2c and send data to correspondent via uart
        {
            let kb_res = board.i2c0_bus.read(LILYGO_KB_I2C_ADDRESS, &mut rbuf);
            match kb_res {
                Ok(..) => {
                    if 0 != rbuf[0] {
                        //println!("\r\n0x{:02x}",rbuf[0]);
                        action_count += 1;
                        let eol = rbuf[0] == 0x0d;
                        let _ = board.uart0.write_bytes(&rbuf);
                        if eol {
                            //tack on a linefeed
                            rbuf[0] = 0x0a;
                            let _ = board.uart0.write_bytes(&rbuf);
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

    }

}
