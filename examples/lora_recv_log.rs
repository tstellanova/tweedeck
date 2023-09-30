#![no_std]
#![no_main]
/**
Copyright (c) 2023 Todd Stellanova. All rights reserved.
LICENSE: BSD3 (See LICENSE file)

Bare metal app that shows how to open a serial port (UART0),
send and receive data, and display on the embedded graphics display.

 */

extern crate tweedeck;




use esp32s3_hal as hal;
use hal::{
    prelude::*,
};


use nb::block;
use esp_backtrace as _;
use esp_println::println;
use tweedeck::{Board,
};

#[cfg(feature = "sdcard")]
use tweedeck::sdcard_utils::{self};


#[entry]
fn main() -> ! {
    let mut board = Board::new();
    println!("board setup done");

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


    //create a reusable 50 ms timer
    board.timer0.start(50u64.millis());

    #[cfg(feature = "lorawan")]
    {
        println!("configure_for_receive");
        board.lora.configure_for_receive(&mut board.delay, 3000);
    }

    loop {
        const RBUF_SIZE: usize = 256;
        let mut rbuf:[u8;RBUF_SIZE] = [0u8; RBUF_SIZE];
        let mut action_count = 0;

        #[cfg(feature = "lorawan")]
        {
            // println!("check rx");
            // let avail = board.lora.rx_bytes_avail(&mut board.delay);
            // if (avail > 0) {
            //     println!("avail: {}", avail);
            //     action_count += 1;
            // }
            //if avail > 0 {
            let recvd = board.lora.read_payload(&mut board.delay, &mut rbuf, RBUF_SIZE);
            if recvd > 0 {
                action_count += 1;
                println!("recvd: [{}]  {:?} \r\n", recvd,  &rbuf[..recvd+1]); //core::str::from_utf8(&rbuf).unwrap());
            }
            //}
        }

        if 0 == action_count {
            block!(board.timer0.wait()).unwrap();
        }

        // on exit or whatever:
        // log_ctx.volume_mgr.close_file(&log_ctx.volume, log_ctx.file);

    }



}


