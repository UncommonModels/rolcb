//! Decode a GridConnect log (stdin or file) into readable OpenLCB messages.
//!
//! ```text
//! cargo run --example gc_dump -- capture.txt
//! ```

use std::io::{self, BufRead, BufReader};

use rolcb::can::CanHeader;
use rolcb::gridconnect::Decoder;
use rolcb::message::{Assembler, Message};

fn main() -> io::Result<()> {
    let input: Box<dyn BufRead> = match std::env::args().nth(1) {
        Some(path) => Box::new(BufReader::new(std::fs::File::open(path)?)),
        None => Box::new(BufReader::new(io::stdin())),
    };
    let mut decoder = Decoder::new();
    let mut assembler = Assembler::new();
    for line in input.lines() {
        for frame in decoder.push(line?.as_bytes()) {
            let frame = match frame {
                Ok(f) => f,
                Err(e) => {
                    println!("!! {e}");
                    continue;
                }
            };
            match frame.header() {
                CanHeader::Message { .. } | CanHeader::Datagram { .. } => {
                    if let Some(raw) = assembler.accept(&frame) {
                        let msg = Message::from_parts(raw.mti, raw.dest, raw.payload);
                        println!("{:03X} {:<28} {msg:?}", raw.src_alias, raw.mti.name());
                    }
                }
                h => println!("{:03X} {h:?} {:02X?}", h.src_alias(), frame.data()),
            }
        }
    }
    Ok(())
}
