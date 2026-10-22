//! GridConnect ASCII framing: `:X19490ABCN0102030405060708;`
//!
//! Used by most LCC USB adapters (RR-CirKits Buffer-USB, TCS, SPROG CANISB) and
//! by GridConnect-over-TCP hubs such as JMRI (port 12021).
//!
//! GridConnect is a simple ASCII encoding of one CAN frame per token:
//!
//! ```text
//!   :X19490ABCN0102030405060708;
//!   │└──┬───┘│└──────┬───────┘└ terminator
//!   │   │    │       └ 0-8 data bytes as hex pairs
//!   │   │    └ 'N' = normal frame ('R' = remote request, accepted when parsing)
//!   │   └ 29-bit identifier, 1-8 hex digits
//!   └ 'X' = extended frame ('S' = standard 11-bit, not used by OpenLCB)
//! ```
//!
//! Frames are usually separated by newlines, but any whitespace (or none)
//! is allowed. The same text is used over USB serial adapters, raw TCP
//! (the JMRI/OpenLCB "GridConnect over TCP" hub protocol) and, in this
//! crate's `hub` module (feature `hub`), WebSocket text messages.
//!
//! * [`format()`] turns a [`CanFrame`] into text (uppercase hex, no newline).
//! * [`parse()`] reads one complete token, leniently (case-insensitive,
//!   surrounding whitespace ignored).
//! * [`Decoder`] splits an arbitrary byte stream into frames, handling
//!   tokens that are split across reads.
//!
//! ```
//! use rolcb::gridconnect;
//!
//! let f = gridconnect::parse(":X19490ABCN0102;")?;
//! assert_eq!(f.id, 0x1949_0ABC);
//! assert_eq!(f.data(), &[1, 2]);
//! assert_eq!(gridconnect::format(&f), ":X19490ABCN0102;");
//! # Ok::<(), rolcb::ParseError>(())
//! ```

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write;

use crate::can::CanFrame;
use crate::{hex_val, ParseError};

/// Format a frame as a GridConnect string (without a trailing newline).
///
/// The identifier is always written as eight uppercase hex digits and the
/// frame type as `N`. Append `'\n'` when writing to a serial port or TCP
/// socket; most peers (JMRI included) expect line-separated frames.
///
/// ```
/// use rolcb::{gridconnect, CanFrame};
///
/// let f = CanFrame::new(0x1070_0285, &[])?;
/// assert_eq!(gridconnect::format(&f), ":X10700285N;");
/// # Ok::<(), rolcb::ParseError>(())
/// ```
pub fn format(frame: &CanFrame) -> String {
    let mut s = String::with_capacity(28);
    let _ = write!(s, ":X{:08X}N", frame.id);
    for b in frame.data() {
        let _ = write!(s, "{b:02X}");
    }
    s.push(';');
    s
}

/// Parse a single GridConnect frame. Leading/trailing whitespace is ignored.
/// Only extended (`X`) frames are meaningful for OpenLCB; standard (`S`)
/// frames are rejected.
///
/// Letters are case-insensitive and `R` (remote frame) is accepted in
/// place of `N`, though the RTR distinction is not kept. Identifiers
/// longer than 29 bits are masked.
///
/// # Errors
///
/// * [`ParseError::Format`] if the `:`/`;` delimiters, the `X` or the
///   `N` are missing, or the frame is a standard frame.
/// * [`ParseError::Hex`] for a non-hex digit.
/// * [`ParseError::Length`] for an empty or over-long identifier, an odd
///   number of data digits, or more than 8 data bytes.
///
/// ```
/// use rolcb::{gridconnect, ParseError};
///
/// let f = gridconnect::parse("  :x195b4123n0501010122000010;\r\n")?;
/// assert_eq!(f.id, 0x195B_4123);
/// assert_eq!(gridconnect::parse(":S123N;"), Err(ParseError::Format("standard frame")));
/// assert_eq!(gridconnect::parse(":X195B4123N0;"), Err(ParseError::Length));
/// # Ok::<(), rolcb::ParseError>(())
/// ```
pub fn parse(s: &str) -> Result<CanFrame, ParseError> {
    let s = s.trim().as_bytes();
    if s.len() < 4 || s[0] != b':' || *s.last().unwrap() != b';' {
        return Err(ParseError::Format("missing ':' or ';'"));
    }
    let body = &s[1..s.len() - 1];
    match body.first() {
        Some(b'X') | Some(b'x') => {}
        Some(b'S') | Some(b's') => return Err(ParseError::Format("standard frame")),
        _ => return Err(ParseError::Format("expected 'X'")),
    }
    let n_pos = body
        .iter()
        .position(|&c| c == b'N' || c == b'n' || c == b'R' || c == b'r')
        .ok_or(ParseError::Format("missing 'N'"))?;
    let id_hex = &body[1..n_pos];
    if id_hex.is_empty() || id_hex.len() > 8 {
        return Err(ParseError::Length);
    }
    let mut id = 0u32;
    for &c in id_hex {
        id = (id << 4) | hex_val(c)? as u32;
    }
    let data_hex = &body[n_pos + 1..];
    if !data_hex.len().is_multiple_of(2) || data_hex.len() > 16 {
        return Err(ParseError::Length);
    }
    let mut data = [0u8; 8];
    for (i, pair) in data_hex.chunks(2).enumerate() {
        data[i] = (hex_val(pair[0])? << 4) | hex_val(pair[1])?;
    }
    CanFrame::new(id, &data[..data_hex.len() / 2])
}

/// Incremental decoder for a GridConnect byte stream (serial port, TCP socket,
/// WebSocket text). Frames may be split arbitrarily across `push` calls and
/// may be separated by any whitespace (or nothing at all).
///
/// Bytes outside a `:`…`;` token are ignored, so the decoder recovers from
/// line noise and from joining a stream mid-frame. A `:` always starts a new
/// token, and a token that grows past 32 characters without a `;` is
/// discarded (reported as [`ParseError::Length`]). Memory use is bounded.
///
/// ```
/// use rolcb::gridconnect::Decoder;
///
/// let mut d = Decoder::new();
/// // A read can end in the middle of a frame…
/// assert!(d.push(b"junk:X10700").is_empty());
/// // …and the next one may hold the rest plus more frames.
/// let frames = d.push(b"285N;\n:X19100285N050101012200;");
/// assert_eq!(frames.len(), 2);
/// assert_eq!(frames[0].as_ref().unwrap().id, 0x1070_0285);
/// assert_eq!(frames[1].as_ref().unwrap().data(), &[5, 1, 1, 1, 0x22, 0]);
///
/// // Malformed frames are reported, and decoding carries on.
/// let frames = d.push(b":X1G;:X10700285N;");
/// assert!(frames[0].is_err());
/// assert!(frames[1].is_ok());
/// ```
#[derive(Debug, Default)]
pub struct Decoder {
    buf: Vec<u8>,
}

impl Decoder {
    /// A decoder with an empty buffer.
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed bytes; returns every complete frame found. Malformed frames are
    /// returned as `Err` so callers can log them, and decoding continues.
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Result<CanFrame, ParseError>> {
        let mut out = Vec::new();
        for &b in bytes {
            match b {
                b':' => {
                    self.buf.clear();
                    self.buf.push(b);
                }
                b';' if !self.buf.is_empty() => {
                    self.buf.push(b);
                    let s = core::str::from_utf8(&self.buf).unwrap_or("");
                    out.push(parse(s));
                    self.buf.clear();
                }
                _ if !self.buf.is_empty() => {
                    if self.buf.len() >= 32 {
                        // Runaway garbage; resynchronise on the next ':'.
                        self.buf.clear();
                        out.push(Err(ParseError::Length));
                    } else if !b.is_ascii_whitespace() {
                        self.buf.push(b);
                    }
                }
                _ => {}
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let f = parse(":X19490ABCN0102030405060708;").unwrap();
        assert_eq!(f.id, 0x1949_0ABC);
        assert_eq!(f.data(), &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(format(&f), ":X19490ABCN0102030405060708;");

        let f = parse(":X10700285N;").unwrap();
        assert_eq!(f.id, 0x1070_0285);
        assert!(f.data().is_empty());
        assert_eq!(format(&f), ":X10700285N;");
    }

    #[test]
    fn lenient_parse() {
        let f = parse("  :x195b4123n0501010122000010;\r\n").unwrap();
        assert_eq!(f.id, 0x195B_4123);
        assert_eq!(f.data().len(), 8);
        assert!(parse(":S123N;").is_err());
        assert!(parse(":X195B4123N0;").is_err());
        assert!(parse(":X195B4123N010203040506070809;").is_err());
        assert!(parse("X195B4123N;").is_err());
    }

    #[test]
    fn stream_decoder() {
        let mut d = Decoder::new();
        let mut frames = d.push(b"junk:X10700");
        assert!(frames.is_empty());
        frames.extend(d.push(b"285N;\n:X19100285N050101012200;:X1949"));
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].as_ref().unwrap().id, 0x1070_0285);
        assert_eq!(frames[1].as_ref().unwrap().data(), &[5, 1, 1, 1, 0x22, 0]);
        let frames = d.push(b"0ABCN;");
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].as_ref().unwrap().id, 0x1949_0ABC);
    }
}
