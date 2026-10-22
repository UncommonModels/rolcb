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
//! is allowed. The same text is used over USB serial adapters and raw TCP (the
//! JMRI/OpenLCB "GridConnect over TCP" hub protocol, also spoken by this
//! crate's `hub` module, feature `hub`).
//!
//! * [`format()`] turns a [`CanFrame`] into text (uppercase hex, no newline);
//!   [`write()`] does the same into any [`core::fmt::Write`] without
//!   allocating. `CanFrame`'s `Display` impl uses it too.
//! * [`parse()`] reads one complete token, leniently (case-insensitive,
//!   surrounding whitespace ignored). `CanFrame`'s `FromStr` impl calls it.
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
#[must_use]
pub fn format(frame: &CanFrame) -> String {
    let mut s = String::with_capacity(28);
    // Writing into a String cannot fail.
    let _ = write(frame, &mut s);
    s
}

/// Write a frame as GridConnect text into `out`, without allocating.
///
/// Produces exactly what [`format()`] returns (no newline). Handy for
/// `no_std` targets writing into a fixed buffer, or for appending many
/// frames to one `String`.
///
/// # Errors
///
/// Only the errors returned by `out` itself.
///
/// ```
/// use core::fmt::Write;
/// use rolcb::{gridconnect, CanFrame};
///
/// let mut text = String::new();
/// for id in [0x1070_0285, 0x1070_1285] {
///     gridconnect::write(&CanFrame::new(id, &[])?, &mut text).unwrap();
///     text.push('\n');
/// }
/// assert_eq!(text, ":X10700285N;\n:X10701285N;\n");
/// # Ok::<(), rolcb::ParseError>(())
/// ```
pub fn write<W: core::fmt::Write + ?Sized>(frame: &CanFrame, out: &mut W) -> core::fmt::Result {
    write!(out, ":X{:08X}N", frame.id)?;
    for b in frame.data() {
        write!(out, "{b:02X}")?;
    }
    out.write_char(';')
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
    let Some(body) = s
        .trim()
        .as_bytes()
        .strip_prefix(b":")
        .and_then(|b| b.strip_suffix(b";"))
        .filter(|b| b.len() >= 2)
    else {
        return Err(ParseError::Format("missing ':' or ';'"));
    };
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

/// Maximum token length the [`Decoder`] buffers, `:` and `;` included.
const MAX_TOKEN: usize = 32;

/// Incremental decoder for a GridConnect byte stream (serial port, TCP socket,
/// or any other byte transport). Frames may be split arbitrarily across `push` calls and
/// may be separated by any whitespace (or nothing at all).
///
/// Bytes outside a `:`…`;` token are ignored, so the decoder recovers from
/// line noise and from joining a stream mid-frame. A `:` always starts a new
/// token; if a previous token was still open, it is reported as
/// `Err(ParseError::Format("unterminated frame"))`. A token that grows past
/// 32 characters without a `;` is discarded (reported as
/// [`ParseError::Length`]). The decoder never allocates on its own: its
/// buffer is a fixed 32-byte array.
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
#[derive(Debug, Default, Clone)]
pub struct Decoder {
    buf: [u8; MAX_TOKEN],
    len: usize,
}

impl Decoder {
    /// A decoder with an empty buffer.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed bytes; returns every complete frame found. Malformed frames are
    /// returned as `Err` so callers can log them, and decoding continues.
    ///
    /// To skip errors, flatten the result:
    /// `for frame in decoder.push(bytes).into_iter().flatten() { … }`.
    #[must_use = "decoded frames are returned, not stored"]
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Result<CanFrame, ParseError>> {
        let mut out = Vec::new();
        self.push_with(bytes, |r| out.push(r));
        out
    }

    /// Feed bytes and call `f` for every complete (or malformed) frame, in
    /// order, without allocating. Same rules as [`Decoder::push`].
    ///
    /// ```
    /// use rolcb::gridconnect::Decoder;
    ///
    /// let mut d = Decoder::new();
    /// let mut ids = Vec::new();
    /// d.push_with(b":X10700285N;:X10701285N050101012200;", |r| {
    ///     if let Ok(frame) = r {
    ///         ids.push(frame.id);
    ///     }
    /// });
    /// assert_eq!(ids, [0x1070_0285, 0x1070_1285]);
    /// ```
    pub fn push_with<F>(&mut self, bytes: &[u8], mut f: F)
    where
        F: FnMut(Result<CanFrame, ParseError>),
    {
        for &b in bytes {
            match b {
                b':' => {
                    if self.len != 0 {
                        f(Err(ParseError::Format("unterminated frame")));
                    }
                    self.buf[0] = b;
                    self.len = 1;
                }
                b';' if self.len != 0 => {
                    let r = match self.buf.get(..self.len) {
                        Some(tok) => {
                            // Only ASCII bytes are ever buffered except for
                            // stray high bytes, which make parsing fail.
                            let mut text = [0u8; MAX_TOKEN + 1];
                            text[..tok.len()].copy_from_slice(tok);
                            text[tok.len()] = b';';
                            match core::str::from_utf8(&text[..=tok.len()]) {
                                Ok(s) => parse(s),
                                Err(_) => Err(ParseError::Format("non-ASCII byte in frame")),
                            }
                        }
                        None => Err(ParseError::Length),
                    };
                    self.len = 0;
                    f(r);
                }
                _ if self.len != 0 => {
                    if self.len >= MAX_TOKEN {
                        // Runaway garbage; resynchronise on the next ':'.
                        self.len = 0;
                        f(Err(ParseError::Length));
                    } else if !b.is_ascii_whitespace() {
                        self.buf[self.len] = b;
                        self.len += 1;
                    }
                }
                _ => {}
            }
        }
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

    const STREAM: &[u8] = b"noise\r\n:X10700285N;\n:X19100285N050101012200;  \
        :x195b4123n0501010122000010;:X195EB285N16A1200100050101;\n:X195EB285N26A1012299;\n";

    fn ids(results: &[Result<CanFrame, ParseError>]) -> Vec<u32> {
        results.iter().map(|r| r.as_ref().unwrap().id).collect()
    }

    #[test]
    fn byte_by_byte() {
        let whole = Decoder::new().push(STREAM);
        assert_eq!(whole.len(), 5);
        assert!(whole.iter().all(Result::is_ok));
        let mut d = Decoder::new();
        let mut got = Vec::new();
        for b in STREAM {
            got.extend(d.push(core::slice::from_ref(b)));
        }
        assert_eq!(got, whole);
    }

    #[test]
    fn random_splits() {
        let whole = Decoder::new().push(STREAM);
        // Small deterministic LCG so the test needs no dependencies.
        let mut seed = 0x1234_5678u32;
        for _ in 0..500 {
            let mut d = Decoder::new();
            let mut got = Vec::new();
            let mut rest = STREAM;
            while !rest.is_empty() {
                seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                let n = 1 + (seed >> 16) as usize % 12;
                let (now, later) = rest.split_at(n.min(rest.len()));
                d.push_with(now, |r| got.push(r));
                rest = later;
            }
            assert_eq!(got, whole);
        }
    }

    #[test]
    fn resync_after_garbage() {
        let mut d = Decoder::new();
        // A runaway token (no ';' within 32 chars) is dropped, then the next
        // ':' resynchronises.
        let mut input = Vec::from(&b":X"[..]);
        input.extend(core::iter::repeat_n(b'F', 40));
        input.extend_from_slice(b";:X10700285N;");
        let r = d.push(&input);
        assert_eq!(r[0], Err(ParseError::Length));
        assert_eq!(ids(&r[1..]), [0x1070_0285]);
        // Bytes between frames are ignored, including a lone ';'.
        let r = d.push(b"; garbage ;;:X10701285N;");
        assert_eq!(ids(&r), [0x1070_1285]);
        // Non-ASCII inside a token is an error, not a panic.
        let r = d.push(b":X1\xFF0700285N;");
        assert!(r[0].is_err());
    }

    #[test]
    fn unterminated_frame() {
        let mut d = Decoder::new();
        let r = d.push(b":X10700285N:X10701285N;");
        assert_eq!(r.len(), 2);
        assert_eq!(r[0], Err(ParseError::Format("unterminated frame")));
        assert_eq!(r[1].as_ref().unwrap().id, 0x1070_1285);
        // Also across pushes.
        assert!(d.push(b":X1070").is_empty());
        let r = d.push(b":X10700285N;");
        assert_eq!(r[0], Err(ParseError::Format("unterminated frame")));
        assert!(r[1].is_ok());
    }

    #[test]
    fn write_matches_format() {
        let f = CanFrame::new(0x195B_4123, &[0xAB, 0x01]).unwrap();
        let mut s = String::new();
        write(&f, &mut s).unwrap();
        assert_eq!(s, format(&f));
        assert_eq!(s, ":X195B4123NAB01;");
        // Edge cases of parse.
        assert!(parse(":;").is_err());
        assert!(parse(":X;").is_err());
        assert!(parse(":XN;").is_err());
        assert!(parse(":X123456789N;").is_err());
        assert_eq!(parse(":XFFFFFFFFN;").unwrap().id, 0x1FFF_FFFF);
    }
}
