//! # rolcb — Rustified OpenLCB
//!
//! A sans-IO implementation of the OpenLCB / LCC protocol stack. The crate never
//! touches sockets, serial ports or clocks: you feed it CAN frames and the
//! current time, and it hands back frames to transmit and decoded messages.
//! That makes it usable from async runtimes, WASM in the browser, and
//! embedded targets alike (`no_std` + `alloc` with `default-features = false`).
//!
//! [OpenLCB](https://openlcb.org) (marketed by the NMRA as **LCC**, Layout
//! Command Control) is a peer-to-peer control bus for model railroads. Nodes
//! (block detectors, signal drivers, turnout controllers, throttles, command
//! stations, dispatch panels) talk over CAN or TCP using a small set of
//! standardised messages. The most important ones are *events* (64-bit IDs
//! meaning "something happened", see [`event`]) and *traction control*
//! (driving trains, see [`traction`]).
//!
//! # Layers
//!
//! Bottom to top:
//!
//! | Module | Purpose | Standard |
//! |---|---|---|
//! | [`can`] | 29-bit CAN identifiers, [`CanFrame`], CID/RID/AMD/AME/AMR control frames | CAN Frame Transfer |
//! | [`gridconnect`] | The `:X19490ABCN;` ASCII encoding used by USB/TCP adapters, streaming [`Decoder`](gridconnect::Decoder) | (de facto) |
//! | [`alias`] | 12-bit alias candidate generator | CAN Frame Transfer |
//! | [`message`] | High-level [`Message`]s, their (multi-)frame encoding and reassembly | Message Network, CAN Frame Transfer |
//! | [`node`] | A virtual [`Node`]: alias reservation, login, standard replies, event identification | Message Network, Event Transport, SNIP |
//! | [`traction`] | Traction Control payloads, [`Speed`](traction::Speed), train search | Traction Control, Train Search |
//! | [`event`], [`node_id`], [`mti`], [`float16`] | Basic types | Event Identifiers, Unique Identifiers |
//! | `hub` *(feature `hub`)* | Async GridConnect hub over TCP, WebSocket and serial | — |
//!
//! The `rolcb-hub` binary (feature `cli`) packages the hub as a standalone
//! program.
//!
//! # Guide
//!
//! Longer, narrative documentation lives in the [`guide`] module:
//!
//! 1. [Getting started](guide::getting_started): add the crate, log a node in,
//!    send and receive your first event.
//! 2. [Architecture and the sans-IO model](guide::architecture): how the
//!    layers fit together and how to drive a node from any I/O stack.
//! 3. [Writing a throttle](guide::throttle): search → assign → speed →
//!    heartbeat → release.
//! 4. [Consuming and producing events](guide::events): a dispatch panel.
//! 5. [Running and embedding the hub](guide::hub): `rolcb-hub` flags, JMRI,
//!    using `hub::Hub::port` from your own server.
//! 6. [`no_std` and WASM](guide::no_std): using the crate in a browser or on
//!    a microcontroller.
//! 7. [Protocol reference](guide::reference): MTIs, CAN header layout, frame
//!    types, PIP bits, traction command bytes, error codes.
//!
//! # Feature flags
//!
//! | Feature | Default | Enables | Dependencies |
//! |---|---|---|---|
//! | `std` | yes | `std::error::Error` for [`ParseError`]. Without it the crate is `no_std` + `alloc`. | — |
//! | `serde` | no | `Serialize`/`Deserialize` for [`NodeId`] and [`EventId`] (as dotted-hex strings) and [`EventState`] | `serde` (no_std compatible) |
//! | `hub` | no | The async `hub` module | `std`, `tokio`, `tokio-tungstenite`, `futures-util` |
//! | `serial` | no | `hub::uplink_serial` for USB GridConnect adapters | `hub`, `tokio-serial` |
//! | `cli` | no | The `rolcb-hub` binary | `hub`, `clap` |
//!
//! # Quick example
//!
//! Two nodes on an in-memory "bus": a block detector produces an event and a
//! panel consumes it. Frames go through GridConnect text on the way, exactly
//! as they would over a USB adapter or a TCP hub.
//!
//! ```
//! use rolcb::{gridconnect, EventId, EventState, Message, Node, NodeId, NodeInfo};
//!
//! /// Deliver everything `from` wants to send to `to`; return what `to` decoded.
//! fn deliver(from: &mut Node, to: &mut Node, now_ms: u64) -> Vec<Message> {
//!     let mut got = Vec::new();
//!     for frame in from.take_outgoing() {
//!         let line = gridconnect::format(&frame);          // ":X195B4…N…;"
//!         let frame = gridconnect::parse(&line).unwrap();
//!         if let Some(rx) = to.receive(&frame, now_ms) {
//!             got.push(rx.msg);
//!         }
//!     }
//!     got
//! }
//!
//! let occupied: EventId = "05.01.01.01.22.00.00.10".parse()?;
//!
//! let mut detector = Node::new("05.01.01.01.22.00".parse()?, NodeInfo::default());
//! detector.add_producer(occupied, EventState::Invalid);
//! let mut panel = Node::new(NodeId::new(0x0501_0101_2299), NodeInfo::default());
//! panel.add_consumer(occupied, EventState::Unknown);
//!
//! // Log both nodes in: CID frames now, RID/AMD/Init Complete 200 ms later.
//! detector.start(0);
//! panel.start(0);
//! deliver(&mut detector, &mut panel, 0);
//! deliver(&mut panel, &mut detector, 0);
//! detector.poll(200);
//! panel.poll(200);
//! deliver(&mut detector, &mut panel, 200);
//! deliver(&mut panel, &mut detector, 200);
//! assert!(detector.is_permitted() && panel.is_permitted());
//!
//! // The block becomes occupied.
//! detector.set_producer_state(occupied, EventState::Valid);
//! detector.send(Message::Pcer(occupied));
//! let got = deliver(&mut detector, &mut panel, 300);
//! assert_eq!(got, vec![Message::Pcer(occupied)]);
//!
//! // The panel learned the detector's Node ID from its AMD frame.
//! assert_eq!(panel.alias_of(detector.node_id()), detector.alias());
//! # Ok::<(), rolcb::ParseError>(())
//! ```
//!
//! With real I/O the shape is the same: write `take_outgoing()` frames to
//! your port, pass every frame you read to [`Node::receive`], and call
//! [`Node::poll`] from a timer. See the
//! [architecture chapter](guide::architecture) for tokio and browser loops.

#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![warn(missing_docs)]

extern crate alloc;

pub mod alias;
pub mod can;
pub mod event;
pub mod float16;
pub mod gridconnect;
#[cfg(feature = "hub")]
#[cfg_attr(docsrs, doc(cfg(feature = "hub")))]
pub mod hub;
pub mod message;
pub mod mti;
pub mod node;
pub mod node_id;
pub mod traction;

pub mod guide {
    //! Narrative documentation: tutorials, design notes and protocol tables.
    //!
    //! These modules contain no code; they exist so the chapters render as
    //! part of the API documentation (and so their examples are compiled
    //! and run as doctests).
    //!
    //! | Chapter | Read this if you want to… |
    //! |---|---|
    //! | [Getting started](getting_started) | get a node online and exchange an event in five minutes |
    //! | [Architecture](architecture) | understand the sans-IO design and wire it to tokio, a browser or a microcontroller |
    //! | [Writing a throttle](throttle) | find, acquire, drive and release a train |
    //! | [Events](events) | build a panel or sensor that produces and consumes events |
    //! | [The hub](hub) | run `rolcb-hub` or embed the hub in your own server |
    //! | [`no_std` and WASM](no_std) | use the crate without `std` |
    //! | [Protocol reference](mod@reference) | look up MTIs, header bits, flags and codes |

    pub mod getting_started {
        #![doc = include_str!("../docs/getting-started.md")]
    }
    pub mod architecture {
        #![doc = include_str!("../docs/architecture.md")]
    }
    pub mod throttle {
        #![doc = include_str!("../docs/throttle.md")]
    }
    pub mod events {
        #![doc = include_str!("../docs/events.md")]
    }
    #[cfg_attr(feature = "hub", doc = include_str!("../docs/hub.md"))]
    #[cfg_attr(
        not(feature = "hub"),
        doc = "# Running and embedding the hub\n\n\
               This chapter documents the `hub` module and the `rolcb-hub` binary. \
               Build the documentation with `--features hub` (or `--all-features`) to see it."
    )]
    pub mod hub {}
    pub mod no_std {
        #![doc = include_str!("../docs/no-std-wasm.md")]
    }
    pub mod reference {
        #![doc = include_str!("../docs/reference.md")]
    }
}

pub use can::{CanFrame, CanHeader};
pub use event::{EventId, EventState};
pub use message::{Message, Received};
pub use mti::Mti;
pub use node::{Node, NodeInfo};
pub use node_id::NodeId;

/// Error returned when parsing textual or binary protocol data fails.
///
/// Returned by the `FromStr` impls of [`NodeId`] and [`EventId`], by
/// [`gridconnect::parse`], and by [`CanFrame::new`]. Most decoders in this
/// crate are deliberately lenient and never fail (see
/// [`Message::from_parts`]).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseError {
    /// Input was not in the expected format. The string says what was wrong.
    Format(&'static str),
    /// A hex digit was expected.
    Hex,
    /// Too many or too few bytes (or hex digits).
    Length,
}

impl core::fmt::Display for ParseError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ParseError::Format(s) => write!(f, "format error: {s}"),
            ParseError::Hex => f.write_str("invalid hex digit"),
            ParseError::Length => f.write_str("invalid length"),
        }
    }
}

#[cfg(feature = "std")]
#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
impl std::error::Error for ParseError {}

pub(crate) fn hex_val(c: u8) -> Result<u8, ParseError> {
    match c {
        b'0'..=b'9' => Ok(c - b'0'),
        b'a'..=b'f' => Ok(c - b'a' + 10),
        b'A'..=b'F' => Ok(c - b'A' + 10),
        _ => Err(ParseError::Hex),
    }
}

/// Parse "05.01.01.01.22.00" style dotted hex (dots optional) into `N` bytes.
pub(crate) fn parse_dotted<const N: usize>(s: &str) -> Result<[u8; N], ParseError> {
    let digits: alloc::vec::Vec<u8> = s
        .bytes()
        .filter(|c| !matches!(c, b'.' | b':' | b'-' | b' '))
        .collect();
    if digits.len() != N * 2 {
        return Err(ParseError::Length);
    }
    let mut out = [0u8; N];
    for (i, pair) in digits.chunks(2).enumerate() {
        out[i] = (hex_val(pair[0])? << 4) | hex_val(pair[1])?;
    }
    Ok(out)
}

pub(crate) fn fmt_dotted(bytes: &[u8], f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
    for (i, b) in bytes.iter().enumerate() {
        if i > 0 {
            f.write_str(".")?;
        }
        write!(f, "{b:02X}")?;
    }
    Ok(())
}
