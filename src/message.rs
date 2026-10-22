//! High-level OpenLCB messages and their encoding onto CAN frames.
//!
//! A [`Message`] is one OpenLCB message as defined by the *Message Network
//! Standard* and the protocol standards built on it (*Event Transport*,
//! *Traction Control*, *Simple Node Information*, *Datagram Transport*).
//! This module converts between messages and CAN frames, following the
//! *CAN Frame Transfer Standard*:
//!
//! * [`Message::try_to_frames`] encodes a message sent from a given source
//!   alias into one or more [`CanFrame`]s, rejecting messages that cannot
//!   be represented on CAN; [`Message::to_frames`] is the lenient variant.
//! * [`Assembler`] collects incoming frames, reassembles multi-frame
//!   messages, and yields [`RawMessage`]s.
//! * [`Message::from_parts`] (or `Message::from(raw)`) decodes a
//!   `RawMessage` (MTI, destination, payload) into a `Message`.
//!
//! [`Node`](crate::Node) wires these together (and resolves aliases into
//! Node IDs for [`Received`]), but they are usable on their own, for
//! example in a bus monitor.
//!
//! # Global, addressed and datagram framing
//!
//! **Global** messages (no destination, e.g. PCER) have at most 8 payload
//! bytes and always fit in one frame, with the CAN-MTI in the header.
//!
//! **Addressed** messages (MTI bit 3 set, e.g. Traction Control) start each
//! frame's data with two bytes holding a 4-bit framing flag and the 12-bit
//! destination alias, leaving 6 payload bytes per frame:
//!
//! ```text
//!  data[0] = FFFF DDDD   FFFF = framing flag, DDDD = dest alias bits 11-8
//!  data[1] = DDDD DDDD   dest alias bits 7-0
//!  data[2..8]            up to 6 payload bytes
//!
//!  flag 0x0_ : only frame        flag 0x1_ : first frame
//!  flag 0x3_ : middle frame      flag 0x2_ : last frame
//! ```
//!
//! So a 9-byte Assign Controller command becomes two frames, the first
//! flagged `0x1_` with 6 bytes and the second flagged `0x2_` with 3.
//!
//! **Datagrams** carry the destination in the header instead, so each frame
//! holds a full 8 bytes, and the framing is expressed by the frame type
//! (only / first / middle / final, see
//! [`MessageFrameType`]).
//!
//! Multi-frame messages from different senders, or to different
//! destinations, may interleave on the bus; the [`Assembler`] keys partial
//! messages by (source, destination, MTI).
//!
//! # Example
//!
//! ```
//! use rolcb::message::{Assembler, Message};
//! use rolcb::{gridconnect, EventId};
//!
//! // A global message: one frame.
//! let pcer = Message::Pcer(EventId(0x0501_0101_2200_0010));
//! let frames = pcer.to_frames(0x123);
//! assert_eq!(gridconnect::format(&frames[0]), ":X195B4123N0501010122000010;");
//!
//! // An addressed message with a 15-byte payload: three frames.
//! let reply = Message::TractionReply { dest: 0xABC, payload: (0..15).collect() };
//! let frames = reply.to_frames(0x123);
//! assert_eq!(frames.len(), 3);
//! assert_eq!(frames[0].data()[0], 0x1A); // first frame, dest bits 11-8 = 0xA
//! assert_eq!(frames[1].data()[0], 0x3A); // middle
//! assert_eq!(frames[2].data()[0], 0x2A); // last
//!
//! // Reassemble and decode.
//! let mut asm = Assembler::new();
//! let raw = frames.iter().find_map(|f| asm.accept(f)).unwrap();
//! assert_eq!(raw.src_alias, 0x123);
//! assert_eq!(Message::from(raw), reply);
//! ```

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::can::{CanFrame, CanHeader, MessageFrameType};
use crate::event::{EventId, EventState};
use crate::mti::Mti;
use crate::node_id::NodeId;

/// Protocol Support Inquiry flag bits (first three bytes of the reply, with
/// byte 0 in bits 23-16).
///
/// The *Message Network Standard*'s Protocol Identification Protocol (PIP)
/// lets a node ask another which optional protocols it implements. The
/// reply is a bit field; this crate represents the first three bytes as a
/// `u32` (byte 0 → bits 23-16, byte 1 → bits 15-8, byte 2 → bits 7-0) and
/// sends three trailing zero bytes. Combine the constants with `|` and pass
/// them to [`Node::set_protocols`](crate::Node::set_protocols); test a
/// reply's `flags` with `&`.
///
/// ```
/// use rolcb::message::protocol;
///
/// let flags = protocol::EVENT_EXCHANGE | protocol::SNIP | protocol::TRACTION_CONTROL;
/// assert_eq!(flags, 0x04_1400);
/// assert!(flags & protocol::SNIP != 0);
/// ```
pub mod protocol {
    /// Byte 0 bit 7: the node implements only the Simple Protocol subset.
    pub const SIMPLE: u32 = 0x80_0000;
    /// Byte 0 bit 6: Datagram Transport.
    pub const DATAGRAM: u32 = 0x40_0000;
    /// Byte 0 bit 5: Stream Transport.
    pub const STREAM: u32 = 0x20_0000;
    /// Byte 0 bit 4: Memory Configuration (over datagrams).
    pub const MEMORY_CONFIGURATION: u32 = 0x10_0000;
    /// Byte 0 bit 3: Reservation protocol.
    pub const RESERVATION: u32 = 0x08_0000;
    /// Byte 0 bit 2: Event Exchange (producer/consumer, *Event Transport Standard*).
    pub const EVENT_EXCHANGE: u32 = 0x04_0000;
    /// Byte 0 bit 1: Identification protocol.
    pub const IDENTIFICATION: u32 = 0x02_0000;
    /// Byte 0 bit 0: Teaching/Learning configuration (Learn Event).
    pub const TEACHING: u32 = 0x01_0000;
    /// Byte 1 bit 7: Remote Button protocol.
    pub const REMOTE_BUTTON: u32 = 0x00_8000;
    /// Byte 1 bit 6: Abbreviated Default CDI (ACDI) memory spaces.
    pub const ACDI: u32 = 0x00_4000;
    /// Byte 1 bit 5: Display protocol.
    pub const DISPLAY: u32 = 0x00_2000;
    /// Byte 1 bit 4: Simple Node Information (*Simple Node Information Standard*).
    pub const SNIP: u32 = 0x00_1000;
    /// Byte 1 bit 3: Configuration Description Information (CDI).
    pub const CDI: u32 = 0x00_0800;
    /// Byte 1 bit 2: Traction Control (*Traction Control Standard*). Train nodes set this.
    pub const TRACTION_CONTROL: u32 = 0x00_0400;
    /// Byte 1 bit 1: Function Description Information (FDI) for trains.
    pub const FDI: u32 = 0x00_0200;
    /// Byte 2 bit 7: Simple Train Node Information.
    pub const SIMPLE_TRAIN_NODE_INFO: u32 = 0x00_0080;
    /// Byte 2 bit 6: Function Configuration memory space for trains.
    pub const FUNCTION_CONFIGURATION: u32 = 0x00_0040;
}

/// A decoded OpenLCB message. Addressed messages carry the 12-bit `dest`
/// alias of the node they are sent to.
///
/// The source of a message is not part of `Message`: when sending, it is
/// the alias of the sending [`Node`](crate::Node); when receiving, it is in
/// the surrounding [`Received`]. Every variant maps to exactly one
/// [`Mti`] (see [`Message::mti`]); MTIs without a dedicated variant are
/// carried by [`Message::Other`], so decoding never fails.
///
/// Traction payloads are kept as raw bytes; decode them with
/// [`traction::Command::decode`](crate::traction::Command::decode) and
/// [`traction::Reply::decode`](crate::traction::Reply::decode). SNIP data
/// decodes with [`NodeInfo::from_snip`](crate::NodeInfo::from_snip).
///
/// ```
/// use rolcb::{EventId, EventState, Message, Mti};
///
/// let m = Message::ProducerIdentified { event: EventId(1), state: EventState::Valid };
/// assert_eq!(m.mti(), Mti::PRODUCER_IDENTIFIED_VALID);
/// assert_eq!(m.dest(), None);
/// assert_eq!(m.payload(), vec![0, 0, 0, 0, 0, 0, 0, 1]);
///
/// let q = Message::SnipRequest { dest: 0x6A1 };
/// assert_eq!(q.dest(), Some(0x6A1));
/// ```
///
/// The enum is `#[non_exhaustive]` so that new MTIs can get dedicated
/// variants without a breaking release: match it with a `_` arm (which also
/// covers [`Message::Other`]).
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Message {
    /// Initialization Complete (MTI `0x0100`/`0x0101`). The last step of a
    /// node's login; announces it is ready to communicate.
    InitializationComplete {
        /// The node that just came online.
        node_id: NodeId,
        /// True for the Simple Protocol variant (MTI `0x0101`).
        simple: bool,
    },
    /// Verify Node ID Global (MTI `0x0490`). Every node, or only the node
    /// whose ID is given, replies with [`Message::VerifiedNodeId`]. Tools use
    /// it to enumerate the network.
    VerifyNodeIdGlobal {
        /// Restrict the query to one Node ID, or `None` for everyone.
        node_id: Option<NodeId>,
    },
    /// Verify Node ID Addressed (MTI `0x0488`): ask one node to identify
    /// itself, e.g. to confirm an alias is still mapped.
    VerifyNodeIdAddressed {
        /// Destination alias.
        dest: u16,
        /// Optional Node ID payload.
        node_id: Option<NodeId>,
    },
    /// Verified Node ID (MTI `0x0170`/`0x0171`): reply to Verify Node ID.
    VerifiedNodeId {
        /// The responding node.
        node_id: NodeId,
        /// True for the Simple Protocol variant (MTI `0x0171`).
        simple: bool,
    },
    /// Optional Interaction Rejected (MTI `0x0068`): the destination sent us
    /// a request we do not support.
    OptionalInteractionRejected {
        /// Destination alias (the node whose request is being rejected).
        dest: u16,
        /// Error code, see [`crate::mti::error_code`].
        code: u16,
        /// The MTI of the rejected message.
        mti: Mti,
    },
    /// Terminate Due To Error (MTI `0x00A8`): an ongoing interaction failed.
    TerminateDueToError {
        /// Destination alias.
        dest: u16,
        /// Error code, see [`crate::mti::error_code`].
        code: u16,
        /// The MTI of the message that caused the error.
        mti: Mti,
    },
    /// Protocol Support Inquiry (MTI `0x0828`): "which protocols do you
    /// support?" [`Node`](crate::Node) answers automatically.
    ProtocolSupportInquiry {
        /// Destination alias.
        dest: u16,
    },
    /// Protocol Support Reply (MTI `0x0668`).
    ProtocolSupportReply {
        /// Destination alias (the node that asked).
        dest: u16,
        /// Supported protocols, a combination of [`protocol`] bits.
        flags: u32,
    },

    /// Identify Consumer (MTI `0x08F4`): nodes that consume this event reply
    /// with [`Message::ConsumerIdentified`].
    IdentifyConsumer(EventId),
    /// Consumer Identified (MTI `0x04C4`/`0x04C5`/`0x04C7`): the sender
    /// consumes `event`, and reports its view of the event's state.
    ConsumerIdentified {
        /// The event consumed.
        event: EventId,
        /// Current state; selects the MTI.
        state: EventState,
    },
    /// Consumer Range Identified (MTI `0x04A4`): the sender consumes a
    /// range of events. The range is encoded in the ID itself: the
    /// low-order bits that are all equal (all 1s or all 0s) are "don't care".
    ConsumerRangeIdentified(EventId),
    /// Identify Producer (MTI `0x0914`): nodes that produce this event reply
    /// with [`Message::ProducerIdentified`]. Also used for train search (see
    /// [`traction::search`](crate::traction::search)).
    IdentifyProducer(EventId),
    /// Producer Identified (MTI `0x0544`/`0x0545`/`0x0547`): the sender
    /// produces `event`, and reports whether it is currently valid.
    ProducerIdentified {
        /// The event produced.
        event: EventId,
        /// Current state; selects the MTI.
        state: EventState,
    },
    /// Producer Range Identified (MTI `0x0524`): the sender produces a range
    /// of events, encoded like [`Message::ConsumerRangeIdentified`].
    ProducerRangeIdentified(EventId),
    /// Identify Events Global (MTI `0x0970`): every node replies with
    /// Producer/Consumer Identified for all its events.
    IdentifyEventsGlobal,
    /// Identify Events Addressed (MTI `0x0968`): one node lists all its events.
    IdentifyEventsAddressed {
        /// Destination alias.
        dest: u16,
    },
    /// Learn Event (MTI `0x0594`): teaching message. A node in "learn" mode
    /// adopts this Event ID for the selected input or output.
    LearnEvent(EventId),
    /// Producer/Consumer Event Report: "this event happened".
    ///
    /// MTI `0x05B4`. The core message of the *Event Transport Standard*:
    /// a producer sends it, and every consumer of the Event ID acts on it.
    Pcer(EventId),

    /// Traction Control Command (MTI `0x05EB`), throttle → train. Build the
    /// payload with [`traction::Command::encode`](crate::traction::Command::encode).
    TractionCommand {
        /// Destination (train) alias.
        dest: u16,
        /// Raw command bytes.
        payload: Vec<u8>,
    },
    /// Traction Control Reply (MTI `0x01E9`), train → throttle. Decode the
    /// payload with [`traction::Reply::decode`](crate::traction::Reply::decode).
    TractionReply {
        /// Destination (throttle) alias.
        dest: u16,
        /// Raw reply bytes.
        payload: Vec<u8>,
    },

    /// Simple Node Information Request (MTI `0x0DE8`).
    SnipRequest {
        /// Destination alias.
        dest: u16,
    },
    /// Simple Node Information Reply (MTI `0x0A08`), usually several frames.
    SnipReply {
        /// Destination alias (the node that asked).
        dest: u16,
        /// Encoded SNIP strings, see [`NodeInfo::snip_payload`](crate::NodeInfo::snip_payload).
        data: Vec<u8>,
    },

    /// Datagram (MTI `0x1C48`): an addressed, acknowledged block of up to 72
    /// bytes (*Datagram Transport Standard*). The first byte identifies the
    /// datagram protocol (e.g. `0x20` for Memory Configuration). The
    /// receiver must answer with [`Message::DatagramReceivedOk`] or
    /// [`Message::DatagramRejected`].
    Datagram {
        /// Destination alias.
        dest: u16,
        /// Datagram contents, at most 72 bytes ([`MAX_DATAGRAM`]). Enforced
        /// by [`Message::try_to_frames`], not by [`Message::to_frames`].
        data: Vec<u8>,
    },
    /// Datagram Received OK (MTI `0x0A28`).
    DatagramReceivedOk {
        /// Destination alias (the datagram's sender).
        dest: u16,
        /// Flags byte: bit 7 = "reply pending", low nibble encodes the
        /// expected reply timeout as 2^N seconds.
        flags: u8,
    },
    /// Datagram Rejected (MTI `0x0A48`).
    DatagramRejected {
        /// Destination alias (the datagram's sender).
        dest: u16,
        /// Error code, see [`crate::mti::error_code`]. Temporary errors
        /// (`0x2xxx`) invite a resend.
        code: u16,
    },

    /// Anything not specifically decoded.
    ///
    /// Also produced for a known MTI whose payload is too short to decode
    /// (for example a PCER with fewer than 8 bytes).
    Other {
        /// The message type.
        mti: Mti,
        /// Destination alias, for addressed MTIs.
        dest: Option<u16>,
        /// Raw payload (excluding the destination bytes).
        payload: Vec<u8>,
    },
}

/// A message received from the bus, with its source.
///
/// Returned by [`Node::receive`](crate::Node::receive).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Received {
    /// The sender's 12-bit CAN alias. Use it as `dest` when replying.
    pub src_alias: u16,
    /// The sender's Node ID, if its alias mapping is known.
    ///
    /// The node learns mappings from AMD frames, Initialization Complete and
    /// Verified Node ID. If you need it and it is `None`, send
    /// [`Message::VerifyNodeIdAddressed`] to the alias, or an AME frame.
    pub src_node: Option<NodeId>,
    /// The decoded message.
    pub msg: Message,
}

fn u16_at(p: &[u8], i: usize) -> u16 {
    match p.get(i..i + 2) {
        Some(&[a, b]) => u16::from_be_bytes([a, b]),
        _ => 0,
    }
}

/// The largest datagram the *Datagram Transport Standard* allows (72 bytes).
pub const MAX_DATAGRAM: usize = 72;

/// Why [`Message::try_to_frames`] refused to encode a message.
///
/// ```
/// use rolcb::{EncodeError, Message, Mti};
///
/// let big = Message::Datagram { dest: 0x456, data: vec![0; 73] };
/// assert_eq!(big.try_to_frames(0x123), Err(EncodeError::DatagramTooLong { len: 73 }));
///
/// let no_dest = Message::Other { mti: Mti(0x0DA8), dest: None, payload: vec![] };
/// assert_eq!(
///     no_dest.try_to_frames(0x123),
///     Err(EncodeError::MissingDestination { mti: Mti(0x0DA8) })
/// );
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum EncodeError {
    /// A global message's payload is longer than one CAN frame (8 bytes).
    GlobalPayloadTooLong {
        /// The payload length.
        len: usize,
    },
    /// The MTI is addressed (bit 3 set) but the message has no destination.
    MissingDestination {
        /// The MTI.
        mti: Mti,
    },
    /// The MTI is global but the message carries a destination.
    UnexpectedDestination {
        /// The MTI.
        mti: Mti,
    },
    /// The MTI has bits above bit 11 set and is not [`Mti::DATAGRAM`], so it
    /// does not fit the 12-bit CAN-MTI field (e.g. stream MTIs, which need
    /// stream frames this crate does not build).
    MtiTooWide {
        /// The MTI.
        mti: Mti,
    },
    /// A datagram is longer than [`MAX_DATAGRAM`] (72) bytes.
    DatagramTooLong {
        /// The payload length.
        len: usize,
    },
}

impl core::fmt::Display for EncodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EncodeError::GlobalPayloadTooLong { len } => {
                write!(f, "global message payload of {len} bytes exceeds 8")
            }
            EncodeError::MissingDestination { mti } => {
                write!(f, "addressed MTI {mti} without a destination")
            }
            EncodeError::UnexpectedDestination { mti } => {
                write!(f, "global MTI {mti} with a destination")
            }
            EncodeError::MtiTooWide { mti } => {
                write!(f, "MTI {mti} does not fit the 12-bit CAN-MTI")
            }
            EncodeError::DatagramTooLong { len } => {
                write!(f, "datagram of {len} bytes exceeds {MAX_DATAGRAM}")
            }
        }
    }
}

impl core::error::Error for EncodeError {}

impl Message {
    /// The MTI this message is sent with.
    #[must_use]
    pub fn mti(&self) -> Mti {
        use Message::*;
        match self {
            InitializationComplete { simple: false, .. } => Mti::INITIALIZATION_COMPLETE,
            InitializationComplete { simple: true, .. } => Mti::INITIALIZATION_COMPLETE_SIMPLE,
            VerifyNodeIdGlobal { .. } => Mti::VERIFY_NODE_ID_GLOBAL,
            VerifyNodeIdAddressed { .. } => Mti::VERIFY_NODE_ID_ADDRESSED,
            VerifiedNodeId { simple: false, .. } => Mti::VERIFIED_NODE_ID,
            VerifiedNodeId { simple: true, .. } => Mti::VERIFIED_NODE_ID_SIMPLE,
            OptionalInteractionRejected { .. } => Mti::OPTIONAL_INTERACTION_REJECTED,
            TerminateDueToError { .. } => Mti::TERMINATE_DUE_TO_ERROR,
            ProtocolSupportInquiry { .. } => Mti::PROTOCOL_SUPPORT_INQUIRY,
            ProtocolSupportReply { .. } => Mti::PROTOCOL_SUPPORT_REPLY,
            IdentifyConsumer(_) => Mti::IDENTIFY_CONSUMER,
            ConsumerIdentified { state, .. } => match state {
                EventState::Valid => Mti::CONSUMER_IDENTIFIED_VALID,
                EventState::Invalid => Mti::CONSUMER_IDENTIFIED_INVALID,
                EventState::Unknown => Mti::CONSUMER_IDENTIFIED_UNKNOWN,
            },
            ConsumerRangeIdentified(_) => Mti::CONSUMER_RANGE_IDENTIFIED,
            IdentifyProducer(_) => Mti::IDENTIFY_PRODUCER,
            ProducerIdentified { state, .. } => match state {
                EventState::Valid => Mti::PRODUCER_IDENTIFIED_VALID,
                EventState::Invalid => Mti::PRODUCER_IDENTIFIED_INVALID,
                EventState::Unknown => Mti::PRODUCER_IDENTIFIED_UNKNOWN,
            },
            ProducerRangeIdentified(_) => Mti::PRODUCER_RANGE_IDENTIFIED,
            IdentifyEventsGlobal => Mti::IDENTIFY_EVENTS_GLOBAL,
            IdentifyEventsAddressed { .. } => Mti::IDENTIFY_EVENTS_ADDRESSED,
            LearnEvent(_) => Mti::LEARN_EVENT,
            Pcer(_) => Mti::PCER,
            TractionCommand { .. } => Mti::TRACTION_CONTROL_COMMAND,
            TractionReply { .. } => Mti::TRACTION_CONTROL_REPLY,
            SnipRequest { .. } => Mti::SNIP_REQUEST,
            SnipReply { .. } => Mti::SNIP_REPLY,
            Datagram { .. } => Mti::DATAGRAM,
            DatagramReceivedOk { .. } => Mti::DATAGRAM_RECEIVED_OK,
            DatagramRejected { .. } => Mti::DATAGRAM_REJECTED,
            Other { mti, .. } => *mti,
        }
    }

    /// Destination alias for addressed messages; `None` for global ones.
    #[must_use]
    pub fn dest(&self) -> Option<u16> {
        use Message::*;
        match self {
            VerifyNodeIdAddressed { dest, .. }
            | OptionalInteractionRejected { dest, .. }
            | TerminateDueToError { dest, .. }
            | ProtocolSupportInquiry { dest }
            | ProtocolSupportReply { dest, .. }
            | IdentifyEventsAddressed { dest }
            | TractionCommand { dest, .. }
            | TractionReply { dest, .. }
            | SnipRequest { dest }
            | SnipReply { dest, .. }
            | Datagram { dest, .. }
            | DatagramReceivedOk { dest, .. }
            | DatagramRejected { dest, .. } => Some(*dest),
            Other { dest, .. } => *dest,
            _ => None,
        }
    }

    /// The message payload (for addressed messages, excluding the destination).
    ///
    /// This is the byte sequence defined by the relevant standard: a Node
    /// ID, an Event ID, error code + MTI, and so on. The Protocol Support
    /// Reply is padded to six bytes.
    #[must_use]
    pub fn payload(&self) -> Vec<u8> {
        use Message::*;
        match self {
            InitializationComplete { node_id, .. } | VerifiedNodeId { node_id, .. } => {
                node_id.to_bytes().to_vec()
            }
            VerifyNodeIdGlobal { node_id } | VerifyNodeIdAddressed { node_id, .. } => {
                node_id.map(|n| n.to_bytes().to_vec()).unwrap_or_default()
            }
            OptionalInteractionRejected { code, mti, .. }
            | TerminateDueToError { code, mti, .. } => {
                let mut v = code.to_be_bytes().to_vec();
                v.extend_from_slice(&mti.0.to_be_bytes());
                v
            }
            ProtocolSupportReply { flags, .. } => {
                let b = flags.to_be_bytes();
                alloc::vec![b[1], b[2], b[3], 0, 0, 0]
            }
            IdentifyConsumer(e)
            | ConsumerIdentified { event: e, .. }
            | ConsumerRangeIdentified(e)
            | IdentifyProducer(e)
            | ProducerIdentified { event: e, .. }
            | ProducerRangeIdentified(e)
            | LearnEvent(e)
            | Pcer(e) => e.to_bytes().to_vec(),
            TractionCommand { payload, .. } | TractionReply { payload, .. } => payload.clone(),
            SnipReply { data, .. } | Datagram { data, .. } => data.clone(),
            DatagramReceivedOk { flags, .. } => alloc::vec![*flags],
            DatagramRejected { code, .. } => code.to_be_bytes().to_vec(),
            Other { payload, .. } => payload.clone(),
            ProtocolSupportInquiry { .. }
            | IdentifyEventsGlobal
            | IdentifyEventsAddressed { .. }
            | SnipRequest { .. } => Vec::new(),
        }
    }

    /// Decode from MTI, destination and payload.
    ///
    /// This is the inverse of [`Message::mti`] + [`Message::dest`] +
    /// [`Message::payload`]. It never fails: unknown MTIs and payloads too
    /// short for their MTI become [`Message::Other`]. `dest` should be
    /// `Some` for addressed MTIs (a missing destination decodes as alias 0).
    /// Usually fed from a [`RawMessage`] produced by an [`Assembler`]; the
    /// `From<RawMessage>` impl does exactly that.
    ///
    /// ```
    /// use rolcb::{EventId, Message, Mti};
    ///
    /// let m = Message::from_parts(Mti::PCER, None, vec![5, 1, 1, 1, 0x22, 0, 0, 0x10]);
    /// assert_eq!(m, Message::Pcer(EventId(0x0501_0101_2200_0010)));
    ///
    /// // Too short to hold an Event ID: kept as Other.
    /// let m = Message::from_parts(Mti::PCER, None, vec![1, 2]);
    /// assert!(matches!(m, Message::Other { .. }));
    /// ```
    #[must_use]
    pub fn from_parts(mti: Mti, dest: Option<u16>, p: Vec<u8>) -> Message {
        use Message::*;
        let d = dest.unwrap_or(0);
        let node = NodeId::from_bytes(&p).ok();
        let event = EventId::from_bytes(&p).ok();
        let simple = mti.0 & 1 == 1;
        match (mti, node, event) {
            (
                Mti::INITIALIZATION_COMPLETE | Mti::INITIALIZATION_COMPLETE_SIMPLE,
                Some(node_id),
                _,
            ) => InitializationComplete { node_id, simple },
            (Mti::VERIFY_NODE_ID_GLOBAL, node_id, _) => VerifyNodeIdGlobal { node_id },
            (Mti::VERIFY_NODE_ID_ADDRESSED, node_id, _) => {
                VerifyNodeIdAddressed { dest: d, node_id }
            }
            (Mti::VERIFIED_NODE_ID | Mti::VERIFIED_NODE_ID_SIMPLE, Some(node_id), _) => {
                VerifiedNodeId { node_id, simple }
            }
            (Mti::OPTIONAL_INTERACTION_REJECTED, ..) => OptionalInteractionRejected {
                dest: d,
                code: u16_at(&p, 0),
                mti: Mti(u16_at(&p, 2)),
            },
            (Mti::TERMINATE_DUE_TO_ERROR, ..) => TerminateDueToError {
                dest: d,
                code: u16_at(&p, 0),
                mti: Mti(u16_at(&p, 2)),
            },
            (Mti::PROTOCOL_SUPPORT_INQUIRY, ..) => ProtocolSupportInquiry { dest: d },
            (Mti::PROTOCOL_SUPPORT_REPLY, ..) => {
                let b = |i: usize| p.get(i).copied().unwrap_or(0) as u32;
                ProtocolSupportReply {
                    dest: d,
                    flags: (b(0) << 16) | (b(1) << 8) | b(2),
                }
            }
            (Mti::IDENTIFY_CONSUMER, _, Some(e)) => IdentifyConsumer(e),
            (Mti::CONSUMER_IDENTIFIED_VALID, _, Some(event)) => ConsumerIdentified {
                event,
                state: EventState::Valid,
            },
            (Mti::CONSUMER_IDENTIFIED_INVALID, _, Some(event)) => ConsumerIdentified {
                event,
                state: EventState::Invalid,
            },
            (Mti::CONSUMER_IDENTIFIED_UNKNOWN, _, Some(event)) => ConsumerIdentified {
                event,
                state: EventState::Unknown,
            },
            (Mti::CONSUMER_RANGE_IDENTIFIED, _, Some(e)) => ConsumerRangeIdentified(e),
            (Mti::IDENTIFY_PRODUCER, _, Some(e)) => IdentifyProducer(e),
            (Mti::PRODUCER_IDENTIFIED_VALID, _, Some(event)) => ProducerIdentified {
                event,
                state: EventState::Valid,
            },
            (Mti::PRODUCER_IDENTIFIED_INVALID, _, Some(event)) => ProducerIdentified {
                event,
                state: EventState::Invalid,
            },
            (Mti::PRODUCER_IDENTIFIED_UNKNOWN, _, Some(event)) => ProducerIdentified {
                event,
                state: EventState::Unknown,
            },
            (Mti::PRODUCER_RANGE_IDENTIFIED, _, Some(e)) => ProducerRangeIdentified(e),
            (Mti::IDENTIFY_EVENTS_GLOBAL, ..) => IdentifyEventsGlobal,
            (Mti::IDENTIFY_EVENTS_ADDRESSED, ..) => IdentifyEventsAddressed { dest: d },
            (Mti::LEARN_EVENT, _, Some(e)) => LearnEvent(e),
            (Mti::PCER, _, Some(e)) => Pcer(e),
            (Mti::TRACTION_CONTROL_COMMAND, ..) => TractionCommand {
                dest: d,
                payload: p,
            },
            (Mti::TRACTION_CONTROL_REPLY, ..) => TractionReply {
                dest: d,
                payload: p,
            },
            (Mti::SNIP_REQUEST, ..) => SnipRequest { dest: d },
            (Mti::SNIP_REPLY, ..) => SnipReply { dest: d, data: p },
            (Mti::DATAGRAM, ..) => Datagram { dest: d, data: p },
            (Mti::DATAGRAM_RECEIVED_OK, ..) => DatagramReceivedOk {
                dest: d,
                flags: p.first().copied().unwrap_or(0),
            },
            (Mti::DATAGRAM_REJECTED, ..) => DatagramRejected {
                dest: d,
                code: u16_at(&p, 0),
            },
            _ => Other {
                mti,
                dest,
                payload: p,
            },
        }
    }

    /// Check that this message can be represented on CAN.
    fn check(&self, payload_len: usize) -> Result<(), EncodeError> {
        let mti = self.mti();
        let dest = self.dest();
        if mti == Mti::DATAGRAM {
            if dest.is_none() {
                return Err(EncodeError::MissingDestination { mti });
            }
            if payload_len > MAX_DATAGRAM {
                return Err(EncodeError::DatagramTooLong { len: payload_len });
            }
            return Ok(());
        }
        if mti.0 > 0x0FFF {
            return Err(EncodeError::MtiTooWide { mti });
        }
        match (mti.is_addressed(), dest) {
            (true, None) => Err(EncodeError::MissingDestination { mti }),
            (false, Some(_)) => Err(EncodeError::UnexpectedDestination { mti }),
            (false, None) if payload_len > 8 => {
                Err(EncodeError::GlobalPayloadTooLong { len: payload_len })
            }
            _ => Ok(()),
        }
    }

    /// Encode as one or more CAN frames sent from `src_alias`, rejecting
    /// messages that cannot be represented faithfully.
    ///
    /// Framing is the same as [`Message::to_frames`]. The checks are:
    ///
    /// | Problem | Error |
    /// |---|---|
    /// | global message with more than 8 payload bytes | [`EncodeError::GlobalPayloadTooLong`] |
    /// | addressed MTI with no `dest` ([`Message::Other`] only) | [`EncodeError::MissingDestination`] |
    /// | global MTI with a `dest` ([`Message::Other`] only) | [`EncodeError::UnexpectedDestination`] |
    /// | MTI with bit 12 or higher set, other than [`Mti::DATAGRAM`] | [`EncodeError::MtiTooWide`] |
    /// | datagram over 72 bytes | [`EncodeError::DatagramTooLong`] |
    ///
    /// # Errors
    ///
    /// Returns the first [`EncodeError`] from the table above.
    ///
    /// ```
    /// use rolcb::{EncodeError, Message, Mti};
    ///
    /// let ok = Message::Datagram { dest: 0x456, data: vec![0x20; 72] };
    /// assert_eq!(ok.try_to_frames(0x123)?.len(), 9);
    ///
    /// let wide = Message::Other { mti: Mti(0x1F88), dest: Some(0x456), payload: vec![] };
    /// assert!(matches!(wide.try_to_frames(0x123), Err(EncodeError::MtiTooWide { .. })));
    /// # Ok::<(), EncodeError>(())
    /// ```
    pub fn try_to_frames(&self, src_alias: u16) -> Result<Vec<CanFrame>, EncodeError> {
        let payload = self.payload();
        self.check(payload.len())?;
        Ok(self.frames_from(src_alias, &payload))
    }

    /// Encode as one or more CAN frames sent from `src_alias`.
    ///
    /// * Global messages produce one frame (payload truncated to 8 bytes).
    /// * Addressed messages produce one frame per 6 payload bytes (at least
    ///   one), each starting with the framing flag and destination alias as
    ///   described in the [module docs](self#global-addressed-and-datagram-framing).
    /// * Datagrams produce one frame per 8 bytes using the datagram frame
    ///   types.
    ///
    /// This encoder is lenient and never fails. For a message that
    /// [`Message::try_to_frames`] would reject it keeps its historical
    /// behaviour: global payloads are truncated to 8 bytes, an addressed MTI
    /// without a destination is sent as if it were global, a destination on
    /// a global MTI is sent as addressed framing, bits 12-15 of the MTI are
    /// dropped, and datagrams over 72 bytes are split into as many frames as
    /// needed. In debug builds such a message trips a `debug_assert!`; use
    /// `try_to_frames` when the message comes from untrusted input.
    ///
    /// # Panics
    ///
    /// In debug builds only, if `try_to_frames` would return an error.
    ///
    /// ```
    /// use rolcb::{gridconnect, Message, NodeId};
    ///
    /// let m = Message::VerifyNodeIdAddressed { dest: 0x456, node_id: None };
    /// let f = m.to_frames(0x123);
    /// assert_eq!(gridconnect::format(&f[0]), ":X19488123N0456;");
    ///
    /// // A 20-byte datagram: frame types 3 (first), 4 (middle), 5 (final).
    /// let d = Message::Datagram { dest: 0x456, data: (0..20).collect() };
    /// let ids: Vec<u32> = d.to_frames(0x123).iter().map(|f| f.id).collect();
    /// assert_eq!(ids, [0x1B45_6123, 0x1C45_6123, 0x1D45_6123]);
    /// ```
    #[must_use]
    pub fn to_frames(&self, src_alias: u16) -> Vec<CanFrame> {
        let payload = self.payload();
        debug_assert!(
            self.check(payload.len()).is_ok(),
            "Message::to_frames: {:?} for {:?}; use try_to_frames",
            self.check(payload.len()),
            self
        );
        self.frames_from(src_alias, &payload)
    }

    /// The lenient encoder behind [`Message::to_frames`], without the debug
    /// assertion. Used by [`Node`](crate::Node).
    pub(crate) fn to_frames_lenient(&self, src_alias: u16) -> Vec<CanFrame> {
        self.frames_from(src_alias, &self.payload())
    }

    fn frames_from(&self, src_alias: u16, payload: &[u8]) -> Vec<CanFrame> {
        let mti = self.mti();

        if mti == Mti::DATAGRAM {
            let dest = self.dest().unwrap_or(0);
            let n = payload.len().div_ceil(8).max(1);
            let mut out = Vec::with_capacity(n);
            for i in 0..n {
                let chunk = payload
                    .get(i * 8..payload.len().min(i * 8 + 8))
                    .unwrap_or(&[]);
                let kind = match (n, i) {
                    (1, _) => MessageFrameType::DatagramOnly,
                    (_, 0) => MessageFrameType::DatagramFirst,
                    (_, i) if i == n - 1 => MessageFrameType::DatagramFinal,
                    _ => MessageFrameType::DatagramMiddle,
                };
                let id = CanHeader::Datagram {
                    kind,
                    dest,
                    alias: src_alias,
                }
                .encode();
                out.push(CanFrame::of(id, chunk));
            }
            return out;
        }

        let id = CanHeader::Message {
            mti,
            alias: src_alias,
        }
        .encode();
        match self.dest() {
            None => {
                let n = payload.len().min(8);
                alloc::vec![CanFrame::of(id, &payload[..n])]
            }
            Some(dest) => {
                let n = payload.len().div_ceil(6).max(1);
                let mut out = Vec::with_capacity(n);
                for i in 0..n {
                    let chunk = payload
                        .get(i * 6..payload.len().min(i * 6 + 6))
                        .unwrap_or(&[]);
                    let flags: u8 = match (n, i) {
                        (1, _) => 0x00,
                        (_, 0) => 0x10,
                        (_, i) if i == n - 1 => 0x20,
                        _ => 0x30,
                    };
                    let mut d = [0u8; 8];
                    d[0] = flags | ((dest >> 8) as u8 & 0x0F);
                    d[1] = dest as u8;
                    d[2..2 + chunk.len()].copy_from_slice(chunk);
                    out.push(CanFrame::of(id, &d[..2 + chunk.len()]));
                }
                out
            }
        }
    }
}

/// A complete message reassembled from frames, before decoding.
///
/// Turn it into a [`Message`] with `Message::from(raw)` (or
/// `Message::from_parts(raw.mti, raw.dest, raw.payload)`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawMessage {
    /// Source alias from the CAN header.
    pub src_alias: u16,
    /// The message type ([`Mti::DATAGRAM`] for datagram frames).
    pub mti: Mti,
    /// Destination alias for addressed messages and datagrams.
    pub dest: Option<u16>,
    /// The payload, with the addressed-message flag/destination bytes removed.
    pub payload: Vec<u8>,
}

impl From<RawMessage> for Message {
    /// Decode with [`Message::from_parts`]. The source alias is dropped.
    fn from(raw: RawMessage) -> Message {
        Message::from_parts(raw.mti, raw.dest, raw.payload)
    }
}

const MAX_PARTIALS: usize = 32;
const MAX_PAYLOAD: usize = 256;

/// How long [`Assembler::accept_at`] keeps a partial multi-frame message
/// that receives no further frames (3 s).
pub const PARTIAL_TIMEOUT_MS: u64 = 3_000;

type PartialKey = (u16, u16, u16);

#[derive(Debug)]
struct Partial {
    data: Vec<u8>,
    /// Time of the last frame added (`accept_at` clock).
    last_ms: u64,
    /// Insertion order, for evicting the oldest entry.
    seq: u64,
    /// Started by a legacy "only"-flagged SNIP reply frame.
    legacy_snip: bool,
}

/// Reassembles multi-frame addressed messages and datagrams.
///
/// Feed every frame to [`Assembler::accept_at`] (or [`Assembler::accept`]).
/// Single-frame messages come straight back; multi-frame ones are buffered
/// until their last frame. Partial messages are keyed by (source alias,
/// destination alias, MTI), so interleaved transfers from different nodes
/// are handled.
///
/// The assembler is defensive about memory:
///
/// * at most 32 partial messages are kept; when a new one starts at the
///   cap, the one that *started* longest ago is dropped;
/// * a partial message that would exceed 256 bytes is discarded;
/// * with [`accept_at`](Assembler::accept_at), a partial message that gets
///   no new frame for [`PARTIAL_TIMEOUT_MS`] (3 s) is dropped. Plain
///   [`accept`](Assembler::accept) has no clock and never expires anything.
///
/// A middle or last frame with no matching first frame is ignored. It is a
/// passive listener, so it reassembles messages for *every* destination,
/// not just one node.
///
/// # Legacy SNIP replies
///
/// Some early nodes send a multi-frame SNIP reply with every frame flagged
/// "only" (`0x0_`) instead of first/middle/last. A complete SNIP payload
/// never fits in one addressed frame (it has at least eight bytes), so the
/// assembler concatenates "only" SNIP reply frames from the same source to
/// the same destination until the payload holds six NUL terminators, then
/// returns it. A truncated reply of that kind is held until it expires or
/// is evicted rather than returned early.
///
/// ```
/// use rolcb::message::{Assembler, Message};
/// use rolcb::traction::Command;
/// use rolcb::NodeId;
///
/// let cmd = Message::TractionCommand {
///     dest: 0x6A1,
///     payload: Command::AssignController { flags: 0, controller: NodeId::new(0x0501_0101_2299) }
///         .encode(),
/// };
/// let frames = cmd.to_frames(0x285);
/// assert_eq!(frames.len(), 2); // 9 bytes → 6 + 3
///
/// let mut asm = Assembler::new();
/// assert!(asm.accept_at(&frames[0], 1_000).is_none());
/// let raw = asm.accept_at(&frames[1], 1_010).expect("complete");
/// assert_eq!(raw.dest, Some(0x6A1));
/// assert_eq!(Message::from(raw), cmd);
///
/// // A first frame whose continuation never arrives is forgotten after 3 s.
/// assert!(asm.accept_at(&frames[0], 2_000).is_none());
/// assert!(asm.accept_at(&frames[1], 5_000).is_none());
/// ```
#[derive(Debug, Default)]
pub struct Assembler {
    partial: BTreeMap<PartialKey, Partial>,
    next_seq: u64,
    now_ms: u64,
}

impl Assembler {
    /// An assembler with no partial messages.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Accept one frame without a clock. Returns a message when one is
    /// complete. Control frames (CID/RID/AMD...) return `None`; the node
    /// handles those.
    ///
    /// Partial messages accepted this way **never expire**; they leave only
    /// when completed, evicted at the cap, or [`forget`](Assembler::forget)
    /// is called. Prefer [`Assembler::accept_at`] in long-running programs.
    #[must_use]
    pub fn accept(&mut self, frame: &CanFrame) -> Option<RawMessage> {
        self.accept_inner(frame)
    }

    /// Accept one frame received at `now_ms` (any monotonic millisecond
    /// clock). Like [`Assembler::accept`], but first drops partial messages
    /// that have had no new frame for [`PARTIAL_TIMEOUT_MS`].
    #[must_use]
    pub fn accept_at(&mut self, frame: &CanFrame, now_ms: u64) -> Option<RawMessage> {
        self.now_ms = now_ms;
        self.partial
            .retain(|_, p| now_ms.saturating_sub(p.last_ms) < PARTIAL_TIMEOUT_MS);
        self.accept_inner(frame)
    }

    /// Number of partial messages currently buffered.
    #[must_use]
    pub fn partial_count(&self) -> usize {
        self.partial.len()
    }

    fn accept_inner(&mut self, frame: &CanFrame) -> Option<RawMessage> {
        let data = frame.data();
        match frame.header() {
            CanHeader::Message { mti, alias } => {
                if !mti.is_addressed() {
                    return Some(RawMessage {
                        src_alias: alias,
                        mti,
                        dest: None,
                        payload: data.to_vec(),
                    });
                }
                let [b0, b1, body @ ..] = data else {
                    return None;
                };
                let dest = (((b0 & 0x0F) as u16) << 8) | *b1 as u16;
                let flags = b0 & 0x30;
                let key = (alias, dest, mti.0);
                let legacy = mti == Mti::SNIP_REPLY;
                self.collect(key, flags, body, legacy)
                    .map(|payload| RawMessage {
                        src_alias: alias,
                        mti,
                        dest: Some(dest),
                        payload,
                    })
            }
            CanHeader::Datagram { kind, dest, alias } => {
                let flags = match kind {
                    MessageFrameType::DatagramOnly => 0x00,
                    MessageFrameType::DatagramFirst => 0x10,
                    MessageFrameType::DatagramFinal => 0x20,
                    _ => 0x30,
                };
                let key = (alias, dest, Mti::DATAGRAM.0);
                self.collect(key, flags, data, false)
                    .map(|payload| RawMessage {
                        src_alias: alias,
                        mti: Mti::DATAGRAM,
                        dest: Some(dest),
                        payload,
                    })
            }
            _ => None,
        }
    }

    fn insert(&mut self, key: PartialKey, data: Vec<u8>, legacy_snip: bool) {
        if self.partial.len() >= MAX_PARTIALS && !self.partial.contains_key(&key) {
            // Drop the entry that started longest ago to bound memory.
            let oldest = self
                .partial
                .iter()
                .min_by_key(|(_, p)| p.seq)
                .map(|(k, _)| *k);
            if let Some(k) = oldest {
                self.partial.remove(&k);
            }
        }
        let seq = self.next_seq;
        self.next_seq += 1;
        self.partial.insert(
            key,
            Partial {
                data,
                last_ms: self.now_ms,
                seq,
                legacy_snip,
            },
        );
    }

    fn collect(&mut self, key: PartialKey, flags: u8, body: &[u8], snip: bool) -> Option<Vec<u8>> {
        match flags {
            0x00 if snip => {
                // Legacy SNIP: "only" frames concatenated until 6 NULs.
                let mut buf = match self.partial.remove(&key) {
                    Some(p) if p.legacy_snip => p.data,
                    _ => Vec::new(),
                };
                buf.extend_from_slice(body);
                if buf.iter().filter(|&&b| b == 0).count() >= 6 {
                    return Some(buf);
                }
                if buf.len() <= MAX_PAYLOAD {
                    self.insert(key, buf, true);
                }
                None
            }
            0x00 => {
                self.partial.remove(&key);
                Some(body.to_vec())
            }
            0x10 => {
                self.insert(key, body.to_vec(), false);
                None
            }
            0x30 => {
                let now = self.now_ms;
                match self.partial.get_mut(&key) {
                    Some(p) if !p.legacy_snip && p.data.len() + body.len() <= MAX_PAYLOAD => {
                        p.data.extend_from_slice(body);
                        p.last_ms = now;
                    }
                    Some(p) if !p.legacy_snip => {
                        self.partial.remove(&key);
                    }
                    _ => {}
                }
                None
            }
            _ => match self.partial.remove(&key) {
                Some(p) if !p.legacy_snip => {
                    let mut buf = p.data;
                    buf.extend_from_slice(body);
                    Some(buf)
                }
                _ => None,
            },
        }
    }

    /// Forget partial messages from `alias` (e.g. after it reset).
    ///
    /// [`Node`](crate::Node) calls this when it sees an AMR for the alias.
    pub fn forget(&mut self, alias: u16) {
        self.partial.retain(|k, _| k.0 != alias);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gridconnect;
    use alloc::vec;

    #[test]
    fn global_encoding() {
        let m = Message::Pcer("05.01.01.01.22.00.00.10".parse().unwrap());
        let f = m.to_frames(0x123);
        assert_eq!(f.len(), 1);
        assert_eq!(gridconnect::format(&f[0]), ":X195B4123N0501010122000010;");

        let m = Message::InitializationComplete {
            node_id: NodeId::new(0x0501_0101_2200),
            simple: false,
        };
        assert_eq!(
            gridconnect::format(&m.to_frames(0x285)[0]),
            ":X19100285N050101012200;"
        );
    }

    #[test]
    fn addressed_single_frame() {
        let m = Message::VerifyNodeIdAddressed {
            dest: 0x456,
            node_id: None,
        };
        let f = m.to_frames(0x123);
        assert_eq!(gridconnect::format(&f[0]), ":X19488123N0456;");
    }

    #[test]
    fn addressed_multi_frame_round_trip() {
        let payload: Vec<u8> = (0..15).collect();
        let m = Message::TractionReply {
            dest: 0xABC,
            payload: payload.clone(),
        };
        let frames = m.to_frames(0x123);
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0].data()[0], 0x1A);
        assert_eq!(frames[1].data()[0], 0x3A);
        assert_eq!(frames[2].data()[0], 0x2A);
        let mut a = Assembler::new();
        assert!(a.accept(&frames[0]).is_none());
        assert!(a.accept(&frames[1]).is_none());
        let raw = a.accept(&frames[2]).unwrap();
        assert_eq!(raw.src_alias, 0x123);
        assert_eq!(raw.dest, Some(0xABC));
        assert_eq!(Message::from_parts(raw.mti, raw.dest, raw.payload), m);
    }

    #[test]
    fn datagram_round_trip() {
        let m = Message::Datagram {
            dest: 0x456,
            data: (0..20).collect(),
        };
        let frames = m.to_frames(0x123);
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0].id, 0x1B45_6123);
        assert_eq!(frames[1].id, 0x1C45_6123);
        assert_eq!(frames[2].id, 0x1D45_6123);
        let mut a = Assembler::new();
        let raws: Vec<_> = frames.iter().filter_map(|f| a.accept(f)).collect();
        assert_eq!(raws.len(), 1);
        assert_eq!(
            Message::from_parts(raws[0].mti, raws[0].dest, raws[0].payload.clone()),
            m
        );
    }

    #[test]
    fn decode_all_event_messages() {
        let e = EventId(0x0102_0304_0506_0708);
        for m in [
            Message::IdentifyConsumer(e),
            Message::ConsumerIdentified {
                event: e,
                state: EventState::Valid,
            },
            Message::ConsumerIdentified {
                event: e,
                state: EventState::Invalid,
            },
            Message::ConsumerIdentified {
                event: e,
                state: EventState::Unknown,
            },
            Message::IdentifyProducer(e),
            Message::ProducerIdentified {
                event: e,
                state: EventState::Valid,
            },
            Message::ProducerIdentified {
                event: e,
                state: EventState::Invalid,
            },
            Message::ProducerIdentified {
                event: e,
                state: EventState::Unknown,
            },
            Message::LearnEvent(e),
            Message::Pcer(e),
            Message::IdentifyEventsGlobal,
            Message::IdentifyEventsAddressed { dest: 0x777 },
            Message::ProtocolSupportReply {
                dest: 0x777,
                flags: protocol::EVENT_EXCHANGE | protocol::SNIP,
            },
            Message::OptionalInteractionRejected {
                dest: 0x777,
                code: 0x1040,
                mti: Mti::DATAGRAM,
            },
        ] {
            let mut a = Assembler::new();
            let raws: Vec<_> = m
                .to_frames(0x321)
                .iter()
                .filter_map(|f| a.accept(f))
                .collect();
            assert_eq!(raws.len(), 1, "{m:?}");
            assert_eq!(
                Message::from_parts(raws[0].mti, raws[0].dest, raws[0].payload.clone()),
                m
            );
        }
    }

    #[test]
    fn interleaved_senders() {
        let a1 = Message::TractionReply {
            dest: 0x100,
            payload: vec![1; 10],
        };
        let a2 = Message::TractionReply {
            dest: 0x100,
            payload: vec![2; 10],
        };
        let f1 = a1.to_frames(0x201);
        let f2 = a2.to_frames(0x202);
        let mut a = Assembler::new();
        assert!(a.accept(&f1[0]).is_none());
        assert!(a.accept(&f2[0]).is_none());
        assert_eq!(a.accept(&f2[1]).unwrap().payload, vec![2; 10]);
        assert_eq!(a.accept(&f1[1]).unwrap().payload, vec![1; 10]);
    }

    fn round_trip(m: &Message) -> Message {
        let frames = m.try_to_frames(0x321).expect("encodable");
        assert_eq!(frames, m.to_frames(0x321));
        let mut a = Assembler::new();
        let mut raws: Vec<_> = frames.iter().filter_map(|f| a.accept_at(f, 5)).collect();
        assert_eq!(raws.len(), 1, "{m:?} gave {raws:?}");
        assert_eq!(a.partial_count(), 0);
        let raw = raws.remove(0);
        assert_eq!(raw.src_alias, 0x321);
        assert_eq!(raw.mti, m.mti());
        assert_eq!(raw.dest, m.dest());
        Message::from(raw)
    }

    #[test]
    fn every_variant_round_trips() {
        let n = NodeId::new(0x0501_0101_2200);
        let e = EventId(0x0102_0304_0506_0708);
        let snip = crate::NodeInfo {
            manufacturer: "Uncommon Models".into(),
            model: "Panel".into(),
            ..Default::default()
        }
        .snip_payload();
        let mut all = vec![
            Message::InitializationComplete {
                node_id: n,
                simple: false,
            },
            Message::InitializationComplete {
                node_id: n,
                simple: true,
            },
            Message::VerifyNodeIdGlobal { node_id: None },
            Message::VerifyNodeIdGlobal { node_id: Some(n) },
            Message::VerifyNodeIdAddressed {
                dest: 0x777,
                node_id: None,
            },
            Message::VerifyNodeIdAddressed {
                dest: 0x777,
                node_id: Some(n),
            },
            Message::VerifiedNodeId {
                node_id: n,
                simple: false,
            },
            Message::VerifiedNodeId {
                node_id: n,
                simple: true,
            },
            Message::OptionalInteractionRejected {
                dest: 0xFFF,
                code: 0x1043,
                mti: Mti::DATAGRAM,
            },
            Message::TerminateDueToError {
                dest: 0x001,
                code: 0x2041,
                mti: Mti::SNIP_REPLY,
            },
            Message::ProtocolSupportInquiry { dest: 0x777 },
            Message::ProtocolSupportReply {
                dest: 0x777,
                flags: 0xFF_FFFF,
            },
            Message::IdentifyConsumer(e),
            Message::ConsumerRangeIdentified(e),
            Message::IdentifyProducer(e),
            Message::ProducerRangeIdentified(e),
            Message::IdentifyEventsGlobal,
            Message::IdentifyEventsAddressed { dest: 0x777 },
            Message::LearnEvent(e),
            Message::Pcer(e),
            Message::TractionCommand {
                dest: 0x6A1,
                payload: vec![],
            },
            Message::TractionCommand {
                dest: 0x6A1,
                payload: (0..9).collect(),
            },
            Message::TractionReply {
                dest: 0x6A1,
                payload: (0..12).collect(),
            },
            Message::TractionReply {
                dest: 0x6A1,
                payload: (0..13).collect(),
            },
            Message::SnipRequest { dest: 0x777 },
            Message::SnipReply {
                dest: 0x777,
                data: snip,
            },
            Message::DatagramReceivedOk {
                dest: 0x777,
                flags: 0x80,
            },
            Message::DatagramRejected {
                dest: 0x777,
                code: 0x2020,
            },
            Message::Other {
                mti: Mti(0x0DA8),
                dest: Some(0x777),
                payload: vec![1, 2, 3, 4, 5, 6, 7],
            },
            Message::Other {
                mti: Mti(0x0DA0),
                dest: None,
                payload: vec![9; 8],
            },
        ];
        for state in [EventState::Valid, EventState::Invalid, EventState::Unknown] {
            all.push(Message::ConsumerIdentified { event: e, state });
            all.push(Message::ProducerIdentified { event: e, state });
        }
        for len in [0usize, 1, 7, 8, 9, 16, 17, 64, 71, 72] {
            all.push(Message::Datagram {
                dest: 0x456,
                data: (0..len as u8).collect(),
            });
        }
        for m in &all {
            assert_eq!(&round_trip(m), m);
        }
    }

    #[test]
    fn datagram_frame_counts() {
        for (len, frames, kinds) in [
            (0usize, 1usize, &[2u32][..]),
            (8, 1, &[2]),
            (9, 2, &[3, 5]),
            (17, 3, &[3, 4, 5]),
            (72, 9, &[3, 4, 4, 4, 4, 4, 4, 4, 5]),
        ] {
            let m = Message::Datagram {
                dest: 0x456,
                data: vec![0xAA; len],
            };
            let f = m.try_to_frames(0x123).unwrap();
            assert_eq!(f.len(), frames, "{len}");
            let got: Vec<u32> = f.iter().map(|f| (f.id >> 24) & 7).collect();
            assert_eq!(got, kinds, "{len}");
            assert_eq!(f.iter().map(|f| f.data().len()).sum::<usize>(), len);
        }
    }

    #[test]
    fn try_to_frames_rejects() {
        let e = |m: Message| m.try_to_frames(0x123).unwrap_err();
        assert_eq!(
            e(Message::Other {
                mti: Mti(0x0DA0),
                dest: None,
                payload: vec![0; 9]
            }),
            EncodeError::GlobalPayloadTooLong { len: 9 }
        );
        assert_eq!(
            e(Message::Other {
                mti: Mti(0x0DA8),
                dest: None,
                payload: vec![]
            }),
            EncodeError::MissingDestination { mti: Mti(0x0DA8) }
        );
        assert_eq!(
            e(Message::Other {
                mti: Mti::DATAGRAM,
                dest: None,
                payload: vec![]
            }),
            EncodeError::MissingDestination { mti: Mti::DATAGRAM }
        );
        assert_eq!(
            e(Message::Other {
                mti: Mti::PCER,
                dest: Some(1),
                payload: vec![0; 8]
            }),
            EncodeError::UnexpectedDestination { mti: Mti::PCER }
        );
        assert_eq!(
            e(Message::Other {
                mti: Mti(0x1F88),
                dest: Some(1),
                payload: vec![]
            }),
            EncodeError::MtiTooWide { mti: Mti(0x1F88) }
        );
        assert_eq!(
            e(Message::Datagram {
                dest: 1,
                data: vec![0; 73]
            }),
            EncodeError::DatagramTooLong { len: 73 }
        );
        // Lenient encoder keeps its old behaviour (without the debug assertion).
        let big = Message::Datagram {
            dest: 1,
            data: vec![0; 80],
        };
        assert_eq!(big.to_frames_lenient(0x123).len(), 10);
        let wide = Message::Other {
            mti: Mti(0x1F88),
            dest: None,
            payload: vec![],
        };
        assert_eq!(
            wide.to_frames_lenient(0x123)[0].header(),
            CanHeader::Message {
                mti: Mti(0x0F88),
                alias: 0x123
            }
        );
        assert!(
            !alloc::string::ToString::to_string(&EncodeError::DatagramTooLong { len: 73 })
                .is_empty()
        );
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "use try_to_frames")]
    fn to_frames_debug_asserts() {
        let _ = Message::Datagram {
            dest: 1,
            data: vec![0; 73],
        }
        .to_frames(0x123);
    }

    #[test]
    fn orphan_middle_and_final_frames_are_ignored() {
        let m = Message::TractionReply {
            dest: 0x100,
            payload: (0..20).collect(),
        };
        let f = m.to_frames(0x201);
        assert_eq!(f.len(), 4);
        let mut a = Assembler::new();
        assert!(a.accept(&f[1]).is_none());
        assert!(a.accept(&f[3]).is_none());
        assert_eq!(a.partial_count(), 0);
        // Datagram middle/final without a first frame too.
        let d = Message::Datagram {
            dest: 0x100,
            data: vec![1; 20],
        }
        .to_frames(0x201);
        assert!(a.accept(&d[1]).is_none());
        assert!(a.accept(&d[2]).is_none());
        assert_eq!(a.partial_count(), 0);
        // A frame too short to hold the destination is ignored.
        let short = CanFrame::of(
            CanHeader::Message {
                mti: Mti::TRACTION_CONTROL_REPLY,
                alias: 0x201,
            }
            .encode(),
            &[0x01],
        );
        assert!(a.accept(&short).is_none());
        // A new first frame restarts the message.
        assert!(a.accept(&f[0]).is_none());
        assert!(a.accept(&f[0]).is_none());
        assert!(a.accept(&f[1]).is_none());
        assert!(a.accept(&f[2]).is_none());
        assert_eq!(Message::from(a.accept(&f[3]).unwrap()), m);
    }

    #[test]
    fn partials_expire_with_accept_at() {
        let m = Message::TractionReply {
            dest: 0x100,
            payload: vec![7; 10],
        };
        let f = m.to_frames(0x201);
        let mut a = Assembler::new();
        assert!(a.accept_at(&f[0], 0).is_none());
        assert_eq!(a.partial_count(), 1);
        // Still alive just before the timeout…
        assert!(a
            .accept_at(
                &Message::Pcer(EventId(1)).to_frames(0x300)[0],
                PARTIAL_TIMEOUT_MS - 1
            )
            .is_some());
        assert_eq!(a.partial_count(), 1);
        // …and gone at it.
        assert!(a.accept_at(&f[1], PARTIAL_TIMEOUT_MS).is_none());
        assert_eq!(a.partial_count(), 0);

        // Middle frames keep a long message alive.
        let long = Message::TractionReply {
            dest: 0x100,
            payload: vec![7; 20],
        };
        let f = long.to_frames(0x201);
        assert!(a.accept_at(&f[0], 10_000).is_none());
        assert!(a.accept_at(&f[1], 12_000).is_none());
        assert!(a.accept_at(&f[2], 14_000).is_none());
        assert_eq!(Message::from(a.accept_at(&f[3], 16_000).unwrap()), long);

        // Plain accept never expires.
        let mut a = Assembler::new();
        assert!(a.accept(&f[0]).is_none());
        assert!(a.accept(&f[1]).is_none());
        assert!(a.accept(&f[2]).is_none());
        assert!(a.accept(&f[3]).is_some());
    }

    #[test]
    fn eviction_drops_the_oldest_partial() {
        let mut a = Assembler::new();
        // Start partials from high aliases down, so the oldest has the
        // *largest* key: eviction must follow age, not key order.
        let first_frame = |alias: u16| {
            Message::TractionReply {
                dest: 0x100,
                payload: vec![alias as u8; 10],
            }
            .to_frames(alias)
        };
        let mut frames = Vec::new();
        for i in 0..MAX_PARTIALS as u16 {
            let f = first_frame(0x800 - i);
            assert!(a.accept_at(&f[0], u64::from(i)).is_none());
            frames.push(f);
        }
        assert_eq!(a.partial_count(), MAX_PARTIALS);
        let extra = first_frame(0x001);
        assert!(a.accept_at(&extra[0], 100).is_none());
        assert_eq!(a.partial_count(), MAX_PARTIALS);
        // The oldest (alias 0x800) was evicted; the next oldest survives.
        assert!(a.accept_at(&frames[0][1], 101).is_none());
        assert!(a.accept_at(&frames[1][1], 102).is_some());
        assert!(a.accept_at(&extra[1], 103).is_some());
    }

    #[test]
    fn oversized_partials_are_dropped() {
        let mut a = Assembler::new();
        let big = Message::TractionReply {
            dest: 0x100,
            payload: vec![1; MAX_PAYLOAD + 12],
        };
        let got: Vec<_> = big
            .to_frames(0x201)
            .iter()
            .filter_map(|f| a.accept(f))
            .collect();
        assert!(got.is_empty());
        assert_eq!(a.partial_count(), 0);
    }

    #[test]
    fn forget_drops_partials_from_alias() {
        let mut a = Assembler::new();
        let f = Message::TractionReply {
            dest: 0x100,
            payload: vec![1; 10],
        }
        .to_frames(0x201);
        assert!(a.accept(&f[0]).is_none());
        a.forget(0x201);
        assert!(a.accept(&f[1]).is_none());
    }

    #[test]
    fn legacy_snip_only_frames_are_joined() {
        let info = crate::NodeInfo {
            manufacturer: "Old Co".into(),
            model: "Relic".into(),
            hardware_version: "1".into(),
            software_version: "2".into(),
            user_name: "Yard".into(),
            user_description: "East end".into(),
        };
        let payload = info.snip_payload();
        let id = CanHeader::Message {
            mti: Mti::SNIP_REPLY,
            alias: 0x0AB,
        }
        .encode();
        let mut a = Assembler::new();
        let mut out = None;
        for chunk in payload.chunks(6) {
            let mut d = vec![0x01, 0x23]; // flag 0 ("only"), dest 0x123
            d.extend_from_slice(chunk);
            assert!(out.is_none(), "completed early");
            out = a.accept_at(&CanFrame::of(id, &d), 0);
        }
        let raw = out.expect("joined");
        assert_eq!(raw.dest, Some(0x123));
        assert_eq!(raw.payload, payload);
        assert_eq!(crate::NodeInfo::from_snip(&raw.payload), info);
        assert_eq!(a.partial_count(), 0);

        // A properly framed reply still works, and a stray legacy partial is
        // replaced by a real first frame.
        let _ = a.accept_at(&CanFrame::of(id, &[0x01, 0x23, 4, b'X', 0]), 10);
        assert_eq!(a.partial_count(), 1);
        let modern = Message::SnipReply {
            dest: 0x123,
            data: payload.clone(),
        };
        let got: Vec<_> = modern
            .to_frames(0x0AB)
            .iter()
            .filter_map(|f| a.accept_at(f, 11))
            .collect();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].payload, payload);
        // An incomplete legacy reply expires.
        let _ = a.accept_at(&CanFrame::of(id, &[0x01, 0x23, 4, b'X', 0]), 20);
        let _ = a.accept_at(
            &Message::Pcer(EventId(1)).to_frames(1)[0],
            20 + PARTIAL_TIMEOUT_MS,
        );
        assert_eq!(a.partial_count(), 0);
    }
}
