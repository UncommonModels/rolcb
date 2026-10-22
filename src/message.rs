//! High-level OpenLCB messages and their encoding onto CAN frames.
//!
//! A [`Message`] is one OpenLCB message as defined by the *Message Network
//! Standard* and the protocol standards built on it (*Event Transport*,
//! *Traction Control*, *Simple Node Information*, *Datagram Transport*).
//! This module converts between messages and CAN frames, following the
//! *CAN Frame Transfer Standard*:
//!
//! * [`Message::to_frames`] encodes a message sent from a given source
//!   alias into one or more [`CanFrame`]s.
//! * [`Assembler`] collects incoming frames, reassembles multi-frame
//!   messages, and yields [`RawMessage`]s.
//! * [`Message::from_parts`] decodes a `RawMessage` (MTI, destination,
//!   payload) into a `Message`.
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
//! assert_eq!(Message::from_parts(raw.mti, raw.dest, raw.payload), reply);
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
#[derive(Debug, Clone, PartialEq, Eq)]
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
        /// Datagram contents (the encoder does not enforce the 72-byte limit).
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

fn event_of(p: &[u8]) -> EventId {
    EventId::from_bytes(p).unwrap_or_default()
}

fn u16_at(p: &[u8], i: usize) -> u16 {
    match p.get(i..i + 2) {
        Some(b) => u16::from_be_bytes([b[0], b[1]]),
        None => 0,
    }
}

impl Message {
    /// The MTI this message is sent with.
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
    /// Usually fed from a [`RawMessage`] produced by an [`Assembler`].
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
    pub fn from_parts(mti: Mti, dest: Option<u16>, p: Vec<u8>) -> Message {
        use Message::*;
        let d = dest.unwrap_or(0);
        let nid = |p: &[u8]| NodeId::from_bytes(p).ok();
        let has_event = p.len() >= 8;
        match mti {
            Mti::INITIALIZATION_COMPLETE | Mti::INITIALIZATION_COMPLETE_SIMPLE if p.len() >= 6 => {
                InitializationComplete {
                    node_id: nid(&p).unwrap(),
                    simple: mti.0 & 1 == 1,
                }
            }
            Mti::VERIFY_NODE_ID_GLOBAL => VerifyNodeIdGlobal { node_id: nid(&p) },
            Mti::VERIFY_NODE_ID_ADDRESSED => VerifyNodeIdAddressed {
                dest: d,
                node_id: nid(&p),
            },
            Mti::VERIFIED_NODE_ID | Mti::VERIFIED_NODE_ID_SIMPLE if p.len() >= 6 => {
                VerifiedNodeId {
                    node_id: nid(&p).unwrap(),
                    simple: mti.0 & 1 == 1,
                }
            }
            Mti::OPTIONAL_INTERACTION_REJECTED => OptionalInteractionRejected {
                dest: d,
                code: u16_at(&p, 0),
                mti: Mti(u16_at(&p, 2)),
            },
            Mti::TERMINATE_DUE_TO_ERROR => TerminateDueToError {
                dest: d,
                code: u16_at(&p, 0),
                mti: Mti(u16_at(&p, 2)),
            },
            Mti::PROTOCOL_SUPPORT_INQUIRY => ProtocolSupportInquiry { dest: d },
            Mti::PROTOCOL_SUPPORT_REPLY => {
                let b = |i: usize| *p.get(i).unwrap_or(&0) as u32;
                ProtocolSupportReply {
                    dest: d,
                    flags: (b(0) << 16) | (b(1) << 8) | b(2),
                }
            }
            Mti::IDENTIFY_CONSUMER if has_event => IdentifyConsumer(event_of(&p)),
            Mti::CONSUMER_IDENTIFIED_VALID if has_event => ConsumerIdentified {
                event: event_of(&p),
                state: EventState::Valid,
            },
            Mti::CONSUMER_IDENTIFIED_INVALID if has_event => ConsumerIdentified {
                event: event_of(&p),
                state: EventState::Invalid,
            },
            Mti::CONSUMER_IDENTIFIED_UNKNOWN if has_event => ConsumerIdentified {
                event: event_of(&p),
                state: EventState::Unknown,
            },
            Mti::CONSUMER_RANGE_IDENTIFIED if has_event => ConsumerRangeIdentified(event_of(&p)),
            Mti::IDENTIFY_PRODUCER if has_event => IdentifyProducer(event_of(&p)),
            Mti::PRODUCER_IDENTIFIED_VALID if has_event => ProducerIdentified {
                event: event_of(&p),
                state: EventState::Valid,
            },
            Mti::PRODUCER_IDENTIFIED_INVALID if has_event => ProducerIdentified {
                event: event_of(&p),
                state: EventState::Invalid,
            },
            Mti::PRODUCER_IDENTIFIED_UNKNOWN if has_event => ProducerIdentified {
                event: event_of(&p),
                state: EventState::Unknown,
            },
            Mti::PRODUCER_RANGE_IDENTIFIED if has_event => ProducerRangeIdentified(event_of(&p)),
            Mti::IDENTIFY_EVENTS_GLOBAL => IdentifyEventsGlobal,
            Mti::IDENTIFY_EVENTS_ADDRESSED => IdentifyEventsAddressed { dest: d },
            Mti::LEARN_EVENT if has_event => LearnEvent(event_of(&p)),
            Mti::PCER if has_event => Pcer(event_of(&p)),
            Mti::TRACTION_CONTROL_COMMAND => TractionCommand {
                dest: d,
                payload: p,
            },
            Mti::TRACTION_CONTROL_REPLY => TractionReply {
                dest: d,
                payload: p,
            },
            Mti::SNIP_REQUEST => SnipRequest { dest: d },
            Mti::SNIP_REPLY => SnipReply { dest: d, data: p },
            Mti::DATAGRAM => Datagram { dest: d, data: p },
            Mti::DATAGRAM_RECEIVED_OK => DatagramReceivedOk {
                dest: d,
                flags: *p.first().unwrap_or(&0),
            },
            Mti::DATAGRAM_REJECTED => DatagramRejected {
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

    /// Encode as one or more CAN frames sent from `src_alias`.
    ///
    /// * Global messages produce one frame (payload truncated to 8 bytes).
    /// * Addressed messages produce one frame per 6 payload bytes (at least
    ///   one), each starting with the framing flag and destination alias as
    ///   described in the [module docs](self#global-addressed-and-datagram-framing).
    /// * Datagrams produce one frame per 8 bytes using the datagram frame
    ///   types.
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
    pub fn to_frames(&self, src_alias: u16) -> Vec<CanFrame> {
        let mti = self.mti();
        let payload = self.payload();
        let mut out = Vec::new();

        if mti == Mti::DATAGRAM {
            let dest = self.dest().unwrap_or(0);
            let chunks: Vec<&[u8]> = if payload.is_empty() {
                alloc::vec![&[][..]]
            } else {
                payload.chunks(8).collect()
            };
            let n = chunks.len();
            for (i, c) in chunks.into_iter().enumerate() {
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
                out.push(CanFrame::of(id, c));
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
                out.push(CanFrame::of(id, &payload[..n]));
            }
            Some(dest) => {
                let chunks: Vec<&[u8]> = if payload.is_empty() {
                    alloc::vec![&[][..]]
                } else {
                    payload.chunks(6).collect()
                };
                let n = chunks.len();
                for (i, c) in chunks.into_iter().enumerate() {
                    let flags: u8 = match (n, i) {
                        (1, _) => 0x00,
                        (_, 0) => 0x10,
                        (_, i) if i == n - 1 => 0x20,
                        _ => 0x30,
                    };
                    let mut d = [0u8; 8];
                    d[0] = flags | ((dest >> 8) as u8 & 0x0F);
                    d[1] = dest as u8;
                    d[2..2 + c.len()].copy_from_slice(c);
                    out.push(CanFrame::of(id, &d[..2 + c.len()]));
                }
            }
        }
        out
    }
}

/// A complete message reassembled from frames, before decoding.
///
/// Turn it into a [`Message`] with
/// `Message::from_parts(raw.mti, raw.dest, raw.payload)`.
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

const MAX_PARTIALS: usize = 32;
const MAX_PAYLOAD: usize = 256;

/// Reassembles multi-frame addressed messages and datagrams.
///
/// Feed every frame to [`Assembler::accept`]. Single-frame messages come
/// straight back; multi-frame ones are buffered until their last frame.
/// Partial messages are keyed by (source alias, destination alias, MTI), so
/// interleaved transfers from different nodes are handled.
///
/// The assembler is defensive about memory: at most 32 partial messages are
/// kept (the oldest-keyed one is dropped to make room) and a partial message
/// that would exceed 256 bytes is discarded. A middle or last frame with no
/// matching first frame is ignored. It is a passive listener, so it
/// reassembles messages for *every* destination, not just one node.
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
/// assert!(asm.accept(&frames[0]).is_none());
/// let raw = asm.accept(&frames[1]).expect("complete");
/// assert_eq!(raw.dest, Some(0x6A1));
/// assert_eq!(Message::from_parts(raw.mti, raw.dest, raw.payload), cmd);
/// ```
#[derive(Debug, Default)]
pub struct Assembler {
    partial: BTreeMap<(u16, u16, u16), Vec<u8>>,
}

impl Assembler {
    /// An assembler with no partial messages.
    pub fn new() -> Self {
        Self::default()
    }

    /// Accept one frame. Returns a message when one is complete. Control
    /// frames (CID/RID/AMD...) return `None`; the node handles those.
    pub fn accept(&mut self, frame: &CanFrame) -> Option<RawMessage> {
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
                if data.len() < 2 {
                    return None;
                }
                let dest = (((data[0] & 0x0F) as u16) << 8) | data[1] as u16;
                let flags = data[0] & 0x30;
                let body = &data[2..];
                let key = (alias, dest, mti.0);
                self.collect(key, flags, body).map(|payload| RawMessage {
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
                self.collect(key, flags, data).map(|payload| RawMessage {
                    src_alias: alias,
                    mti: Mti::DATAGRAM,
                    dest: Some(dest),
                    payload,
                })
            }
            _ => None,
        }
    }

    fn collect(&mut self, key: (u16, u16, u16), flags: u8, body: &[u8]) -> Option<Vec<u8>> {
        match flags {
            0x00 => {
                self.partial.remove(&key);
                Some(body.to_vec())
            }
            0x10 => {
                if self.partial.len() >= MAX_PARTIALS && !self.partial.contains_key(&key) {
                    // Drop the oldest-keyed entry to bound memory.
                    let first = *self.partial.keys().next().unwrap();
                    self.partial.remove(&first);
                }
                self.partial.insert(key, body.to_vec());
                None
            }
            0x30 => {
                if let Some(buf) = self.partial.get_mut(&key) {
                    if buf.len() + body.len() <= MAX_PAYLOAD {
                        buf.extend_from_slice(body);
                    } else {
                        self.partial.remove(&key);
                    }
                }
                None
            }
            _ => {
                let mut buf = self.partial.remove(&key)?;
                buf.extend_from_slice(body);
                Some(buf)
            }
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
}
