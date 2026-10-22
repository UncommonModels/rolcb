//! Message Type Indicators (16-bit, as defined in the Message Network Standard).
//!
//! Every OpenLCB message starts with a 16-bit **MTI** that says what kind of
//! message it is. The *Message Network Standard* packs a few properties into
//! fixed bit positions, so a node can handle an unknown MTI sensibly:
//!
//! ```text
//!  bits 15-13 : reserved (0)
//!  bit  12    : stream or datagram (set only for Datagram/Stream MTIs)
//!  bits 11-10 : static priority
//!  bits  9-5  : type within priority
//!  bit   4    : "simple" flag: simple nodes must also process this message
//!  bit   3    : 1 = addressed (payload begins with a destination)
//!  bit   2    : 1 = an Event ID is present
//!  bits  1-0  : modifier (e.g. Valid/Invalid/Unknown for Identified
//!               messages, or "sent by a simple node" for Init Complete)
//! ```
//!
//! On CAN only the low 12 bits fit into the frame header ("CAN-MTI"); see
//! [`Mti::can_mti`]. Datagrams (`0x1C48`) and streams have bit 12 set, do not fit,
//! and instead travel in dedicated datagram/stream frame types, see
//! [`MessageFrameType`](crate::can::MessageFrameType).
//!
//! The full MTI table is in the
//! [protocol reference](crate::guide::reference#message-type-indicators).
//!
//! ```
//! use rolcb::Mti;
//!
//! assert!(Mti::TRACTION_CONTROL_COMMAND.is_addressed());
//! assert!(!Mti::PCER.is_addressed());
//! assert!(Mti::PCER.has_event());
//! assert_eq!(Mti::PCER.name(), "PCER");
//! assert_eq!(Mti::VERIFY_NODE_ID_GLOBAL.can_mti(), 0x490);
//! ```

/// A 16-bit OpenLCB Message Type Indicator.
///
/// The associated constants name every MTI the crate decodes into a
/// dedicated [`Message`](crate::Message) variant. Other values can still be
/// built with `Mti(0x…)` and are carried by
/// [`Message::Other`](crate::Message::Other).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Mti(pub u16);

impl Mti {
    /// Initialization Complete (global). Sent once by a node after it has
    /// finished logging in; carries the 6-byte Node ID.
    pub const INITIALIZATION_COMPLETE: Mti = Mti(0x0100);
    /// Initialization Complete from a node that implements only the
    /// *Simple Protocol* subset.
    pub const INITIALIZATION_COMPLETE_SIMPLE: Mti = Mti(0x0101);
    /// Verify Node ID, addressed to one node. Optional Node ID payload.
    pub const VERIFY_NODE_ID_ADDRESSED: Mti = Mti(0x0488);
    /// Verify Node ID, global: "everyone (or the node with this ID) please
    /// announce yourself". Optional Node ID payload.
    pub const VERIFY_NODE_ID_GLOBAL: Mti = Mti(0x0490);
    /// Verified Node ID (global): the answer to Verify Node ID, carrying the
    /// responder's Node ID.
    pub const VERIFIED_NODE_ID: Mti = Mti(0x0170);
    /// Verified Node ID from a Simple Protocol node.
    pub const VERIFIED_NODE_ID_SIMPLE: Mti = Mti(0x0171);
    /// Optional Interaction Rejected (addressed): the receiver does not
    /// support or cannot handle the request. Payload is an error code (see
    /// [`error_code`]) and the MTI being rejected.
    pub const OPTIONAL_INTERACTION_REJECTED: Mti = Mti(0x0068);
    /// Terminate Due To Error (addressed): an interaction was aborted.
    /// Payload is an error code and the related MTI.
    pub const TERMINATE_DUE_TO_ERROR: Mti = Mti(0x00A8);
    /// Protocol Support Inquiry (addressed): "which protocols do you speak?"
    pub const PROTOCOL_SUPPORT_INQUIRY: Mti = Mti(0x0828);
    /// Protocol Support Reply (addressed): a bit field of supported
    /// protocols, see [`crate::message::protocol`].
    pub const PROTOCOL_SUPPORT_REPLY: Mti = Mti(0x0668);

    /// Identify Consumer (global, with event): "who consumes this event?"
    pub const IDENTIFY_CONSUMER: Mti = Mti(0x08F4);
    /// Consumer Range Identified: the sender consumes a whole range of
    /// events, encoded as a mask in the low-order bits of the Event ID.
    pub const CONSUMER_RANGE_IDENTIFIED: Mti = Mti(0x04A4);
    /// Consumer Identified, state [`Valid`](crate::EventState::Valid).
    pub const CONSUMER_IDENTIFIED_VALID: Mti = Mti(0x04C4);
    /// Consumer Identified, state [`Invalid`](crate::EventState::Invalid).
    pub const CONSUMER_IDENTIFIED_INVALID: Mti = Mti(0x04C5);
    /// Consumer Identified, state [`Unknown`](crate::EventState::Unknown).
    pub const CONSUMER_IDENTIFIED_UNKNOWN: Mti = Mti(0x04C7);
    /// Identify Producer (global, with event): "who produces this event?"
    /// Also used by the *Train Search Standard* (see
    /// [`traction::search`](crate::traction::search)).
    pub const IDENTIFY_PRODUCER: Mti = Mti(0x0914);
    /// Producer Range Identified: the sender produces a range of events.
    pub const PRODUCER_RANGE_IDENTIFIED: Mti = Mti(0x0524);
    /// Producer Identified, state [`Valid`](crate::EventState::Valid).
    pub const PRODUCER_IDENTIFIED_VALID: Mti = Mti(0x0544);
    /// Producer Identified, state [`Invalid`](crate::EventState::Invalid).
    pub const PRODUCER_IDENTIFIED_INVALID: Mti = Mti(0x0545);
    /// Producer Identified, state [`Unknown`](crate::EventState::Unknown).
    pub const PRODUCER_IDENTIFIED_UNKNOWN: Mti = Mti(0x0547);
    /// Identify Events (addressed): "tell me everything you produce and consume".
    pub const IDENTIFY_EVENTS_ADDRESSED: Mti = Mti(0x0968);
    /// Identify Events (global): every node lists all its events.
    pub const IDENTIFY_EVENTS_GLOBAL: Mti = Mti(0x0970);
    /// Learn Event: teaching message used by blue/gold button configuration.
    pub const LEARN_EVENT: Mti = Mti(0x0594);
    /// Producer/Consumer Event Report: "this event just happened".
    pub const PCER: Mti = Mti(0x05B4);

    /// Traction Control Command (addressed, throttle → train), defined by the
    /// *Traction Control Standard*. See [`crate::traction::Command`].
    pub const TRACTION_CONTROL_COMMAND: Mti = Mti(0x05EB);
    /// Traction Control Reply (addressed, train → throttle). See
    /// [`crate::traction::Reply`].
    pub const TRACTION_CONTROL_REPLY: Mti = Mti(0x01E9);

    /// Simple Node Information Request (addressed), from the *Simple Node
    /// Information Standard*.
    pub const SNIP_REQUEST: Mti = Mti(0x0DE8);
    /// Simple Node Information Reply (addressed, multi-frame). Payload format
    /// in [`NodeInfo::snip_payload`](crate::NodeInfo::snip_payload).
    pub const SNIP_REPLY: Mti = Mti(0x0A08);

    /// Datagram (addressed, up to 72 bytes), from the *Datagram Transport
    /// Standard*. On CAN it uses datagram frame types instead of a CAN-MTI.
    pub const DATAGRAM: Mti = Mti(0x1C48);
    /// Datagram Received OK (addressed). Optional one-byte flags field.
    pub const DATAGRAM_RECEIVED_OK: Mti = Mti(0x0A28);
    /// Datagram Rejected (addressed), with an [`error_code`].
    pub const DATAGRAM_REJECTED: Mti = Mti(0x0A48);

    /// True if the message carries a destination (addressed message).
    ///
    /// This is bit 3 (`0x0008`) of the MTI. On CAN, addressed messages put
    /// the 12-bit destination alias in the first two data bytes; see
    /// [`Message::to_frames`](crate::Message::to_frames).
    pub const fn is_addressed(self) -> bool {
        self.0 & 0x0008 != 0
    }

    /// True if the message carries an Event ID (bit 2, `0x0004`).
    pub const fn has_event(self) -> bool {
        self.0 & 0x0004 != 0
    }

    /// The 12-bit value carried in the CAN header for this MTI.
    ///
    /// This is simply the low 12 bits. It is only meaningful for MTIs sent
    /// in global/addressed frames (frame type 1); [`Mti::DATAGRAM`] uses
    /// datagram frame types instead.
    pub const fn can_mti(self) -> u16 {
        self.0 & 0x0FFF
    }

    /// A short human readable name, for logging.
    pub fn name(self) -> &'static str {
        match self {
            Mti::INITIALIZATION_COMPLETE => "InitComplete",
            Mti::INITIALIZATION_COMPLETE_SIMPLE => "InitCompleteSimple",
            Mti::VERIFY_NODE_ID_ADDRESSED => "VerifyNodeIdAddressed",
            Mti::VERIFY_NODE_ID_GLOBAL => "VerifyNodeIdGlobal",
            Mti::VERIFIED_NODE_ID => "VerifiedNodeId",
            Mti::VERIFIED_NODE_ID_SIMPLE => "VerifiedNodeIdSimple",
            Mti::OPTIONAL_INTERACTION_REJECTED => "OptionalInteractionRejected",
            Mti::TERMINATE_DUE_TO_ERROR => "TerminateDueToError",
            Mti::PROTOCOL_SUPPORT_INQUIRY => "ProtocolSupportInquiry",
            Mti::PROTOCOL_SUPPORT_REPLY => "ProtocolSupportReply",
            Mti::IDENTIFY_CONSUMER => "IdentifyConsumer",
            Mti::CONSUMER_RANGE_IDENTIFIED => "ConsumerRangeIdentified",
            Mti::CONSUMER_IDENTIFIED_VALID => "ConsumerIdentifiedValid",
            Mti::CONSUMER_IDENTIFIED_INVALID => "ConsumerIdentifiedInvalid",
            Mti::CONSUMER_IDENTIFIED_UNKNOWN => "ConsumerIdentifiedUnknown",
            Mti::IDENTIFY_PRODUCER => "IdentifyProducer",
            Mti::PRODUCER_RANGE_IDENTIFIED => "ProducerRangeIdentified",
            Mti::PRODUCER_IDENTIFIED_VALID => "ProducerIdentifiedValid",
            Mti::PRODUCER_IDENTIFIED_INVALID => "ProducerIdentifiedInvalid",
            Mti::PRODUCER_IDENTIFIED_UNKNOWN => "ProducerIdentifiedUnknown",
            Mti::IDENTIFY_EVENTS_ADDRESSED => "IdentifyEventsAddressed",
            Mti::IDENTIFY_EVENTS_GLOBAL => "IdentifyEventsGlobal",
            Mti::LEARN_EVENT => "LearnEvent",
            Mti::PCER => "PCER",
            Mti::TRACTION_CONTROL_COMMAND => "TractionCommand",
            Mti::TRACTION_CONTROL_REPLY => "TractionReply",
            Mti::SNIP_REQUEST => "SnipRequest",
            Mti::SNIP_REPLY => "SnipReply",
            Mti::DATAGRAM => "Datagram",
            Mti::DATAGRAM_RECEIVED_OK => "DatagramReceivedOk",
            Mti::DATAGRAM_REJECTED => "DatagramRejected",
            _ => "Unknown",
        }
    }

    /// Map a 12-bit CAN-MTI back to the full MTI. All MTIs that are sent in
    /// CAN "global/addressed" frames fit in 12 bits, so this is the identity.
    pub const fn from_can_mti(v: u16) -> Mti {
        Mti(v & 0x0FFF)
    }
}

/// Error codes used in Optional Interaction Rejected / Datagram Rejected.
///
/// Codes are 16 bits. The top nibble classifies the error: `0x1xxx` is
/// *permanent* (do not retry the same request), `0x2xxx` is *temporary*
/// (the request may succeed if retried later). Lower bits refine the
/// reason. Only the codes this crate uses are named here; see the *Message
/// Network Standard* for the full list.
pub mod error_code {
    /// Permanent error, no further detail.
    pub const PERMANENT: u16 = 0x1000;
    /// Permanent: the requested protocol or MTI is not implemented. Sent by
    /// [`Node`](crate::Node) for addressed messages it does not support.
    pub const NOT_IMPLEMENTED: u16 = 0x1040;
    /// Permanent: the sub-command is not known.
    pub const NOT_IMPLEMENTED_SUBCOMMAND: u16 = 0x1041;
    /// Permanent: the command is not known.
    pub const NOT_IMPLEMENTED_COMMAND: u16 = 0x1042;
    /// Temporary error, no further detail; retrying may succeed.
    pub const TEMPORARY: u16 = 0x2000;
    /// Temporary: the receiver has no buffer available right now.
    pub const BUFFER_UNAVAILABLE: u16 = 0x2020;
    /// Temporary: frames arrived out of order (multi-frame reassembly failed).
    pub const OUT_OF_ORDER: u16 = 0x2040;
}
