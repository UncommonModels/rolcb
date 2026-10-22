//! 64-bit OpenLCB Event IDs and event states.
//!
//! OpenLCB's *Event Transport Standard* implements a producer/consumer
//! model. An **event** is identified by a globally unique 64-bit
//! [`EventId`]; there is no payload. A *producer* announces "this event
//! happened" with a [PCER](crate::Message::Pcer) (Producer/Consumer Event
//! Report) and every *consumer* of that Event ID reacts to it. The bus does
//! not route: every node sees every PCER and acts on the IDs it cares about.
//!
//! Event IDs are normally allocated from the Node ID of the device that
//! creates them: the first six bytes are a Node ID and the last two are free
//! for that node to number its events (see the *Event Identifiers Standard*).
//! For example a block detector with Node ID `05.01.01.01.22.00` might use
//! `05.01.01.01.22.00.00.10` for "block 1 occupied" and `…00.11` for
//! "block 1 clear". A few well-known IDs (emergency stop, "is train") use the
//! reserved `01.xx` range; they are provided as constants on [`EventId`].
//!
//! Besides PCERs, nodes advertise which events they produce and consume
//! (Producer/Consumer Identified messages) together with an [`EventState`]
//! saying whether that event is currently *valid* (true), *invalid* (false)
//! or *unknown*. Configuration tools and throttles use the identification
//! exchange to discover state without waiting for the next PCER. See the
//! [events guide](crate::guide::events) for a worked example.
//!
//! ```
//! use rolcb::EventId;
//!
//! let e: EventId = "05.01.01.01.22.00.00.10".parse()?;
//! assert_eq!(e, EventId(0x0501_0101_2200_0010));
//! assert_eq!(e.to_string(), "05.01.01.01.22.00.00.10");
//! assert_eq!(EventId::from_bytes(&e.to_bytes())?, e);
//! # Ok::<(), rolcb::ParseError>(())
//! ```

use core::fmt;
use core::str::FromStr;

use crate::{fmt_dotted, parse_dotted, ParseError};

/// A 64-bit event identifier, e.g. `05.01.01.01.22.00.00.10`.
///
/// The inner value is public; `EventId(0x0501_0101_2200_0010)` is as good
/// as parsing. `Display`/`FromStr` use eight dotted hex bytes (separators
/// optional, as for [`NodeId`](crate::NodeId)); with the `serde` feature
/// the ID (de)serializes as that string. On the wire it is eight big-endian
/// bytes, which fill a whole CAN frame.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct EventId(pub u64);

impl EventId {
    /// Well-known: Emergency Off (de-energize).
    pub const EMERGENCY_OFF: EventId = EventId(0x0100_0000_0000_FFFF);
    /// Well-known: Clear Emergency Off.
    pub const CLEAR_EMERGENCY_OFF: EventId = EventId(0x0100_0000_0000_FFFE);
    /// Well-known: Emergency Stop of all operations.
    pub const EMERGENCY_STOP: EventId = EventId(0x0100_0000_0000_FFFD);
    /// Well-known: Clear Emergency Stop.
    pub const CLEAR_EMERGENCY_STOP: EventId = EventId(0x0100_0000_0000_FFFC);
    /// Well-known: this node is a train (Traction protocol).
    pub const IS_TRAIN: EventId = EventId(0x0101_0000_0000_0303);

    /// Wrap a raw 64-bit value. Same as `EventId(v)`, usable in `const`.
    pub const fn new(v: u64) -> Self {
        EventId(v)
    }

    /// The raw 64-bit value.
    pub const fn as_u64(self) -> u64 {
        self.0
    }

    /// The eight big-endian bytes, as carried in event messages.
    pub fn to_bytes(self) -> [u8; 8] {
        self.0.to_be_bytes()
    }

    /// Read an Event ID from the first eight big-endian bytes of `b`.
    /// Extra trailing bytes are ignored.
    ///
    /// # Errors
    ///
    /// [`ParseError::Length`] if `b` is shorter than eight bytes.
    pub fn from_bytes(b: &[u8]) -> Result<Self, ParseError> {
        let arr: [u8; 8] = b.get(..8).ok_or(ParseError::Length)?.try_into().unwrap();
        Ok(EventId(u64::from_be_bytes(arr)))
    }
}

impl fmt::Debug for EventId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "EventId({self})")
    }
}

impl fmt::Display for EventId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt_dotted(&self.to_bytes(), f)
    }
}

impl FromStr for EventId {
    type Err = ParseError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(EventId(u64::from_be_bytes(parse_dotted::<8>(s)?)))
    }
}

#[cfg(feature = "serde")]
#[cfg_attr(docsrs, doc(cfg(feature = "serde")))]
impl serde::Serialize for EventId {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(self)
    }
}

#[cfg(feature = "serde")]
#[cfg_attr(docsrs, doc(cfg(feature = "serde")))]
impl<'de> serde::Deserialize<'de> for EventId {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = <alloc::string::String as serde::Deserialize>::deserialize(d)?;
        s.parse().map_err(serde::de::Error::custom)
    }
}

/// The state reported in Producer/Consumer Identified messages.
///
/// The *Event Transport Standard* defines three variants of each
/// Identified message (MTI low bits `…4`, `…5`, `…7`). They let a node say
/// not only "I produce/consume this event" but also whether the condition
/// it represents currently holds. Events usually come in pairs (e.g.
/// "occupied"/"clear"); a producer typically reports one of the pair as
/// `Valid` and the other as `Invalid`, so a newly started panel can learn
/// the layout state by sending Identify Producer and reading the replies.
///
/// | Variant | Producer Identified MTI | Consumer Identified MTI |
/// |---|---|---|
/// | [`Valid`](EventState::Valid) | `0x0544` | `0x04C4` |
/// | [`Invalid`](EventState::Invalid) | `0x0545` | `0x04C5` |
/// | [`Unknown`](EventState::Unknown) | `0x0547` | `0x04C7` |
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum EventState {
    /// The event is currently "true" (e.g. the block is occupied right now):
    /// the last PCER for this ID describes the present condition.
    Valid,
    /// The event is currently "false": the condition it reports does not hold
    /// (typically because the paired event is the valid one).
    Invalid,
    /// The node does not know, or the event has no persistent state (a
    /// momentary push button, for example). This is the default.
    #[default]
    Unknown,
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn round_trip() {
        let e: EventId = "05.01.01.01.22.00.00.10".parse().unwrap();
        assert_eq!(e.0, 0x0501_0101_2200_0010);
        assert_eq!(e.to_string(), "05.01.01.01.22.00.00.10");
        assert_eq!(EventId::from_bytes(&e.to_bytes()).unwrap(), e);
        assert!("05.01.01.01.22.00.00".parse::<EventId>().is_err());
        assert!("05.01.01.01.22.00.00.1G".parse::<EventId>().is_err());
    }
}
