//! 48-bit OpenLCB Node IDs.
//!
//! Every OpenLCB node carries a globally unique 48-bit **Node ID**, assigned
//! from blocks handed out by the OpenLCB group (see the *Unique Identifiers
//! Standard*). It is conventionally written as six dot-separated hex bytes,
//! most significant first: `05.01.01.01.22.00`. The `05.01.01.01.xx.xx`
//! range is the one OpenLCB reserves for individual hobbyists.
//!
//! On CAN the full Node ID is too large to fit into every frame header, so
//! nodes also reserve a short 12-bit *alias* (see [`crate::can`] and
//! [`crate::node`]). The Node ID still travels in the data bytes of
//! [Initialization Complete](crate::Message::InitializationComplete),
//! [Verified Node ID](crate::Message::VerifiedNodeId) and the AMD control
//! frame, which is how other nodes learn the alias → Node ID mapping.
//!
//! ```
//! use rolcb::NodeId;
//!
//! let id: NodeId = "05.01.01.01.22.0F".parse()?;
//! assert_eq!(id.as_u64(), 0x0501_0101_220F);
//! assert_eq!(id.to_string(), "05.01.01.01.22.0F");
//! assert_eq!(id.to_bytes(), [0x05, 0x01, 0x01, 0x01, 0x22, 0x0F]);
//!
//! // Separators are optional and may be '.', ':', '-' or ' '; hex is case-insensitive.
//! assert_eq!("05:01:01:01:22:0f".parse::<NodeId>()?, id);
//! assert_eq!("05010101220F".parse::<NodeId>()?, id);
//! # Ok::<(), rolcb::ParseError>(())
//! ```

use core::fmt;
use core::str::FromStr;

use crate::{fmt_dotted, parse_dotted, ParseError};

/// A unique 48-bit OpenLCB node identifier, e.g. `05.01.01.01.22.00`.
///
/// The value is stored in the low 48 bits of a `u64`; the constructor
/// guarantees the upper 16 bits are always zero. `Display` and `FromStr`
/// use the dotted-hex form, `Debug` prints `NodeId(05.01.01.01.22.00)`.
/// With the `serde` feature it (de)serializes as the dotted-hex string.
///
/// The all-zero ID ([`NodeId::default()`]) is not a valid node and is used
/// by this crate as a "not known" placeholder in a few lenient decoders.
///
/// # Conversions
///
/// | From | To | How | Out-of-range input |
/// |---|---|---|---|
/// | `u64` | `NodeId` | [`NodeId::new`] | **silently masked** to 48 bits |
/// | `u64` | `NodeId` | [`NodeId::checked`] | `None` |
/// | `u64` | `NodeId` | `NodeId::try_from(u64)` | `Err(`[`ParseError::Length`]`)` |
/// | `[u8; 6]` | `NodeId` | `NodeId::from([u8; 6])` | (cannot fail) |
/// | `&[u8]` | `NodeId` | [`NodeId::from_bytes`] | `Err` if shorter than 6 bytes |
/// | `NodeId` | `u64` | `u64::from(id)`, [`NodeId::as_u64`] | |
/// | `NodeId` | `[u8; 6]` | [`NodeId::to_bytes`] | |
///
/// ```
/// use rolcb::NodeId;
///
/// let id = NodeId::new(0x0501_0101_2200);
/// assert_eq!(NodeId::from_bytes(&id.to_bytes())?, id);
/// assert_eq!(NodeId::from([0x05, 0x01, 0x01, 0x01, 0x22, 0x00]), id);
/// assert_eq!(u64::from(id), 0x0501_0101_2200);
///
/// // `new` DROPS bits above 48 without telling you…
/// assert_eq!(NodeId::new(0xFFFF_0501_0101_2200), id);
/// // …so prefer `checked` / `try_from` for values from outside the program.
/// assert_eq!(NodeId::checked(0xFFFF_0501_0101_2200), None);
/// assert!(NodeId::try_from(0x1_0000_0000_0000_u64).is_err());
/// assert_eq!(NodeId::try_from(0x0501_0101_2200_u64)?, id);
/// # Ok::<(), rolcb::ParseError>(())
/// ```
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct NodeId(u64);

impl NodeId {
    /// The largest representable Node ID value, `0xFFFF_FFFF_FFFF` (48 bits).
    pub const MAX: u64 = 0xFFFF_FFFF_FFFF;

    /// Create from the low 48 bits of `v`.
    ///
    /// **Bits 48-63 are silently discarded**: `NodeId::new(0x1_0000_0000_0001)`
    /// is `00.00.00.00.00.01`. That is convenient for literals, but for
    /// values read from a file, a UI or another program use
    /// [`NodeId::checked`] or `NodeId::try_from(u64)`, which reject values
    /// above [`NodeId::MAX`] instead of truncating them.
    #[must_use]
    pub const fn new(v: u64) -> Self {
        NodeId(v & Self::MAX)
    }

    /// Create from `v` if it fits in 48 bits, otherwise `None`.
    ///
    /// ```
    /// use rolcb::NodeId;
    /// assert_eq!(NodeId::checked(NodeId::MAX).map(|n| n.as_u64()), Some(NodeId::MAX));
    /// assert_eq!(NodeId::checked(NodeId::MAX + 1), None);
    /// ```
    #[must_use]
    pub const fn checked(v: u64) -> Option<Self> {
        if v > Self::MAX {
            None
        } else {
            Some(NodeId(v))
        }
    }

    /// The Node ID as an integer (always `<=` [`NodeId::MAX`]).
    #[must_use]
    pub const fn as_u64(self) -> u64 {
        self.0
    }

    /// The six big-endian bytes, as transmitted on the wire (for example in
    /// the data field of an AMD frame or a Verified Node ID message).
    #[must_use]
    pub fn to_bytes(self) -> [u8; 6] {
        let b = self.0.to_be_bytes();
        [b[2], b[3], b[4], b[5], b[6], b[7]]
    }

    /// Read a Node ID from the first six big-endian bytes of `b`. Extra
    /// trailing bytes are ignored.
    ///
    /// # Errors
    ///
    /// [`ParseError::Length`] if `b` is shorter than six bytes.
    pub fn from_bytes(b: &[u8]) -> Result<Self, ParseError> {
        let arr = b.first_chunk::<6>().ok_or(ParseError::Length)?;
        Ok(NodeId::from(*arr))
    }
}

impl From<[u8; 6]> for NodeId {
    /// Big-endian bytes, as on the wire.
    fn from(b: [u8; 6]) -> Self {
        NodeId(u64::from_be_bytes([
            0, 0, b[0], b[1], b[2], b[3], b[4], b[5],
        ]))
    }
}

impl From<NodeId> for u64 {
    fn from(n: NodeId) -> u64 {
        n.0
    }
}

impl TryFrom<u64> for NodeId {
    type Error = ParseError;

    /// Fails with [`ParseError::Length`] if `v` does not fit in 48 bits.
    fn try_from(v: u64) -> Result<Self, ParseError> {
        NodeId::checked(v).ok_or(ParseError::Length)
    }
}

impl fmt::Debug for NodeId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "NodeId({self})")
    }
}

impl fmt::Display for NodeId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt_dotted(&self.to_bytes(), f)
    }
}

impl FromStr for NodeId {
    type Err = ParseError;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(NodeId::from(parse_dotted::<6>(s)?))
    }
}

#[cfg(feature = "serde")]
impl serde::Serialize for NodeId {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(self)
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for NodeId {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let s = <alloc::string::String as serde::Deserialize>::deserialize(d)?;
        s.parse().map_err(serde::de::Error::custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn round_trip() {
        let n: NodeId = "05.01.01.01.22.0F".parse().unwrap();
        assert_eq!(n.as_u64(), 0x0501_0101_220F);
        assert_eq!(n.to_string(), "05.01.01.01.22.0F");
        assert_eq!(NodeId::from_bytes(&n.to_bytes()).unwrap(), n);
        assert_eq!("05010101220f".parse::<NodeId>().unwrap(), n);
        assert!("05.01.01".parse::<NodeId>().is_err());
    }

    #[test]
    fn conversions() {
        let n = NodeId::new(0x0501_0101_220F);
        assert_eq!(u64::from(n), 0x0501_0101_220F);
        assert_eq!(NodeId::from(n.to_bytes()), n);
        assert_eq!(NodeId::try_from(NodeId::MAX), Ok(NodeId::new(NodeId::MAX)));
        assert_eq!(NodeId::try_from(NodeId::MAX + 1), Err(ParseError::Length));
        assert_eq!(NodeId::try_from(u64::MAX), Err(ParseError::Length));
        assert_eq!(NodeId::checked(0), Some(NodeId::default()));
        assert_eq!(NodeId::checked(1 << 48), None);
        assert_eq!(NodeId::new(1 << 48), NodeId::default());
        assert_eq!(
            NodeId::from_bytes(&[1, 2, 3, 4, 5]),
            Err(ParseError::Length)
        );
        assert_eq!(
            NodeId::from_bytes(&[1, 2, 3, 4, 5, 6, 7]),
            Ok(NodeId::new(0x0102_0304_0506))
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip() {
        let n = NodeId::new(0x0501_0101_220F);
        let json = serde_json::to_string(&n).unwrap();
        assert_eq!(json, "\"05.01.01.01.22.0F\"");
        assert_eq!(serde_json::from_str::<NodeId>(&json).unwrap(), n);
        assert_eq!(
            serde_json::from_str::<NodeId>("\"05010101220f\"").unwrap(),
            n
        );
        assert!(serde_json::from_str::<NodeId>("\"05.01\"").is_err());
        assert!(serde_json::from_str::<NodeId>("17").is_err());
    }
}
