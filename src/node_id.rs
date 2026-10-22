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
/// ```
/// use rolcb::NodeId;
///
/// let id = NodeId::new(0x0501_0101_2200);
/// assert_eq!(NodeId::from_bytes(&id.to_bytes())?, id);
/// // Bits above 48 are dropped.
/// assert_eq!(NodeId::new(0xFFFF_0501_0101_2200), id);
/// # Ok::<(), rolcb::ParseError>(())
/// ```
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct NodeId(u64);

impl NodeId {
    /// The largest representable Node ID value, `0xFFFF_FFFF_FFFF` (48 bits).
    pub const MAX: u64 = 0xFFFF_FFFF_FFFF;

    /// Create from the low 48 bits of `v`. Higher bits are discarded.
    pub const fn new(v: u64) -> Self {
        NodeId(v & Self::MAX)
    }

    /// The Node ID as an integer (always `<=` [`NodeId::MAX`]).
    pub const fn as_u64(self) -> u64 {
        self.0
    }

    /// The six big-endian bytes, as transmitted on the wire (for example in
    /// the data field of an AMD frame or a Verified Node ID message).
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
        if b.len() < 6 {
            return Err(ParseError::Length);
        }
        let mut v = 0u64;
        for &x in &b[..6] {
            v = (v << 8) | x as u64;
        }
        Ok(NodeId(v))
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
        NodeId::from_bytes(&parse_dotted::<6>(s)?)
    }
}

#[cfg(feature = "serde")]
#[cfg_attr(docsrs, doc(cfg(feature = "serde")))]
impl serde::Serialize for NodeId {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(self)
    }
}

#[cfg(feature = "serde")]
#[cfg_attr(docsrs, doc(cfg(feature = "serde")))]
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
}
