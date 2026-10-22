//! CAN frame layer: 29-bit extended identifiers as laid out by the OpenLCB
//! CAN Frame Transfer Standard.
//!
//! ```text
//!  bit 28     : reserved, always 1
//!  bit 27     : 1 = OpenLCB message frame, 0 = CAN control frame
//!  bits 26-24 : frame type (message) / sequence (control)
//!  bits 23-12 : variable field (CAN-MTI, destination alias, or Node ID chunk)
//!  bits 11-0  : source alias
//! ```
//!
//! OpenLCB on CAN uses only CAN 2.0B *extended* frames. The 29-bit
//! identifier is split into fields as shown above; the *CAN Frame Transfer
//! Standard* defines two families of frames:
//!
//! * **Control frames** (bit 27 = 0) manage aliases. They never carry an
//!   OpenLCB message. See the alias login below.
//! * **Message frames** (bit 27 = 1) carry OpenLCB messages. Bits 26-24
//!   select the [`MessageFrameType`]: a global/addressed message whose
//!   CAN-MTI is in the variable field, one of four datagram frame kinds
//!   with the destination alias in the variable field, or a stream frame.
//!
//! Because CAN arbitration lets the lowest identifier win, the layout also
//! acts as a priority scheme: control frames beat message frames, and lower
//! MTIs beat higher ones.
//!
//! # Aliases and the login sequence
//!
//! A 48-bit Node ID does not fit into the identifier, so each node reserves
//! a 12-bit *alias* (see [`crate::alias`]) and uses it as the source field
//! of every frame. Reserving an alias takes these control frames:
//!
//! ```text
//!   CID7  [Node ID bits 47-36 | alias]   "I want this alias" (4 frames,
//!   CID6  [Node ID bits 35-24 | alias]    together they spell out the
//!   CID5  [Node ID bits 23-12 | alias]    whole Node ID, so two nodes
//!   CID4  [Node ID bits 11-0  | alias]    can't send identical frames)
//!     … wait at least 200 ms; if anyone objects or uses the alias, start over …
//!   RID   [alias]                         "Reserved"
//!   AMD   [alias] data = Node ID          "alias ↔ Node ID mapping defined"
//!   Initialization Complete (a normal message frame) — node is now online
//! ```
//!
//! After that the node must *defend* its alias: if another node sends a CID
//! for it, answer with RID; if another node actually transmits with it,
//! send AMR (Alias Map Reset) and choose a new one. AME (Alias Mapping
//! Enquiry) asks nodes to repeat their AMD, either all of them (empty data)
//! or the one whose Node ID is in the data. [`Node`](crate::Node) implements
//! all of this; [`cid_frames`] and [`CanHeader`] are the building blocks.
//!
//! # Example
//!
//! ```
//! use rolcb::can::{CanFrame, CanHeader, MessageFrameType};
//! use rolcb::Mti;
//!
//! // A Verify Node ID Global message from alias 0xABC.
//! let id = CanHeader::Message { mti: Mti::VERIFY_NODE_ID_GLOBAL, alias: 0xABC }.encode();
//! assert_eq!(id, 0x1949_0ABC);
//!
//! // Decoding is total: any 29-bit value maps to some CanHeader.
//! assert_eq!(CanHeader::decode(0x1070_1285), CanHeader::Amd { alias: 0x285 });
//! assert_eq!(
//!     CanHeader::decode(0x1A45_6123),
//!     CanHeader::Datagram { kind: MessageFrameType::DatagramOnly, dest: 0x456, alias: 0x123 },
//! );
//!
//! let frame = CanFrame::new(id, &[])?;
//! assert_eq!(frame.src_alias(), 0xABC);
//! assert!(matches!(frame.header(), CanHeader::Message { mti: Mti::VERIFY_NODE_ID_GLOBAL, .. }));
//! # Ok::<(), rolcb::ParseError>(())
//! ```

use crate::mti::Mti;
use crate::node_id::NodeId;
use crate::ParseError;

/// A CAN 2.0B frame with a 29-bit identifier and up to 8 data bytes.
///
/// `CanFrame` is a small `Copy` value (no heap allocation), the unit of
/// exchange between this crate and your I/O layer. Convert to and from
/// GridConnect text with [`gridconnect`](crate::gridconnect), or map to a
/// SocketCAN / embedded CAN driver frame using [`CanFrame::id`] and
/// [`CanFrame::data`]. `Debug` prints `CanFrame(19490ABC [01 02])`;
/// `Display` and `FromStr` use GridConnect text (`:X19490ABCN0102;`, see
/// [`gridconnect`](crate::gridconnect)).
///
/// ```
/// use rolcb::CanFrame;
///
/// let f = CanFrame::new(0x195B_4123, &[5, 1, 1, 1, 0x22, 0, 0, 0x10])?;
/// assert_eq!(f.data().len(), 8);
/// assert_eq!(f.src_alias(), 0x123);
/// assert!(CanFrame::new(0x195B_4123, &[0; 9]).is_err());
///
/// assert_eq!(f.to_string(), ":X195B4123N0501010122000010;");
/// assert_eq!(":X195B4123N0501010122000010;".parse::<CanFrame>()?, f);
/// # Ok::<(), rolcb::ParseError>(())
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct CanFrame {
    /// The 29-bit extended identifier (upper three bits always zero when
    /// built with [`CanFrame::new`]). Decode it with [`CanFrame::header`].
    pub id: u32,
    len: u8,
    data: [u8; 8],
}

impl CanFrame {
    /// Build a frame. `id` is masked to 29 bits.
    ///
    /// # Errors
    ///
    /// [`ParseError::Length`] if `data` is longer than 8 bytes.
    pub fn new(id: u32, data: &[u8]) -> Result<Self, ParseError> {
        if data.len() > 8 {
            return Err(ParseError::Length);
        }
        let mut d = [0u8; 8];
        d[..data.len()].copy_from_slice(data);
        Ok(CanFrame {
            id: id & 0x1FFF_FFFF,
            len: data.len() as u8,
            data: d,
        })
    }

    /// Like [`CanFrame::new`] for internal use where lengths are guaranteed
    /// by construction. Bytes past the eighth are dropped (and trip a
    /// `debug_assert!`).
    pub(crate) fn of(id: u32, data: &[u8]) -> Self {
        debug_assert!(data.len() <= 8, "CAN payload over 8 bytes");
        let n = data.len().min(8);
        let mut d = [0u8; 8];
        d[..n].copy_from_slice(&data[..n]);
        CanFrame {
            id: id & 0x1FFF_FFFF,
            len: n as u8,
            data: d,
        }
    }

    /// The data bytes (0 to 8).
    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    /// Decode the identifier into its OpenLCB meaning.
    #[must_use]
    pub fn header(&self) -> CanHeader {
        CanHeader::decode(self.id)
    }

    /// The 12-bit source alias (bits 11-0 of the identifier). Present in
    /// every OpenLCB frame, control or message.
    #[must_use]
    pub fn src_alias(&self) -> u16 {
        (self.id & 0xFFF) as u16
    }
}

impl core::fmt::Display for CanFrame {
    /// GridConnect text, e.g. `:X19490ABCN0102;` (no newline). Same as
    /// [`gridconnect::format`](crate::gridconnect::format).
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        crate::gridconnect::write(self, f)
    }
}

impl core::str::FromStr for CanFrame {
    type Err = ParseError;

    /// Parse GridConnect text, as [`gridconnect::parse`](crate::gridconnect::parse).
    fn from_str(s: &str) -> Result<Self, ParseError> {
        crate::gridconnect::parse(s)
    }
}

impl core::fmt::Debug for CanFrame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "CanFrame({:08X} [", self.id)?;
        for (i, b) in self.data().iter().enumerate() {
            if i > 0 {
                f.write_str(" ")?;
            }
            write!(f, "{b:02X}")?;
        }
        f.write_str("])")
    }
}

/// OpenLCB message frame types (bits 26-24 when bit 27 is set).
///
/// | Bits 26-24 | Variant | Variable field (bits 23-12) |
/// |---|---|---|
/// | 0 | [`Reserved(0)`](MessageFrameType::Reserved) | — |
/// | 1 | [`GlobalAddressed`](MessageFrameType::GlobalAddressed) | CAN-MTI |
/// | 2 | [`DatagramOnly`](MessageFrameType::DatagramOnly) | destination alias |
/// | 3 | [`DatagramFirst`](MessageFrameType::DatagramFirst) | destination alias |
/// | 4 | [`DatagramMiddle`](MessageFrameType::DatagramMiddle) | destination alias |
/// | 5 | [`DatagramFinal`](MessageFrameType::DatagramFinal) | destination alias |
/// | 6 | [`Reserved(6)`](MessageFrameType::Reserved) | — |
/// | 7 | [`Stream`](MessageFrameType::Stream) | destination alias |
///
/// Datagrams (up to 72 bytes, *Datagram Transport Standard*) are split into
/// 8-byte chunks: a datagram that fits in one frame is sent as
/// `DatagramOnly`; longer ones as `DatagramFirst`, zero or more
/// `DatagramMiddle`, then `DatagramFinal`. Unlike addressed messages, the
/// whole 8 data bytes are payload because the destination is in the header.
///
/// In this crate only the datagram kinds appear inside
/// [`CanHeader::Datagram`]; global/addressed and stream frames have their
/// own [`CanHeader`] variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MessageFrameType {
    /// Global or addressed message; the variable field holds the CAN-MTI.
    GlobalAddressed,
    /// Frame type 2: a complete datagram in a single frame (0-8 bytes).
    DatagramOnly,
    /// Frame type 3: first frame of a multi-frame datagram.
    DatagramFirst,
    /// Frame type 4: a middle frame of a multi-frame datagram.
    DatagramMiddle,
    /// Frame type 5: the last frame of a multi-frame datagram.
    DatagramFinal,
    /// Frame type 7: stream data (*Stream Transport Standard*).
    Stream,
    /// Frame types 0 and 6, reserved by the standard. Holds the raw value.
    Reserved(u8),
}

/// A decoded CAN identifier.
///
/// [`CanHeader::decode`] interprets any 29-bit identifier and
/// [`CanHeader::encode`] builds one; `encode(decode(id)) == id` for every
/// identifier with bit 28 set. Every variant carries the source `alias`
/// (bits 11-0), available uniformly through [`CanHeader::src_alias`].
///
/// | Bit 27 | Bits 26-24 | Bits 23-12 | Variant |
/// |---|---|---|---|
/// | 0 | 7, 6, 5, 4 | Node ID chunk | [`Cid`](CanHeader::Cid) |
/// | 0 | 0 | `0x700` | [`Rid`](CanHeader::Rid) |
/// | 0 | 0 | `0x701` | [`Amd`](CanHeader::Amd) |
/// | 0 | 0 | `0x702` | [`Ame`](CanHeader::Ame) |
/// | 0 | 0 | `0x703` | [`Amr`](CanHeader::Amr) |
/// | 0 | other | other | [`OtherControl`](CanHeader::OtherControl) (e.g. error information reports) |
/// | 1 | 1 | CAN-MTI | [`Message`](CanHeader::Message) |
/// | 1 | 2-5 | dest alias | [`Datagram`](CanHeader::Datagram) |
/// | 1 | 7 | dest alias | [`Stream`](CanHeader::Stream) |
/// | 1 | 0, 6 | any | [`Reserved`](CanHeader::Reserved) |
///
/// ```
/// use rolcb::can::CanHeader;
/// use rolcb::Mti;
///
/// let h = CanHeader::Message { mti: Mti::PCER, alias: 0x123 };
/// assert_eq!(h.encode(), 0x195B_4123);
/// assert_eq!(CanHeader::decode(0x195B_4123), h);
/// assert_eq!(CanHeader::Rid { alias: 0x285 }.encode(), 0x1070_0285);
/// ```
///
/// The enum is `#[non_exhaustive]`: match it with a `_` arm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum CanHeader {
    /// Check ID frame. `seq` is 7..=4; `node_id_bits` is the 12-bit chunk of
    /// the Node ID being reserved (7 = bits 47-36 ... 4 = bits 11-0).
    ///
    /// The first step of alias reservation. A node sends CID7, CID6, CID5,
    /// CID4 with its candidate alias, then waits 200 ms for objections. A
    /// node that already owns the alias answers with [`CanHeader::Rid`].
    /// Build the whole set with [`cid_frames`].
    Cid {
        /// Sequence number 7, 6, 5 or 4 (bits 26-24).
        seq: u8,
        /// The 12-bit slice of the Node ID this frame carries.
        node_id_bits: u16,
        /// The alias being requested.
        alias: u16,
    },
    /// Reserve ID. Sent after the 200 ms CID wait to claim the alias, and by
    /// an alias owner to object to someone else's CID for its alias.
    Rid {
        /// The alias being reserved or defended.
        alias: u16,
    },
    /// Alias Map Definition. Data is the 6-byte Node ID. Announces "this
    /// alias now belongs to this Node ID"; the node becomes *permitted* to
    /// send messages. Also the reply to [`CanHeader::Ame`].
    Amd {
        /// The alias being mapped.
        alias: u16,
    },
    /// Alias Mapping Enquiry. With empty data, every permitted node replies
    /// with its AMD; with a 6-byte Node ID, only that node replies. Used by
    /// gateways and tools to rebuild their alias tables.
    Ame {
        /// The alias of the node asking.
        alias: u16,
    },
    /// Alias Map Reset. Data is the 6-byte Node ID. The sender is giving up
    /// this alias (for example after a collision); others must forget the
    /// mapping and any partial multi-frame messages from it.
    Amr {
        /// The alias being released.
        alias: u16,
    },
    /// Some other control frame (for example the error information report
    /// frames), passed through undecoded.
    OtherControl {
        /// Bits 26-24.
        seq: u8,
        /// Bits 23-12.
        var: u16,
        /// Source alias.
        alias: u16,
    },
    /// Global or addressed OpenLCB message.
    ///
    /// For addressed MTIs the destination alias is *not* in the header but
    /// in the first two data bytes; see
    /// [`Message::to_frames`](crate::Message::to_frames).
    Message {
        /// The message type (only the low 12 bits travel on CAN).
        mti: Mti,
        /// Source alias.
        alias: u16,
    },
    /// Datagram frame addressed to `dest`.
    Datagram {
        /// Which part of the datagram this frame is (only the four datagram
        /// variants of [`MessageFrameType`] are produced by `decode`).
        kind: MessageFrameType,
        /// Destination alias.
        dest: u16,
        /// Source alias.
        alias: u16,
    },
    /// Stream data frame addressed to `dest`.
    Stream {
        /// Destination alias.
        dest: u16,
        /// Source alias.
        alias: u16,
    },
    /// Reserved frame type.
    Reserved {
        /// The frame type, 0 or 6.
        kind: u8,
        /// Bits 23-12.
        var: u16,
        /// Source alias.
        alias: u16,
    },
}

const RESERVED_BIT: u32 = 1 << 28;
const MESSAGE_BIT: u32 = 1 << 27;

impl CanHeader {
    /// Interpret a 29-bit identifier. Never fails: unrecognised values map to
    /// [`OtherControl`](CanHeader::OtherControl) or
    /// [`Reserved`](CanHeader::Reserved). Bit 28 is not checked.
    #[must_use]
    pub fn decode(id: u32) -> CanHeader {
        let alias = (id & 0xFFF) as u16;
        let var = ((id >> 12) & 0xFFF) as u16;
        let kind = ((id >> 24) & 0x7) as u8;
        if id & MESSAGE_BIT == 0 {
            match (kind, var) {
                (4..=7, _) => CanHeader::Cid {
                    seq: kind,
                    node_id_bits: var,
                    alias,
                },
                (0, 0x700) => CanHeader::Rid { alias },
                (0, 0x701) => CanHeader::Amd { alias },
                (0, 0x702) => CanHeader::Ame { alias },
                (0, 0x703) => CanHeader::Amr { alias },
                _ => CanHeader::OtherControl {
                    seq: kind,
                    var,
                    alias,
                },
            }
        } else {
            match kind {
                1 => CanHeader::Message {
                    mti: Mti::from_can_mti(var),
                    alias,
                },
                2 => CanHeader::Datagram {
                    kind: MessageFrameType::DatagramOnly,
                    dest: var,
                    alias,
                },
                3 => CanHeader::Datagram {
                    kind: MessageFrameType::DatagramFirst,
                    dest: var,
                    alias,
                },
                4 => CanHeader::Datagram {
                    kind: MessageFrameType::DatagramMiddle,
                    dest: var,
                    alias,
                },
                5 => CanHeader::Datagram {
                    kind: MessageFrameType::DatagramFinal,
                    dest: var,
                    alias,
                },
                7 => CanHeader::Stream { dest: var, alias },
                k => CanHeader::Reserved {
                    kind: k,
                    var,
                    alias,
                },
            }
        }
    }

    /// Build the 29-bit identifier (with the reserved bit 28 set).
    ///
    /// A [`Datagram`](CanHeader::Datagram) whose `kind` is not one of the
    /// four datagram variants is encoded as `DatagramOnly`. Every field is
    /// masked to its width (3 bits for `seq`/`kind`, 12 bits for aliases and
    /// the variable field), so an out-of-range value can never spill into a
    /// neighbouring field.
    #[must_use]
    pub fn encode(&self) -> u32 {
        let ctl = |seq: u32, var: u16, alias: u16| {
            RESERVED_BIT
                | ((seq & 0x7) << 24)
                | ((var as u32 & 0xFFF) << 12)
                | (alias as u32 & 0xFFF)
        };
        let msg = |kind: u32, var: u16, alias: u16| ctl(kind, var, alias) | MESSAGE_BIT;
        match *self {
            CanHeader::Cid {
                seq,
                node_id_bits,
                alias,
            } => ctl(seq as u32, node_id_bits, alias),
            CanHeader::Rid { alias } => ctl(0, 0x700, alias),
            CanHeader::Amd { alias } => ctl(0, 0x701, alias),
            CanHeader::Ame { alias } => ctl(0, 0x702, alias),
            CanHeader::Amr { alias } => ctl(0, 0x703, alias),
            CanHeader::OtherControl { seq, var, alias } => ctl(seq as u32, var, alias),
            CanHeader::Message { mti, alias } => msg(1, mti.can_mti(), alias),
            CanHeader::Datagram { kind, dest, alias } => {
                let k = match kind {
                    MessageFrameType::DatagramOnly => 2,
                    MessageFrameType::DatagramFirst => 3,
                    MessageFrameType::DatagramMiddle => 4,
                    MessageFrameType::DatagramFinal => 5,
                    _ => 2,
                };
                msg(k, dest, alias)
            }
            CanHeader::Stream { dest, alias } => msg(7, dest, alias),
            CanHeader::Reserved { kind, var, alias } => msg(kind as u32, var, alias),
        }
    }

    /// The source alias, whatever the frame type.
    #[must_use]
    pub fn src_alias(&self) -> u16 {
        match *self {
            CanHeader::Cid { alias, .. }
            | CanHeader::Rid { alias }
            | CanHeader::Amd { alias }
            | CanHeader::Ame { alias }
            | CanHeader::Amr { alias }
            | CanHeader::OtherControl { alias, .. }
            | CanHeader::Message { alias, .. }
            | CanHeader::Datagram { alias, .. }
            | CanHeader::Stream { alias, .. }
            | CanHeader::Reserved { alias, .. } => alias,
        }
    }
}

/// Build the four CID frames (CID7..CID4) reserving `alias` for `node`.
///
/// Send them in order, then wait [`ALIAS_RESERVE_MS`](crate::node::ALIAS_RESERVE_MS)
/// (200 ms) before sending RID and AMD. [`Node::start`](crate::Node::start)
/// does this for you.
///
/// ```
/// use rolcb::can::{cid_frames, CanHeader};
/// use rolcb::NodeId;
///
/// let f = cid_frames(NodeId::new(0x0501_0101_2200), 0x285);
/// assert_eq!(f[0].id, 0x1705_0285); // CID7: bits 47-36 = 0x050
/// assert_eq!(f[3].id, 0x1420_0285); // CID4: bits 11-0  = 0x200
/// assert!(f.iter().all(|f| f.data().is_empty()));
/// assert!(matches!(f[1].header(), CanHeader::Cid { seq: 6, node_id_bits: 0x101, alias: 0x285 }));
/// ```
#[must_use]
pub fn cid_frames(node: NodeId, alias: u16) -> [CanFrame; 4] {
    let n = node.as_u64();
    let chunk = |seq: u8| {
        let shift = (seq as u32 - 4) * 12;
        let bits = ((n >> shift) & 0xFFF) as u16;
        CanFrame::of(
            CanHeader::Cid {
                seq,
                node_id_bits: bits,
                alias,
            }
            .encode(),
            &[],
        )
    };
    [chunk(7), chunk(6), chunk(5), chunk(4)]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn message_header() {
        let h = CanHeader::Message {
            mti: Mti::VERIFY_NODE_ID_GLOBAL,
            alias: 0xABC,
        };
        assert_eq!(h.encode(), 0x1949_0ABC);
        assert_eq!(CanHeader::decode(0x1949_0ABC), h);
        let h = CanHeader::Message {
            mti: Mti::PCER,
            alias: 0x123,
        };
        assert_eq!(h.encode(), 0x195B_4123);
        assert_eq!(CanHeader::decode(0x195B_4123), h);
    }

    #[test]
    fn control_headers() {
        assert_eq!(CanHeader::Rid { alias: 0x285 }.encode(), 0x1070_0285);
        assert_eq!(CanHeader::Amd { alias: 0x285 }.encode(), 0x1070_1285);
        assert_eq!(CanHeader::Ame { alias: 0x000 }.encode(), 0x1070_2000);
        assert_eq!(CanHeader::Amr { alias: 0x285 }.encode(), 0x1070_3285);
        assert_eq!(
            CanHeader::decode(0x1070_1285),
            CanHeader::Amd { alias: 0x285 }
        );
    }

    #[test]
    fn cid_sequence() {
        let node = NodeId::new(0x0501_0101_2200);
        let f = cid_frames(node, 0x285);
        assert_eq!(f[0].id, 0x1705_0285); // bits 47-36 = 0x050
        assert_eq!(f[1].id, 0x1610_1285); // bits 35-24 = 0x101
        assert_eq!(f[2].id, 0x1501_2285); // bits 23-12 = 0x012
        assert_eq!(f[3].id, 0x1420_0285); // bits 11-0  = 0x200
        assert!(matches!(
            f[0].header(),
            CanHeader::Cid {
                seq: 7,
                node_id_bits: 0x050,
                alias: 0x285
            }
        ));
    }

    #[test]
    fn datagram_header() {
        let h = CanHeader::Datagram {
            kind: MessageFrameType::DatagramOnly,
            dest: 0x456,
            alias: 0x123,
        };
        assert_eq!(h.encode(), 0x1A45_6123);
        assert_eq!(CanHeader::decode(0x1A45_6123), h);
    }

    /// `encode(decode(id)) == id` for identifiers with bit 28 set, sampled
    /// across the whole 29-bit space (plus every control/frame-type corner).
    #[test]
    fn header_sweep() {
        let check = |id: u32| {
            let id = id | RESERVED_BIT;
            let h = CanHeader::decode(id);
            assert_eq!(h.encode(), id, "{id:08X} -> {h:?}");
            assert_eq!(h.src_alias() as u32, id & 0xFFF);
        };
        let mut id = 0u32;
        while id < 0x1000_0000 {
            check(id);
            id += 0x0000_1F3D; // odd stride, hits every alias/var nibble pattern
        }
        for kind in 0..8u32 {
            for var in [0x000, 0x700, 0x701, 0x702, 0x703, 0x704, 0xFFF] {
                for msg in [0, MESSAGE_BIT] {
                    check(msg | (kind << 24) | (var << 12) | 0xABC);
                }
            }
        }
    }

    #[test]
    fn encode_masks_fields() {
        let h = CanHeader::Cid {
            seq: 0xFF,
            node_id_bits: 0xFFFF,
            alias: 0xFFFF,
        };
        assert_eq!(h.encode(), 0x17FF_FFFF);
        let h = CanHeader::OtherControl {
            seq: 0x0B,
            var: 0x123,
            alias: 1,
        };
        assert_eq!(h.encode() & !0x1FFF_FFFF, 0);
        assert_eq!((h.encode() >> 24) & 7, 3);
        let h = CanHeader::Reserved {
            kind: 0x0E,
            var: 0,
            alias: 0,
        };
        assert_eq!(h.encode(), 0x1E00_0000);
    }

    #[test]
    fn frame_display_parse() {
        let f = CanFrame::new(0x1949_0ABC, &[1, 2]).unwrap();
        assert_eq!(alloc::format!("{f}"), ":X19490ABCN0102;");
        assert_eq!(":X19490ABCN0102;".parse::<CanFrame>().unwrap(), f);
        assert_eq!(alloc::format!("{f:?}"), "CanFrame(19490ABC [01 02])");
        assert_eq!(CanFrame::of(0xFFFF_FFFF, &[]).id, 0x1FFF_FFFF);
    }
}
