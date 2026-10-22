//! 12-bit alias generation, using the pseudo-random sequence suggested by
//! the CAN Frame Transfer Standard (seeded from the Node ID).
//!
//! # Why CAN needs aliases
//!
//! A CAN 2.0B extended identifier has 29 bits, of which OpenLCB can only
//! spare 12 for the source address. A 48-bit [`NodeId`] does not fit, so on
//! CAN each node picks a short **alias** (1..=0xFFF) and uses it as the
//! source address in every frame header. Addressed messages likewise carry
//! the destination's alias rather than its Node ID. Aliases are local to
//! one CAN segment; gateways and TCP links translate them.
//!
//! Since aliases are chosen without coordination, two nodes may pick the
//! same one. The *CAN Frame Transfer Standard* resolves this with a
//! reservation protocol (CID frames, a 200 ms wait, RID, AMD; see
//! [`crate::node`]) and by having each node generate candidates from a
//! pseudo-random sequence seeded with its own Node ID. Different nodes then
//! almost always try different aliases, and after a collision a node simply
//! moves on to the next value in its sequence.
//!
//! The sequence here is the 48-bit LFSR-like generator recommended by the
//! standard (also used by OpenMRN and JMRI), so a given Node ID tends to
//! receive the same alias after each power cycle.
//!
//! ```
//! use rolcb::alias::AliasGenerator;
//! use rolcb::NodeId;
//!
//! let mut gen = AliasGenerator::new(NodeId::new(0x0501_0101_2200));
//! let first = gen.next_alias();
//! assert!((1..=0xFFF).contains(&first));
//! assert_ne!(gen.next_alias(), first);
//!
//! // Deterministic: the same Node ID gives the same sequence.
//! assert_eq!(AliasGenerator::new(NodeId::new(0x0501_0101_2200)).next_alias(), first);
//! ```

use crate::node_id::NodeId;

/// Produces a deterministic sequence of candidate aliases for a node.
///
/// [`Node`](crate::Node) owns one of these and draws a fresh candidate each
/// time it (re)starts alias reservation. You only need it directly when
/// implementing your own CAN login logic.
#[derive(Debug, Clone)]
pub struct AliasGenerator {
    lfsr1: u32,
    lfsr2: u32,
}

impl AliasGenerator {
    /// Seed the generator from a Node ID: the upper 24 bits seed one half of
    /// the state and the lower 24 bits the other.
    pub fn new(node: NodeId) -> Self {
        let n = node.as_u64();
        AliasGenerator {
            lfsr1: ((n >> 24) & 0xFF_FFFF) as u32,
            lfsr2: (n & 0xFF_FFFF) as u32,
        }
    }

    fn step(&mut self) {
        let temp1 = ((self.lfsr1 << 9) | ((self.lfsr2 >> 15) & 0x1FF)) & 0xFF_FFFF;
        let temp2 = (self.lfsr2 << 9) & 0xFF_FFFF;
        self.lfsr1 = self.lfsr1 + temp1 + 0x1B_0CA3;
        self.lfsr2 = self.lfsr2 + temp2 + 0x7A_4BA9;
        self.lfsr1 = (self.lfsr1 & 0xFF_FFFF) + ((self.lfsr2 & 0xFF00_0000) >> 24);
        self.lfsr2 &= 0xFF_FFFF;
    }

    fn current(&self) -> u16 {
        ((self.lfsr1 ^ self.lfsr2 ^ (self.lfsr1 >> 12) ^ (self.lfsr2 >> 12)) & 0xFFF) as u16
    }

    /// Return the next candidate alias, in `1..=0xFFF`.
    ///
    /// Alias 0 is reserved by the standard and is skipped.
    pub fn next_alias(&mut self) -> u16 {
        loop {
            self.step();
            let a = self.current();
            if a != 0 {
                return a;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_and_varied() {
        let node = NodeId::new(0x0501_0101_2200);
        let mut a = AliasGenerator::new(node);
        let mut b = AliasGenerator::new(node);
        let seq: alloc::vec::Vec<u16> = (0..64).map(|_| a.next_alias()).collect();
        for &x in &seq {
            assert_eq!(x, b.next_alias());
            assert!(x != 0 && x <= 0xFFF);
        }
        // Successive aliases should not all collide.
        let mut sorted = seq.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert!(sorted.len() > 56);
        // Different nodes start differently.
        let mut c = AliasGenerator::new(NodeId::new(0x0501_0101_2201));
        assert_ne!(c.next_alias(), seq[0]);
    }

    #[test]
    fn zero_node_id_still_works() {
        let mut g = AliasGenerator::new(NodeId::new(0));
        assert_ne!(g.next_alias(), 0);
    }
}
