//! A sans-IO virtual OpenLCB node on a CAN segment.
//!
//! The node handles alias reservation and defence, the login sequence,
//! the mandatory replies (Verify Node ID, AME, Protocol Support, SNIP) and
//! event identification. Applications drive it with three calls:
//!
//! * [`Node::receive`] for every frame read from the bus,
//! * [`Node::poll`] periodically (or at [`Node::next_deadline`]),
//! * [`Node::take_outgoing`] to collect frames to transmit.
//!
//! Time is passed in as milliseconds from any monotonic clock.
//!
//! # Lifecycle
//!
//! ```text
//!            start(now)                 poll(now >= now+200ms)
//!  Inhibited ─────────► Reserving ─────────────────────────► Permitted
//!      ▲        (4× CID sent)  │  (RID, AMD, Init Complete,       │
//!      │                       │   Identified for every event,    │
//!      │                       │   then queued messages)          │
//!      │   alias collision:    │                                  │
//!      │   restart with next ◄─┘                                  │
//!      │   candidate alias                                        │
//!      │                       ◄──────────────────────────────────┘
//!      │                         someone else *used* our alias, or
//!      │                         start() called again: AMR, then start
//!      │                         over (no 2nd Init Complete)
//!      └──────────── reset() (link lost) ─────────────────────────┘
//! ```
//!
//! While **reserving**, any frame from our candidate alias means someone
//! else has it, so the node picks the next candidate and sends new CIDs.
//! Once **permitted**, a CID for our alias is answered with RID (we keep
//! it), while any other frame using our alias forces us to release it
//! with AMR and reserve a new one, as the *CAN Frame Transfer Standard*
//! requires.
//!
//! # Automatic behaviour
//!
//! Once permitted, [`Node::receive`] answers these by itself:
//!
//! | Incoming | Reply |
//! |---|---|
//! | Verify Node ID Global (no ID, or our ID) / Addressed to us | Verified Node ID |
//! | AME (empty, or our Node ID) | AMD |
//! | Protocol Support Inquiry | Protocol Support Reply with [`Node::set_protocols`] flags |
//! | SNIP Request | SNIP Reply from [`NodeInfo`] |
//! | Identify Events Global / Addressed to us | Producer/Consumer Identified for every registered event |
//! | Identify Consumer / Producer for a registered event | Consumer / Producer Identified with its state |
//! | Datagram to us | Datagram Rejected (`0x1043`, [`NOT_IMPLEMENTED_UNKNOWN_MTI`](error_code::NOT_IMPLEMENTED_UNKNOWN_MTI)) |
//! | Traction Command to us, unless [`protocol::TRACTION_CONTROL`] is set or it is a Controller Changing Notify (`20 04`) | Optional Interaction Rejected (`0x1043`) |
//! | Any other unknown addressed message to us | Optional Interaction Rejected (`0x1043`) |
//!
//! Replies (Traction Control Reply, Datagram Received OK / Rejected,
//! Optional Interaction Rejected, SNIP reply, …) are never rejected.
//! Everything (including messages that were auto-answered) is still
//! returned to the application as a [`Received`].
//!
//! The node also produces [`EventId::DUPLICATE_NODE_ID`] (as a PCER, once)
//! the first time it sees another node announce its Node ID; see
//! [`Node::duplicate_node_id_detected`].
//!
//! # Example: login driven by a fake clock
//!
//! ```
//! use rolcb::can::CanHeader;
//! use rolcb::node::ALIAS_RESERVE_MS;
//! use rolcb::{EventId, EventState, Message, Node, NodeId, NodeInfo};
//!
//! let mut node = Node::new(
//!     NodeId::new(0x0501_0101_2299),
//!     NodeInfo { manufacturer: "Example".into(), model: "Panel".into(), ..Default::default() },
//! );
//! node.add_consumer(EventId(0x0501_0101_2200_0010), EventState::Unknown);
//! // Sending before login is fine; the message is held until we have an alias.
//! node.send(Message::Pcer(EventId(0x0501_0101_2299_0001)));
//!
//! let mut now = 1_000;
//! node.start(now);
//! let cids = node.take_outgoing();
//! assert_eq!(cids.len(), 4);
//! assert!(cids.iter().all(|f| matches!(f.header(), CanHeader::Cid { .. })));
//! assert_eq!(node.next_deadline(), Some(now + ALIAS_RESERVE_MS));
//!
//! // Nothing happens until 200 ms have passed.
//! now += 199;
//! node.poll(now);
//! assert!(node.take_outgoing().is_empty());
//! assert!(!node.is_permitted());
//!
//! now += 1;
//! node.poll(now);
//! let out = node.take_outgoing();
//! let alias = node.alias().unwrap();
//! assert_eq!(out[0].header(), CanHeader::Rid { alias });
//! assert_eq!(out[1].header(), CanHeader::Amd { alias });
//! // …followed by Initialization Complete, Consumer Identified and the queued PCER.
//! assert_eq!(out.len(), 5);
//! assert_eq!(node.next_deadline(), None);
//! ```
//!
//! In a real application, call [`Node::poll`] from a timer (every 10-50 ms
//! is plenty) or sleep until [`Node::next_deadline`], and drain
//! [`Node::take_outgoing`] (or [`Node::pop_outgoing`], one frame at a time)
//! after every `start`, `poll`, `receive` and `send`. The
//! [throttle guide](crate::guide::throttle) shows a complete loop.

use alloc::collections::{BTreeMap, VecDeque};
use alloc::string::String;
use alloc::vec::Vec;

use crate::alias::AliasGenerator;
use crate::can::{cid_frames, CanFrame, CanHeader};
use crate::event::{EventId, EventState};
use crate::message::{protocol, Assembler, Message, Received};
use crate::mti::{error_code, Mti};
use crate::node_id::NodeId;
use crate::traction;

/// How long to wait after the CID frames before claiming the alias.
///
/// The *CAN Frame Transfer Standard* requires at least 200 ms between the
/// last CID frame and the RID frame, giving nodes that already use the
/// alias time to object.
pub const ALIAS_RESERVE_MS: u64 = 200;

/// How many messages [`Node::send`] holds while the node is not yet
/// permitted. Further messages are dropped ([`Node::try_send`] reports it).
pub const MAX_PENDING: usize = 512;

/// Simple Node Information, reported to configuration tools such as JMRI.
///
/// The *Simple Node Information Standard* (SNIP) lets a tool ask any node
/// for six human-readable strings without reading its configuration memory.
/// The first four are fixed by the manufacturer; the last two are set by the
/// user (JMRI shows them as the node's name and description).
/// [`Node`] answers SNIP requests from its `NodeInfo` automatically.
///
/// Wire format of the reply payload:
///
/// ```text
///  0x04                        manufacturer section version
///  manufacturer \0             max 41 bytes incl. NUL
///  model \0                    max 41
///  hardware_version \0         max 21
///  software_version \0         max 21
///  0x02                        user section version
///  user_name \0                max 63
///  user_description \0         max 64
/// ```
///
/// ```
/// use rolcb::NodeInfo;
///
/// let info = NodeInfo {
///     manufacturer: "Uncommon Models".into(),
///     model: "Dispatch Panel".into(),
///     software_version: "0.1.0".into(),
///     user_name: "Yard tower".into(),
///     ..Default::default()
/// };
/// let bytes = info.snip_payload();
/// assert_eq!(bytes[0], 4);
/// assert_eq!(NodeInfo::from_snip(&bytes), info);
/// ```
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct NodeInfo {
    /// Manufacturer name (truncated to 40 bytes on the wire).
    pub manufacturer: String,
    /// Model name (truncated to 40 bytes).
    pub model: String,
    /// Hardware version (truncated to 20 bytes).
    pub hardware_version: String,
    /// Software version (truncated to 20 bytes).
    pub software_version: String,
    /// User-assigned node name (truncated to 62 bytes).
    pub user_name: String,
    /// User-assigned description (truncated to 63 bytes).
    pub user_description: String,
}

impl NodeInfo {
    /// Encode as a SNIP reply payload.
    #[must_use]
    ///
    /// Over-long strings are truncated at a UTF-8 character boundary to fit
    /// the field limits shown in the [type docs](NodeInfo).
    pub fn snip_payload(&self) -> Vec<u8> {
        fn put(v: &mut Vec<u8>, s: &str, max: usize) {
            let mut n = s.len().min(max - 1);
            while !s.is_char_boundary(n) {
                n -= 1;
            }
            v.extend_from_slice(&s.as_bytes()[..n]);
            v.push(0);
        }
        let mut v = Vec::with_capacity(64);
        v.push(4);
        put(&mut v, &self.manufacturer, 41);
        put(&mut v, &self.model, 41);
        put(&mut v, &self.hardware_version, 21);
        put(&mut v, &self.software_version, 21);
        v.push(2);
        put(&mut v, &self.user_name, 63);
        put(&mut v, &self.user_description, 64);
        v
    }

    /// Decode a SNIP reply payload (lenient).
    ///
    /// The two version bytes are skipped without checking, missing fields
    /// become empty strings and invalid UTF-8 is replaced with U+FFFD, so
    /// this never fails.
    #[must_use]
    pub fn from_snip(p: &[u8]) -> NodeInfo {
        let mut fields = Vec::new();
        let mut i = 0;
        let take = |skip_version: bool, i: &mut usize| {
            if skip_version {
                *i += 1;
            }
            let start = (*i).min(p.len());
            let end = p[start..]
                .iter()
                .position(|&b| b == 0)
                .map(|n| start + n)
                .unwrap_or(p.len());
            *i = end + 1;
            String::from_utf8_lossy(&p[start..end]).into_owned()
        };
        for k in 0..6 {
            fields.push(take(k == 0 || k == 4, &mut i));
        }
        NodeInfo {
            manufacturer: fields[0].clone(),
            model: fields[1].clone(),
            hardware_version: fields[2].clone(),
            software_version: fields[3].clone(),
            user_name: fields[4].clone(),
            user_description: fields[5].clone(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Inhibited,
    Reserving { alias: u16, until: u64 },
    Permitted { alias: u16 },
}

/// A virtual OpenLCB node.
///
/// `Node` is a pure state machine: it owns no socket or timer. See the
/// [module documentation](self) for the lifecycle, the replies it sends on
/// its own, and a complete login example. A node is cheap; an application
/// such as a throttle typically runs exactly one.
///
/// Event registration ([`add_producer`](Node::add_producer),
/// [`add_consumer`](Node::add_consumer)) only affects identification
/// replies. `Node` does not filter PCERs: every received PCER is returned
/// from [`receive`](Node::receive) and the application decides what to do.
#[derive(Debug)]
pub struct Node {
    node_id: NodeId,
    info: NodeInfo,
    protocols: u32,
    alias_gen: AliasGenerator,
    state: State,
    initialized: bool,
    duplicate_node_id: bool,
    outbox: VecDeque<CanFrame>,
    pending: VecDeque<Message>,
    assembler: Assembler,
    aliases: BTreeMap<u16, NodeId>,
    produced: BTreeMap<EventId, EventState>,
    consumed: BTreeMap<EventId, EventState>,
}

impl Node {
    /// Create an inhibited (offline) node. Call [`Node::start`] to log in.
    ///
    /// The node initially advertises
    /// [`EVENT_EXCHANGE`](protocol::EVENT_EXCHANGE) | [`SNIP`](protocol::SNIP);
    /// change that with [`Node::set_protocols`].
    #[must_use]
    pub fn new(node_id: NodeId, info: NodeInfo) -> Self {
        Node {
            node_id,
            info,
            protocols: protocol::EVENT_EXCHANGE | protocol::SNIP,
            alias_gen: AliasGenerator::new(node_id),
            state: State::Inhibited,
            initialized: false,
            duplicate_node_id: false,
            outbox: VecDeque::new(),
            pending: VecDeque::new(),
            assembler: Assembler::new(),
            aliases: BTreeMap::new(),
            produced: BTreeMap::new(),
            consumed: BTreeMap::new(),
        }
    }

    /// This node's 48-bit Node ID.
    #[must_use]
    pub fn node_id(&self) -> NodeId {
        self.node_id
    }

    /// Our alias, once it has been successfully reserved.
    ///
    /// `None` while inhibited or reserving. The alias can change at runtime
    /// if a collision forces a new reservation, so do not cache it for long.
    #[must_use]
    pub fn alias(&self) -> Option<u16> {
        match self.state {
            State::Permitted { alias } => Some(alias),
            _ => None,
        }
    }

    /// True once the alias is reserved and the node may send messages
    /// (the CAN "Permitted" state).
    #[must_use]
    pub fn is_permitted(&self) -> bool {
        matches!(self.state, State::Permitted { .. })
    }

    /// True if another node on the bus announced our Node ID (in an AMD,
    /// Initialization Complete or Verified Node ID). The user must configure
    /// a different Node ID.
    ///
    /// The first detection also sends a PCER for
    /// [`EventId::DUPLICATE_NODE_ID`] (held until login if necessary). The
    /// flag stays set until the node is recreated.
    #[must_use]
    pub fn duplicate_node_id_detected(&self) -> bool {
        self.duplicate_node_id
    }

    /// Set the Protocol Support Inquiry reply flags (see [`protocol`]).
    ///
    /// Setting [`protocol::TRACTION_CONTROL`] also stops the node from
    /// auto-rejecting Traction Control Commands, so a train implementation
    /// can handle them itself.
    pub fn set_protocols(&mut self, flags: u32) {
        self.protocols = flags;
    }

    /// The Protocol Support Inquiry reply flags (see [`Node::set_protocols`]).
    #[must_use]
    pub fn protocols(&self) -> u32 {
        self.protocols
    }

    /// The Simple Node Information this node reports.
    #[must_use]
    pub fn info(&self) -> &NodeInfo {
        &self.info
    }

    /// Replace the Simple Node Information. Takes effect with the next SNIP
    /// request; nothing is sent by itself.
    pub fn set_info(&mut self, info: NodeInfo) {
        self.info = info;
    }

    /// Begin (or restart) alias reservation.
    ///
    /// Picks the next candidate alias, queues the four CID frames and arms the
    /// 200 ms timer. Call [`Node::poll`] at or after
    /// [`Node::next_deadline`] to finish the login.
    ///
    /// CID frames of an earlier reservation that are still in the outbox are
    /// dropped, so a restart never puts two sets on the wire.
    ///
    /// **Calling `start` while permitted** gives up the current alias: an
    /// AMR frame for it is queued first, then the node reserves a new alias
    /// as above. It does not send a second Initialization Complete (use
    /// [`Node::reset`] first for a full re-login).
    pub fn start(&mut self, now_ms: u64) {
        self.outbox
            .retain(|f| !matches!(f.header(), CanHeader::Cid { .. }));
        if let State::Permitted { alias } = self.state {
            self.outbox.push_back(CanFrame::of(
                CanHeader::Amr { alias }.encode(),
                &self.node_id.to_bytes(),
            ));
        }
        let alias = self.alias_gen.next_alias();
        self.outbox.extend(cid_frames(self.node_id, alias));
        self.state = State::Reserving {
            alias,
            until: now_ms + ALIAS_RESERVE_MS,
        };
    }

    /// Go offline (e.g. the link dropped). Call [`Node::start`] to rejoin.
    ///
    /// Clears the outbox, partial messages and the alias table, and
    /// re-arms Initialization Complete for the next login. Registered events
    /// and messages queued with [`Node::send`] are kept.
    pub fn reset(&mut self) {
        self.state = State::Inhibited;
        self.initialized = false;
        self.outbox.clear();
        self.assembler = Assembler::new();
        self.aliases.clear();
    }

    /// The next time [`Node::poll`] has work to do, if any.
    ///
    /// Only `Some` while an alias reservation is in progress.
    #[must_use]
    pub fn next_deadline(&self) -> Option<u64> {
        match self.state {
            State::Reserving { until, .. } => Some(until),
            _ => None,
        }
    }

    /// Advance timers.
    ///
    /// When the reservation timer expires this queues RID and AMD, then (on
    /// the first login only) Initialization Complete and Identified messages
    /// for every registered event, and finally any messages queued by
    /// [`Node::send`] before login. Cheap to call often.
    pub fn poll(&mut self, now_ms: u64) {
        if let State::Reserving { alias, until } = self.state {
            if now_ms >= until {
                self.outbox
                    .push_back(CanFrame::of(CanHeader::Rid { alias }.encode(), &[]));
                self.outbox.push_back(CanFrame::of(
                    CanHeader::Amd { alias }.encode(),
                    &self.node_id.to_bytes(),
                ));
                self.state = State::Permitted { alias };
                if !self.initialized {
                    self.initialized = true;
                    self.emit(Message::InitializationComplete {
                        node_id: self.node_id,
                        simple: false,
                    });
                    self.identify_all_events();
                }
                while let Some(m) = self.pending.pop_front() {
                    self.emit(m);
                }
            }
        }
    }

    /// Queue a message for transmission. Messages sent before login completes
    /// are held and sent as soon as the alias is reserved.
    ///
    /// At most [`MAX_PENDING`] (512) messages are held while offline;
    /// further ones are silently dropped (use [`Node::try_send`] to find
    /// out). Once permitted, the message is immediately encoded into frames;
    /// collect them with [`Node::take_outgoing`].
    ///
    /// Encoding uses the lenient rules of [`Message::to_frames`] (without
    /// its debug assertion); validate untrusted messages with
    /// [`Message::try_to_frames`] first.
    pub fn send(&mut self, msg: Message) {
        let _ = self.try_send(msg);
    }

    /// Like [`Node::send`], but returns `false` if the message was dropped
    /// because [`MAX_PENDING`] messages are already waiting for login.
    #[must_use = "the message was dropped if this returns false"]
    pub fn try_send(&mut self, msg: Message) -> bool {
        if self.is_permitted() {
            self.emit(msg);
            true
        } else if self.pending.len() < MAX_PENDING {
            self.pending.push_back(msg);
            true
        } else {
            false
        }
    }

    /// Drain frames waiting to be transmitted.
    ///
    /// Frames are returned in the order they must be sent. Write each to the
    /// bus (for example with [`gridconnect::format`](crate::gridconnect::format)).
    #[must_use = "the frames must be written to the bus"]
    pub fn take_outgoing(&mut self) -> Vec<CanFrame> {
        self.outbox.drain(..).collect()
    }

    /// Remove and return the next frame to transmit, without allocating.
    ///
    /// Useful with a CAN driver that has a small transmit queue: pop a frame,
    /// try to send it, and keep it for later if the driver is full.
    ///
    /// ```
    /// use rolcb::{Node, NodeId, NodeInfo};
    ///
    /// let mut node = Node::new(NodeId::new(0x0501_0101_2299), NodeInfo::default());
    /// node.start(0);
    /// let mut n = 0;
    /// while let Some(_frame) = node.pop_outgoing() {
    ///     n += 1; // write _frame to the driver
    /// }
    /// assert_eq!(n, 4); // the CID frames
    /// ```
    pub fn pop_outgoing(&mut self) -> Option<CanFrame> {
        self.outbox.pop_front()
    }

    /// Register an event we produce (we send PCERs for it).
    ///
    /// If the node is already permitted and the event is new, a Producer
    /// Identified message is sent right away. Registering again with a
    /// different state just updates the state.
    pub fn add_producer(&mut self, event: EventId, state: EventState) {
        let new = self.produced.insert(event, state).is_none();
        if new && self.is_permitted() {
            self.emit(Message::ProducerIdentified { event, state });
        }
    }

    /// Register an event we consume (we act on PCERs for it).
    ///
    /// Announced with Consumer Identified, like
    /// [`add_producer`](Node::add_producer). Registering a consumer tells
    /// producers and tools that someone listens; it does not filter what
    /// [`receive`](Node::receive) returns.
    pub fn add_consumer(&mut self, event: EventId, state: EventState) {
        let new = self.consumed.insert(event, state).is_none();
        if new && self.is_permitted() {
            self.emit(Message::ConsumerIdentified { event, state });
        }
    }

    /// Update the state reported for a registered produced event. Ignored for
    /// unregistered events. Does not send anything by itself; send a
    /// [`Message::Pcer`] if the event just happened.
    pub fn set_producer_state(&mut self, event: EventId, state: EventState) {
        if let Some(s) = self.produced.get_mut(&event) {
            *s = state;
        }
    }

    /// Update the state reported for a registered consumed event. Ignored for
    /// unregistered events.
    pub fn set_consumer_state(&mut self, event: EventId, state: EventState) {
        if let Some(s) = self.consumed.get_mut(&event) {
            *s = state;
        }
    }

    /// Forget all registered producers and consumers.
    pub fn clear_events(&mut self) {
        self.produced.clear();
        self.consumed.clear();
    }

    /// Look up the alias of a node (from AMD / Init Complete / Verified Node ID).
    ///
    /// Returns `None` if the mapping has not been seen yet or was reset by an
    /// AMR.
    #[must_use]
    pub fn alias_of(&self, node: NodeId) -> Option<u16> {
        self.aliases
            .iter()
            .find(|(_, n)| **n == node)
            .map(|(a, _)| *a)
    }

    /// Look up the node behind an alias, if known.
    #[must_use]
    pub fn node_of(&self, alias: u16) -> Option<NodeId> {
        self.aliases.get(&alias).copied()
    }

    /// Process a frame from the bus. Returns the decoded message, if the
    /// frame completed one. Standard replies are queued automatically.
    ///
    /// Pass every frame read from the bus, including ones addressed to other
    /// nodes. Control frames update alias state and return `None`; the first
    /// frames of multi-frame messages also return `None`. `now_ms` is used
    /// when an alias collision forces a new reservation and to expire
    /// incomplete multi-frame messages (see [`Assembler::accept_at`]).
    ///
    /// **Never feed the node its own transmitted frames** (for example from
    /// a CAN driver or hub that echoes transmissions back). A frame carrying
    /// our own alias is, by definition, an alias collision: the node would
    /// release its alias with AMR and log in again, over and over. Filter
    /// out local echo before calling `receive`.
    ///
    /// ```
    /// use rolcb::node::ALIAS_RESERVE_MS;
    /// use rolcb::{EventId, Message, Node, NodeId, NodeInfo};
    ///
    /// let mut node = Node::new(NodeId::new(0x0501_0101_2299), NodeInfo::default());
    /// node.start(0);
    /// node.poll(ALIAS_RESERVE_MS);
    /// let _ = node.take_outgoing();
    ///
    /// // A PCER from some other node (alias 0x333):
    /// let frame = Message::Pcer(EventId(5)).to_frames(0x333)[0];
    /// let rx = node.receive(&frame, 300).unwrap();
    /// assert_eq!(rx.src_alias, 0x333);
    /// assert_eq!(rx.src_node, None); // we never saw its AMD
    /// assert_eq!(rx.msg, Message::Pcer(EventId(5)));
    /// ```
    pub fn receive(&mut self, frame: &CanFrame, now_ms: u64) -> Option<Received> {
        let header = frame.header();
        let src = header.src_alias();

        // Alias collision handling.
        match self.state {
            State::Reserving { alias, .. } if alias == src => {
                self.start(now_ms);
                return None;
            }
            State::Permitted { alias } if alias == src => {
                if matches!(header, CanHeader::Cid { .. }) {
                    self.outbox
                        .push_back(CanFrame::of(CanHeader::Rid { alias }.encode(), &[]));
                } else {
                    // start() sends AMR for the alias we are giving up.
                    self.start(now_ms);
                }
                return None;
            }
            _ => {}
        }

        match header {
            CanHeader::Amd { .. } => {
                if let Ok(n) = NodeId::from_bytes(frame.data()) {
                    self.learn_alias(src, n);
                }
                return None;
            }
            CanHeader::Amr { .. } => {
                self.aliases.remove(&src);
                self.assembler.forget(src);
                return None;
            }
            CanHeader::Ame { .. } => {
                if let State::Permitted { alias } = self.state {
                    let d = frame.data();
                    if d.is_empty() || NodeId::from_bytes(d).ok() == Some(self.node_id) {
                        self.outbox.push_back(CanFrame::of(
                            CanHeader::Amd { alias }.encode(),
                            &self.node_id.to_bytes(),
                        ));
                    }
                }
                return None;
            }
            CanHeader::Message { .. } | CanHeader::Datagram { .. } => {}
            // CID, RID, other control frames, streams, reserved types.
            _ => return None,
        }

        let raw = self.assembler.accept_at(frame, now_ms)?;
        let msg = Message::from(raw);
        match &msg {
            Message::InitializationComplete { node_id, .. }
            | Message::VerifiedNodeId { node_id, .. } => {
                self.learn_alias(src, *node_id);
            }
            _ => {}
        }
        if let State::Permitted { alias } = self.state {
            self.auto_reply(&msg, src, alias);
        }
        Some(Received {
            src_alias: src,
            src_node: self.aliases.get(&src).copied(),
            msg,
        })
    }

    fn learn_alias(&mut self, alias: u16, node: NodeId) {
        if node == self.node_id {
            if !self.duplicate_node_id {
                self.duplicate_node_id = true;
                self.send(Message::Pcer(EventId::DUPLICATE_NODE_ID));
            }
            // Never map someone else's alias to our own Node ID.
            return;
        }
        // An alias change for a known node replaces the old mapping.
        self.aliases.retain(|_, n| *n != node);
        self.aliases.insert(alias, node);
    }

    fn auto_reply(&mut self, msg: &Message, src: u16, me: u16) {
        let to_me = msg.dest() == Some(me);
        match msg {
            Message::VerifyNodeIdGlobal { node_id } => {
                if node_id.is_none() || *node_id == Some(self.node_id) {
                    self.emit(Message::VerifiedNodeId {
                        node_id: self.node_id,
                        simple: false,
                    });
                }
            }
            Message::VerifyNodeIdAddressed { .. } if to_me => {
                self.emit(Message::VerifiedNodeId {
                    node_id: self.node_id,
                    simple: false,
                });
            }
            Message::ProtocolSupportInquiry { .. } if to_me => {
                self.emit(Message::ProtocolSupportReply {
                    dest: src,
                    flags: self.protocols,
                });
            }
            Message::SnipRequest { .. } if to_me => {
                self.emit(Message::SnipReply {
                    dest: src,
                    data: self.info.snip_payload(),
                });
            }
            Message::IdentifyEventsGlobal => self.identify_all_events(),
            Message::IdentifyEventsAddressed { .. } if to_me => self.identify_all_events(),
            Message::IdentifyConsumer(e) => {
                if let Some(&state) = self.consumed.get(e) {
                    self.emit(Message::ConsumerIdentified { event: *e, state });
                }
            }
            Message::IdentifyProducer(e) => {
                if let Some(&state) = self.produced.get(e) {
                    self.emit(Message::ProducerIdentified { event: *e, state });
                }
            }
            Message::Datagram { .. } if to_me => {
                self.emit(Message::DatagramRejected {
                    dest: src,
                    code: error_code::NOT_IMPLEMENTED_UNKNOWN_MTI,
                });
            }
            Message::Other { mti, .. } if to_me => {
                self.emit(Message::OptionalInteractionRejected {
                    dest: src,
                    code: error_code::NOT_IMPLEMENTED_UNKNOWN_MTI,
                    mti: *mti,
                });
            }
            Message::TractionCommand { payload, .. }
                if to_me
                    && self.protocols & protocol::TRACTION_CONTROL == 0
                    && !traction::Command::is_controller_changing_notify(payload) =>
            {
                self.emit(Message::OptionalInteractionRejected {
                    dest: src,
                    code: error_code::NOT_IMPLEMENTED_UNKNOWN_MTI,
                    mti: Mti::TRACTION_CONTROL_COMMAND,
                });
            }
            _ => {}
        }
    }

    fn identify_all_events(&mut self) {
        let c: Vec<_> = self.consumed.iter().map(|(e, s)| (*e, *s)).collect();
        let p: Vec<_> = self.produced.iter().map(|(e, s)| (*e, *s)).collect();
        for (event, state) in c {
            self.emit(Message::ConsumerIdentified { event, state });
        }
        for (event, state) in p {
            self.emit(Message::ProducerIdentified { event, state });
        }
    }

    fn emit(&mut self, msg: Message) {
        if let State::Permitted { alias } = self.state {
            self.outbox.extend(msg.to_frames_lenient(alias));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gridconnect;
    use alloc::string::ToString;
    use alloc::vec;

    fn gc(frames: &[CanFrame]) -> Vec<String> {
        frames.iter().map(gridconnect::format).collect()
    }

    fn node() -> Node {
        Node::new(
            NodeId::new(0x0501_0101_2200),
            NodeInfo {
                manufacturer: "Uncommon Models".into(),
                model: "Test".into(),
                ..Default::default()
            },
        )
    }

    fn logged_in() -> (Node, u16) {
        let mut n = node();
        n.start(0);
        n.poll(ALIAS_RESERVE_MS);
        let _ = n.take_outgoing();
        let a = n.alias().unwrap();
        (n, a)
    }

    /// Decode a batch of outgoing frames into messages.
    fn msgs(frames: &[CanFrame]) -> Vec<Message> {
        let mut asm = Assembler::new();
        frames
            .iter()
            .filter_map(|f| asm.accept(f))
            .map(Message::from)
            .collect()
    }

    /// Feed `msg` from alias 0x111 and return what the node replied.
    fn reply_to(n: &mut Node, msg: Message) -> Vec<Message> {
        for f in msg.to_frames(0x111) {
            let _ = n.receive(&f, 100);
        }
        msgs(&n.take_outgoing())
    }

    fn oir(dest: u16, mti: Mti) -> Message {
        Message::OptionalInteractionRejected {
            dest,
            code: error_code::NOT_IMPLEMENTED_UNKNOWN_MTI,
            mti,
        }
    }

    #[test]
    fn login_sequence() {
        let mut n = node();
        n.add_consumer(EventId(0x0501_0101_2200_0010), EventState::Unknown);
        n.send(Message::Pcer(EventId(0x0501_0101_2200_0100)));
        n.start(1000);
        let out = n.take_outgoing();
        assert_eq!(out.len(), 4);
        let alias = out[0].src_alias();
        assert!(out
            .iter()
            .all(|f| matches!(f.header(), CanHeader::Cid { .. })));
        assert!(alias != 0);

        n.poll(1199);
        assert!(n.take_outgoing().is_empty());
        assert!(!n.is_permitted());
        assert_eq!(n.next_deadline(), Some(1200));

        n.poll(1200);
        let out = gc(&n.take_outgoing());
        assert_eq!(
            out,
            vec![
                gridconnect::format(&CanFrame::of(CanHeader::Rid { alias }.encode(), &[])),
                gridconnect::format(&CanFrame::of(
                    CanHeader::Amd { alias }.encode(),
                    &[5, 1, 1, 1, 0x22, 0]
                )),
                gridconnect::format(
                    &Message::InitializationComplete {
                        node_id: n.node_id(),
                        simple: false
                    }
                    .to_frames(alias)[0]
                ),
                gridconnect::format(
                    &Message::ConsumerIdentified {
                        event: EventId(0x0501_0101_2200_0010),
                        state: EventState::Unknown
                    }
                    .to_frames(alias)[0]
                ),
                gridconnect::format(
                    &Message::Pcer(EventId(0x0501_0101_2200_0100)).to_frames(alias)[0]
                ),
            ]
        );
        assert_eq!(n.alias(), Some(alias));
    }

    #[test]
    fn collision_during_reservation_picks_new_alias() {
        let mut n = node();
        n.start(0);
        let first = n.take_outgoing()[0].src_alias();
        // Someone else uses our candidate alias.
        let other = Message::Pcer(EventId(1)).to_frames(first)[0];
        assert!(n.receive(&other, 50).is_none());
        let out = n.take_outgoing();
        assert_eq!(out.len(), 4);
        let second = out[0].src_alias();
        assert_ne!(first, second);
        n.poll(200);
        assert!(!n.is_permitted(), "timer restarted at 50ms");
        n.poll(250);
        assert_eq!(n.alias(), Some(second));
    }

    #[test]
    fn defends_alias_against_cid() {
        let (mut n, a) = logged_in();
        let cid = cid_frames(NodeId::new(0x0909_0909_0909), a)[0];
        n.receive(&cid, 1000);
        let out = n.take_outgoing();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].header(), CanHeader::Rid { alias: a });
        assert_eq!(n.alias(), Some(a));
    }

    #[test]
    fn non_cid_collision_releases_alias() {
        let (mut n, a) = logged_in();
        let f = Message::Pcer(EventId(1)).to_frames(a)[0];
        n.receive(&f, 1000);
        let out = n.take_outgoing();
        assert_eq!(out[0].header(), CanHeader::Amr { alias: a });
        assert_eq!(out.len(), 5); // AMR + 4 CID
        assert!(!n.is_permitted());
        n.poll(1200);
        let out = n.take_outgoing();
        // RID + AMD only; no second Initialization Complete.
        assert_eq!(out.len(), 2);
        assert!(n.is_permitted());
    }

    #[test]
    fn answers_verify_and_ame() {
        let (mut n, a) = logged_in();
        n.receive(
            &Message::VerifyNodeIdGlobal { node_id: None }.to_frames(0x111)[0],
            0,
        );
        let out = n.take_outgoing();
        assert_eq!(
            gc(&out),
            vec![alloc::format!(":X19170{a:03X}N050101012200;")]
        );

        n.receive(
            &Message::VerifyNodeIdGlobal {
                node_id: Some(NodeId::new(1)),
            }
            .to_frames(0x111)[0],
            0,
        );
        assert!(n.take_outgoing().is_empty());

        n.receive(
            &CanFrame::of(CanHeader::Ame { alias: 0x111 }.encode(), &[]),
            0,
        );
        assert_eq!(n.take_outgoing()[0].header(), CanHeader::Amd { alias: a });
    }

    #[test]
    fn identify_and_snip() {
        let (mut n, a) = logged_in();
        let e = EventId(0x0501_0101_2200_0100);
        n.add_producer(e, EventState::Invalid);
        assert_eq!(n.take_outgoing().len(), 1);
        n.receive(&Message::IdentifyProducer(e).to_frames(0x111)[0], 0);
        let out = n.take_outgoing();
        assert_eq!(
            out[0].header(),
            CanHeader::Message {
                mti: Mti::PRODUCER_IDENTIFIED_INVALID,
                alias: a
            }
        );
        n.receive(
            &Message::IdentifyProducer(EventId(9)).to_frames(0x111)[0],
            0,
        );
        assert!(n.take_outgoing().is_empty());

        n.receive(&Message::SnipRequest { dest: a }.to_frames(0x111)[0], 0);
        let frames = n.take_outgoing();
        let mut asm = Assembler::new();
        let raw = frames.iter().filter_map(|f| asm.accept(f)).next().unwrap();
        assert_eq!(raw.dest, Some(0x111));
        let info = NodeInfo::from_snip(&raw.payload);
        assert_eq!(info.manufacturer, "Uncommon Models");
        assert_eq!(info.model, "Test");
    }

    #[test]
    fn rejects_unknown_addressed() {
        let (mut n, a) = logged_in();
        let m = Message::Other {
            mti: Mti(0x0DA8),
            dest: Some(a),
            payload: vec![],
        };
        n.receive(&m.to_frames(0x111)[0], 0);
        let out = n.take_outgoing();
        let mut asm = Assembler::new();
        let raw = out.iter().filter_map(|f| asm.accept(f)).next().unwrap();
        assert_eq!(raw.mti, Mti::OPTIONAL_INTERACTION_REJECTED);
        // Not addressed to us: ignored.
        let m = Message::Other {
            mti: Mti(0x0DA8),
            dest: Some(0x222),
            payload: vec![],
        };
        n.receive(&m.to_frames(0x111)[0], 0);
        assert!(n.take_outgoing().is_empty());
    }

    #[test]
    fn alias_map_and_duplicates() {
        let (mut n, _) = logged_in();
        let other = NodeId::new(0x0909_0909_0909);
        n.receive(
            &CanFrame::of(CanHeader::Amd { alias: 0x333 }.encode(), &other.to_bytes()),
            0,
        );
        assert_eq!(n.alias_of(other), Some(0x333));
        let r = n
            .receive(&Message::Pcer(EventId(5)).to_frames(0x333)[0], 0)
            .unwrap();
        assert_eq!(r.src_node, Some(other));
        n.receive(
            &CanFrame::of(CanHeader::Amr { alias: 0x333 }.encode(), &other.to_bytes()),
            0,
        );
        assert_eq!(n.alias_of(other), None);

        assert!(!n.duplicate_node_id_detected());
        let dup = Message::InitializationComplete {
            node_id: n.node_id(),
            simple: false,
        };
        n.receive(&dup.to_frames(0x444)[0], 0);
        assert!(n.duplicate_node_id_detected());
        let _ = n.node_id().to_string();
    }

    #[test]
    fn ame_and_verify_for_other_nodes_are_ignored() {
        let (mut n, a) = logged_in();
        let other = NodeId::new(0x0909_0909_0909);
        // AME naming another node: no AMD.
        n.receive(
            &CanFrame::of(CanHeader::Ame { alias: 0x111 }.encode(), &other.to_bytes()),
            0,
        );
        assert!(n.take_outgoing().is_empty());
        // AME naming us: AMD.
        n.receive(
            &CanFrame::of(
                CanHeader::Ame { alias: 0x111 }.encode(),
                &n.node_id().to_bytes(),
            ),
            0,
        );
        assert_eq!(n.take_outgoing()[0].header(), CanHeader::Amd { alias: a });
        // Verify Node ID Global for another node: no reply.
        assert!(reply_to(
            &mut n,
            Message::VerifyNodeIdGlobal {
                node_id: Some(other)
            }
        )
        .is_empty());
        // Verify Node ID Global for us: reply.
        let me = n.node_id();
        assert_eq!(
            reply_to(&mut n, Message::VerifyNodeIdGlobal { node_id: Some(me) }),
            vec![Message::VerifiedNodeId {
                node_id: me,
                simple: false
            }]
        );
        // Verify Node ID Addressed to another alias: no reply.
        assert!(reply_to(
            &mut n,
            Message::VerifyNodeIdAddressed {
                dest: 0x222,
                node_id: None
            }
        )
        .is_empty());
        // …and to us: reply.
        assert_eq!(
            reply_to(
                &mut n,
                Message::VerifyNodeIdAddressed {
                    dest: a,
                    node_id: None
                }
            )
            .len(),
            1
        );
    }

    #[test]
    fn protocol_support_reply_uses_flags() {
        let (mut n, a) = logged_in();
        assert_eq!(n.protocols(), protocol::EVENT_EXCHANGE | protocol::SNIP);
        assert_eq!(
            reply_to(&mut n, Message::ProtocolSupportInquiry { dest: a }),
            vec![Message::ProtocolSupportReply {
                dest: 0x111,
                flags: protocol::EVENT_EXCHANGE | protocol::SNIP
            }]
        );
        let flags = protocol::DATAGRAM | protocol::TRACTION_CONTROL | protocol::FDI;
        n.set_protocols(flags);
        assert_eq!(n.protocols(), flags);
        assert_eq!(
            reply_to(&mut n, Message::ProtocolSupportInquiry { dest: a }),
            vec![Message::ProtocolSupportReply { dest: 0x111, flags }]
        );
        // Not to us: nothing.
        assert!(reply_to(&mut n, Message::ProtocolSupportInquiry { dest: 0x222 }).is_empty());
    }

    #[test]
    fn traction_rejected_unless_train() {
        let (mut n, a) = logged_in();
        let cmd = Message::TractionCommand {
            dest: a,
            payload: crate::traction::Command::QuerySpeed.encode(),
        };
        assert_eq!(
            reply_to(&mut n, cmd.clone()),
            vec![oir(0x111, Mti::TRACTION_CONTROL_COMMAND)]
        );
        // To another node: nothing.
        let other = Message::TractionCommand {
            dest: 0x222,
            payload: vec![0x10],
        };
        assert!(reply_to(&mut n, other).is_empty());
        // A train handles traction itself.
        n.set_protocols(protocol::TRACTION_CONTROL);
        assert!(reply_to(&mut n, cmd).is_empty());
    }

    #[test]
    fn controller_changing_notify_is_delivered_not_rejected() {
        let (mut n, a) = logged_in();
        let notify = crate::traction::Command::ControllerChangingNotify {
            flags: 0,
            new_controller: NodeId::new(0x0501_0101_2277),
        };
        let m = Message::TractionCommand {
            dest: a,
            payload: notify.encode(),
        };
        let frames = m.to_frames(0x6A1);
        assert_eq!(frames.len(), 2);
        assert!(n.receive(&frames[0], 0).is_none());
        let rx = n.receive(&frames[1], 0).expect("delivered");
        assert_eq!(rx.msg, m);
        assert!(
            n.take_outgoing().is_empty(),
            "no OIR for Controller Changing Notify"
        );
    }

    #[test]
    fn datagram_rejected_with_1043() {
        let (mut n, a) = logged_in();
        for len in [0usize, 8, 9, 72] {
            let d = Message::Datagram {
                dest: a,
                data: vec![0x20; len],
            };
            assert_eq!(
                reply_to(&mut n, d),
                vec![Message::DatagramRejected {
                    dest: 0x111,
                    code: error_code::NOT_IMPLEMENTED_UNKNOWN_MTI
                }],
                "{len}-byte datagram"
            );
        }
        // Datagram to someone else: nothing.
        assert!(reply_to(
            &mut n,
            Message::Datagram {
                dest: 0x222,
                data: vec![1]
            }
        )
        .is_empty());
    }

    #[test]
    fn unknown_addressed_gets_1043() {
        let (mut n, a) = logged_in();
        let m = Message::Other {
            mti: Mti(0x0DA8),
            dest: Some(a),
            payload: vec![1, 2],
        };
        assert_eq!(reply_to(&mut n, m), vec![oir(0x111, Mti(0x0DA8))]);
        // Unknown *global* MTI: nothing.
        let m = Message::Other {
            mti: Mti(0x0DA0),
            dest: None,
            payload: vec![],
        };
        assert!(reply_to(&mut n, m).is_empty());
    }

    #[test]
    fn replies_are_never_rejected() {
        let (mut n, a) = logged_in();
        for m in [
            Message::TractionReply {
                dest: a,
                payload: crate::traction::Reply::Heartbeat { timeout_s: 3 }.encode(),
            },
            Message::OptionalInteractionRejected {
                dest: a,
                code: 0x1043,
                mti: Mti::DATAGRAM,
            },
            Message::TerminateDueToError {
                dest: a,
                code: 0x2000,
                mti: Mti::DATAGRAM,
            },
            Message::DatagramReceivedOk { dest: a, flags: 0 },
            Message::DatagramRejected {
                dest: a,
                code: 0x2020,
            },
            Message::ProtocolSupportReply {
                dest: a,
                flags: protocol::SNIP,
            },
            Message::SnipReply {
                dest: a,
                data: NodeInfo::default().snip_payload(),
            },
            Message::VerifiedNodeId {
                node_id: NodeId::new(0x0909_0909_0909),
                simple: false,
            },
        ] {
            assert!(reply_to(&mut n, m.clone()).is_empty(), "{m:?}");
        }
    }

    #[test]
    fn reset_and_start_reannounces() {
        let mut n = node();
        let e = EventId(0x0501_0101_2200_0001);
        n.add_producer(e, EventState::Valid);
        n.start(0);
        n.poll(ALIAS_RESERVE_MS);
        let first = msgs(&n.take_outgoing());
        assert!(matches!(first[0], Message::InitializationComplete { .. }));

        n.reset();
        assert!(!n.is_permitted());
        assert_eq!(n.alias(), None);
        assert!(n.take_outgoing().is_empty());
        n.start(1000);
        n.poll(1000 + ALIAS_RESERVE_MS);
        let again = msgs(&n.take_outgoing());
        assert_eq!(again, first, "Init Complete and Identified are sent again");
    }

    #[test]
    fn pending_cap_and_try_send() {
        let mut n = node();
        for i in 0..MAX_PENDING {
            assert!(n.try_send(Message::Pcer(EventId(i as u64))));
        }
        assert!(!n.try_send(Message::Pcer(EventId(9999))));
        n.send(Message::Pcer(EventId(9999))); // silently dropped
        n.start(0);
        n.poll(ALIAS_RESERVE_MS);
        let out = msgs(&n.take_outgoing());
        // Init Complete + the held messages, in order.
        assert_eq!(out.len(), 1 + MAX_PENDING);
        assert_eq!(out[1], Message::Pcer(EventId(0)));
        assert_eq!(
            out[MAX_PENDING],
            Message::Pcer(EventId(MAX_PENDING as u64 - 1))
        );
        // Once permitted, try_send always succeeds.
        assert!(n.try_send(Message::Pcer(EventId(9999))));
    }

    #[test]
    fn duplicate_node_id_via_amd_sends_pcer_once() {
        let (mut n, _) = logged_in();
        let amd = CanFrame::of(
            CanHeader::Amd { alias: 0x444 }.encode(),
            &n.node_id().to_bytes(),
        );
        n.receive(&amd, 0);
        assert!(n.duplicate_node_id_detected());
        assert_eq!(
            msgs(&n.take_outgoing()),
            vec![Message::Pcer(EventId::DUPLICATE_NODE_ID)]
        );
        // Our own Node ID is never mapped to someone else's alias.
        assert_eq!(n.alias_of(n.node_id()), None);
        // Second detection: no second PCER.
        n.receive(&amd, 10);
        let dup = Message::VerifiedNodeId {
            node_id: n.node_id(),
            simple: false,
        };
        n.receive(&dup.to_frames(0x445)[0], 20);
        assert!(n.take_outgoing().is_empty());
    }

    #[test]
    fn duplicate_before_login_is_held() {
        let mut n = node();
        n.start(0);
        let _ = n.take_outgoing();
        let amd = CanFrame::of(
            CanHeader::Amd { alias: 0x444 }.encode(),
            &n.node_id().to_bytes(),
        );
        n.receive(&amd, 50);
        assert!(n.duplicate_node_id_detected());
        n.poll(ALIAS_RESERVE_MS);
        let out = msgs(&n.take_outgoing());
        assert!(
            out.contains(&Message::Pcer(EventId::DUPLICATE_NODE_ID)),
            "{out:?}"
        );
    }

    #[test]
    fn collision_restart_drops_queued_cids() {
        let mut n = node();
        n.start(0);
        // Not drained: the first CIDs are still queued when the collision hits.
        let first = n.outbox[0].src_alias();
        let other = Message::Pcer(EventId(1)).to_frames(first)[0];
        assert!(n.receive(&other, 50).is_none());
        let out = n.take_outgoing();
        assert_eq!(out.len(), 4, "only the new CIDs");
        assert!(out.iter().all(|f| f.src_alias() != first));
        // An explicit restart while reserving also replaces them.
        n.start(60);
        n.start(70);
        assert_eq!(n.take_outgoing().len(), 4);
    }

    #[test]
    fn start_while_permitted_sends_amr_first() {
        let (mut n, a) = logged_in();
        n.start(500);
        let out = n.take_outgoing();
        assert_eq!(out.len(), 5);
        assert_eq!(out[0].header(), CanHeader::Amr { alias: a });
        assert_eq!(out[0].data(), n.node_id().to_bytes());
        assert!(out[1..]
            .iter()
            .all(|f| matches!(f.header(), CanHeader::Cid { .. })));
        assert!(!n.is_permitted());
        n.poll(500 + ALIAS_RESERVE_MS);
        let out = n.take_outgoing();
        assert_eq!(out.len(), 2, "RID + AMD, no second Init Complete");
        assert_ne!(n.alias(), Some(a));
    }

    #[test]
    fn rid_and_amd_from_our_alias_while_reserving_restart() {
        let mut n = node();
        n.start(0);
        let first = n.take_outgoing()[0].src_alias();
        let rid = CanFrame::of(CanHeader::Rid { alias: first }.encode(), &[]);
        n.receive(&rid, 10);
        let second = n.take_outgoing()[0].src_alias();
        assert_ne!(first, second);
        // A CID for our candidate from someone else also restarts.
        let cid = cid_frames(NodeId::new(0x0909_0909_0909), second)[0];
        n.receive(&cid, 20);
        assert_ne!(n.take_outgoing()[0].src_alias(), second);
    }

    #[test]
    fn pop_outgoing_info_and_set_info() {
        let mut n = node();
        assert_eq!(n.info().model, "Test");
        n.set_info(NodeInfo {
            model: "Changed".into(),
            ..Default::default()
        });
        assert_eq!(n.info().model, "Changed");
        n.start(0);
        let mut popped = Vec::new();
        while let Some(f) = n.pop_outgoing() {
            popped.push(f);
        }
        assert_eq!(popped.len(), 4);
        assert!(n.pop_outgoing().is_none());
        n.poll(ALIAS_RESERVE_MS);
        let a = n.alias().unwrap();
        let _ = n.take_outgoing();
        let snip = reply_to(&mut n, Message::SnipRequest { dest: a });
        match &snip[..] {
            [Message::SnipReply { data, .. }] => {
                assert_eq!(NodeInfo::from_snip(data).model, "Changed")
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn incomplete_multiframe_expires() {
        let (mut n, a) = logged_in();
        let m = Message::TractionReply {
            dest: a,
            payload: vec![1; 10],
        };
        let f = m.to_frames(0x111);
        assert!(n.receive(&f[0], 1_000).is_none());
        // The last frame arrives too late: the partial was dropped.
        assert!(n
            .receive(&f[1], 1_000 + crate::message::PARTIAL_TIMEOUT_MS)
            .is_none());
        // In time, it completes.
        assert!(n.receive(&f[0], 10_000).is_none());
        assert_eq!(n.receive(&f[1], 10_100).unwrap().msg, m);
    }
}
