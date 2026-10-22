//! Traction Control protocol payloads (MTI 0x05EB command / 0x01E9 reply)
//! and the Train Search protocol.
//!
//! In OpenLCB every locomotive (or consist) is a **train node**. For DCC
//! locos the train node is usually virtual and lives in the command
//! station, which turns traction commands into DCC packets. A throttle
//! drives a train by sending addressed *Traction Control Command* messages
//! to it and reading *Traction Control Reply* messages back, as defined by
//! the *Traction Control Standard*.
//!
//! This module only encodes and decodes the payload bytes. Wrap them in
//! [`Message::TractionCommand`](crate::Message::TractionCommand) /
//! [`Message::TractionReply`](crate::Message::TractionReply) and send them
//! through a [`Node`](crate::Node). The [throttle guide](crate::guide::throttle)
//! walks through a whole session.
//!
//! # A throttle session
//!
//! 1. **Find the train.** Send Identify Producer for a
//!    [search event](search) built from the DCC address. The train node
//!    (allocated by the command station if needed) answers with Producer
//!    Identified for the same event; its source alias is the train.
//! 2. **Assign controller.** Send [`Command::AssignController`] with our
//!    Node ID. The train answers [`Reply::AssignController`] with result 0
//!    if accepted. Only the assigned controller should send speed commands.
//! 3. **Drive.** [`Command::SetSpeed`], [`Command::SetFunction`],
//!    [`Command::EmergencyStop`]. Query state with [`Command::QuerySpeed`] /
//!    [`Command::QueryFunction`].
//! 4. **Stay alive.** A train may send [`Reply::Heartbeat`] asking for any
//!    command within `timeout_s` seconds; answer with [`Command::Noop`] or
//!    the train will stop itself.
//! 5. **Release** with [`Command::ReleaseController`].
//!
//! # Controller handover
//!
//! When a second throttle sends Assign Controller to a train that already
//! has a controller, the train asks the current controller first. It
//! sends the *command* [`Command::ControllerChangingNotify`] (`20 04 flags
//! NodeID`, carrying the new controller's Node ID) **to the current
//! controller**, which answers with the *reply* [`Reply::ControllerChanged`]
//! (`20 04 result`): result 0 accepts the handover, nonzero refuses it.
//! So a throttle receives a Traction Control *Command* in this one case,
//! even though it is not a train; [`Node`](crate::Node) delivers it
//! instead of rejecting it. On accepting, the throttle should stop sending
//! commands to that train and tell the user it lost control. The
//! [throttle guide](crate::guide::throttle#losing-control-to-another-throttle)
//! shows the exchange.
//!
//! # Command bytes
//!
//! | Byte 0 | Byte 1 | Meaning | Rest |
//! |---|---|---|---|
//! | `0x00` | | Set Speed | half-float speed (2 bytes) |
//! | `0x01` | | Set Function | 24-bit address, 16-bit value |
//! | `0x02` | | Emergency Stop | |
//! | `0x10` | | Query Speed | |
//! | `0x11` | | Query Function | 24-bit address |
//! | `0x20` | `0x01` | Assign Controller | flags, Node ID (6) |
//! | `0x20` | `0x02` | Release Controller | flags, Node ID (6) |
//! | `0x20` | `0x03` | Query Controller | |
//! | `0x20` | `0x04` | Controller Changing Notify (train → current controller) | flags, new controller Node ID (6) |
//! | `0x30` | | Consist (listener) config | |
//! | `0x40` | `0x01` | Management: Reserve | |
//! | `0x40` | `0x02` | Management: Release | |
//! | `0x40` | `0x03` | Management: Noop / heartbeat | |
//!
//! Replies reuse the same command bytes; see [`Reply`]. Constants are in
//! [`cmd`]. Bit 7 of byte 0 ([`cmd::LISTENER_FLAG`]) marks a command
//! forwarded to a consist member and is ignored by [`Command::decode`].
//!
//! ```
//! use rolcb::traction::{Command, Reply, Speed};
//!
//! // 1.0 m/s forward is half-float 0x3C00.
//! assert_eq!(Command::SetSpeed(Speed::from_mps(1.0, false)).encode(), [0x00, 0x3C, 0x00]);
//! // Same speed in reverse: the sign bit is set.
//! assert_eq!(Command::SetSpeed(Speed::from_mps(1.0, true)).encode(), [0x00, 0xBC, 0x00]);
//! // F3 on.
//! assert_eq!(Command::SetFunction { addr: 3, value: 1 }.encode(), [0x01, 0, 0, 3, 0, 1]);
//!
//! assert_eq!(Reply::decode(&[0x40, 0x03, 10]), Reply::Heartbeat { timeout_s: 10 });
//! ```

use alloc::vec::Vec;

use crate::event::EventId;
use crate::float16;
use crate::node_id::NodeId;

const MPS_PER_MPH: f32 = 0.44704;

/// A traction speed: IEEE half-float in scale m/s, sign bit = reverse.
///
/// The inner `u16` is the raw half-float as sent on the wire (see
/// [`crate::float16`]). The *Traction Control Standard* expresses speed in
/// scale meters per second, with the sign bit as the direction: set means
/// reverse. Direction is therefore kept even at zero speed, which is why
/// there are two stop values. The magnitude accessors ([`Speed::mps`],
/// [`Speed::mph`]) always return a non-negative number; use
/// [`Speed::is_reverse`] for the direction.
///
/// ```
/// use rolcb::traction::Speed;
///
/// let s = Speed::from_mph(30.0, true);
/// assert!(s.is_reverse());
/// assert!((s.mph() - 30.0).abs() < 0.05);
/// assert!(s.mps() > 0.0);
///
/// assert_eq!(Speed::from_mps(0.0, true), Speed::STOP_REVERSE);
/// assert_eq!(Speed::default(), Speed::STOP_FORWARD);
/// assert!(Speed::NAN.is_nan());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Speed(pub u16);

impl Speed {
    /// "Not available", used in Query Speed replies.
    pub const NAN: Speed = Speed(0xFFFF);
    /// Zero speed, facing forward (`+0.0`, `0x0000`).
    pub const STOP_FORWARD: Speed = Speed(0x0000);
    /// Zero speed, facing reverse (`-0.0`, `0x8000`).
    pub const STOP_REVERSE: Speed = Speed(0x8000);

    /// Build from a magnitude in scale m/s and a direction. The sign of `mps`
    /// is ignored; only `reverse` sets the direction.
    #[must_use]
    pub fn from_mps(mps: f32, reverse: bool) -> Speed {
        let mut v = float16::from_f32(mps.abs());
        if reverse {
            v |= 0x8000;
        }
        Speed(v)
    }

    /// Build from a magnitude in scale miles per hour (1 mph = 0.44704 m/s)
    /// and a direction.
    #[must_use]
    pub fn from_mph(mph: f32, reverse: bool) -> Speed {
        Self::from_mps(mph * MPS_PER_MPH, reverse)
    }

    /// True if the direction (sign) bit is set.
    #[must_use]
    pub fn is_reverse(self) -> bool {
        self.0 & 0x8000 != 0
    }

    /// True for any half-float NaN, such as [`Speed::NAN`].
    #[must_use]
    pub fn is_nan(self) -> bool {
        float16::to_f32(self.0).is_nan()
    }

    /// Magnitude in m/s (never negative; see [`Speed::is_reverse`]).
    ///
    /// For a NaN speed such as [`Speed::NAN`] ("not available") this
    /// returns `f32::NAN`, so check [`Speed::is_nan`] before doing
    /// arithmetic or comparisons with the result.
    ///
    /// ```
    /// use rolcb::traction::Speed;
    /// assert!(Speed::NAN.mps().is_nan());
    /// assert_eq!(Speed::from_mps(2.0, true).mps(), 2.0);
    /// ```
    #[must_use]
    pub fn mps(self) -> f32 {
        float16::to_f32(self.0 & 0x7FFF)
    }

    /// Magnitude in mph. DCC command stations conventionally map mph 1:1
    /// onto 128-step speed steps (126 = full). NaN for a NaN speed, like
    /// [`Speed::mps`].
    #[must_use]
    pub fn mph(self) -> f32 {
        self.mps() / MPS_PER_MPH
    }
}

/// Traction Control command and sub-command byte values.
///
/// Byte 0 of every command and reply is one of the top-level values; for
/// [`CONTROLLER_CONFIG`](crate::traction::cmd::CONTROLLER_CONFIG) and
/// [`MANAGEMENT`](crate::traction::cmd::MANAGEMENT), byte 1 is a sub-command.
pub mod cmd {
    /// Set Speed (followed by a 2-byte half-float speed).
    pub const SET_SPEED: u8 = 0x00;
    /// Set Function (24-bit function address, 16-bit value).
    pub const SET_FUNCTION: u8 = 0x01;
    /// Emergency Stop: stop now and stay stopped until a new speed is set.
    pub const EMERGENCY_STOP: u8 = 0x02;
    /// Query Speed; the reply carries set, commanded and actual speeds.
    pub const QUERY_SPEED: u8 = 0x10;
    /// Query Function (24-bit function address).
    pub const QUERY_FUNCTION: u8 = 0x11;
    /// Controller Configuration group (assign/release/query controller).
    pub const CONTROLLER_CONFIG: u8 = 0x20;
    /// Consist (listener) Configuration group. Not decoded by this crate.
    pub const CONSIST_CONFIG: u8 = 0x30;
    /// Traction Management group (reserve/release/noop).
    pub const MANAGEMENT: u8 = 0x40;
    /// Set on commands forwarded to consist listeners.
    pub const LISTENER_FLAG: u8 = 0x80;

    /// Controller Configuration sub-command: Assign Controller.
    pub const CONTROLLER_ASSIGN: u8 = 0x01;
    /// Controller Configuration sub-command: Release Controller.
    pub const CONTROLLER_RELEASE: u8 = 0x02;
    /// Controller Configuration sub-command: Query Controller.
    pub const CONTROLLER_QUERY: u8 = 0x03;
    /// Controller Configuration sub-command: Controller Changing Notify. As
    /// a command the train sends it to its current controller when another
    /// node asks to take over; as a reply the controller accepts (0) or
    /// refuses the handover.
    pub const CONTROLLER_CHANGED: u8 = 0x04;

    /// Management sub-command: Reserve (lock the train for a multi-step operation).
    pub const MGMT_RESERVE: u8 = 0x01;
    /// Management sub-command: Release a reservation.
    pub const MGMT_RELEASE: u8 = 0x02;
    /// Management sub-command: No-op / heartbeat. As a command it is a
    /// keep-alive; as a reply it is a heartbeat request with a timeout.
    pub const MGMT_NOOP: u8 = 0x03;
}

/// A Traction Control Command (throttle → train).
///
/// Encode with [`Command::encode`] and send as the payload of
/// [`Message::TractionCommand`](crate::Message::TractionCommand). A train
/// implementation decodes incoming payloads with [`Command::decode`].
///
/// ```
/// use rolcb::traction::{Command, Speed};
/// use rolcb::NodeId;
///
/// let me = NodeId::new(0x0501_0101_2299);
/// let bytes = Command::AssignController { flags: 0, controller: me }.encode();
/// assert_eq!(bytes, [0x20, 0x01, 0x00, 0x05, 0x01, 0x01, 0x01, 0x22, 0x99]);
/// assert_eq!(Command::decode(&bytes), Command::AssignController { flags: 0, controller: me });
///
/// assert_eq!(Command::Noop.encode(), [0x40, 0x03]);
/// assert_eq!(Command::decode(&[0x7F]), Command::Unknown(vec![0x7F]));
/// ```
///
/// The enum is `#[non_exhaustive]`: match it with a `_` arm.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Command {
    /// Set speed and direction (`0x00`). Also clears an emergency stop.
    SetSpeed(Speed),
    /// Set a function (`0x01`). For DCC trains, addresses 0..=28 are F0..F28.
    SetFunction {
        /// 24-bit function address.
        addr: u32,
        /// New value; for on/off functions 0 = off, 1 = on (see [`function_value`]).
        value: u16,
    },
    /// Emergency stop (`0x02`): stop immediately, ignoring momentum.
    EmergencyStop,
    /// Ask for the current speed (`0x10`); answered by [`Reply::QuerySpeed`].
    QuerySpeed,
    /// Ask for a function value (`0x11`); answered by [`Reply::QueryFunction`].
    QueryFunction {
        /// 24-bit function address.
        addr: u32,
    },
    /// Become this train's controller (`0x20 0x01`). Answered by
    /// [`Reply::AssignController`]; if another throttle was in control, the
    /// train first asks it with [`Command::ControllerChangingNotify`].
    AssignController {
        /// Flags byte (0 for a normal throttle).
        flags: u8,
        /// The controlling throttle's Node ID.
        controller: NodeId,
    },
    /// Stop being this train's controller (`0x20 0x02`).
    ReleaseController {
        /// Flags byte (normally 0).
        flags: u8,
        /// The Node ID that was assigned.
        controller: NodeId,
    },
    /// Ask which node controls the train (`0x20 0x03`).
    QueryController,
    /// Controller Changing Notify (`0x20 0x04 flags NodeID`), **train →
    /// current controller**: another node (`new_controller`) asked to take
    /// over. The controller answers with [`Reply::ControllerChanged`]
    /// (result 0 = accept). This is the one command a throttle receives;
    /// [`Node`](crate::Node) never auto-rejects it.
    ControllerChangingNotify {
        /// Flags byte (normally 0).
        flags: u8,
        /// The node that wants to become the controller.
        new_controller: NodeId,
    },
    /// Reserve the train for a multi-step operation (`0x40 0x01`).
    Reserve,
    /// Release a reservation (`0x40 0x02`).
    Release,
    /// Keep-alive, also the answer to a heartbeat request.
    ///
    /// Encoded as `0x40 0x03`.
    Noop,
    /// Anything not recognised (or too short to decode), raw bytes kept.
    Unknown(Vec<u8>),
}

fn addr24(addr: u32) -> [u8; 3] {
    let b = addr.to_be_bytes();
    [b[1], b[2], b[3]]
}

fn read24(p: &[u8], i: usize) -> Option<u32> {
    match p.get(i..i + 3) {
        Some(&[a, b, c]) => Some(u32::from_be_bytes([0, a, b, c])),
        _ => None,
    }
}

fn read16(p: &[u8], i: usize) -> Option<u16> {
    match p.get(i..i + 2) {
        Some(&[a, b]) => Some(u16::from_be_bytes([a, b])),
        _ => None,
    }
}

impl Command {
    /// Serialize to the command payload bytes.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        use cmd::*;
        let mut v = Vec::with_capacity(9);
        match self {
            Command::SetSpeed(s) => {
                v.push(SET_SPEED);
                v.extend_from_slice(&s.0.to_be_bytes());
            }
            Command::SetFunction { addr, value } => {
                v.push(SET_FUNCTION);
                v.extend_from_slice(&addr24(*addr));
                v.extend_from_slice(&value.to_be_bytes());
            }
            Command::EmergencyStop => v.push(EMERGENCY_STOP),
            Command::QuerySpeed => v.push(QUERY_SPEED),
            Command::QueryFunction { addr } => {
                v.push(QUERY_FUNCTION);
                v.extend_from_slice(&addr24(*addr));
            }
            Command::AssignController { flags, controller } => {
                v.extend_from_slice(&[CONTROLLER_CONFIG, CONTROLLER_ASSIGN, *flags]);
                v.extend_from_slice(&controller.to_bytes());
            }
            Command::ReleaseController { flags, controller } => {
                v.extend_from_slice(&[CONTROLLER_CONFIG, CONTROLLER_RELEASE, *flags]);
                v.extend_from_slice(&controller.to_bytes());
            }
            Command::QueryController => v.extend_from_slice(&[CONTROLLER_CONFIG, CONTROLLER_QUERY]),
            Command::ControllerChangingNotify {
                flags,
                new_controller,
            } => {
                v.extend_from_slice(&[CONTROLLER_CONFIG, CONTROLLER_CHANGED, *flags]);
                v.extend_from_slice(&new_controller.to_bytes());
            }
            Command::Reserve => v.extend_from_slice(&[MANAGEMENT, MGMT_RESERVE]),
            Command::Release => v.extend_from_slice(&[MANAGEMENT, MGMT_RELEASE]),
            Command::Noop => v.extend_from_slice(&[MANAGEMENT, MGMT_NOOP]),
            Command::Unknown(p) => v.extend_from_slice(p),
        }
        v
    }

    /// Parse command payload bytes. Never fails; unknown or truncated
    /// commands become [`Command::Unknown`]. The consist
    /// [`LISTENER_FLAG`](cmd::LISTENER_FLAG) bit in byte 0 is ignored.
    #[must_use]
    pub fn decode(p: &[u8]) -> Command {
        use cmd::*;
        let unknown = || Command::Unknown(p.to_vec());
        let Some(&first) = p.first() else {
            return Command::Unknown(Vec::new());
        };
        let sub = p.get(1).copied().unwrap_or(0);
        // flags byte + Node ID, for the controller sub-commands.
        let flags_node = || match p.get(2..) {
            Some([flags, rest @ ..]) => NodeId::from_bytes(rest).ok().map(|n| (*flags, n)),
            _ => None,
        };
        match first & !LISTENER_FLAG {
            SET_SPEED => read16(p, 1).map_or_else(unknown, |s| Command::SetSpeed(Speed(s))),
            SET_FUNCTION => match (read24(p, 1), read16(p, 4)) {
                (Some(addr), Some(value)) => Command::SetFunction { addr, value },
                _ => unknown(),
            },
            EMERGENCY_STOP => Command::EmergencyStop,
            QUERY_SPEED => Command::QuerySpeed,
            QUERY_FUNCTION => {
                read24(p, 1).map_or_else(unknown, |addr| Command::QueryFunction { addr })
            }
            CONTROLLER_CONFIG => match (sub, flags_node()) {
                (CONTROLLER_ASSIGN, Some((flags, controller))) => {
                    Command::AssignController { flags, controller }
                }
                (CONTROLLER_RELEASE, Some((flags, controller))) => {
                    Command::ReleaseController { flags, controller }
                }
                (CONTROLLER_QUERY, _) => Command::QueryController,
                (CONTROLLER_CHANGED, Some((flags, new_controller))) => {
                    Command::ControllerChangingNotify {
                        flags,
                        new_controller,
                    }
                }
                _ => unknown(),
            },
            MANAGEMENT => match sub {
                MGMT_RESERVE => Command::Reserve,
                MGMT_RELEASE => Command::Release,
                MGMT_NOOP => Command::Noop,
                _ => unknown(),
            },
            _ => unknown(),
        }
    }

    /// True if `payload` is a Controller Changing Notify command
    /// (`20 04 …`), the one Traction Control Command a throttle receives.
    /// Cheaper than a full [`Command::decode`].
    ///
    /// ```
    /// use rolcb::traction::Command;
    /// assert!(Command::is_controller_changing_notify(&[0x20, 0x04, 0, 5, 1, 1, 1, 0x22, 0x77]));
    /// assert!(!Command::is_controller_changing_notify(&[0x20, 0x01]));
    /// ```
    #[must_use]
    pub fn is_controller_changing_notify(payload: &[u8]) -> bool {
        payload.starts_with(&[cmd::CONTROLLER_CONFIG, cmd::CONTROLLER_CHANGED])
    }
}

/// A Traction Control Reply (train → throttle).
///
/// Decode the payload of a received
/// [`Message::TractionReply`](crate::Message::TractionReply) with
/// [`Reply::decode`]; a train implementation builds replies with
/// [`Reply::encode`].
///
/// ```
/// use rolcb::traction::{Reply, Speed};
///
/// let r = Reply::QuerySpeed {
///     set_speed: Speed::from_mph(10.0, true),
///     estop: false,
///     commanded: Speed::NAN,
///     actual: Speed::NAN,
/// };
/// assert_eq!(Reply::decode(&r.encode()), r);
/// assert_eq!(Reply::decode(&[0x20, 0x01, 0x00]), Reply::AssignController { result: 0 });
/// ```
///
/// The enum is `#[non_exhaustive]`: match it with a `_` arm.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Reply {
    /// Answer to [`Command::QuerySpeed`] (`0x10`).
    QuerySpeed {
        /// The last speed set by a controller.
        set_speed: Speed,
        /// True if the train is in emergency stop (status byte bit 0).
        estop: bool,
        /// The speed currently being commanded (may lag `set_speed` with
        /// momentum); [`Speed::NAN`] if not available or not sent.
        commanded: Speed,
        /// Measured actual speed; [`Speed::NAN`] if not available or not sent.
        actual: Speed,
    },
    /// Answer to [`Command::QueryFunction`] (`0x11`).
    QueryFunction {
        /// 24-bit function address.
        addr: u32,
        /// Current value.
        value: u16,
    },
    /// Result 0 = accepted.
    ///
    /// Answer to [`Command::AssignController`] (`0x20 0x01`). A nonzero
    /// result means the assignment was refused (for example, the current
    /// controller did not agree to hand over).
    AssignController {
        /// 0 on success; nonzero error flags otherwise.
        result: u8,
    },
    /// Answer to [`Command::QueryController`] (`0x20 0x03`).
    QueryController {
        /// Flags byte.
        flags: u8,
        /// The current controller; all-zero if none.
        controller: NodeId,
    },
    /// Answer to [`Command::ControllerChangingNotify`] (`0x20 0x04 result`),
    /// sent **by the current controller back to the train**.
    ///
    /// Despite the name, a throttle does not receive this; it *sends* it
    /// after the train asked whether another node may take over. Result 0
    /// accepts the handover (the throttle should then consider the train
    /// lost); nonzero refuses it, and the train refuses the new controller's
    /// Assign Controller.
    ControllerChanged {
        /// 0 = handover accepted; nonzero = refused.
        result: u8,
    },
    /// Answer to [`Command::Reserve`] (`0x40 0x01`).
    Reserve {
        /// 0 on success.
        result: u8,
    },
    /// The train wants a [`Command::Noop`] within `timeout_s` seconds.
    ///
    /// Encoded as `0x40 0x03 timeout`. If the controller stays silent past
    /// the timeout, the train stops itself as a safety measure.
    Heartbeat {
        /// Seconds the controller has to answer.
        timeout_s: u8,
    },
    /// Anything not recognised, raw bytes kept.
    Unknown(Vec<u8>),
}

impl Reply {
    /// Serialize to the reply payload bytes.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        use cmd::*;
        let mut v = Vec::with_capacity(9);
        match self {
            Reply::QuerySpeed {
                set_speed,
                estop,
                commanded,
                actual,
            } => {
                v.push(QUERY_SPEED);
                v.extend_from_slice(&set_speed.0.to_be_bytes());
                v.push(*estop as u8);
                v.extend_from_slice(&commanded.0.to_be_bytes());
                v.extend_from_slice(&actual.0.to_be_bytes());
            }
            Reply::QueryFunction { addr, value } => {
                v.push(QUERY_FUNCTION);
                v.extend_from_slice(&addr24(*addr));
                v.extend_from_slice(&value.to_be_bytes());
            }
            Reply::AssignController { result } => {
                v.extend_from_slice(&[CONTROLLER_CONFIG, CONTROLLER_ASSIGN, *result])
            }
            Reply::QueryController { flags, controller } => {
                v.extend_from_slice(&[CONTROLLER_CONFIG, CONTROLLER_QUERY, *flags]);
                v.extend_from_slice(&controller.to_bytes());
            }
            Reply::ControllerChanged { result } => {
                v.extend_from_slice(&[CONTROLLER_CONFIG, CONTROLLER_CHANGED, *result])
            }
            Reply::Reserve { result } => v.extend_from_slice(&[MANAGEMENT, MGMT_RESERVE, *result]),
            Reply::Heartbeat { timeout_s } => {
                v.extend_from_slice(&[MANAGEMENT, MGMT_NOOP, *timeout_s])
            }
            Reply::Unknown(p) => v.extend_from_slice(p),
        }
        v
    }

    /// Parse reply payload bytes. Never fails; unknown or truncated replies
    /// become [`Reply::Unknown`]. Missing optional fields of a Query Speed
    /// reply decode as [`Speed::NAN`]; a missing result byte decodes as 0.
    #[must_use]
    pub fn decode(p: &[u8]) -> Reply {
        use cmd::*;
        let unknown = || Reply::Unknown(p.to_vec());
        let Some(&first) = p.first() else {
            return Reply::Unknown(Vec::new());
        };
        let sub = p.get(1).copied().unwrap_or(0);
        let byte = |i: usize| p.get(i).copied().unwrap_or(0);
        match first {
            QUERY_SPEED => match read16(p, 1) {
                Some(set) => Reply::QuerySpeed {
                    set_speed: Speed(set),
                    estop: byte(3) & 1 != 0,
                    commanded: read16(p, 4).map_or(Speed::NAN, Speed),
                    actual: read16(p, 6).map_or(Speed::NAN, Speed),
                },
                None => unknown(),
            },
            QUERY_FUNCTION => match (read24(p, 1), read16(p, 4)) {
                (Some(addr), Some(value)) => Reply::QueryFunction { addr, value },
                _ => unknown(),
            },
            CONTROLLER_CONFIG => match sub {
                CONTROLLER_ASSIGN => Reply::AssignController { result: byte(2) },
                CONTROLLER_QUERY => Reply::QueryController {
                    flags: byte(2),
                    controller: NodeId::from_bytes(p.get(3..).unwrap_or(&[])).unwrap_or_default(),
                },
                CONTROLLER_CHANGED => Reply::ControllerChanged { result: byte(2) },
                _ => unknown(),
            },
            MANAGEMENT => match sub {
                MGMT_RESERVE => Reply::Reserve { result: byte(2) },
                MGMT_NOOP => Reply::Heartbeat { timeout_s: byte(2) },
                _ => unknown(),
            },
            _ => unknown(),
        }
    }
}

/// Train Search protocol: find (or allocate) a train node by DCC address
/// by emitting Identify Producer for a specially formed event ID. Matching
/// train nodes answer with Producer Identified for the same event.
///
/// The *Train Search Standard* reserves the event range `09.00.99.FF.xx.xx.xx.xx`.
/// The low 32 bits encode the query:
///
/// ```text
///  63            32 31                         8 7        0
/// +----------------+----------------------------+----------+
/// |  0x090099FF    | 6 nibbles: query digits,   |  flags   |
/// |  (PREFIX)      | right-aligned, unused = F  |          |
/// +----------------+----------------------------+----------+
///
///  e.g. DCC 1234:  09.00.99.FF . FF.12.34 . 6C
///
///  flags: 0x80 ALLOCATE      create the train if none matches
///         0x40 EXACT         exact match only
///         0x20 ADDRESS_ONLY  match the address, not the name
///         0x08 DCC           DCC protocol
///         0x04 DCC_LONG_ADDRESS
///         0x03 speed steps   0 default, 1 = 14, 2 = 28, 3 = 128
/// ```
///
/// A throttle sends [`Message::IdentifyProducer`](crate::Message::IdentifyProducer)
/// with the search event. Every train node that matches (the command
/// station allocates a new virtual train node first if `ALLOCATE` is set)
/// answers with Producer Identified for the same event ID; the source alias
/// of that reply is the train to control.
///
/// ```
/// use rolcb::traction::search;
///
/// let e = search::dcc_event(3, false, 0);
/// assert_eq!(e.0, 0x0900_99FF_FFFF_F368); // digits "3", EXACT|ADDRESS_ONLY|DCC
/// let e = search::dcc_event(1234, false, search::ALLOCATE);
/// assert_eq!(e.0, 0x0900_99FF_FF12_34EC); // >127 forces the long-address flag
/// assert!(search::is_search_event(e));
/// assert_eq!(search::decode(e), Some((1234, 0xEC)));
/// ```
pub mod search {
    use super::*;

    /// Upper 32 bits of every train search event.
    pub const PREFIX: u64 = 0x0900_99FF;

    /// Create the train if no match exists (command station allocates it).
    pub const ALLOCATE: u8 = 0x80;
    /// Only exact matches.
    pub const EXACT: u8 = 0x40;
    /// Match only against the address, not the train name.
    pub const ADDRESS_ONLY: u8 = 0x20;
    /// DCC protocol.
    pub const DCC: u8 = 0x08;
    /// Force a DCC long (extended) address.
    pub const DCC_LONG_ADDRESS: u8 = 0x04;
    /// Speed step bits (with `DCC`): 0 = default, 1 = 14, 2 = 28, 3 = 128.
    pub const SPEED_STEPS_MASK: u8 = 0x03;

    /// Build the search event for a DCC address. Addresses above 127 are
    /// always long; `long` forces a long address for low numbers.
    ///
    /// Sets [`EXACT`] | [`ADDRESS_ONLY`] | [`DCC`], plus
    /// [`DCC_LONG_ADDRESS`] when needed, plus `extra_flags` (typically
    /// [`ALLOCATE`] and/or a speed-step value).
    #[must_use]
    pub fn dcc_event(address: u16, long: bool, extra_flags: u8) -> EventId {
        let mut flags = EXACT | ADDRESS_ONLY | DCC | extra_flags;
        if long || address > 127 {
            flags |= DCC_LONG_ADDRESS;
        }
        query_event(address as u32, flags)
    }

    /// Build a search event from a numeric query (up to 6 decimal digits).
    ///
    /// Each decimal digit occupies one nibble, right-aligned; unused leading
    /// nibbles are `0xF`. Numbers above 999 999 are clamped. `flags` is used
    /// as-is.
    ///
    /// ```
    /// use rolcb::traction::search;
    ///
    /// let e = search::query_event(42, 0);
    /// assert_eq!(e.0, 0x0900_99FF_FFFF_4200);
    /// ```
    #[must_use]
    pub fn query_event(number: u32, flags: u8) -> EventId {
        let mut nibbles: u32 = 0xFF_FFFF;
        let mut n = number.min(999_999);
        let mut shift = 0;
        loop {
            nibbles &= !(0xF << shift);
            nibbles |= (n % 10) << shift;
            n /= 10;
            shift += 4;
            if n == 0 || shift >= 24 {
                break;
            }
        }
        EventId((PREFIX << 32) | ((nibbles as u64) << 8) | flags as u64)
    }

    /// True if `e` lies in the train search event range ([`PREFIX`]).
    #[must_use]
    pub fn is_search_event(e: EventId) -> bool {
        e.0 >> 32 == PREFIX
    }

    /// Decode a search event into (number, flags). Non-digit nibbles are skipped.
    ///
    /// Returns `None` if `e` is not a search event.
    #[must_use]
    pub fn decode(e: EventId) -> Option<(u32, u8)> {
        if !is_search_event(e) {
            return None;
        }
        let nibbles = ((e.0 >> 8) & 0xFF_FFFF) as u32;
        let mut n = 0u32;
        for i in (0..6).rev() {
            let d = (nibbles >> (i * 4)) & 0xF;
            if d <= 9 {
                n = n * 10 + d;
            }
        }
        Some((n, e.0 as u8))
    }
}

/// Function number helpers: F0..F28 map directly to function addresses 0..28.
///
/// Returns the [`Command::SetFunction`] value for an on/off function:
/// 1 for on, 0 for off.
///
/// ```
/// use rolcb::traction::{function_value, Command};
///
/// let headlight_on = Command::SetFunction { addr: 0, value: function_value(true) };
/// assert_eq!(headlight_on.encode(), [0x01, 0, 0, 0, 0, 1]);
/// ```
#[must_use]
pub fn function_value(on: bool) -> u16 {
    on as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn set_speed_bytes() {
        // 1.0 m/s forward = 0x3C00, reverse = 0xBC00.
        assert_eq!(
            Command::SetSpeed(Speed::from_mps(1.0, false)).encode(),
            vec![0x00, 0x3C, 0x00]
        );
        assert_eq!(
            Command::SetSpeed(Speed::from_mps(1.0, true)).encode(),
            vec![0x00, 0xBC, 0x00]
        );
        assert_eq!(
            Command::SetSpeed(Speed::STOP_REVERSE).encode(),
            vec![0x00, 0x80, 0x00]
        );
        let s = Speed::from_mph(126.0, true);
        assert!(s.is_reverse());
        assert!((s.mph() - 126.0).abs() < 0.1);
    }

    #[test]
    fn command_round_trips() {
        let me = NodeId::new(0x0501_0101_2299);
        for c in [
            Command::SetSpeed(Speed::from_mph(40.0, false)),
            Command::SetFunction { addr: 28, value: 1 },
            Command::EmergencyStop,
            Command::QuerySpeed,
            Command::QueryFunction { addr: 5 },
            Command::AssignController {
                flags: 0,
                controller: me,
            },
            Command::ReleaseController {
                flags: 0,
                controller: me,
            },
            Command::QueryController,
            Command::ControllerChangingNotify {
                flags: 0,
                new_controller: me,
            },
            Command::Reserve,
            Command::Release,
            Command::Noop,
        ] {
            assert_eq!(Command::decode(&c.encode()), c);
        }
        assert_eq!(
            Command::AssignController {
                flags: 0,
                controller: me
            }
            .encode(),
            vec![0x20, 0x01, 0x00, 0x05, 0x01, 0x01, 0x01, 0x22, 0x99]
        );
        assert_eq!(
            Command::SetFunction { addr: 3, value: 1 }.encode(),
            vec![0x01, 0, 0, 3, 0, 1]
        );
    }

    #[test]
    fn reply_round_trips() {
        for r in [
            Reply::QuerySpeed {
                set_speed: Speed::from_mph(10.0, true),
                estop: false,
                commanded: Speed::NAN,
                actual: Speed::NAN,
            },
            Reply::QueryFunction { addr: 0, value: 1 },
            Reply::AssignController { result: 0 },
            Reply::QueryController {
                flags: 0,
                controller: NodeId::new(0x0501_0101_2201),
            },
            Reply::ControllerChanged { result: 0 },
            Reply::Reserve { result: 0 },
            Reply::Heartbeat { timeout_s: 10 },
        ] {
            assert_eq!(Reply::decode(&r.encode()), r);
        }
    }

    #[test]
    fn search_events() {
        let e = search::dcc_event(3, false, 0);
        assert_eq!(e.0, 0x0900_99FF_FFFF_F368);
        let e = search::dcc_event(1234, false, 0);
        assert_eq!(e.0, 0x0900_99FF_FF12_346C);
        assert_eq!(search::decode(e), Some((1234, 0x6C)));
        let e = search::dcc_event(3, true, search::ALLOCATE);
        assert_eq!(e.0 & 0xFF, 0xEC);
        assert!(search::is_search_event(e));
        assert!(!search::is_search_event(EventId::IS_TRAIN));
    }

    #[test]
    fn controller_changing_notify_round_trip() {
        let new = NodeId::new(0x0501_0101_2277);
        let c = Command::ControllerChangingNotify {
            flags: 0,
            new_controller: new,
        };
        let bytes = c.encode();
        assert_eq!(
            bytes,
            vec![0x20, 0x04, 0x00, 0x05, 0x01, 0x01, 0x01, 0x22, 0x77]
        );
        assert!(Command::is_controller_changing_notify(&bytes));
        assert_eq!(Command::decode(&bytes), c);
        // The controller's answer is a reply with the same sub-command.
        let r = Reply::ControllerChanged { result: 0 };
        assert_eq!(r.encode(), vec![0x20, 0x04, 0x00]);
        assert_eq!(Reply::decode(&r.encode()), r);
        assert_eq!(
            Reply::decode(&[0x20, 0x04, 0x01]),
            Reply::ControllerChanged { result: 1 }
        );
        // Through the message layer: 9 bytes, two frames.
        let m = crate::Message::TractionCommand {
            dest: 0x285,
            payload: bytes,
        };
        let mut asm = crate::message::Assembler::new();
        let raw = m
            .to_frames(0x6A1)
            .iter()
            .find_map(|f| asm.accept(f))
            .unwrap();
        assert_eq!(crate::Message::from(raw), m);
    }

    #[test]
    fn truncated_payloads_are_unknown() {
        for p in [
            &[0x00][..],
            &[0x00, 0x3C],
            &[0x01, 0, 0, 3, 0],
            &[0x11, 0, 0],
            &[0x20, 0x01, 0x00, 5, 1, 1, 1, 0x22],
            &[0x20, 0x02],
            &[0x20, 0x04, 0x00],
            &[0x20, 0x09],
            &[0x40, 0x07],
            &[0x30, 0x01],
        ] {
            assert_eq!(Command::decode(p), Command::Unknown(p.to_vec()), "{p:02X?}");
            // Unknown commands re-encode to the same bytes.
            assert_eq!(Command::decode(p).encode(), p);
        }
        assert_eq!(Command::decode(&[]), Command::Unknown(vec![]));
        for p in [
            &[0x10, 0x3C][..],
            &[0x11, 0, 0, 1, 0],
            &[0x20, 0x09],
            &[0x77],
        ] {
            assert_eq!(Reply::decode(p), Reply::Unknown(p.to_vec()), "{p:02X?}");
        }
        assert_eq!(Reply::decode(&[]), Reply::Unknown(vec![]));
        // Optional Query Speed fields default to NaN; result bytes to 0.
        assert_eq!(
            Reply::decode(&[0x10, 0x3C, 0x00]),
            Reply::QuerySpeed {
                set_speed: Speed(0x3C00),
                estop: false,
                commanded: Speed::NAN,
                actual: Speed::NAN
            }
        );
        assert_eq!(
            Reply::decode(&[0x20, 0x01]),
            Reply::AssignController { result: 0 }
        );
        assert_eq!(
            Reply::decode(&[0x20, 0x03, 0x00, 1, 2]),
            Reply::QueryController {
                flags: 0,
                controller: NodeId::default()
            }
        );
    }

    #[test]
    fn listener_flag_is_ignored() {
        assert_eq!(
            Command::decode(&[0x80, 0x3C, 0x00]),
            Command::SetSpeed(Speed(0x3C00))
        );
        assert_eq!(
            Command::decode(&[0x81, 0, 0, 2, 0, 1]),
            Command::SetFunction { addr: 2, value: 1 }
        );
        assert_eq!(Command::decode(&[0x82]), Command::EmergencyStop);
        assert_eq!(Command::decode(&[0xC0, 0x03]), Command::Noop);
    }

    #[test]
    fn query_event_clamps_and_decodes() {
        assert_eq!(search::query_event(0, 0).0, 0x0900_99FF_FFFF_F000);
        assert_eq!(search::query_event(999_999, 0).0, 0x0900_99FF_9999_9900);
        assert_eq!(
            search::query_event(1_000_000, 0x12),
            search::query_event(999_999, 0x12)
        );
        assert_eq!(
            search::query_event(u32::MAX, 0),
            search::query_event(999_999, 0)
        );
        assert_eq!(
            search::decode(search::query_event(4_294, 0x08)),
            Some((4_294, 0x08))
        );
        assert_eq!(search::decode(EventId::IS_TRAIN), None);
        assert_eq!(
            search::dcc_event(9_999, false, 0).0 & 0xFF,
            (search::EXACT | search::ADDRESS_ONLY | search::DCC | search::DCC_LONG_ADDRESS) as u64
        );
    }

    #[test]
    fn speed_nan_and_hash() {
        assert!(Speed::NAN.is_nan());
        assert!(Speed::NAN.mps().is_nan());
        assert!(Speed::NAN.mph().is_nan());
        assert!(!Speed::STOP_REVERSE.is_nan());
        assert_eq!(Speed::from_mps(-3.0, false), Speed::from_mps(3.0, false));
        let mut set = alloc::collections::BTreeSet::new();
        set.insert(alloc::format!("{:?}", Command::Noop));
        assert_eq!(set.len(), 1);
        fn hashable<T: core::hash::Hash>(_: &T) {}
        hashable(&Command::Noop);
        hashable(&Reply::Heartbeat { timeout_s: 1 });
    }
}
