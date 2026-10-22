# Protocol reference

Quick-reference tables for the parts of OpenLCB this crate implements.
Values are hexadecimal. The authoritative source is the set of OpenLCB
Standards and Technical Notes published at <https://openlcb.org>.

## Message Type Indicators

From the *Message Network Standard*, *Event Transport Standard*,
*Traction Control Standard*, *Simple Node Information Standard* and
*Datagram Transport Standard*. "A" = addressed (bit 3), "E" = carries an
Event ID (bit 2). Constants are on [`Mti`](crate::Mti); variants on
[`Message`](crate::Message).

| MTI | CAN-MTI | Flags | Name | `Mti` constant | Payload |
|---|---|---|---|---|---|
| `0100` | `100` | | Initialization Complete | `INITIALIZATION_COMPLETE` | Node ID (6) |
| `0101` | `101` | | Initialization Complete (simple) | `INITIALIZATION_COMPLETE_SIMPLE` | Node ID (6) |
| `0488` | `488` | A | Verify Node ID (addressed) | `VERIFY_NODE_ID_ADDRESSED` | optional Node ID |
| `0490` | `490` | | Verify Node ID (global) | `VERIFY_NODE_ID_GLOBAL` | optional Node ID |
| `0170` | `170` | | Verified Node ID | `VERIFIED_NODE_ID` | Node ID (6) |
| `0171` | `171` | | Verified Node ID (simple) | `VERIFIED_NODE_ID_SIMPLE` | Node ID (6) |
| `0068` | `068` | A | Optional Interaction Rejected | `OPTIONAL_INTERACTION_REJECTED` | error code (2), MTI (2) |
| `00A8` | `0A8` | A | Terminate Due To Error | `TERMINATE_DUE_TO_ERROR` | error code (2), MTI (2) |
| `0828` | `828` | A | Protocol Support Inquiry | `PROTOCOL_SUPPORT_INQUIRY` | — |
| `0668` | `668` | A | Protocol Support Reply | `PROTOCOL_SUPPORT_REPLY` | flags (6) |
| `08F4` | `8F4` | E | Identify Consumer | `IDENTIFY_CONSUMER` | Event ID (8) |
| `04A4` | `4A4` | E | Consumer Range Identified | `CONSUMER_RANGE_IDENTIFIED` | Event ID (8) |
| `04C4` | `4C4` | E | Consumer Identified, valid | `CONSUMER_IDENTIFIED_VALID` | Event ID (8) |
| `04C5` | `4C5` | E | Consumer Identified, invalid | `CONSUMER_IDENTIFIED_INVALID` | Event ID (8) |
| `04C7` | `4C7` | E | Consumer Identified, unknown | `CONSUMER_IDENTIFIED_UNKNOWN` | Event ID (8) |
| `0914` | `914` | E | Identify Producer | `IDENTIFY_PRODUCER` | Event ID (8) |
| `0524` | `524` | E | Producer Range Identified | `PRODUCER_RANGE_IDENTIFIED` | Event ID (8) |
| `0544` | `544` | E | Producer Identified, valid | `PRODUCER_IDENTIFIED_VALID` | Event ID (8) |
| `0545` | `545` | E | Producer Identified, invalid | `PRODUCER_IDENTIFIED_INVALID` | Event ID (8) |
| `0547` | `547` | E | Producer Identified, unknown | `PRODUCER_IDENTIFIED_UNKNOWN` | Event ID (8) |
| `0968` | `968` | A | Identify Events (addressed) | `IDENTIFY_EVENTS_ADDRESSED` | — |
| `0970` | `970` | | Identify Events (global) | `IDENTIFY_EVENTS_GLOBAL` | — |
| `0594` | `594` | E | Learn Event | `LEARN_EVENT` | Event ID (8) |
| `05B4` | `5B4` | E | Producer/Consumer Event Report (PCER) | `PCER` | Event ID (8) |
| `05EB` | `5EB` | A | Traction Control Command | `TRACTION_CONTROL_COMMAND` | see [traction commands](#traction-control-commands) |
| `01E9` | `1E9` | A | Traction Control Reply | `TRACTION_CONTROL_REPLY` | see [traction replies](#traction-control-replies) |
| `0DE8` | `DE8` | A | Simple Node Information Request | `SNIP_REQUEST` | — |
| `0A08` | `A08` | A | Simple Node Information Reply | `SNIP_REPLY` | see [SNIP](#simple-node-information) |
| `1C48` | (frame types 2-5) | A | Datagram | `DATAGRAM` | up to 72 bytes |
| `0A28` | `A28` | A | Datagram Received OK | `DATAGRAM_RECEIVED_OK` | optional flags (1) |
| `0A48` | `A48` | A | Datagram Rejected | `DATAGRAM_REJECTED` | error code (2) |

## CAN identifier layout

From the *CAN Frame Transfer Standard*. All frames are CAN 2.0B extended
(29-bit identifier), 125 kbit/s.

```text
  28  27  26 25 24  23 ................ 12  11 ................. 0
 ┌───┬───┬─────────┬─────────────────────────┬───────────────────────┐
 │ 1 │ F │  type   │     variable field      │     source alias      │
 └───┴───┴─────────┴─────────────────────────┴───────────────────────┘
   │   │      │                │
   │   │      │                └ CAN-MTI (type 1), destination alias (types 2-5, 7),
   │   │      │                  Node ID slice (CID) or control code (RID/AMD/AME/AMR)
   │   │      └ message frame type (F=1) or control sequence (F=0)
   │   └ F: 1 = OpenLCB message frame, 0 = CAN control frame
   └ reserved, always 1 (send) / ignored (receive)
```

See [`CanHeader`](crate::CanHeader).

### Control frames (bit 27 = 0)

| Bits 26-24 | Bits 23-12 | Frame | Data | Purpose |
|---|---|---|---|---|
| `7` | Node ID bits 47-36 | CID7 | — | Check ID, 1st of 4 |
| `6` | Node ID bits 35-24 | CID6 | — | Check ID, 2nd |
| `5` | Node ID bits 23-12 | CID5 | — | Check ID, 3rd |
| `4` | Node ID bits 11-0 | CID4 | — | Check ID, 4th |
| `0` | `700` | RID | — | Reserve ID: claim alias after 200 ms / defend against CID |
| `0` | `701` | AMD | Node ID | Alias Map Definition: alias ↔ Node ID; node is now *permitted* |
| `0` | `702` | AME | optional Node ID | Alias Mapping Enquiry: request AMDs |
| `0` | `703` | AMR | Node ID | Alias Map Reset: alias released |

Example identifiers for alias `285` and Node ID `05.01.01.01.22.00`:

```text
 CID7 :X17050285N;   CID6 :X16101285N;   CID5 :X15012285N;   CID4 :X14200285N;
 RID  :X10700285N;   AMD  :X10701285N050101012200;
 Init Complete :X19100285N050101012200;
```

### Message frame types (bit 27 = 1)

| Bits 26-24 | Frame | Variable field | Data |
|---|---|---|---|
| `0` | reserved | | |
| `1` | Global / addressed message | CAN-MTI | addressed: 2-byte flags+dest, then ≤ 6 bytes; global: ≤ 8 bytes |
| `2` | Datagram, only frame | destination alias | ≤ 8 bytes |
| `3` | Datagram, first frame | destination alias | 8 bytes |
| `4` | Datagram, middle frame | destination alias | 8 bytes |
| `5` | Datagram, final frame | destination alias | ≤ 8 bytes |
| `6` | reserved | | |
| `7` | Stream data | destination alias | |

See [`MessageFrameType`](crate::can::MessageFrameType).

### Addressed message framing

The first two data bytes of every frame of an addressed message:

```text
 byte 0: F F F F D D D D     FFFF framing (bits 5-4 used), DDDD = dest alias bits 11-8
 byte 1: D D D D D D D D     dest alias bits 7-0
```

| Byte 0 high nibble | Meaning |
|---|---|
| `0` | only frame |
| `1` | first frame of several |
| `3` | middle frame |
| `2` | last frame |

For example a 9-byte Assign Controller from alias `285` to alias `6A1`:

```text
:X195EB285N16A1200100050101   first: flag 1, dest 6A1, 6 payload bytes
:X195EB285N26A1012299         last:  flag 2, dest 6A1, 3 payload bytes
```

## Protocol Support (PIP) bits

Reply payload bytes 0-2 as used by
[`message::protocol`](crate::message::protocol) (byte 0 = bits 23-16 of the
`u32`).

| Byte | Bit | Mask (`u32`) | Protocol | Constant |
|---|---|---|---|---|
| 0 | 7 | `80_0000` | Simple Protocol subset | `SIMPLE` |
| 0 | 6 | `40_0000` | Datagram | `DATAGRAM` |
| 0 | 5 | `20_0000` | Stream | `STREAM` |
| 0 | 4 | `10_0000` | Memory Configuration | `MEMORY_CONFIGURATION` |
| 0 | 3 | `08_0000` | Reservation | `RESERVATION` |
| 0 | 2 | `04_0000` | Event Exchange | `EVENT_EXCHANGE` |
| 0 | 1 | `02_0000` | Identification | `IDENTIFICATION` |
| 0 | 0 | `01_0000` | Teaching / Learning | `TEACHING` |
| 1 | 7 | `00_8000` | Remote Button | `REMOTE_BUTTON` |
| 1 | 6 | `00_4000` | Abbreviated Default CDI | `ACDI` |
| 1 | 5 | `00_2000` | Display | `DISPLAY` |
| 1 | 4 | `00_1000` | Simple Node Information | `SNIP` |
| 1 | 3 | `00_0800` | Configuration Description Information | `CDI` |
| 1 | 2 | `00_0400` | Traction Control | `TRACTION_CONTROL` |
| 1 | 1 | `00_0200` | Function Description Information | `FDI` |
| 2 | 7 | `00_0080` | Simple Train Node Information | `SIMPLE_TRAIN_NODE_INFO` |
| 2 | 6 | `00_0040` | Function Configuration | `FUNCTION_CONFIGURATION` |

A default [`Node`](crate::Node) advertises `EVENT_EXCHANGE | SNIP` (`04_1000`).

## Traction Control commands

Payload of MTI `05EB`; see [`traction::Command`](crate::traction::Command)
and [`traction::cmd`](crate::traction::cmd). Multi-byte fields are
big-endian.

| Bytes | Command | Variant |
|---|---|---|
| `00 SS SS` | Set Speed (half-float m/s, sign = reverse) | `SetSpeed` |
| `01 AA AA AA VV VV` | Set Function (24-bit address, 16-bit value) | `SetFunction` |
| `02` | Emergency Stop | `EmergencyStop` |
| `10` | Query Speed | `QuerySpeed` |
| `11 AA AA AA` | Query Function | `QueryFunction` |
| `20 01 FF NN NN NN NN NN NN` | Assign Controller (flags, Node ID) | `AssignController` |
| `20 02 FF NN NN NN NN NN NN` | Release Controller | `ReleaseController` |
| `20 03` | Query Controller | `QueryController` |
| `30 …` | Consist (listener) configuration | `Unknown` |
| `40 01` | Management: Reserve | `Reserve` |
| `40 02` | Management: Release | `Release` |
| `40 03` | Management: No-op / heartbeat answer | `Noop` |

Bit 7 of byte 0 (`80`, `LISTENER_FLAG`) marks a command forwarded to a
consist member; decoding ignores it.

## Traction Control replies

Payload of MTI `01E9`; see [`traction::Reply`](crate::traction::Reply).

| Bytes | Reply | Variant |
|---|---|---|
| `10 SS SS ST CC CC AA AA` | Query Speed: set speed, status (bit 0 = e-stop), commanded, actual | `QuerySpeed` |
| `11 AA AA AA VV VV` | Query Function | `QueryFunction` |
| `20 01 RR` | Assign Controller result (0 = OK) | `AssignController` |
| `20 03 FF NN NN NN NN NN NN` | Query Controller (flags, controller Node ID) | `QueryController` |
| `20 04 RR` | Controller Changed notify | `ControllerChanged` |
| `40 01 RR` | Reserve result | `Reserve` |
| `40 03 TT` | Heartbeat request, timeout `TT` seconds | `Heartbeat` |

## Half-float speeds

| Speed | Bits | Notes |
|---|---|---|
| stop, forward | `0000` | `Speed::STOP_FORWARD` |
| stop, reverse | `8000` | `Speed::STOP_REVERSE` (-0.0) |
| 1.0 m/s forward | `3C00` | |
| 1.0 m/s reverse | `BC00` | sign bit set |
| 0.5 m/s | `3800` | |
| 126 mph ≈ 56.33 m/s | `530A` | DCC "full speed" by convention |
| not available | `FFFF` | `Speed::NAN`, in Query Speed replies |

## Train search events

From the *Train Search Standard*; see
[`traction::search`](crate::traction::search).

```text
 09.00.99.FF . D5 D4 . D3 D2 . D1 D0 . FLAGS
             └──── 6 BCD nibbles ───┘  F = unused (leading)
```

| Flag | Mask | Meaning |
|---|---|---|
| `ALLOCATE` | `80` | Create the train if none matches |
| `EXACT` | `40` | Only exact matches |
| `ADDRESS_ONLY` | `20` | Match address only, not name |
| `DCC` | `08` | DCC protocol |
| `DCC_LONG_ADDRESS` | `04` | Long (extended) DCC address |
| `SPEED_STEPS_MASK` | `03` | 0 default, 1 = 14, 2 = 28, 3 = 128 steps |

| Search | Event ID |
|---|---|
| DCC 3, short | `09.00.99.FF.FF.FF.F3.68` |
| DCC 3, short, allocate | `09.00.99.FF.FF.FF.F3.E8` |
| DCC 1234 (long) | `09.00.99.FF.FF.12.34.6C` |

## Simple Node Information

Payload of the SNIP reply; see [`NodeInfo`](crate::NodeInfo).

| Field | Max bytes incl. NUL |
|---|---|
| version (`04`) | 1 |
| manufacturer | 41 |
| model | 41 |
| hardware version | 21 |
| software version | 21 |
| user version (`02`) | 1 |
| user name | 63 |
| user description | 64 |

## Error codes

Used by Optional Interaction Rejected, Terminate Due To Error and Datagram
Rejected; see [`mti::error_code`](crate::mti::error_code).

| Code | Class | Meaning | Constant |
|---|---|---|---|
| `1000` | permanent | permanent error | `PERMANENT` |
| `1040` | permanent | not implemented | `NOT_IMPLEMENTED` |
| `1041` | permanent | not implemented: unknown sub-command | `NOT_IMPLEMENTED_SUBCOMMAND` |
| `1042` | permanent | not implemented: unknown command | `NOT_IMPLEMENTED_COMMAND` |
| `2000` | temporary | temporary error, retry later | `TEMPORARY` |
| `2020` | temporary | buffer unavailable | `BUFFER_UNAVAILABLE` |
| `2040` | temporary | out of order | `OUT_OF_ORDER` |

## Well-known events

| Event ID | Meaning | Constant |
|---|---|---|
| `01.00.00.00.00.00.FF.FF` | Emergency Off (de-energize) | `EventId::EMERGENCY_OFF` |
| `01.00.00.00.00.00.FF.FE` | Clear Emergency Off | `EventId::CLEAR_EMERGENCY_OFF` |
| `01.00.00.00.00.00.FF.FD` | Emergency Stop of all operations | `EventId::EMERGENCY_STOP` |
| `01.00.00.00.00.00.FF.FC` | Clear Emergency Stop | `EventId::CLEAR_EMERGENCY_STOP` |
| `01.01.00.00.00.00.03.03` | Is Train (produced by train nodes) | `EventId::IS_TRAIN` |

## Timing and limits

| Item | Value |
|---|---|
| CID → RID wait | 200 ms ([`ALIAS_RESERVE_MS`](crate::node::ALIAS_RESERVE_MS)) |
| Messages held before login | 512 |
| Partial multi-frame messages tracked | 32 |
| Maximum reassembled payload | 256 bytes |
| Datagram maximum (standard) | 72 bytes |
| GridConnect decoder token limit | 32 characters |
| Hub per-port queue | 4096 frames |
| Hub write batch | 64 frames |
| Hub reconnect delay | 2 s |
