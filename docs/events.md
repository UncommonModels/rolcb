# Consuming and producing events

Events are how almost everything on an OpenLCB layout talks: occupancy
detectors, turnout motors, signal heads, fascia buttons and panel lamps.
This chapter explains the *Event Transport Standard* model and builds a
small dispatch panel.

## The model

An event is just a 64-bit [`EventId`](crate::EventId). It has no payload
and no destination. A node that *produces* an event sends a
**PCER** (Producer/Consumer Event Report, [`Message::Pcer`](crate::Message::Pcer))
when the thing happens; every node that *consumes* that ID reacts. The
binding between a button and a turnout is made by configuring both with
the same Event ID, not by addressing.

```text
  detector 05.01.01.01.22.00                 panel                  signal driver
  ──────────────────────────                 ─────                  ─────────────
  block 1 occupied ──PCER 05.01.01.01.22.00.00.10──► lamp red         ► aspect red
  block 1 clear    ──PCER 05.01.01.01.22.00.00.11──► lamp off         ► aspect green
                       panel button ──PCER 05.01.01.01.22.99.00.01──► turnout 5 thrown
```

Most physical states are modelled as an **event pair**: one ID for
"occupied", one for "clear"; one for "thrown", one for "closed".

### Event IDs

By convention an Event ID starts with the Node ID of the node that
*created* it (6 bytes), followed by a 16-bit number of that node's
choosing (*Event Identifiers Standard*). That makes IDs globally unique
without coordination. IDs starting with `01.` are well-known events
defined by the standards, such as
[`EventId::EMERGENCY_STOP`](crate::EventId::EMERGENCY_STOP) and
[`EventId::IS_TRAIN`](crate::EventId::IS_TRAIN). The `09.00.99.FF` range
is used by [train search](crate::traction::search).

### Identification and state

PCERs are fire-and-forget. A node that starts up later (a panel opened in a
browser) has missed them. The identification messages solve that:

| You send | Nodes answer | Meaning |
|---|---|---|
| [`IdentifyProducer(e)`](crate::Message::IdentifyProducer) | [`ProducerIdentified { e, state }`](crate::Message::ProducerIdentified) | "I produce `e`, and it is currently *state*" |
| [`IdentifyConsumer(e)`](crate::Message::IdentifyConsumer) | [`ConsumerIdentified { e, state }`](crate::Message::ConsumerIdentified) | "I consume `e`" |
| [`IdentifyEventsGlobal`](crate::Message::IdentifyEventsGlobal) | both, for every event of every node | full inventory (heavy on a big layout) |
| [`IdentifyEventsAddressed`](crate::Message::IdentifyEventsAddressed) | both, for every event of one node | one node's inventory |

The [`EventState`](crate::EventState) is the key to recovering state:

* **Valid**: the condition this event reports is true right now (the
  block *is* occupied).
* **Invalid**: it is false (so typically the paired event is the valid
  one).
* **Unknown**: the producer cannot say, or the event is momentary.

So a panel learns current occupancy by sending Identify Producer for each
"occupied" event and looking at the state in the replies.

### What `Node` does for you

[`Node`](crate::Node) keeps a registry of produced and consumed events
([`add_producer`](crate::Node::add_producer),
[`add_consumer`](crate::Node::add_consumer)) and answers identification
requests from it, including the burst of Identified messages at login. It
does **not** filter or interpret PCERs: every received message comes back
from [`receive`](crate::Node::receive), and your code decides what an
event means. Keep the registry's states current with
[`set_producer_state`](crate::Node::set_producer_state) so that answers are
truthful.

## Example: a dispatch panel

The panel below shows occupancy for two blocks and has a button that
throws turnout 5. A simulated detector node plays the other side.

```rust
use std::collections::BTreeMap;

use rolcb::node::ALIAS_RESERVE_MS;
use rolcb::{EventId, EventState, Message, Node, NodeId, NodeInfo, Received};

fn deliver(from: &mut Node, to: &mut Node, now: u64) -> Vec<Received> {
    from.take_outgoing().iter().filter_map(|f| to.receive(f, now)).collect()
}

/// Which block an event refers to, and whether it means "occupied".
#[derive(Clone, Copy)]
struct BlockEvent {
    block: &'static str,
    occupied: bool,
}

struct Panel {
    node: Node,
    blocks: BTreeMap<EventId, BlockEvent>,
    occupancy: BTreeMap<&'static str, Option<bool>>, // None = unknown
    throw_turnout_5: EventId,
}

impl Panel {
    fn new() -> Panel {
        let mut node = Node::new(
            NodeId::new(0x0501_0101_2299),
            NodeInfo { model: "Dispatch panel".into(), ..Default::default() },
        );
        let detector = 0x0501_0101_2200_0000u64;
        let mut blocks = BTreeMap::new();
        for (i, name) in ["Block 1", "Block 2"].into_iter().enumerate() {
            let occ = EventId(detector | (0x10 + 2 * i as u64));
            let clr = EventId(detector | (0x11 + 2 * i as u64));
            blocks.insert(occ, BlockEvent { block: name, occupied: true });
            blocks.insert(clr, BlockEvent { block: name, occupied: false });
            node.add_consumer(occ, EventState::Unknown);
            node.add_consumer(clr, EventState::Unknown);
        }
        // Our own event: "throw turnout 5". Allocated from our Node ID.
        let throw_turnout_5 = EventId(0x0501_0101_2299_0005);
        node.add_producer(throw_turnout_5, EventState::Unknown);

        let occupancy = blocks.values().map(|b| (b.block, None)).collect();
        Panel { node, blocks, occupancy, throw_turnout_5 }
    }

    /// After login, ask the producers for the current state of each block.
    fn query_state(&mut self) {
        let occupied: Vec<EventId> =
            self.blocks.iter().filter(|(_, b)| b.occupied).map(|(e, _)| *e).collect();
        for e in occupied {
            self.node.send(Message::IdentifyProducer(e));
        }
    }

    fn handle(&mut self, rx: &Received) {
        match rx.msg {
            // Live updates.
            Message::Pcer(e) => {
                if let Some(b) = self.blocks.get(&e) {
                    self.occupancy.insert(b.block, Some(b.occupied));
                }
            }
            // Answers to query_state(): Valid means "this is the current state".
            Message::ProducerIdentified { event, state } => {
                if let Some(b) = self.blocks.get(&event) {
                    let now = match state {
                        EventState::Valid => Some(b.occupied),
                        EventState::Invalid => Some(!b.occupied),
                        EventState::Unknown => None,
                    };
                    self.occupancy.insert(b.block, now);
                }
            }
            _ => {}
        }
    }

    fn press_turnout_button(&mut self) {
        self.node.send(Message::Pcer(self.throw_turnout_5));
    }
}

// A detector that knows Block 1 is occupied and Block 2 is clear.
let mut detector = Node::new(NodeId::new(0x0501_0101_2200), NodeInfo::default());
for (event, state) in [
    (0x0501_0101_2200_0010, EventState::Valid),   // block 1 occupied: true
    (0x0501_0101_2200_0011, EventState::Invalid), // block 1 clear: false
    (0x0501_0101_2200_0012, EventState::Invalid), // block 2 occupied: false
    (0x0501_0101_2200_0013, EventState::Valid),   // block 2 clear: true
] {
    detector.add_producer(EventId(event), state);
}

let mut panel = Panel::new();
panel.node.start(0);
detector.start(0);
deliver(&mut panel.node, &mut detector, 0);
deliver(&mut detector, &mut panel.node, 0);
panel.node.poll(ALIAS_RESERVE_MS);
detector.poll(ALIAS_RESERVE_MS);

// At login both nodes announce their events. The detector's Producer
// Identified messages already tell the panel the current state…
deliver(&mut panel.node, &mut detector, 200);
for rx in deliver(&mut detector, &mut panel.node, 200) {
    panel.handle(&rx);
}
assert_eq!(panel.occupancy["Block 1"], Some(true));
assert_eq!(panel.occupancy["Block 2"], Some(false));

// …and a panel that starts later asks explicitly.
panel.occupancy.values_mut().for_each(|v| *v = None);
panel.query_state();
for rx in deliver(&mut panel.node, &mut detector, 300) {
    let _ = rx; // the detector's Node answers Identify Producer by itself
}
for rx in deliver(&mut detector, &mut panel.node, 300) {
    panel.handle(&rx);
}
assert_eq!(panel.occupancy["Block 1"], Some(true));
assert_eq!(panel.occupancy["Block 2"], Some(false));

// A train leaves block 1: the detector updates its state and reports.
let clear1 = EventId(0x0501_0101_2200_0011);
let occ1 = EventId(0x0501_0101_2200_0010);
detector.set_producer_state(occ1, EventState::Invalid);
detector.set_producer_state(clear1, EventState::Valid);
detector.send(Message::Pcer(clear1));
for rx in deliver(&mut detector, &mut panel.node, 400) {
    panel.handle(&rx);
}
assert_eq!(panel.occupancy["Block 1"], Some(false));

// The dispatcher throws turnout 5: the panel produces an event.
panel.press_turnout_button();
let seen = deliver(&mut panel.node, &mut detector, 500);
assert_eq!(seen[0].msg, Message::Pcer(EventId(0x0501_0101_2299_0005)));
```

## Tips

* **Register before `start`.** Events registered before login are
  announced in the login burst; events added later are announced
  individually.
* **Use pairs.** Represent each binary state with two events and keep both
  states in the registry consistent (one Valid, one Invalid). Tools like
  JMRI rely on this to show state.
* **Ranges.** Some nodes announce whole blocks of events with
  [`ProducerRangeIdentified`](crate::Message::ProducerRangeIdentified) /
  [`ConsumerRangeIdentified`](crate::Message::ConsumerRangeIdentified).
  The range is encoded in the ID: the trailing run of identical low-order
  bits marks the "don't care" part.
* **Teaching.** [`LearnEvent`](crate::Message::LearnEvent) is part of the
  blue/gold button configuration method; a node in learn mode adopts the
  received ID. Your application decides whether to support it.
* **Emergency events.** The well-known
  [`EMERGENCY_STOP`](crate::EventId::EMERGENCY_STOP) /
  [`EMERGENCY_OFF`](crate::EventId::EMERGENCY_OFF) events (and their
  `CLEAR_` counterparts) are consumed by command stations and trains;
  a dispatch panel should offer a big red button that produces them.
