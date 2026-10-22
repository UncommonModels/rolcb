# Writing a throttle

This chapter builds a complete throttle session against a simulated train:
**search → assign → speed and functions → heartbeat → release**. Every
step is real code that runs as a doctest.

## Background: trains in OpenLCB

The *Traction Control Standard* models every locomotive (or consist) as a
**train node**: an OpenLCB node with its own Node ID and alias that accepts
Traction Control Commands. For DCC locomotives the train node is
*virtual*: it lives inside the command station (or a JMRI instance),
which translates commands into DCC packets on the track.

A throttle never talks DCC. It:

1. **Finds** the train node with the *Train Search Standard*: it sends
   Identify Producer for a special event ID that encodes "DCC address 3".
   The command station creates the virtual train node if needed
   (`ALLOCATE` flag), and the train node answers Producer Identified for
   that same event. The source alias of the answer *is* the train.
2. **Assigns** itself as the train's controller (Assign Controller). From
   then on it is the train's owner. If another throttle asks for the
   train, the train sends the current owner a *Controller Changing
   Notify* command, and the owner answers whether it agrees (see
   [below](#losing-control-to-another-throttle)).
3. **Drives** it with Set Speed / Set Function, and may query state.
4. **Answers heartbeats.** A train may ask its controller to prove it is
   still alive by sending a heartbeat request with a timeout; the throttle
   answers with a No-op command. If it doesn't, the train stops itself.
5. **Releases** the train when done.

Speeds are IEEE half-floats in scale metres per second, and the **sign bit
is the direction** (set = reverse). That means "stopped, facing reverse"
(`-0.0`) is distinct from "stopped, facing forward" (`+0.0`), so a throttle
can set the direction before moving. See
[`traction::Speed`](crate::traction::Speed).

## The walkthrough

The simulated train is just another [`Node`](crate::Node) with a few lines
of application logic, the same thing a command station does for each
virtual train. Both nodes are connected by a `deliver` helper that moves
frames from one to the other, standing in for the CAN bus.

```rust
use rolcb::message::{protocol, Message};
use rolcb::node::ALIAS_RESERVE_MS;
use rolcb::traction::{search, Command, Reply, Speed};
use rolcb::{EventState, Node, NodeId, NodeInfo, Received};

/// The "bus": move all frames `from` wants to send into `to`, and return
/// the messages `to` decoded.
fn deliver(from: &mut Node, to: &mut Node, now: u64) -> Vec<Received> {
    from.take_outgoing()
        .iter()
        .filter_map(|f| to.receive(f, now))
        .collect()
}

// ---------------------------------------------------------------------------
// A tiny virtual DCC train for address 3, like a command station would host.
// ---------------------------------------------------------------------------
#[derive(Default, Debug)]
struct TrainState {
    controller: Option<NodeId>,
    speed: Speed,
    f0: bool,
}

fn train_logic(train: &mut Node, st: &mut TrainState, rx: &Received) {
    let me = train.alias().unwrap();
    match &rx.msg {
        // Answer train searches for DCC address 3.
        Message::IdentifyProducer(ev) if search::decode(*ev).map(|(n, _)| n) == Some(3) => {
            train.send(Message::ProducerIdentified { event: *ev, state: EventState::Valid });
        }
        Message::TractionCommand { dest, payload } if *dest == me => {
            let reply = |payload: Vec<u8>| Message::TractionReply { dest: rx.src_alias, payload };
            match Command::decode(payload) {
                Command::AssignController { controller, .. } => {
                    st.controller = Some(controller);
                    train.send(reply(Reply::AssignController { result: 0 }.encode()));
                }
                Command::ReleaseController { controller, .. } if st.controller == Some(controller) => {
                    st.controller = None;
                }
                Command::SetSpeed(s) => st.speed = s,
                Command::EmergencyStop => {
                    st.speed = if st.speed.is_reverse() { Speed::STOP_REVERSE } else { Speed::STOP_FORWARD }
                }
                Command::SetFunction { addr: 0, value } => st.f0 = value != 0,
                Command::QuerySpeed => train.send(reply(
                    Reply::QuerySpeed {
                        set_speed: st.speed,
                        estop: false,
                        commanded: st.speed,
                        actual: Speed::NAN,
                    }
                    .encode(),
                )),
                Command::Noop => {} // heartbeat answered; nothing to do
                _ => {}
            }
        }
        _ => {}
    }
}

// ---------------------------------------------------------------------------
// Set up both nodes and log them in.
// ---------------------------------------------------------------------------
let me = NodeId::new(0x0501_0101_2299);
let mut throttle = Node::new(me, NodeInfo { model: "Handheld throttle".into(), ..Default::default() });

let mut train = Node::new(NodeId::new(0x0501_0101_2201), NodeInfo::default());
// A train must advertise Traction Control; this also stops the Node from
// auto-rejecting traction commands addressed to it.
train.set_protocols(protocol::EVENT_EXCHANGE | protocol::SNIP | protocol::TRACTION_CONTROL);
let mut st = TrainState::default();

let mut now = 0;
throttle.start(now);
train.start(now);
deliver(&mut throttle, &mut train, now);
deliver(&mut train, &mut throttle, now);
now += ALIAS_RESERVE_MS;
throttle.poll(now);
train.poll(now);
deliver(&mut throttle, &mut train, now);
deliver(&mut train, &mut throttle, now);
assert!(throttle.is_permitted() && train.is_permitted());

// One round trip: throttle → train (train logic runs) → throttle.
let round_trip = |throttle: &mut Node, train: &mut Node, st: &mut TrainState, now: u64| {
    for rx in deliver(throttle, train, now) {
        train_logic(train, st, &rx);
    }
    deliver(train, throttle, now)
};

// ---------------------------------------------------------------------------
// 1. Search for DCC address 3 (short address, allocate if missing).
// ---------------------------------------------------------------------------
let search_event = search::dcc_event(3, false, search::ALLOCATE);
throttle.send(Message::IdentifyProducer(search_event));

let replies = round_trip(&mut throttle, &mut train, &mut st, now);
let train_alias = replies
    .iter()
    .find_map(|rx| match rx.msg {
        Message::ProducerIdentified { event, .. } if event == search_event => Some(rx.src_alias),
        _ => None,
    })
    .expect("no train answered");
assert_eq!(Some(train_alias), train.alias());

// ---------------------------------------------------------------------------
// 2. Assign ourselves as controller. 9 payload bytes → 2 CAN frames.
// ---------------------------------------------------------------------------
let cmd = |c: Command| Message::TractionCommand { dest: train_alias, payload: c.encode() };

throttle.send(cmd(Command::AssignController { flags: 0, controller: me }));
let replies = round_trip(&mut throttle, &mut train, &mut st, now);
let accepted = replies.iter().any(|rx| match &rx.msg {
    Message::TractionReply { payload, .. } => {
        Reply::decode(payload) == Reply::AssignController { result: 0 }
    }
    _ => false,
});
assert!(accepted);
assert_eq!(st.controller, Some(me));

// ---------------------------------------------------------------------------
// 3. Headlight on, then 25 scale mph forward.
// ---------------------------------------------------------------------------
throttle.send(cmd(Command::SetFunction { addr: 0, value: 1 }));
throttle.send(cmd(Command::SetSpeed(Speed::from_mph(25.0, false))));
round_trip(&mut throttle, &mut train, &mut st, now);
assert!(st.f0);
assert!((st.speed.mph() - 25.0).abs() < 0.1 && !st.speed.is_reverse());

// Read the speed back.
throttle.send(cmd(Command::QuerySpeed));
let replies = round_trip(&mut throttle, &mut train, &mut st, now);
match &replies[0].msg {
    Message::TractionReply { payload, .. } => match Reply::decode(payload) {
        Reply::QuerySpeed { set_speed, .. } => assert_eq!(set_speed, st.speed),
        r => panic!("unexpected {r:?}"),
    },
    m => panic!("unexpected {m:?}"),
}

// ---------------------------------------------------------------------------
// 4. The train asks for a heartbeat; answer with Noop before the timeout.
// ---------------------------------------------------------------------------
now += 5_000;
let my_alias = throttle.alias().unwrap();
train.send(Message::TractionReply {
    dest: my_alias,
    payload: Reply::Heartbeat { timeout_s: 3 }.encode(),
});
for rx in deliver(&mut train, &mut throttle, now) {
    if let Message::TractionReply { payload, .. } = &rx.msg {
        if let Reply::Heartbeat { timeout_s } = Reply::decode(payload) {
            // A real throttle would just make sure it sends *something*
            // within timeout_s seconds. Noop is the canonical answer.
            assert_eq!(timeout_s, 3);
            throttle.send(Message::TractionCommand {
                dest: rx.src_alias,
                payload: Command::Noop.encode(),
            });
        }
    }
}
round_trip(&mut throttle, &mut train, &mut st, now);

// ---------------------------------------------------------------------------
// 5. Reverse: stop facing reverse first, then move. Then emergency stop.
// ---------------------------------------------------------------------------
throttle.send(cmd(Command::SetSpeed(Speed::STOP_REVERSE)));
throttle.send(cmd(Command::SetSpeed(Speed::from_mph(10.0, true))));
round_trip(&mut throttle, &mut train, &mut st, now);
assert!(st.speed.is_reverse());

throttle.send(cmd(Command::EmergencyStop));
round_trip(&mut throttle, &mut train, &mut st, now);
assert_eq!(st.speed, Speed::STOP_REVERSE);

// ---------------------------------------------------------------------------
// 6. Release the train.
// ---------------------------------------------------------------------------
throttle.send(cmd(Command::ReleaseController { flags: 0, controller: me }));
round_trip(&mut throttle, &mut train, &mut st, now);
assert_eq!(st.controller, None);

// ---------------------------------------------------------------------------
// 7. Losing control. We take the train again, then another throttle asks for
//    it: the train sends *us* a Controller Changing Notify command.
// ---------------------------------------------------------------------------
throttle.send(cmd(Command::AssignController { flags: 0, controller: me }));
round_trip(&mut throttle, &mut train, &mut st, now);
assert_eq!(st.controller, Some(me));

let other_throttle = NodeId::new(0x0501_0101_2277);
train.send(Message::TractionCommand {
    dest: my_alias,
    payload: Command::ControllerChangingNotify { flags: 0, new_controller: other_throttle }
        .encode(),
});

let mut lost = false;
for rx in deliver(&mut train, &mut throttle, now) {
    // The throttle's Node delivers this command instead of rejecting it,
    // even though the throttle does not advertise TRACTION_CONTROL.
    if let Message::TractionCommand { payload, .. } = &rx.msg {
        if let Command::ControllerChangingNotify { new_controller, .. } = Command::decode(payload) {
            // Agree to the handover (result 0)…
            throttle.send(Message::TractionReply {
                dest: rx.src_alias,
                payload: Reply::ControllerChanged { result: 0 }.encode(),
            });
            // …and mark the train as lost: stop sending it commands, stop
            // answering its heartbeats, and tell the user.
            lost = true;
            println!("train taken over by {new_controller}");
        }
    }
}
assert!(lost);

// Exactly one frame went back: our answer, not an Optional Interaction Rejected.
let answers = deliver(&mut throttle, &mut train, now);
match &answers[..] {
    [rx] => match &rx.msg {
        Message::TractionReply { payload, .. } => {
            assert_eq!(Reply::decode(payload), Reply::ControllerChanged { result: 0 })
        }
        m => panic!("unexpected {m:?}"),
    },
    other => panic!("unexpected {other:?}"),
}
```

## Losing control to another throttle

When a second throttle sends Assign Controller for a train you control,
the train does not simply switch. It first sends **you** a Traction Control
*Command*, [`Command::ControllerChangingNotify`](crate::traction::Command::ControllerChangingNotify)
(`20 04 flags NodeID`, with the Node ID of the throttle that wants the
train), and waits for your *reply*,
[`Reply::ControllerChanged`](crate::traction::Reply::ControllerChanged)
(`20 04 result`):

| You answer | Meaning | Then |
|---|---|---|
| `result: 0` | handover accepted | the train assigns the new throttle; treat the train as lost |
| nonzero | handover refused | the train rejects the other throttle's Assign Controller |

Step 7 above shows the exchange. Three things to get right:

* **Look for it in `TractionCommand`, not `TractionReply`.** It is the one
  traction *command* a throttle ever receives. [`Node`](crate::Node) never
  auto-rejects a command starting `20 04`, whether or not you advertise
  [`TRACTION_CONTROL`](crate::message::protocol::TRACTION_CONTROL).
* **Answer it.** If you stay silent the train may wait and eventually
  decide for you; answering promptly keeps the handover snappy.
* **Mark the throttle as lost** once you accept: stop sending speed
  commands and Noops to that train and tell the user. Most throttles
  accept by default and offer a "steal back" by sending Assign Controller
  again.

## Notes for a real throttle

* **Keep the search event.** Several trains (or a train and a consist)
  can answer one search; collect answers for a short time (say 500 ms)
  and let the user choose if there is more than one. Without the `EXACT`
  flag, nodes may also answer partial matches.
* **Alias vs Node ID.** The train's alias can change if it has to
  re-reserve after a collision. [`Received::src_node`](crate::Received)
  gives you its Node ID once the mapping is known; use
  [`Node::alias_of`](crate::Node::alias_of) to look up the current alias
  before sending.
* **Heartbeat timing.** Respond to [`Reply::Heartbeat`](crate::traction::Reply::Heartbeat)
  well within the timeout, or simply send a [`Command::Noop`](crate::traction::Command::Noop)
  (or a Set Speed) every few seconds while you own the train.
* **Controller handover.** If another throttle assigns itself, the train
  sends you [`Command::ControllerChangingNotify`](crate::traction::Command::ControllerChangingNotify);
  answer with [`Reply::ControllerChanged`](crate::traction::Reply::ControllerChanged)
  and, if you accepted, tell the user they lost the train (see
  [above](#losing-control-to-another-throttle)).
* **Speed steps vs m/s.** DCC command stations conventionally map 1 scale
  mph to one 128-step speed step (126 mph = full speed), which is why
  [`Speed::from_mph`](crate::traction::Speed::from_mph) exists. Use
  [`search::SPEED_STEPS_MASK`](crate::traction::search::SPEED_STEPS_MASK)
  values in the search flags to request 14/28/128 steps when allocating.
* **Functions.** F0..F28 are function addresses 0..28; on/off functions
  use value 1/0 ([`function_value`](crate::traction::function_value)).
* **Emergency stop for everyone.** To stop the whole layout, send a PCER
  for [`EventId::EMERGENCY_STOP`](crate::EventId::EMERGENCY_STOP) (and
  [`CLEAR_EMERGENCY_STOP`](crate::EventId::CLEAR_EMERGENCY_STOP) to resume),
  instead of commanding each train.
