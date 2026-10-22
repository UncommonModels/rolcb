# Getting started

This chapter gets an OpenLCB node online and exchanges an event, using
nothing but the crate and a byte buffer standing in for the wire.

## Add the dependency

```toml
[dependencies]
rolcb = "0.1"

# Or, for the async hub and the rolcb-hub binary:
# rolcb = { version = "0.1", features = ["hub"] }

# In a WASM or embedded build:
# rolcb = { version = "0.1", default-features = false }
```

## Five things to know first

1. **Nodes and Node IDs.** Everything on an OpenLCB network is a *node*,
   identified by a unique 48-bit [`NodeId`](crate::NodeId) such as
   `05.01.01.01.22.99`. If you are a hobbyist without your own range, the
   `05.01.01.01.xx.xx` block is reserved for you; pick a value nobody else
   on your layout uses.
2. **Aliases.** On CAN, a node also reserves a 12-bit *alias* that appears
   in every frame instead of the long Node ID. Reserving one takes about
   200 ms (the "login"). [`Node`](crate::Node) does this for you.
3. **Messages.** A node communicates with [`Message`](crate::Message)s:
   global ones everybody sees (events, Initialization Complete) and
   addressed ones sent to a single alias (traction commands, SNIP).
4. **Events.** The workhorse of layout control: a 64-bit
   [`EventId`](crate::EventId) that one node *produces* and others
   *consume*. "Block 4 occupied", "throw turnout 12", "signal 3 red".
5. **Sans-IO.** The crate never does I/O. You move
   [`CanFrame`](crate::CanFrame)s between the node and your transport, and
   you tell it what time it is.

## A node, end to end

The example below logs a node in, receives an event from another node and
sends one of its own. A `Vec<u8>` plays the role of a serial port: we write
GridConnect text into it and read GridConnect text from a byte string.

```rust
use rolcb::gridconnect::{self, Decoder};
use rolcb::{EventId, EventState, Message, Node, NodeInfo};

// Any monotonic millisecond clock works. In a real program:
//   let t0 = std::time::Instant::now();
//   let now = || t0.elapsed().as_millis() as u64;
let mut now: u64 = 0;

let block_occupied: EventId = "05.01.01.01.22.00.00.10".parse()?;
let my_button: EventId = "05.01.01.01.22.99.00.01".parse()?;

let mut node = Node::new(
    "05.01.01.01.22.99".parse()?,
    NodeInfo {
        manufacturer: "Example Co".into(),
        model: "First node".into(),
        software_version: "0.1".into(),
        ..Default::default()
    },
);
node.add_consumer(block_occupied, EventState::Unknown);
node.add_producer(my_button, EventState::Unknown);

let mut wire: Vec<u8> = Vec::new(); // "serial port" output
let flush = |node: &mut Node, wire: &mut Vec<u8>| {
    for frame in node.take_outgoing() {
        wire.extend_from_slice(gridconnect::format(&frame).as_bytes());
        wire.push(b'\n');
    }
};

// 1. Login: four CID frames now…
node.start(now);
flush(&mut node, &mut wire);
assert_eq!(wire.iter().filter(|&&b| b == b'\n').count(), 4);

// …then RID, AMD, Initialization Complete and our Identified messages
// once 200 ms have passed. Call poll() regularly; it is cheap.
now += 200;
node.poll(now);
flush(&mut node, &mut wire);
assert!(node.is_permitted());
println!("alias {:03X}\n{}", node.alias().unwrap(), String::from_utf8_lossy(&wire));

// 2. Receive. Bytes from the port go through a streaming Decoder, which
// copes with frames split across reads.
let mut decoder = Decoder::new();
let incoming = b":X195B4123N0501010122000010;\n";
for frame in decoder.push(incoming) {
    let frame = frame?; // or log and skip malformed frames
    if let Some(rx) = node.receive(&frame, now) {
        // rx.src_alias = 0x123, rx.src_node = None (we never saw its AMD)
        if rx.msg == Message::Pcer(block_occupied) {
            println!("block 1 is now occupied");
        }
    }
}

// 3. Send. Queue a message, then flush.
wire.clear();
node.send(Message::Pcer(my_button));
flush(&mut node, &mut wire);
let line = String::from_utf8(wire.clone()).unwrap();
assert!(line.starts_with(":X195B4"));       // PCER from our alias
assert!(line.contains("0501010122990001")); // the event ID
# Ok::<(), rolcb::ParseError>(())
```

That is the entire contract: **`start` once, `poll` often, `receive` every
frame, `send` messages, drain `take_outgoing` after each of those.**

## Connecting to a real layout

You need a way to put frames on the bus:

* **A USB-CAN adapter** (RR-CirKits LCC Buffer-USB, SPROG CANISB, TCS
  CS-105 USB and others). These speak GridConnect over a serial port.
* **A GridConnect TCP hub** such as JMRI's OpenLCB hub or this crate's
  `rolcb-hub` (port 12021). Open a TCP socket and exchange GridConnect
  lines.
* **A browser.** Run `rolcb-hub` and connect with a WebSocket to port
  12022; see [`no_std` and WASM](crate::guide::no_std).

The [architecture chapter](crate::guide::architecture) shows complete loops for
blocking sockets and for tokio, and the [hub chapter](crate::guide::hub) covers
`rolcb-hub`.

## Watching traffic

The `gc_dump` example decodes a GridConnect capture (from JMRI's monitor,
`rolcb-hub -v`, or `cat /dev/ttyACM0`):

```text
$ cargo run --example gc_dump -- capture.txt
285 Cid { seq: 7, node_id_bits: 80, alias: 645 } []
285 Rid { alias: 645 } []
285 InitComplete                 InitializationComplete { node_id: NodeId(05.01.01.01.22.00), simple: false }
123 PCER                         Pcer(EventId(05.01.01.01.22.00.00.10))
```

## Next steps

* [Writing a throttle](crate::guide::throttle): drive a train.
* [Consuming and producing events](crate::guide::events): build a panel.
* [Protocol reference](crate::guide::reference): every MTI, flag and code.
