# `no_std` and WASM

Everything except the `hub` module works without the standard library.
The crate needs only `core` and `alloc` (for `Vec`, `String`, `BTreeMap` and
`VecDeque`), so it runs in the browser via `wasm32-unknown-unknown` and on
microcontrollers that have a heap.

## Enabling `no_std`

```toml
[dependencies]
rolcb = { version = "0.1", default-features = false }
# Optional: NodeId/EventId as dotted-hex strings in JSON, also no_std.
# rolcb = { version = "0.1", default-features = false, features = ["serde"] }
```

With `std` off:

| Available | Not available |
|---|---|
| `can`, `gridconnect`, `message`, `node`, `traction`, `event`, `node_id`, `mti`, `float16`, `alias` | `hub` (needs tokio) |
| `ParseError` with `Display` | `impl std::error::Error for ParseError` |
| `serde` support (feature `serde`) | |

Check that a build really is `no_std`:

```sh
cargo build --no-default-features --target wasm32-unknown-unknown
cargo build --no-default-features --target thumbv7em-none-eabihf   # needs a global allocator in the final binary
```

## Time

The crate never reads a clock. Pass any monotonic millisecond count to
`start`, `poll` and `receive`:

| Environment | Clock |
|---|---|
| Browser (WASM) | `performance.now()` as `u64` (via `web-sys` or `js-sys`) |
| Embassy | `embassy_time::Instant::now().as_millis()` |
| RTIC / bare metal | a SysTick or timer counter |
| Tests | a plain `u64` you advance by hand |

Only the 200 ms alias reservation wait depends on time, so a coarse clock
(10 ms resolution) is fine.

## In the browser

A typical browser app (such as a dispatch panel or a web throttle) runs
one `Node` in WASM and talks to `rolcb-hub` over a WebSocket:

```text
  ┌──────────── browser tab ─────────────┐
  │  UI (JS / Yew / Leptos / Dioxus)     │
  │        ▲ events, speeds              │
  │  rolcb::Node  (WASM)                 │          ┌───────────┐
  │        ▲ CanFrame                    │  ws://   │ rolcb-hub │── CAN / JMRI
  │  gridconnect::{format, Decoder}  ◄───┼─────────►│   :12022  │
  └──────────────────────────────────────┘   text   └───────────┘
```

The glue is small. With `wasm-bindgen` and `web-sys` (not dependencies of
this crate, so this snippet is not compiled):

```rust,ignore
use std::cell::RefCell;
use std::rc::Rc;

use rolcb::gridconnect::{self, Decoder};
use rolcb::{Node, NodeInfo};
use wasm_bindgen::prelude::*;
use web_sys::{MessageEvent, WebSocket};

fn now_ms() -> u64 {
    web_sys::window().unwrap().performance().unwrap().now() as u64
}

fn flush(node: &mut Node, ws: &WebSocket) {
    let mut text = String::new();
    for f in node.take_outgoing() {
        text.push_str(&gridconnect::format(&f));
        text.push('\n');
    }
    if !text.is_empty() {
        let _ = ws.send_with_str(&text);
    }
}

#[wasm_bindgen(start)]
pub fn start() -> Result<(), JsValue> {
    let ws = WebSocket::new("ws://localhost:12022/")?;
    let node = Rc::new(RefCell::new(Node::new(
        "05.01.01.01.22.97".parse().unwrap(),
        NodeInfo::default(),
    )));
    let decoder = Rc::new(RefCell::new(Decoder::new()));

    // Frames from the hub.
    {
        let (node, decoder, ws2) = (node.clone(), decoder.clone(), ws.clone());
        let onmessage = Closure::<dyn FnMut(MessageEvent)>::new(move |e: MessageEvent| {
            let Some(text) = e.data().as_string() else { return };
            let mut node = node.borrow_mut();
            for frame in decoder.borrow_mut().push(text.as_bytes()).into_iter().flatten() {
                if let Some(rx) = node.receive(&frame, now_ms()) {
                    // hand rx.msg to the UI
                }
            }
            flush(&mut node, &ws2);
        });
        ws.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));
        onmessage.forget();
    }
    // Log in once the socket is open, and poll on a timer.
    {
        let (node, ws2) = (node.clone(), ws.clone());
        let onopen = Closure::<dyn FnMut()>::new(move || {
            let mut node = node.borrow_mut();
            node.start(now_ms());
            flush(&mut node, &ws2);
        });
        ws.set_onopen(Some(onopen.as_ref().unchecked_ref()));
        onopen.forget();
    }
    {
        let (node, ws2) = (node.clone(), ws.clone());
        let tick = Closure::<dyn FnMut()>::new(move || {
            let mut node = node.borrow_mut();
            node.poll(now_ms());
            flush(&mut node, &ws2);
        });
        web_sys::window()
            .unwrap()
            .set_interval_with_callback_and_timeout_and_arguments_0(tick.as_ref().unchecked_ref(), 50)?;
        tick.forget();
    }
    Ok(())
}
```

Points to remember in the browser:

* When the socket closes, call [`Node::reset`](crate::Node::reset), then
  [`Node::start`](crate::Node::start) again after reconnecting. The alias
  table and partial messages are cleared; registered events are kept and
  re-announced.
* Pick a Node ID per installation (for example store a random
  `05.01.01.01.xx.xx` in `localStorage`); two tabs with the same Node ID
  will see each other as duplicates
  ([`Node::duplicate_node_id_detected`](crate::Node::duplicate_node_id_detected)).
* With the `serde` feature, `NodeId` and `EventId` serialize as
  `"05.01.01.01.22.00.00.10"` strings, which is convenient for passing
  configuration to and from JavaScript.

## On a microcontroller

On an MCU with a CAN peripheral there is no GridConnect at all: map
[`CanFrame`](crate::CanFrame) to your driver's frame type using the raw
29-bit [`id`](crate::CanFrame::id) and [`data`](crate::CanFrame::data). The
service routine is the same as everywhere else:

```rust
use rolcb::{CanFrame, Message, Node};

/// Whatever your HAL provides. (Defined here so the example compiles.)
trait CanDriver {
    fn try_receive(&mut self) -> Option<(u32, [u8; 8], usize)>;
    fn try_transmit(&mut self, id: u32, data: &[u8]) -> bool;
}

/// Call from the main loop or a periodic task.
fn service(node: &mut Node, can: &mut impl CanDriver, tx_queue: &mut Vec<CanFrame>, now_ms: u64) {
    while let Some((id, data, len)) = can.try_receive() {
        let Ok(frame) = CanFrame::new(id, &data[..len]) else { continue };
        if let Some(rx) = node.receive(&frame, now_ms) {
            if let Message::Pcer(_event) = rx.msg {
                // drive an output
            }
        }
    }
    node.poll(now_ms);
    tx_queue.extend(node.take_outgoing());
    // Transmit in order; stop when the mailbox is full and retry next time.
    while let Some(frame) = tx_queue.first() {
        if !can.try_transmit(frame.id, frame.data()) {
            break;
        }
        tx_queue.remove(0);
    }
}

// A loopback "driver" to exercise the routine.
struct Fake { rx: Vec<(u32, [u8; 8], usize)>, sent: Vec<u32> }
impl CanDriver for Fake {
    fn try_receive(&mut self) -> Option<(u32, [u8; 8], usize)> { self.rx.pop() }
    fn try_transmit(&mut self, id: u32, _data: &[u8]) -> bool { self.sent.push(id); true }
}

let mut node = Node::new(rolcb::NodeId::new(0x0501_0101_2296), rolcb::NodeInfo::default());
let mut can = Fake { rx: Vec::new(), sent: Vec::new() };
let mut queue = Vec::new();
node.start(0);
service(&mut node, &mut can, &mut queue, 0);
assert_eq!(can.sent.len(), 4); // CID7..CID4
service(&mut node, &mut can, &mut queue, 200);
assert!(node.is_permitted());
```

Make sure the CAN peripheral is configured for **extended (29-bit)
identifiers at 125 kbit/s**, the OpenLCB CAN bit rate, and that it does
not deliver its own transmitted frames back to you.
