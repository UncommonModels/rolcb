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
| `ParseError` and `EncodeError` with `Display` and `core::error::Error` (the same trait as `std::error::Error`) | |
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

A browser app (such as a dispatch panel or a web throttle) can run one
`Node` in WASM and talk to a GridConnect USB-CAN adapter directly through
the [Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API)
(Chromium-based browsers):

```text
  ┌──────────── browser tab ─────────────┐
  │  UI (JS / Yew / Leptos / Dioxus)     │
  │        ▲ events, speeds              │
  │  rolcb::Node  (WASM)                 │  Web     ┌─────────────┐
  │        ▲ CanFrame                    │  Serial  │ USB-CAN     │── CAN bus
  │  gridconnect::{format, Decoder}  ◄───┼─────────►│ adapter     │
  └──────────────────────────────────────┘  bytes   └─────────────┘
```

The glue is small. This sketch uses `wasm-bindgen`, `wasm-bindgen-futures`,
`js-sys` and `web-sys` (not dependencies of this crate, so it is not
compiled). web-sys only exposes `Serial` when built with
`RUSTFLAGS="--cfg=web_sys_unstable_apis"` and the `Navigator`, `Serial`,
`SerialPort`, `SerialOptions`, `ReadableStream`,
`ReadableStreamDefaultReader`, `WritableStream` and
`WritableStreamDefaultWriter` features. That API is unstable, so expect the
exact method names to drift between web-sys releases:

```rust,ignore
use std::cell::RefCell;
use std::rc::Rc;

use js_sys::{Reflect, Uint8Array};
use rolcb::gridconnect::{self, Decoder};
use rolcb::{Node, NodeInfo};
use wasm_bindgen::prelude::*;
use wasm_bindgen_futures::{spawn_local, JsFuture};
use web_sys::{ReadableStreamDefaultReader, SerialOptions, SerialPort, WritableStreamDefaultWriter};

fn now_ms() -> u64 {
    web_sys::window().unwrap().performance().unwrap().now() as u64
}

fn flush(node: &mut Node, writer: &WritableStreamDefaultWriter) {
    let mut text = String::new();
    for f in node.take_outgoing() {
        text.push_str(&gridconnect::format(&f));
        text.push('\n');
    }
    if !text.is_empty() {
        // Writes are queued by the stream in order; no need to await each one.
        let _ = writer.write_with_chunk(&Uint8Array::from(text.as_bytes()));
    }
}

/// Call from a button's click handler: `requestPort` needs a user gesture.
#[wasm_bindgen]
pub async fn connect() -> Result<(), JsValue> {
    let serial = web_sys::window().unwrap().navigator().serial();
    let port: SerialPort = JsFuture::from(serial.request_port()).await?.dyn_into()?;
    JsFuture::from(port.open(&SerialOptions::new(115_200))).await?;

    let reader: ReadableStreamDefaultReader = port.readable().get_reader().dyn_into()?;
    let writer = Rc::new(port.writable().get_writer()?);
    let node = Rc::new(RefCell::new(Node::new(
        "05.01.01.01.22.97".parse().unwrap(),
        NodeInfo::default(),
    )));

    // Log in, then poll on a timer.
    {
        let mut n = node.borrow_mut();
        n.start(now_ms());
        flush(&mut n, &writer);
    }
    {
        let (node, writer) = (node.clone(), writer.clone());
        let tick = Closure::<dyn FnMut()>::new(move || {
            let mut node = node.borrow_mut();
            node.poll(now_ms());
            flush(&mut node, &writer);
        });
        web_sys::window()
            .unwrap()
            .set_interval_with_callback_and_timeout_and_arguments_0(tick.as_ref().unchecked_ref(), 50)?;
        tick.forget();
    }

    // Bytes from the adapter.
    spawn_local(async move {
        let mut decoder = Decoder::new();
        loop {
            let Ok(chunk) = JsFuture::from(reader.read()).await else { break };
            if Reflect::get(&chunk, &"done".into()).unwrap().is_truthy() {
                break; // adapter unplugged or port closed
            }
            let bytes = Uint8Array::new(&Reflect::get(&chunk, &"value".into()).unwrap()).to_vec();
            let mut node = node.borrow_mut();
            for frame in decoder.push(&bytes).into_iter().flatten() {
                if let Some(rx) = node.receive(&frame, now_ms()) {
                    // hand rx.msg to the UI
                }
            }
            flush(&mut node, &writer);
        }
    });
    Ok(())
}
```

The same shape works for any other transport a browser offers (WebUSB, a
`fetch` stream, a JS bridge to a native app): push whatever bytes arrive
into a [`Decoder`](crate::gridconnect::Decoder), hand each frame to
[`Node::receive`](crate::Node::receive), and write
[`take_outgoing`](crate::Node::take_outgoing) back out as GridConnect text.

Points to remember in the browser:

* When the adapter is unplugged or the port closes, call
  [`Node::reset`](crate::Node::reset), then
  [`Node::start`](crate::Node::start) again after reopening it. The alias
  table and partial messages are cleared; registered events are kept and
  re-announced.
* Pick a Node ID per installation (for example store a random
  `05.01.01.01.xx.xx` in `localStorage`); two browsers with the same Node
  ID on one bus will see each other as duplicates
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
