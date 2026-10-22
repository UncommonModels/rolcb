# rolcb
Rustified OpenLCB library.

`rolcb` is a sans-IO implementation of the [OpenLCB / LCC](https://openlcb.org) protocol stack for model railroads. You feed it CAN frames and the time. It gives back frames to send and decoded messages. Because of that it works the same from tokio, from WASM in a browser, or from `no_std` embedded code.

It also includes **`rolcb-hub`**, a standalone GridConnect hub. It relays traffic between TCP clients (JMRI etc.), WebSocket clients (browser apps), other hubs and USB serial adapters.

## Documentation

**API docs and guide: <https://www.uncommonmodels.com/rolcb/>** (built from `main` by [`.github/workflows/docs.yml`](.github/workflows/docs.yml); also on [docs.rs](https://docs.rs/rolcb) once published).

Besides the API reference, the docs include guide chapters (sources in [`docs/`](docs/), and every Rust example in them runs as a doctest):

- Getting started
- Architecture and the sans-IO model
- Writing a throttle: search, assign, speed, heartbeat, release
- Consuming and producing events: a dispatch-panel example
- Running and embedding the hub: CLI flags, `Hub::port()` from your own server, JMRI
- `no_std` and WASM
- Protocol reference tables: MTIs, CAN header layout, frame types, PIP bits, traction bytes, error codes

Build and view them locally:

```sh
scripts/build-docs.sh --open        # or: cargo doc --no-deps --all-features --open
```

The script uses nightly with `--cfg docsrs` (feature badges, as on docs.rs) if you have a nightly toolchain, and stable otherwise. Output goes to `target/doc/`.

## Library

| Module | What it does |
|---|---|
| `can` | 29-bit OpenLCB CAN identifiers (`CanFrame`, `CanHeader`), CID/RID/AMD/AME/AMR |
| `gridconnect` | `:X19490ABCN;` text encoding plus a streaming decoder |
| `message` | `Message` enum ⇄ frames, multi-frame addressed messages, datagrams |
| `node` | Virtual node: alias reservation and defence, login, Verify Node ID, PIP, SNIP, event identification |
| `traction` | Traction Control commands and replies, half-float speeds, Train Search events |
| `event`, `node_id`, `mti`, `float16` | Basic types |
| `hub` *(feature `hub`)* | Async GridConnect hub built on tokio |

```rust
use rolcb::{Node, NodeId, NodeInfo, EventId, EventState, Message, gridconnect};

let mut node = Node::new("05.01.01.01.22.99".parse()?, NodeInfo::default());
node.add_consumer("05.01.01.01.22.00.00.10".parse()?, EventState::Unknown);
node.start(now_ms());                      // sends CID frames
// ... every few ms:
node.poll(now_ms());                       // RID/AMD/Init Complete after 200 ms
for frame in node.take_outgoing() {
    port.write_all(gridconnect::format(&frame).as_bytes())?;
}
// for each frame read from the bus:
if let Some(rx) = node.receive(&frame, now_ms()) {
    if let Message::Pcer(ev) = rx.msg { /* an event happened */ }
}
node.send(Message::Pcer(EventId(0x0501_0101_2200_0100)));
```

Cargo features:

- `std` (default). Turn it off for `no_std` + `alloc`.
- `serde`: `NodeId`/`EventId` serialize as dotted hex strings.
- `hub`: the async hub module (tokio, tokio-tungstenite).
- `serial`: serial-port uplinks for the hub.
- `cli`: the `rolcb-hub` binary.

## rolcb-hub

```sh
cargo install --path . --features cli,serial

rolcb-hub                                   # GridConnect TCP :12021 + WebSocket :12022
rolcb-hub --serial /dev/ttyACM0             # bridge a USB LCC buffer onto the network
rolcb-hub --upstream jmri.local:12021 -v    # join another hub and print decoded traffic
```

Every frame received on one port is sent to all other ports and never echoed back to its sender. Uplinks reconnect automatically. Each WebSocket text message may carry any number of GridConnect frames.

## Tools

```sh
cargo run --example gc_dump -- capture.txt   # decode a GridConnect log
```

## Testing

```sh
cargo test --features cli,serial
cargo build --no-default-features --target wasm32-unknown-unknown
```

MIT licensed.
