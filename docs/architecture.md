# Architecture and the sans-IO model

`rolcb` is a *sans-IO* protocol library: it implements the OpenLCB state
machines and encodings, but performs no I/O and reads no clock. This
chapter explains the layers, the contract between the library and your
code, and shows complete driver loops for common environments.

## Layers

```text
 ┌────────────────────────────────────────────────────────────────────┐
 │ your application: throttle UI, dispatch panel, train, bridge, …    │
 └───────────────┬───────────────────────────────────▲────────────────┘
       Message / send()                     Received / receive()
 ┌───────────────▼───────────────────────────────────┴────────────────┐
 │ node      Node: alias reservation + defence, login, auto-replies,  │
 │           alias ↔ Node ID table, event registry                    │
 ├────────────────────────────────────────────────────────────────────┤
 │ message   Message ⇄ frames: global / addressed multi-frame /       │
 │           datagram, Assembler for reassembly                       │
 │ traction  Command / Reply payloads, Speed, train search events     │
 ├────────────────────────────────────────────────────────────────────┤
 │ can       CanFrame, CanHeader (29-bit identifier), CID frames      │
 │ alias     alias candidate generator                                │
 │ mti, event, node_id, float16 — value types                         │
 └───────────────┬───────────────────────────────────▲────────────────┘
       take_outgoing(): CanFrame              CanFrame
 ┌───────────────▼───────────────────────────────────┴────────────────┐
 │ gridconnect  format() / parse() / Decoder  (":X195B4123N…;")       │
 └───────────────┬───────────────────────────────────▲────────────────┘
                 ▼          bytes / text             │
 ┌────────────────────────────────────────────────────────────────────┐
 │ YOUR I/O: serial port, TCP socket, WebSocket, SocketCAN, MCU CAN   │
 │           (or the `hub` module, which does this part for you)      │
 └────────────────────────────────────────────────────────────────────┘
```

You can enter at any level. A bus monitor needs only
[`gridconnect`](crate::gridconnect) and
[`Assembler`](crate::message::Assembler); a throttle uses
[`Node`](crate::Node) and [`traction`](crate::traction); a hub needs
nothing above `gridconnect`.

## Why sans-IO

* **One implementation everywhere.** The same `Node` runs under tokio,
  in a browser (WASM, driven by `WebSocket` callbacks and
  `requestAnimationFrame`), on a microcontroller with an interrupt-driven
  CAN peripheral, or in a plain thread with blocking sockets.
* **Deterministic tests.** Time is a parameter, so the 200 ms alias wait,
  collisions and heartbeats can be tested instantly with a fake clock. The
  crate's own tests and every example in these docs work that way.
* **No hidden tasks.** Nothing runs unless you call it. There are no
  background threads, no locks and no allocation outside the calls you
  make.

## The contract

A [`Node`](crate::Node) has a small surface:

| Call | When | Effect |
|---|---|---|
| [`start(now)`](crate::Node::start) | once the link is up | queues CID frames, arms the 200 ms timer |
| [`poll(now)`](crate::Node::poll) | periodically, or at [`next_deadline()`](crate::Node::next_deadline) | finishes login when the timer expires |
| [`receive(&frame, now)`](crate::Node::receive) | for **every** frame read | updates alias state, queues auto-replies, returns decoded messages |
| [`send(msg)`](crate::Node::send) | whenever the app wants | encodes immediately, or holds it until login completes |
| [`take_outgoing()`](crate::Node::take_outgoing) | after any of the above | returns frames to write, in order |
| [`reset()`](crate::Node::reset) | when the link drops | back to inhibited; call `start` again on reconnect |

Rules of thumb:

* **Drain after every call.** `receive` can queue replies (Verified Node
  ID, SNIP, an RID defending our alias). Write them promptly.
* **Feed everything.** Pass frames addressed to other nodes too; the node
  needs them to learn aliases and detect collisions.
* **Don't echo.** Never feed the node its own transmitted frames. A
  frame with our alias that we did not send means a collision, and the
  node will give up its alias. GridConnect hubs (including `rolcb-hub`)
  and CAN controllers don't echo, so this only matters if you build your
  own loopback.
* **Monotonic milliseconds.** `now` must not go backwards. Its origin is
  irrelevant: `Instant::elapsed()`, `performance.now()` or a SysTick
  counter all work.

## A blocking driver (std only)

The simplest possible real driver: a thread, a TCP connection to a
GridConnect hub, and a read timeout to give `poll` a chance to run.

```rust,no_run
use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use rolcb::gridconnect::{self, Decoder};
use rolcb::{Message, Node, NodeInfo};

fn main() -> std::io::Result<()> {
    let t0 = Instant::now();
    let now = || t0.elapsed().as_millis() as u64;

    let mut sock = TcpStream::connect("localhost:12021")?;
    sock.set_read_timeout(Some(Duration::from_millis(20)))?;

    let mut node = Node::new("05.01.01.01.22.99".parse().unwrap(), NodeInfo::default());
    let mut decoder = Decoder::new();
    let mut buf = [0u8; 1024];
    node.start(now());

    loop {
        // 1. Read whatever arrived (or time out after 20 ms).
        match sock.read(&mut buf) {
            Ok(0) => return Ok(()), // hub closed the connection
            Ok(n) => {
                for frame in decoder.push(&buf[..n]).into_iter().flatten() {
                    if let Some(rx) = node.receive(&frame, now()) {
                        if let Message::Pcer(ev) = rx.msg {
                            println!("event {ev} from {:03X}", rx.src_alias);
                        }
                    }
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock
                || e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(e),
        }
        // 2. Run timers.
        node.poll(now());
        // 3. Write everything the node wants to send.
        for frame in node.take_outgoing() {
            writeln!(sock, "{}", gridconnect::format(&frame))?;
        }
    }
}
```

## A tokio driver

With tokio, `select!` between the socket, a timer and an application
channel. (This example needs the `hub` feature only because that is what
pulls in tokio for the docs; your application just depends on tokio.)

```rust,no_run
# #[cfg(feature = "hub")]
# mod demo {
use std::time::Duration;

use rolcb::gridconnect::{self, Decoder};
use rolcb::{Message, Node};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use tokio::time::{interval, Instant};

/// Run `node` on a GridConnect TCP link. Messages from `app_rx` are sent;
/// received messages go to `app_tx`.
pub async fn run(
    mut node: Node,
    addr: &str,
    mut app_rx: mpsc::Receiver<Message>,
    app_tx: mpsc::Sender<rolcb::Received>,
) -> std::io::Result<()> {
    let t0 = Instant::now();
    let now = move || t0.elapsed().as_millis() as u64;

    let sock = TcpStream::connect(addr).await?;
    sock.set_nodelay(true)?;
    let (mut rd, mut wr) = sock.into_split();
    let mut decoder = Decoder::new();
    let mut buf = [0u8; 2048];
    let mut tick = interval(Duration::from_millis(25));

    node.start(now());
    loop {
        tokio::select! {
            n = rd.read(&mut buf) => {
                let n = n?;
                if n == 0 { node.reset(); return Ok(()); }
                for frame in decoder.push(&buf[..n]).into_iter().flatten() {
                    if let Some(rx) = node.receive(&frame, now()) {
                        let _ = app_tx.send(rx).await;
                    }
                }
            }
            Some(msg) = app_rx.recv() => node.send(msg),
            _ = tick.tick() => node.poll(now()),
        }
        let mut out = String::new();
        for frame in node.take_outgoing() {
            out.push_str(&gridconnect::format(&frame));
            out.push('\n');
        }
        if !out.is_empty() {
            wr.write_all(out.as_bytes()).await?;
        }
    }
}
# }
# fn main() {}
```

If your application already runs an in-process [`hub`](crate::guide::hub), use
a hub `Port` instead of a TCP socket; the loop is identical except that
frames are passed directly, without GridConnect text.

## Reading messages without a node

A passive tool (logger, bus analyser, the `rolcb-hub -v` monitor) does not
need a `Node`, because it never transmits. An
[`Assembler`](crate::message::Assembler) reassembles multi-frame messages
from all senders:

```rust
use rolcb::can::CanHeader;
use rolcb::gridconnect::Decoder;
use rolcb::message::{Assembler, Message};

let capture = b":X10701285N050101012200;\n:X19100285N050101012200;\n:X195B4285N0501010122000010;\n";
let mut decoder = Decoder::new();
let mut asm = Assembler::new();
let mut log = Vec::new();
for frame in decoder.push(capture).into_iter().flatten() {
    match asm.accept(&frame) {
        Some(raw) => {
            let msg = Message::from_parts(raw.mti, raw.dest, raw.payload);
            log.push(format!("{:03X} {}", raw.src_alias, raw.mti.name()));
            let _ = msg;
        }
        None => {
            if let CanHeader::Amd { alias } = frame.header() {
                log.push(format!("{alias:03X} AMD"));
            }
        }
    }
}
assert_eq!(log, ["285 AMD", "285 InitComplete", "285 PCER"]);
```

## Memory and allocation

`CanFrame` is a fixed 16-byte `Copy` value. `Message` owns its payload in a
`Vec` only for variable-length messages (traction, SNIP, datagrams,
`Other`). The `Node` holds its outbox, pending messages (at most 512 while
offline), the alias table and the event registry in `alloc` collections;
the `Assembler` keeps at most 32 partial messages of at most 256 bytes. All
of it works with any global allocator, which is all that `no_std` users
need to supply (see [`no_std` and WASM](crate::guide::no_std)).
