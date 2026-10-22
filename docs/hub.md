# Running and embedding the hub

The `hub` module (feature `hub`) is an async GridConnect hub built on
tokio. The `rolcb-hub` binary (feature `cli`) wraps it in a command-line
program. Use it to connect browser apps, JMRI, USB-CAN adapters and other
hubs into one virtual CAN segment.

## What a hub does

```text
  JMRI ──── TCP :12021 ────┐                      ┌──── WebSocket :12022 ──── browser app
                           │                      │
  other hub ── uplink ─────┤      ┌────────┐      ├──── WebSocket :12022 ──── another tab
                           ├─────►│  Hub   │◄─────┤
  USB buffer ── serial ────┤      └────────┘      ├──── hub.port() ─────────── your Rust code
                           │                      │
  rolcb-hub -v ── monitor ─┘                      └──── TCP :12021 ─────────── a GridConnect tool
```

* **Fan-out:** every frame received from one port is sent to all other
  ports.
* **No echo:** a frame is never sent back to the port it came from, exactly
  like a CAN controller does not receive its own transmissions. OpenLCB
  alias collision detection depends on this.
* **Transparent:** the hub does not parse messages, track aliases or
  appear as a node. It relays frames verbatim (re-encoded as canonical
  uppercase GridConnect). Malformed input is dropped.
* **Lossy under overload:** each port has a 4096-frame queue. A port that
  can't keep up loses its oldest frames rather than stalling the others.
* **Batched writes:** frames queued for a port are coalesced into a single
  TCP write or WebSocket message (up to 64 frames), newline-separated.

## The `rolcb-hub` command

```sh
cargo install --path . --features cli,serial   # or: cargo install rolcb --features cli,serial
```

| Flag | Default | Meaning |
|---|---|---|
| `--tcp ADDR` | `0.0.0.0:12021` | Listen for GridConnect TCP clients (JMRI, tools, other hubs) |
| `--no-tcp` | | Don't open the TCP listener |
| `--ws ADDR` | `0.0.0.0:12022` | Listen for WebSocket clients (browser apps); any URL path is accepted |
| `--no-ws` | | Don't open the WebSocket listener |
| `--upstream HOST:PORT` | | Connect to another GridConnect TCP hub; repeatable; reconnects every 2 s |
| `--serial PATH` | | Open a serial GridConnect adapter (`/dev/ttyACM0`, `COM3`); repeatable; needs the `serial` feature |
| `--baud N` | `115200` | Serial baud rate (ignored by most USB CDC adapters) |
| `-v`, `--verbose` | | Print every frame and its decoded message to stdout |
| `-V`, `--version` / `-h`, `--help` | | Version / help |

Typical setups:

```sh
# A standalone hub on a Raspberry Pi with an LCC USB buffer attached.
rolcb-hub --serial /dev/ttyACM0

# Browser apps only, joined to an existing JMRI hub, with a traffic log.
rolcb-hub --no-tcp --upstream jmri.local:12021 -v

# Two hubs bridging two buildings.
rolcb-hub --upstream other-building.local:12021
```

Logs go to stderr with a `[rolcb-hub]` prefix (connections, disconnects,
lagging ports); `-v` output goes to stdout:

```text
:X19100285N050101012200;       285 InitializationComplete { node_id: NodeId(05.01.01.01.22.00), simple: false }
:X10701285N050101012200;       Amd { alias: 645 }
```

Running it as a systemd service:

```ini
[Unit]
Description=OpenLCB GridConnect hub
After=network-online.target

[Service]
ExecStart=/usr/local/bin/rolcb-hub --serial /dev/ttyACM0
Restart=always

[Install]
WantedBy=multi-user.target
```

## Connecting JMRI

**JMRI as a client of `rolcb-hub`.** In JMRI, open *Edit → Preferences →
Connections*, add a connection with:

* System manufacturer: **LCC / OpenLCB**
* System connection: **CAN via GridConnect Network Interface**
* IP address / host name: the machine running `rolcb-hub`
* TCP/UDP port: **12021**
* Connection protocol: **OpenLCB CAN**

**`rolcb-hub` as a client of JMRI.** If JMRI already runs its own LCC hub
(*LCC → Start LCC Hub*), point `rolcb-hub` at it, and disable the local TCP
listener if both run on the same machine:

```sh
rolcb-hub --no-tcp --upstream localhost:12021
```

Either way, browser apps connected to the WebSocket port and JMRI see the
same traffic.

## Embedding the hub in your program

Everything the binary does is available as library functions:
[`Hub::new`](crate::hub::Hub::new), [`serve_tcp`](crate::hub::serve_tcp),
[`serve_websocket`](crate::hub::serve_websocket),
[`uplink_tcp`](crate::hub::uplink_tcp), `uplink_serial` (feature
`serial`), and the lower-level
[`run_gridconnect_stream`](crate::hub::run_gridconnect_stream) and
[`run_websocket`](crate::hub::run_websocket) for connections you accept
yourself.

```rust,no_run
use rolcb::hub::{self, Hub};
use tokio::net::TcpListener;

#[tokio::main]
async fn main() -> std::io::Result<()> {
    let hub = Hub::new();
    tokio::spawn(hub::serve_tcp(hub.clone(), TcpListener::bind("0.0.0.0:12021").await?));
    tokio::spawn(hub::serve_websocket(hub.clone(), TcpListener::bind("0.0.0.0:12022").await?));
    tokio::spawn(hub::uplink_tcp(hub.clone(), "jmri.local:12021".into()));
    tokio::signal::ctrl_c().await
}
```

### Ports: plugging in your own transport

A [`Port`](crate::hub::Port) is one attachment point:
[`send`](crate::hub::Port::send) a frame to everybody else,
[`recv`](crate::hub::Port::recv) the next frame from anybody else. Anything
that can carry frames can become a port. This function bridges a pair of
text channels (for example the two halves of a WebSocket from your web
framework) into the hub:

```rust,no_run
use rolcb::gridconnect::{self, Decoder};
use rolcb::hub::Port;
use tokio::sync::mpsc;

/// Relay GridConnect text between a hub port and a text transport.
/// `incoming` yields text received from the client; `outgoing` takes text
/// to send to it. Returns when either side closes.
pub async fn bridge_text(
    mut port: Port,
    mut incoming: mpsc::Receiver<String>,
    outgoing: mpsc::Sender<String>,
) {
    let mut decoder = Decoder::new();
    loop {
        tokio::select! {
            text = incoming.recv() => match text {
                Some(text) => {
                    for frame in decoder.push(text.as_bytes()).into_iter().flatten() {
                        port.send(frame);
                    }
                }
                None => return,
            },
            frame = port.recv() => match frame {
                Some(frame) => {
                    if outgoing.send(gridconnect::format(&frame)).await.is_err() {
                        return;
                    }
                }
                None => return,
            },
        }
    }
}
# fn main() {}
```

### Inside an axum server

If your application already has a web server, serve the WebSocket from it
instead of running a second listener. With axum (not a dependency of this
crate, so this snippet is not compiled):

```rust,ignore
use std::sync::Arc;

use axum::extract::ws::{Message as WsMessage, WebSocket, WebSocketUpgrade};
use axum::extract::State;
use axum::response::Response;
use axum::{routing::get, Router};
use futures_util::{SinkExt, StreamExt};
use rolcb::gridconnect::{self, Decoder};
use rolcb::hub::Hub;

async fn lcc_ws(ws: WebSocketUpgrade, State(hub): State<Arc<Hub>>) -> Response {
    ws.on_upgrade(move |socket| relay(socket, hub))
}

async fn relay(socket: WebSocket, hub: Arc<Hub>) {
    let mut port = hub.port(); // attach; dropped (detached) when we return
    let (mut tx, mut rx) = socket.split();
    let mut decoder = Decoder::new();
    loop {
        tokio::select! {
            msg = rx.next() => match msg {
                Some(Ok(WsMessage::Text(text))) => {
                    for frame in decoder.push(text.as_bytes()).into_iter().flatten() {
                        port.send(frame); // fan out to every other port, never back to us
                    }
                }
                Some(Ok(WsMessage::Close(_))) | None | Some(Err(_)) => return,
                Some(Ok(_)) => {}
            },
            frame = port.recv() => {
                let Some(frame) = frame else { return };
                let line = gridconnect::format(&frame) + "\n";
                if tx.send(WsMessage::Text(line.into())).await.is_err() {
                    return;
                }
            }
        }
    }
}

#[tokio::main]
async fn main() {
    let hub = Hub::new();
    // Keep JMRI and other GridConnect tools working alongside the web app.
    let tcp = tokio::net::TcpListener::bind("0.0.0.0:12021").await.unwrap();
    tokio::spawn(rolcb::hub::serve_tcp(hub.clone(), tcp));

    let app = Router::new().route("/lcc", get(lcc_ws)).with_state(hub);
    let web = tokio::net::TcpListener::bind("0.0.0.0:8080").await.unwrap();
    axum::serve(web, app).await.unwrap();
}
```

### An OpenLCB node inside the hub process

A server-side node (a logic controller, a virtual train, a fast clock)
can sit directly on a port, with no GridConnect text involved. This is the
[architecture chapter](crate::guide::architecture)'s driver loop with a `Port` in
place of a socket:

```rust,no_run
use std::time::Duration;

use rolcb::hub::Hub;
use rolcb::{EventId, EventState, Message, Node, NodeInfo};
use tokio::time::{interval, Instant};

#[tokio::main]
async fn main() {
    let hub = Hub::new();
    // ... start serve_tcp / serve_websocket / uplinks here ...

    let mut port = hub.port();
    let mut node = Node::new("05.01.01.01.22.98".parse().unwrap(), NodeInfo::default());
    let watched = EventId(0x0501_0101_2200_0010);
    node.add_consumer(watched, EventState::Unknown);

    let t0 = Instant::now();
    let now = || t0.elapsed().as_millis() as u64;
    let mut tick = interval(Duration::from_millis(25));
    node.start(now());
    loop {
        tokio::select! {
            frame = port.recv() => {
                let Some(frame) = frame else { break };
                if let Some(rx) = node.receive(&frame, now()) {
                    if rx.msg == Message::Pcer(watched) {
                        // React by producing another event.
                        node.send(Message::Pcer(EventId(0x0501_0101_2298_0001)));
                    }
                }
            }
            _ = tick.tick() => node.poll(now()),
        }
        for frame in node.take_outgoing() {
            port.send(frame);
        }
    }
}
```

Because the hub never echoes, the node does not see its own frames, just
as on a real CAN bus.

## Browser clients

A browser connects with a plain WebSocket and exchanges GridConnect text:

```text
const ws = new WebSocket("ws://raspberrypi.local:12022/");
ws.onmessage = (e) => e.data.split(/\s+/).filter(Boolean).forEach(handleFrame);
ws.send(":X19490ABCN;\n");   // any number of frames per message
```

With `rolcb` compiled to WASM, pass `e.data` straight to a
[`Decoder`](crate::gridconnect::Decoder); see
[`no_std` and WASM](crate::guide::no_std).
