# Running and embedding the hub

The `hub` module (feature `hub`) is an async GridConnect hub built on
tokio. The `rolcb-hub` binary (feature `cli`) wraps it in a command-line
program. Use it to connect JMRI, desktop apps, USB-CAN adapters and other
hubs into one virtual CAN segment.

## What a hub does

```text
  JMRI ──── TCP :12021 ────┐                      ┌──── TCP :12021 ─────────── desktop app
                           │                      │
  other hub ── uplink ─────┤      ┌────────┐      ├──── TCP :12021 ─────────── another desktop app
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
* **Lossy under overload:** all ports share one broadcast ring of 4096
  frames, and each port reads it at its own pace. A port that falls more
  than 4096 frames behind skips the frames it missed (and resumes with the
  oldest one still in the ring) rather than stalling the others.
* **Reads never wait for writes:** each connection is read and written
  concurrently, so a peer that stops reading cannot stop the hub from
  forwarding what that peer sends.
* **Batched writes:** frames queued for a port are coalesced into a single
  TCP or serial write (up to 64 frames), newline-separated.

## The `rolcb-hub` command

Prebuilt binaries are attached to each [GitHub release](https://github.com/UncommonModels/rolcb/releases):

| Download | Runs on |
|---|---|
| `rolcb-hub-…-x86_64-unknown-linux-musl.tar.gz` | Linux PCs (any distro) |
| `rolcb-hub-…-aarch64-unknown-linux-musl.tar.gz` | Raspberry Pi 3/4/5 with 64-bit OS, other ARM64 Linux |
| `rolcb-hub-…-armv7-unknown-linux-musleabihf.tar.gz` | Raspberry Pi with 32-bit Raspberry Pi OS |
| `rolcb-hub-…-aarch64-apple-darwin.tar.gz` | Macs with Apple silicon |
| `rolcb-hub-…-x86_64-apple-darwin.tar.gz` | Intel Macs |
| `rolcb-hub-…-x86_64-pc-windows-msvc.zip` | Windows |

The Linux builds are static, so they need nothing installed. Unpack and run:

```sh
tar xzf rolcb-hub-*-aarch64-unknown-linux-musl.tar.gz
./rolcb-hub-*/rolcb-hub --serial /dev/ttyACM0
```

On macOS, clear the download quarantine first: `xattr -d com.apple.quarantine rolcb-hub`. `SHA256SUMS` lists the checksums.

Or build it yourself (the binary is part of the `rolcb` crate, behind the `cli` feature):

```sh
cargo install --path . --features cli,serial   # or: cargo install rolcb --features cli,serial
```

| Flag | Default | Meaning |
|---|---|---|
| `--tcp ADDR` | `0.0.0.0:12021` | Listen for GridConnect TCP clients (JMRI, tools, other hubs) |
| `--no-tcp` | | Don't open the TCP listener |
| `--upstream HOST:PORT` | | Connect to another GridConnect TCP hub; repeatable; reconnects every 2 s |
| `--serial PATH` | | Open a serial GridConnect adapter (`/dev/ttyACM0`, `COM3`); repeatable; needs the `serial` feature |
| `--baud N` | `115200` | Serial baud rate (ignored by most USB CDC adapters) |
| `-v`, `--verbose` | | Print every frame and its decoded message to stdout |
| `-V`, `--version` / `-h`, `--help` | | Version / help |

Typical setups:

```sh
# A standalone hub on a Raspberry Pi with an LCC USB buffer attached.
rolcb-hub --serial /dev/ttyACM0

# A traffic log of an existing JMRI hub, with no local listener.
rolcb-hub --no-tcp --upstream jmri.local:12021 -v

# Two hubs bridging two buildings.
rolcb-hub --upstream other-building.local:12021
```

Logs go to stderr with a `[rolcb-hub]` prefix (connections, disconnects,
accept errors, lagging ports); `-v` output goes to stdout. When you embed
the `hub` module the library prints nothing; enable the `log` feature to
receive the same messages through the [`log`](https://docs.rs/log) crate
(target `rolcb::hub`) with whatever logger your program installs.

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

Either way, desktop apps connected to `rolcb-hub`, serial adapters and
JMRI see the same traffic.

## Embedding the hub in your program

Everything the binary does is available as library functions:
[`Hub::new`](crate::hub::Hub::new), [`serve_tcp`](crate::hub::serve_tcp),
[`uplink_tcp`](crate::hub::uplink_tcp), `uplink_serial` (feature
`serial`), and the lower-level
[`run_gridconnect_stream`](crate::hub::run_gridconnect_stream) for
connections you accept yourself.

```rust,no_run
use rolcb::hub::{self, Hub};
use tokio::net::TcpListener;

#[tokio::main]
async fn main() -> std::io::Result<()> {
    let hub = Hub::new();
    tokio::spawn(hub::serve_tcp(hub.clone(), TcpListener::bind("0.0.0.0:12021").await?));
    tokio::spawn(hub::uplink_tcp(hub.clone(), "jmri.local:12021".into()));
    tokio::signal::ctrl_c().await
}
```

### Ports: plugging in your own transport

A [`Port`](crate::hub::Port) is one attachment point:
[`send`](crate::hub::Port::send) a frame to everybody else,
[`recv`](crate::hub::Port::recv) the next frame from anybody else. Anything
that can carry frames can become a port. This function bridges a pair of
text channels (for example the two directions of an IPC pipe or a message
bus in your application) into the hub:

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

### Inside a desktop application

A GUI application (a dispatch panel, a throttle) can own the hub instead of
running `rolcb-hub` next to it. Run the hub on a background tokio runtime and
give the UI thread its own port. [`Port::send`](crate::hub::Port::send) and
[`Port::try_recv`](crate::hub::Port::try_recv) never block, so they can be
called from the UI's frame or timer callback:

```rust,no_run
use rolcb::hub::{self, Hub};

fn main() -> std::io::Result<()> {
    let hub = Hub::new();
    let mut port = hub.port(); // the UI's attachment point

    // The hub's tasks live on a background runtime.
    let rt = tokio::runtime::Runtime::new()?;
    rt.spawn(hub::uplink_tcp(hub.clone(), "jmri.local:12021".into()));
    // With the `serial` feature, attach a USB-CAN adapter instead:
    // rt.spawn(hub::uplink_serial(hub.clone(), "/dev/ttyACM0".into(), 115_200));

    // Stand-in for the GUI event loop.
    loop {
        while let Some(frame) = port.try_recv() {
            // decode and update the UI
            let _ = frame;
        }
        // port.send(frame) for anything the UI wants to transmit
        std::thread::sleep(std::time::Duration::from_millis(16));
    }
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
    // ... start serve_tcp / uplinks here ...

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
