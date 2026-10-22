//! An async GridConnect hub (feature `hub`).
//!
//! Every frame received on one [`Port`] is forwarded to every other port.
//! Ports can be TCP GridConnect clients (JMRI, other tools), WebSocket
//! clients (browser apps), uplinks to another hub, or serial adapters
//! (feature `serial`). Applications can also attach their own ports with
//! [`Hub::port`], e.g. to bridge a custom WebSocket server into the hub.
//!
//! # Semantics
//!
//! The hub behaves like a CAN bus segment for GridConnect links:
//!
//! * **Fan-out.** A frame sent by one port is delivered to every *other*
//!   attached port, in order.
//! * **No echo.** A port never receives its own frames. (On a real CAN bus
//!   a transmitter does not receive its own frame either, and OpenLCB alias
//!   logic depends on that: seeing "your" alias from someone else means a
//!   collision.)
//! * **No routing or filtering.** Frames are relayed verbatim; the hub does
//!   not parse messages or track aliases, so it is not an OpenLCB node and
//!   never appears on the network. Malformed GridConnect input is dropped.
//! * **Back-pressure by dropping.** Each port has a bounded queue (4096
//!   frames). A port that falls behind loses its oldest frames, like a
//!   congested CAN interface, instead of slowing down everybody else.
//!
//! ```text
//!   JMRI ──TCP:12021──┐                     ┌── browser (WebSocket :12022)
//!                     │      ┌───────┐      │
//!   other hub ◄─uplink┼─────►│  Hub  │◄─────┼── your app: hub.port()
//!                     │      └───────┘      │
//!   USB buffer ─serial┘                     └── rolcb-hub -v monitor
//! ```
//!
//! Outgoing frames are written as newline-terminated GridConnect text;
//! frames already queued are coalesced into a single write or WebSocket
//! message (up to 64 frames) to keep latency and syscall counts low.
//!
//! # Example
//!
//! ```no_run
//! use rolcb::hub::{self, Hub};
//! use tokio::net::TcpListener;
//!
//! # #[tokio::main] async fn main() -> std::io::Result<()> {
//! let hub = Hub::new();
//! tokio::spawn(hub::serve_tcp(hub.clone(), TcpListener::bind("0.0.0.0:12021").await?));
//! tokio::spawn(hub::serve_websocket(hub.clone(), TcpListener::bind("0.0.0.0:12022").await?));
//! tokio::spawn(hub::uplink_tcp(hub.clone(), "jmri.local:12021".to_string()));
//!
//! // Watch the traffic from inside the process.
//! let mut port = hub.port();
//! while let Some(frame) = port.recv().await {
//!     println!("{}", rolcb::gridconnect::format(&frame));
//! }
//! # Ok(()) }
//! ```
//!
//! See the [hub guide](crate::guide::hub) for the command-line tool and for
//! embedding the hub in an axum or other WebSocket server.

use std::io;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast::{self, error::RecvError, error::TryRecvError};
use tokio_tungstenite::tungstenite::Message as WsMessage;
use tokio_tungstenite::WebSocketStream;

use crate::can::CanFrame;
use crate::gridconnect::{self, Decoder};

const CHANNEL_DEPTH: usize = 4096;
const RECONNECT_DELAY: Duration = Duration::from_secs(2);
/// Upper bound on frames coalesced into one write / WebSocket message.
const MAX_BATCH: usize = 64;

#[derive(Debug, Clone, Copy)]
struct Packet {
    from: u64,
    frame: CanFrame,
}

/// The shared frame bus. Create with [`Hub::new`] and attach ports.
///
/// `Hub` is always used behind an [`Arc`] so that listener and uplink tasks
/// can share it. It holds no sockets itself: tasks such as [`serve_tcp`]
/// attach a [`Port`] per connection and drop it when the connection ends.
#[derive(Debug)]
pub struct Hub {
    tx: broadcast::Sender<Packet>,
    next_id: AtomicU64,
}

impl Hub {
    /// Create an empty hub with no ports.
    pub fn new() -> Arc<Hub> {
        let (tx, _) = broadcast::channel(CHANNEL_DEPTH);
        Arc::new(Hub {
            tx,
            next_id: AtomicU64::new(1),
        })
    }

    /// Attach a new port. Frames it sends go to every other port.
    ///
    /// The port only receives frames sent *after* it was created. Dropping
    /// the port detaches it.
    ///
    /// ```
    /// # #[tokio::main(flavor = "current_thread")] async fn main() {
    /// use rolcb::hub::Hub;
    ///
    /// let hub = Hub::new();
    /// let a = hub.port();
    /// let mut b = hub.port();
    /// let frame = rolcb::gridconnect::parse(":X10700285N;").unwrap();
    /// a.send(frame);
    /// assert_eq!(b.recv().await, Some(frame));
    /// # }
    /// ```
    pub fn port(&self) -> Port {
        Port {
            id: self.next_id.fetch_add(1, Ordering::Relaxed),
            tx: self.tx.clone(),
            rx: self.tx.subscribe(),
        }
    }

    /// Number of ports currently attached (including ones created with
    /// [`Hub::port`] by the application).
    pub fn port_count(&self) -> usize {
        self.tx.receiver_count()
    }
}

/// One attachment point on a [`Hub`].
///
/// A port is both a sender and a receiver. It is the building block for
/// custom transports: read frames from your connection and [`send`](Port::send)
/// them, and write every frame from [`recv`](Port::recv) back to it.
#[derive(Debug)]
pub struct Port {
    id: u64,
    tx: broadcast::Sender<Packet>,
    rx: broadcast::Receiver<Packet>,
}

impl Port {
    /// A process-unique identifier for this port (used for no-echo filtering
    /// and in log messages).
    pub fn id(&self) -> u64 {
        self.id
    }

    /// Send a frame to every other port.
    ///
    /// Never blocks. If no other port is attached the frame is discarded.
    pub fn send(&self, frame: CanFrame) {
        let _ = self.tx.send(Packet {
            from: self.id,
            frame,
        });
    }

    /// Wait for the next frame from another port. Returns `None` if the hub
    /// is gone. If this port falls too far behind, the oldest frames are
    /// dropped (like a congested CAN interface).
    pub async fn recv(&mut self) -> Option<CanFrame> {
        loop {
            match self.rx.recv().await {
                Ok(p) if p.from == self.id => continue,
                Ok(p) => return Some(p.frame),
                Err(RecvError::Lagged(n)) => {
                    log(format_args!("port {} lagged, dropped {n} frames", self.id))
                }
                Err(RecvError::Closed) => return None,
            }
        }
    }

    /// Return a frame if one is ready, without waiting.
    pub fn try_recv(&mut self) -> Option<CanFrame> {
        loop {
            match self.rx.try_recv() {
                Ok(p) if p.from == self.id => continue,
                Ok(p) => return Some(p.frame),
                Err(TryRecvError::Lagged(_)) => continue,
                Err(_) => return None,
            }
        }
    }

    /// Wait for a frame, then gather any others already queued, formatted as
    /// newline-separated GridConnect text.
    async fn recv_batch(&mut self) -> Option<String> {
        let first = self.recv().await?;
        let mut out = gridconnect::format(&first);
        out.push('\n');
        for _ in 1..MAX_BATCH {
            match self.try_recv() {
                Some(f) => {
                    out.push_str(&gridconnect::format(&f));
                    out.push('\n');
                }
                None => break,
            }
        }
        Some(out)
    }
}

pub(crate) fn log(args: core::fmt::Arguments<'_>) {
    eprintln!("[rolcb-hub] {args}");
}

/// Relay GridConnect text between a byte stream (TCP socket, serial port)
/// and the hub until the stream closes.
///
/// Attaches a new port for the lifetime of the call. Incoming bytes are
/// decoded with a [`Decoder`] (malformed frames are dropped); outgoing frames
/// are written as `\n`-terminated GridConnect lines. Returns `Ok(())` on EOF,
/// or the first I/O error.
pub async fn run_gridconnect_stream<S>(hub: &Hub, stream: S) -> io::Result<()>
where
    S: AsyncRead + AsyncWrite + Unpin,
{
    let mut port = hub.port();
    let (mut rd, mut wr) = tokio::io::split(stream);
    let mut decoder = Decoder::new();
    let mut buf = [0u8; 2048];
    loop {
        tokio::select! {
            n = rd.read(&mut buf) => {
                let n = n?;
                if n == 0 {
                    return Ok(());
                }
                for frame in decoder.push(&buf[..n]).into_iter().flatten() {
                    port.send(frame);
                }
            }
            batch = port.recv_batch() => match batch {
                Some(text) => wr.write_all(text.as_bytes()).await?,
                None => return Ok(()),
            },
        }
    }
}

/// Relay GridConnect text over an accepted WebSocket. Each WebSocket text
/// message may carry any number of frames.
///
/// Binary messages are decoded the same way as text. Outgoing frames are
/// sent as text messages containing one or more newline-separated frames.
/// Returns when the peer closes the socket.
pub async fn run_websocket<S>(hub: &Hub, ws: WebSocketStream<S>) -> io::Result<()>
where
    S: AsyncRead + AsyncWrite + Unpin,
{
    let mut port = hub.port();
    let (mut sink, mut stream) = ws.split();
    let mut decoder = Decoder::new();
    loop {
        tokio::select! {
            msg = stream.next() => match msg {
                Some(Ok(WsMessage::Text(t))) => {
                    for frame in decoder.push(t.as_bytes()).into_iter().flatten() {
                        port.send(frame);
                    }
                }
                Some(Ok(WsMessage::Binary(b))) => {
                    for frame in decoder.push(&b).into_iter().flatten() {
                        port.send(frame);
                    }
                }
                Some(Ok(WsMessage::Close(_))) | None => return Ok(()),
                Some(Ok(_)) => {}
                Some(Err(e)) => return Err(io::Error::other(e)),
            },
            batch = port.recv_batch() => match batch {
                Some(text) => sink.send(WsMessage::text(text)).await.map_err(io::Error::other)?,
                None => return Ok(()),
            },
        }
    }
}

/// Accept GridConnect TCP clients forever.
///
/// Spawns a task per client running [`run_gridconnect_stream`], with
/// `TCP_NODELAY` set. Port 12021 is the conventional GridConnect hub port
/// (JMRI's default). Returns only if `accept` fails.
pub async fn serve_tcp(hub: Arc<Hub>, listener: TcpListener) -> io::Result<()> {
    loop {
        let (sock, peer) = listener.accept().await?;
        let _ = sock.set_nodelay(true);
        let hub = hub.clone();
        tokio::spawn(async move {
            log(format_args!("tcp client {peer} connected"));
            let r = run_gridconnect_stream(&hub, sock).await;
            log(format_args!(
                "tcp client {peer} disconnected {}",
                err_suffix(&r)
            ));
        });
    }
}

/// Accept WebSocket clients forever (any path).
///
/// Performs the WebSocket handshake and runs [`run_websocket`] for each
/// client. Browser apps connect with `new WebSocket("ws://host:12022/")`
/// and exchange GridConnect text.
pub async fn serve_websocket(hub: Arc<Hub>, listener: TcpListener) -> io::Result<()> {
    loop {
        let (sock, peer) = listener.accept().await?;
        let _ = sock.set_nodelay(true);
        let hub = hub.clone();
        tokio::spawn(async move {
            let ws = match tokio_tungstenite::accept_async(sock).await {
                Ok(ws) => ws,
                Err(e) => {
                    log(format_args!("websocket handshake from {peer} failed: {e}"));
                    return;
                }
            };
            log(format_args!("websocket client {peer} connected"));
            let r = run_websocket(&hub, ws).await;
            log(format_args!(
                "websocket client {peer} disconnected {}",
                err_suffix(&r)
            ));
        });
    }
}

/// Keep a connection to an upstream GridConnect TCP hub, reconnecting forever.
///
/// `addr` is `host:port`, for example a JMRI "OpenLCB / LCC GridConnect
/// server" on port 12021 or another `rolcb-hub`. After a failure or
/// disconnect it waits two seconds and retries. Never returns.
pub async fn uplink_tcp(hub: Arc<Hub>, addr: String) {
    loop {
        match TcpStream::connect(&addr).await {
            Ok(sock) => {
                let _ = sock.set_nodelay(true);
                log(format_args!("uplink {addr} connected"));
                let r = run_gridconnect_stream(&hub, sock).await;
                log(format_args!("uplink {addr} lost {}", err_suffix(&r)));
            }
            Err(e) => log(format_args!("uplink {addr}: {e}")),
        }
        tokio::time::sleep(RECONNECT_DELAY).await;
    }
}

/// Keep a USB/serial GridConnect adapter attached, reopening it forever.
///
/// `path` is e.g. `/dev/ttyACM0` or `COM3`. Most USB-CAN adapters ignore the
/// baud rate (they are USB CDC devices), but 115200 is a safe default. If
/// the device is unplugged, the hub retries every two seconds. Never returns.
#[cfg(feature = "serial")]
#[cfg_attr(docsrs, doc(cfg(feature = "serial")))]
pub async fn uplink_serial(hub: Arc<Hub>, path: String, baud: u32) {
    use tokio_serial::SerialPortBuilderExt;
    loop {
        match tokio_serial::new(&path, baud).open_native_async() {
            Ok(port) => {
                log(format_args!("serial {path} opened at {baud} baud"));
                let r = run_gridconnect_stream(&hub, port).await;
                log(format_args!("serial {path} closed {}", err_suffix(&r)));
            }
            Err(e) => log(format_args!("serial {path}: {e}")),
        }
        tokio::time::sleep(RECONNECT_DELAY).await;
    }
}

fn err_suffix(r: &io::Result<()>) -> String {
    match r {
        Ok(()) => String::new(),
        Err(e) => format!("({e})"),
    }
}
