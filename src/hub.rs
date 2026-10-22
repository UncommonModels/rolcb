//! An async GridConnect hub (feature `hub`).
//!
//! Every frame received on one [`Port`] is forwarded to every other port.
//! Ports can be TCP GridConnect clients (JMRI, desktop apps, other tools),
//! uplinks to another hub, or serial adapters (feature `serial`).
//! Applications can also attach their own ports with [`Hub::port`], e.g. to
//! run an OpenLCB node inside the same process as the hub.
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
//! * **Back-pressure by dropping.** All ports share **one broadcast ring of
//!   4096 frames** (a `tokio::sync::broadcast` channel); each port reads it
//!   at its own pace. A port that falls more than 4096 frames behind skips
//!   the frames it missed and carries on from the oldest one still in the
//!   ring, like a congested CAN interface, instead of slowing down everybody
//!   else. The skip is reported through the `log` feature.
//! * **Reads never wait for writes.** Each stream is read and written
//!   concurrently, so a peer that stops reading cannot stop the hub from
//!   draining what that peer sends.
//!
//! # Logging
//!
//! The library prints nothing by default. With the `log` feature,
//! connections, disconnections, accept errors and lagging ports are
//! reported through the [`log`](https://docs.rs/log) crate (target
//! `rolcb::hub`). The `rolcb-hub` binary enables it and prints to stderr.
//!
//! ```text
//!   JMRI ──TCP:12021──┐                     ┌── desktop app (TCP :12021)
//!                     │      ┌───────┐      │
//!   other hub ◄─uplink┼─────►│  Hub  │◄─────┼── your app: hub.port()
//!                     │      └───────┘      │
//!   USB buffer ─serial┘                     └── rolcb-hub -v monitor
//! ```
//!
//! Outgoing frames are written as newline-terminated GridConnect text;
//! frames already queued are coalesced into a single write (up to 64
//! frames) to keep latency and syscall counts low.
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
//! embedding the hub in your own program.

use std::io;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast::{self, error::RecvError, error::TryRecvError};

use crate::can::CanFrame;
use crate::gridconnect::{self, Decoder};

/// Capacity of the broadcast ring shared by all ports, in frames.
pub const CHANNEL_DEPTH: usize = 4096;
const RECONNECT_DELAY: Duration = Duration::from_secs(2);
const ACCEPT_BACKOFF: Duration = Duration::from_millis(100);
/// Upper bound on frames coalesced into one write.
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
    #[must_use]
    #[allow(clippy::new_ret_no_self)]
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
    #[must_use]
    pub fn port(&self) -> Port {
        Port {
            id: self.next_id.fetch_add(1, Ordering::Relaxed),
            tx: self.tx.clone(),
            rx: self.tx.subscribe(),
        }
    }

    /// Number of ports currently attached (including ones created with
    /// [`Hub::port`] by the application). Falls as ports are dropped.
    #[must_use]
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
    #[must_use]
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
    /// is gone.
    ///
    /// If this port fell more than [`CHANNEL_DEPTH`] frames behind, the
    /// frames it missed are skipped (and logged with the `log` feature) and
    /// reception continues with the oldest frame still in the ring.
    pub async fn recv(&mut self) -> Option<CanFrame> {
        loop {
            match self.rx.recv().await {
                Ok(p) if p.from == self.id => continue,
                Ok(p) => return Some(p.frame),
                Err(RecvError::Lagged(n)) => self.lagged(n),
                Err(RecvError::Closed) => return None,
            }
        }
    }

    /// Return a frame if one is ready, without waiting. Lag is handled
    /// exactly as in [`Port::recv`].
    pub fn try_recv(&mut self) -> Option<CanFrame> {
        loop {
            match self.rx.try_recv() {
                Ok(p) if p.from == self.id => continue,
                Ok(p) => return Some(p.frame),
                Err(TryRecvError::Lagged(n)) => self.lagged(n),
                Err(TryRecvError::Empty | TryRecvError::Closed) => return None,
            }
        }
    }

    fn lagged(&self, n: u64) {
        log_warn(format_args!("port {} lagged, dropped {n} frames", self.id));
    }

    /// Wait for a frame, then gather any others already queued, formatted as
    /// newline-separated GridConnect text.
    async fn recv_batch(&mut self) -> Option<String> {
        let first = self.recv().await?;
        let mut out = String::with_capacity(32);
        push_line(&mut out, &first);
        for _ in 1..MAX_BATCH {
            match self.try_recv() {
                Some(f) => push_line(&mut out, &f),
                None => break,
            }
        }
        Some(out)
    }
}

fn push_line(out: &mut String, frame: &CanFrame) {
    // Writing into a String cannot fail.
    let _ = gridconnect::write(frame, out);
    out.push('\n');
}

#[cfg(feature = "log")]
fn log_info(args: core::fmt::Arguments<'_>) {
    log::info!(target: "rolcb::hub", "{args}");
}

#[cfg(feature = "log")]
fn log_warn(args: core::fmt::Arguments<'_>) {
    log::warn!(target: "rolcb::hub", "{args}");
}

#[cfg(not(feature = "log"))]
fn log_info(_args: core::fmt::Arguments<'_>) {}

#[cfg(not(feature = "log"))]
fn log_warn(_args: core::fmt::Arguments<'_>) {}

/// Relay GridConnect text between a byte stream (TCP socket, serial port)
/// and the hub until the stream closes.
///
/// Attaches a new port for the lifetime of the call. Incoming bytes are
/// decoded with a [`Decoder`] (malformed frames are dropped); outgoing frames
/// are written as `\n`-terminated GridConnect lines.
///
/// Reading and writing run concurrently: while a write to a slow peer is
/// blocked, frames the peer sends are still read and forwarded. Frames for a
/// peer that stays blocked pile up in the shared ring and are eventually
/// skipped for that port (see the [module docs](self#semantics)).
///
/// # Errors
///
/// Returns `Ok(())` when the peer closes the stream (EOF) or the hub is
/// dropped, and the first I/O error from reading or writing otherwise.
pub async fn run_gridconnect_stream<S>(hub: &Hub, stream: S) -> io::Result<()>
where
    S: AsyncRead + AsyncWrite + Unpin,
{
    let mut port = hub.port();
    let (id, tx) = (port.id, port.tx.clone());
    let (mut rd, mut wr) = tokio::io::split(stream);

    let reader = async move {
        let mut decoder = Decoder::new();
        let mut buf = [0u8; 2048];
        loop {
            let n = rd.read(&mut buf).await?;
            if n == 0 {
                return Ok::<(), io::Error>(());
            }
            decoder.push_with(&buf[..n], |r| {
                if let Ok(frame) = r {
                    let _ = tx.send(Packet { from: id, frame });
                }
            });
        }
    };
    let writer = async move {
        while let Some(text) = port.recv_batch().await {
            wr.write_all(text.as_bytes()).await?;
        }
        Ok::<(), io::Error>(())
    };
    // Both halves are polled by the same task; a pending write does not stop
    // the reader from being polled. Whichever half finishes first ends the
    // relay (EOF, the hub going away, or an I/O error).
    tokio::select! {
        r = reader => r,
        w = writer => w,
    }
}

/// Accept GridConnect TCP clients forever.
///
/// Spawns a task per client running [`run_gridconnect_stream`], with
/// `TCP_NODELAY` set. Port 12021 is the conventional GridConnect hub port
/// (JMRI's default).
///
/// A failed `accept` (for example "too many open files") is logged, followed
/// by a 100 ms pause, and the loop continues; one bad moment does not take
/// the listener down.
///
/// # Errors
///
/// None in practice: the function only returns if the runtime shuts it
/// down. The `io::Result` return type is kept so it composes with `?` and
/// `tokio::spawn` the same way as before.
pub async fn serve_tcp(hub: Arc<Hub>, listener: TcpListener) -> io::Result<()> {
    loop {
        let (sock, peer) = match listener.accept().await {
            Ok(conn) => conn,
            Err(e) => {
                log_warn(format_args!("tcp accept failed: {e}"));
                tokio::time::sleep(ACCEPT_BACKOFF).await;
                continue;
            }
        };
        let _ = sock.set_nodelay(true);
        let hub = hub.clone();
        tokio::spawn(async move {
            log_info(format_args!("tcp client {peer} connected"));
            let r = run_gridconnect_stream(&hub, sock).await;
            log_info(format_args!(
                "tcp client {peer} disconnected{}",
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
                log_info(format_args!("uplink {addr} connected"));
                let r = run_gridconnect_stream(&hub, sock).await;
                log_info(format_args!("uplink {addr} lost{}", err_suffix(&r)));
            }
            Err(e) => log_warn(format_args!("uplink {addr}: {e}")),
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
pub async fn uplink_serial(hub: Arc<Hub>, path: String, baud: u32) {
    use tokio_serial::SerialPortBuilderExt;
    loop {
        match tokio_serial::new(&path, baud).open_native_async() {
            Ok(port) => {
                log_info(format_args!("serial {path} opened at {baud} baud"));
                let r = run_gridconnect_stream(&hub, port).await;
                log_info(format_args!("serial {path} closed{}", err_suffix(&r)));
            }
            Err(e) => log_warn(format_args!("serial {path}: {e}")),
        }
        tokio::time::sleep(RECONNECT_DELAY).await;
    }
}

fn err_suffix(r: &io::Result<()>) -> String {
    match r {
        Ok(()) => String::new(),
        Err(e) => format!(" ({e})"),
    }
}
