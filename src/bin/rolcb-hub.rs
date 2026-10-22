//! `rolcb-hub`: a standalone OpenLCB GridConnect hub.
//!
//! ```text
//! rolcb-hub                                   # GridConnect TCP on :12021
//! rolcb-hub --serial /dev/ttyACM0             # also bridge a USB LCC buffer
//! rolcb-hub --upstream jmri.local:12021 -v    # join another hub, print traffic
//! ```

use std::sync::Arc;
use std::time::Instant;

use clap::Parser;
use rolcb::hub::{self, Hub};
use rolcb::message::{Assembler, Message};
use tokio::net::TcpListener;

/// Minimal `log` backend: every record from the hub library goes to stderr
/// with a `[rolcb-hub]` prefix, like the binary's own messages.
struct StderrLogger;

impl log::Log for StderrLogger {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        metadata.level() <= log::Level::Info
    }

    fn log(&self, record: &log::Record<'_>) {
        if self.enabled(record.metadata()) {
            eprintln!("[rolcb-hub] {}", record.args());
        }
    }

    fn flush(&self) {}
}

static LOGGER: StderrLogger = StderrLogger;

#[derive(Parser, Debug)]
#[command(
    name = "rolcb-hub",
    version,
    about = "OpenLCB GridConnect hub: relays CAN frames between TCP clients, other hubs and serial ports"
)]
struct Args {
    /// Address for GridConnect TCP clients (JMRI, other hubs).
    #[arg(long, default_value = "0.0.0.0:12021")]
    tcp: String,
    /// Disable the TCP listener.
    #[arg(long)]
    no_tcp: bool,
    /// Upstream GridConnect TCP hub(s) to connect to (host:port). Repeatable.
    #[arg(long)]
    upstream: Vec<String>,
    /// Serial GridConnect adapter(s), e.g. /dev/ttyACM0 or COM3. Repeatable.
    #[arg(long)]
    serial: Vec<String>,
    /// Serial baud rate.
    #[arg(long, default_value_t = 115200)]
    baud: u32,
    /// Print every frame and decoded message.
    #[arg(short, long)]
    verbose: bool,
}

#[tokio::main]
async fn main() -> std::io::Result<()> {
    let args = Args::parse();
    // Only fails if a logger is already installed, which cannot happen here.
    if log::set_logger(&LOGGER).is_ok() {
        log::set_max_level(log::LevelFilter::Info);
    }
    let hub = Hub::new();

    if !args.no_tcp {
        let l = TcpListener::bind(&args.tcp).await?;
        eprintln!("[rolcb-hub] GridConnect TCP on {}", l.local_addr()?);
        tokio::spawn(hub::serve_tcp(hub.clone(), l));
    }
    for addr in args.upstream {
        tokio::spawn(hub::uplink_tcp(hub.clone(), addr));
    }
    for path in args.serial {
        spawn_serial(&hub, path, args.baud);
    }
    if args.verbose {
        tokio::spawn(monitor(hub.clone()));
    }

    tokio::signal::ctrl_c().await?;
    eprintln!("[rolcb-hub] shutting down");
    Ok(())
}

#[cfg(feature = "serial")]
fn spawn_serial(hub: &Arc<Hub>, path: String, baud: u32) {
    tokio::spawn(hub::uplink_serial(hub.clone(), path, baud));
}

#[cfg(not(feature = "serial"))]
fn spawn_serial(_hub: &Arc<Hub>, path: String, _baud: u32) {
    eprintln!("[rolcb-hub] cannot open {path}: built without the `serial` feature");
}

async fn monitor(hub: Arc<Hub>) {
    let t0 = Instant::now();
    let mut port = hub.port();
    let mut asm = Assembler::new();
    while let Some(frame) = port.recv().await {
        let text = frame.to_string();
        let now_ms = t0.elapsed().as_millis() as u64;
        match asm.accept_at(&frame, now_ms) {
            Some(raw) => {
                let src = raw.src_alias;
                let msg = Message::from(raw);
                println!("{text:<30} {src:03X} {msg:?}");
            }
            None => println!("{text:<30} {:?}", frame.header()),
        }
    }
}
