//! `rolcb-hub`: a standalone OpenLCB GridConnect hub.
//!
//! ```text
//! rolcb-hub                                   # TCP :12021 + WebSocket :12022
//! rolcb-hub --serial /dev/ttyACM0             # also bridge a USB LCC buffer
//! rolcb-hub --upstream jmri.local:12021 -v    # join another hub, print traffic
//! ```

use std::sync::Arc;

use clap::Parser;
use rolcb::hub::{self, Hub};
use rolcb::message::{Assembler, Message};
use tokio::net::TcpListener;

#[derive(Parser, Debug)]
#[command(
    version,
    about = "OpenLCB GridConnect hub: relays CAN frames between TCP, WebSocket and serial ports"
)]
struct Args {
    /// Address for GridConnect TCP clients (JMRI, other hubs).
    #[arg(long, default_value = "0.0.0.0:12021")]
    tcp: String,
    /// Disable the TCP listener.
    #[arg(long)]
    no_tcp: bool,
    /// Address for WebSocket clients (browser apps such as System4Dispatch).
    #[arg(long, default_value = "0.0.0.0:12022")]
    ws: String,
    /// Disable the WebSocket listener.
    #[arg(long)]
    no_ws: bool,
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
    let hub = Hub::new();

    if !args.no_tcp {
        let l = TcpListener::bind(&args.tcp).await?;
        eprintln!("[rolcb-hub] GridConnect TCP on {}", l.local_addr()?);
        tokio::spawn(hub::serve_tcp(hub.clone(), l));
    }
    if !args.no_ws {
        let l = TcpListener::bind(&args.ws).await?;
        eprintln!("[rolcb-hub] WebSocket on ws://{}", l.local_addr()?);
        tokio::spawn(hub::serve_websocket(hub.clone(), l));
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
    let mut port = hub.port();
    let mut asm = Assembler::new();
    while let Some(frame) = port.recv().await {
        let text = rolcb::gridconnect::format(&frame);
        match asm.accept(&frame) {
            Some(raw) => {
                let msg = Message::from_parts(raw.mti, raw.dest, raw.payload);
                println!("{text:<30} {:03X} {msg:?}", raw.src_alias);
            }
            None => println!("{text:<30} {:?}", frame.header()),
        }
    }
}
