//! The hub relays frames between TCP clients, never echoing a frame back to
//! its sender.

use std::time::Duration;

use rolcb::hub::{self, Hub};
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::time::timeout;

const T: Duration = Duration::from_secs(2);

#[tokio::test]
async fn relays_between_tcp_clients() {
    let hub = Hub::new();
    let tcp = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = tcp.local_addr().unwrap();
    tokio::spawn(hub::serve_tcp(hub.clone(), tcp));

    let mut a = BufReader::new(TcpStream::connect(addr).await.unwrap());
    let mut b = BufReader::new(TcpStream::connect(addr).await.unwrap());
    let mut c = BufReader::new(TcpStream::connect(addr).await.unwrap());
    // Wait until all three clients are attached.
    timeout(T, async {
        while hub.port_count() < 3 {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();

    // A -> B and C; sent in two pieces with lowercase hex.
    a.get_mut().write_all(b":x195b4123n05010101").await.unwrap();
    a.get_mut().write_all(b"22000010;").await.unwrap();
    for client in [&mut b, &mut c] {
        let mut line = String::new();
        timeout(T, client.read_line(&mut line))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(line.trim(), ":X195B4123N0501010122000010;");
    }

    // C -> A and B, two frames in one write.
    c.get_mut()
        .write_all(b":X19490ABCN;:X19170ABCN050101012200;")
        .await
        .unwrap();
    for client in [&mut a, &mut b] {
        let mut l1 = String::new();
        let mut l2 = String::new();
        timeout(T, client.read_line(&mut l1))
            .await
            .unwrap()
            .unwrap();
        timeout(T, client.read_line(&mut l2))
            .await
            .unwrap()
            .unwrap();
        assert_eq!(l1.trim(), ":X19490ABCN;");
        assert_eq!(l2.trim(), ":X19170ABCN050101012200;");
    }

    // A's first line above was C's frame, so A never got its own; C never
    // gets its own frames back either.
    let mut line = String::new();
    assert!(timeout(Duration::from_millis(200), c.read_line(&mut line))
        .await
        .is_err());
}

#[tokio::test]
async fn library_ports() {
    let hub = Hub::new();
    let p1 = hub.port();
    let mut p2 = hub.port();
    let f = rolcb::gridconnect::parse(":X10700285N;").unwrap();
    p1.send(f);
    assert_eq!(timeout(T, p2.recv()).await.unwrap(), Some(f));
}

fn frame(i: u32) -> rolcb::CanFrame {
    rolcb::CanFrame::new(0x195B_4000 | (i & 0xFFF), &i.to_be_bytes()).unwrap()
}

#[tokio::test]
async fn lagging_port_recovers() {
    let hub = Hub::new();
    let tx = hub.port();
    let mut slow = hub.port();
    // Overrun the shared ring.
    let n = hub::CHANNEL_DEPTH as u32 + 500;
    for i in 0..n {
        tx.send(frame(i));
    }
    // The slow port skips what it missed and resumes with the oldest frame
    // still in the ring, then reads the rest in order.
    let first = timeout(T, slow.recv()).await.unwrap().unwrap();
    assert_eq!(first, frame(n - hub::CHANNEL_DEPTH as u32));
    let mut count = 1;
    while let Some(f) = slow.try_recv() {
        assert_eq!(f, frame(n - hub::CHANNEL_DEPTH as u32 + count));
        count += 1;
    }
    assert_eq!(count, hub::CHANNEL_DEPTH as u32);
    // And keeps working afterwards.
    tx.send(frame(1));
    assert_eq!(timeout(T, slow.recv()).await.unwrap(), Some(frame(1)));

    // try_recv handles lag the same way.
    for i in 0..n {
        tx.send(frame(i));
    }
    assert_eq!(slow.try_recv(), Some(frame(n - hub::CHANNEL_DEPTH as u32)));
}

#[tokio::test]
async fn port_count_falls_on_drop() {
    let hub = Hub::new();
    assert_eq!(hub.port_count(), 0);
    let a = hub.port();
    let b = hub.port();
    assert_eq!(hub.port_count(), 2);
    assert_ne!(a.id(), b.id());
    drop(a);
    assert_eq!(hub.port_count(), 1);
    drop(b);
    assert_eq!(hub.port_count(), 0);

    // A stream relay holds a port only while the stream is open.
    let (client, server) = tokio::io::duplex(1024);
    let h = hub.clone();
    let task = tokio::spawn(async move { hub::run_gridconnect_stream(&h, server).await });
    timeout(T, async {
        while hub.port_count() < 1 {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await
    .unwrap();
    drop(client);
    timeout(T, task).await.unwrap().unwrap().unwrap();
    assert_eq!(hub.port_count(), 0);
}

#[tokio::test]
async fn malformed_input_is_dropped() {
    let hub = Hub::new();
    let mut watch = hub.port();
    let (mut client, server) = tokio::io::duplex(1024);
    let h = hub.clone();
    tokio::spawn(async move { hub::run_gridconnect_stream(&h, server).await });
    client
        .write_all(b"garbage\n:X1G;\n:S123N;\n:X10700285N0;\n:X10700:X10701285N050101012200;\n")
        .await
        .unwrap();
    let got = timeout(T, watch.recv()).await.unwrap().unwrap();
    assert_eq!(got.to_string(), ":X10701285N050101012200;");
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert_eq!(watch.try_recv(), None);
}

/// A peer that never reads must not stop the hub from reading what it sends.
#[tokio::test]
async fn blocked_writes_do_not_block_reads() {
    let hub = Hub::new();
    let mut watch = hub.port();
    // A tiny pipe: the hub's writes to `client` block after 64 bytes.
    let (mut client, server) = tokio::io::duplex(64);
    let h = hub.clone();
    tokio::spawn(async move { hub::run_gridconnect_stream(&h, server).await });
    timeout(T, async {
        while hub.port_count() < 2 {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await
    .unwrap();

    // Flood the client, which never reads.
    for i in 0..200 {
        watch.send(frame(i));
    }
    tokio::time::sleep(Duration::from_millis(50)).await;

    // The client still gets its frames into the hub.
    client.write_all(b":X19490ABCN;\n").await.unwrap();
    let got = timeout(T, watch.recv()).await.unwrap().unwrap();
    assert_eq!(got.to_string(), ":X19490ABCN;");
}

#[tokio::test]
async fn library_port_try_recv() {
    let hub = Hub::new();
    let a = hub.port();
    let mut b = hub.port();
    assert_eq!(b.try_recv(), None);
    a.send(frame(7));
    assert_eq!(b.try_recv(), Some(frame(7)));
    // No echo.
    let mut a = a;
    assert_eq!(a.try_recv(), None);
}
