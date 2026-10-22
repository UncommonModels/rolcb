//! The hub relays frames between TCP and WebSocket clients, never echoing
//! a frame back to its sender.

use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use rolcb::hub::{self, Hub};
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::time::timeout;
use tokio_tungstenite::tungstenite::Message as WsMessage;

const T: Duration = Duration::from_secs(2);

#[tokio::test]
async fn relays_between_tcp_and_websocket() {
    let hub = Hub::new();
    let tcp = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let ws = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let tcp_addr = tcp.local_addr().unwrap();
    let ws_addr = ws.local_addr().unwrap();
    tokio::spawn(hub::serve_tcp(hub.clone(), tcp));
    tokio::spawn(hub::serve_websocket(hub.clone(), ws));

    let mut a = BufReader::new(TcpStream::connect(tcp_addr).await.unwrap());
    let mut b = BufReader::new(TcpStream::connect(tcp_addr).await.unwrap());
    let (mut w, _) = tokio_tungstenite::connect_async(format!("ws://{ws_addr}/"))
        .await
        .unwrap();
    // Wait until all three clients are attached.
    timeout(T, async {
        while hub.port_count() < 3 {
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();

    // TCP A -> TCP B and WebSocket; sent in two pieces with lowercase hex.
    a.get_mut().write_all(b":x195b4123n05010101").await.unwrap();
    a.get_mut().write_all(b"22000010;").await.unwrap();
    let mut line = String::new();
    timeout(T, b.read_line(&mut line)).await.unwrap().unwrap();
    assert_eq!(line.trim(), ":X195B4123N0501010122000010;");
    let msg = timeout(T, w.next()).await.unwrap().unwrap().unwrap();
    assert_eq!(
        msg.to_text().unwrap().trim(),
        ":X195B4123N0501010122000010;"
    );

    // WebSocket -> both TCP clients, two frames in one message.
    w.send(WsMessage::text(":X19490ABCN;:X19170ABCN050101012200;"))
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

    // A never received its own frame: the next thing it sees is the WS
    // frames (checked above). And the WS client never got its own frames.
    assert!(timeout(Duration::from_millis(200), w.next()).await.is_err());
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
