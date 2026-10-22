//! End-to-end: a throttle node finds a train by DCC address, takes control,
//! sets speed and answers a heartbeat — exercised against a scripted train.

use rolcb::message::Message;
use rolcb::node::ALIAS_RESERVE_MS;
use rolcb::traction::{search, Command, Reply, Speed};
use rolcb::{EventState, Node, NodeId, NodeInfo};

const TRAIN_ALIAS: u16 = 0x6A1;

fn decode_all(frames: Vec<rolcb::CanFrame>) -> Vec<Message> {
    // Feed our own output through a fresh "listener" node to decode it.
    let mut listener = Node::new(NodeId::new(0x0909_0909_0909), NodeInfo::default());
    frames
        .iter()
        .filter_map(|f| listener.receive(f, 0))
        .map(|r| r.msg)
        .collect()
}

#[test]
fn acquire_drive_release() {
    let me = NodeId::new(0x0501_0101_2299);
    let mut node = Node::new(me, NodeInfo::default());
    node.start(0);
    node.poll(ALIAS_RESERVE_MS);
    let my_alias = node.alias().unwrap();
    node.take_outgoing();

    // 1. Search for DCC 3.
    let ev = search::dcc_event(3, false, search::ALLOCATE);
    node.send(Message::IdentifyProducer(ev));
    let out = decode_all(node.take_outgoing());
    assert_eq!(out, vec![Message::IdentifyProducer(ev)]);

    // 2. Train answers.
    let reply = Message::ProducerIdentified {
        event: ev,
        state: EventState::Valid,
    };
    let r = node.receive(&reply.to_frames(TRAIN_ALIAS)[0], 10).unwrap();
    assert_eq!(r.src_alias, TRAIN_ALIAS);

    // 3. Assign controller (9 bytes -> 2 frames) and set speed.
    node.send(Message::TractionCommand {
        dest: TRAIN_ALIAS,
        payload: Command::AssignController {
            flags: 0,
            controller: me,
        }
        .encode(),
    });
    node.send(Message::TractionCommand {
        dest: TRAIN_ALIAS,
        payload: Command::SetSpeed(Speed::from_mph(20.0, false)).encode(),
    });
    let frames = node.take_outgoing();
    assert_eq!(frames.len(), 3);
    let out = decode_all(frames);
    assert_eq!(out.len(), 2);
    match &out[0] {
        Message::TractionCommand { dest, payload } => {
            assert_eq!(*dest, TRAIN_ALIAS);
            assert_eq!(
                Command::decode(payload),
                Command::AssignController {
                    flags: 0,
                    controller: me
                }
            );
        }
        m => panic!("{m:?}"),
    }

    // 4. Heartbeat request from train is delivered to the app.
    let hb = Message::TractionReply {
        dest: my_alias,
        payload: Reply::Heartbeat { timeout_s: 10 }.encode(),
    };
    let r = node.receive(&hb.to_frames(TRAIN_ALIAS)[0], 20).unwrap();
    match r.msg {
        Message::TractionReply { payload, .. } => {
            assert_eq!(Reply::decode(&payload), Reply::Heartbeat { timeout_s: 10 })
        }
        m => panic!("{m:?}"),
    }
    // Replies are not auto-rejected.
    assert!(node.take_outgoing().is_empty());
}
