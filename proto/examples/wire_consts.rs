//! Every wire constant the host tools speak, as JSON.
//!
//! `tools/wio_link.py` used to restate the framing byte, the USB command
//! ids, the bulk ops and the ack statuses by hand, and nothing checked the
//! copy. It reads `tools/wire_consts.json` now, and a test in this crate
//! holds that file to what this prints - so a constant that moves is a
//! failing test, not a tool that quietly sends the wrong byte.
//!
//! ```text
//! cargo run --example wire_consts > ../tools/wire_consts.json
//! ```

use midair_proto::session::KNOBS;
use midair_proto::{ble, link};
use midair_proto::gps_proto::packet;

fn main() {
    print!("{}", midair_proto::wire_consts_json());
    let _ = (KNOBS.len(), ble::OP_BEGIN, link::SYNC, packet::ACK_OK);
}
