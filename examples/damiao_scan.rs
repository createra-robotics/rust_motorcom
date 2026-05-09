//! Scan a CAN bus for Damiao motors and print each motor's `motor_id`
//! and `master_id` (the id it replies on).
//!
//! ```text
//! sudo ip link set can0 up type can bitrate 1000000
//! cargo run --release --example damiao_scan
//! cargo run --release --example damiao_scan -- can0 1 64
//! ```
//!
//! Args (all optional): `<interface> <start_id> <end_id>`. Defaults are
//! `can0 1 64`.
//!
//! Probe: a feedback-request frame (`D[2] = 0xCC`) is broadcast on the
//! Damiao register id `0x7FF`; the motor replies on its configured master
//! id with the 8-byte MIT status frame. Status fields are decoded with the
//! DM-J4340P-2EC limits — swap [`DM4340P`] for a different variant if your
//! motor differs.

use std::env;
use std::time::Duration;

use motorcom::damiao::{mit, DM4340P};
use motorcom::transport::{CanFrame, CanId, CanTransport, SocketCanTransport};

const BROADCAST_ID: u16 = 0x7FF;
const FEEDBACK_REQ: u8 = 0xCC;
const REPLY_TIMEOUT: Duration = Duration::from_millis(20);

fn feedback_request(motor_id: u16) -> CanFrame {
    CanFrame::classic(
        CanId::standard(BROADCAST_ID),
        [
            (motor_id & 0xFF) as u8,
            ((motor_id >> 8) & 0xFF) as u8,
            FEEDBACK_REQ,
            0x00,
            0, 0, 0, 0,
        ],
    )
}

fn drain<T: CanTransport>(bus: &mut T) {
    while bus.recv(Duration::from_millis(1)).is_ok() {}
}

fn main() {
    let mut args = env::args().skip(1);
    let iface = args.next().unwrap_or_else(|| "can0".to_string());
    let start: u16 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0x01);
    let end: u16 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0x40);

    let mut bus = SocketCanTransport::open(&iface).expect("open CAN interface");

    println!(
        "scanning {iface} for Damiao motors over id range 0x{start:02X}..=0x{end:02X}\n"
    );
    println!(
        "  {:>8}  {:>9}  {:>10}  {:>11}  {:>9}  {:>5}  {:>5}",
        "CAN ID", "FEEDBACK ID (MASTER ID)", "POS[rad]", "VEL[rad/s]", "TAU[N·m]", "MOSFET Temperature", "Roter Temperature",
    );
    println!("  {}", "-".repeat(72));

    let mut found = 0usize;
    for motor_id in start..=end {
        drain(&mut bus); // discard stragglers before each probe

        if bus.send(&feedback_request(motor_id)).is_err() {
            continue;
        }
        let Ok(reply) = bus.recv(REPLY_TIMEOUT) else { continue };
        if reply.data.len() != 8 {
            continue;
        }

        let master = match reply.id {
            CanId::Standard(v) => v,
            CanId::Extended(v) => (v & 0x7FF) as u16,
        };
        let st = mit::decode_state(&DM4340P, &reply.data);
        println!(
            "    0x{:02X}       0x{:03X}    {:>+9.3}    {:>+9.3}    {:>+6.2}    {:>3}    {:>3}",
            motor_id, master, st.position, st.velocity, st.torque, st.t_mos, st.t_rotor,
        );
        found += 1;
    }

    println!("\ndone — {found} motor(s) responded");
}
