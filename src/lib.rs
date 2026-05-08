//! Rust library for CAN-bus actuator motors (Damiao, Robstride, ...).
//!
//! ## Layers
//!
//! * [`transport`] — backend-agnostic CAN(-FD) frame I/O. The default
//!   backend is [`transport::SocketCanTransport`], which works for
//!   Candlelight (`gs_usb`) USB adapters and for the MCP2518FD on
//!   Raspberry Pi (`mcp251xfd`), as well as `vcan` for tests.
//! * [`damiao`] — Damiao MIT-mode protocol (classic CAN, 1 Mbit).
//!
//! ## Example
//!
//! ```no_run
//! use motorcom::damiao::Damiao;
//! use motorcom::damiao::mit::MitSetpoint;
//! use motorcom::transport::SocketCanTransport;
//!
//! let mut bus = SocketCanTransport::open("can0").unwrap();
//! let dm = Damiao::new(/* motor_id */ 0x01, /* master_id */ 0x11);
//!
//! dm.enable(&mut bus).unwrap();
//! dm.mit_control(&mut bus, &MitSetpoint { kp: 50.0, kd: 1.0, ..Default::default() }).unwrap();
//! dm.disable(&mut bus).unwrap();
//! ```
//!
//! Bring up the interface beforehand with:
//!
//! ```text
//! sudo ip link set can0 up type can bitrate 1000000
//! ```

pub mod transport;
pub mod damiao;
pub mod robstride;
