//! Damiao DM-series motor driver.
//!
//! Damiao motors run on **classic CAN** at 1 Mbit. Each motor listens on its
//! own 11-bit standard id and replies on the configured master id with an
//! 8-byte status frame. Impedance setpoints are encoded with the shared
//! [`crate::damiao::mit`] codec.
//!
//! ```no_run
//! use motorcom::damiao::{Damiao, DM4310};
//! use motorcom::damiao::mit::MitSetpoint;
//! use motorcom::transport::SocketCanTransport;
//!
//! let mut bus = SocketCanTransport::open("can0").unwrap();
//! let dm = Damiao::new(0x01, 0x11).with_limits(DM4310);
//! dm.enable(&mut bus).unwrap();
//! dm.mit_control(&mut bus, &MitSetpoint { kp: 50.0, kd: 1.0, ..Default::default() }).unwrap();
//! dm.disable(&mut bus).unwrap();
//! ```
//!
//! ## Motor variants
//!
//! Each variant exports a `MitLimits` constant with the manufacturer's
//! `[PMAX, VMAX, TMAX]`. Pass into [`Damiao::with_limits`] when constructing
//! the driver, or fetch via the matching module ([`dm4310`], [`dm4340p`], ...).

mod driver;
pub mod error;
pub mod mit;
pub mod modes;
pub mod registers;
pub mod status;

pub use driver::Damiao;
pub use error::Error;
pub use modes::{ControlMode, ControlSetpoint};
pub use registers::{Access, BaudRate, DataType, RegisterInfo, RegisterValue, REGISTER_TABLE};
pub use status::MotorStatus;

// Motor variants — one module per Damiao DM-series part.
pub mod dm3507;
pub mod dm4310;
pub mod dm4310p;
pub mod dm4340;
pub mod dm4340p;
pub mod dm6006;
pub mod dm8006;
pub mod dm8009;
pub mod dm10010;
pub mod dm10010l;
pub mod dmh3510;
pub mod dmg6215;
pub mod dmh6220;
pub mod dmjh11;
pub mod dm6248p;

pub use dm3507::DM3507;
pub use dm4310::DM4310;
pub use dm4310p::DM4310P;
pub use dm4340::DM4340;
pub use dm4340p::DM4340P;
pub use dm6006::DM6006;
pub use dm8006::DM8006;
pub use dm8009::DM8009;
pub use dm10010::DM10010;
pub use dm10010l::DM10010L;
pub use dmh3510::DMH3510;
pub use dmg6215::DMG6215;
pub use dmh6220::DMH6220;
pub use dmjh11::DMJH11;
pub use dm6248p::DM6248P;
