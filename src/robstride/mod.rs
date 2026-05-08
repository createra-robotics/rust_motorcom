//! Robstride RS-series motor driver.
//!
//! Robstride motors run on **classic CAN** at 1 Mbit with **29-bit
//! extended ids**. The id is split into three packed fields — see
//! [`frame`] — and `OPERATION_CONTROL` carries its torque setpoint in
//! the id's `extra_data` field rather than the data payload.
//!
//! ```no_run
//! use motorcom::robstride::{Robstride, RS00};
//! use motorcom::robstride::mit::MitSetpoint;
//! use motorcom::transport::SocketCanTransport;
//!
//! let mut bus = SocketCanTransport::open("can0").unwrap();
//! let rs = Robstride::new(0x01).with_limits(RS00);
//! rs.enable(&mut bus).unwrap();
//! rs.operation_control(&mut bus, &MitSetpoint {
//!     position: 0.0, velocity: 0.0, torque: 0.0, kp: 50.0, kd: 1.0,
//! }).unwrap();
//! rs.disable(&mut bus).unwrap();
//! ```
//!
//! The codec for the operation/status frame lives in
//! [`crate::robstride::mit`]; per-bit status flags and the
//! fault-report layout live in [`status`]; the parameter table lives
//! in [`parameters`].

mod driver;
pub mod error;
pub mod frame;
pub mod mit;
pub mod parameters;
pub mod status;

pub use driver::{DeviceInfo, OperationStatus, Robstride};
pub use error::Error;

// Motor variants.
pub mod rs00;
pub mod rs01;
pub mod rs02;
pub mod rs03;
pub mod rs04;
pub mod rs05;
pub mod rs06;

pub use rs00::RS00;
pub use rs01::RS01;
pub use rs02::RS02;
pub use rs03::RS03;
pub use rs04::RS04;
pub use rs05::RS05;
pub use rs06::RS06;
