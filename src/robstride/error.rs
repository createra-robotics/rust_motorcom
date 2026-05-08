//! Robstride driver errors.

use thiserror::Error;

use crate::transport::CanError;

use super::parameters::DataType;
use super::status::FaultReport;

#[derive(Debug, Error)]
pub enum Error {
    #[error("transport error: {0}")]
    Transport(#[from] CanError),

    #[error("motor returned a fault report: {0:?}")]
    FaultReport(FaultReport),

    #[error("unexpected communication_type: got {got}, expected {expected}")]
    UnexpectedCommType { got: u8, expected: u8 },

    #[error("parameter type mismatch: parameter id 0x{rid:04X} expects {expected:?}, got {got:?}")]
    TypeMismatch { rid: u16, expected: DataType, got: DataType },

    #[error("malformed reply: {0}")]
    MalformedReply(&'static str),
}
