//! Damiao driver errors. Wraps transport errors and adds protocol-level
//! cases (unknown register, read-only register, type mismatch, ...).

use thiserror::Error;

use crate::transport::CanError;

use super::registers::DataType;

#[derive(Debug, Error)]
pub enum Error {
    #[error("transport error: {0}")]
    Transport(#[from] CanError),

    #[error("register {0} is not in the Damiao register table")]
    UnknownRegister(u8),

    #[error("register {0} is read-only")]
    ReadOnlyRegister(u8),

    #[error("register {rid}: expected {expected:?}, got {got:?}")]
    TypeMismatch { rid: u8, expected: DataType, got: DataType },

    #[error("unexpected reply (data did not match register-reply pattern): {0:?}")]
    UnexpectedReply(Vec<u8>),

    #[error("control mode verification failed: wrote {wrote}, motor reports {got}")]
    ControlModeVerifyFailed { wrote: u8, got: u8 },
}
