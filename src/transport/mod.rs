//! CAN(-FD) transport abstraction.
//!
//! Motor protocol code (Damiao, Robstride, ...) talks to this trait, not to a
//! concrete adapter. Backends in this module wrap real hardware:
//!
//! * [`socketcan_backend::SocketCanTransport`] — Linux SocketCAN. Covers the
//!   Candlelight (`gs_usb`) USB adapters and the MCP2518FD on Raspberry Pi
//!   (`mcp251xfd`), plus `vcan` for tests.
//! * [`mock::MockTransport`] — in-process loopback for unit tests.

use std::time::Duration;
use thiserror::Error;

#[cfg(feature = "socketcan-backend")]
pub mod socketcan_backend;
pub mod mock;

#[cfg(feature = "socketcan-backend")]
pub use socketcan_backend::SocketCanTransport;
pub use mock::MockTransport;

/// Either an 11-bit standard or 29-bit extended CAN identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanId {
    Standard(u16),
    Extended(u32),
}

impl CanId {
    pub fn standard(raw: u16) -> Self {
        CanId::Standard(raw & 0x7FF)
    }
    pub fn extended(raw: u32) -> Self {
        CanId::Extended(raw & 0x1FFF_FFFF)
    }
    pub fn raw(&self) -> u32 {
        match *self {
            CanId::Standard(v) => v as u32,
            CanId::Extended(v) => v,
        }
    }
}

/// One CAN or CAN-FD frame. Up to 8 bytes for classic, 64 for FD.
#[derive(Debug, Clone)]
pub struct CanFrame {
    pub id: CanId,
    pub data: Vec<u8>,
    /// True for CAN-FD frames. Required for Robstride; must be false for Damiao.
    pub fd: bool,
    /// FD bit-rate switch (data-phase rate). Ignored when `fd == false`.
    pub brs: bool,
}

impl CanFrame {
    pub fn classic(id: CanId, data: impl Into<Vec<u8>>) -> Self {
        Self { id, data: data.into(), fd: false, brs: false }
    }
    pub fn fd(id: CanId, data: impl Into<Vec<u8>>, brs: bool) -> Self {
        Self { id, data: data.into(), fd: true, brs }
    }
}

#[derive(Debug, Error)]
pub enum CanError {
    #[error("timeout waiting for frame")]
    Timeout,
    #[error("frame too large for selected mode: {0} bytes")]
    OversizedFrame(usize),
    #[error("invalid CAN id")]
    InvalidId,
    #[error("backend error: {0}")]
    Backend(#[from] std::io::Error),
}

/// Synchronous, frame-oriented transport.
///
/// One implementation per physical bus (`can0`, `can1`, ...). A single
/// instance is shared across all motors on that bus.
pub trait CanTransport: Send {
    fn send(&mut self, frame: &CanFrame) -> Result<(), CanError>;
    fn recv(&mut self, timeout: Duration) -> Result<CanFrame, CanError>;

    /// Send `frame`, then wait up to `timeout` for the first frame whose id
    /// matches `expect_id`. Frames with other ids that arrive in the meantime
    /// are dropped — useful for request/response on a shared bus.
    fn request(
        &mut self,
        frame: &CanFrame,
        expect_id: CanId,
        timeout: Duration,
    ) -> Result<CanFrame, CanError> {
        self.send(frame)?;
        let deadline = std::time::Instant::now() + timeout;
        loop {
            let remaining = deadline
                .checked_duration_since(std::time::Instant::now())
                .ok_or(CanError::Timeout)?;
            let rx = self.recv(remaining)?;
            if rx.id == expect_id {
                return Ok(rx);
            }
        }
    }
}
