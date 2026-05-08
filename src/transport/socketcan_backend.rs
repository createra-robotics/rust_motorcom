//! SocketCAN backend.
//!
//! Works for any Linux CAN interface — `gs_usb` (Candlelight / CANable),
//! `mcp251xfd` (MCP2518FD on Raspberry Pi via SPI), `vcan` for tests, PCAN,
//! Kvaser, etc. The interface must already be brought up via `ip link`:
//!
//! ```text
//! sudo ip link set can0 up type can bitrate 1000000 \
//!     dbitrate 5000000 fd on sample-point 0.75 dsample-point 0.75
//! ```
//!
//! For classic-CAN-only Damiao setups, drop the `dbitrate ... fd on` bits.

use std::time::Duration;

use embedded_can::{ExtendedId, Frame as EmbeddedFrame, Id, StandardId};
use socketcan::{
    CanAnyFrame, CanFdFrame, CanFdSocket, CanFilter, CanFrame as ScCanFrame, Socket,
    SocketOptions,
};

use super::{CanError, CanFrame, CanId, CanTransport};

/// Linux CAN extended-frame flag (`CAN_EFF_FLAG` from `<linux/can.h>`).
const EFF_FLAG: u32 = 0x8000_0000;

pub struct SocketCanTransport {
    sock: CanFdSocket,
}

impl SocketCanTransport {
    /// Open `iface` (e.g. `"can0"`). Interface must already be configured up.
    pub fn open(iface: &str) -> Result<Self, CanError> {
        let sock = CanFdSocket::open(iface).map_err(io_other)?;
        Ok(Self { sock })
    }

    /// Hardware-side filter: only deliver frames whose id matches one of these.
    /// Pass an empty slice to clear (the kernel default is "accept everything").
    pub fn set_filter(&self, ids: &[CanId]) -> Result<(), CanError> {
        let filters: Vec<CanFilter> = ids
            .iter()
            .map(|id| match *id {
                CanId::Standard(v) => CanFilter::new(v as u32, 0x7FF),
                CanId::Extended(v) => CanFilter::new(
                    v | EFF_FLAG,
                    0x1FFF_FFFF | EFF_FLAG,
                ),
            })
            .collect();
        self.sock.set_filters(&filters).map_err(io_other)
    }
}

impl CanTransport for SocketCanTransport {
    fn send(&mut self, frame: &CanFrame) -> Result<(), CanError> {
        let id = to_embedded_id(frame.id)?;
        let any: CanAnyFrame = if frame.fd {
            if frame.data.len() > 64 {
                return Err(CanError::OversizedFrame(frame.data.len()));
            }
            let mut f = CanFdFrame::new(id, &frame.data)
                .ok_or(CanError::OversizedFrame(frame.data.len()))?;
            if frame.brs {
                f.set_brs(true);
            }
            f.into()
        } else {
            if frame.data.len() > 8 {
                return Err(CanError::OversizedFrame(frame.data.len()));
            }
            ScCanFrame::new(id, &frame.data)
                .ok_or(CanError::OversizedFrame(frame.data.len()))?
                .into()
        };
        self.sock.write_frame(&any).map_err(io_other)
    }

    fn recv(&mut self, timeout: Duration) -> Result<CanFrame, CanError> {
        self.sock.set_read_timeout(timeout).map_err(io_other)?;
        match self.sock.read_frame() {
            Ok(any) => Ok(from_any_frame(any)),
            Err(e) => match e.kind() {
                std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut => {
                    Err(CanError::Timeout)
                }
                _ => Err(CanError::Backend(e)),
            },
        }
    }
}

fn to_embedded_id(id: CanId) -> Result<Id, CanError> {
    match id {
        CanId::Standard(v) => StandardId::new(v).map(Id::Standard).ok_or(CanError::InvalidId),
        CanId::Extended(v) => ExtendedId::new(v).map(Id::Extended).ok_or(CanError::InvalidId),
    }
}

fn from_embedded_id(id: Id) -> CanId {
    match id {
        Id::Standard(s) => CanId::Standard(s.as_raw()),
        Id::Extended(e) => CanId::Extended(e.as_raw()),
    }
}

fn from_any_frame(any: CanAnyFrame) -> CanFrame {
    match any {
        CanAnyFrame::Normal(f) => CanFrame {
            id: from_embedded_id(f.id()),
            data: f.data().to_vec(),
            fd: false,
            brs: false,
        },
        CanAnyFrame::Remote(f) => CanFrame {
            id: from_embedded_id(f.id()),
            data: f.data().to_vec(),
            fd: false,
            brs: false,
        },
        CanAnyFrame::Error(f) => CanFrame {
            id: from_embedded_id(f.id()),
            data: f.data().to_vec(),
            fd: false,
            brs: false,
        },
        CanAnyFrame::Fd(f) => CanFrame {
            id: from_embedded_id(f.id()),
            data: f.data().to_vec(),
            fd: true,
            brs: f.is_brs(),
        },
    }
}

fn io_other<E: std::error::Error + Send + Sync + 'static>(e: E) -> CanError {
    CanError::Backend(std::io::Error::new(std::io::ErrorKind::Other, e))
}
