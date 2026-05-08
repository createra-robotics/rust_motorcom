//! Robstride driver — encodes/decodes the 29-bit extended-id frames, the
//! `OPERATION_CONTROL`/`OPERATION_STATUS` exchange, and the 16-bit
//! parameter read/write protocol.

use std::time::{Duration, Instant};

use crate::robstride::mit::{self as op, MotorState, MitLimits, MitSetpoint};
use crate::transport::{CanError, CanFrame, CanId, CanTransport};

use super::error::Error;
use super::frame::{self, comm_type};
use super::parameters::{ParameterDef, ParameterValue};
use super::status::{FaultReport, StatusFlags};

const DEFAULT_TIMEOUT: Duration = Duration::from_millis(20);
/// Manufacturer recommends host_id > all device_ids; `0xFF` is the maximum.
const DEFAULT_HOST_ID: u16 = 0xFF;

pub struct Robstride {
    pub motor_id: u8,
    pub host_id: u16,
    pub timeout: Duration,
    pub limits: MitLimits,
}

/// Result of any command that returns an `OPERATION_STATUS` frame.
#[derive(Debug, Clone, Copy)]
pub struct OperationStatus {
    pub state: MotorState,
    pub flags: StatusFlags,
    pub device_id: u8,
}

/// Result of `ping` / `set_device_id`.
#[derive(Debug, Clone, Copy)]
pub struct DeviceInfo {
    pub device_id: u8,
    pub uuid: [u8; 8],
}

impl Robstride {
    pub fn new(motor_id: u8) -> Self {
        Self {
            motor_id,
            host_id: DEFAULT_HOST_ID,
            timeout: DEFAULT_TIMEOUT,
            limits: MitLimits {
                p_max: 4.0 * std::f32::consts::PI,
                v_max: 50.0,
                t_max: 17.0,
                kp_max: 500.0,
                kd_max: 5.0,
            },
        }
    }

    pub fn with_limits(mut self, limits: MitLimits) -> Self {
        self.limits = limits;
        self
    }
    pub fn with_host_id(mut self, host_id: u16) -> Self {
        self.host_id = host_id;
        self
    }
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    // -------------------------------------------------------------------
    // Basic motor commands
    // -------------------------------------------------------------------

    pub fn enable(&self, can: &mut dyn CanTransport) -> Result<OperationStatus, Error> {
        self.send_simple(can, comm_type::ENABLE)?;
        self.await_op_status(can)
    }

    pub fn disable(&self, can: &mut dyn CanTransport) -> Result<OperationStatus, Error> {
        self.send_simple(can, comm_type::DISABLE)?;
        self.await_op_status(can)
    }

    pub fn set_zero_position(&self, can: &mut dyn CanTransport) -> Result<OperationStatus, Error> {
        self.send_simple(can, comm_type::SET_ZERO_POSITION)?;
        self.await_op_status(can)
    }

    /// Send an `OPERATION_CONTROL` setpoint and read back the resulting
    /// `OPERATION_STATUS` frame.
    pub fn operation_control(
        &self,
        can: &mut dyn CanTransport,
        sp: &MitSetpoint,
    ) -> Result<OperationStatus, Error> {
        let enc = op::encode(&self.limits, sp);
        let ext_id = frame::encode(comm_type::OPERATION_CONTROL, enc.torque_extra, self.motor_id);
        let f = CanFrame::classic(CanId::Extended(ext_id), enc.data.to_vec());
        can.send(&f).map_err(Error::Transport)?;
        self.await_op_status(can)
    }

    /// `GET_DEVICE_ID` ping — returns the motor's id and 8-byte MCU UUID.
    pub fn ping(&self, can: &mut dyn CanTransport) -> Result<DeviceInfo, Error> {
        self.send_simple(can, comm_type::GET_DEVICE_ID)?;
        let (extra, data) = self.await_reply(can, comm_type::GET_DEVICE_ID)?;
        Ok(DeviceInfo {
            device_id: (extra & 0xFF) as u8,
            uuid: data,
        })
    }

    /// Reassign the motor's device id (persists after `save_parameters`).
    pub fn set_device_id(
        &self,
        can: &mut dyn CanTransport,
        new_id: u8,
    ) -> Result<DeviceInfo, Error> {
        let extra = ((new_id as u16) << 8) | (self.host_id & 0xFF);
        let ext_id = frame::encode(comm_type::SET_DEVICE_ID, extra, self.motor_id);
        let f = CanFrame::classic(CanId::Extended(ext_id), vec![0u8; 8]);
        can.send(&f).map_err(Error::Transport)?;
        let (resp_extra, data) = self.await_reply(can, comm_type::GET_DEVICE_ID)?;
        Ok(DeviceInfo {
            device_id: (resp_extra & 0xFF) as u8,
            uuid: data,
        })
    }

    // -------------------------------------------------------------------
    // Parameter read / write / save
    // -------------------------------------------------------------------

    /// Read a parameter. The reply's value bytes are decoded per
    /// `param.data_type`.
    pub fn read_parameter(
        &self,
        can: &mut dyn CanTransport,
        param: &ParameterDef,
    ) -> Result<ParameterValue, Error> {
        let mut data = [0u8; 8];
        data[0..2].copy_from_slice(&param.id.to_le_bytes());
        // bytes 2..8: zero
        let ext_id = frame::encode(comm_type::READ_PARAMETER, self.host_id, self.motor_id);
        let f = CanFrame::classic(CanId::Extended(ext_id), data.to_vec());
        can.send(&f).map_err(Error::Transport)?;

        let (_extra, reply) = self.await_reply(can, comm_type::READ_PARAMETER)?;
        let mut value_bytes = [0u8; 4];
        value_bytes.copy_from_slice(&reply[4..8]);
        Ok(ParameterValue::from_le_bytes(param.data_type, value_bytes))
    }

    /// Write a parameter. The variant of `value` must match `param.data_type`.
    pub fn write_parameter(
        &self,
        can: &mut dyn CanTransport,
        param: &ParameterDef,
        value: ParameterValue,
    ) -> Result<OperationStatus, Error> {
        if value.data_type() != param.data_type {
            return Err(Error::TypeMismatch {
                rid: param.id,
                expected: param.data_type,
                got: value.data_type(),
            });
        }
        let mut data = [0u8; 8];
        data[0..2].copy_from_slice(&param.id.to_le_bytes());
        // bytes 2..4: zero padding
        data[4..8].copy_from_slice(&value.to_le_bytes());

        let ext_id = frame::encode(comm_type::WRITE_PARAMETER, self.host_id, self.motor_id);
        let f = CanFrame::classic(CanId::Extended(ext_id), data.to_vec());
        can.send(&f).map_err(Error::Transport)?;

        // Motor confirms with an OPERATION_STATUS frame.
        self.await_op_status(can)
    }

    /// Persist all parameters to flash. Fire-and-forget; the python
    /// reference doesn't expose a confirmation method.
    pub fn save_parameters(&self, can: &mut dyn CanTransport) -> Result<(), Error> {
        self.send_simple(can, comm_type::SAVE_PARAMETERS)?;
        Ok(())
    }

    // -------------------------------------------------------------------
    // Helpers (private)
    // -------------------------------------------------------------------

    /// Send a command frame with no data payload (8 zero bytes).
    fn send_simple(&self, can: &mut dyn CanTransport, comm: u8) -> Result<(), Error> {
        let ext_id = frame::encode(comm, self.host_id, self.motor_id);
        let f = CanFrame::classic(CanId::Extended(ext_id), vec![0u8; 8]);
        can.send(&f).map_err(Error::Transport)
    }

    /// Receive frames until one matches `want_comm` (returning its
    /// `(extra_data, data)`), drop unrelated traffic, surface
    /// `FAULT_REPORT` as `Error::FaultReport`.
    fn await_reply(
        &self,
        can: &mut dyn CanTransport,
        want_comm: u8,
    ) -> Result<(u16, [u8; 8]), Error> {
        let deadline = Instant::now() + self.timeout;
        loop {
            let remaining = deadline
                .checked_duration_since(Instant::now())
                .ok_or(Error::Transport(CanError::Timeout))?;
            let f = can.recv(remaining).map_err(Error::Transport)?;
            let ext_id = match f.id {
                CanId::Extended(v) => v,
                _ => continue,
            };
            let (comm, extra, _low) = frame::decode(ext_id);
            if comm == comm_type::FAULT_REPORT {
                return Err(Error::FaultReport(FaultReport::from_data(&f.data)));
            }
            if comm == want_comm {
                let mut data = [0u8; 8];
                let n = f.data.len().min(8);
                data[..n].copy_from_slice(&f.data[..n]);
                return Ok((extra, data));
            }
            // Unrelated traffic (e.g., other motor's status, ACTIVE_REPORT) — drop.
        }
    }

    /// Convenience: wait for an `OPERATION_STATUS` reply and assemble
    /// the high-level [`OperationStatus`].
    fn await_op_status(&self, can: &mut dyn CanTransport) -> Result<OperationStatus, Error> {
        let (extra, data) = self.await_reply(can, comm_type::OPERATION_STATUS)?;
        let device_id = (extra & 0xFF) as u8;
        let flags = StatusFlags::from_extra(extra);
        let state = op::decode_status(&self.limits, &data);
        Ok(OperationStatus { state, flags, device_id })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::parameters;
    use crate::transport::MockTransport;

    fn op_status_frame(motor_id: u8, host_id: u16, state_data: [u8; 8]) -> CanFrame {
        let extra = motor_id as u16; // no flags, just device_id
        let _ = state_data; // suppress
        let ext_id = frame::encode(comm_type::OPERATION_STATUS, extra, (host_id & 0xFF) as u8);
        CanFrame::classic(CanId::Extended(ext_id), state_data.to_vec())
    }

    #[test]
    fn enable_sends_ext_id_with_comm_type_3() {
        let (mut host, mut motor) = MockTransport::pair();
        let rs = Robstride::new(0x05);
        // Pre-stage an OPERATION_STATUS reply.
        motor.send(&op_status_frame(0x05, rs.host_id, [0u8; 8])).unwrap();

        rs.enable(&mut host).unwrap();

        let sent = motor.peek().unwrap();
        let ext_id = match sent.id {
            CanId::Extended(v) => v,
            _ => panic!("expected extended id"),
        };
        let (c, e, d) = frame::decode(ext_id);
        assert_eq!(c, comm_type::ENABLE);
        assert_eq!(e, rs.host_id);
        assert_eq!(d, 0x05);
    }

    #[test]
    fn operation_control_puts_torque_in_extra_data() {
        let (mut host, mut motor) = MockTransport::pair();
        let rs = Robstride::new(0x07);
        motor.send(&op_status_frame(0x07, rs.host_id, [0u8; 8])).unwrap();

        // torque = 0 → torque_u16 = 0x7FFF
        let sp = MitSetpoint::default();
        rs.operation_control(&mut host, &sp).unwrap();

        let sent = motor.peek().unwrap();
        let ext_id = match sent.id {
            CanId::Extended(v) => v,
            _ => panic!(),
        };
        let (c, e, d) = frame::decode(ext_id);
        assert_eq!(c, comm_type::OPERATION_CONTROL);
        assert_eq!(e, 0x7FFF);
        assert_eq!(d, 0x07);

        // Data payload's pos field should be 0x7FFF (big-endian).
        assert_eq!(&sent.data[0..2], &0x7FFF_u16.to_be_bytes());
    }

    #[test]
    fn read_parameter_decodes_float_reply() {
        let (mut host, mut motor) = MockTransport::pair();
        let rs = Robstride::new(0x01);

        // Pre-stage a READ_PARAMETER reply with VBUS = 24.0.
        let value = 24.0_f32;
        let mut reply = [0u8; 8];
        reply[0..2].copy_from_slice(&parameters::VBUS.id.to_le_bytes());
        reply[4..8].copy_from_slice(&value.to_le_bytes());
        let ext_id = frame::encode(comm_type::READ_PARAMETER, rs.host_id, (rs.host_id & 0xFF) as u8);
        motor
            .send(&CanFrame::classic(CanId::Extended(ext_id), reply.to_vec()))
            .unwrap();

        let v = rs.read_parameter(&mut host, &parameters::VBUS).unwrap();
        assert_eq!(v, ParameterValue::Float32(24.0));
    }

    #[test]
    fn write_parameter_rejects_type_mismatch() {
        let (mut host, _motor) = MockTransport::pair();
        let rs = Robstride::new(0x01);
        let err = rs
            .write_parameter(&mut host, &parameters::MODE, ParameterValue::Float32(0.0))
            .unwrap_err();
        assert!(matches!(err, Error::TypeMismatch { .. }));
    }

    #[test]
    fn fault_report_surfaces_as_error() {
        let (mut host, mut motor) = MockTransport::pair();
        let rs = Robstride::new(0x01);

        // Build a FAULT_REPORT frame: undervoltage bit set.
        let mut data = [0u8; 8];
        let fault = 1u32 << 2;
        data[0..4].copy_from_slice(&fault.to_le_bytes());
        let ext_id = frame::encode(comm_type::FAULT_REPORT, 0x01, (rs.host_id & 0xFF) as u8);
        motor
            .send(&CanFrame::classic(CanId::Extended(ext_id), data.to_vec()))
            .unwrap();

        let err = rs.enable(&mut host).unwrap_err();
        match err {
            Error::FaultReport(f) => assert!(f.undervoltage),
            other => panic!("expected FaultReport, got {other:?}"),
        }
    }
}
