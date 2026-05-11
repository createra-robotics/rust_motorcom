//! Damiao driver — framing, addressing, and command/register protocol.
//!
//! The MIT impedance setpoint payload is encoded by [`crate::damiao::mit`];
//! this module owns the Damiao-specific parts:
//!
//! * classic-CAN framing, 11-bit standard ids
//! * `0xFF FF FF FF FF FF FF Fx` enable / disable / set-zero / clear-error
//! * non-MIT control modes (POS_VEL, VEL, FORCE_POS) on `0x100/0x200/0x300 + motor_id`
//! * register read/write/save-flash/feedback-request frames on broadcast id `0x7FF`

use std::time::{Duration, Instant};

use crate::damiao::mit::{self, MitLimits, MitSetpoint, MotorState};
use crate::transport::{CanError, CanFrame, CanId, CanTransport};

use super::error::Error;
use super::modes::{self, ControlMode, ControlSetpoint};
use super::registers::{self, BaudRate, DataType, RegisterValue};

const ENABLE_CMD:      [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC];
const DISABLE_CMD:     [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];
const SET_ZERO_CMD:    [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE];
const CLEAR_ERROR_CMD: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB];

/// Broadcast id used for register read/write/save/feedback-request frames.
const REGISTER_BROADCAST_ID: u16 = 0x7FF;
const READ_CMD:         u8 = 0x33;
const WRITE_CMD:        u8 = 0x55;
const STORE_CMD:        u8 = 0xAA;
const FEEDBACK_REQ_CMD: u8 = 0xCC;

const DEFAULT_TIMEOUT: Duration = Duration::from_millis(20);
const CONTROL_MODE_WRITE_DELAY: Duration = Duration::from_millis(100);

/// Driver for one Damiao motor on a shared CAN bus.
pub struct Damiao {
    /// 11-bit motor id (the id the motor *listens* on).
    pub motor_id: u16,
    /// 11-bit master id (the id the motor *replies* on).
    pub master_id: u16,
    /// Per-command response timeout.
    pub timeout: Duration,
    /// Encoding limits — varies per motor variant (DM4310, DM4340, ...).
    pub limits: MitLimits,
}

impl Damiao {
    pub fn new(motor_id: u16, master_id: u16) -> Self {
        Self {
            motor_id,
            master_id,
            timeout: DEFAULT_TIMEOUT,
            limits: MitLimits::default(),
        }
    }

    pub fn with_limits(mut self, limits: MitLimits) -> Self {
        self.limits = limits;
        self
    }

    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    // -------------------------------------------------------------------
    // FF-prefix commands (enable / disable / set-zero / clear-error)
    // -------------------------------------------------------------------

    pub fn enable(&self, can: &mut dyn CanTransport) -> Result<MotorState, Error> {
        self.cmd_motor_id(can, ENABLE_CMD)
    }

    pub fn disable(&self, can: &mut dyn CanTransport) -> Result<MotorState, Error> {
        self.cmd_motor_id(can, DISABLE_CMD)
    }

    pub fn set_zero(&self, can: &mut dyn CanTransport) -> Result<MotorState, Error> {
        self.cmd_motor_id(can, SET_ZERO_CMD)
    }

    /// Clear motor faults (overtemp, lost-comm, ...). Trailing byte is `0xFB`.
    pub fn clear_error(&self, can: &mut dyn CanTransport) -> Result<MotorState, Error> {
        self.cmd_motor_id(can, CLEAR_ERROR_CMD)
    }

    // -------------------------------------------------------------------
    // Control modes
    // -------------------------------------------------------------------

    /// MIT impedance setpoint (mode 1). Send only when register 10 is `1`.
    pub fn mit_control(
        &self,
        can: &mut dyn CanTransport,
        setpoint: &MitSetpoint,
    ) -> Result<MotorState, Error> {
        let payload = mit::encode(&self.limits, setpoint);
        self.cmd_motor_id(can, payload)
    }

    /// POS_VEL command (mode 2). Send only when register 10 is `2`.
    /// Frame goes to `0x100 + motor_id`.
    pub fn send_pos_vel(
        &self,
        can: &mut dyn CanTransport,
        position: f32,
        velocity: f32,
    ) -> Result<MotorState, Error> {
        let payload = modes::pos_vel_payload(position, velocity);
        self.cmd_at_arbitration(can, ControlMode::PosVel.arbitration_id(self.motor_id), payload)
    }

    /// VEL command (mode 3). Frame goes to `0x200 + motor_id`.
    pub fn send_vel(
        &self,
        can: &mut dyn CanTransport,
        velocity: f32,
    ) -> Result<MotorState, Error> {
        let payload = modes::vel_payload(velocity);
        self.cmd_at_arbitration(can, ControlMode::Vel.arbitration_id(self.motor_id), payload)
    }

    /// FORCE_POS command (mode 4). Frame goes to `0x300 + motor_id`.
    ///
    /// `velocity_limit` is clamped to `[0, 100]` rad/s before encoding;
    /// `torque_ratio` is clamped to `[0, 1]` and scales `t_max` from the
    /// motor's limits.
    pub fn send_force_pos(
        &self,
        can: &mut dyn CanTransport,
        position: f32,
        velocity_limit: f32,
        torque_ratio: f32,
    ) -> Result<MotorState, Error> {
        let payload = modes::force_pos_payload(position, velocity_limit, torque_ratio);
        self.cmd_at_arbitration(can, ControlMode::ForcePos.arbitration_id(self.motor_id), payload)
    }

    /// Unified entry point that dispatches to [`mit_control`], [`send_pos_vel`],
    /// [`send_vel`], or [`send_force_pos`] based on the [`ControlSetpoint`]
    /// variant. Lets callers switch control modes at the call site without
    /// branching on mode themselves.
    ///
    /// The motor must already be in the matching control mode (register 10);
    /// pair with [`ensure_control_mode`] when switching modes at runtime.
    ///
    /// [`mit_control`]: Self::mit_control
    /// [`send_pos_vel`]: Self::send_pos_vel
    /// [`send_vel`]: Self::send_vel
    /// [`send_force_pos`]: Self::send_force_pos
    /// [`ensure_control_mode`]: Self::ensure_control_mode
    pub fn control(
        &self,
        can: &mut dyn CanTransport,
        setpoint: &ControlSetpoint,
    ) -> Result<MotorState, Error> {
        match setpoint {
            ControlSetpoint::Mit(sp) => self.mit_control(can, sp),
            ControlSetpoint::PosVel { position, velocity } => {
                self.send_pos_vel(can, *position, *velocity)
            }
            ControlSetpoint::Vel(velocity) => self.send_vel(can, *velocity),
            ControlSetpoint::ForcePos {
                position,
                velocity_limit,
                torque_ratio,
            } => self.send_force_pos(can, *position, *velocity_limit, *torque_ratio),
        }
    }

    // -------------------------------------------------------------------
    // Register protocol (broadcast id 0x7FF)
    // -------------------------------------------------------------------

    /// Read a register value. Sends a `0x33` request and waits for the
    /// matching reply (matched by `D[2]==0x33 && D[3]==rid`).
    pub fn read_register(
        &self,
        can: &mut dyn CanTransport,
        rid: u8,
    ) -> Result<RegisterValue, Error> {
        let info = registers::lookup(rid).ok_or(Error::UnknownRegister(rid))?;
        let frame = self.register_frame(self.read_request_payload(rid));
        can.send(&frame).map_err(Error::Transport)?;

        let value_bytes = self.recv_register_reply(can, rid)?;
        Ok(RegisterValue::from_le_bytes(info.data_type, value_bytes))
    }

    /// Write a register value. The variant of `value` must match the
    /// register's declared `data_type`.
    pub fn write_register(
        &self,
        can: &mut dyn CanTransport,
        rid: u8,
        value: RegisterValue,
    ) -> Result<(), Error> {
        let info = registers::lookup(rid).ok_or(Error::UnknownRegister(rid))?;
        if info.access != registers::Access::ReadWrite {
            return Err(Error::ReadOnlyRegister(rid));
        }
        if value.data_type() != info.data_type {
            return Err(Error::TypeMismatch {
                rid,
                expected: info.data_type,
                got: value.data_type(),
            });
        }
        let frame = self.register_frame(self.write_request_payload(rid, value.to_le_bytes()));
        can.send(&frame).map_err(Error::Transport)?;
        Ok(())
    }

    /// Persist current parameter values to flash. `D[2]=0xAA, D[3]=0x01`.
    pub fn store_parameters(&self, can: &mut dyn CanTransport) -> Result<(), Error> {
        let frame = self.register_frame(self.store_payload());
        can.send(&frame).map_err(Error::Transport)?;
        Ok(())
    }

    /// Ask the motor to send its current MIT-format status frame on the
    /// master id. `D[2]=0xCC`.
    pub fn request_feedback(&self, can: &mut dyn CanTransport) -> Result<MotorState, Error> {
        let frame = self.register_frame(self.feedback_request_payload());
        let reply = can
            .request(&frame, CanId::standard(self.master_id), self.timeout)
            .map_err(Error::Transport)?;
        Ok(mit::decode_state(&self.limits, &reply.data))
    }

    // -------------------------------------------------------------------
    // Higher-level helpers built on register I/O
    // -------------------------------------------------------------------

    /// Read register 10, write+verify if the motor isn't already in `mode`.
    pub fn ensure_control_mode(
        &self,
        can: &mut dyn CanTransport,
        mode: ControlMode,
    ) -> Result<(), Error> {
        let current = self.read_register(can, 10)?.as_u32().ok_or_else(|| {
            Error::TypeMismatch {
                rid: 10,
                expected: DataType::Uint32,
                got: DataType::Float,
            }
        })?;

        let desired = mode as u32;
        if current == desired {
            return Ok(());
        }
        self.write_register(can, 10, RegisterValue::Uint32(desired))?;
        std::thread::sleep(CONTROL_MODE_WRITE_DELAY);

        let verify = self.read_register(can, 10)?.as_u32().ok_or_else(|| {
            Error::TypeMismatch {
                rid: 10,
                expected: DataType::Uint32,
                got: DataType::Float,
            }
        })?;
        if verify != desired {
            return Err(Error::ControlModeVerifyFailed {
                wrote: desired as u8,
                got: verify as u8,
            });
        }
        Ok(())
    }

    /// Set CAN bitrate (register 35) and persist to flash. The new rate
    /// only takes effect after the motor reboots.
    pub fn set_can_baud_rate(
        &self,
        can: &mut dyn CanTransport,
        baud: BaudRate,
    ) -> Result<(), Error> {
        self.write_register(can, 35, RegisterValue::Uint32(baud as u32))?;
        self.store_parameters(can)?;
        Ok(())
    }

    /// Set the timeout-alarm (register 9) in milliseconds. Saturates at
    /// `u32::MAX` register units (1 unit = 50 µs).
    pub fn set_can_timeout(
        &self,
        can: &mut dyn CanTransport,
        timeout_ms: u32,
    ) -> Result<(), Error> {
        let units = (timeout_ms as u64).saturating_mul(20).min(u32::MAX as u64) as u32;
        self.write_register(can, 9, RegisterValue::Uint32(units))
    }

    // -------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------

    /// Send `payload` on the motor's standard id, wait for the master-id reply.
    fn cmd_motor_id(
        &self,
        can: &mut dyn CanTransport,
        payload: [u8; 8],
    ) -> Result<MotorState, Error> {
        self.cmd_at_arbitration(can, self.motor_id, payload)
    }

    /// Send `payload` on `arbitration_id`, wait for the master-id reply.
    fn cmd_at_arbitration(
        &self,
        can: &mut dyn CanTransport,
        arbitration_id: u16,
        payload: [u8; 8],
    ) -> Result<MotorState, Error> {
        let frame = CanFrame::classic(CanId::standard(arbitration_id), payload);
        let reply = can
            .request(&frame, CanId::standard(self.master_id), self.timeout)
            .map_err(Error::Transport)?;
        Ok(mit::decode_state(&self.limits, &reply.data))
    }

    fn register_frame(&self, payload: [u8; 8]) -> CanFrame {
        CanFrame::classic(CanId::standard(REGISTER_BROADCAST_ID), payload)
    }

    fn read_request_payload(&self, rid: u8) -> [u8; 8] {
        [
            (self.motor_id & 0xFF) as u8,
            ((self.motor_id >> 8) & 0xFF) as u8,
            READ_CMD,
            rid,
            0, 0, 0, 0,
        ]
    }

    fn write_request_payload(&self, rid: u8, value: [u8; 4]) -> [u8; 8] {
        [
            (self.motor_id & 0xFF) as u8,
            ((self.motor_id >> 8) & 0xFF) as u8,
            WRITE_CMD,
            rid,
            value[0], value[1], value[2], value[3],
        ]
    }

    fn store_payload(&self) -> [u8; 8] {
        [
            (self.motor_id & 0xFF) as u8,
            ((self.motor_id >> 8) & 0xFF) as u8,
            STORE_CMD,
            0x01,
            0, 0, 0, 0,
        ]
    }

    fn feedback_request_payload(&self) -> [u8; 8] {
        [
            (self.motor_id & 0xFF) as u8,
            ((self.motor_id >> 8) & 0xFF) as u8,
            FEEDBACK_REQ_CMD,
            0x00,
            0, 0, 0, 0,
        ]
    }

    /// Drain frames from `can` until one matches the register-reply pattern
    /// for `rid` (`D[2]==0x33 && D[3]==rid`); return the 4 value bytes.
    fn recv_register_reply(
        &self,
        can: &mut dyn CanTransport,
        rid: u8,
    ) -> Result<[u8; 4], Error> {
        let deadline = Instant::now() + self.timeout;
        loop {
            let remaining = deadline
                .checked_duration_since(Instant::now())
                .ok_or(Error::Transport(CanError::Timeout))?;
            let frame = can.recv(remaining).map_err(Error::Transport)?;
            if frame.data.len() == 8 && frame.data[2] == READ_CMD && frame.data[3] == rid {
                let mut out = [0u8; 4];
                out.copy_from_slice(&frame.data[4..8]);
                return Ok(out);
            }
            // Other frames (status replies, other register replies) ignored.
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::MockTransport;

    fn empty_status_frame(master_id: u16) -> CanFrame {
        CanFrame::classic(CanId::standard(master_id), vec![0u8; 8])
    }

    #[test]
    fn enable_sends_ff_fc_to_motor_id() {
        let (mut host, mut motor) = MockTransport::pair();
        let dm = Damiao::new(0x01, 0x11);
        motor.send(&empty_status_frame(0x11)).unwrap();

        dm.enable(&mut host).unwrap();

        let sent = motor.peek().unwrap();
        assert_eq!(sent.id, CanId::Standard(0x01));
        assert_eq!(sent.data, ENABLE_CMD.to_vec());
    }

    #[test]
    fn pos_vel_targets_0x100_plus_motor_id() {
        let (mut host, mut motor) = MockTransport::pair();
        let dm = Damiao::new(0x05, 0x15);
        motor.send(&empty_status_frame(0x15)).unwrap();

        dm.send_pos_vel(&mut host, 1.5, 0.5).unwrap();

        let sent = motor.peek().unwrap();
        assert_eq!(sent.id, CanId::Standard(0x105));
        assert_eq!(&sent.data[0..4], &1.5_f32.to_le_bytes());
        assert_eq!(&sent.data[4..8], &0.5_f32.to_le_bytes());
    }

    #[test]
    fn vel_targets_0x200_plus_motor_id() {
        let (mut host, mut motor) = MockTransport::pair();
        let dm = Damiao::new(0x07, 0x17);
        motor.send(&empty_status_frame(0x17)).unwrap();

        dm.send_vel(&mut host, -2.5).unwrap();

        let sent = motor.peek().unwrap();
        assert_eq!(sent.id, CanId::Standard(0x207));
        assert_eq!(&sent.data[0..4], &(-2.5_f32).to_le_bytes());
        assert_eq!(&sent.data[4..8], &[0u8; 4]);
    }

    #[test]
    fn force_pos_targets_0x300_and_scales_correctly() {
        let (mut host, mut motor) = MockTransport::pair();
        let dm = Damiao::new(0x02, 0x12);
        motor.send(&empty_status_frame(0x12)).unwrap();

        dm.send_force_pos(&mut host, 0.0, 50.0, 0.5).unwrap();

        let sent = motor.peek().unwrap();
        assert_eq!(sent.id, CanId::Standard(0x302));
        let v = u16::from_le_bytes([sent.data[4], sent.data[5]]);
        let r = u16::from_le_bytes([sent.data[6], sent.data[7]]);
        assert_eq!(v, 5_000); // 50 rad/s * 100
        assert_eq!(r, 5_000); // 0.5 * 10000
    }

    #[test]
    fn read_register_decodes_uint32_reply() {
        let (mut host, mut motor) = MockTransport::pair();
        let dm = Damiao::new(0x01, 0x11);

        // Pre-stage register-10 reply (CTRL_MODE = uint32).
        let value = 2u32; // POS_VEL
        let mut reply_data = [0u8; 8];
        reply_data[0] = (dm.motor_id & 0xFF) as u8;
        reply_data[1] = ((dm.motor_id >> 8) & 0xFF) as u8;
        reply_data[2] = READ_CMD;
        reply_data[3] = 10;
        reply_data[4..8].copy_from_slice(&value.to_le_bytes());
        motor
            .send(&CanFrame::classic(CanId::standard(REGISTER_BROADCAST_ID), reply_data.to_vec()))
            .unwrap();

        let v = dm.read_register(&mut host, 10).unwrap();
        assert_eq!(v, RegisterValue::Uint32(2));

        // Verify the request that went out.
        let sent = motor.peek().unwrap();
        assert_eq!(sent.id, CanId::Standard(REGISTER_BROADCAST_ID));
        assert_eq!(sent.data[2], READ_CMD);
        assert_eq!(sent.data[3], 10);
    }

    #[test]
    fn write_register_rejects_read_only() {
        let (mut host, _motor) = MockTransport::pair();
        let dm = Damiao::new(0x01, 0x11);
        // Register 80 (p_m) is RO.
        let err = dm
            .write_register(&mut host, 80, RegisterValue::Float(0.0))
            .unwrap_err();
        assert!(matches!(err, Error::ReadOnlyRegister(80)));
    }

    #[test]
    fn write_register_rejects_type_mismatch() {
        let (mut host, _motor) = MockTransport::pair();
        let dm = Damiao::new(0x01, 0x11);
        // Register 10 is Uint32; passing Float must fail.
        let err = dm
            .write_register(&mut host, 10, RegisterValue::Float(2.0))
            .unwrap_err();
        assert!(matches!(err, Error::TypeMismatch { rid: 10, .. }));
    }

    #[test]
    fn store_parameters_uses_aa_01() {
        let (mut host, motor) = MockTransport::pair();
        let dm = Damiao::new(0x03, 0x13);

        dm.store_parameters(&mut host).unwrap();

        let sent = motor.peek().unwrap();
        assert_eq!(sent.id, CanId::Standard(REGISTER_BROADCAST_ID));
        assert_eq!(sent.data[0], 0x03);
        assert_eq!(sent.data[1], 0x00);
        assert_eq!(sent.data[2], STORE_CMD);
        assert_eq!(sent.data[3], 0x01);
    }

    #[test]
    fn set_can_timeout_converts_ms_to_50us_units() {
        let (mut host, motor) = MockTransport::pair();
        let dm = Damiao::new(0x01, 0x11);

        dm.set_can_timeout(&mut host, 5).unwrap(); // 5 ms → 100 register units

        let sent = motor.peek().unwrap();
        assert_eq!(sent.id, CanId::Standard(REGISTER_BROADCAST_ID));
        assert_eq!(sent.data[2], WRITE_CMD);
        assert_eq!(sent.data[3], 9);
        let units = u32::from_le_bytes([sent.data[4], sent.data[5], sent.data[6], sent.data[7]]);
        assert_eq!(units, 100);
    }

    #[test]
    fn mit_control_routes_through_shared_codec() {
        let (mut host, mut motor) = MockTransport::pair();
        let dm = Damiao::new(0x02, 0x12);
        motor.send(&empty_status_frame(0x12)).unwrap();

        let sp = MitSetpoint { q: 0.5, dq: 0.0, kp: 50.0, kd: 1.0, tau: 0.0 };
        dm.mit_control(&mut host, &sp).unwrap();

        let sent = motor.peek().unwrap();
        assert_eq!(sent.id, CanId::Standard(0x02));
        assert_eq!(sent.data, mit::encode(&dm.limits, &sp).to_vec());
    }

    #[test]
    fn control_dispatches_by_setpoint_variant() {
        // MIT → motor_id arbitration.
        let (mut host, mut motor) = MockTransport::pair();
        let dm = Damiao::new(0x04, 0x14);
        motor.send(&empty_status_frame(0x14)).unwrap();
        let sp_mit = MitSetpoint { q: 0.1, dq: 0.0, kp: 30.0, kd: 1.0, tau: 0.0 };
        dm.control(&mut host, &ControlSetpoint::Mit(sp_mit)).unwrap();
        assert_eq!(motor.peek().unwrap().id, CanId::Standard(0x04));

        // POS_VEL → 0x100 + motor_id arbitration.
        let (mut host, mut motor) = MockTransport::pair();
        let dm = Damiao::new(0x04, 0x14);
        motor.send(&empty_status_frame(0x14)).unwrap();
        dm.control(
            &mut host,
            &ControlSetpoint::PosVel { position: 0.5, velocity: 1.0 },
        )
        .unwrap();
        assert_eq!(motor.peek().unwrap().id, CanId::Standard(0x104));

        // VEL → 0x200 + motor_id arbitration.
        let (mut host, mut motor) = MockTransport::pair();
        let dm = Damiao::new(0x04, 0x14);
        motor.send(&empty_status_frame(0x14)).unwrap();
        dm.control(&mut host, &ControlSetpoint::Vel(2.0)).unwrap();
        assert_eq!(motor.peek().unwrap().id, CanId::Standard(0x204));

        // FORCE_POS → 0x300 + motor_id arbitration.
        let (mut host, mut motor) = MockTransport::pair();
        let dm = Damiao::new(0x04, 0x14);
        motor.send(&empty_status_frame(0x14)).unwrap();
        dm.control(
            &mut host,
            &ControlSetpoint::ForcePos {
                position: 0.0,
                velocity_limit: 10.0,
                torque_ratio: 0.5,
            },
        )
        .unwrap();
        assert_eq!(motor.peek().unwrap().id, CanId::Standard(0x304));
    }
}
