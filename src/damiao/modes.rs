//! Damiao control modes and their wire payloads.
//!
//! Mode is selected by writing register 10 (`CTRL_MODE`). Each mode also
//! uses a different CAN arbitration id for its command frame:
//!
//! | Mode      | reg 10 | arbitration id        |
//! |-----------|--------|-----------------------|
//! | MIT       | 1      | `motor_id`            |
//! | POS_VEL   | 2      | `0x100 + motor_id`    |
//! | VEL       | 3      | `0x200 + motor_id`    |
//! | FORCE_POS | 4      | `0x300 + motor_id`    |
//!
//! Replies in every mode arrive on the configured master id with the
//! standard MIT-format status frame (see [`crate::damiao::mit`]).

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ControlMode {
    Mit      = 1,
    PosVel   = 2,
    Vel      = 3,
    ForcePos = 4,
}

impl ControlMode {
    /// Decode register-10 readback.
    pub fn from_register(value: u32) -> Option<Self> {
        match value {
            1 => Some(Self::Mit),
            2 => Some(Self::PosVel),
            3 => Some(Self::Vel),
            4 => Some(Self::ForcePos),
            _ => None,
        }
    }

    /// CAN arbitration id used for command frames in this mode.
    pub fn arbitration_id(&self, motor_id: u16) -> u16 {
        match self {
            Self::Mit      => motor_id,
            Self::PosVel   => 0x100 + motor_id,
            Self::Vel      => 0x200 + motor_id,
            Self::ForcePos => 0x300 + motor_id,
        }
    }
}

/// POS_VEL payload: little-endian f32 position + f32 velocity.
pub fn pos_vel_payload(position: f32, velocity: f32) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0..4].copy_from_slice(&position.to_le_bytes());
    out[4..8].copy_from_slice(&velocity.to_le_bytes());
    out
}

/// VEL payload: little-endian f32 velocity + 4 padding zeros.
pub fn vel_payload(velocity: f32) -> [u8; 8] {
    let mut out = [0u8; 8];
    out[0..4].copy_from_slice(&velocity.to_le_bytes());
    out
}

/// FORCE_POS payload: little-endian f32 position + u16 v_des_scaled + u16 i_des_scaled.
///
/// * `velocity_limit` is clamped to `[0, 100]` rad/s and scaled by 100.
/// * `torque_ratio` is clamped to `[0, 1]` and scaled by 10000.
pub fn force_pos_payload(position: f32, velocity_limit: f32, torque_ratio: f32) -> [u8; 8] {
    let v = velocity_limit.clamp(0.0, 100.0);
    let v_scaled = ((v * 100.0) as u32).min(10_000) as u16;

    let r = torque_ratio.clamp(0.0, 1.0);
    let r_scaled = ((r * 10_000.0) as u32).min(10_000) as u16;

    let mut out = [0u8; 8];
    out[0..4].copy_from_slice(&position.to_le_bytes());
    out[4..6].copy_from_slice(&v_scaled.to_le_bytes());
    out[6..8].copy_from_slice(&r_scaled.to_le_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arbitration_offsets() {
        assert_eq!(ControlMode::Mit.arbitration_id(0x05), 0x05);
        assert_eq!(ControlMode::PosVel.arbitration_id(0x05), 0x105);
        assert_eq!(ControlMode::Vel.arbitration_id(0x05), 0x205);
        assert_eq!(ControlMode::ForcePos.arbitration_id(0x05), 0x305);
    }

    #[test]
    fn pos_vel_layout_is_little_endian_pos_then_vel() {
        let bytes = pos_vel_payload(1.0_f32, -2.0_f32);
        assert_eq!(&bytes[0..4], &1.0_f32.to_le_bytes());
        assert_eq!(&bytes[4..8], &(-2.0_f32).to_le_bytes());
    }

    #[test]
    fn vel_payload_zero_pads_tail() {
        let bytes = vel_payload(3.5_f32);
        assert_eq!(&bytes[0..4], &3.5_f32.to_le_bytes());
        assert_eq!(&bytes[4..8], &[0, 0, 0, 0]);
    }

    #[test]
    fn force_pos_clamps_velocity_and_ratio() {
        // velocity_limit > 100 → clamped to 100 → 10000
        // torque_ratio > 1.0 → clamped to 1.0 → 10000
        let bytes = force_pos_payload(0.0, 250.0, 5.0);
        let v_u16 = u16::from_le_bytes([bytes[4], bytes[5]]);
        let r_u16 = u16::from_le_bytes([bytes[6], bytes[7]]);
        assert_eq!(v_u16, 10_000);
        assert_eq!(r_u16, 10_000);
    }
}
