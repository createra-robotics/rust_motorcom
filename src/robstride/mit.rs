//! Robstride OperationControl codec.
//!
//! Encodes the impedance setpoint sent with `communication_type = 1`
//! (`OPERATION_CONTROL`) and decodes the corresponding status reply
//! (`communication_type = 2`, `OPERATION_STATUS`).
//!
//! ## Wire format
//!
//! The Robstride frame uses a **29-bit extended id** with three packed
//! fields (see [`crate::robstride::frame`]):
//!
//! ```text
//!  bits 28..24: communication_type (5 bits)
//!  bits 23..8:  extra_data         (16 bits)
//!  bits  7..0:  device_id          (8 bits)
//! ```
//!
//! For `OPERATION_CONTROL`, the 16-bit torque setpoint is placed in
//! `extra_data`; the 8-byte data payload carries position/velocity/kp/kd
//! as four **big-endian** `u16`s:
//!
//! | bytes | field         |
//! |-------|---------------|
//! | 0..2  | position_u16  |
//! | 2..4  | velocity_u16  |
//! | 4..6  | kp_u16        |
//! | 6..8  | kd_u16        |
//!
//! ## Quantization
//!
//! Position, velocity, and torque use a **signed offset** mapping —
//! `u16 = ((x / x_max) + 1) · 0x7FFF`, so 0 maps to `0x7FFF`,
//! `+x_max` to `0xFFFE`, `-x_max` to `0`.
//!
//! KP and KD use **unsigned linear** mapping —
//! `u16 = (x / x_max) · 0xFFFF`.

/// Per-motor scaling limits. KP_MAX / KD_MAX vary across models
/// (small motors: 500 / 5; large motors: 5000 / 100).
#[derive(Debug, Clone, Copy)]
pub struct MitLimits {
    pub p_max: f32,   // PMAX, rad
    pub v_max: f32,   // VMAX, rad/s
    pub t_max: f32,   // TMAX, N·m
    pub kp_max: f32,
    pub kd_max: f32,
}

/// Impedance setpoint sent on every operation-control tick.
#[derive(Debug, Clone, Copy, Default)]
pub struct MitSetpoint {
    pub position: f32, // rad
    pub velocity: f32, // rad/s
    pub torque: f32,   // N·m feedforward
    pub kp: f32,
    pub kd: f32,
}

/// Encoded operation-control frame: `extra_data` for the arbitration id
/// and 8 data bytes for the payload.
#[derive(Debug, Clone, Copy)]
pub struct EncodedMit {
    pub torque_extra: u16,
    pub data: [u8; 8],
}

/// Decoded `OPERATION_STATUS` frame.
#[derive(Debug, Clone, Copy)]
pub struct MotorState {
    pub position: f32,    // rad
    pub velocity: f32,    // rad/s
    pub torque: f32,      // N·m
    pub temperature: f32, // °C, raw value × 0.1
}

pub fn encode(lim: &MitLimits, sp: &MitSetpoint) -> EncodedMit {
    let pos_u = signed_offset_u16(sp.position, lim.p_max);
    let vel_u = signed_offset_u16(sp.velocity, lim.v_max);
    let kp_u = unsigned_linear_u16(sp.kp.clamp(0.0, lim.kp_max), lim.kp_max);
    let kd_u = unsigned_linear_u16(sp.kd.clamp(0.0, lim.kd_max), lim.kd_max);
    let tau_u = signed_offset_u16(sp.torque, lim.t_max);

    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&pos_u.to_be_bytes());
    data[2..4].copy_from_slice(&vel_u.to_be_bytes());
    data[4..6].copy_from_slice(&kp_u.to_be_bytes());
    data[6..8].copy_from_slice(&kd_u.to_be_bytes());

    EncodedMit { torque_extra: tau_u, data }
}

pub fn decode_status(lim: &MitLimits, data: &[u8]) -> MotorState {
    debug_assert!(data.len() >= 8, "OPERATION_STATUS data must be 8 bytes");
    let pos_u = u16::from_be_bytes([data[0], data[1]]);
    let vel_u = u16::from_be_bytes([data[2], data[3]]);
    let tau_u = u16::from_be_bytes([data[4], data[5]]);
    let temp_u = u16::from_be_bytes([data[6], data[7]]);

    MotorState {
        position: signed_offset_to_float(pos_u, lim.p_max),
        velocity: signed_offset_to_float(vel_u, lim.v_max),
        torque:   signed_offset_to_float(tau_u, lim.t_max),
        temperature: (temp_u as f32) * 0.1,
    }
}

fn signed_offset_u16(x: f32, x_max: f32) -> u16 {
    let clamped = x.clamp(-x_max, x_max);
    let raw = ((clamped / x_max + 1.0) * 0x7FFF as f32) as i32;
    raw.clamp(0, 0xFFFF) as u16
}

fn signed_offset_to_float(u: u16, x_max: f32) -> f32 {
    ((u as f32) / 0x7FFF as f32 - 1.0) * x_max
}

fn unsigned_linear_u16(x: f32, x_max: f32) -> u16 {
    let raw = (x / x_max * 0xFFFF as f32) as i32;
    raw.clamp(0, 0xFFFF) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rs00_limits() -> MitLimits {
        MitLimits { p_max: 12.566_370_614, v_max: 50.0, t_max: 17.0, kp_max: 500.0, kd_max: 5.0 }
    }

    #[test]
    fn zero_position_maps_to_0x7fff() {
        let lim = rs00_limits();
        let sp = MitSetpoint { position: 0.0, ..Default::default() };
        let enc = encode(&lim, &sp);
        let pos_u = u16::from_be_bytes([enc.data[0], enc.data[1]]);
        assert_eq!(pos_u, 0x7FFF);
    }

    #[test]
    fn positive_max_position_saturates_top_end() {
        let lim = rs00_limits();
        let sp = MitSetpoint { position: lim.p_max, ..Default::default() };
        let enc = encode(&lim, &sp);
        let pos_u = u16::from_be_bytes([enc.data[0], enc.data[1]]);
        // (1+1) * 0x7FFF = 0xFFFE, clipped at 0xFFFF
        assert!(pos_u == 0xFFFE || pos_u == 0xFFFF);
    }

    #[test]
    fn kp_at_max_maps_to_0xffff() {
        let lim = rs00_limits();
        let sp = MitSetpoint { kp: lim.kp_max, ..Default::default() };
        let enc = encode(&lim, &sp);
        let kp_u = u16::from_be_bytes([enc.data[4], enc.data[5]]);
        assert_eq!(kp_u, 0xFFFF);
    }

    #[test]
    fn torque_goes_into_extra_data_not_data_bytes() {
        let lim = rs00_limits();
        let sp = MitSetpoint { torque: 0.0, ..Default::default() };
        let enc = encode(&lim, &sp);
        assert_eq!(enc.torque_extra, 0x7FFF);
        // Data tail (kd region) untouched by torque.
        let kd_u = u16::from_be_bytes([enc.data[6], enc.data[7]]);
        assert_eq!(kd_u, 0);
    }

    #[test]
    fn status_decode_recovers_zeros() {
        let lim = rs00_limits();
        let mut data = [0u8; 8];
        data[0..2].copy_from_slice(&0x7FFF_u16.to_be_bytes()); // pos = 0
        data[2..4].copy_from_slice(&0x7FFF_u16.to_be_bytes()); // vel = 0
        data[4..6].copy_from_slice(&0x7FFF_u16.to_be_bytes()); // tau = 0
        data[6..8].copy_from_slice(&250_u16.to_be_bytes());    // temp = 25.0 °C
        let st = decode_status(&lim, &data);
        assert!(st.position.abs() < 1e-3);
        assert!(st.velocity.abs() < 1e-2);
        assert!(st.torque.abs() < 1e-2);
        assert!((st.temperature - 25.0).abs() < 1e-3);
    }
}
