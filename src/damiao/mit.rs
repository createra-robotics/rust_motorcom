//! MIT impedance-control payload codec.
//!
//! The MIT cheetah-style command is a single 8-byte CAN frame carrying an
//! impedance-control setpoint:
//!
//! ```text
//! tau_cmd = kp * (q* - q) + kd * (dq* - dq) + tau_ff
//! ```
//!
//! Both Damiao DM-series and Robstride RS-series motors accept this exact
//! payload (the 16/12/12/12/12-bit packing below); the frame envelope —
//! classic CAN vs CAN-FD, standard vs extended id, enable/disable scheme —
//! is vendor-specific and lives in [`crate::damiao`] / `crate::robstride`.
//!
//! ## Wire format (8 bytes)
//!
//! | byte | bits  | field          |
//! |------|-------|----------------|
//! | 0    | 15..8 | q\*  high      |
//! | 1    | 7..0  | q\*  low       |
//! | 2    | 11..4 | dq\* high      |
//! | 3    | 3..0  | dq\* low (hi 4 of next field follow) |
//! | 3    | 11..8 | kp   high      |
//! | 4    | 7..0  | kp   low       |
//! | 5    | 11..4 | kd   high      |
//! | 6    | 3..0  | kd   low (hi 4 of next field follow) |
//! | 6    | 11..8 | tau  high      |
//! | 7    | 7..0  | tau  low       |
//!
//! Reply frames carry q, dq, tau plus motor id, error flags, and two
//! temperature bytes. See [`MotorState`].

/// Per-motor encoding limits. Values are clamped into `[-x_max, x_max]`
/// (or `[0, x_max]` for kp/kd) before quantization.
///
/// Defaults match Damiao DM4310. Override per motor variant — see your
/// motor's datasheet for the correct ranges.
#[derive(Debug, Clone, Copy)]
pub struct MitLimits {
    pub p_max: f32,  // rad
    pub v_max: f32,  // rad/s
    pub kp_max: f32,
    pub kd_max: f32,
    pub t_max: f32,  // N·m
}

impl Default for MitLimits {
    fn default() -> Self {
        // DM4310 — adjust per motor variant.
        Self { p_max: 12.5, v_max: 30.0, kp_max: 500.0, kd_max: 5.0, t_max: 10.0 }
    }
}

/// MIT impedance-control setpoint.
#[derive(Debug, Clone, Copy, Default)]
pub struct MitSetpoint {
    pub q: f32,    // rad
    pub dq: f32,   // rad/s
    pub kp: f32,
    pub kd: f32,
    pub tau: f32,  // N·m feedforward torque
}

/// Decoded reply frame (q, dq, tau in SI units).
#[derive(Debug, Clone, Copy)]
pub struct MotorState {
    pub motor_id: u8,
    pub error: u8,
    pub position: f32, // rad
    pub velocity: f32, // rad/s
    pub torque: f32,   // N·m
    pub t_mos: u8,     // °C, MOSFET temperature
    pub t_rotor: u8,   // °C, rotor temperature
}

/// Encode an MIT setpoint into the 8-byte payload.
pub fn encode(lim: &MitLimits, sp: &MitSetpoint) -> [u8; 8] {
    let q   = float_to_uint(sp.q,   -lim.p_max,  lim.p_max,  16);
    let dq  = float_to_uint(sp.dq,  -lim.v_max,  lim.v_max,  12);
    let kp  = float_to_uint(sp.kp,  0.0,         lim.kp_max, 12);
    let kd  = float_to_uint(sp.kd,  0.0,         lim.kd_max, 12);
    let tau = float_to_uint(sp.tau, -lim.t_max,  lim.t_max,  12);

    [
        (q  >> 8)        as u8,
        (q  & 0xFF)      as u8,
        (dq >> 4)        as u8,
        (((dq & 0xF) << 4) | ((kp >> 8) & 0xF)) as u8,
        (kp & 0xFF)      as u8,
        (kd >> 4)        as u8,
        (((kd & 0xF) << 4) | ((tau >> 8) & 0xF)) as u8,
        (tau & 0xFF)     as u8,
    ]
}

/// Decode an 8-byte motor reply into a [`MotorState`].
///
/// Panics in debug if `data.len() < 8`.
pub fn decode_state(lim: &MitLimits, data: &[u8]) -> MotorState {
    debug_assert!(data.len() >= 8, "MIT reply must be at least 8 bytes");
    let id    =  data[0]       & 0x0F;
    let err   = (data[0] >> 4) & 0x0F;
    let q_u   = ((data[1] as u32) << 8) | (data[2] as u32);
    let dq_u  = ((data[3] as u32) << 4) | ((data[4] as u32) >> 4);
    let tau_u = (((data[4] as u32) & 0x0F) << 8) | (data[5] as u32);
    MotorState {
        motor_id: id,
        error:    err,
        position: uint_to_float(q_u,  -lim.p_max, lim.p_max, 16),
        velocity: uint_to_float(dq_u, -lim.v_max, lim.v_max, 12),
        torque:   uint_to_float(tau_u, -lim.t_max, lim.t_max, 12),
        t_mos:    data[6],
        t_rotor:  data[7],
    }
}

fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u32) -> u32 {
    let span = x_max - x_min;
    let clamped = x.clamp(x_min, x_max);
    let scale = ((1u32 << bits) - 1) as f32;
    (((clamped - x_min) * scale) / span) as u32
}

fn uint_to_float(v: u32, x_min: f32, x_max: f32, bits: u32) -> f32 {
    let span = x_max - x_min;
    let scale = ((1u32 << bits) - 1) as f32;
    (v as f32) * span / scale + x_min
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q_roundtrips_within_quantization() {
        let lim = MitLimits::default();
        let sp = MitSetpoint { q: 1.234, dq: -2.5, kp: 50.0, kd: 1.0, tau: 0.5 };
        let bytes = encode(&lim, &sp);

        let q_u = ((bytes[0] as u32) << 8) | (bytes[1] as u32);
        let q = uint_to_float(q_u, -lim.p_max, lim.p_max, 16);
        assert!((q - 1.234).abs() < 1e-3, "q decoded = {q}");
    }

    #[test]
    fn decode_state_recovers_id_and_position() {
        let lim = MitLimits::default();
        // Build a reply that says id=3, err=0, q=0, dq=0, tau=0, temps=40/45.
        let q_zero  = float_to_uint(0.0, -lim.p_max, lim.p_max, 16);
        let dq_zero = float_to_uint(0.0, -lim.v_max, lim.v_max, 12);
        let t_zero  = float_to_uint(0.0, -lim.t_max, lim.t_max, 12);
        let data = [
            0x03,
            (q_zero  >> 8) as u8,
            (q_zero  & 0xFF) as u8,
            (dq_zero >> 4) as u8,
            (((dq_zero & 0xF) << 4) | ((t_zero >> 8) & 0xF)) as u8,
            (t_zero  & 0xFF) as u8,
            40,
            45,
        ];
        let st = decode_state(&lim, &data);
        assert_eq!(st.motor_id, 3);
        assert_eq!(st.error, 0);
        assert!(st.position.abs() < 1e-3);
        assert!(st.velocity.abs() < 1e-2);
        assert!(st.torque.abs() < 1e-2);
        assert_eq!(st.t_mos, 40);
        assert_eq!(st.t_rotor, 45);
    }

    #[test]
    fn encode_clamps_out_of_range_inputs() {
        let lim = MitLimits::default();
        let sp_hi = MitSetpoint { q: 1e6, ..Default::default() };
        let bytes = encode(&lim, &sp_hi);
        // q field is 16 bits, all-ones means clamped to +p_max.
        assert_eq!(bytes[0], 0xFF);
        assert_eq!(bytes[1], 0xFF);
    }
}
