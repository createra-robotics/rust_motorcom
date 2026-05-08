use crate::robstride::mit::MitLimits;

/// RS-00: position ±4π rad, velocity ±50 rad/s, torque ±17 N·m, kp 500, kd 5.
pub const RS00: MitLimits = MitLimits {
    p_max: 4.0 * std::f32::consts::PI,
    v_max: 50.0,
    t_max: 17.0,
    kp_max: 500.0,
    kd_max: 5.0,
};
