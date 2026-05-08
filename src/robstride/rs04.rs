use crate::robstride::mit::MitLimits;

/// RS-04: position ±4π rad, velocity ±15 rad/s, torque ±120 N·m, kp 5000, kd 100.
pub const RS04: MitLimits = MitLimits {
    p_max: 4.0 * std::f32::consts::PI,
    v_max: 15.0,
    t_max: 120.0,
    kp_max: 5000.0,
    kd_max: 100.0,
};
