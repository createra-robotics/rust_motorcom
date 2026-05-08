use crate::robstride::mit::MitLimits;

/// RS-03: position ±4π rad, velocity ±50 rad/s, torque ±60 N·m, kp 5000, kd 100.
pub const RS03: MitLimits = MitLimits {
    p_max: 4.0 * std::f32::consts::PI,
    v_max: 50.0,
    t_max: 60.0,
    kp_max: 5000.0,
    kd_max: 100.0,
};
