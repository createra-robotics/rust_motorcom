use crate::robstride::mit::MitLimits;

/// RS-06: position ±4π rad, velocity ±20 rad/s, torque ±60 N·m, kp 5000, kd 100.
pub const RS06: MitLimits = MitLimits {
    p_max: 4.0 * std::f32::consts::PI,
    v_max: 20.0,
    t_max: 60.0,
    kp_max: 5000.0,
    kd_max: 100.0,
};
