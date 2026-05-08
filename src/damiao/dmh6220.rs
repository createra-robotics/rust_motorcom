use crate::damiao::mit::MitLimits;

/// DM-H6220: position ±12.5 rad, velocity ±45 rad/s, torque ±10 N·m.
pub const DMH6220: MitLimits = MitLimits {
    p_max: 12.5,
    v_max: 45.0,
    kp_max: 500.0,
    kd_max: 5.0,
    t_max: 10.0,
};
