use crate::damiao::mit::MitLimits;

/// DM-H3510: position ±12.5 rad, velocity ±280 rad/s, torque ±1 N·m.
pub const DMH3510: MitLimits = MitLimits {
    p_max: 12.5,
    v_max: 280.0,
    kp_max: 500.0,
    kd_max: 5.0,
    t_max: 1.0,
};
