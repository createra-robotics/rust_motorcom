use crate::damiao::mit::MitLimits;

/// DM4310: position ±12.5 rad, velocity ±30 rad/s, torque ±10 N·m.
pub const DM4310: MitLimits = MitLimits {
    p_max: 12.5,
    v_max: 30.0,
    kp_max: 500.0,
    kd_max: 5.0,
    t_max: 10.0,
};
