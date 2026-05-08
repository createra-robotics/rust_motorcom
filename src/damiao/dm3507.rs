use crate::damiao::mit::MitLimits;

/// DM3507: position ±12.566 rad, velocity ±50 rad/s, torque ±5 N·m.
pub const DM3507: MitLimits = MitLimits {
    p_max: 12.566,
    v_max: 50.0,
    kp_max: 500.0,
    kd_max: 5.0,
    t_max: 5.0,
};
