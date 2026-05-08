use crate::damiao::mit::MitLimits;

/// DM10010: position ±12.5 rad, velocity ±20 rad/s, torque ±200 N·m.
pub const DM10010: MitLimits = MitLimits {
    p_max: 12.5,
    v_max: 20.0,
    kp_max: 500.0,
    kd_max: 5.0,
    t_max: 200.0,
};
