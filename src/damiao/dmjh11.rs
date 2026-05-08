use crate::damiao::mit::MitLimits;

/// DM-JH11: position ±12.5 rad, velocity ±10 rad/s, torque ±12 N·m.
pub const DMJH11: MitLimits = MitLimits {
    p_max: 12.5,
    v_max: 10.0,
    kp_max: 500.0,
    kd_max: 5.0,
    t_max: 12.0,
};
