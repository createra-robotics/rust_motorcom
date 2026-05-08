use crate::damiao::mit::MitLimits;

/// DM6248P: position ±12.566 rad, velocity ±20 rad/s, torque ±120 N·m.
pub const DM6248P: MitLimits = MitLimits {
    p_max: 12.566,
    v_max: 20.0,
    kp_max: 500.0,
    kd_max: 5.0,
    t_max: 120.0,
};
