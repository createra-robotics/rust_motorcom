//! Damiao register table (parameter ids 0–81).
//!
//! Register frames are addressed to the broadcast id `0x7FF`; the target
//! motor id is packed into the first two bytes of the payload. The third
//! byte is the command type — `0x33` read, `0x55` write, `0xAA` save to
//! flash, `0xCC` request feedback. Read replies echo `0x33` in byte 2 and
//! the register id in byte 3, with the 4-byte little-endian value in
//! bytes 4–7.

/// Register access mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    ReadOnly,
    ReadWrite,
}

/// Register payload type. Damiao uses only 32-bit float and uint32.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataType {
    Float,
    Uint32,
}

/// One row of the manufacturer-supplied register table.
#[derive(Debug, Clone, Copy)]
pub struct RegisterInfo {
    pub rid: u8,
    pub name: &'static str,
    pub description: &'static str,
    pub access: Access,
    pub data_type: DataType,
}

/// A typed register value, used by both [`crate::damiao::Damiao::read_register`]
/// and [`crate::damiao::Damiao::write_register`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RegisterValue {
    Float(f32),
    Uint32(u32),
}

impl RegisterValue {
    pub fn data_type(&self) -> DataType {
        match self {
            Self::Float(_) => DataType::Float,
            Self::Uint32(_) => DataType::Uint32,
        }
    }

    pub fn to_le_bytes(&self) -> [u8; 4] {
        match self {
            Self::Float(v) => v.to_le_bytes(),
            Self::Uint32(v) => v.to_le_bytes(),
        }
    }

    pub fn from_le_bytes(dtype: DataType, bytes: [u8; 4]) -> Self {
        match dtype {
            DataType::Float => Self::Float(f32::from_le_bytes(bytes)),
            DataType::Uint32 => Self::Uint32(u32::from_le_bytes(bytes)),
        }
    }

    pub fn as_f32(&self) -> Option<f32> {
        match self {
            Self::Float(v) => Some(*v),
            _ => None,
        }
    }

    pub fn as_u32(&self) -> Option<u32> {
        match self {
            Self::Uint32(v) => Some(*v),
            _ => None,
        }
    }
}

/// CAN bitrate codes accepted by register 35 (`can_br`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BaudRate {
    K125 = 0,
    K200 = 1,
    K250 = 2,
    K500 = 3,
    M1   = 4,
}

impl BaudRate {
    pub fn bps(&self) -> u32 {
        match self {
            Self::K125 => 125_000,
            Self::K200 => 200_000,
            Self::K250 => 250_000,
            Self::K500 => 500_000,
            Self::M1   => 1_000_000,
        }
    }
}

const RW: Access = Access::ReadWrite;
const RO: Access = Access::ReadOnly;
const F: DataType = DataType::Float;
const U: DataType = DataType::Uint32;

/// Manufacturer register table.
pub const REGISTER_TABLE: &[RegisterInfo] = &[
    // Protection and basic parameters (0–6)
    RegisterInfo { rid: 0,  name: "UV_Value", description: "Under-voltage protection value", access: RW, data_type: F },
    RegisterInfo { rid: 1,  name: "KT_Value", description: "Torque coefficient",             access: RW, data_type: F },
    RegisterInfo { rid: 2,  name: "OT_Value", description: "Over-temperature protection",    access: RW, data_type: F },
    RegisterInfo { rid: 3,  name: "OC_Value", description: "Over-current protection",        access: RW, data_type: F },
    RegisterInfo { rid: 4,  name: "ACC",      description: "Acceleration",                   access: RW, data_type: F },
    RegisterInfo { rid: 5,  name: "DEC",      description: "Deceleration",                   access: RW, data_type: F },
    RegisterInfo { rid: 6,  name: "MAX_SPD",  description: "Maximum speed",                  access: RW, data_type: F },
    // System identification and configuration (7–10)
    RegisterInfo { rid: 7,  name: "MST_ID",    description: "Feedback (master) CAN id",       access: RW, data_type: U },
    RegisterInfo { rid: 8,  name: "ESC_ID",    description: "Receive (motor) CAN id",          access: RW, data_type: U },
    RegisterInfo { rid: 9,  name: "TIMEOUT",   description: "Timeout alarm (1 unit = 50 us)",  access: RW, data_type: U },
    RegisterInfo { rid: 10, name: "CTRL_MODE", description: "Control mode 1=MIT 2=POS_VEL 3=VEL 4=FORCE_POS", access: RW, data_type: U },
    // Motor physical parameters (11–20) — read-only
    RegisterInfo { rid: 11, name: "Damp",     description: "Viscous damping coefficient",    access: RO, data_type: F },
    RegisterInfo { rid: 12, name: "Inertia",  description: "Moment of inertia",              access: RO, data_type: F },
    RegisterInfo { rid: 13, name: "hw_ver",   description: "Hardware version (reserved)",    access: RO, data_type: U },
    RegisterInfo { rid: 14, name: "sw_ver",   description: "Software version",               access: RO, data_type: U },
    RegisterInfo { rid: 15, name: "SN",       description: "Serial number (reserved)",       access: RO, data_type: U },
    RegisterInfo { rid: 16, name: "NPP",      description: "Motor pole pairs",               access: RO, data_type: U },
    RegisterInfo { rid: 17, name: "Rs",       description: "Phase resistance",               access: RO, data_type: F },
    RegisterInfo { rid: 18, name: "Ls",       description: "Phase inductance",               access: RO, data_type: F },
    RegisterInfo { rid: 19, name: "Flux",     description: "Flux linkage",                   access: RO, data_type: F },
    RegisterInfo { rid: 20, name: "Gr",       description: "Gear reduction ratio",           access: RO, data_type: F },
    // Mapping ranges (21–23)
    RegisterInfo { rid: 21, name: "PMAX", description: "Position mapping range", access: RW, data_type: F },
    RegisterInfo { rid: 22, name: "VMAX", description: "Speed mapping range",    access: RW, data_type: F },
    RegisterInfo { rid: 23, name: "TMAX", description: "Torque mapping range",   access: RW, data_type: F },
    // Control loop parameters (24–28)
    RegisterInfo { rid: 24, name: "I_BW",   description: "Current loop bandwidth", access: RW, data_type: F },
    RegisterInfo { rid: 25, name: "KP_ASR", description: "Speed loop Kp",          access: RW, data_type: F },
    RegisterInfo { rid: 26, name: "KI_ASR", description: "Speed loop Ki",          access: RW, data_type: F },
    RegisterInfo { rid: 27, name: "KP_APR", description: "Position loop Kp",       access: RW, data_type: F },
    RegisterInfo { rid: 28, name: "KI_APR", description: "Position loop Ki",       access: RW, data_type: F },
    // Protection and efficiency (29–32)
    RegisterInfo { rid: 29, name: "OV_Value", description: "Over-voltage protection",   access: RW, data_type: F },
    RegisterInfo { rid: 30, name: "GREF",     description: "Gear torque efficiency",     access: RW, data_type: F },
    RegisterInfo { rid: 31, name: "Deta",     description: "Speed loop damping",         access: RW, data_type: F },
    RegisterInfo { rid: 32, name: "V_BW",     description: "Speed loop filter bandwidth",access: RW, data_type: F },
    // Enhancement coefficients (33–34)
    RegisterInfo { rid: 33, name: "IQ_c1", description: "Current loop enhancement", access: RW, data_type: F },
    RegisterInfo { rid: 34, name: "VL_c1", description: "Speed loop enhancement",   access: RW, data_type: F },
    // CAN and version (35–36)
    RegisterInfo { rid: 35, name: "can_br",  description: "CAN baud rate code (0..4)", access: RW, data_type: U },
    RegisterInfo { rid: 36, name: "sub_ver", description: "Sub-version number",         access: RO, data_type: U },
    // Calibration parameters (50–56) — read-only
    RegisterInfo { rid: 50, name: "u_off", description: "U-phase offset",       access: RO, data_type: F },
    RegisterInfo { rid: 51, name: "v_off", description: "V-phase offset",       access: RO, data_type: F },
    RegisterInfo { rid: 52, name: "k1",    description: "Compensation factor 1",access: RO, data_type: F },
    RegisterInfo { rid: 53, name: "k2",    description: "Compensation factor 2",access: RO, data_type: F },
    RegisterInfo { rid: 54, name: "m_off", description: "Output-shaft angle offset", access: RO, data_type: F },
    RegisterInfo { rid: 55, name: "dir",   description: "Direction",            access: RO, data_type: F },
    RegisterInfo { rid: 56, name: "m_off_motor", description: "Motor-side angle offset", access: RO, data_type: F },
    // Motor and driver board parameters (59–65) — read-only
    RegisterInfo { rid: 59, name: "Imax",   description: "Driver board max current", access: RO, data_type: F },
    RegisterInfo { rid: 60, name: "VBus",   description: "Power supply voltage",     access: RO, data_type: F },
    RegisterInfo { rid: 61, name: "Tpcb",   description: "Driver board temperature", access: RO, data_type: F },
    RegisterInfo { rid: 62, name: "Tmtr",   description: "Motor temperature",        access: RO, data_type: F },
    RegisterInfo { rid: 63, name: "Iu_off", description: "U-phase current offset",   access: RO, data_type: F },
    RegisterInfo { rid: 64, name: "Iv_off", description: "V-phase current offset",   access: RO, data_type: F },
    RegisterInfo { rid: 65, name: "Iw_off", description: "W-phase current offset",   access: RO, data_type: F },
    // Position feedback (80–81) — read-only
    RegisterInfo { rid: 80, name: "p_m",  description: "Motor position",         access: RO, data_type: F },
    RegisterInfo { rid: 81, name: "xout", description: "Output shaft position",  access: RO, data_type: F },
];

/// Look up a register definition by id, or `None` if `rid` isn't in the
/// manufacturer table.
pub fn lookup(rid: u8) -> Option<&'static RegisterInfo> {
    REGISTER_TABLE.iter().find(|r| r.rid == rid)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_registers_lookup() {
        assert_eq!(lookup(0).unwrap().name, "UV_Value");
        assert_eq!(lookup(10).unwrap().name, "CTRL_MODE");
        assert_eq!(lookup(35).unwrap().data_type, DataType::Uint32);
        assert_eq!(lookup(81).unwrap().access, Access::ReadOnly);
    }

    #[test]
    fn gaps_return_none() {
        assert!(lookup(37).is_none());
        assert!(lookup(57).is_none());
        assert!(lookup(82).is_none());
    }

    #[test]
    fn register_value_roundtrips() {
        let v = RegisterValue::Float(1.5);
        assert_eq!(
            RegisterValue::from_le_bytes(DataType::Float, v.to_le_bytes()),
            v
        );
        let u = RegisterValue::Uint32(0xDEAD_BEEF);
        assert_eq!(
            RegisterValue::from_le_bytes(DataType::Uint32, u.to_le_bytes()),
            u
        );
    }

    #[test]
    fn baud_rate_codes_match_manufacturer_table() {
        assert_eq!(BaudRate::K125 as u8, 0);
        assert_eq!(BaudRate::M1 as u8, 4);
        assert_eq!(BaudRate::K500.bps(), 500_000);
    }
}
