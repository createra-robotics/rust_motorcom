//! Robstride parameter table (`READ_PARAMETER` / `WRITE_PARAMETER`).
//!
//! Each parameter is identified by a 16-bit id. Read/write frames carry
//! the id in bytes 0..2 of the data payload (little-endian); the value
//! occupies bytes 4..8 and is encoded per the parameter's `DataType`.
//!
//! Smaller types pad with zeros to fill 4 bytes:
//!
//! | DataType | layout (bytes 0..4 of value field)        |
//! |----------|-------------------------------------------|
//! | Uint8/Int8   | `[v, 0, 0, 0]`                        |
//! | Uint16/Int16 | `[v_lo, v_hi, 0, 0]` little-endian    |
//! | Uint32/Int32 | full 4 bytes little-endian            |
//! | Float32      | full 4 bytes little-endian            |

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataType {
    Uint8,
    Int8,
    Uint16,
    Int16,
    Uint32,
    Int32,
    Float32,
}

#[derive(Debug, Clone, Copy)]
pub struct ParameterDef {
    pub id: u16,
    pub name: &'static str,
    pub data_type: DataType,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParameterValue {
    Uint8(u8),
    Int8(i8),
    Uint16(u16),
    Int16(i16),
    Uint32(u32),
    Int32(i32),
    Float32(f32),
}

impl ParameterValue {
    pub fn data_type(&self) -> DataType {
        match self {
            Self::Uint8(_)   => DataType::Uint8,
            Self::Int8(_)    => DataType::Int8,
            Self::Uint16(_)  => DataType::Uint16,
            Self::Int16(_)   => DataType::Int16,
            Self::Uint32(_)  => DataType::Uint32,
            Self::Int32(_)   => DataType::Int32,
            Self::Float32(_) => DataType::Float32,
        }
    }

    pub fn to_le_bytes(&self) -> [u8; 4] {
        let mut out = [0u8; 4];
        match self {
            Self::Uint8(v)   => out[0] = *v,
            Self::Int8(v)    => out[0] = *v as u8,
            Self::Uint16(v)  => out[0..2].copy_from_slice(&v.to_le_bytes()),
            Self::Int16(v)   => out[0..2].copy_from_slice(&v.to_le_bytes()),
            Self::Uint32(v)  => out.copy_from_slice(&v.to_le_bytes()),
            Self::Int32(v)   => out.copy_from_slice(&v.to_le_bytes()),
            Self::Float32(v) => out.copy_from_slice(&v.to_le_bytes()),
        }
        out
    }

    pub fn from_le_bytes(dtype: DataType, bytes: [u8; 4]) -> Self {
        match dtype {
            DataType::Uint8   => Self::Uint8(bytes[0]),
            DataType::Int8    => Self::Int8(bytes[0] as i8),
            DataType::Uint16  => Self::Uint16(u16::from_le_bytes([bytes[0], bytes[1]])),
            DataType::Int16   => Self::Int16(i16::from_le_bytes([bytes[0], bytes[1]])),
            DataType::Uint32  => Self::Uint32(u32::from_le_bytes(bytes)),
            DataType::Int32   => Self::Int32(i32::from_le_bytes(bytes)),
            DataType::Float32 => Self::Float32(f32::from_le_bytes(bytes)),
        }
    }

    pub fn as_f32(&self) -> Option<f32> {
        if let Self::Float32(v) = self { Some(*v) } else { None }
    }
    pub fn as_i8(&self) -> Option<i8> {
        if let Self::Int8(v) = self { Some(*v) } else { None }
    }
    pub fn as_u8(&self) -> Option<u8> {
        if let Self::Uint8(v) = self { Some(*v) } else { None }
    }
    pub fn as_u32(&self) -> Option<u32> {
        if let Self::Uint32(v) = self { Some(*v) } else { None }
    }
}

// -----------------------------------------------------------------------
// Manufacturer parameter table
// -----------------------------------------------------------------------

const fn p(id: u16, name: &'static str, data_type: DataType) -> ParameterDef {
    ParameterDef { id, name, data_type }
}

// 0x2xxx range — calibration / mechanical
pub const MECHANICAL_OFFSET:        ParameterDef = p(0x2005, "mechOffset",      DataType::Float32);

// 0x3xxx range — measured (read-only telemetry)
pub const MEASURED_POSITION:        ParameterDef = p(0x3016, "mechPos",         DataType::Float32);
pub const MEASURED_VELOCITY:        ParameterDef = p(0x3017, "mechVel",         DataType::Float32);
pub const MEASURED_TORQUE:          ParameterDef = p(0x302C, "torque_fdb",      DataType::Float32);

// 0x7xxx range — control loop / setpoints
pub const MODE:                     ParameterDef = p(0x7005, "run_mode",        DataType::Int8);
pub const IQ_TARGET:                ParameterDef = p(0x7006, "iq_ref",          DataType::Float32);
pub const VELOCITY_TARGET:          ParameterDef = p(0x700A, "spd_ref",         DataType::Float32);
pub const TORQUE_LIMIT:             ParameterDef = p(0x700B, "limit_torque",    DataType::Float32);
pub const CURRENT_KP:               ParameterDef = p(0x7010, "cur_kp",          DataType::Float32);
pub const CURRENT_KI:               ParameterDef = p(0x7011, "cur_ki",          DataType::Float32);
pub const CURRENT_FILTER_GAIN:      ParameterDef = p(0x7014, "cur_filter_gain", DataType::Float32);
pub const POSITION_TARGET:          ParameterDef = p(0x7016, "lof_ref",         DataType::Float32);
pub const VELOCITY_LIMIT:           ParameterDef = p(0x7017, "limit_spd",       DataType::Float32);
pub const CURRENT_LIMIT:            ParameterDef = p(0x7018, "limit_cur",       DataType::Float32);
pub const MECHANICAL_POSITION:      ParameterDef = p(0x7019, "mechPos",         DataType::Float32);
pub const IQ_FILTERED:              ParameterDef = p(0x701A, "iqf",             DataType::Float32);
pub const MECHANICAL_VELOCITY:      ParameterDef = p(0x701B, "mechVel",         DataType::Float32);
pub const VBUS:                     ParameterDef = p(0x701C, "VBUS",            DataType::Float32);
pub const POSITION_KP:              ParameterDef = p(0x701E, "loc_kp",          DataType::Float32);
pub const VELOCITY_KP:              ParameterDef = p(0x701F, "spd_kp",          DataType::Float32);
pub const VELOCITY_KI:              ParameterDef = p(0x7020, "spd_ki",          DataType::Float32);
pub const VELOCITY_FILTER_GAIN:     ParameterDef = p(0x7021, "spd_filter_gain", DataType::Float32);
pub const VEL_ACCELERATION_TARGET:  ParameterDef = p(0x7022, "acc_rad",         DataType::Float32);
pub const PP_VELOCITY_MAX:          ParameterDef = p(0x7024, "vel_max",         DataType::Float32);
pub const PP_ACCELERATION_TARGET:   ParameterDef = p(0x7025, "acc_set",         DataType::Float32);
pub const EPSCAN_TIME:              ParameterDef = p(0x7026, "EPScan_time",     DataType::Uint16);
pub const CAN_TIMEOUT:              ParameterDef = p(0x7028, "canTimeout",      DataType::Uint32);
pub const ZERO_STATE:               ParameterDef = p(0x7029, "zero_sta",        DataType::Uint8);

/// Every parameter in the manufacturer table.
pub const ALL_PARAMETERS: &[ParameterDef] = &[
    MECHANICAL_OFFSET,
    MEASURED_POSITION, MEASURED_VELOCITY, MEASURED_TORQUE,
    MODE, IQ_TARGET, VELOCITY_TARGET, TORQUE_LIMIT,
    CURRENT_KP, CURRENT_KI, CURRENT_FILTER_GAIN,
    POSITION_TARGET, VELOCITY_LIMIT, CURRENT_LIMIT,
    MECHANICAL_POSITION, IQ_FILTERED, MECHANICAL_VELOCITY, VBUS,
    POSITION_KP, VELOCITY_KP, VELOCITY_KI, VELOCITY_FILTER_GAIN,
    VEL_ACCELERATION_TARGET, PP_VELOCITY_MAX, PP_ACCELERATION_TARGET,
    EPSCAN_TIME, CAN_TIMEOUT, ZERO_STATE,
];

/// Look up a parameter by id.
pub fn lookup(id: u16) -> Option<&'static ParameterDef> {
    ALL_PARAMETERS.iter().find(|p| p.id == id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pads_short_types_to_4_bytes() {
        let v = ParameterValue::Uint8(0xAB);
        assert_eq!(v.to_le_bytes(), [0xAB, 0, 0, 0]);

        let v = ParameterValue::Int16(-1);
        assert_eq!(v.to_le_bytes(), [0xFF, 0xFF, 0, 0]);
    }

    #[test]
    fn float_roundtrips() {
        let v = ParameterValue::Float32(1.5);
        let b = v.to_le_bytes();
        assert_eq!(ParameterValue::from_le_bytes(DataType::Float32, b), v);
    }

    #[test]
    fn lookup_finds_known_ids() {
        assert_eq!(lookup(0x7005).unwrap().name, "run_mode");
        assert_eq!(lookup(0x302C).unwrap().data_type, DataType::Float32);
        assert!(lookup(0x9999).is_none());
    }
}
