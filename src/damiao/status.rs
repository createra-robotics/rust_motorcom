//! Damiao motor status codes.
//!
//! The motor encodes its status in the high 4 bits of byte 0 of every
//! feedback frame (see [`crate::damiao::mit::MotorState::error`]).

/// Damiao motor status, decoded from the high 4 bits of feedback byte 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorStatus {
    Disabled,
    Enabled,
    OverVoltage,
    UnderVoltage,
    OverCurrent,
    MosOverTemp,
    RotorOverTemp,
    LostComm,
    Overload,
    /// Status code outside the documented set.
    Unknown(u8),
}

impl MotorStatus {
    /// Decode the 4-bit status code from a feedback frame.
    pub fn from_code(code: u8) -> Self {
        match code {
            0x0 => Self::Disabled,
            0x1 => Self::Enabled,
            0x8 => Self::OverVoltage,
            0x9 => Self::UnderVoltage,
            0xA => Self::OverCurrent,
            0xB => Self::MosOverTemp,
            0xC => Self::RotorOverTemp,
            0xD => Self::LostComm,
            0xE => Self::Overload,
            other => Self::Unknown(other),
        }
    }

    /// True if the motor is in any fault state (anything except
    /// Disabled / Enabled).
    pub fn is_fault(&self) -> bool {
        !matches!(self, Self::Disabled | Self::Enabled)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_codes_decode() {
        assert_eq!(MotorStatus::from_code(0x0), MotorStatus::Disabled);
        assert_eq!(MotorStatus::from_code(0x1), MotorStatus::Enabled);
        assert_eq!(MotorStatus::from_code(0xD), MotorStatus::LostComm);
    }

    #[test]
    fn unknown_code_kept_for_diagnostics() {
        assert_eq!(MotorStatus::from_code(0x7), MotorStatus::Unknown(0x7));
    }

    #[test]
    fn fault_flag_skips_normal_states() {
        assert!(!MotorStatus::Disabled.is_fault());
        assert!(!MotorStatus::Enabled.is_fault());
        assert!(MotorStatus::OverVoltage.is_fault());
        assert!(MotorStatus::LostComm.is_fault());
    }
}
