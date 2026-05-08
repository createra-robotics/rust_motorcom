//! Status flags packed into the `extra_data` field of `OPERATION_STATUS`
//! reply frames, plus the two-word fault report layout.

/// Per-bit status flags from `OPERATION_STATUS.extra_data`.
///
/// `extra_data` is 16 bits; the bottom 8 bits echo the motor's `device_id`,
/// the upper bits carry warning flags:
///
/// | bit | flag                     |
/// |-----|--------------------------|
/// | 8   | undervoltage             |
/// | 9   | overcurrent              |
/// | 10  | overtemperature          |
/// | 11  | magnetic encoder fault   |
/// | 12  | stalled                  |
/// | 13  | uncalibrated             |
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StatusFlags {
    pub undervoltage: bool,
    pub overcurrent: bool,
    pub overtemperature: bool,
    pub magnetic_encoder_fault: bool,
    pub stalled: bool,
    pub uncalibrated: bool,
}

impl StatusFlags {
    pub fn from_extra(extra: u16) -> Self {
        Self {
            undervoltage:           (extra >> 8)  & 1 != 0,
            overcurrent:            (extra >> 9)  & 1 != 0,
            overtemperature:        (extra >> 10) & 1 != 0,
            magnetic_encoder_fault: (extra >> 11) & 1 != 0,
            stalled:                (extra >> 12) & 1 != 0,
            uncalibrated:           (extra >> 13) & 1 != 0,
        }
    }

    /// True if any warning bit is set.
    pub fn any(&self) -> bool {
        self.undervoltage
            || self.overcurrent
            || self.overtemperature
            || self.magnetic_encoder_fault
            || self.stalled
            || self.uncalibrated
    }
}

/// Decoded `FAULT_REPORT` (`communication_type = 21`) frame.
///
/// The 8-byte data is two little-endian `u32`s: a fault word and a
/// warning word. Each bit corresponds to a specific condition.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FaultReport {
    pub motor_overtemperature: bool, // fault bit 0
    pub gate: bool,                  // fault bit 1
    pub undervoltage: bool,          // fault bit 2
    pub overvoltage: bool,           // fault bit 3
    pub encoder_uncalibrated: bool,  // fault bit 7
    pub stall_current: bool,         // warning bit 14
    pub warning_overtemperature: bool, // warning bit 0
}

impl FaultReport {
    pub fn from_data(data: &[u8]) -> Self {
        debug_assert!(data.len() >= 8);
        let fault = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let warn  = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        Self {
            motor_overtemperature:    (fault >> 0) & 1 != 0,
            gate:                     (fault >> 1) & 1 != 0,
            undervoltage:             (fault >> 2) & 1 != 0,
            overvoltage:              (fault >> 3) & 1 != 0,
            encoder_uncalibrated:     (fault >> 7) & 1 != 0,
            stall_current:            (warn  >> 14) & 1 != 0,
            warning_overtemperature:  (warn  >> 0) & 1 != 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extra_with_no_flags_decodes_clean() {
        let flags = StatusFlags::from_extra(0x05);
        assert_eq!(flags, StatusFlags::default());
        assert!(!flags.any());
    }

    #[test]
    fn extra_high_bits_set_corresponding_flags() {
        let extra = (1 << 8) | (1 << 13);
        let flags = StatusFlags::from_extra(extra);
        assert!(flags.undervoltage);
        assert!(flags.uncalibrated);
        assert!(!flags.stalled);
    }

    #[test]
    fn fault_report_decodes_bits() {
        let mut data = [0u8; 8];
        let fault = (1u32 << 2) | (1u32 << 3); // undervoltage + overvoltage
        data[0..4].copy_from_slice(&fault.to_le_bytes());
        let f = FaultReport::from_data(&data);
        assert!(f.undervoltage);
        assert!(f.overvoltage);
        assert!(!f.gate);
    }
}
