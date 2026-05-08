//! 29-bit extended-id pack/unpack and `communication_type` constants.
//!
//! ```text
//!  bits 28..24: communication_type (5 bits, 0..31)
//!  bits 23..8:  extra_data         (16 bits)
//!  bits  7..0:  device_id (out)  /  host_id (in)
//! ```
//!
//! Every Robstride frame uses this layout. The bottom byte echoes the
//! source: outgoing frames put the *target* device_id there; replies put
//! the *requesting* host_id there.

/// `communication_type` values (5-bit field, top of the extended id).
///
/// Names mirror the Robstride user manual.
pub mod comm_type {
    pub const GET_DEVICE_ID:     u8 = 0;
    pub const OPERATION_CONTROL: u8 = 1;
    pub const OPERATION_STATUS:  u8 = 2;
    pub const ENABLE:            u8 = 3;
    pub const DISABLE:           u8 = 4;
    pub const SET_ZERO_POSITION: u8 = 6;
    pub const SET_DEVICE_ID:     u8 = 7;
    pub const READ_PARAMETER:    u8 = 17;
    pub const WRITE_PARAMETER:   u8 = 18;
    pub const FAULT_REPORT:      u8 = 21;
    pub const SAVE_PARAMETERS:   u8 = 22;
    pub const SET_BAUDRATE:      u8 = 23;
    pub const ACTIVE_REPORT:     u8 = 24;
    pub const SET_PROTOCOL:      u8 = 25;
}

/// Pack `(communication_type, extra_data, device_id)` into a 29-bit id.
pub fn encode(comm_type: u8, extra_data: u16, device_id: u8) -> u32 {
    ((comm_type as u32 & 0x1F) << 24) | ((extra_data as u32) << 8) | (device_id as u32)
}

/// Unpack a 29-bit id into `(communication_type, extra_data, low_byte)`.
/// `low_byte` is the device_id on outgoing frames or host_id on replies.
pub fn decode(ext_id: u32) -> (u8, u16, u8) {
    let comm_type  = ((ext_id >> 24) & 0x1F) as u8;
    let extra_data = ((ext_id >> 8) & 0xFFFF) as u16;
    let low_byte   = (ext_id & 0xFF) as u8;
    (comm_type, extra_data, low_byte)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let id = encode(comm_type::OPERATION_CONTROL, 0x7FFF, 0x05);
        let (c, e, d) = decode(id);
        assert_eq!(c, 1);
        assert_eq!(e, 0x7FFF);
        assert_eq!(d, 0x05);
    }

    #[test]
    fn truncates_comm_type_to_5_bits() {
        let id = encode(0xFF, 0, 1);
        let (c, _, _) = decode(id);
        assert_eq!(c, 0x1F);
    }
}
