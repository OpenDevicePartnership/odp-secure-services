#[allow(unused)]
#[repr(C, packed(1))]
pub struct Version {
    pub major: u8,
    pub minor: u8,
    pub spin: u8,
    pub res0: u8,
}

#[allow(unused)]
#[repr(C, packed(1))]
pub struct Capabilities {
    pub events: u32,
    pub fw_version: Version,
    pub secure_state: u8,
    pub boot_status: u8,
    pub fan_mask: u8,
    pub battery_mask: u8,
    pub temp_mask: u16,
    pub key_mask: u16,
    pub debug_mask: u16,
    pub res0: u16,
}

#[allow(unused)]
#[repr(C, packed(1))]
pub struct Notifications {
    pub service: u16,
    pub event: u16,
}

#[allow(unused)]
#[repr(C, packed(1))]
pub struct TimeAlarm {
    pub events: u32,
    pub capability: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub valid: u8,
    pub daylight: u8,
    pub res1: u8,
    pub milli: u16,
    pub time_zone: u16,
    pub res2: u16,
    pub alarm_status: u32,
    pub ac_time_val: u32,
    pub dc_time_val: u32,
}

#[allow(unused)]
#[repr(C, packed(1))]
pub struct Battery {
    pub events: u32,
    pub status: u32,
    pub last_full_charge: u32,
    pub cycle_count: u32,
    pub state: u32,
    pub present_rate: u32,
    pub remain_cap: u32,
    pub present_volt: u32,
    pub psr_state: u32,
    pub psr_max_out: u32,
    pub psr_max_in: u32,
    pub peak_level: u32,
    pub peak_power: u32,
    pub sus_level: u32,
    pub sus_power: u32,
    pub peak_thres: u32,
    pub sus_thres: u32,
    pub trip_thres: u32,
    pub bmc_data: u32,
    pub bmd_data: u32,
    pub bmd_flags: u32,
    pub bmd_count: u32,
    pub charge_time: u32,
    pub run_time: u32,
    pub sample_time: u32,
}

#[allow(unused)]
#[repr(C, packed(1))]
pub struct Thermal {
    pub events: u32,
    pub cool_mode: u32,
    pub dba_limit: u32,
    pub sonne_limit: u32,
    pub ma_limit: u32,
    pub fan1_on_temp: u32,
    pub fan1_ramp_temp: u32,
    pub fan1_max_temp: u32,
    pub fan1_crt_temp: u32,
    pub fan1_hot_temp: u32,
    pub fan1_max_rpm: u32,
    pub fan1_cur_rpm: u32,
    pub tmp1_val: u32,
    pub tmp1_timeout: u32,
    pub tmp1_low: u32,
    pub tmp1_high: u32,
}

#[allow(unused)]
#[repr(C, packed(1))]
pub struct EcMemory {
    pub ver: Version,
    pub caps: Capabilities,
    pub notifications: Notifications,
    pub tas: TimeAlarm,
    pub batt: Battery,
    pub therm: Thermal,
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn version_size_is_four_bytes() {
        assert_eq!(size_of::<Version>(), 4, "Version must be 4 bytes for C layout");
    }

    #[test]
    fn notifications_size_is_four_bytes() {
        assert_eq!(
            size_of::<Notifications>(),
            4,
            "Notifications must be 4 bytes for C layout"
        );
    }

    #[test]
    fn ec_memory_struct_is_packed() {
        // EcMemory should have no trailing padding; size is sum of field sizes
        let expected_min = size_of::<Version>()
            + size_of::<Capabilities>()
            + size_of::<Notifications>()
            + size_of::<TimeAlarm>()
            + size_of::<Battery>()
            + size_of::<Thermal>();
        assert_eq!(
            size_of::<EcMemory>(),
            expected_min,
            "EcMemory must be packed with no padding"
        );
    }
}
