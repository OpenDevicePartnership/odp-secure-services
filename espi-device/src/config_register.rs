use crate::register_enum_types::*;
use bit_register::{bit_register, TryFromBits, TryIntoBits};
use num_enum::{IntoPrimitive, TryFromPrimitive};
use static_assertions::const_assert;

pub trait ConfigRegister:
    TryFrom<u32, Error = &'static str> + TryInto<u32, Error = &'static str> + core::fmt::Debug
{
    const OFFSET: u16;
}

macro_rules! config_register {
    ($name:ident = $offset:expr) => {
        impl ConfigRegister for $name {
            const OFFSET: u16 = $offset;
        }
    };
}

config_register!(DeviceId = 0x04);
bit_register! {
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct DeviceId: little_endian u32 {
        pub version_id: u8 => [0:7],
    }
}

config_register!(GeneralCapabilities = 0x08);
bit_register! {
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct GeneralCapabilities: little_endian u32 {
        pub crc_checking_enable: bool => [31],
        pub response_modifier_enable: bool => [30],
        /* [29] - reserved */
        pub alert_mode: AlertMode => [28],
        pub io_mode_select: IoMode => [26:27],
        pub io_mode_support: IoModeSupport => [24:25],
        pub open_drain_alert_selected: bool => [23],
        pub operating_frequency: Frequency => [20:22],
        pub open_drain_alert_supported: bool => [19],
        pub max_frequency_supported: Frequency => [16:18],
        pub max_wait_state_allowed: u8 => [12:15],
        /* [4:11] - reserved */
        pub flash_access_channel_support: bool => [3],
        pub oob_message_channel_support: bool => [2],
        pub virtual_wire_channel_support: bool => [1],
        pub peripheral_channel_support: bool => [0]
    }
}

config_register!(PeripheralChannelCapabilities = 0x10);
bit_register! {
    /// Channel 0 (Peripheral) capabilities and configurations
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct PeripheralChannelCapabilities: little_endian u32 {
        /// Peripheral Channel Maximum Read Request Size: eSPI master
        /// sets the maximum read request size for the Peripheral channel.
        /// The length of the read request must not cross the naturally aligned
        /// address boundary of the corresponding Maximum Read Request Size.
        pub max_read_request_size: MaxReadRequestSize => [12:14],
        /// Peripheral Channel Maximum Payload Size Selected: eSPI
        /// master sets the maximum payload size for the Peripheral channel.
        /// The value set by the eSPI master must never be more than the value
        /// advertised in the Max Payload Size Supported field.
        /// The payload of the transaction must not cross the naturally aligned
        /// address boundary of the corresponding Maximum Payload Size.
        pub max_payload_size_selected: MaxPayloadSize => [8:10],
        /// Peripheral Channel Maximum Payload Size Supported: This
        /// field advertises the Maximum Payload Size supported by the slave.
        pub max_payload_size_supported: MaxPayloadSizeSupport => [4:6],
        /// Bus Master Enable: When this bit is a ‘0’, it disables the slave from
        /// generating bus mastering cycles on the Peripheral channel. When this
        /// bit is a ‘1’, it allows the slave to generate bus mastering cycles on
        /// the Peripheral channel.
        /// Prior to clearing the Bus Master Enable bit from ‘1’ to ‘0’, there must
        /// be no outstanding non-posted cycle pending completion from the
        /// slave.
        pub bus_master_enable: bool => [2],
        /// Peripheral Channel Ready: When this bit is a ‘1’, it indicates that
        /// the slave is ready to accept transactions on the Peripheral channel.
        /// eSPI master should poll this bit after the channel is enabled before
        /// running any transaction on this channel to the slave.
        pub channel_ready: bool => [1],
        pub channel_enable: bool => [0],
    }
}

config_register!(VwireChannelCapabilities = 0x20);
bit_register! {
    /// Channel 1 (Virtual Wire) capabilities and configurations
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct VwireChannelCapabilities: little_endian u32 {
        pub max_vw_count: u8 => [16:21],
        pub max_vw_count_support: u8 => [8:13],
        pub channel_ready: bool => [1],
        pub channel_enable: bool => [0],
    }
}

config_register!(OobChannelCapabilities = 0x30);
bit_register! {
    /// Channel 2 (OOB Message) capabilities and configurations
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct OobChannelCapabilities: little_endian u32 {
        pub max_payload_size: MaxPayloadSize => [8:10],
        pub max_payload_size_support: MaxPayloadSizeSupport => [4:6],
        pub channel_ready: bool => [1],
        pub channel_enable: bool => [0],
    }
}
config_register!(FlashChannelCapabilities = 0x40);
bit_register! {
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct FlashChannelCapabilities: little_endian u32 {
        pub max_read_request_size: MaxReadRequestSize => [12:14],
        pub flash_sharing_mode: FlashSharingMode => [11:11],
        pub max_payload_size_selected: MaxPayloadSize => [8:10],
        pub max_payload_size_supported: MaxPayloadSizeSupport => [4:6],
        pub flash_block_erase_size: FlashEraseBlockSize => [2:4],
        pub channel_ready: bool => [1],
        pub channel_enable: bool => [0],
    }
}
