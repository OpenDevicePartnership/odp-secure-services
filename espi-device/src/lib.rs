#![no_std]
// TODO: remove this once implementation is complete
#![allow(unused)]

#[macro_use]
extern crate bit_register;

pub mod config_register;
mod espi_error;
mod espi_types;
pub mod register_enum_types;
mod status_register;

use bit_register::{NumBytes, TryFromBits, TryIntoBits};
use config_register::ConfigRegister;
// pub use cycle_type::*;
pub use espi_error::EspiError;
pub use espi_types::cycle_type::*;
pub use espi_types::{EspiCommandOpCode, ShortOpData};
pub use status_register::StatusRegister;

pub type Result<T> = core::result::Result<T, EspiError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Tag(u8);
impl Tag {
    pub const MAX: u8 = 0x0F;
    pub fn encode(self) -> u16 {
        self.0 as u16
    }
}
impl TryFrom<u8> for Tag {
    type Error = EspiError;

    fn try_from(value: u8) -> core::result::Result<Self, Self::Error> {
        if value > Self::MAX {
            Err(EspiError::Other("Tag value out of range"))
        } else {
            Ok(Tag(value))
        }
    }
}
impl TryFromBits<u32> for Tag {
    fn try_from_bits(bits: u32) -> core::result::Result<Self, &'static str> {
        if bits > Self::MAX as u32 {
            Err("Tag value out of range")
        } else {
            Ok(Tag(bits as u8))
        }
    }
}
impl TryIntoBits<u32> for Tag {
    fn try_into_bits(self) -> core::result::Result<u32, &'static str> {
        Ok(self.0 as u32)
    }
}
impl NumBytes for Tag {
    const NUM_BYTES: usize = 1;
}

pub trait EspiDevice {}

/// eSPI Independent Channel functions
pub trait IndependentChannel: Sized {
    /// Get configuration register
    fn get_configuration_register<T: ConfigRegister>(&self) -> Result<T>;

    /// Set configuration register
    fn set_configuration_register<T: ConfigRegister>(&self, value: T) -> Result<()>;

    /// Get status register
    fn get_status(&self) -> Result<StatusRegister>;

    /// Reset the eSPI device
    fn reset(&self) -> Result<()>;
}

// eSPI Peripheral Channel functions
pub trait PeripheralChannel: Sized {
    // fn put_posted<C: PutPostedCycleType>(&self, cycle_type: C, tag: Tag) -> Result<C::Out>;

    /// Perform a short memory write command (PUT_MEMWR32_SHORT)
    fn mem32_write_short<D: ShortOpData>(&self, address: u32, data: D) -> Result<()>;

    /// Perform a short memory read command (PUT_MEMRD32_SHORT)
    fn mem32_read_short<D: ShortOpData>(&self, address: u32) -> Result<D>;

    /// Perform a short IO read command (PUT_IORD_SHORT)
    fn io_read_short<D: ShortOpData>(&self, address: u16) -> Result<D>;

    /// Perform a short IO write command (PUT_IOWR_SHORT)
    fn io_write_short<D: ShortOpData>(&self, address: u16, data: D) -> Result<()>;

    /// Perform a non-posted memory write command, 32 bit address (PUT_NP)
    fn put_posted_mem32_write(&self, tag: Tag, address: u32, data: &[u8]) -> Result<()>;

    /// Perform a non-posted memory read command, 32 bit address (PUT_NP)
    fn put_np_mem32_read<'buf>(&self, tag: Tag, address: u32, buffer: &'buf mut [u8]) -> Result<&'buf [u8]>;

    /// Perform a non-posted memory write command, 64 bit address (PUT_NP)
    fn put_posted_mem64_write(&self, tag: Tag, address: u64, data: &[u8]) -> Result<()>;

    /// Perform a non-posted memory read command, 64 bit address (PUT_NP)
    fn put_np_mem64_read<'buf>(&self, tag: Tag, address: u64, buffer: &'buf mut [u8]) -> Result<&'buf [u8]>;

    /// Perform a non-posted message command (PUT_NP)
    fn put_posted_message(&self, tag: Tag, code: u8, specific_bytes: &[u8; 4]) -> Result<()>;

    /// Perform a non-posted message with data command (PUT_NP)
    fn put_posted_message_with_data(&self, tag: Tag, code: u8, specific_bytes: &[u8; 4], data: &[u8]) -> Result<()>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PutVwireData {
    pub index: u8,
    pub data: u8,
}

pub trait VirtualWireChannel: Sized {
    /// Perform a virtual wire write command (PUT_VWIRE)
    fn put_vwire(&self, data: &[PutVwireData]) -> Result<()>;

    /// Perform a virtual wire read command (GET_VWIRE)
    fn get_vwire<'a>(&self, indexes: &'a mut [u8]) -> Result<&'a [u8]>;
}

pub trait OobChannel: Sized {
    /// Put an OOB (Tunneled SMBus) message
    fn put_oob(&self, data: &[u8], tag: Tag) -> Result<()>;

    /// Get an OOB (Tunneled SMBus) message
    fn get_oob<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8]>;
}

pub trait FlashChannel: Sized {}
