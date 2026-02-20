#![no_std]

use espi_device::config_register::ConfigRegister;
use espi_device::{IndependentChannel, OobChannel, PeripheralChannel, Result, ShortOpData, Tag};
use log::info;

pub struct EspiDeviceStub;

impl EspiDeviceStub {
    pub fn new() -> Self {
        Self {}
    }
}

impl Default for EspiDeviceStub {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[allow(unused)]
struct StubConfigRegister(u32);

impl ConfigRegister for StubConfigRegister {
    const OFFSET: u16 = 0x00;
}

impl TryFrom<u32> for StubConfigRegister {
    type Error = &'static str;

    fn try_from(value: u32) -> core::result::Result<Self, Self::Error> {
        Ok(StubConfigRegister(value))
    }
}

impl TryInto<u32> for StubConfigRegister {
    type Error = &'static str;

    fn try_into(self) -> core::result::Result<u32, Self::Error> {
        Ok(self.0)
    }
}

impl IndependentChannel for EspiDeviceStub {
    fn get_configuration_register<T: ConfigRegister>(&self) -> Result<T> {
        info!("get_configuration_register: {:?}", T::OFFSET);
        Ok(T::try_from(0).unwrap())
    }

    fn set_configuration_register<T: ConfigRegister>(&self, value: T) -> Result<()> {
        info!("set_configuration_register: {:?}", value);
        Ok(())
    }

    fn get_status(&self) -> Result<espi_device::StatusRegister> {
        info!("get_status");
        Ok(espi_device::StatusRegister::try_from(0).unwrap())
    }

    fn reset(&self) -> Result<()> {
        info!("reset");
        Ok(())
    }
}

impl PeripheralChannel for EspiDeviceStub {
    fn mem32_write_short<D: ShortOpData>(&self, address: u32, data: D) -> Result<()> {
        info!("mem32_write_short: {:?} = {:?}", address, data);
        Ok(())
    }

    fn mem32_read_short<D: ShortOpData>(&self, address: u32) -> Result<D> {
        info!("mem32_read_short: {:?}", address);
        Ok(D::try_from_u32(0).unwrap())
    }

    fn io_read_short<D: ShortOpData>(&self, address: u16) -> Result<D> {
        info!("io_read_short: {:?}", address);
        Ok(D::try_from_u32(0).unwrap())
    }

    fn io_write_short<D: ShortOpData>(&self, address: u16, data: D) -> Result<()> {
        info!("io_write_short: {:?} = {:?}", address, data);
        Ok(())
    }

    fn put_posted_mem32_write(&self, tag: Tag, address: u32, data: &[u8]) -> Result<()> {
        info!("put_posted_mem32_write({:?}, {:?}): {:?}", tag, address, data);
        Ok(())
    }

    fn put_np_mem32_read<'buf>(&self, tag: Tag, address: u32, buffer: &'buf mut [u8]) -> Result<&'buf [u8]> {
        info!("put_np_mem32_read({:?}, {:?}): {:?}", tag, address, buffer);
        Ok(buffer)
    }

    fn put_posted_mem64_write(&self, tag: Tag, address: u64, data: &[u8]) -> Result<()> {
        info!("put_posted_mem64_write({:?}, {:?}): {:?}", tag, address, data);
        Ok(())
    }

    fn put_np_mem64_read<'buf>(&self, tag: Tag, address: u64, buffer: &'buf mut [u8]) -> Result<&'buf [u8]> {
        info!("put_np_mem64_read({:?}, {:?}): {:?}", tag, address, buffer);
        Ok(buffer)
    }

    fn put_posted_message(&self, tag: Tag, code: u8, specific_bytes: &[u8; 4]) -> Result<()> {
        info!("put_posted_message({:?}, {:?}, {:?})", tag, code, specific_bytes);
        Ok(())
    }

    fn put_posted_message_with_data(&self, tag: Tag, code: u8, specific_bytes: &[u8; 4], data: &[u8]) -> Result<()> {
        info!(
            "put_posted_message_with_data({:?}, {:?}, {:?}): {:?}",
            tag, code, specific_bytes, data
        );
        Ok(())
    }
}

impl OobChannel for EspiDeviceStub {
    fn put_oob(&self, data: &[u8], tag: Tag) -> Result<()> {
        info!("put_oob({:?}): {:?}", tag, data);
        Ok(())
    }

    fn get_oob<'a>(&self, buffer: &'a mut [u8]) -> Result<&'a [u8]> {
        info!("get_oob({:?})", buffer);
        Ok(buffer)
    }
}
