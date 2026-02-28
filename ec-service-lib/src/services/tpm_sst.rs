//! # TPM Service State Translation Library (Rust)
//!
//! Implementation for the TPM Service State Translation Library. This library's
//! main purpose is to translate the states of the TPM service's CRB states to
//! the main TPM's interface states. (i.e. TPM PC CRB -> TPM FIFO) A user of the
//! TPM service should only need to update this library with the proper TPM
//! interface type for their device.
#![allow(dead_code, unused_imports, unused_variables)]

use core::ptr;
use log::info;
use odp_ffa::{ErrorCode, Function, Yield};

// ---------------------------------------------------------------------------
// TPM Service State Translation Defines
// ---------------------------------------------------------------------------
pub const TPM_LOCALITY_OFFSET: u64 = 0x1000;

const INTERFACE_TYPE_MASK: u32 = 0x00F;
const IDLE_BYPASS_MASK: u32 = 0x200;

const DELAY_AMOUNT: u64 = 30000;
const YIELD_AMOUNT: u64 = 10 * 1000; // 10ms

const DEBUG_ENABLED: bool = false;

// ---------------------------------------------------------------------------
// PTP CRB Constants
// ---------------------------------------------------------------------------
pub const PTP_CRB_CONTROL_AREA_STATUS_TPM_IDLE: u32 = 1 << 1;
pub const PTP_CRB_CONTROL_AREA_STATUS_TPM_STATUS: u32 = 1 << 0;
pub const PTP_CRB_CONTROL_AREA_REQUEST_GO_IDLE: u32 = 1 << 1;
pub const PTP_CRB_CONTROL_AREA_REQUEST_COMMAND_READY: u32 = 1 << 0;
pub const PTP_CRB_CONTROL_START: u32 = 1 << 0;
pub const PTP_CRB_CONTROL_CANCEL: u32 = 1 << 0;

pub const PTP_CRB_LOCALITY_STATE_TPM_REG_VALID_STATUS: u32 = 1 << 7;
pub const PTP_CRB_LOCALITY_STATE_LOCALITY_ASSIGNED: u32 = 1 << 1;
pub const PTP_CRB_LOCALITY_STATE_TPM_ESTABLISHED: u32 = 1 << 0;

pub const PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_0: u32 = 0;
pub const PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_1: u32 = 1 << 2;
pub const PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_2: u32 = 1 << 3;
pub const PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_3: u32 = (1 << 2) | (1 << 3);
pub const PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_4: u32 = 1 << 4;

pub const PTP_CRB_LOCALITY_CONTROL_RELINQUISH: u32 = 1 << 1;
pub const PTP_CRB_LOCALITY_CONTROL_REQUEST_ACCESS: u32 = 1 << 0;

pub const PTP_CRB_LOCALITY_STATUS_GRANTED: u32 = 1 << 0;

// ---------------------------------------------------------------------------
// PTP FIFO Constants
// ---------------------------------------------------------------------------
pub const PTP_FIFO_STS_VALID: u32 = 1 << 7;
pub const PTP_FIFO_STS_DATA: u32 = 1 << 4;
pub const PTP_FIFO_STS_EXPECT: u32 = 1 << 3;
pub const PTP_FIFO_STS_READY: u32 = 1 << 6;
pub const PTP_FIFO_STS_GO: u32 = 1 << 5;

pub const PTP_FIFO_ACC_RQUUSE: u8 = 1 << 1;
pub const PTP_FIFO_ACC_ACTIVE: u8 = 1 << 5;
pub const PTP_FIFO_VALID: u8 = 1 << 7;

// ---------------------------------------------------------------------------
// PTP Timeout Constants
// ---------------------------------------------------------------------------
pub const PTP_TIMEOUT_A: u64 = 750 * 1000; // 750ms
pub const PTP_TIMEOUT_B: u64 = 2000 * 1000; // 2s
pub const PTP_TIMEOUT_C: u64 = 200 * 1000; // 200ms
pub const PTP_TIMEOUT_D: u64 = 30 * 1000; // 30ms
pub const PTP_TIMEOUT_MAX: u64 = 90000 * 1000; // 90s

// ---------------------------------------------------------------------------
// PTP CRB Registers
// ---------------------------------------------------------------------------
pub const CRB_DATA_BUFFER_SIZE: usize = 0xF80;
#[repr(C)]
pub struct PtpCrbRegisters {
    pub locality_state: u32,                         // 0x00
    pub reserved1: [u8; 4],                          // 0x04
    pub locality_control: u32,                       // 0x08
    pub locality_status: u32,                        // 0x0C
    pub reserved2: [u8; 0x20],                       // 0x10
    pub interface_id: u32,                           // 0x30
    pub vid: u16,                                    // 0x34
    pub did: u16,                                    // 0x36
    pub crb_control_extension: u64,                  // 0x38
    pub crb_control_request: u32,                    // 0x40
    pub crb_control_status: u32,                     // 0x44
    pub crb_control_cancel: u32,                     // 0x48
    pub crb_control_start: u32,                      // 0x4C
    pub crb_interrupt_enable: u32,                   // 0x50
    pub crb_interrupt_status: u32,                   // 0x54
    pub crb_control_command_size: u32,               // 0x58
    pub crb_control_command_address_low: u32,        // 0x5C
    pub crb_control_command_address_high: u32,       // 0x60
    pub crb_control_response_size: u32,              // 0x64
    pub crb_control_response_address: u64,           // 0x68
    pub reserved4: [u8; 0x10],                       // 0x70
    pub crb_data_buffer: [u8; CRB_DATA_BUFFER_SIZE], // 0x80
}

// ---------------------------------------------------------------------------
// PTP FIFO Registers
// ---------------------------------------------------------------------------
#[repr(C, packed)]
pub struct PtpFifoRegisters {
    pub access: u8,                // 0x00
    pub reserved1: [u8; 7],        // 0x01
    pub int_enable: u32,           // 0x08
    pub int_vector: u8,            // 0x0C
    pub reserved2: [u8; 3],        // 0x0D
    pub int_sts: u32,              // 0x10
    pub interface_capability: u32, // 0x14
    pub status: u8,                // 0x18
    pub burst_count: u16,          // 0x19
    pub status_ex: u8,             // 0x1B
    pub reserved3: [u8; 8],        // 0x1C
    pub data_fifo: u32,            // 0x24
    pub reserved4: [u8; 8],        // 0x28
    pub interface_id: u32,         // 0x30
    pub reserved5: [u8; 0x4C],     // 0x34
    pub x_data_fifo: u32,          // 0x80
    pub reserved6: [u8; 0xE7C],    // 0x84
    pub vid: u16,                  // 0xF00
    pub did: u16,                  // 0xF02
    pub rid: u8,                   // 0xF04
    pub reserved7: [u8; 0xFB],     // 0xF05
}

// ---------------------------------------------------------------------------
// Debug Helpers
// ---------------------------------------------------------------------------
fn dump_tpm_input_block(input_block_size: u32, input_block: &[u8]) {
    info!("TpmCommand Send - {:02X?}", &input_block[..input_block_size as usize]);
}

fn dump_tpm_output_block(output_block_size: u32, output_block: &[u8]) {
    info!("TpmCommand Rec - {:02X?}", &output_block[..output_block_size as usize]);
}

// ---------------------------------------------------------------------------
// TpmSstOps Trait
// ---------------------------------------------------------------------------
pub trait TpmSstOps {
    fn go_idle(&mut self, locality: u8) -> ErrorCode;
    fn cmd_ready(&mut self, locality: u8) -> ErrorCode;
    fn start(&mut self, locality: u8, crb: *mut PtpCrbRegisters) -> ErrorCode;
    fn locality_request(&mut self, locality: u8) -> ErrorCode;
    fn locality_relinquish(&mut self, locality: u8) -> ErrorCode;
    fn is_idle_bypass_supported(&self) -> bool;
    fn init(&mut self, internal_tpm_address: u64);
}

// ---------------------------------------------------------------------------
// TpmSst — TPM Service State Translation Library implementation
// ---------------------------------------------------------------------------
pub struct TpmSst {
    is_crb_interface: bool,
    is_idle_bypass_supported: bool,
    tpm_crb_address: u64,
}

impl Default for TpmSst {
    fn default() -> Self {
        Self::new()
    }
}

// NOTE: Most of the internal functions to the TPM service state translation library
//       are marked as "unsafe" due to them directly accessing MMIO registers for either
//       a CRB or FIFO interface. The mechanism to read/write to these addresses are
//       inherently unsafe functions as they require pointers to manipulate the memory.
impl TpmSst {
    // Creates an uninitialized `TpmSst`. Call [`TpmSstOps::init()`] before use. Init
    // will initialize the internal variables.
    pub const fn new() -> Self {
        Self {
            is_crb_interface: false,
            is_idle_bypass_supported: false,
            tpm_crb_address: 0x60120000,
        }
    }

    // Returns a raw pointer to the external CRB registers for the given locality.
    fn external_crb_ptr(&self, locality: u8) -> *mut PtpCrbRegisters {
        (self.tpm_crb_address + ((locality as u64) * TPM_LOCALITY_OFFSET)) as *mut PtpCrbRegisters
    }

    // Returns a raw pointer to the external FIFO registers for the given locality.
    fn external_fifo_ptr(&self, locality: u8) -> *mut PtpFifoRegisters {
        (self.tpm_crb_address + ((locality as u64) * TPM_LOCALITY_OFFSET)) as *mut PtpFifoRegisters
    }

    // Temp function to busy loop before checking register contents.
    fn delay(&self, delay_amount: u64) {
        for i in 0..delay_amount {
            // Do nothing
        }
    }

    // SAFETY: Function accesses the MMIO region associated with the external FIFO burst count
    //         register. The CRB/FIFO address is defaulted but is passed in during initialization
    //         of the library. It's the user's responsibility to verify they are initializing the
    //         library with a valid CRB/FIFO address.
    unsafe fn fifo_read_burst_count(&self, external_fifo: *mut PtpFifoRegisters) -> Result<u16, ErrorCode> {
        // Slight delay before we start checking the registers.
        self.delay(DELAY_AMOUNT);

        let mut delay_amount: u64 = 0;
        loop {
            let burst_count_ptr = ptr::addr_of!((*external_fifo).burst_count) as *const u8;
            let burst_count_lo: u8 = ptr::read_volatile(burst_count_ptr);
            let burst_count_hi: u8 = ptr::read_volatile(burst_count_ptr.add(1));
            let burst_count: u16 = (burst_count_hi as u16) << 8 | burst_count_lo as u16;

            if burst_count != 0 {
                return Ok(burst_count);
            }

            if delay_amount >= PTP_TIMEOUT_D {
                break;
            }

            // Convert milliseconds to nanoseconds.
            Yield::new(YIELD_AMOUNT * 1000).exec().unwrap();

            delay_amount += YIELD_AMOUNT;
        }

        // NOTE: There should be a timeout or device error code for when the TPM
        //       doesn't respond.
        Err(ErrorCode::Denied)
    }

    // SAFETY: Function accesses the register being passed into the function. The CRB/FIFO address
    //         is defaulted but is passed in during initialization  of the library. It's the user's
    //         responsibility to verify they are initializing the library with a valid CRB/FIFO
    //         address.
    unsafe fn wait_register_bits(&self, register: *mut u32, bit_set: u32, bit_clear: u32, timeout: u64) -> ErrorCode {
        // Slight delay before we start checking the registers.
        self.delay(DELAY_AMOUNT);

        let mut delay_amount: u64 = 0;
        loop {
            // Attempt to read the register based on the TPM type.
            let reg_read = if self.is_crb_interface {
                ptr::read_volatile(register)
            } else {
                ptr::read_volatile(register as *mut u8) as u32
            };

            // Verify the register contents.
            if ((reg_read & bit_set) == bit_set) && ((reg_read & bit_clear) == 0) {
                return ErrorCode::Ok;
            }

            if delay_amount >= timeout {
                break;
            }

            // Convert milliseconds to nanoseconds.
            Yield::new(YIELD_AMOUNT * 1000).exec().unwrap();

            delay_amount += YIELD_AMOUNT;
        }

        // NOTE: There should be a timeout or device error code for when the TPM
        //       doesn't respond.
        ErrorCode::Denied
    }

    // SAFETY: Function copies data from the internal CRB to the external CRB/FIFO. This
    //         function, in the FIFO instance, also calls fifo_read_burst_count and
    //         wait_register_bits which are both marked as unsafe functions.
    unsafe fn copy_command_data(&self, locality: u8, tpm_command_buffer: &[u8], command_data_len: u32) -> ErrorCode {
        // Determine which TPM structure to access.
        if self.is_crb_interface {
            let external_crb = self.external_crb_ptr(locality);

            // Copy the command data to the CRB buffer.
            #[allow(clippy::needless_range_loop)]
            for i in 0..(command_data_len as usize) {
                ptr::write_volatile(
                    ptr::addr_of!((*external_crb).crb_data_buffer[i]) as *mut u8,
                    tpm_command_buffer[i],
                );
            }

            ErrorCode::Ok
        } else {
            let external_fifo = self.external_fifo_ptr(locality);
            let mut status = ErrorCode::Ok;
            let len = command_data_len as usize;

            // Copy the command data to the FIFO depending on the burst count.
            let mut index: usize = 0;
            while index < len {
                let burst_count = match self.fifo_read_burst_count(external_fifo) {
                    Ok(bc) => bc,
                    Err(error) => {
                        status = error;
                        break;
                    }
                };

                let mut remaining_burst = burst_count;
                while remaining_burst > 0 && index < len {
                    ptr::write_volatile(
                        ptr::addr_of!((*external_fifo).data_fifo) as *mut u8,
                        tpm_command_buffer[index],
                    );
                    index += 1;
                    remaining_burst -= 1;
                }
            }

            if status == ErrorCode::Ok {
                // Check to make sure the STS_EXPECT register changed from 1 to 0.
                status = self.wait_register_bits(
                    ptr::addr_of!((*external_fifo).status) as *mut u32,
                    PTP_FIFO_STS_VALID,
                    PTP_FIFO_STS_EXPECT,
                    PTP_TIMEOUT_C,
                );
            }

            status
        }
    }

    // SAFETY: Function writes to the start/go register which initiates a TPM transaction.
    //         This function also calls wait_register_bits which is marked as unsafe.
    unsafe fn start_command(&self, locality: u8) -> ErrorCode {
        // Determine which TPM structure to access.
        if self.is_crb_interface {
            let external_crb = self.external_crb_ptr(locality);

            ptr::write_volatile(
                ptr::addr_of!((*external_crb).crb_control_start) as *mut u32,
                PTP_CRB_CONTROL_START,
            );

            self.wait_register_bits(
                ptr::addr_of!((*external_crb).crb_control_start) as *mut u32,
                0,
                PTP_CRB_CONTROL_START,
                PTP_TIMEOUT_MAX,
            )
        } else {
            let external_fifo = self.external_fifo_ptr(locality);

            // Set the tpmGo bit in the Status register.
            ptr::write_volatile(ptr::addr_of!((*external_fifo).status) as *mut u8, PTP_FIFO_STS_GO as u8);

            self.wait_register_bits(
                ptr::addr_of!((*external_fifo).status) as *mut u32,
                PTP_FIFO_STS_VALID | PTP_FIFO_STS_DATA,
                0,
                PTP_TIMEOUT_MAX,
            )
        }
    }

    // SAFETY: Function copies data from the external CRB/FIFO to the internal CRB. This
    //         function, in the FIFO instance, also calls fifo_read_burst_count which is
    //         also marked as unsafe.
    unsafe fn copy_response_data(
        &self,
        locality: u8,
        tpm_command_buffer: &mut [u8],
        response_data_len: u32,
    ) -> ErrorCode {
        // Determine which TPM structure to access.
        if self.is_crb_interface {
            let external_crb = self.external_crb_ptr(locality);

            // Copy the CRB buffer data to the response buffer.
            #[allow(clippy::needless_range_loop)]
            for i in 0..(response_data_len as usize) {
                tpm_command_buffer[i] = ptr::read_volatile(ptr::addr_of!((*external_crb).crb_data_buffer[i]));
            }

            ErrorCode::Ok
        } else {
            let external_fifo = self.external_fifo_ptr(locality);
            let mut status = ErrorCode::Ok;
            let len = response_data_len as usize;

            let mut index: usize = 0;
            while index < len {
                let burst_count = match self.fifo_read_burst_count(external_fifo) {
                    Ok(bc) => bc,
                    Err(error) => {
                        status = error;
                        break;
                    }
                };

                let mut remaining_burst = burst_count;
                while remaining_burst > 0 {
                    tpm_command_buffer[index] =
                        ptr::read_volatile(ptr::addr_of!((*external_fifo).data_fifo) as *mut u8);
                    index += 1;
                    remaining_burst -= 1;
                    if index == len {
                        return ErrorCode::Ok;
                    }
                }
            }

            status
        }
    }
}

// ---------------------------------------------------------------------------
// TpmSstOps trait implementation for TpmSst
// ---------------------------------------------------------------------------
impl TpmSstOps for TpmSst {
    // Initiates the transition to the Idle state.
    fn go_idle(&mut self, locality: u8) -> ErrorCode {
        unsafe {
            // Determine which TPM structure to access.
            if self.is_crb_interface {
                let external_crb = self.external_crb_ptr(locality);

                // Set the goIdle bit in the CRB Control Request register. Wait for it
                // to clear and then check the CRB Control Area Status register to make
                // sure the tpmIdle bit was set.
                ptr::write_volatile(
                    ptr::addr_of!((*external_crb).crb_control_request) as *mut u32,
                    PTP_CRB_CONTROL_AREA_REQUEST_GO_IDLE,
                );

                let status = self.wait_register_bits(
                    ptr::addr_of!((*external_crb).crb_control_request) as *mut u32,
                    0,
                    PTP_CRB_CONTROL_AREA_REQUEST_GO_IDLE,
                    PTP_TIMEOUT_C,
                );

                if status == ErrorCode::Ok {
                    return self.wait_register_bits(
                        ptr::addr_of!((*external_crb).crb_control_status) as *mut u32,
                        PTP_CRB_CONTROL_AREA_STATUS_TPM_IDLE,
                        0,
                        PTP_TIMEOUT_C,
                    );
                }

                status
            } else {
                let external_fifo = self.external_fifo_ptr(locality);

                // Note that there is no goIdle in the FIFO TPM implementation.
                // Going idle is the same as commandReady.
                ptr::write_volatile(
                    ptr::addr_of!((*external_fifo).status) as *mut u8,
                    PTP_FIFO_STS_READY as u8,
                );

                // Set the commandReady bit in the Status register. Read it back and verify it is set which
                // indicates the TPM is ready.
                self.wait_register_bits(
                    ptr::addr_of!((*external_fifo).status) as *mut u32,
                    PTP_FIFO_STS_READY,
                    0,
                    PTP_TIMEOUT_B,
                )
            }
        }
    }

    // Initiates the transition to the commandReady state.
    fn cmd_ready(&mut self, locality: u8) -> ErrorCode {
        unsafe {
            // Determine which TPM structure to access.
            if self.is_crb_interface {
                let external_crb = self.external_crb_ptr(locality);

                // Set the cmdReady bit in the CRB Control Request register. Wait for it
                // to clear and then check the CRB Control Area Status register to make
                // sure the tpmIdle bit was cleared.
                ptr::write_volatile(
                    ptr::addr_of!((*external_crb).crb_control_request) as *mut u32,
                    PTP_CRB_CONTROL_AREA_REQUEST_COMMAND_READY,
                );

                let status = self.wait_register_bits(
                    ptr::addr_of!((*external_crb).crb_control_request) as *mut u32,
                    0,
                    PTP_CRB_CONTROL_AREA_REQUEST_COMMAND_READY,
                    PTP_TIMEOUT_C,
                );

                if status == ErrorCode::Ok {
                    return self.wait_register_bits(
                        ptr::addr_of!((*external_crb).crb_control_status) as *mut u32,
                        0,
                        PTP_CRB_CONTROL_AREA_STATUS_TPM_IDLE,
                        PTP_TIMEOUT_C,
                    );
                }

                status
            } else {
                let external_fifo = self.external_fifo_ptr(locality);

                // Set the commandReady bit in the Status register. Read it back and
                // verify it is set which indicates the TPM is ready.
                ptr::write_volatile(
                    ptr::addr_of!((*external_fifo).status) as *mut u8,
                    PTP_FIFO_STS_READY as u8,
                );

                self.wait_register_bits(
                    ptr::addr_of!((*external_fifo).status) as *mut u32,
                    PTP_FIFO_STS_READY,
                    0,
                    PTP_TIMEOUT_B,
                )
            }
        }
    }

    // Initiates command execution. Copies command data from the internal CRB
    // to the external TPM, executes the command, and copies the response back.
    fn start(&mut self, locality: u8, internal_tpm_crb: *mut PtpCrbRegisters) -> ErrorCode {
        unsafe {
            let response_data_len = (*internal_tpm_crb).crb_control_response_size;
            let command_data_len = (*internal_tpm_crb).crb_control_command_size;

            let mut tpm_command_buffer = [0u8; CRB_DATA_BUFFER_SIZE];

            // Copy the CRB command data to the local buffer.
            ptr::copy_nonoverlapping(
                (*internal_tpm_crb).crb_data_buffer.as_ptr(),
                tpm_command_buffer.as_mut_ptr(),
                command_data_len as usize,
            );

            // Debug printout for input data
            if DEBUG_ENABLED {
                dump_tpm_input_block(command_data_len, &tpm_command_buffer[..command_data_len as usize]);
            }

            // Copy the command data to the external TPM.
            let mut status = self.copy_command_data(locality, &tpm_command_buffer, command_data_len);

            if status != ErrorCode::Ok {
                return status;
            }

            // Start command execution.
            status = self.start_command(locality);
            if status != ErrorCode::Ok {
                return status;
            }

            // Copy the response data from the external TPM.
            status = self.copy_response_data(locality, &mut tpm_command_buffer, response_data_len);
            if status != ErrorCode::Ok {
                return status;
            }

            // Copy the CRB response data from the local buffer.
            ptr::copy_nonoverlapping(
                tpm_command_buffer.as_ptr(),
                (*internal_tpm_crb).crb_data_buffer.as_mut_ptr(),
                response_data_len as usize,
            );

            // Debug printout for output data
            if DEBUG_ENABLED {
                dump_tpm_output_block(response_data_len, &tpm_command_buffer[..response_data_len as usize]);
            }

            status
        }
    }

    // Requests access to the given locality on the external TPM.
    fn locality_request(&mut self, locality: u8) -> ErrorCode {
        unsafe {
            // Determine which TPM structure to access.
            if self.is_crb_interface {
                let external_crb = self.external_crb_ptr(locality);

                ptr::write_volatile(
                    ptr::addr_of!((*external_crb).locality_control) as *mut u32,
                    PTP_CRB_LOCALITY_CONTROL_REQUEST_ACCESS,
                );

                self.wait_register_bits(
                    ptr::addr_of!((*external_crb).locality_status) as *mut u32,
                    PTP_CRB_LOCALITY_STATUS_GRANTED,
                    0,
                    PTP_TIMEOUT_A,
                )
            } else {
                let external_fifo = self.external_fifo_ptr(locality);

                ptr::write_volatile(ptr::addr_of!((*external_fifo).access) as *mut u8, PTP_FIFO_ACC_RQUUSE);

                self.wait_register_bits(
                    ptr::addr_of!((*external_fifo).access) as *mut u32,
                    (PTP_FIFO_ACC_ACTIVE as u32) | (PTP_FIFO_VALID as u32),
                    0,
                    PTP_TIMEOUT_A,
                )
            }
        }
    }

    // Relinquishes access to the given locality on the external TPM.
    fn locality_relinquish(&mut self, locality: u8) -> ErrorCode {
        unsafe {
            // Determine which TPM structure to access.
            if self.is_crb_interface {
                let external_crb = self.external_crb_ptr(locality);

                ptr::write_volatile(
                    ptr::addr_of!((*external_crb).locality_control) as *mut u32,
                    PTP_CRB_LOCALITY_CONTROL_RELINQUISH,
                );

                self.wait_register_bits(
                    ptr::addr_of!((*external_crb).locality_status) as *mut u32,
                    0,
                    PTP_CRB_LOCALITY_STATUS_GRANTED,
                    PTP_TIMEOUT_A,
                )
            } else {
                let external_fifo = self.external_fifo_ptr(locality);

                ptr::write_volatile(ptr::addr_of!((*external_fifo).access) as *mut u8, PTP_FIFO_ACC_ACTIVE);

                self.wait_register_bits(
                    ptr::addr_of!((*external_fifo).access) as *mut u32,
                    PTP_FIFO_VALID as u32,
                    PTP_FIFO_ACC_ACTIVE as u32,
                    PTP_TIMEOUT_A,
                )
            }
        }
    }

    // Returns if IdleBypass is supported.
    fn is_idle_bypass_supported(&self) -> bool {
        self.is_idle_bypass_supported
    }

    // Initializes the TPM Service State Translation Library by reading the
    // interface identifier register to determine the TPM interface type and
    // idle bypass support.
    fn init(&mut self, tpm_crb_address: u64) {
        // Note that the register we are looking at is located at the same address
        // regardless of if the TPM type is FIFO or CRB.
        let external_crb = self.tpm_crb_address as *const PtpCrbRegisters;

        // SAFETY:
        unsafe {
            // Need to determine the TPM interface type.
            let interface_id = ptr::read_volatile(ptr::addr_of!((*external_crb).interface_id) as *mut u32);
            self.is_crb_interface = (interface_id & INTERFACE_TYPE_MASK) == 1;

            // Need to determine if idle bypass is supported.
            self.is_idle_bypass_supported = (interface_id & IDLE_BYPASS_MASK) != 0;
        }

        // Set the tpm CRB address.
        self.tpm_crb_address = tpm_crb_address;
    }
}
