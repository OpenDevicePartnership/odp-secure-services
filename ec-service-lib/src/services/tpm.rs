//! # TPM Service Library (Rust)
//!
//! Implementation for the TPM Service. This library is based off the ARM spec:
//! TPM Service Command Response Buffer Interface Over FF-A.
//! <https://developer.arm.com/documentation/den0138/0100/?lang=en>
//!
//! The state flow is based off the TCG PC Client Specific Platform TPM Profile
//! for TPM 2.0.
//! <https://trustedcomputinggroup.org/resource/pc-client-platform-tpm-profile-ptp-specification/>
//!
//! Figure 4 — TPM State Diagram for CRB Interface
#![allow(dead_code, unused_imports, unused_variables)]

use crate::{Result, Service};
use log::{error, info};
use odp_ffa::{ErrorCode, MsgSendDirectReq2, MsgSendDirectResp2, Payload, RegisterPayload};
use uuid::{uuid, Uuid};

use core::mem;
use core::ptr;

// ---------------------------------------------------------------------------
// Import types and constants from TpmServiceStateTranslationLib
// ---------------------------------------------------------------------------
use super::tpm_sst::{
    PtpCrbRegisters, TpmSstOps, CRB_DATA_BUFFER_SIZE, PTP_CRB_CONTROL_AREA_REQUEST_COMMAND_READY,
    PTP_CRB_CONTROL_AREA_REQUEST_GO_IDLE, PTP_CRB_CONTROL_AREA_STATUS_TPM_IDLE, PTP_CRB_CONTROL_START,
    PTP_CRB_LOCALITY_CONTROL_RELINQUISH, PTP_CRB_LOCALITY_CONTROL_REQUEST_ACCESS,
    PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_0, PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_1,
    PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_2, PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_3,
    PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_4, PTP_CRB_LOCALITY_STATE_LOCALITY_ASSIGNED,
    PTP_CRB_LOCALITY_STATE_TPM_REG_VALID_STATUS, PTP_CRB_LOCALITY_STATUS_GRANTED, TPM_LOCALITY_OFFSET,
};

// ---------------------------------------------------------------------------
// PTP CRB Interface Identifier
// ---------------------------------------------------------------------------
#[derive(Clone, Copy, Default)]
#[repr(transparent)]
pub struct PtpCrbInterfaceIdentifier(pub u32);

impl PtpCrbInterfaceIdentifier {
    pub fn new() -> Self {
        Self(0)
    }

    // Bits [0:3]
    pub fn interface_type(self) -> u32 {
        self.0 & 0xF
    }
    pub fn set_interface_type(&mut self, val: u32) {
        self.0 = (self.0 & !0xF) | (val & 0xF);
    }

    // Bits [4:7]
    pub fn interface_version(self) -> u32 {
        (self.0 >> 4) & 0xF
    }
    pub fn set_interface_version(&mut self, val: u32) {
        self.0 = (self.0 & !(0xF << 4)) | ((val & 0xF) << 4);
    }

    // Bit [8]
    pub fn cap_locality(self) -> u32 {
        (self.0 >> 8) & 1
    }
    pub fn set_cap_locality(&mut self, val: u32) {
        self.0 = (self.0 & !(1 << 8)) | ((val & 1) << 8);
    }

    // Bit [9]
    pub fn cap_crb_idle_bypass(self) -> u32 {
        (self.0 >> 9) & 1
    }
    pub fn set_cap_crb_idle_bypass(&mut self, val: u32) {
        self.0 = (self.0 & !(1 << 9)) | ((val & 1) << 9);
    }

    // Bit [14]
    pub fn cap_crb(self) -> u32 {
        (self.0 >> 14) & 1
    }
    pub fn set_cap_crb(&mut self, val: u32) {
        self.0 = (self.0 & !(1 << 14)) | ((val & 1) << 14);
    }

    // Other bits are unused. Can update here if needed in the future.
}

// ---------------------------------------------------------------------------
// TPM Service Function IDs
// ---------------------------------------------------------------------------
pub const TPM2_FFA_GET_INTERFACE_VERSION: u64 = 0x0f00_0001;
pub const TPM2_FFA_GET_FEATURE_INFO: u64 = 0x0f00_0101;
pub const TPM2_FFA_START: u64 = 0x0f00_0201;
pub const TPM2_FFA_REGISTER_FOR_NOTIFICATION: u64 = 0x0f00_0301;
pub const TPM2_FFA_UNREGISTER_FROM_NOTIFICATION: u64 = 0x0f00_0401;
pub const TPM2_FFA_FINISH_NOTIFIED: u64 = 0x0f00_0501;
pub const TPM2_FFA_MANAGE_LOCALITY: u64 = 0x1f00_0001;

// ---------------------------------------------------------------------------
// TPM Service Status Codes
// ---------------------------------------------------------------------------
pub type TpmStatus = u64;

pub const TPM2_FFA_SUCCESS_OK: TpmStatus = 0x0500_0001;
pub const TPM2_FFA_SUCCESS_OK_RESULTS_RETURNED: TpmStatus = 0x0500_0002;
pub const TPM2_FFA_ERROR_NOFUNC: TpmStatus = 0x8e00_0001;
pub const TPM2_FFA_ERROR_NOTSUP: TpmStatus = 0x8e00_0002;
pub const TPM2_FFA_ERROR_INVARG: TpmStatus = 0x8e00_0005;
pub const TPM2_FFA_ERROR_INV_CRB_CTRL_DATA: TpmStatus = 0x8e00_0006;
pub const TPM2_FFA_ERROR_ALREADY: TpmStatus = 0x8e00_0009;
pub const TPM2_FFA_ERROR_DENIED: TpmStatus = 0x8e00_000a;
pub const TPM2_FFA_ERROR_NOMEM: TpmStatus = 0x8e00_000b;

// ---------------------------------------------------------------------------
// TPM Service Start Qualifiers / Manage Locality Operations
// ---------------------------------------------------------------------------
pub const TPM2_FFA_START_FUNC_QUALIFIER_COMMAND: u16 = 0x0;
pub const TPM2_FFA_START_FUNC_QUALIFIER_LOCALITY: u16 = 0x1;

pub const TPM2_FFA_MANAGE_LOCALITY_OPEN: u16 = 0x0;
pub const TPM2_FFA_MANAGE_LOCALITY_CLOSE: u16 = 0x1;

// ---------------------------------------------------------------------------
// TPM Service Defines
// ---------------------------------------------------------------------------
pub const TPM_MAJOR_VER: u64 = 0x1;
pub const TPM_MINOR_VER: u64 = 0x0;
pub const NUM_LOCALITIES: u8 = 5;
const NO_ACTIVE_LOCALITY: u8 = NUM_LOCALITIES; // Invalid locality

// ---------------------------------------------------------------------------
// TPM Service States
// ---------------------------------------------------------------------------
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TpmState {
    Idle = 0,
    Ready = 1,
    Complete = 2,
}

// ---------------------------------------------------------------------------
// TPM Locality States
// ---------------------------------------------------------------------------
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TpmLocalityState {
    Closed = 0,
    Open = 1,
}

// ---------------------------------------------------------------------------
// TPM Service FF-A Communication Structures
// ---------------------------------------------------------------------------
struct TpmRequest {
    opcode: u64,   // Arg0
    function: u64, // Arg1
    locality: u64, // Arg2
}

struct TpmResponse {
    tpm_status: u64,  // Arg0
    tpm_payload: u64, // Arg1
}

impl From<MsgSendDirectReq2> for TpmRequest {
    fn from(msg: MsgSendDirectReq2) -> Self {
        let opcode = msg.register_at(0); // Arg0, x4
        let function = msg.register_at(1); // Arg1, x5
        let locality = msg.register_at(2); // Arg2, x6

        TpmRequest {
            opcode,
            function,
            locality,
        }
    }
}

impl From<TpmResponse> for RegisterPayload {
    fn from(resp: TpmResponse) -> Self {
        // x4-x17 are for payload (14 registers)
        let payload_regs = [resp.tpm_status, resp.tpm_payload];
        let payload_bytes_iter = payload_regs.iter().flat_map(|&reg| u64::to_le_bytes(reg).into_iter());
        RegisterPayload::from_iter(payload_bytes_iter)
    }
}

// ---------------------------------------------------------------------------
// TPM Service Implementation
// ---------------------------------------------------------------------------
pub struct TpmService<S: TpmSstOps> {
    current_state: TpmState,
    active_locality: u8,
    interface_id_default: PtpCrbInterfaceIdentifier,
    locality_states: [TpmLocalityState; NUM_LOCALITIES as usize],
    tpm_internal_crb_address: u64,
    pub sst: S,
}

impl<S: TpmSstOps> TpmService<S> {
    // Creates an uninitialized 'TpmService'. Call [`init`] to set up state. Init
    // will initialize the internal variables.
    pub fn new(sst: S) -> Self {
        Self {
            current_state: TpmState::Idle,
            active_locality: NO_ACTIVE_LOCALITY,
            interface_id_default: PtpCrbInterfaceIdentifier::new(),
            locality_states: [TpmLocalityState::Closed; NUM_LOCALITIES as usize],
            tpm_internal_crb_address: 0x10000200000,
            sst,
        }
    }

    // Returns a mutable pointer to the CRB register block for `locality`.
    fn crb_ptr(&self, locality: u8) -> *mut PtpCrbRegisters {
        let addr = self.tpm_internal_crb_address + ((locality as u64) * TPM_LOCALITY_OFFSET);
        addr as *mut PtpCrbRegisters
    }

    // Converts an [`ErrorCode`] to a [`TpmStatus`].
    fn convert_error_to_status(status: ErrorCode) -> TpmStatus {
        match status {
            ErrorCode::Ok => TPM2_FFA_SUCCESS_OK,
            ErrorCode::Denied => TPM2_FFA_ERROR_DENIED,
            ErrorCode::NoMemory => TPM2_FFA_ERROR_NOMEM,
            _ => TPM2_FFA_ERROR_DENIED,
        }
    }

    // Initializes the internal CRB for a given locality.
    // SAFETY: Function accesses the internal CRB. The address is defaulted but should
    //         be set during init of the library. The helper that returns the pointer
    //         based on the locality should always return a valid locality region as
    //         locality is always verified before any calls to these functions. It's
    //         the user's responsibility to make sure the address is valid when
    //         initializing the library.
    unsafe fn init_internal_crb(&self, locality: u8) {
        let crb = self.crb_ptr(locality);
        info!("Locality: {:X} - InternalTpmCrb Address: {:X}", locality, crb as usize);
        ptr::write_bytes(crb, 0x00, 1);

        let crb_ref = &mut *crb;
        crb_ref.interface_id = self.interface_id_default.0;
        crb_ref.crb_control_status = PTP_CRB_CONTROL_AREA_STATUS_TPM_IDLE;

        // Set the CRB Command/Response buffer address + size.
        let buf_addr = crb_ref.crb_data_buffer.as_ptr() as usize;
        crb_ref.crb_control_command_address_high = (buf_addr >> 32) as u32;
        crb_ref.crb_control_command_address_low = buf_addr as u32;
        crb_ref.crb_control_command_size = mem::size_of_val(&crb_ref.crb_data_buffer) as u32;
        crb_ref.crb_control_response_address = buf_addr as u64;
        crb_ref.crb_control_response_size = mem::size_of_val(&crb_ref.crb_data_buffer) as u32;
    }

    // Cleans the internal CRB, putting registers into a known-good state.
    // SAFETY: Function accesses the internal CRB. The address is defaulted but should
    //         be set during init of the library. The helper that returns the pointer
    //         based on the locality should always return a valid locality region as
    //         locality is always verified before any calls to these functions. It's
    //         the user's responsibility to make sure the address is valid when
    //         initializing the library.
    unsafe fn clean_internal_crb(&self) {
        // If the user has never requested a locality, don't clean, no need.
        // We should only ever clean the active locality as when localities change
        // we clear the entire CRB region.
        if self.active_locality == NO_ACTIVE_LOCALITY {
            return;
        }

        let crb = &mut *self.crb_ptr(self.active_locality);

        // Set the locality state based on the active locality.
        match self.active_locality {
            0 => crb.locality_state |= PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_0,
            1 => crb.locality_state |= PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_1,
            2 => crb.locality_state |= PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_2,
            3 => crb.locality_state |= PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_3,
            4 => crb.locality_state |= PTP_CRB_LOCALITY_STATE_ACTIVE_LOCALITY_4,
            _ => {}
        }

        crb.locality_state |= PTP_CRB_LOCALITY_STATE_TPM_REG_VALID_STATUS;
        crb.locality_state |= PTP_CRB_LOCALITY_STATE_LOCALITY_ASSIGNED;
        crb.locality_status |= PTP_CRB_LOCALITY_STATUS_GRANTED;
        crb.locality_control = 0;
        crb.interface_id = self.interface_id_default.0;
        crb.crb_control_extension = 0;
        crb.crb_control_request = 0;
        crb.crb_control_cancel = 0;
        crb.crb_control_start = 0;
        crb.crb_interrupt_enable = 0;
        crb.crb_interrupt_status = 0;

        // Set the current TPM status based on the current state.
        if self.current_state == TpmState::Idle {
            crb.crb_control_status = PTP_CRB_CONTROL_AREA_STATUS_TPM_IDLE;
        } else {
            crb.crb_control_status = 0;
        }

        // Set the CRB Command/Response buffer address + size.
        let buf_addr = crb.crb_data_buffer.as_ptr() as usize;
        crb.crb_control_command_address_high = (buf_addr >> 32) as u32;
        crb.crb_control_command_address_low = buf_addr as u32;
        crb.crb_control_command_size = mem::size_of_val(&crb.crb_data_buffer) as u32;
        crb.crb_control_response_address = buf_addr as u64;
        crb.crb_control_response_size = mem::size_of_val(&crb.crb_data_buffer) as u32;

        // Remaining registers can be ignored.
    }

    // Handles TPM commands according to the CRB state machine.
    // SAFETY: Function may initialize/zero the internal CRB buffer during successful completion
    //         of a TPM transaction. The address is defaulted but should be set during init of the
    //         library. The helper that returns the pointer based on the locality should always
    //         return a valid locality region as locality is always verified before any calls to
    //         these functions. It's the user's responsibility to make sure the address is valid when
    //         initializing the library.
    unsafe fn handle_command(&mut self) -> TpmStatus {
        let crb = &mut *self.crb_ptr(self.active_locality);
        let mut status: ErrorCode = ErrorCode::Denied;

        // The normal state flow should be: IDLE -> READY -> COMPLETE -> IDLE.
        // Depending on our current state, we will investigate specific registers and
        // make state transitions or deny commands.
        match self.current_state {
            // -- IDLE ------------------------------------------------------
            // The TPM can transition to IDLE from any state outside of command execution when the
            // SW sets the goIdle bit in the CrbControlRequest register. When the TPM transitions to
            // IDLE from COMPLETE it should clear the buffer.
            TpmState::Idle => {
                // Check the cmdReady bit in the CrbControlRequest register to see if we need to
                // transition to the READY state, otherwise, deny the request.
                if crb.crb_control_request & PTP_CRB_CONTROL_AREA_REQUEST_COMMAND_READY != 0 {
                    info!("IDLE State - Handle TPM Command cmdReady Request");
                    status = self.sst.cmd_ready(self.active_locality);
                    if status == ErrorCode::Ok {
                        self.current_state = TpmState::Ready;
                    }
                }
            }

            // -- READY -----------------------------------------------------
            // The TPM can transition to READY from IDLE or COMPLETE when the SW sets the cmdReady bit
            // in the CrbControlRequest register. When the TPM transitions to READY from COMPLETE it
            // should clear the buffer.
            TpmState::Ready => {
                // Check the goIdle bit in the CrbControlRequest register to see if we need to
                // transition back to the IDLE state.
                if crb.crb_control_request & PTP_CRB_CONTROL_AREA_REQUEST_GO_IDLE != 0 {
                    info!("READY State - Handle TPM Command goIdle Request");
                    status = self.sst.go_idle(self.active_locality);
                    if status == ErrorCode::Ok {
                        self.current_state = TpmState::Idle;
                    }
                // Check the cmdReady bit in the CrbControlRequest register, clear it if it has been
                // set again.
                } else if crb.crb_control_request & PTP_CRB_CONTROL_AREA_REQUEST_COMMAND_READY != 0 {
                    info!("READY State - Handle TPM Command cmdReady Request");
                    status = self.sst.cmd_ready(self.active_locality);
                // Check the CrbControlStart register to see if we need to start executing a command.
                // Once the command completes, transition to the COMPLETE state.
                } else if crb.crb_control_start & PTP_CRB_CONTROL_START != 0 {
                    info!("READY State - Handle TPM Command Start Request");
                    status = self.sst.start(self.active_locality, crb as *mut PtpCrbRegisters);
                    if status == ErrorCode::Ok {
                        self.current_state = TpmState::Complete;
                    }
                }
            }

            // -- COMPLETE --------------------------------------------------
            // The TPM can transition to COMPLETE only from READY when the SW writes a 1 to the
            // CrbControlStart register and the command execution finishes. The SW can write more
            // data to the buffer and set the register again to trigger another command execution;
            // this is only if TPM_CapCRBIdleBypass is 1.
            TpmState::Complete => {
                // Check the goIdle bit in the CrbControlRequest register to see if we need to
                // transition to the IDLE state.
                if crb.crb_control_request & PTP_CRB_CONTROL_AREA_REQUEST_GO_IDLE != 0 {
                    info!("COMPLETE State - Handle TPM Command goIdle Request");
                    status = self.sst.go_idle(self.active_locality);
                    if status == ErrorCode::Ok {
                        self.current_state = TpmState::Idle;
                        ptr::write_bytes(crb.crb_data_buffer.as_mut_ptr(), 0x00, CRB_DATA_BUFFER_SIZE);
                    }
                // Check the cmdReady bit in the CrbControlRequest register to see if we need to
                // transition back to the READY state.
                } else if crb.crb_control_request & PTP_CRB_CONTROL_AREA_REQUEST_COMMAND_READY != 0 {
                    info!("COMPLETE State - Handle TPM Command cmdReady Request");
                    if self.sst.is_idle_bypass_supported() {
                        status = self.sst.cmd_ready(self.active_locality);
                        if status == ErrorCode::Ok {
                            self.current_state = TpmState::Ready;
                            ptr::write_bytes(crb.crb_data_buffer.as_mut_ptr(), 0x00, CRB_DATA_BUFFER_SIZE);
                        }
                    }
                // Check the CrbControlStart register to see if we need to execute another command.
                } else if crb.crb_control_start & PTP_CRB_CONTROL_START != 0 {
                    info!("COMPLETE State - Handle TPM Command Start Request");
                    // Execution of another command from COMPLETE is only supported if TPM_CapCRBIdleBypass
                    // is 1.
                    if self.sst.is_idle_bypass_supported() {
                        status = self.sst.start(self.active_locality, crb as *mut PtpCrbRegisters);
                    }
                }
            }
        }

        // Clear the internal CRB start register to indicate successful completion and response ready.
        if status != ErrorCode::Ok {
            error!("Command Failed w/ Status: {:X}", status as usize);
        }

        Self::convert_error_to_status(status)
    }

    // Handles locality request / relinquish.
    // SAFETY: Upon successful TPM locality request the internal CRB at that locality gets initialized.
    //         The function to handle this is marked as unsafe as it access the memory thus this function
    //         is also unsafe.
    unsafe fn handle_locality_request(&mut self, locality: u8) -> TpmStatus {
        let crb = &mut *self.crb_ptr(locality);
        let status: ErrorCode;
        let new_active_locality: u8;

        // Check if we are doing a locality relinquish.
        if crb.locality_control & PTP_CRB_LOCALITY_CONTROL_RELINQUISH != 0 {
            // Make sure the locality being relinquished is the active locality.
            if locality != self.active_locality {
                error!("Locality Relinquish Failed - Invalid Locality");
                return TPM2_FFA_ERROR_DENIED;
            }

            info!("Handle TPM Locality{:X} Relinquish", locality);
            status = self.sst.locality_relinquish(locality);
            new_active_locality = NO_ACTIVE_LOCALITY;
        // Check if we are doing a locality request.
        } else if crb.locality_control & PTP_CRB_LOCALITY_CONTROL_REQUEST_ACCESS != 0 {
            // Make sure there is no active locality if requesting a different locality.
            if self.active_locality != NO_ACTIVE_LOCALITY && self.active_locality != locality {
                error!("Locality Request Failed - Locality Not Relinquished");
                return TPM2_FFA_ERROR_DENIED;
            }

            info!("Handle TPM Locality{:X} Request", locality);
            status = self.sst.locality_request(locality);
            new_active_locality = locality;
        // Otherwise, the host didn't set the correct bits, invalid.
        } else {
            error!("Request/Relinquish Bit Not Set");
            return TPM2_FFA_ERROR_DENIED;
        }

        // Update the internal TPM CRB.
        if status == ErrorCode::Ok {
            self.init_internal_crb(locality);
            self.active_locality = new_active_locality;
        } else {
            error!("Locality Request Failed w/ Status: {:X}", status as usize);
        }

        Self::convert_error_to_status(status)
    }

    // Specification Functions
    fn get_interface_version_handler(&self, _request: &TpmRequest, response: &mut TpmResponse) -> TpmStatus {
        response.tpm_payload = (TPM_MAJOR_VER << 16) | TPM_MINOR_VER;
        TPM2_FFA_SUCCESS_OK_RESULTS_RETURNED
    }

    fn get_feature_info_handler(&self, _request: &TpmRequest, _response: &mut TpmResponse) -> TpmStatus {
        error!("Unsupported Function");
        TPM2_FFA_ERROR_NOTSUP
    }

    // SAFETY: Function invokes both handle_command and handle_locality_request which can
    //         access/alter the internal CRB. This function also calls clean_internal_crb
    //         which makes sure the CRB at the current active locality is in a known and
    //         valid state by setting the memory contents (registers) to known values.
    unsafe fn start_handler(&mut self, request: &TpmRequest, _response: &mut TpmResponse) -> TpmStatus {
        let function = request.function as u16;
        let locality = request.locality as u8;
        let return_val: TpmStatus;

        if locality >= NUM_LOCALITIES {
            error!("Invalid Locality");
            return_val = TPM2_FFA_ERROR_INVARG;
        } else if self.locality_states[locality as usize] == TpmLocalityState::Closed {
            error!("Locality Closed");
            return_val = TPM2_FFA_ERROR_DENIED;
        } else if function == TPM2_FFA_START_FUNC_QUALIFIER_COMMAND {
            if locality == self.active_locality {
                return_val = self.handle_command();
            } else {
                error!("Locality Mismatch");
                return_val = TPM2_FFA_ERROR_INVARG;
            }
        } else if function == TPM2_FFA_START_FUNC_QUALIFIER_LOCALITY {
            return_val = self.handle_locality_request(locality);
        } else {
            error!("Invalid Start Function");
            return_val = TPM2_FFA_ERROR_INVARG;
        }

        // Clean up the internal CRB.
        self.clean_internal_crb();
        return_val
    }

    fn register_handler(&self, _request: &TpmRequest, _response: &mut TpmResponse) -> TpmStatus {
        error!("Unsupported Function");
        TPM2_FFA_ERROR_NOTSUP
    }

    fn unregister_handler(&self, _request: &TpmRequest, _response: &mut TpmResponse) -> TpmStatus {
        error!("Unsupported Function");
        TPM2_FFA_ERROR_NOTSUP
    }

    fn finish_handler(&self, _request: &TpmRequest, _response: &mut TpmResponse) -> TpmStatus {
        error!("Unsupported Function");
        TPM2_FFA_ERROR_NOTSUP
    }

    fn manage_locality_handler(&mut self, request: &TpmRequest, _response: &mut TpmResponse) -> TpmStatus {
        let locality_operation = request.function as u16;
        let locality = request.locality as u8;
        let mut return_val: TpmStatus = TPM2_FFA_SUCCESS_OK;

        if locality >= NUM_LOCALITIES {
            error!("Invalid Locality");
            return TPM2_FFA_ERROR_INVARG;
        }

        if locality_operation == TPM2_FFA_MANAGE_LOCALITY_OPEN {
            info!("Locality{:X} Opened", locality);
            self.locality_states[locality as usize] = TpmLocalityState::Open;
        } else if locality_operation == TPM2_FFA_MANAGE_LOCALITY_CLOSE {
            info!("Locality{:X} Closed", locality);
            self.locality_states[locality as usize] = TpmLocalityState::Closed;
        } else {
            error!("Invalid Manage Locality Operation");
            return_val = TPM2_FFA_ERROR_INVARG;
        }

        return_val
    }

    /// Initializes the TPM service (equivalent to `TpmServiceInit`).
    /// # Safety
    /// Function initializes the internal CRB at all localities by calling
    /// init_internal_crb. This sets all locality regions to known valid
    /// values. The internal TPM CRB address must be a valid memory region
    /// passed in by the caller.
    pub unsafe fn init(&mut self, tpm_internal_crb_address: u64) {
        // Build the default interface ID.
        self.interface_id_default = PtpCrbInterfaceIdentifier::new();
        self.interface_id_default.set_interface_type(1); // CRB active
        self.interface_id_default.set_interface_version(1); // CRB interface version
        self.interface_id_default.set_cap_locality(1); // 5 localities supported
        self.interface_id_default.set_cap_crb(1); // CRB supported

        for locality in 0..NUM_LOCALITIES {
            self.init_internal_crb(locality);
        }

        // Initialize the TPM Service State Translation Library.
        self.sst.init(0x60120000);

        self.current_state = TpmState::Idle;
        self.active_locality = NO_ACTIVE_LOCALITY;
        self.tpm_internal_crb_address = tpm_internal_crb_address;
    }

    /// De-initializes the TPM service (equivalent to `TpmServiceDeInit`).
    pub fn deinit(&mut self) {
        // Nothing to de-init.
    }
}

const TPM_SERVICE_UUID: Uuid = uuid!("17b862a4-1806-4faf-86b3-089a58353861");

impl<S: TpmSstOps> Service for TpmService<S> {
    fn service_name(&self) -> &'static str {
        "Tpm"
    }

    fn service_uuid(&self) -> Uuid {
        TPM_SERVICE_UUID
    }

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let req_payload: TpmRequest = msg.clone().into();
        let mut resp_payload: TpmResponse = TpmResponse {
            tpm_status: (TPM2_FFA_SUCCESS_OK),
            tpm_payload: (0),
        };

        resp_payload.tpm_status = match req_payload.opcode {
            TPM2_FFA_GET_INTERFACE_VERSION => self.get_interface_version_handler(&req_payload, &mut resp_payload),
            TPM2_FFA_GET_FEATURE_INFO => self.get_feature_info_handler(&req_payload, &mut resp_payload),
            TPM2_FFA_START => unsafe { self.start_handler(&req_payload, &mut resp_payload) },
            TPM2_FFA_REGISTER_FOR_NOTIFICATION => self.register_handler(&req_payload, &mut resp_payload),
            TPM2_FFA_UNREGISTER_FROM_NOTIFICATION => self.unregister_handler(&req_payload, &mut resp_payload),
            TPM2_FFA_FINISH_NOTIFIED => self.finish_handler(&req_payload, &mut resp_payload),
            TPM2_FFA_MANAGE_LOCALITY => {
                // Only allow from a logical SP owned by TF-A (source_id high byte == 0xFF).
                // NOTE: This should really be msg.source_id()
                if (msg.destination_id() & 0xFF00) != 0xFF00 {
                    error!("Invalid Source ID: {:X}", msg.destination_id());
                    TPM2_FFA_ERROR_DENIED
                } else {
                    self.manage_locality_handler(&req_payload, &mut resp_payload)
                }
            }
            _ => {
                error!("Invalid TPM Service Opcode");
                TPM2_FFA_ERROR_NOFUNC
            }
        };

        let payload: RegisterPayload = RegisterPayload::from(resp_payload);
        Ok(MsgSendDirectResp2::from_req_with_payload(&msg, payload))
    }
}
