use crate::{Result, Service};
use log::{debug, error, info};
use odp_ffa::{DirectMessagePayload, ErrorCode, HasRegisterPayload, MsgSendDirectReq2, MsgSendDirectResp2};
use uuid::{uuid, Uuid};

// Hard cap for the number of services that can be registered
// and number of mappings per service.
const NOTIFY_MAX_SERVICES: usize = 16;
const NOTIFY_MAX_MAPPINGS: usize = 64;

// Maximum number of mappings that can be registered in a single request, as restricted
// by the number of registers available.
const NOTIFY_MAX_MAPPINGS_PER_REQ: usize = 8;

const MESSAGE_INFO_DIR_RESP: u64 = 0x100; // Base for direct response messages

#[derive(Debug, Clone, Copy, PartialEq, Eq, num_enum::TryFromPrimitive, num_enum::IntoPrimitive)]
#[repr(u8)]
enum MessageID {
    Add = 0,
    Remove = 1,
    Setup = 2,
    Destroy = 3,
    Assign = 4,
    Unassign = 5,
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
enum NotifyType {
    #[default]
    Global,
    PerVcpu,
}

#[derive(Default)]
struct NfyGenericRsp {
    status: i64,
}

#[derive(Debug)]
struct NfySetupRsp {
    reserved: u64,
    sender_uuid: Uuid,
    receiver_uuid: Uuid,
    msg_info: u64,
    status: ErrorCode,
}

#[derive(Debug, Clone, Copy)]
struct NotifyReq {
    src_id: u16, // Source ID of the request
    sender_uuid: Uuid,
    receiver_uuid: Uuid,
    msg_info: MessageInfo,
    count: u8,
    notifications: [(u32, u16, NotifyType); 7], // Cookie, Notification ID, Type
}

impl NotifyReq {
    fn extract_tuple(value: u64) -> (u32, u16, NotifyType) {
        let cookie = (value >> 32) as u32;
        let id = ((value >> 23) & 0x1FF) as u16;
        let ntype = match (value & 0x1) != 0 {
            false => NotifyType::Global,
            true => NotifyType::PerVcpu,
        };
        (cookie, id, ntype)
    }
}

impl From<MsgSendDirectReq2> for NotifyReq {
    fn from(msg: MsgSendDirectReq2) -> Self {
        let payload = msg.payload();
        let src_id = msg.source_id();
        let sender_uuid =
            Uuid::from_u128_le(((payload.register_at(2) as u128) << 64) | (payload.register_at(1) as u128));
        let receiver_uuid =
            Uuid::from_u128_le(((payload.register_at(4) as u128) << 64) | (payload.register_at(3) as u128));
        let msg_info = MessageInfo::from_raw(payload.register_at(5));
        let count = (payload.register_at(6) & 0x1ff).min(7) as u8; // Count is lower 9 bits
        let mut notifications = [(0, 0, NotifyType::Global); 7];
        for (i, notif) in notifications.iter_mut().enumerate().take(count as usize) {
            *notif = NotifyReq::extract_tuple(payload.register_at(7 + i));
        }

        NotifyReq {
            src_id,
            sender_uuid,
            receiver_uuid,
            msg_info,
            count,
            notifications,
        }
    }
}

impl From<NfyGenericRsp> for DirectMessagePayload {
    fn from(value: NfyGenericRsp) -> Self {
        DirectMessagePayload::from_iter(value.status.to_le_bytes())
    }
}

impl From<NfySetupRsp> for DirectMessagePayload {
    fn from(rsp: NfySetupRsp) -> Self {
        //
        // x4-x17 are for payload (14 registers)
        let payload_regs = [
            rsp.reserved,
            rsp.sender_uuid.as_u64_pair().0,
            rsp.sender_uuid.as_u64_pair().1,
            rsp.receiver_uuid.as_u64_pair().0,
            rsp.receiver_uuid.as_u64_pair().1,
            rsp.msg_info,
            rsp.status as u64,
        ];

        let payload_bytes_iter = payload_regs.iter().flat_map(|&reg| u64::to_le_bytes(reg).into_iter());
        DirectMessagePayload::from_iter(payload_bytes_iter)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MessageInfo(u64);

impl MessageInfo {
    /// Get the message ID (bits 0–2).
    fn message_id(&self) -> MessageID {
        ((self.0 & 0b111) as u8).try_into().expect("Invalid Message ID")
    }

    /// Construct from a raw u64.
    fn from_raw(value: u64) -> Self {
        MessageInfo(value)
    }
}

#[derive(Default, Debug, Copy, Clone)]
struct NfyMapping {
    cookie: u32,       // Cookie for the notification
    id: u16,           // Global bitmask value
    ntype: NotifyType, // Type of notification (Global or PerVcpu)
    src_id: u16,       // Source ID for the notification
    in_use: bool,      // Whether the notification mapping is currently in use
}

#[derive(Debug, Copy, Clone)]
struct NfyEntry {
    service_uuid: Uuid,
    in_use: bool,
    mappings: [NfyMapping; NOTIFY_MAX_MAPPINGS], // This will hold the mappings for this service
}

impl Default for NfyEntry {
    fn default() -> Self {
        Self {
            service_uuid: Uuid::nil(),
            in_use: false,
            mappings: [NfyMapping::default(); NOTIFY_MAX_MAPPINGS],
        }
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct Notify {
    // We will carry the registered notifications in this struct.
    // which will be an array of NfyEntry with size of NOTIFY_MAX_SERVICES.
    entries: [NfyEntry; NOTIFY_MAX_SERVICES],

    // Here we also keep track of the global bitmap to the best of our knowledge.
    // So that the multiple mappings will not conflict on the same bit.
    global_bitmap: u64,
}

impl Notify {
    pub fn new() -> Self {
        Self::default()
    }

    fn nfy_find_entry(&self, uuid: Uuid) -> Option<usize> {
        self.entries
            .iter()
            .position(|entry| entry.service_uuid == uuid && entry.in_use)
    }

    fn nfy_find_empty_slot(&self) -> Option<usize> {
        self.entries.iter().position(|entry| !entry.in_use)
    }

    fn nfy_find_matching_cookie(&self, entry_index: usize, cookie: u32) -> Option<usize> {
        if entry_index >= NOTIFY_MAX_SERVICES {
            return None;
        }

        let entry = &self.entries[entry_index];
        entry
            .mappings
            .iter()
            .position(|mapping| mapping.in_use && mapping.cookie == cookie)
    }

    fn nfy_register_mapping(&mut self, entry_index: usize, req: NotifyReq) -> ErrorCode {
        if entry_index >= NOTIFY_MAX_SERVICES {
            error!("Invalid entry index: {entry_index}");
            return ErrorCode::InvalidParameters;
        }

        // Make a copy of the entries and global bitmap so that we will iterate
        // through the incoming request without mutating the original state.
        let mut temp_entries = self.entries;
        let mut temp_bitmask = self.global_bitmap;

        // loop through the mappings in the req and register them
        // We will iterate through the notifications, with a maximum of req.count
        for (cookie, id, ntype) in req.notifications.iter().take(req.count as usize) {
            let mut applied = false;
            if let Some(_mapping_index) = self.nfy_find_matching_cookie(entry_index, *cookie) {
                // If we found a matching cookie, this does not make sense, so we return an error
                error!("Found matching cookie for entry {entry_index}: {cookie}");
                return ErrorCode::InvalidParameters;
            } else if temp_bitmask & (1 << id) != 0 {
                // If the bit is already set, we cannot register this mapping
                error!("Bitmask already set for entry {entry_index}: {id}");
                return ErrorCode::InvalidParameters;
            } else {
                // No matching cookie found, we can register this mapping
                // We will use the first empty mapping slot in the entry
                let cookie = *cookie;
                let id = *id;
                let ntype = *ntype;

                let entry = &mut temp_entries[entry_index];
                for mapping in &mut entry.mappings {
                    if !mapping.in_use {
                        info!("Mapping: cookie: {cookie}, id: {id}, ntype: {ntype:?}");
                        mapping.cookie = cookie;
                        mapping.id = id;
                        mapping.ntype = ntype;
                        mapping.src_id = req.src_id;
                        mapping.in_use = true;
                        temp_bitmask |= 1 << id; // Set the bit in the global bitmap
                        applied = true;
                        break;
                    }
                }
            }
            if !applied {
                error!("Unable to apply mapping for cookie: {cookie}, id: {id}, ntype: {ntype:?}");
                // Something went wrong, we could not apply the mapping, just bail here
                return ErrorCode::NoMemory;
            }
        }

        // If we reach here, we have successfully registered the mappings, on to
        // the temporary entries and global bitmap. Now we can copy the content
        // back into the original entries and global bitmap.
        self.entries = temp_entries;
        self.global_bitmap = temp_bitmask;

        ErrorCode::Ok
    }

    fn nfy_unregister_mapping(&mut self, entry_index: usize, req: NotifyReq) -> ErrorCode {
        if entry_index >= NOTIFY_MAX_SERVICES {
            error!("Invalid entry index: {entry_index}");
            return ErrorCode::InvalidParameters;
        }

        // Make a copy of the entries and global bitmap so that we will iterate
        // through the incoming request without mutating the original state.
        let mut temp_entries = self.entries;
        let mut temp_bitmask = self.global_bitmap;

        // loop through the mappings in the req and register them
        // We will iterate through the notifications, with a maximum of req.count
        for (cookie, id, ntype) in req.notifications.iter().take(req.count as usize) {
            let mapping_index = match self.nfy_find_matching_cookie(entry_index, *cookie) {
                Some(index) => index,
                None => {
                    // If we could not find a matching cookie, this is an error request
                    error!("No matching cookie found for entry {entry_index}: {cookie}");
                    return ErrorCode::InvalidParameters;
                }
            };

            let t_id = temp_entries[entry_index].mappings[mapping_index].id;
            let t_ntype = temp_entries[entry_index].mappings[mapping_index].ntype;
            let t_src_id = temp_entries[entry_index].mappings[mapping_index].src_id;

            if t_id != *id {
                // If the cookie does not match, this is an error request
                error!("Cookie does not match for entry {entry_index}: {t_id} != {id}");
                return ErrorCode::InvalidParameters;
            }

            if t_ntype != *ntype {
                // If the type does not match, this is an error request
                error!("Type does not match for entry {entry_index}: {t_ntype:?} != {ntype:?}");
                return ErrorCode::InvalidParameters;
            }

            if t_src_id != req.src_id {
                // If the source ID does not match, this is an error request
                error!(
                    "Source ID does not match for entry {}: {} != {}",
                    entry_index, t_src_id, req.src_id
                );
                return ErrorCode::InvalidParameters;
            }

            // Enough checks, we can now unregister the mapping
            temp_entries[entry_index].mappings[mapping_index].in_use = false;
            temp_entries[entry_index].mappings[mapping_index].cookie = 0;
            temp_entries[entry_index].mappings[mapping_index].id = 0;
            temp_entries[entry_index].mappings[mapping_index].ntype = NotifyType::Global;
            temp_entries[entry_index].mappings[mapping_index].src_id = 0;

            temp_bitmask &= !(1 << t_id); // Clear the bit in the global bitmap
        }

        // If we reach here, we have successfully registered the mappings, on to
        // the temporary entries and global bitmap. Now we can copy the content
        // back into the original entries and global bitmap.
        self.entries = temp_entries;
        self.global_bitmap = temp_bitmask;

        ErrorCode::Ok
    }

    fn nfy_setup(&mut self, req: NotifyReq) -> NfySetupRsp {
        info!("cmd: {:?}", req.msg_info.message_id());
        info!("sender_uuid: {:?}", req.sender_uuid);
        info!("receiver_uuid: {:?}", req.receiver_uuid);
        info!("Count: {:?}", req.count);

        if req.count == 0 || req.count >= NOTIFY_MAX_MAPPINGS_PER_REQ as u8 {
            // If the count is zero or exceeds the maximum allowed mappings per request,
            // we cannot register the service
            error!("Invalid parameters: count is zero or exceeds maximum allowed mappings per request");
            return NfySetupRsp {
                reserved: 0,
                sender_uuid: req.sender_uuid,
                receiver_uuid: req.receiver_uuid,
                msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Setup as u64, // Response message for notification registration failure
                status: ErrorCode::InvalidParameters,
            };
        }

        // First check to see if the service is already registered
        let entry;
        if let Some(entry_index) = self.nfy_find_entry(req.receiver_uuid) {
            // If not registered, we will find an empty slot
            info!("Service already registered, reusing entry: {entry_index}");
            entry = Some(entry_index);
        } else if let Some(empty_slot) = self.nfy_find_empty_slot() {
            // If we found an empty slot, we can register the service
            self.entries[empty_slot].in_use = true;
            self.entries[empty_slot].service_uuid = req.receiver_uuid;
            self.entries[empty_slot].mappings = [NfyMapping::default(); NOTIFY_MAX_MAPPINGS];
            info!("Service registered, entry: {empty_slot}");
            entry = Some(empty_slot);
        } else {
            // If no empty slot is found, we cannot register the service
            return NfySetupRsp {
                reserved: 0,
                sender_uuid: req.sender_uuid,
                receiver_uuid: req.receiver_uuid,
                msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Setup as u64, // Response message for notification registration failure
                status: ErrorCode::NoMemory,
            };
        }

        if let Some(service_entry) = entry {
            // Now we can process the request
            let res = self.nfy_register_mapping(service_entry, req);

            // Regardless of the result, we will return a response
            return NfySetupRsp {
                reserved: 0,
                sender_uuid: req.sender_uuid,
                receiver_uuid: req.receiver_uuid,
                msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Setup as u64, // Response message for notification registration failure
                status: res,
            };
        }

        NfySetupRsp {
            reserved: 0,
            sender_uuid: req.sender_uuid,
            receiver_uuid: req.receiver_uuid,
            msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Setup as u64, // Response message for notification registration
            status: ErrorCode::NoMemory,
        }
    }

    fn nfy_destroy(&mut self, req: NotifyReq) -> NfySetupRsp {
        // First check to see if the service is already registered
        let entry = match self.nfy_find_entry(req.receiver_uuid) {
            Some(entry_index) => {
                // If registered, we will use the entry index
                info!("Service found, entry: {entry_index}");
                entry_index
            }
            None => {
                // If not registered, we cannot unregister the service
                error!("Service not found for UUID: {:?}", req.receiver_uuid);
                // If no service entry is not found, we cannot unregister the service
                return NfySetupRsp {
                    reserved: 0,
                    sender_uuid: req.sender_uuid,
                    receiver_uuid: req.receiver_uuid,
                    msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Destroy as u64, // Response message for notification destroy failure
                    status: ErrorCode::InvalidParameters,
                };
            }
        };

        // Now we can process the request
        let res = self.nfy_unregister_mapping(entry, req);

        // Regardless of the result, we will return a response
        NfySetupRsp {
            reserved: 0,
            sender_uuid: req.sender_uuid,
            receiver_uuid: req.receiver_uuid,
            msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Destroy as u64, // Response message for notification destroy
            status: res,
        }
    }

    fn nfy_add(&mut self, req: NotifyReq) -> NfySetupRsp {
        if req.count == 0 || req.count >= NOTIFY_MAX_MAPPINGS_PER_REQ as u8 {
            return NfySetupRsp {
                reserved: 0,
                sender_uuid: req.sender_uuid,
                receiver_uuid: req.receiver_uuid,
                msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Add as u64,
                status: ErrorCode::InvalidParameters,
            };
        }

        let entry_index = if let Some(idx) = self.nfy_find_entry(req.receiver_uuid) {
            idx
        } else if let Some(empty) = self.nfy_find_empty_slot() {
            self.entries[empty].in_use = true;
            self.entries[empty].service_uuid = req.receiver_uuid;
            self.entries[empty].mappings = [NfyMapping::default(); NOTIFY_MAX_MAPPINGS];
            empty
        } else {
            return NfySetupRsp {
                reserved: 0,
                sender_uuid: req.sender_uuid,
                receiver_uuid: req.receiver_uuid,
                msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Add as u64,
                status: ErrorCode::NoMemory,
            };
        };

        let res = self.nfy_register_mapping(entry_index, req);
        NfySetupRsp {
            reserved: 0,
            sender_uuid: req.sender_uuid,
            receiver_uuid: req.receiver_uuid,
            msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Add as u64,
            status: res,
        }
    }

    fn nfy_remove(&mut self, req: NotifyReq) -> NfySetupRsp {
        let entry = match self.nfy_find_entry(req.receiver_uuid) {
            Some(idx) => idx,
            None => {
                return NfySetupRsp {
                    reserved: 0,
                    sender_uuid: req.sender_uuid,
                    receiver_uuid: req.receiver_uuid,
                    msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Remove as u64,
                    status: ErrorCode::InvalidParameters,
                };
            }
        };

        let res = self.nfy_unregister_mapping(entry, req);
        NfySetupRsp {
            reserved: 0,
            sender_uuid: req.sender_uuid,
            receiver_uuid: req.receiver_uuid,
            msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Remove as u64,
            status: res,
        }
    }

    fn nfy_assign(&mut self, req: NotifyReq) -> NfySetupRsp {
        let entry_index = match self.nfy_find_entry(req.receiver_uuid) {
            Some(idx) => idx,
            None => {
                return NfySetupRsp {
                    reserved: 0,
                    sender_uuid: req.sender_uuid,
                    receiver_uuid: req.receiver_uuid,
                    msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Assign as u64,
                    status: ErrorCode::InvalidParameters,
                };
            }
        };

        for (cookie, id, ntype) in req.notifications.iter().take(req.count as usize) {
            match self.nfy_find_matching_cookie(entry_index, *cookie) {
                Some(mapping_index) => {
                    let mapping = &mut self.entries[entry_index].mappings[mapping_index];
                    // Clear old bit, set new bit
                    self.global_bitmap &= !(1 << mapping.id);
                    if self.global_bitmap & (1 << id) != 0 {
                        error!("Bitmask conflict during assign for id: {id}");
                        return NfySetupRsp {
                            reserved: 0,
                            sender_uuid: req.sender_uuid,
                            receiver_uuid: req.receiver_uuid,
                            msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Assign as u64,
                            status: ErrorCode::InvalidParameters,
                        };
                    }
                    mapping.id = *id;
                    mapping.ntype = *ntype;
                    self.global_bitmap |= 1 << id;
                }
                None => {
                    error!("No matching cookie for assign: {cookie}");
                    return NfySetupRsp {
                        reserved: 0,
                        sender_uuid: req.sender_uuid,
                        receiver_uuid: req.receiver_uuid,
                        msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Assign as u64,
                        status: ErrorCode::InvalidParameters,
                    };
                }
            }
        }

        NfySetupRsp {
            reserved: 0,
            sender_uuid: req.sender_uuid,
            receiver_uuid: req.receiver_uuid,
            msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Assign as u64,
            status: ErrorCode::Ok,
        }
    }

    fn nfy_unassign(&mut self, req: NotifyReq) -> NfySetupRsp {
        let entry_index = match self.nfy_find_entry(req.receiver_uuid) {
            Some(idx) => idx,
            None => {
                return NfySetupRsp {
                    reserved: 0,
                    sender_uuid: req.sender_uuid,
                    receiver_uuid: req.receiver_uuid,
                    msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Unassign as u64,
                    status: ErrorCode::InvalidParameters,
                };
            }
        };

        for (cookie, _id, _ntype) in req.notifications.iter().take(req.count as usize) {
            match self.nfy_find_matching_cookie(entry_index, *cookie) {
                Some(mapping_index) => {
                    let mapping = &mut self.entries[entry_index].mappings[mapping_index];
                    self.global_bitmap &= !(1 << mapping.id);
                    mapping.id = 0;
                    mapping.ntype = NotifyType::Global;
                    // Keep in_use = true and cookie — slot is reserved, just unassigned
                }
                None => {
                    error!("No matching cookie for unassign: {cookie}");
                    return NfySetupRsp {
                        reserved: 0,
                        sender_uuid: req.sender_uuid,
                        receiver_uuid: req.receiver_uuid,
                        msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Unassign as u64,
                        status: ErrorCode::InvalidParameters,
                    };
                }
            }
        }

        NfySetupRsp {
            reserved: 0,
            sender_uuid: req.sender_uuid,
            receiver_uuid: req.receiver_uuid,
            msg_info: MESSAGE_INFO_DIR_RESP + MessageID::Unassign as u64,
            status: ErrorCode::Ok,
        }
    }
}

impl Service for Notify {
    const UUID: Uuid = uuid!("e474d87e-5731-4044-a727-cb3e8cf3c8df");
    const NAME: &'static str = "Notify";

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let req: NotifyReq = msg.clone().into();
        debug!("Received notify command: {:?}", req.msg_info.message_id());

        let payload = match req.msg_info.message_id() {
            MessageID::Setup => DirectMessagePayload::from(self.nfy_setup(req)),
            MessageID::Destroy => DirectMessagePayload::from(self.nfy_destroy(req)),
            MessageID::Add => DirectMessagePayload::from(self.nfy_add(req)),
            MessageID::Remove => DirectMessagePayload::from(self.nfy_remove(req)),
            MessageID::Assign => DirectMessagePayload::from(self.nfy_assign(req)),
            MessageID::Unassign => DirectMessagePayload::from(self.nfy_unassign(req)),
        };

        Ok(MsgSendDirectResp2::from_req_with_payload(&msg, payload))
    }
}

// ===========================================================================
// Notify Unit Tests
// ===========================================================================
#[cfg(test)]
mod tests {
    use super::*;
    use odp_ffa::{DirectMessagePayload, HasRegisterPayload};
    use uuid::uuid;

    const NOTIFY_UUID: Uuid = uuid!("e474d87e-5731-4044-a727-cb3e8cf3c8df");
    const SENDER_UUID: Uuid = uuid!("aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee");
    const RECEIVER_UUID: Uuid = uuid!("11111111-2222-3333-4444-555555555555");

    /// Build a notify request with given message_id, count, and notification tuples.
    /// Each tuple is (cookie: u32, id: u16, ntype: 0=Global / 1=PerVcpu).
    fn notify_req(msg_id: MessageID, count: u8, notifs: &[(u32, u16, u8)]) -> MsgSendDirectReq2 {
        let mut regs = [0u64; 14];
        // regs[1-2] = sender_uuid as LE u128 split into two u64s
        let sender_le = SENDER_UUID.to_u128_le();
        regs[1] = sender_le as u64;
        regs[2] = (sender_le >> 64) as u64;
        // regs[3-4] = receiver_uuid as LE u128 split into two u64s
        let receiver_le = RECEIVER_UUID.to_u128_le();
        regs[3] = receiver_le as u64;
        regs[4] = (receiver_le >> 64) as u64;
        // regs[5] = msg_info (bits 0-2 = message_id)
        regs[5] = msg_id as u64;
        // regs[6] = count (lower 9 bits)
        regs[6] = count as u64;
        // regs[7..] = notification tuples: cookie(bits63:32) | id(bits31:23) | type(bit0)
        for (i, (cookie, id, ntype)) in notifs.iter().enumerate().take(7) {
            regs[7 + i] = ((*cookie as u64) << 32) | ((*id as u64) << 23) | (*ntype as u64);
        }
        let payload_bytes = regs.iter().flat_map(|r| r.to_le_bytes());
        let payload = DirectMessagePayload::from_iter(payload_bytes);
        MsgSendDirectReq2::new(0x0001, 0x8001, NOTIFY_UUID, payload)
    }

    /// Extract the ErrorCode status from a NfySetupRsp-shaped response.
    /// NfySetupRsp layout: reg0=reserved, reg1-2=sender_uuid, reg3-4=receiver_uuid,
    /// reg5=msg_info, reg6=status.
    fn resp_error_code(resp: &MsgSendDirectResp2) -> i64 {
        resp.payload().register_at(6) as i64
    }

    /// Extract msg_info from response.
    fn resp_msg_info(resp: &MsgSendDirectResp2) -> u64 {
        resp.payload().register_at(5)
    }

    // ===================================================================
    // Notify::Setup Test(s)
    // ===================================================================
    #[test]
    fn test_setup_registers_service() {
        let mut svc = Notify::new();
        let msg = notify_req(MessageID::Setup, 1, &[(100, 1, 0)]);
        let resp = svc.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_error_code(&resp), ErrorCode::Ok as i64);
        assert_eq!(resp_msg_info(&resp), MESSAGE_INFO_DIR_RESP + MessageID::Setup as u64);
    }

    #[test]
    fn test_setup_invalid_count_zero() {
        let mut svc = Notify::new();
        let msg = notify_req(MessageID::Setup, 0, &[]);
        let resp = svc.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_error_code(&resp), ErrorCode::InvalidParameters as i64);
    }

    #[test]
    fn test_setup_overflow_count() {
        let mut svc = Notify::new();
        // NOTIFY_MAX_MAPPINGS_PER_REQ is 8, so count >= 8 should fail
        let msg = notify_req(MessageID::Setup, NOTIFY_MAX_MAPPINGS_PER_REQ as u8, &[(100, 1, 0)]);
        let resp = svc.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_error_code(&resp), ErrorCode::InvalidParameters as i64);
    }

    // ===================================================================
    // Notify::Destroy Test(s)
    // ===================================================================
    #[test]
    fn test_destroy_after_setup() {
        let mut svc = Notify::new();
        // Setup first
        let setup_msg = notify_req(MessageID::Setup, 1, &[(100, 1, 0)]);
        let setup_resp = svc.ffa_msg_send_direct_req2(setup_msg).unwrap();
        assert_eq!(resp_error_code(&setup_resp), ErrorCode::Ok as i64);

        // Destroy with matching cookie/id/ntype
        let destroy_msg = notify_req(MessageID::Destroy, 1, &[(100, 1, 0)]);
        let destroy_resp = svc.ffa_msg_send_direct_req2(destroy_msg).unwrap();
        assert_eq!(resp_error_code(&destroy_resp), ErrorCode::Ok as i64);
        assert_eq!(
            resp_msg_info(&destroy_resp),
            MESSAGE_INFO_DIR_RESP + MessageID::Destroy as u64
        );
    }

    #[test]
    fn test_destroy_unregistered_returns_error() {
        let mut svc = Notify::new();
        let msg = notify_req(MessageID::Destroy, 1, &[(100, 1, 0)]);
        let resp = svc.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_error_code(&resp), ErrorCode::InvalidParameters as i64);
    }

    // ===================================================================
    // Notify::Add Test(s)
    // ===================================================================
    #[test]
    fn test_add_registers_mapping() {
        let mut svc = Notify::new();
        let msg = notify_req(MessageID::Add, 1, &[(200, 2, 0)]);
        let resp = svc.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_error_code(&resp), ErrorCode::Ok as i64);
        assert_eq!(resp_msg_info(&resp), MESSAGE_INFO_DIR_RESP + MessageID::Add as u64);
    }

    // ===================================================================
    // Notify::Remove Test(s)
    // ===================================================================
    #[test]
    fn test_add_then_remove() {
        let mut svc = Notify::new();
        // First: Add a mapping
        let add_msg = notify_req(MessageID::Add, 1, &[(200, 2, 0)]);
        let add_resp = svc.ffa_msg_send_direct_req2(add_msg).unwrap();
        assert_eq!(resp_error_code(&add_resp), ErrorCode::Ok as i64);

        // Then: Remove it
        let remove_msg = notify_req(MessageID::Remove, 1, &[(200, 2, 0)]);
        let remove_resp = svc.ffa_msg_send_direct_req2(remove_msg).unwrap();
        assert_eq!(resp_error_code(&remove_resp), ErrorCode::Ok as i64);
        assert_eq!(
            resp_msg_info(&remove_resp),
            MESSAGE_INFO_DIR_RESP + MessageID::Remove as u64
        );
    }

    #[test]
    fn test_remove_without_add_returns_error() {
        let mut svc = Notify::new();
        let msg = notify_req(MessageID::Remove, 1, &[(200, 2, 0)]);
        let resp = svc.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_error_code(&resp), ErrorCode::InvalidParameters as i64);
    }

    // ===================================================================
    // Notify::Assign Test(s)
    // ===================================================================
    #[test]
    fn test_assign_updates_mapping() {
        let mut svc = Notify::new();
        // Setup with 1 notification (cookie=100, id=1)
        let setup_msg = notify_req(MessageID::Setup, 1, &[(100, 1, 0)]);
        let setup_resp = svc.ffa_msg_send_direct_req2(setup_msg).unwrap();
        assert_eq!(resp_error_code(&setup_resp), ErrorCode::Ok as i64);

        // Assign same cookie with new id=5
        let assign_msg = notify_req(MessageID::Assign, 1, &[(100, 5, 0)]);
        let assign_resp = svc.ffa_msg_send_direct_req2(assign_msg).unwrap();
        assert_eq!(resp_error_code(&assign_resp), ErrorCode::Ok as i64);
        assert_eq!(
            resp_msg_info(&assign_resp),
            MESSAGE_INFO_DIR_RESP + MessageID::Assign as u64
        );
    }

    #[test]
    fn test_assign_without_entry_returns_error() {
        let mut svc = Notify::new();
        let msg = notify_req(MessageID::Assign, 1, &[(100, 5, 0)]);
        let resp = svc.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_error_code(&resp), ErrorCode::InvalidParameters as i64);
    }

    // ===================================================================
    // Notify::Unassign Test(s)
    // ===================================================================
    #[test]
    fn test_unassign_clears_event() {
        let mut svc = Notify::new();
        // Setup with 1 notification (cookie=100, id=1)
        let setup_msg = notify_req(MessageID::Setup, 1, &[(100, 1, 0)]);
        let setup_resp = svc.ffa_msg_send_direct_req2(setup_msg).unwrap();
        assert_eq!(resp_error_code(&setup_resp), ErrorCode::Ok as i64);

        // Unassign by cookie
        let unassign_msg = notify_req(MessageID::Unassign, 1, &[(100, 1, 0)]);
        let unassign_resp = svc.ffa_msg_send_direct_req2(unassign_msg).unwrap();
        assert_eq!(resp_error_code(&unassign_resp), ErrorCode::Ok as i64);
        assert_eq!(
            resp_msg_info(&unassign_resp),
            MESSAGE_INFO_DIR_RESP + MessageID::Unassign as u64
        );
    }

    #[test]
    fn test_unassign_without_entry_returns_error() {
        let mut svc = Notify::new();
        let msg = notify_req(MessageID::Unassign, 1, &[(100, 1, 0)]);
        let resp = svc.ffa_msg_send_direct_req2(msg).unwrap();
        assert_eq!(resp_error_code(&resp), ErrorCode::InvalidParameters as i64);
    }
}
