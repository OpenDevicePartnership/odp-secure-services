use crate::{Result, Service};
use log::{debug, error, info};
use odp_ffa::{ErrorCode, MsgSendDirectReq2, MsgSendDirectResp2, Payload, RegisterPayload};
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
        let src_id = msg.source_id();
        let sender_uuid = Uuid::from_u128_le(((msg.register_at(2) as u128) << 64) | (msg.register_at(1) as u128));
        let receiver_uuid = Uuid::from_u128_le(((msg.register_at(4) as u128) << 64) | (msg.register_at(3) as u128));
        let msg_info = MessageInfo::from_raw(msg.register_at(5));
        let count = (msg.register_at(6) & 0x1ff).min(7) as u8; // Count is lower 9 bits
        let mut notifications = [(0, 0, NotifyType::Global); 7];
        for (i, notif) in notifications.iter_mut().enumerate().take(count as usize) {
            *notif = NotifyReq::extract_tuple(msg.register_at(7 + i));
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

impl From<NfyGenericRsp> for RegisterPayload {
    fn from(value: NfyGenericRsp) -> Self {
        RegisterPayload::from_iter(value.status.to_le_bytes())
    }
}

impl From<NfySetupRsp> for RegisterPayload {
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
        RegisterPayload::from_iter(payload_bytes_iter)
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
}

const UUID: Uuid = uuid!("e474d87e-5731-4044-a727-cb3e8cf3c8df");

impl Service for Notify {
    fn service_name(&self) -> &'static str {
        "Notify"
    }

    fn service_uuid(&self) -> Uuid {
        UUID
    }

    async fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let req: NotifyReq = msg.clone().into();
        debug!("Received notify command: {:?}", req.msg_info.message_id());

        let payload = match req.msg_info.message_id() {
            MessageID::Setup => RegisterPayload::from(self.nfy_setup(req)),
            MessageID::Destroy => RegisterPayload::from(self.nfy_destroy(req)),
            MessageID::Add | MessageID::Remove | MessageID::Assign | MessageID::Unassign => {
                // For Add, Remove, Assign, and Unassign, we just return unsupported
                NfyGenericRsp {
                    status: ErrorCode::NotSupported as i64,
                }
                .into()
            }
        };

        Ok(MsgSendDirectResp2::from_req_with_payload(&msg, payload))
    }
}
