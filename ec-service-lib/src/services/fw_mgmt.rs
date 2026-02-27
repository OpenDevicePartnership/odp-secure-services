use crate::{Result, Service};
use log::{debug, error};
use odp_ffa::{DirectMessagePayload, HasRegisterPayload, MemRetrieveReq, MsgSendDirectReq2, MsgSendDirectResp2};
use odp_ffa::{Function, NotificationSet};
use uuid::{uuid, Uuid};

// Protocol CMD definitions for FwMgmt
const EC_CAP_INDIRECT_MSG: u8 = 0x0;
const EC_CAP_GET_FW_STATE: u8 = 0x1;
const EC_CAP_GET_SVC_LIST: u8 = 0x2;
const EC_CAP_GET_BID: u8 = 0x3;
const EC_CAP_TEST_NFY: u8 = 0x4;
const EC_CAP_MAP_SHARE: u8 = 0x5;

#[derive(Default)]
struct FwStateRsp {
    fw_version: u16,
    secure_state: u8,
    boot_status: u8,
}

impl From<FwStateRsp> for DirectMessagePayload {
    fn from(rsp: FwStateRsp) -> Self {
        let iter = rsp
            .fw_version
            .to_le_bytes()
            .into_iter()
            .chain(rsp.secure_state.to_le_bytes())
            .chain(rsp.boot_status.to_le_bytes());

        DirectMessagePayload::from_iter(iter)
    }
}

#[derive(Default)]
struct ServiceListRsp {
    status: i64,
    debug_mask: u16,
    battery_mask: u8,
    fan_mask: u8,
    thermal_mask: u8,
    hid_mask: u8,
    key_mask: u16,
}

impl From<ServiceListRsp> for DirectMessagePayload {
    fn from(rsp: ServiceListRsp) -> Self {
        let iter = rsp
            .status
            .to_le_bytes()
            .into_iter()
            .chain(rsp.debug_mask.to_le_bytes())
            .chain(rsp.battery_mask.to_le_bytes())
            .chain(rsp.fan_mask.to_le_bytes())
            .chain(rsp.thermal_mask.to_le_bytes())
            .chain(rsp.hid_mask.to_le_bytes())
            .chain(rsp.key_mask.to_le_bytes());
        DirectMessagePayload::from_iter(iter)
    }
}

#[derive(Default)]
struct GetBidRsp {
    _status: i64,
    _bid: u64,
}

impl From<GetBidRsp> for DirectMessagePayload {
    fn from(rsp: GetBidRsp) -> Self {
        let iter = rsp._status.to_le_bytes().into_iter().chain(rsp._bid.to_le_bytes());
        DirectMessagePayload::from_iter(iter)
    }
}

#[derive(Default)]
struct GenericRsp {
    _status: i64,
}

impl From<GenericRsp> for DirectMessagePayload {
    fn from(rsp: GenericRsp) -> Self {
        let iter = rsp._status.to_le_bytes().into_iter();
        DirectMessagePayload::from_iter(iter)
    }
}

#[derive(Default)]
pub struct FwMgmt {}

impl FwMgmt {
    pub fn new() -> Self {
        Self::default()
    }

    fn get_fw_state(&self) -> FwStateRsp {
        FwStateRsp {
            fw_version: 0x0100,
            secure_state: 0x0,
            boot_status: 0x1,
        }
    }

    fn get_svc_list(&self) -> ServiceListRsp {
        ServiceListRsp {
            status: 0x0,
            debug_mask: 0x1,
            battery_mask: 0x1,
            fan_mask: 0x1,
            thermal_mask: 0x1,
            hid_mask: 0x0,
            key_mask: 0x7,
        }
    }

    fn get_bid(&self) -> GetBidRsp {
        GetBidRsp {
            _status: 0x0,
            _bid: 0xdead0001,
        }
    }

    fn map_share(&self, _address: u64, _length: u64) -> GenericRsp {
        // TODO - do not hardcode address and length in MemRetrieveReq
        MemRetrieveReq::new().exec().unwrap();
        GenericRsp { _status: 0x0 }
    }

    fn test_notify(&self, msg: MsgSendDirectReq2) -> GenericRsp {
        // let nfy = FfaNotify {
        //     function_id: FunctionId::NotificationSet.into(),
        //     source_id: msg.destination_id,
        //     destination_id: msg.source_id,
        //     args64: [
        //         0x2, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        //     ],
        // };

        // let _result = nfy.exec();

        let flags = 0b10;
        let notification_bitmap = 0b10;
        NotificationSet::new(msg.destination_id(), msg.source_id(), flags, notification_bitmap)
            .exec()
            .unwrap();

        // Return status success
        GenericRsp { _status: 0x0 }
    }

    fn process_indirect(&self, seq_num: u16, _rx_buffer: u64, _tx_buffer: u64) -> GenericRsp {
        debug!("Processing indirect message: 0x{:x}", seq_num);
        // let msg = FfaIndirectMsg::new();
        // let mut in_buf: [u8; 256] = [0; 256];
        // let mut status;

        // unsafe {
        //     status = msg.read_indirect_msg(rx_buffer, seq_num, &mut in_buf);
        // };

        // if status == FfaError::Ok {
        //     error!("Indirect Message: {:?}", in_buf);
        // }

        // // Populate TX buffer with response and matching seq num
        // let buf: [u8; 16] = [
        //     0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF,
        // ];
        // unsafe {
        //     status = msg.write_indirect_msg(tx_buffer, seq_num, &buf);
        // };

        // GenericRsp { _status: status.into() }
        GenericRsp { _status: 0x0 }
    }
}

impl Service for FwMgmt {
    const UUID: Uuid = uuid!("330c1273-fde5-4757-9819-5b6539037502");
    const NAME: &'static str = "FwMgmt";

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let cmd = msg.payload().u8_at(0);
        debug!("Received FwMgmt command 0x{:x}", cmd);

        let payload = match cmd {
            EC_CAP_INDIRECT_MSG => DirectMessagePayload::from(self.process_indirect(
                msg.payload().u8_at(1) as u16,
                msg.payload().register_at(4),
                msg.payload().register_at(5),
            )),
            EC_CAP_GET_FW_STATE => DirectMessagePayload::from(self.get_fw_state()),
            EC_CAP_GET_SVC_LIST => DirectMessagePayload::from(self.get_svc_list()),
            EC_CAP_GET_BID => DirectMessagePayload::from(self.get_bid()),
            EC_CAP_TEST_NFY => DirectMessagePayload::from(self.test_notify(msg.clone())),
            EC_CAP_MAP_SHARE => {
                // First parameter is pointer to memory descriptor
                DirectMessagePayload::from(self.map_share(msg.payload().register_at(1), msg.payload().register_at(2)))
            }
            _ => {
                error!("Unknown FwMgmt Command: {}", cmd);
                return Err(odp_ffa::Error::Other("Unknown FwMgmt Command"));
            }
        };

        Ok(MsgSendDirectResp2::from_req_with_payload(&msg, payload))
    }
}
