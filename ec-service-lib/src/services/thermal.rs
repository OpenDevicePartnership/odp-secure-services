use crate::service::{Result, Service};
use log::{debug, error};
use odp_ffa::{Function, MsgSendDirectReq2, MsgSendDirectResp2, Payload, RegisterPayload, Yield};
use uuid::{uuid, Builder, Uuid};

// Protocol CMD definitions for Thermal
const EC_THM_GET_TMP: u8 = 0x1;
const EC_THM_SET_THRS: u8 = 0x2;
const EC_THM_GET_THRS: u8 = 0x3;
const EC_THM_SET_SCP: u8 = 0x4;
const EC_THM_GET_VAR: u8 = 0x5;
const EC_THM_SET_VAR: u8 = 0x6;

#[derive(Default)]
struct GenericRsp {
    status: i64,
}

impl From<GenericRsp> for RegisterPayload {
    fn from(value: GenericRsp) -> Self {
        RegisterPayload::from_iter(value.status.to_le_bytes())
    }
}

#[derive(Default)]
struct TempRsp {
    status: i64,
    temp: u64,
}

impl From<TempRsp> for RegisterPayload {
    fn from(value: TempRsp) -> Self {
        RegisterPayload::from_iter(value.status.to_le_bytes().into_iter().chain(value.temp.to_le_bytes()))
    }
}

#[derive(Default)]
struct ThresholdReq {
    id: u8,
    timeout: u32,
    low_temp: u32,
    high_temp: u32,
}
impl<P: Payload> From<&P> for ThresholdReq {
    fn from(msg: &P) -> ThresholdReq {
        ThresholdReq {
            id: msg.u8_at(1),
            timeout: msg.u16_at(3) as u32,
            low_temp: msg.u32_at(5),
            high_temp: msg.u32_at(9),
        }
    }
}

#[derive(Default)]
struct ReadVarReq {
    id: u8,
    len: u16,
    var_uuid: Uuid,
}

impl From<ReadVarReq> for RegisterPayload {
    fn from(value: ReadVarReq) -> Self {
        let iter = value
            .id
            .to_le_bytes()
            .into_iter()
            .chain(value.len.to_le_bytes())
            .chain(value.var_uuid.as_bytes().iter().copied());

        RegisterPayload::from_iter(iter)
    }
}

impl<P: Payload> From<&P> for ReadVarReq {
    fn from(msg: &P) -> ReadVarReq {
        ReadVarReq {
            id: msg.u8_at(1),
            len: msg.u16_at(2),
            var_uuid: Builder::from_slice_le(msg.slice(4..20)).unwrap().into_uuid(),
        }
    }
}

#[derive(Default)]
struct ReadVarRsp {
    status: i64,
    data: u32,
}

impl From<ReadVarRsp> for RegisterPayload {
    fn from(value: ReadVarRsp) -> Self {
        RegisterPayload::from_iter(value.status.to_le_bytes().into_iter().chain(value.data.to_le_bytes()))
    }
}

#[derive(Default)]
struct SetVarReq {
    id: u8,
    len: u16,
    var_uuid: Uuid,
    data: u32,
}

impl<P: Payload> From<&P> for SetVarReq {
    fn from(msg: &P) -> SetVarReq {
        SetVarReq {
            id: msg.u8_at(1),
            len: msg.u16_at(2),
            var_uuid: Builder::from_slice_le(msg.slice(4..20)).unwrap().into_uuid(),
            data: msg.u32_at(20),
        }
    }
}

#[derive(Default)]
pub struct Thermal {}

impl Thermal {
    pub fn new() -> Self {
        Self::default()
    }

    fn get_temperature(&self, msg: &MsgSendDirectReq2) -> TempRsp {
        debug!("get_temperature sensor 0x{:x}", msg.u8_at(1));

        // Tell OS to delay 1 ms
        Yield::new(0x100000000).exec().unwrap();

        TempRsp {
            status: 0x0,
            temp: 0x1234,
        }
    }

    fn set_threshold(&self, msg: &MsgSendDirectReq2) -> GenericRsp {
        let req: ThresholdReq = msg.into();
        debug!(
            "set_threshold temperature sensor 0x{:x}
                Timeout: 0x{:x}
                LowThreshold: 0x{:x}
                HighThreshold: 0x{:x}",
            req.id, req.timeout, req.low_temp, req.high_temp
        );

        GenericRsp { status: 0x0 }
    }

    fn get_threshold(&self, _msg: &MsgSendDirectReq2) -> GenericRsp {
        GenericRsp { status: 0x0 }
    }

    fn set_cooling_policy(&self, _msg: &MsgSendDirectReq2) -> GenericRsp {
        GenericRsp { status: 0x0 }
    }

    fn get_variable(&self, msg: &MsgSendDirectReq2) -> ReadVarRsp {
        let req: ReadVarReq = msg.into();
        debug!(
            "get_variable instance id: 0x{:x}
                length: 0x{:x}
                uuid: {}",
            req.id, req.len, req.var_uuid
        );

        // Only support DWORD customized IO for now
        if req.len != 4 {
            error!("get_variable only supports DWORD read")
        }

        ReadVarRsp {
            status: 0x0,
            data: 0xdeadbeef,
        }
    }

    fn set_variable(&self, msg: &MsgSendDirectReq2) -> GenericRsp {
        let req: SetVarReq = msg.into();
        debug!(
            "get_variable instance id: 0x{:x}
                length: 0x{:x}
                uuid: {}
                data: 0x{:x}",
            req.id, req.len, req.var_uuid, req.data
        );

        GenericRsp { status: 0x0 }
    }
}

const UUID: Uuid = uuid!("31f56da7-593c-4d72-a4b3-8fc7171ac073");

impl Service for Thermal {
    fn service_name(&self) -> &'static str {
        "Thermal"
    }

    fn service_uuid(&self) -> Uuid {
        UUID
    }

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let cmd = msg.u8_at(0);
        debug!("Received ThmMgmt command 0x{:x}", cmd);

        let payload = match cmd {
            EC_THM_GET_TMP => RegisterPayload::from(self.get_temperature(&msg)),
            EC_THM_SET_THRS => RegisterPayload::from(self.set_threshold(&msg)),
            EC_THM_GET_THRS => RegisterPayload::from(self.get_threshold(&msg)),
            EC_THM_SET_SCP => RegisterPayload::from(self.set_cooling_policy(&msg)),
            EC_THM_GET_VAR => RegisterPayload::from(self.get_variable(&msg)),
            EC_THM_SET_VAR => RegisterPayload::from(self.set_variable(&msg)),
            _ => {
                error!("Unknown Thermal Command: {}", cmd);
                return Err(odp_ffa::Error::Other("Unknown Thermal Command"));
            }
        };

        Ok(MsgSendDirectResp2::from_req_with_payload(&msg, payload))
    }
}
