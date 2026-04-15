use crate::service::Service;
use crate::Result;
use log::{debug, error};
use odp_ffa::{DirectMessagePayload, Function, HasRegisterPayload, MsgSendDirectReq2, MsgSendDirectResp2, Yield};
use uuid::{uuid, Builder, Uuid};

// Protocol CMD definitions for Thermal
const EC_THM_GET_TMP: u8 = 0x1;
const EC_THM_SET_THRS: u8 = 0x2;
const EC_THM_GET_THRS: u8 = 0x3;
const EC_THM_SET_SCP: u8 = 0x4;
const EC_THM_GET_VAR: u8 = 0x5;
const EC_THM_SET_VAR: u8 = 0x6;

const MAX_SENSORS: usize = 8;
const MAX_VARS: usize = 16;

#[derive(Default)]
struct GenericRsp {
    status: i64,
}

impl From<GenericRsp> for DirectMessagePayload {
    fn from(value: GenericRsp) -> Self {
        DirectMessagePayload::from_iter(value.status.to_le_bytes())
    }
}

#[derive(Default)]
struct TempRsp {
    status: i64,
    temp: u64,
}

impl From<TempRsp> for DirectMessagePayload {
    fn from(value: TempRsp) -> Self {
        DirectMessagePayload::from_iter(value.status.to_le_bytes().into_iter().chain(value.temp.to_le_bytes()))
    }
}

#[derive(Default)]
struct ThresholdReq {
    id: u8,
    timeout: u32,
    low_temp: u32,
    high_temp: u32,
}
impl From<&DirectMessagePayload> for ThresholdReq {
    fn from(msg: &DirectMessagePayload) -> ThresholdReq {
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

impl From<ReadVarReq> for DirectMessagePayload {
    fn from(value: ReadVarReq) -> Self {
        let iter = value
            .id
            .to_le_bytes()
            .into_iter()
            .chain(value.len.to_le_bytes())
            .chain(value.var_uuid.as_bytes().iter().copied());

        DirectMessagePayload::from_iter(iter)
    }
}

impl From<&DirectMessagePayload> for ReadVarReq {
    fn from(msg: &DirectMessagePayload) -> ReadVarReq {
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

impl From<ReadVarRsp> for DirectMessagePayload {
    fn from(value: ReadVarRsp) -> Self {
        DirectMessagePayload::from_iter(value.status.to_le_bytes().into_iter().chain(value.data.to_le_bytes()))
    }
}

#[derive(Default)]
struct SetVarReq {
    id: u8,
    len: u16,
    var_uuid: Uuid,
    data: u32,
}

impl From<&DirectMessagePayload> for SetVarReq {
    fn from(msg: &DirectMessagePayload) -> SetVarReq {
        SetVarReq {
            id: msg.u8_at(1),
            len: msg.u16_at(2),
            var_uuid: Builder::from_slice_le(msg.slice(4..20)).unwrap().into_uuid(),
            data: msg.u32_at(20),
        }
    }
}

#[derive(Default, Clone, Copy)]
struct ThresholdData {
    timeout: u32,
    low_temp: u32,
    high_temp: u32,
}

#[derive(Default, Clone, Copy)]
struct CoolingPolicyData {
    _policy_type: u8,
}

struct ThresholdRsp {
    status: i64,
    timeout: u32,
    low_temp: u32,
    high_temp: u32,
}

impl From<ThresholdRsp> for DirectMessagePayload {
    fn from(value: ThresholdRsp) -> Self {
        DirectMessagePayload::from_iter(
            value
                .status
                .to_le_bytes()
                .into_iter()
                .chain(value.timeout.to_le_bytes())
                .chain(value.low_temp.to_le_bytes())
                .chain(value.high_temp.to_le_bytes()),
        )
    }
}

struct CoolingPolicyReq {
    id: u8,
    policy_type: u8,
}

impl From<&DirectMessagePayload> for CoolingPolicyReq {
    fn from(msg: &DirectMessagePayload) -> CoolingPolicyReq {
        CoolingPolicyReq {
            id: msg.u8_at(1),
            policy_type: msg.u8_at(2),
        }
    }
}

pub struct Thermal {
    thresholds: [Option<ThresholdData>; MAX_SENSORS],
    cooling_policies: [Option<CoolingPolicyData>; MAX_SENSORS],
    variables: [(Uuid, u32); MAX_VARS],
    var_count: usize,
    var_timestamps: [u64; MAX_VARS],
    var_clock: u64,
}

impl Default for Thermal {
    fn default() -> Self {
        Self::new()
    }
}

impl Thermal {
    pub fn new() -> Self {
        Thermal {
            thresholds: [None; MAX_SENSORS],
            cooling_policies: [None; MAX_SENSORS],
            variables: [(Uuid::nil(), 0u32); MAX_VARS],
            var_count: 0,
            var_timestamps: [0u64; MAX_VARS],
            var_clock: 0,
        }
    }

    fn get_temperature(&mut self, msg: &MsgSendDirectReq2) -> TempRsp {
        debug!("get_temperature sensor 0x{:x}", msg.payload().u8_at(1));

        // Tell OS to delay 1 ms
        Yield::new(0x100000000).exec().unwrap();

        TempRsp {
            status: 0x0,
            temp: 0x1234,
        }
    }

    fn set_threshold(&mut self, msg: &MsgSendDirectReq2) -> GenericRsp {
        let req: ThresholdReq = msg.payload().into();
        debug!(
            "set_threshold sensor 0x{:x} timeout=0x{:x} low=0x{:x} high=0x{:x}",
            req.id, req.timeout, req.low_temp, req.high_temp
        );

        let idx = req.id as usize;
        if idx >= MAX_SENSORS {
            return GenericRsp { status: -1 };
        }

        self.thresholds[idx] = Some(ThresholdData {
            timeout: req.timeout,
            low_temp: req.low_temp,
            high_temp: req.high_temp,
        });

        GenericRsp { status: 0 }
    }

    fn get_threshold(&mut self, msg: &MsgSendDirectReq2) -> ThresholdRsp {
        let id = msg.payload().u8_at(1);
        let idx = id as usize;

        if idx >= MAX_SENSORS {
            return ThresholdRsp {
                status: -1,
                timeout: 0,
                low_temp: 0,
                high_temp: 0,
            };
        }

        match self.thresholds[idx] {
            Some(data) => ThresholdRsp {
                status: 0,
                timeout: data.timeout,
                low_temp: data.low_temp,
                high_temp: data.high_temp,
            },
            None => ThresholdRsp {
                status: 0,
                timeout: 0,
                low_temp: 0,
                high_temp: 0,
            },
        }
    }

    fn set_cooling_policy(&mut self, msg: &MsgSendDirectReq2) -> GenericRsp {
        let req: CoolingPolicyReq = msg.payload().into();
        debug!(
            "set_cooling_policy sensor 0x{:x} policy=0x{:x}",
            req.id, req.policy_type
        );

        let idx = req.id as usize;
        if idx >= MAX_SENSORS {
            return GenericRsp { status: -1 };
        }

        self.cooling_policies[idx] = Some(CoolingPolicyData {
            _policy_type: req.policy_type,
        });

        GenericRsp { status: 0 }
    }

    fn get_variable(&mut self, msg: &MsgSendDirectReq2) -> ReadVarRsp {
        let req: ReadVarReq = msg.payload().into();
        debug!(
            "get_variable instance id: 0x{:x} length: 0x{:x} uuid: {}",
            req.id, req.len, req.var_uuid
        );

        if req.len != 4 {
            error!("get_variable only supports DWORD read");
            return ReadVarRsp { status: -1, data: 0 };
        }

        // Linear scan for UUID match
        for i in 0..self.var_count {
            if self.variables[i].0 == req.var_uuid {
                // Touch timestamp for LRU tracking
                self.var_clock += 1;
                self.var_timestamps[i] = self.var_clock;
                return ReadVarRsp {
                    status: 0,
                    data: self.variables[i].1,
                };
            }
        }

        // UUID not found
        ReadVarRsp { status: -1, data: 0 }
    }

    fn set_variable(&mut self, msg: &MsgSendDirectReq2) -> GenericRsp {
        let req: SetVarReq = msg.payload().into();
        debug!(
            "set_variable instance id: 0x{:x} length: 0x{:x} uuid: {} data: 0x{:x}",
            req.id, req.len, req.var_uuid, req.data
        );

        // Upsert: check if UUID already exists
        for i in 0..self.var_count {
            if self.variables[i].0 == req.var_uuid {
                self.variables[i].1 = req.data;
                self.var_clock += 1;
                self.var_timestamps[i] = self.var_clock;
                return GenericRsp { status: 0 };
            }
        }

        // Insert into open slot or evict LRU
        if self.var_count < MAX_VARS {
            self.variables[self.var_count] = (req.var_uuid, req.data);
            self.var_clock += 1;
            self.var_timestamps[self.var_count] = self.var_clock;
            self.var_count += 1;
        } else {
            // LRU eviction: find entry with lowest timestamp
            let mut oldest_idx = 0;
            let mut oldest_ts = self.var_timestamps[0];
            for i in 1..MAX_VARS {
                if self.var_timestamps[i] < oldest_ts {
                    oldest_ts = self.var_timestamps[i];
                    oldest_idx = i;
                }
            }
            self.variables[oldest_idx] = (req.var_uuid, req.data);
            self.var_clock += 1;
            self.var_timestamps[oldest_idx] = self.var_clock;
        }

        GenericRsp { status: 0 }
    }
}

impl Service for Thermal {
    const UUID: Uuid = uuid!("31f56da7-593c-4d72-a4b3-8fc7171ac073");
    const NAME: &'static str = "Thermal";

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let cmd = msg.payload().u8_at(0);
        debug!("Received ThmMgmt command 0x{:x}", cmd);

        let payload = match cmd {
            EC_THM_GET_TMP => DirectMessagePayload::from(self.get_temperature(&msg)),
            EC_THM_SET_THRS => DirectMessagePayload::from(self.set_threshold(&msg)),
            EC_THM_GET_THRS => DirectMessagePayload::from(self.get_threshold(&msg)),
            EC_THM_SET_SCP => DirectMessagePayload::from(self.set_cooling_policy(&msg)),
            EC_THM_GET_VAR => DirectMessagePayload::from(self.get_variable(&msg)),
            EC_THM_SET_VAR => DirectMessagePayload::from(self.set_variable(&msg)),
            _ => {
                error!("Unknown Thermal Command: {}", cmd);
                return Err(odp_ffa::Error::Other("Unknown Thermal Command"));
            }
        };

        Ok(MsgSendDirectResp2::from_req_with_payload(&msg, payload))
    }
}
