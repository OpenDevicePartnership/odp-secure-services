use ec_service_lib::{Result, Service};
use log::{debug, error};
use odp_ffa::{DirectMessagePayload, HasRegisterPayload, MsgSendDirectReq2, MsgSendDirectResp2};
use uuid::{uuid, Uuid};

// Protocol CMD definitions for Battery
const EC_BAT_GET_BIX: u8 = 0x1;
const EC_BAT_GET_BST: u8 = 0x2;
const EC_BAT_GET_PSR: u8 = 0x3;
const EC_BAT_GET_PIF: u8 = 0x4;
const EC_BAT_GET_BPS: u8 = 0x5;
const EC_BAT_GET_BTP: u8 = 0x6;
const EC_BAT_GET_BPT: u8 = 0x7;
const EC_BAT_GET_BPC: u8 = 0x8;
const EC_BAT_GET_BMC: u8 = 0x9;
const EC_BAT_GET_BMD: u8 = 0xa;
const EC_BAT_GET_BCT: u8 = 0xb;
const EC_BAT_GET_BTM: u8 = 0xc;
const EC_BAT_GET_BMS: u8 = 0xd;
const EC_BAT_GET_BMA: u8 = 0xe;
const EC_BAT_GET_STA: u8 = 0xf;

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
struct BstRsp {
    state: u32,
    present_rate: u32,
    remaining_cap: u32,
    present_volt: u32,
}

impl From<BstRsp> for DirectMessagePayload {
    fn from(value: BstRsp) -> Self {
        let payload_regs = [value.state, value.present_rate, value.remaining_cap, value.present_volt];
        DirectMessagePayload::from_iter(payload_regs.iter().flat_map(|&reg| u32::to_le_bytes(reg).into_iter()))
    }
}

impl From<&DirectMessagePayload> for BstRsp {
    fn from(payload: &DirectMessagePayload) -> Self {
        BstRsp {
            state: payload.u32_at(0),
            present_rate: payload.u32_at(4),
            remaining_cap: payload.u32_at(8),
            present_volt: payload.u32_at(12),
        }
    }
}

#[derive(Default)]
pub struct Battery {}

impl Battery {
    pub fn new() -> Self {
        Self::default()
    }

    fn get_bst(&self, _msg: &MsgSendDirectReq2) -> BstRsp {
        BstRsp {
            state: 0x1,          // Battery discharging
            present_rate: 500,   // Power being supplied to battery
            remaining_cap: 5000, // Remaining capacity of battery
            present_volt: 12000, // 12V or 12000mV
        }
    }

    fn generic_test(&self, _msg: &MsgSendDirectReq2) -> GenericRsp {
        GenericRsp { status: 0x0 }
    }
}

impl Service for Battery {
    const UUID: Uuid = uuid!("25cb5207-ac36-427d-aaef-3aa78877d27e");
    const NAME: &'static str = "Battery";

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let cmd = msg.payload().u8_at(0);
        debug!("Received Battery command 0x{:x}", cmd);

        let payload = match cmd {
            EC_BAT_GET_BIX => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BST => DirectMessagePayload::from(self.get_bst(&msg)),
            EC_BAT_GET_PSR => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_PIF => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BPS => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BTP => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BPT => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BPC => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BMC => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BMD => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BCT => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BTM => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BMS => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_BMA => DirectMessagePayload::from(self.generic_test(&msg)),
            EC_BAT_GET_STA => DirectMessagePayload::from(self.generic_test(&msg)),
            _ => {
                error!("Unknown Battery Command: {}", cmd);
                return Err(odp_ffa::Error::Other("Unknown Battery Command"));
            }
        };

        Ok(MsgSendDirectResp2::from_req_with_payload(&msg, payload))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use odp_ffa::HasRegisterPayload;

    #[test]
    fn battery_get_bst_works() {
        let mut bat = Battery::new();
        let msg = MsgSendDirectReq2::new(
            0,
            0,
            Battery::UUID,
            DirectMessagePayload::from_iter(vec![EC_BAT_GET_BST]),
        );
        let resp = bat.ffa_msg_send_direct_req2(msg).unwrap();
        let payload = resp.payload();
        let bst = BstRsp::from(payload);
        assert_eq!(bst.state, 0x1);
        assert_eq!(bst.present_rate, 500);
        assert_eq!(bst.remaining_cap, 5000);
        assert_eq!(bst.present_volt, 12000);
    }
}
