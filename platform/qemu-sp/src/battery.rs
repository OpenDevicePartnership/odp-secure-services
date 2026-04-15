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

struct BixRsp {
    events: u32,
    status: u32,
    last_full_charge: u32,
    cycle_count: u32,
    state: u32,
    present_rate: u32,
    remain_cap: u32,
    present_volt: u32,
    psr_state: u32,
    psr_max_out: u32,
    psr_max_in: u32,
}

impl From<BixRsp> for DirectMessagePayload {
    fn from(value: BixRsp) -> Self {
        let regs = [
            value.events,
            value.status,
            value.last_full_charge,
            value.cycle_count,
            value.state,
            value.present_rate,
            value.remain_cap,
            value.present_volt,
            value.psr_state,
            value.psr_max_out,
            value.psr_max_in,
        ];
        DirectMessagePayload::from_iter(regs.iter().flat_map(|&reg| u32::to_le_bytes(reg)))
    }
}

impl From<&DirectMessagePayload> for BixRsp {
    fn from(payload: &DirectMessagePayload) -> Self {
        BixRsp {
            events: payload.u32_at(0),
            status: payload.u32_at(4),
            last_full_charge: payload.u32_at(8),
            cycle_count: payload.u32_at(12),
            state: payload.u32_at(16),
            present_rate: payload.u32_at(20),
            remain_cap: payload.u32_at(24),
            present_volt: payload.u32_at(28),
            psr_state: payload.u32_at(32),
            psr_max_out: payload.u32_at(36),
            psr_max_in: payload.u32_at(40),
        }
    }
}

struct PsrRsp {
    psr_state: u32,
}

impl From<PsrRsp> for DirectMessagePayload {
    fn from(value: PsrRsp) -> Self {
        DirectMessagePayload::from_iter(value.psr_state.to_le_bytes())
    }
}

impl From<&DirectMessagePayload> for PsrRsp {
    fn from(payload: &DirectMessagePayload) -> Self {
        PsrRsp {
            psr_state: payload.u32_at(0),
        }
    }
}

struct PifRsp {
    max_power: u32,
}

impl From<PifRsp> for DirectMessagePayload {
    fn from(value: PifRsp) -> Self {
        DirectMessagePayload::from_iter(value.max_power.to_le_bytes())
    }
}

impl From<&DirectMessagePayload> for PifRsp {
    fn from(payload: &DirectMessagePayload) -> Self {
        PifRsp {
            max_power: payload.u32_at(0),
        }
    }
}

struct StaRsp {
    sta_status: u32,
}

impl From<StaRsp> for DirectMessagePayload {
    fn from(value: StaRsp) -> Self {
        DirectMessagePayload::from_iter(value.sta_status.to_le_bytes())
    }
}

impl From<&DirectMessagePayload> for StaRsp {
    fn from(payload: &DirectMessagePayload) -> Self {
        StaRsp {
            sta_status: payload.u32_at(0),
        }
    }
}

struct ValueRsp {
    value: u32,
}

impl From<ValueRsp> for DirectMessagePayload {
    fn from(value: ValueRsp) -> Self {
        DirectMessagePayload::from_iter(value.value.to_le_bytes())
    }
}

impl From<&DirectMessagePayload> for ValueRsp {
    fn from(payload: &DirectMessagePayload) -> Self {
        ValueRsp {
            value: payload.u32_at(0),
        }
    }
}

#[allow(dead_code)]
pub struct Battery {
    // BIX fields
    events: u32,
    status: u32,
    last_full_charge: u32,
    cycle_count: u32,
    // BST fields (used by get_bst)
    state: u32,
    present_rate: u32,
    remain_cap: u32,
    present_volt: u32,
    // PSR fields
    psr_state: u32,
    psr_max_out: u32,
    psr_max_in: u32,
    // BPT/BPC fields
    peak_level: u32,
    peak_power: u32,
    sus_level: u32,
    sus_power: u32,
    peak_thres: u32,
    sus_thres: u32,
    // BTP field (settable)
    trip_thres: u32,
    // BMC/BMD fields
    bmc_data: u32,
    bmd_data: u32,
    bmd_flags: u32,
    bmd_count: u32,
    // BCT/BTM/BMS/BMA fields
    charge_time: u32,
    run_time: u32,
    sample_time: u32,
    // STA field
    sta_status: u32,
    // PIF fields
    pif_max_power: u32,
    // BPS field
    bps_status: u32,
    // BMA field
    bma_data: u32,
    // BMS field
    bms_data: u32,
    // BTM field
    btm_temp: u32,
}

impl Default for Battery {
    fn default() -> Self {
        Self::new()
    }
}

impl Battery {
    pub fn new() -> Self {
        Battery {
            events: 0,
            status: 0,              // no pending events
            last_full_charge: 4500, // mWh
            cycle_count: 42,
            state: 0x1,          // discharging
            present_rate: 500,   // mW draw
            remain_cap: 5000,    // mWh remaining
            present_volt: 12000, // 12V in mV
            psr_state: 0x1,      // AC adapter present
            psr_max_out: 65000,  // 65W max output (mW)
            psr_max_in: 0,
            peak_level: 100,   // percentage
            peak_power: 45000, // mW
            sus_level: 30,
            sus_power: 15000,
            peak_thres: 80,
            sus_thres: 20,
            trip_thres: 10, // default trip point at 10%
            bmc_data: 0,
            bmd_data: 0,
            bmd_flags: 0,
            bmd_count: 0,
            charge_time: 120,     // minutes to full
            run_time: 300,        // minutes remaining
            sample_time: 1000,    // ms between samples
            sta_status: 0x1F,     // present + enabled + functional + show in UI
            pif_max_power: 65000, // mW
            bps_status: 0x1,      // battery physically present
            bma_data: 0,
            bms_data: 0x1,  // managed
            btm_temp: 2980, // 298.0K (25°C in tenths of Kelvin)
        }
    }

    fn get_bst(&self, _msg: &MsgSendDirectReq2) -> BstRsp {
        BstRsp {
            state: self.state,
            present_rate: self.present_rate,
            remaining_cap: self.remain_cap,
            present_volt: self.present_volt,
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
