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

    fn get_bix(&self, _msg: &MsgSendDirectReq2) -> BixRsp {
        BixRsp {
            events: self.events,
            status: self.status,
            last_full_charge: self.last_full_charge,
            cycle_count: self.cycle_count,
            state: self.state,
            present_rate: self.present_rate,
            remain_cap: self.remain_cap,
            present_volt: self.present_volt,
            psr_state: self.psr_state,
            psr_max_out: self.psr_max_out,
            psr_max_in: self.psr_max_in,
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

    fn get_psr(&self, _msg: &MsgSendDirectReq2) -> PsrRsp {
        PsrRsp {
            psr_state: self.psr_state,
        }
    }

    fn get_pif(&self, _msg: &MsgSendDirectReq2) -> PifRsp {
        PifRsp {
            max_power: self.pif_max_power,
        }
    }

    fn get_bps(&self, _msg: &MsgSendDirectReq2) -> ValueRsp {
        ValueRsp { value: self.bps_status }
    }

    fn handle_btp(&mut self, msg: &MsgSendDirectReq2) -> ValueRsp {
        // Byte 8 is a set flag: non-zero means "store the value at offset 4"
        // This allows setting trip_thres to 0 (disable trip point per ACPI)
        let set_flag = msg.payload().u8_at(8);
        if set_flag != 0 {
            self.trip_thres = msg.payload().u32_at(4);
        }
        ValueRsp { value: self.trip_thres }
    }

    fn get_bpt(&self, _msg: &MsgSendDirectReq2) -> ValueRsp {
        ValueRsp { value: self.peak_thres }
    }

    fn get_bpc(&self, _msg: &MsgSendDirectReq2) -> ValueRsp {
        ValueRsp { value: self.peak_level }
    }

    fn handle_bmc(&mut self, msg: &MsgSendDirectReq2) -> ValueRsp {
        let set_flag = msg.payload().u8_at(8);
        if set_flag != 0 {
            self.bmc_data = msg.payload().u32_at(4);
        }
        ValueRsp { value: self.bmc_data }
    }

    fn get_bmd(&self, _msg: &MsgSendDirectReq2) -> ValueRsp {
        ValueRsp { value: self.bmd_data }
    }

    fn get_bct(&self, _msg: &MsgSendDirectReq2) -> ValueRsp {
        ValueRsp {
            value: self.charge_time,
        }
    }

    fn get_btm(&self, _msg: &MsgSendDirectReq2) -> ValueRsp {
        ValueRsp { value: self.btm_temp }
    }

    fn get_bms(&self, _msg: &MsgSendDirectReq2) -> ValueRsp {
        ValueRsp { value: self.bms_data }
    }

    fn get_bma(&self, _msg: &MsgSendDirectReq2) -> ValueRsp {
        ValueRsp { value: self.bma_data }
    }

    fn get_sta(&self, _msg: &MsgSendDirectReq2) -> StaRsp {
        StaRsp {
            sta_status: self.sta_status,
        }
    }
}

impl Service for Battery {
    const UUID: Uuid = uuid!("25cb5207-ac36-427d-aaef-3aa78877d27e");
    const NAME: &'static str = "Battery";

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let cmd = msg.payload().u8_at(0);
        debug!("Received Battery command 0x{:x}", cmd);

        let payload = match cmd {
            EC_BAT_GET_BIX => DirectMessagePayload::from(self.get_bix(&msg)),
            EC_BAT_GET_BST => DirectMessagePayload::from(self.get_bst(&msg)),
            EC_BAT_GET_PSR => DirectMessagePayload::from(self.get_psr(&msg)),
            EC_BAT_GET_PIF => DirectMessagePayload::from(self.get_pif(&msg)),
            EC_BAT_GET_BPS => DirectMessagePayload::from(self.get_bps(&msg)),
            EC_BAT_GET_BTP => DirectMessagePayload::from(self.handle_btp(&msg)),
            EC_BAT_GET_BPT => DirectMessagePayload::from(self.get_bpt(&msg)),
            EC_BAT_GET_BPC => DirectMessagePayload::from(self.get_bpc(&msg)),
            EC_BAT_GET_BMC => DirectMessagePayload::from(self.handle_bmc(&msg)),
            EC_BAT_GET_BMD => DirectMessagePayload::from(self.get_bmd(&msg)),
            EC_BAT_GET_BCT => DirectMessagePayload::from(self.get_bct(&msg)),
            EC_BAT_GET_BTM => DirectMessagePayload::from(self.get_btm(&msg)),
            EC_BAT_GET_BMS => DirectMessagePayload::from(self.get_bms(&msg)),
            EC_BAT_GET_BMA => DirectMessagePayload::from(self.get_bma(&msg)),
            EC_BAT_GET_STA => DirectMessagePayload::from(self.get_sta(&msg)),
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

    /// Build a battery request with just the opcode command byte.
    fn bat_req(cmd: u8) -> MsgSendDirectReq2 {
        MsgSendDirectReq2::new(0, 0, Battery::UUID, DirectMessagePayload::from_iter(vec![cmd]))
    }

    /// Build a battery request with opcode + u32 value at offset 4 (for BTP/BMC set operations).
    /// Sets a flag byte at offset 8 to indicate "store this value".
    fn bat_req_with_value(cmd: u8, value: u32) -> MsgSendDirectReq2 {
        let mut bytes = [0u8; 14 * 8];
        bytes[0] = cmd;
        bytes[4..8].copy_from_slice(&value.to_le_bytes());
        bytes[8] = 1; // set flag
        MsgSendDirectReq2::new(0, 0, Battery::UUID, DirectMessagePayload::from_iter(bytes))
    }

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

    #[test]
    fn test_get_bix() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BIX)).unwrap();
        let bix = BixRsp::from(resp.payload());
        assert_eq!(bix.events, 0);
        assert_eq!(bix.status, 0);
        assert_eq!(bix.last_full_charge, 4500);
        assert_eq!(bix.cycle_count, 42);
        assert_eq!(bix.state, 0x1);
        assert_eq!(bix.present_rate, 500);
        assert_eq!(bix.remain_cap, 5000);
        assert_eq!(bix.present_volt, 12000);
        assert_eq!(bix.psr_state, 0x1);
        assert_eq!(bix.psr_max_out, 65000);
        assert_eq!(bix.psr_max_in, 0);
    }

    #[test]
    fn test_get_bst_from_state() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BST)).unwrap();
        let bst = BstRsp::from(resp.payload());
        assert_eq!(bst.state, 0x1);
        assert_eq!(bst.present_rate, 500);
        assert_eq!(bst.remaining_cap, 5000);
        assert_eq!(bst.present_volt, 12000);
    }

    #[test]
    fn test_get_psr() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_PSR)).unwrap();
        let psr = PsrRsp::from(resp.payload());
        assert_eq!(psr.psr_state, 0x1);
    }

    #[test]
    fn test_get_pif() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_PIF)).unwrap();
        let pif = PifRsp::from(resp.payload());
        assert_eq!(pif.max_power, 65000);
    }

    #[test]
    fn test_get_bps() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BPS)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 0x1);
    }

    #[test]
    fn test_get_btp_default() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BTP)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 10);
    }

    #[test]
    fn test_get_bpt() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BPT)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 80);
    }

    #[test]
    fn test_get_bpc() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BPC)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 100);
    }

    #[test]
    fn test_get_bmc_default() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BMC)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 0);
    }

    #[test]
    fn test_get_bmd() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BMD)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 0);
    }

    #[test]
    fn test_get_bct() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BCT)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 120);
    }

    #[test]
    fn test_get_btm() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BTM)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 2980);
    }

    #[test]
    fn test_get_bms() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BMS)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 0x1);
    }

    #[test]
    fn test_get_bma() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BMA)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 0);
    }

    #[test]
    fn test_get_sta() {
        let mut bat = Battery::new();
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_STA)).unwrap();
        let sta = StaRsp::from(resp.payload());
        assert_eq!(sta.sta_status, 0x1F);
    }

    #[test]
    fn test_btp_set_get_round_trip() {
        let mut bat = Battery::new();
        // Set trip_thres to 25 via payload value at offset 4
        let resp = bat
            .ffa_msg_send_direct_req2(bat_req_with_value(EC_BAT_GET_BTP, 25))
            .unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 25);
        // Get without set value — should return the persisted value
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BTP)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 25);
    }

    #[test]
    fn test_bmc_set_get_round_trip() {
        let mut bat = Battery::new();
        // Set bmc_data to 0x42
        let resp = bat
            .ffa_msg_send_direct_req2(bat_req_with_value(EC_BAT_GET_BMC, 0x42))
            .unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 0x42);
        // Get without set — verify persistence
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BMC)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 0x42);
    }

    #[test]
    fn test_btp_set_overwrites_previous() {
        let mut bat = Battery::new();
        // Set to 50
        let _ = bat
            .ffa_msg_send_direct_req2(bat_req_with_value(EC_BAT_GET_BTP, 50))
            .unwrap();
        // Overwrite to 75
        let _ = bat
            .ffa_msg_send_direct_req2(bat_req_with_value(EC_BAT_GET_BTP, 75))
            .unwrap();
        // Verify latest value
        let resp = bat.ffa_msg_send_direct_req2(bat_req(EC_BAT_GET_BTP)).unwrap();
        let val = ValueRsp::from(resp.payload());
        assert_eq!(val.value, 75);
    }

    #[test]
    fn test_unknown_command_returns_error() {
        let mut bat = Battery::new();
        let result = bat.ffa_msg_send_direct_req2(bat_req(0xFF));
        assert!(result.is_err());
    }
}
