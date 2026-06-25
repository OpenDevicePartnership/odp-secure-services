//! `Thermal` — FFA service that relays Normal-World Thermal requests to
//! the EC over MCTP. Mirrors [`crate::services::battery::Battery`].
//!
//! Service id `0x09`; commands `GetTmp=1, SetThrs=2, GetThrs=3, SetScp=4,
//! GetVar=5, SetVar=6`. Only `GetTmp` is relayed so far.

use core::cell::RefCell;

use uuid::{uuid, Uuid};

use crate::services::ec_relay::{take_array, EcRelayError, Relay};
use crate::{Result, Service};
use odp_ffa::{DirectMessagePayload, Error as FfaError, HasRegisterPayload, MsgSendDirectReq2, MsgSendDirectResp2};

pub const THERMAL_SERVICE_ID: u8 = 0x09;

/// Thermal command ids (FFA request byte 0 / ODP message id).
#[derive(Debug, Clone, Copy, PartialEq, Eq, num_enum::TryFromPrimitive, num_enum::IntoPrimitive)]
#[repr(u16)]
pub enum ThermalCommand {
    GetTmp = 1,
    SetThrs = 2,
    GetThrs = 3,
    SetScp = 4,
    GetVar = 5,
    SetVar = 6,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThermalError {
    Relay(EcRelayError),
    UnexpectedResponse,
}

impl From<EcRelayError> for ThermalError {
    fn from(e: EcRelayError) -> Self {
        ThermalError::Relay(e)
    }
}

/// FFA reply for `GetTmp` (`temp` = EC u32 DeciKelvin; `status: -1` on relay failure).
#[derive(Default, Debug, Clone, Copy)]
struct TempRsp {
    status: i64,
    temp: u64,
}

impl From<TempRsp> for DirectMessagePayload {
    fn from(value: TempRsp) -> Self {
        DirectMessagePayload::from_iter(value.status.to_le_bytes().into_iter().chain(value.temp.to_le_bytes()))
    }
}

pub struct Thermal<'r, R: Relay> {
    relay: &'r RefCell<R>,
}

impl<'r, R: Relay> Thermal<'r, R> {
    pub fn new(relay: &'r RefCell<R>) -> Self {
        Self { relay }
    }

    /// Relay a GetTmp round-trip; returns the EC's `u32` DeciKelvin.
    pub fn get_temperature(&self, instance_id: u8) -> core::result::Result<u32, ThermalError> {
        self.relay
            .borrow_mut()
            .invoke_request(
                THERMAL_SERVICE_ID,
                ThermalCommand::GetTmp.into(),
                &[instance_id],
                |body| {
                    let (temp, _) = take_array(body)?;
                    Ok(u32::from_le_bytes(temp))
                },
            )
            .map_err(ThermalError::Relay)
    }
}

impl<R: Relay> Service for Thermal<'_, R> {
    const UUID: Uuid = uuid!("31f56da7-593c-4d72-a4b3-8fc7171ac073");
    const NAME: &'static str = "Thermal";

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let cmd = msg.payload().u8_at(0);
        let Ok(command) = ThermalCommand::try_from(cmd as u16) else {
            return Err(FfaError::Other("Unknown Thermal Command"));
        };

        match command {
            ThermalCommand::GetTmp => {
                let instance_id = msg.payload().u8_at(1);
                let rsp = match self.get_temperature(instance_id) {
                    Ok(dk) => TempRsp {
                        status: 0,
                        temp: dk as u64,
                    },
                    Err(_) => TempRsp { status: -1, temp: 0 },
                };
                Ok(MsgSendDirectResp2::from_req_with_payload(
                    &msg,
                    DirectMessagePayload::from(rsp),
                ))
            }
            // The other five commands are relayed in a follow-up.
            ThermalCommand::SetThrs
            | ThermalCommand::GetThrs
            | ThermalCommand::SetScp
            | ThermalCommand::GetVar
            | ThermalCommand::SetVar => Err(FfaError::Other("Thermal command not yet relayed")),
        }
    }
}

// Wire-format gate: round-trips bytes through the EC's own serializer so
// any drift fails the build.

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::ec_relay::test_util::{
        frame_response_packets, strip_mctp_framing, LoopbackTransport, TimeoutUart,
    };
    use crate::services::ec_relay::{self, EcRelay, MctpSerialTransport};
    use embedded_services::relay::SerializableMessage;
    use thermal_service_relay::{DeciKelvin, ThermalRequest, ThermalResponse};

    const GET_TMP_RESPONSE_BODY_LEN: usize = 4;

    #[test]
    fn produces_canonical_get_tmp_request_bytes() {
        // EC GetTmp response: 2982 dK ≈ 25 °C.
        let mut response_payload = [0u8; GET_TMP_RESPONSE_BODY_LEN];
        let n = ThermalResponse::ThermalGetTmpResponse {
            temperature: DeciKelvin(2982),
        }
        .serialize(&mut response_payload)
        .expect("ec-side serialize");
        assert_eq!(n, GET_TMP_RESPONSE_BODY_LEN, "GetTmp response body must be 4 bytes");

        let response_header = ec_relay::build_odp_header(false, THERMAL_SERVICE_ID, ThermalCommand::GetTmp.into());
        let framed_response = frame_response_packets(response_header, &response_payload);

        let mut transport = LoopbackTransport::new();
        transport.prime_rx(framed_response.iter().copied());
        let relay = RefCell::new(EcRelay::new(transport));
        let svc = Thermal::new(&relay);

        let dk = svc.get_temperature(0x07).expect("relay GetTmp");
        assert_eq!(dk, 2982);

        // Request bytes: OdpHeader [0x02, 0x09, 0x00, 0x01] + payload [0x07].
        let tx_bytes = relay.borrow().transport().tx.clone();
        let inner_tx = strip_mctp_framing(&tx_bytes);
        assert_eq!(
            inner_tx,
            std::vec![0x02, 0x09, 0x00, 0x01, 0x07],
            "Thermal GetTmp request wire bytes must match the EC's expected encoding exactly"
        );

        // The SP-produced bytes parse back via the EC's own deserializer.
        let (is_req, svc_id, _is_err, msg_id) = ec_relay::parse_odp_header(&inner_tx[..4]).expect("parse header");
        assert!(is_req, "must be a request");
        assert_eq!(svc_id, THERMAL_SERVICE_ID);
        assert_eq!(msg_id, u16::from(ThermalCommand::GetTmp));
        let decoded =
            ThermalRequest::deserialize(msg_id, &inner_tx[4..]).expect("ec-side decoder must accept SP-produced bytes");
        assert!(
            matches!(decoded, ThermalRequest::ThermalGetTmpRequest { instance_id: 0x07 }),
            "EC-side decoder must reconstruct the original request variant"
        );
    }

    #[test]
    fn get_temperature_surfaces_transport_read_timeout_as_relay_err() {
        let transport = MctpSerialTransport::new(TimeoutUart);
        let relay = RefCell::new(EcRelay::new(transport));
        let svc = Thermal::new(&relay);
        let err = svc.get_temperature(0).expect_err("timeout should propagate");
        assert_eq!(err, ThermalError::Relay(EcRelayError::TransportReadTimeout));
    }

    #[test]
    fn get_temperature_rejects_short_response_body() {
        // EC replies with a valid GetTmp header but a 2-byte payload (< 4).
        let response_header = ec_relay::build_odp_header(false, THERMAL_SERVICE_ID, ThermalCommand::GetTmp.into());
        let framed = frame_response_packets(response_header, &[0xAA, 0xBB]);
        let mut transport = LoopbackTransport::new();
        transport.prime_rx(framed.iter().copied());
        let relay = RefCell::new(EcRelay::new(transport));
        let svc = Thermal::new(&relay);
        assert_eq!(
            svc.get_temperature(0x07),
            Err(ThermalError::Relay(EcRelayError::BodyTooShort))
        );
    }
}
