//! `Thermal` — FFA service that proxies Normal-World Thermal requests
//! to the EC's `ThermalServiceRelayHandler` over MCTP.
//!
//! Mirrors [`crate::services::battery::Battery`] exactly — see that file
//! for the canonical pattern (generic `<'r, R: Relay>` shape,
//! `&RefCell<R>` sharing of a single physical EC channel, manual
//! SP-side serialization rationale, and the wire-format gate test).
//!
//! # Wire format (must match the EC's `OdpRelayHandler` byte-for-byte)
//!
//! Thermal service id = `0x09`; command discriminants follow
//! `embedded-services/thermal-service-relay/src/serialization.rs::ThermalCmd`
//! (`GetTmp = 1`, `SetThrs = 2`, `GetThrs = 3`, `SetScp = 4`,
//! `GetVar = 5`, `SetVar = 6`).
//!
//! T2 (this commit) relays `GetTmp` only:
//!
//! Request body (post OdpHeader): `[instance_id: u8]` (1 byte).
//! Response body (post OdpHeader): `[temperature: u32 LE DeciKelvin]` (4 bytes).
//!
//! The other five commands return `odp_ffa::Error::Other(..)` for now;
//! T3 wires them on top of this shape.
//!
//! # SP-side runtime serialization is manual
//!
//! `embedded_services::relay::SerializableMessage` does not compile for
//! `aarch64-unknown-none-softfloat` (the SBSA SP target) because
//! `embassy-sync::ThreadModeRawMutex` is `cortex_m`-gated. As a
//! workaround, SP-side runtime serialization is performed MANUALLY
//! (1-byte request payload; 4 LE bytes for the GetTmp response). The
//! wire-format gate test in the `tests` module below round-trips bytes
//! through the EC's OWN `SerializableMessage` impl (via
//! `[dev-dependencies]`) so any drift fails the build.

use core::cell::RefCell;

use uuid::{uuid, Uuid};

use crate::services::ec_relay::{EcRelayError, Relay};
use crate::{Result, Service};
use odp_ffa::{DirectMessagePayload, Error as FfaError, HasRegisterPayload, MsgSendDirectReq2, MsgSendDirectResp2};

/// ODP service-id for Thermal (matches the EC `OdpRelayHandler`
/// instantiation in `embedded-services::thermal-service-relay`).
pub const THERMAL_SERVICE_ID: u8 = 0x09;

/// `ThermalCmd` discriminants from
/// `embedded-services/thermal-service-relay/src/serialization.rs`. The
/// enum itself is private to the EC crate; we mirror the integer
/// constants here.
pub const THERMAL_CMD_GET_TMP: u16 = 1;
pub const THERMAL_CMD_SET_THRS: u16 = 2;
pub const THERMAL_CMD_GET_THRS: u16 = 3;
pub const THERMAL_CMD_SET_SCP: u16 = 4;
pub const THERMAL_CMD_GET_VAR: u16 = 5;
pub const THERMAL_CMD_SET_VAR: u16 = 6;

/// Body of a `GetTmp` response, post-OdpHeader (4 LE bytes / one u32 DeciKelvin).
pub const GET_TMP_RESPONSE_BODY_LEN: usize = 4;

/// Errors returned by [`Thermal::get_temperature`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThermalError {
    /// Relay (MCTP + transport) failure — see [`EcRelayError`].
    Relay(EcRelayError),
    /// EC's response was not a Thermal service response or not the
    /// expected message id.
    UnexpectedResponse,
}

impl From<EcRelayError> for ThermalError {
    fn from(e: EcRelayError) -> Self {
        ThermalError::Relay(e)
    }
}

/// FFA reply for `GetTmp`. `temp` is `u32 DeciKelvin` zero-extended
/// to `u64`. `status: -1` indicates relay failure (matches the
/// existing in-SP mock's reply shape that Normal-World callers expect).
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

/// `Thermal` holds a handle to a shared [`Relay`] — it owns no
/// transport, no assembly buffer, no MCTP framing state. Construction
/// takes only a borrow of the wiring-layer-owned `RefCell<R>`. Mirrors
/// [`crate::services::battery::Battery`] field-for-field; see that
/// type's doc-comment for the rationale of generic `R: Relay`.
pub struct Thermal<'r, R: Relay> {
    relay: &'r RefCell<R>,
}

impl<'r, R: Relay> Thermal<'r, R> {
    pub fn new(relay: &'r RefCell<R>) -> Self {
        Self { relay }
    }

    /// Drive a single GetTmp request/response round-trip over the EC
    /// MCTP relay. Returns the raw `u32 DeciKelvin` or a relay /
    /// wire-format error.
    pub fn get_temperature(&self, instance_id: u8) -> core::result::Result<u32, ThermalError> {
        self.relay
            .borrow_mut()
            .invoke_request(
                THERMAL_SERVICE_ID,
                THERMAL_CMD_GET_TMP,
                &[instance_id],
                GET_TMP_RESPONSE_BODY_LEN,
                |payload| u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            )
            .map_err(ThermalError::Relay)
    }
}

impl<R: Relay> Service for Thermal<'_, R> {
    const UUID: Uuid = uuid!("31f56da7-593c-4d72-a4b3-8fc7171ac073");
    const NAME: &'static str = "Thermal";

    fn ffa_msg_send_direct_req2(&mut self, msg: MsgSendDirectReq2) -> Result<MsgSendDirectResp2> {
        let cmd = msg.payload().u8_at(0);

        match cmd as u16 {
            THERMAL_CMD_GET_TMP => {
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
            // T3 will relay these five commands to the EC. For now they
            // surface as a structured FFA error so the dispatch match
            // stays exhaustive and the in-SP mock behavior is gone.
            THERMAL_CMD_SET_THRS
            | THERMAL_CMD_GET_THRS
            | THERMAL_CMD_SET_SCP
            | THERMAL_CMD_GET_VAR
            | THERMAL_CMD_SET_VAR => Err(FfaError::Other("thermal: command not yet relayed")),
            _ => Err(FfaError::Other("Unknown Thermal Command")),
        }
    }
}

// ===========================================================================
// Wire-format compatibility gate
//
// Round-trips bytes through the EC's OWN `SerializableMessage::serialize` /
// `deserialize` impls from `embedded-services/thermal-service-relay`. Any
// drift in field order, endianness, or discriminant numbering causes the
// assertion to fail. Runs on host (std available) — no QEMU.
// ===========================================================================

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::services::ec_relay::test_util::{frame_response_packets, strip_mctp_framing, LoopbackTransport, TimeoutUart};
    use crate::services::ec_relay::{self, EcRelay, MctpSerialTransport};
    use embedded_services::relay::SerializableMessage;
    use thermal_service_relay::{DeciKelvin, ThermalRequest, ThermalResponse};

    #[test]
    fn produces_canonical_get_tmp_request_bytes() {
        // -- Synthesize a canonical EC response: GetTmp body = u32 LE
        //    DeciKelvin. 2982 dK ≈ 25.05 °C.
        let mut response_payload = [0u8; GET_TMP_RESPONSE_BODY_LEN];
        let n = ThermalResponse::ThermalGetTmpResponse {
            temperature: DeciKelvin(2982),
        }
        .serialize(&mut response_payload)
        .expect("ec-side serialize");
        assert_eq!(n, GET_TMP_RESPONSE_BODY_LEN, "GetTmp response body must be 4 bytes");

        let response_header = ec_relay::build_odp_header(false, THERMAL_SERVICE_ID, THERMAL_CMD_GET_TMP);
        let framed_response = frame_response_packets(response_header, &response_payload);

        // -- Construct the shared EcRelay (wrapping a loopback transport)
        //    and hand a borrow to the Thermal service.
        let mut transport = LoopbackTransport::new();
        transport.prime_rx(framed_response.iter().copied());
        let relay = RefCell::new(EcRelay::new(transport));
        let svc = Thermal::new(&relay);

        // -- Drive the round-trip.
        let dk = svc.get_temperature(0x07).expect("relay GetTmp");
        assert_eq!(dk, 2982);

        // -- ASSERT 1 (request side, byte-level):
        //    Thermal's TX bytes (MCTP-framed) decode back to exactly:
        //      OdpHeader [0x02, 0x09, 0x00, 0x01]  +  payload [0x07]
        let tx_bytes = relay.borrow().transport().tx.clone();
        let inner_tx = strip_mctp_framing(&tx_bytes);
        assert_eq!(
            inner_tx,
            std::vec![0x02, 0x09, 0x00, 0x01, 0x07],
            "Thermal GetTmp request wire bytes must match the EC's expected encoding exactly"
        );

        // -- ASSERT 2 (round-trip via the EC's OWN deserializer):
        //    The bytes Thermal produced parse back to
        //    GetTmpRequest { instance_id: 0x07 } via
        //    `ThermalRequest::deserialize`.
        let (is_req, svc_id, _is_err, msg_id) =
            ec_relay::parse_odp_header(&inner_tx[..4]).expect("parse header");
        assert!(is_req, "must be a request");
        assert_eq!(svc_id, THERMAL_SERVICE_ID);
        assert_eq!(msg_id, THERMAL_CMD_GET_TMP);
        let decoded = ThermalRequest::deserialize(msg_id, &inner_tx[4..])
            .expect("ec-side decoder must accept SP-produced bytes");
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
}
