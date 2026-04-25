//! SP-side MCTP framer over `SmbusEspiMedium` for SP↔EC interop.
//!
//! Service-agnostic: the public API takes `service_id`, `message_id`, and
//! a per-service payload, and produces / parses the SmbusEspi+MCTP+ODP-relay
//! wire framing that EC's `embedded-services::uart-service` expects.
//! Mirrors that crate byte-for-byte (verified against captured wire bytes
//! from a successful Battery::GetSta SP→EC→SP ping-pong on the QEMU SBSA
//! secure UART at `0x60030000`).
//!
//! Runtime path uses only stack/static buffers — no heap.

#![no_std]

use core::fmt;

use mctp_rs::smbus_espi::{SmbusEspiMedium, SmbusEspiReplyContext};
use mctp_rs::{
    EndpointId, MctpMessageHeaderTrait, MctpMessageTrait, MctpPacketContext, MctpPacketError,
    MctpPacketResult, MctpReplyContext, MctpSequenceNumber, MctpMessageTag,
};

// ---------------------------------------------------------------------------
// Wire constants
// ---------------------------------------------------------------------------
//
// All on-wire constants below were derived from a captured SP→EC ping-pong
// session and cross-checked against:
//
//   * EC's `embedded-services::uart-service::process_response`
//     (uart-service/src/lib.rs:65-95) — slave-address values + endpoint-id /
//     message-tag conventions.
//   * `mctp-rs`'s `SmbusEspiMedium::serialize`
//     (medium/smbus_espi.rs:58-102) — confirms reply_context source/dest
//     slave addresses are SWAPPED on serialize, and the bit_register layout
//     puts `destination_slave_address` in bits 25..31 of the LE u32 that is
//     then written `to_be_bytes()` (so it lands in wire byte 0).
//   * `mctp-rs`'s `serialize_packet` / `SerializePacketState::next`
//     (mctp_packet_context.rs:155-187, serialize.rs:25-85) — confirms the
//     transport-header source_endpoint_id / destination_endpoint_id are
//     SWAPPED on serialize (line 62-63 of serialize.rs), and that
//     `packet_sequence_number` is `.inc()`'d before being serialized (so the
//     initial value 0 yields wire seq=1 → matches captured byte 0xd3).
//   * `embedded-services::relay::mctp` ODP relay header `bitfield!`
//     (embedded-service/src/relay/mod.rs:285-303) — formal bit layout for
//     the ODP relay header u32 (big-endian on wire):
//         bit 25     : is_request (1 bit)
//         bits 23..16: service_id (8 bits)
//         bit 15     : is_error   (1 bit)
//         bits 14..0 : message_id (15 bits)
//
// Names below mirror `uart-service/src/lib.rs:69-78` so the SP↔EC interop
// surface is auditable side-by-side.

/// SP-side MCTP endpoint ID (matches `uart-service` source_endpoint_id 0x80).
const SP_EID: u8 = 0x80;
/// MCTP message tag — `uart-service` hard-codes 3.
const MCTP_MSG_TAG: u8 = 3;
/// SmbusEspi destination slave (semantic — wire byte ordering is swapped by
/// `SmbusEspiMedium::serialize`). Matches `uart-service` line 75: `destination_slave_address: 1`.
const SMBUS_DST_SLAVE: u8 = 1;
/// SmbusEspi source slave. Matches `uart-service` line 76: `source_slave_address: 0`.
const SMBUS_SRC_SLAVE: u8 = 0;
/// Number of leading wire bytes consumed by everything in front of the
/// per-service payload: 4-byte SmbusEspi header + 4-byte MCTP transport
/// header + 1-byte MCTP message-type + 4-byte ODP relay header.
const FRAME_OVERHEAD: usize = 4 + 4 + 1 + 4;
/// Internal mctp-rs scratch buffer size. Sized for a single-packet
/// SmbusEspi frame (max body 32 + 4 header + 1 PEC = 37) plus the prefix
/// `serialize_packet` carves off (msg_type 1 + odp header 4 = 5). Round to 64.
const SCRATCH_SIZE: usize = 64;
/// Maximum per-service payload the framer can encode/decode in one call.
/// Bounded by `SCRATCH_SIZE` minus `FRAME_OVERHEAD` minus the 1-byte PEC slot
/// the deserializer expects. Callers passing larger payloads get `BufTooSmall`.
pub const MAX_PAYLOAD_LEN: usize = SCRATCH_SIZE - FRAME_OVERHEAD - 1;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Errors surfaced by the framer. No allocator required.
#[derive(Debug, PartialEq, Eq)]
pub enum FramerError {
    /// Output / scratch buffer too small for the requested operation.
    BufTooSmall,
    /// Input buffer too short to contain the expected wire framing.
    Truncated,
    /// Underlying mctp-rs decoder/encoder rejected the input.
    DecodeFailed,
    /// Underlying mctp-rs decoder/encoder rejected the encode.
    EncodeFailed,
}

impl fmt::Display for FramerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FramerError::BufTooSmall => f.write_str("output buffer too small"),
            FramerError::Truncated => f.write_str("input truncated"),
            FramerError::DecodeFailed => f.write_str("decode failed"),
            FramerError::EncodeFailed => f.write_str("encode failed"),
        }
    }
}

/// Structured view of a decoded EC→SP ODP-relay response.
///
/// The `body` slice borrows from the caller's input buffer and contains the
/// per-service result payload (i.e. everything after the 4-byte ODP relay
/// header echo).
#[derive(Debug, PartialEq, Eq)]
pub struct OdpRelayResponse<'a> {
    pub is_request: bool,
    pub service_id: u8,
    pub is_error: bool,
    pub message_id: u16,
    pub body: &'a [u8],
}

/// Encode a generic ODP-relay request into `buf` over the SmbusEspiMedium
/// framing used by EC's `uart-service`.
///
/// The framer prepends the 4-byte ODP relay header echo to `payload` before
/// handing it to mctp-rs, so callers pass only the per-service payload bytes
/// (e.g. `&[battery_id]` for `Battery::GetSta`).
///
/// `service_id` is used both as the MCTP destination endpoint ID on the wire
/// and as the `service_id` field of the ODP relay header — these are the same
/// value by ODP convention (each service is its own MCTP endpoint).
///
/// Returns the number of bytes written. The output is the on-wire byte
/// sequence WITHOUT the trailing PEC byte — `uart-service` strips PEC before
/// writing to UART (uart-service/src/lib.rs:88).
pub fn encode_request(
    buf: &mut [u8],
    service_id: u8,
    message_id: u16,
    payload: &[u8],
) -> Result<usize, FramerError> {
    if payload.len() > MAX_PAYLOAD_LEN {
        return Err(FramerError::BufTooSmall);
    }
    // Cheap up-front guard so callers with obviously-too-small buffers fail
    // fast.
    let min_output = FRAME_OVERHEAD + payload.len();
    if buf.len() < min_output {
        return Err(FramerError::BufTooSmall);
    }

    let mut assembly_buf = [0u8; SCRATCH_SIZE];
    let mut ctx =
        MctpPacketContext::<SmbusEspiMedium>::new(SmbusEspiMedium, &mut assembly_buf);

    let reply_ctx = MctpReplyContext::<SmbusEspiMedium> {
        // Note the swap convention (see file-header comment): `source_*`
        // here becomes the on-wire DESTINATION after `SmbusEspiMedium::serialize`
        // / `SerializePacketState::next` swap them. So to put the target
        // service on wire-dst and SP on wire-src, we set source=service, dest=SP.
        source_endpoint_id: EndpointId::Id(service_id),
        destination_endpoint_id: EndpointId::Id(SP_EID),
        packet_sequence_number: MctpSequenceNumber::new(0),
        message_tag: MctpMessageTag::try_from(MCTP_MSG_TAG)
            .map_err(|_| FramerError::EncodeFailed)?,
        medium_context: SmbusEspiReplyContext {
            destination_slave_address: SMBUS_DST_SLAVE,
            source_slave_address: SMBUS_SRC_SLAVE,
        },
    };

    // Outbound ODP relay header for the request.
    let odp_header = OdpRelayHeader {
        is_request: true,
        service_id,
        is_error: false,
        message_id,
    };

    // ODP body = 4-byte relay-header echo (so EC can dispatch without
    // re-parsing the transport header) followed by the per-service payload.
    // Derive the first 4 bytes from `odp_header` rather than hand-encoding
    // them so a future header-layout tweak can't silently desync against the
    // body.
    let header_bytes = odp_header.to_u32().to_be_bytes();
    let mut body_buf = [0u8; MAX_PAYLOAD_LEN + 4];
    body_buf[..4].copy_from_slice(&header_bytes);
    body_buf[4..4 + payload.len()].copy_from_slice(payload);
    let body = OdpRelayBody {
        bytes: &body_buf[..4 + payload.len()],
    };

    let mut packet_state = ctx
        .serialize_packet(reply_ctx, (odp_header, body))
        .map_err(|_| FramerError::EncodeFailed)?;

    // The framer is single-packet (the supported services all fit in one MCTP
    // body), so we expect exactly one `next()`.
    let packet = packet_state
        .next()
        .ok_or(FramerError::EncodeFailed)?
        .map_err(|_| FramerError::EncodeFailed)?;

    // Drop trailing PEC byte to match the on-wire form `uart-service`
    // produces.
    let stripped = packet
        .get(..packet.len().saturating_sub(1))
        .ok_or(FramerError::EncodeFailed)?;

    if buf.len() < stripped.len() {
        return Err(FramerError::BufTooSmall);
    }
    buf[..stripped.len()].copy_from_slice(stripped);
    Ok(stripped.len())
}

/// Decode an EC-originated SmbusEspi+MCTP framed ODP-relay response from `buf`.
///
/// `buf` is the on-wire byte sequence as produced by `uart-service`
/// (i.e. PEC byte already stripped). The returned `body` slice borrows from
/// `buf` and contains the per-service result payload (everything past the
/// 4-byte ODP relay header echo).
///
/// Callers gate on `service_id` themselves — a successful decode does NOT
/// imply the response targets any particular service.
pub fn decode_response(buf: &[u8]) -> Result<OdpRelayResponse<'_>, FramerError> {
    // mctp-rs's `SmbusEspiMedium::deserialize` requires a trailing PEC byte
    // (medium/smbus_espi.rs:46-55). The on-wire form `uart-service` produces
    // does NOT include PEC. Re-attach a placeholder PEC byte so mctp-rs can
    // parse — the deserializer reads but does not validate the PEC value.
    if buf.len() < 4 {
        return Err(FramerError::Truncated);
    }
    let byte_count = buf[2] as usize;
    let needed = 4 + byte_count;
    if buf.len() < needed {
        return Err(FramerError::Truncated);
    }

    let mut padded = [0u8; SCRATCH_SIZE];
    if needed + 1 > padded.len() {
        // Wire-claimed size exceeds our internal padded scratch. Surface as
        // BufTooSmall (the requested operation cannot fit our buffer) rather
        // than DecodeFailed (which implies semantic rejection).
        return Err(FramerError::BufTooSmall);
    }
    padded[..needed].copy_from_slice(&buf[..needed]);
    padded[needed] = 0; // dummy PEC

    let mut assembly_buf = [0u8; SCRATCH_SIZE];
    let mut ctx =
        MctpPacketContext::<SmbusEspiMedium>::new(SmbusEspiMedium, &mut assembly_buf);

    let message = ctx
        .deserialize_packet(&padded[..needed + 1])
        .map_err(|_| FramerError::DecodeFailed)?
        .ok_or(FramerError::DecodeFailed)?;

    let (header, body) = message
        .parse_as::<OdpRelayBodyDecode>()
        .map_err(|_| FramerError::DecodeFailed)?;

    // `body.0` is the per-service result payload (everything past the 4-byte
    // ODP header). It borrows from `assembly_buf` which lives only for this
    // call; copy the few-byte borrow into a slice that borrows from the
    // caller's input instead. We do this by re-slicing `buf` directly.
    //
    // Layout of `buf` (PEC stripped):
    //   [0..4)   SmbusEspi header
    //   [4..8)   MCTP transport header
    //   [8]      MCTP message type byte (0x7d)
    //   [9..13)  ODP relay header
    //   [13..)   per-service result body
    //
    // Defense-in-depth: if `byte_count` is small enough that the header alone
    // doesn't fit (`needed < 13`), `buf.get(13..needed)` is an inverted range.
    // Treat that as truncation rather than silently substituting `&[]`, since
    // a downstream caller that gates only on `service_id` could otherwise
    // accept a malformed-but-mctp-rs-parsed frame.
    let _ = body;
    let result_body = buf.get(13..needed).ok_or(FramerError::Truncated)?;

    Ok(OdpRelayResponse {
        is_request: header.is_request,
        service_id: header.service_id,
        is_error: header.is_error,
        message_id: header.message_id,
        body: result_body,
    })
}

// ---------------------------------------------------------------------------
// ODP relay header + body — minimal local impls of mctp-rs's
// `MctpMessageHeaderTrait` / `MctpMessageTrait` so we can drive
// `serialize_packet` / `parse_as` with the ODP message type (0x7d).
// ---------------------------------------------------------------------------

/// MCTP message type byte that ODP uses for relay traffic (captured wire byte 8).
const ODP_MSG_TYPE: u8 = 0x7d;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct OdpRelayHeader {
    is_request: bool,
    service_id: u8,
    is_error: bool,
    message_id: u16,
}

impl OdpRelayHeader {
    /// Bit-extraction layout per `embedded-services::relay::mctp` `bitfield!`
    /// (embedded-service/src/relay/mod.rs:285-303), wire bytes are
    /// big-endian:
    ///     bit 25     : is_request (1)
    ///     bits 23..16: service_id (8)
    ///     bit 15     : is_error   (1)
    ///     bits 14..0 : message_id (15)
    fn from_u32(raw: u32) -> Self {
        Self {
            is_request: ((raw >> 25) & 1) != 0,
            service_id: ((raw >> 16) & 0xff) as u8,
            is_error: ((raw >> 15) & 1) != 0,
            message_id: (raw & 0x7fff) as u16,
        }
    }

    fn to_u32(self) -> u32 {
        ((self.is_request as u32) << 25)
            | ((self.service_id as u32) << 16)
            | ((self.is_error as u32) << 15)
            | ((self.message_id as u32) & 0x7fff)
    }
}

impl MctpMessageHeaderTrait for OdpRelayHeader {
    fn serialize<M: mctp_rs::MctpMedium>(self, buffer: &mut [u8]) -> MctpPacketResult<usize, M> {
        if buffer.len() < 4 {
            return Err(MctpPacketError::SerializeError(
                "buffer too small for odp relay header",
            ));
        }
        buffer[..4].copy_from_slice(&self.to_u32().to_be_bytes());
        Ok(4)
    }

    fn deserialize<M: mctp_rs::MctpMedium>(
        buffer: &[u8],
    ) -> MctpPacketResult<(Self, &[u8]), M> {
        if buffer.len() < 4 {
            return Err(MctpPacketError::HeaderParseError(
                "buffer too small for odp relay header",
            ));
        }
        let raw = u32::from_be_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
        Ok((OdpRelayHeader::from_u32(raw), &buffer[4..]))
    }
}

/// Encode-side ODP body — borrows the assembled (4-byte header echo +
/// per-service payload) byte sequence so the framer can support variable-length
/// per-service payloads without a const generic. Encode-only — see
/// `OdpRelayBodyDecode` for the decode-side counterpart.
struct OdpRelayBody<'a> {
    bytes: &'a [u8],
}

impl<'buf, 'a> MctpMessageTrait<'buf> for OdpRelayBody<'a> {
    type Header = OdpRelayHeader;
    const MESSAGE_TYPE: u8 = ODP_MSG_TYPE;

    fn serialize<M: mctp_rs::MctpMedium>(self, buffer: &mut [u8]) -> MctpPacketResult<usize, M> {
        if buffer.len() < self.bytes.len() {
            return Err(MctpPacketError::SerializeError(
                "buffer too small for odp relay body",
            ));
        }
        buffer[..self.bytes.len()].copy_from_slice(self.bytes);
        Ok(self.bytes.len())
    }

    fn deserialize<M: mctp_rs::MctpMedium>(
        _: &Self::Header,
        _buffer: &'buf [u8],
    ) -> MctpPacketResult<Self, M> {
        // Encode-side type — should never be used on the decode path.
        Err(MctpPacketError::HeaderParseError(
            "OdpRelayBody is encode-only",
        ))
    }
}

/// Decode-side marker type. Carries no payload — the body bytes are
/// re-sliced from the caller's input buffer in `decode_battery_response`
/// (so they outlive `mctp-rs`'s transient assembly buffer). Renamed from
/// the misleading `OdpRelayBodyOwned`: it owns nothing.
struct OdpRelayBodyDecode;

impl<'buf> MctpMessageTrait<'buf> for OdpRelayBodyDecode {
    type Header = OdpRelayHeader;
    const MESSAGE_TYPE: u8 = ODP_MSG_TYPE;

    fn serialize<M: mctp_rs::MctpMedium>(self, _: &mut [u8]) -> MctpPacketResult<usize, M> {
        Err(MctpPacketError::SerializeError(
            "OdpRelayBodyDecode is decode-only",
        ))
    }

    fn deserialize<M: mctp_rs::MctpMedium>(
        _: &Self::Header,
        _buffer: &'buf [u8],
    ) -> MctpPacketResult<Self, M> {
        Ok(OdpRelayBodyDecode)
    }
}

// ---------------------------------------------------------------------------
// Test fixtures (re-exported for downstream test crates under the
// `test-fixtures` feature). Kept here as the single source of truth for the
// captured ping-pong wire bytes — duplicating them across submodule
// boundaries (e.g. `mctp_ping.rs`'s host tests) silently masks wire-format
// drift if one copy is updated and the other is not.
// ---------------------------------------------------------------------------

#[cfg(any(test, feature = "test-fixtures"))]
pub mod test_fixtures {
    /// Captured TX (SP→EC, Battery::GetSta request, battery_id=0).
    ///
    /// Byte breakdown:
    ///   00 0f 0e 03   SmbusEspi header (dst_slave=0, src_slave=1, byte_count=14, cmd=0x0f MCTP)
    ///   01 08 80 d3   MCTP transport header (ver=1, dst_eid=0x08 EC, src_eid=0x80 SP, SOM/EOM/seq=1/tag=3)
    ///   7d            MCTP message type byte (ODP relay)
    ///   02 08 00 0f   ODP relay header (is_request=1, service_id=0x08, msg_id=15)
    ///   02 08 00 0f 00  ODP body: relay-header re-echo + battery_id=0
    pub const PHASE_12_TX_18B: [u8; 18] = [
        0x00, 0x0f, 0x0e, 0x03,
        0x01, 0x08, 0x80, 0xd3,
        0x7d,
        0x02, 0x08, 0x00, 0x0f,
        0x02, 0x08, 0x00, 0x0f, 0x00,
    ];

    /// Captured RX (EC→SP, Battery error response, message_id=1).
    ///
    /// Byte breakdown:
    ///   00 0f 09 03   SmbusEspi header (byte_count=0x09)
    ///   01 80 08 d3   MCTP transport header (dst_eid=0x80 SP, src_eid=0x08 EC battery)
    ///   7d            MCTP message type byte (ODP relay)
    ///   00 08 80 01   ODP relay header (is_request=0, service_id=0x08, is_error=1, msg_id=1)
    pub const PHASE_12_RX_13B: [u8; 13] = [
        0x00, 0x0f, 0x09, 0x03,
        0x01, 0x80, 0x08, 0xd3,
        0x7d,
        0x00, 0x08, 0x80, 0x01,
    ];
}

#[cfg(test)]
mod tests {
    use super::test_fixtures::{PHASE_12_RX_13B, PHASE_12_TX_18B};
    use super::{decode_response, encode_request, FramerError};
    use pretty_assertions::assert_eq;

    // Concrete service constants used to drive the captured Battery::GetSta
    // ping-pong fixture. The framer itself is service-agnostic — these live
    // in the tests because the fixtures they validate are battery traffic.
    const BATTERY_SVC_ID: u8 = 0x08;
    const BATTERY_GETSTA_MSG_ID: u16 = 15;

    #[test]
    fn encode_request_matches_phase12_battery_capture() {
        let mut buf = [0u8; 64];
        let n = encode_request(
            &mut buf,
            BATTERY_SVC_ID,
            BATTERY_GETSTA_MSG_ID,
            &[/*battery_id=*/ 0u8],
        )
        .expect("encode ok");
        assert_eq!(n, PHASE_12_TX_18B.len(), "encoded length mismatch");
        assert_eq!(&buf[..n], &PHASE_12_TX_18B[..], "encoded bytes diverge from captured wire");
    }

    #[test]
    fn decode_response_parses_phase12_battery_rx() {
        let resp = decode_response(&PHASE_12_RX_13B).expect("decode ok");
        assert!(!resp.is_request, "expected response (is_request=false)");
        assert_eq!(resp.service_id, 0x08, "expected Battery service");
        assert!(resp.is_error, "expected is_error=1 from spike capture");
        assert_eq!(resp.message_id, 1, "expected message_id=1");
        // Captured RX has byte_count=9 → needed=13, so the per-service body
        // is empty (just an ODP relay header echo). Pin that explicitly so a
        // future regression to the body slice offset is caught.
        assert_eq!(resp.body, &[][..], "Phase 12 RX has zero body bytes");
    }

    #[test]
    fn decode_response_extracts_body_bytes() {
        // Synthetic frame with a non-empty per-service body: header byte_count
        // bumped to 0x0d so needed=17, then 4 trailing bytes (0xDE 0xAD 0xBE
        // 0xEF) appended after the 13-byte header. Validates that
        // `decode_response` returns the correct body slice (catches any future
        // regression to the `13..needed` re-slice offset).
        let mut frame = [0u8; 17];
        frame[..PHASE_12_RX_13B.len()].copy_from_slice(&PHASE_12_RX_13B);
        frame[2] = 0x0d; // byte_count = 4 (header) + 9 (transport+ODP) → needed=17
        frame[13] = 0xDE;
        frame[14] = 0xAD;
        frame[15] = 0xBE;
        frame[16] = 0xEF;
        let resp = decode_response(&frame).expect("decode ok");
        assert_eq!(resp.body, &[0xDE, 0xAD, 0xBE, 0xEF][..]);
    }

    /// Roundtrip: encode each interesting battery_id, decode the result,
    /// assert header fields are preserved. Catches encoder regressions where
    /// the literal byte happens to match the captured fixture but the
    /// semantic field has drifted.
    #[test]
    fn encode_decode_roundtrip_preserves_header_fields() {
        for &battery_id in &[0u8, 1, 0x7D, 0x7E, 0xFF] {
            let mut tx = [0u8; 64];
            let n = encode_request(&mut tx, BATTERY_SVC_ID, BATTERY_GETSTA_MSG_ID, &[battery_id])
                .unwrap_or_else(|e| panic!("encode failed for battery_id={battery_id}: {e:?}"));
            // The encoder produces a REQUEST frame; `decode_response` is
            // permissive about request vs response and just decodes the ODP
            // relay header. Assert the fields the SP cares about.
            let resp = decode_response(&tx[..n])
                .unwrap_or_else(|e| panic!("decode failed for battery_id={battery_id}: {e:?}"));
            assert!(resp.is_request, "encoded frame must mark is_request=1");
            assert_eq!(resp.service_id, BATTERY_SVC_ID, "service_id must round-trip");
            assert!(!resp.is_error, "request frames have is_error=0");
            assert_eq!(resp.message_id, BATTERY_GETSTA_MSG_ID, "message_id must round-trip");
            assert_eq!(
                resp.body.last().copied(),
                Some(battery_id),
                "payload byte must appear at end of body"
            );
        }
    }

    #[test]
    fn encode_short_buf_errors() {
        // Pin the BufTooSmall boundary for a 1-byte payload at 18 bytes
        // (FRAME_OVERHEAD=13 + 1 payload byte → 14? no — the header echo
        // adds 4 bytes inside the ODP body, so total = 13 + 4 + 1 = 18).
        let mut too_small = [0u8; 17];
        let result = encode_request(&mut too_small, BATTERY_SVC_ID, BATTERY_GETSTA_MSG_ID, &[0]);
        assert!(
            matches!(result, Err(FramerError::BufTooSmall)),
            "expected BufTooSmall at 17 bytes, got {result:?}"
        );

        let mut just_enough = [0u8; 18];
        let result = encode_request(&mut just_enough, BATTERY_SVC_ID, BATTERY_GETSTA_MSG_ID, &[0]);
        assert!(
            result.is_ok(),
            "expected Ok at 18 bytes, got {result:?}"
        );
    }

    #[test]
    fn encode_oversize_payload_errors_as_buf_too_small() {
        // Payload exceeds MAX_PAYLOAD_LEN — must surface as BufTooSmall
        // (configuration error) not EncodeFailed (mctp-rs rejection).
        let mut buf = [0u8; 256];
        let oversize = [0u8; super::MAX_PAYLOAD_LEN + 1];
        let result = encode_request(&mut buf, 0x10, 0, &oversize);
        assert!(
            matches!(result, Err(FramerError::BufTooSmall)),
            "expected BufTooSmall for oversized payload, got {result:?}"
        );
    }

    #[test]
    fn decode_truncated_header_errors() {
        let result = decode_response(&[0x00, 0x0f]); // truncated SmbusEspi header
        assert!(
            matches!(
                result,
                Err(FramerError::Truncated) | Err(FramerError::DecodeFailed)
            ),
            "expected Truncated or DecodeFailed, got {result:?}"
        );
    }

    #[test]
    fn decode_partial_body_errors() {
        // Header says byte_count=9 → needed=13, but only 6 bytes supplied.
        // Exercises the second truncation guard (`buf.len() < needed`),
        // distinct from the 4-byte header guard above.
        let frame = [0x00u8, 0x0f, 0x09, 0x03, 0x01, 0x80];
        let result = decode_response(&frame);
        assert_eq!(
            result,
            Err(FramerError::Truncated),
            "partial body must surface as Truncated"
        );
    }

    #[test]
    fn decode_oversize_byte_count_errors_as_buf_too_small() {
        // byte_count=200 → needed=204, exceeding the internal 64-byte
        // padded scratch. Regression test for the oversize-claimed-length
        // path — must surface as BufTooSmall (not DecodeFailed).
        let mut big = [0u8; 204];
        big[2] = 200;
        let result = decode_response(&big);
        assert_eq!(
            result,
            Err(FramerError::BufTooSmall),
            "wire-claimed oversize must surface as BufTooSmall (not DecodeFailed)"
        );
    }
}
