//! SP-side MCTP framer over `SmbusEspiMedium` for SP↔EC interop.
//!
//! Mirrors EC's `embedded-services::uart-service` framing byte-for-byte
//! (verified by the Phase 12 spike capture; see
//! `.planning/phases/12-spike-mctp-uart-ping/12-VERDICT.md`).
//!
//! Runtime path uses only stack/static buffers — no heap (MF-03).

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
// All on-wire constants below are derived from the Phase 12 spike capture
// (12-VERDICT.md §(c)) cross-checked against:
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
/// EC battery-service MCTP endpoint ID (matches `BatteryService` discriminant).
const BATTERY_SVC_EID: u8 = 0x08;
/// MCTP message tag — `uart-service` hard-codes 3.
const MCTP_MSG_TAG: u8 = 3;
/// SmbusEspi destination slave (semantic — wire byte ordering is swapped by
/// `SmbusEspiMedium::serialize`). Matches `uart-service` line 75: `destination_slave_address: 1`.
const SMBUS_DST_SLAVE: u8 = 1;
/// SmbusEspi source slave. Matches `uart-service` line 76: `source_slave_address: 0`.
const SMBUS_SRC_SLAVE: u8 = 0;
/// Battery::GetSta message id (verdict §(c) byte 12 = 0x0f = 15).
const BATTERY_GETSTA_MSG_ID: u16 = 15;
/// Battery service id used by the ODP relay header (= BATTERY_SVC_EID).
const BATTERY_SVC_ID: u8 = 0x08;

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

/// Structured view of a decoded EC→SP Battery response.
///
/// The `body` slice borrows from the caller's input buffer and contains the
/// per-service result payload (i.e. everything after the 4-byte ODP relay
/// header).
#[derive(Debug, PartialEq, Eq)]
pub struct BatteryResponse<'a> {
    pub is_request: bool,
    pub service_id: u8,
    pub is_error: bool,
    pub message_id: u16,
    pub body: &'a [u8],
}

/// Encode a Battery::GetSta request for `battery_id` into `buf` over the
/// SmbusEspiMedium framing used by EC's `uart-service`.
///
/// Returns the number of bytes written. The output is the on-wire byte
/// sequence WITHOUT the trailing PEC byte — `uart-service` strips PEC before
/// writing to UART (uart-service/src/lib.rs:88), and the Phase 12 capture is
/// the post-strip form. Phase 16 (the actual TX wiring) decides whether to
/// re-add PEC if the underlying transport requires it.
pub fn encode_battery_request(buf: &mut [u8], battery_id: u8) -> Result<usize, FramerError> {
    // Cheap up-front guard so callers with obviously-too-small buffers fail
    // fast (Test 3 — `encode_short_buf_errors`).
    const MIN_OUTPUT: usize = 18;
    if buf.len() < MIN_OUTPUT {
        return Err(FramerError::BufTooSmall);
    }

    // Scratch buffer for mctp-rs's packet assembly. Sized for the worst
    // single-packet SmbusEspi frame (max body 32 + 4 header + 1 PEC = 37);
    // we also need room for the prefix that `serialize_packet` carves off
    // (msg_type 1 + odp header 4 + odp body 5 = 10). Round to 64.
    let mut assembly_buf = [0u8; 64];
    let mut ctx =
        MctpPacketContext::<SmbusEspiMedium>::new(SmbusEspiMedium, &mut assembly_buf);

    let reply_ctx = MctpReplyContext::<SmbusEspiMedium> {
        // Note the swap convention (see file-header comment): `source_*`
        // here becomes the on-wire DESTINATION after `SmbusEspiMedium::serialize`
        // / `SerializePacketState::next` swap them. So to put EC (0x08) on
        // wire-dst and SP (0x80) on wire-src, we set source=EC, dest=SP.
        source_endpoint_id: EndpointId::Id(BATTERY_SVC_EID),
        destination_endpoint_id: EndpointId::Id(SP_EID),
        packet_sequence_number: MctpSequenceNumber::new(0),
        message_tag: MctpMessageTag::try_from(MCTP_MSG_TAG)
            .map_err(|_| FramerError::EncodeFailed)?,
        medium_context: SmbusEspiReplyContext {
            destination_slave_address: SMBUS_DST_SLAVE,
            source_slave_address: SMBUS_SRC_SLAVE,
        },
    };

    // ODP relay header for an outbound Battery::GetSta request:
    //   is_request=1, service_id=0x08, is_error=0, message_id=15
    let odp_header = OdpRelayHeader {
        is_request: true,
        service_id: BATTERY_SVC_ID,
        is_error: false,
        message_id: BATTERY_GETSTA_MSG_ID,
    };

    // ODP body for Battery::GetSta(battery_id) — verdict §(c) shows the
    // payload is the relay-header re-echo + a single battery_id byte.
    let body = OdpRelayBody([
        0x02,                    // is_request=1 (high nibble)
        BATTERY_SVC_ID,
        0x00,
        BATTERY_GETSTA_MSG_ID as u8,
        battery_id,
    ]);

    let mut packet_state = ctx
        .serialize_packet(reply_ctx, (odp_header, body))
        .map_err(|_| FramerError::EncodeFailed)?;

    // The framer is single-packet (Battery::GetSta easily fits in one MCTP
    // body), so we expect exactly one `next()`.
    let packet = packet_state
        .next()
        .ok_or(FramerError::EncodeFailed)?
        .map_err(|_| FramerError::EncodeFailed)?;

    // Drop trailing PEC byte to match the on-wire form `uart-service`
    // produces (and the Phase 12 capture).
    let stripped = packet
        .get(..packet.len().saturating_sub(1))
        .ok_or(FramerError::EncodeFailed)?;

    if buf.len() < stripped.len() {
        return Err(FramerError::BufTooSmall);
    }
    buf[..stripped.len()].copy_from_slice(stripped);
    Ok(stripped.len())
}

/// Decode an EC-originated SmbusEspi+MCTP framed Battery response from `buf`.
///
/// `buf` is the on-wire byte sequence as produced by `uart-service`
/// (i.e. PEC byte already stripped).
pub fn decode_battery_response(buf: &[u8]) -> Result<BatteryResponse<'_>, FramerError> {
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

    let mut padded = [0u8; 64];
    if needed + 1 > padded.len() {
        return Err(FramerError::DecodeFailed);
    }
    padded[..needed].copy_from_slice(&buf[..needed]);
    padded[needed] = 0; // dummy PEC

    let mut assembly_buf = [0u8; 64];
    let mut ctx =
        MctpPacketContext::<SmbusEspiMedium>::new(SmbusEspiMedium, &mut assembly_buf);

    let message = ctx
        .deserialize_packet(&padded[..needed + 1])
        .map_err(|_| FramerError::DecodeFailed)?
        .ok_or(FramerError::DecodeFailed)?;

    let (header, body) = message
        .parse_as::<OdpRelayBodyOwned>()
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
    let _ = body;
    let result_body = buf.get(13..needed).unwrap_or(&[]);

    Ok(BatteryResponse {
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

/// MCTP message type byte that ODP uses for relay traffic (verdict §(c) byte 8).
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

/// Outbound (encode-side) ODP body — owns its 5-byte payload by value so it
/// satisfies the lifetime gymnastics of `MctpMessageTrait<'buf>` without
/// needing a borrow into the caller's stack.
struct OdpRelayBody([u8; 5]);

impl<'buf> MctpMessageTrait<'buf> for OdpRelayBody {
    type Header = OdpRelayHeader;
    const MESSAGE_TYPE: u8 = ODP_MSG_TYPE;

    fn serialize<M: mctp_rs::MctpMedium>(self, buffer: &mut [u8]) -> MctpPacketResult<usize, M> {
        if buffer.len() < self.0.len() {
            return Err(MctpPacketError::SerializeError(
                "buffer too small for odp relay body",
            ));
        }
        buffer[..self.0.len()].copy_from_slice(&self.0);
        Ok(self.0.len())
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

/// Decode-side ODP body wrapper — owns no data; we re-slice the caller's
/// input buffer in `decode_battery_response` to produce the borrow.
struct OdpRelayBodyOwned;

impl<'buf> MctpMessageTrait<'buf> for OdpRelayBodyOwned {
    type Header = OdpRelayHeader;
    const MESSAGE_TYPE: u8 = ODP_MSG_TYPE;

    fn serialize<M: mctp_rs::MctpMedium>(self, _: &mut [u8]) -> MctpPacketResult<usize, M> {
        Err(MctpPacketError::SerializeError(
            "OdpRelayBodyOwned is decode-only",
        ))
    }

    fn deserialize<M: mctp_rs::MctpMedium>(
        _: &Self::Header,
        _buffer: &'buf [u8],
    ) -> MctpPacketResult<Self, M> {
        Ok(OdpRelayBodyOwned)
    }
}

#[cfg(test)]
mod tests;
