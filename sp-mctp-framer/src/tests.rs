//! Golden-bytes round-trip tests against Phase 12 spike captures.
//!
//! See `.planning/phases/12-spike-mctp-uart-ping/12-VERDICT.md` §(c) for the
//! provenance of the byte sequences asserted below. These bytes were captured
//! verbatim from a successful SP→EC→SP round-trip on the QEMU SBSA secure
//! UART (`0x60030000`) and represent the canonical interop contract this
//! framer must preserve.

#![cfg(test)]

use crate::{decode_battery_response, encode_battery_request, FramerError};
use pretty_assertions::assert_eq;

/// Phase 12 spike-captured TX (SP→EC, Battery::GetSta request, battery_id=0).
///
/// Byte breakdown (verdict §(c)):
///   00 0f 0e 03   SmbusEspi header (dst_slave=0, src_slave=1, byte_count=14, cmd=0x0f MCTP)
///   01 08 80 d3   MCTP transport header (ver=1, dst_eid=0x08 EC, src_eid=0x80 SP, SOM/EOM/seq=1/tag=3)
///   7d            MCTP message type byte (ODP relay)
///   02 08 00 0f   ODP relay header (is_request=1, service_id=0x08, msg_id=15)
///   02 08 00 0f 00  ODP body: relay-header re-echo + battery_id=0
const GOLDEN_TX_18B: &[u8] = &[
    0x00, 0x0f, 0x0e, 0x03,
    0x01, 0x08, 0x80, 0xd3,
    0x7d,
    0x02, 0x08, 0x00, 0x0f,
    0x02, 0x08, 0x00, 0x0f, 0x00,
];

/// Phase 12 spike-captured RX (EC→SP, Battery error response, message_id=1).
///
/// Byte breakdown (verdict §(c)):
///   00 0f 09 03   SmbusEspi header (byte_count=0x09)
///   01 80 08 d3   MCTP transport header (dst_eid=0x80 SP, src_eid=0x08 EC battery)
///   7d            MCTP message type byte (ODP relay)
///   00 08 80 01   ODP relay header (is_request=0, service_id=0x08, is_error=1, msg_id=1)
const GOLDEN_RX_13B: &[u8] = &[
    0x00, 0x0f, 0x09, 0x03,
    0x01, 0x80, 0x08, 0xd3,
    0x7d,
    0x00, 0x08, 0x80, 0x01,
];

#[test]
fn encode_battery_request_matches_phase12_capture() {
    let mut buf = [0u8; 64];
    let n = encode_battery_request(&mut buf, /*battery_id=*/ 0).expect("encode ok");
    assert_eq!(n, GOLDEN_TX_18B.len(), "encoded length mismatch");
    assert_eq!(&buf[..n], GOLDEN_TX_18B, "encoded bytes diverge from Phase 12 capture");
}

#[test]
fn decode_battery_response_parses_phase12_rx() {
    let resp = decode_battery_response(GOLDEN_RX_13B).expect("decode ok");
    assert!(!resp.is_request, "expected response (is_request=false)");
    assert_eq!(resp.service_id, 0x08, "expected Battery service");
    assert!(resp.is_error, "expected is_error=1 from spike capture");
    assert_eq!(resp.message_id, 1, "expected message_id=1");
}

#[test]
fn encode_short_buf_errors() {
    let mut buf = [0u8; 4]; // intentionally too small
    let result = encode_battery_request(&mut buf, 0);
    assert!(
        matches!(result, Err(FramerError::BufTooSmall)),
        "expected BufTooSmall, got {:?}",
        result
    );
}

#[test]
fn decode_truncated_errors() {
    let result = decode_battery_response(&[0x00, 0x0f]); // truncated SmbusEspi header
    assert!(
        matches!(
            result,
            Err(FramerError::Truncated) | Err(FramerError::DecodeFailed)
        ),
        "expected Truncated or DecodeFailed, got {:?}",
        result
    );
}
