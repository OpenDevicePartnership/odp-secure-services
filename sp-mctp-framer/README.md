# sp-mctp-framer

SP-side MCTP framer over `SmbusEspiMedium`. Encodes ODP-relay requests and
decodes ODP-relay responses byte-for-byte against EC's
`embedded-services::uart-service` framing. Service-agnostic — caller
supplies `service_id`, `message_id`, and the per-service payload bytes.

## Usage

```rust
use sp_mctp_framer::{encode_request, decode_response};

const BATTERY_SVC_ID: u8 = 0x08;
const BATTERY_GETSTA_MSG_ID: u16 = 15;

let mut tx = [0u8; 32];
let n = encode_request(&mut tx, BATTERY_SVC_ID, BATTERY_GETSTA_MSG_ID, &[/* battery_id */ 0])?;
// transmit tx[..n] over the wire ...
let resp = decode_response(&rx[..])?;
```

Runtime path uses only stack/static buffers (no heap). Maximum per-service
payload is `MAX_PAYLOAD_LEN` bytes — larger payloads return `BufTooSmall`.

## Features

- `test-fixtures` — re-export captured ping-pong wire bytes
  (`PHASE_12_TX_18B`, `PHASE_12_RX_13B`) for downstream test crates.
  These are the captured Battery::GetSta ping-pong from QEMU SBSA spike-up
  and serve as the reference for both encode and decode validation.
