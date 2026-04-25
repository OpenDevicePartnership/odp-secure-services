# sp-mctp-framer

SP-side MCTP framer over `SmbusEspiMedium`. Encodes Battery::GetSta requests
and decodes EC→SP Battery responses byte-for-byte against EC's
`embedded-services::uart-service` framing.

## Usage

```rust
use sp_mctp_framer::{encode_battery_request, decode_battery_response};

let mut tx = [0u8; 32];
let n = encode_battery_request(&mut tx, /* battery_id = */ 0)?;
// transmit tx[..n] over the wire ...
let resp = decode_battery_response(&rx[..])?;
```

Runtime path uses only stack/static buffers (no heap).

## Features

- `test-fixtures` — re-export captured ping-pong wire bytes
  (`PHASE_12_TX_18B`, `PHASE_12_RX_13B`) for downstream test crates.
