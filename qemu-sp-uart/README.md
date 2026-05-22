# qemu-sp-uart

Minimal blocking PL011 MMIO driver for the QEMU SBSA secure-partition's
`ec_uart` device-region. `#![no_std]`, no allocator, no async — only
`core` is used.

## Public API

- `Pl011Uart::new(base: u64) -> Self` (`unsafe`) — production constructor over
  `RawMmio`; `base` is the SP-mapped device-region address.
- `Pl011Uart::write_bytes(&mut self, &[u8]) -> Result<(), Error>` — blocking
  write; polls `UARTFR.TXFF` per byte.
- `Pl011Uart::read_byte_blocking(&mut self) -> Result<u8, Error>` — busy-spin
  forever until `UARTFR.RXFE` clears.

## Base address

Constructor-parameter-driven. The outer platform repo's SP DTS declares
`ec_uart` at `0x60030000` (in the `odp-platform-qemu-sbsa` repo under
`mod/secure-services/platform/linker/qemu-ec-sp.dts`). The platform binary
wires `Pl011Uart::new(0x60030000)` from `qemu-ec-sp::main`. The literal
does NOT appear in this crate.

## Targets

Builds for both:

- `aarch64-unknown-none-softfloat` (workspace default per `rust-toolchain.toml`)
- `aarch64-unknown-none` (canonical embedded triple)

## Safety

`RawMmio::new(base)` is `unsafe`: caller must have mapped at least `0x40`
bytes of device memory (R/W) at `base`, matching the SP manifest
device-region attributes. The driver does NOT touch `UARTCR` / `UARTLCR_H`
— TF-A and QEMU init are assumed.
