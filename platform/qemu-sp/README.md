# QEMU Embedded Controller Secure Partition

An implementation of embedded controller secure partition for QEMU haf-ec-service.

- `aarch64-paging` for page table management.
- `aarch64-rt` for the entry point and exception handling.

## Building

```
cargo build --target=aarch64-unknown-none
cargo objcopy --target=aarch64-unknown-none -- -O binary qemu-ec-sp.bin
```
