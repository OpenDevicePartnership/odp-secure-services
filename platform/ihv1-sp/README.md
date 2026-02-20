# IHV Embedded Controller Secure Partition

An example secure partition reference for independent hardware vendors.

- `aarch64-paging` for page table management.
- `aarch64-rt` for the entry point and exception handling.

## Building

```
cargo build --target=aarch64-unknown-none
cargo objcopy --target=aarch64-unknown-none -- -O binary ihv1-ec-sp.bin
```
