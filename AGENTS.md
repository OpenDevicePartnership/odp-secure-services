# AGENTS.md

Guidance for AI coding agents (GitHub Copilot, Claude, Codex, etc.) working in
the `odp-secure-services` repository. Human contributors should also find this a
useful orientation, but the audience is automated assistants and the tone is
prescriptive.

## 1. Project overview

`odp-secure-services` is a sample implementation of the **Embedded Controller
(EC) service** that runs as a dedicated **secure partition** under
[Hafnium](https://hafnium.review.trustedfirmware.org/) on ARM AArch64. It
depends on FF-A, MU UEFI, Hafnium and Trusted Firmware-A (TF-A).

The repository is a Cargo workspace with `resolver = "2"` and a mix of host and
`aarch64-unknown-none` (no_std) crates. The MSRV is **Rust 1.85**.

### Workspace layout

| Crate                  | Purpose                                                 |
|------------------------|---------------------------------------------------------|
| `aarch64-haf`          | Low-level AArch64 / Hafnium runtime support.            |
| `dev-hooks`            | Developer tooling / helper hooks (host-side).           |
| `ec-service-lib`       | Common code shared by EC secure-partition implementations. |
| `espi-device`          | eSPI device abstraction used by the EC service.         |
| `espi-device-stub`     | Stub eSPI device for host-side testing.                 |
| `hafnium`              | Rust bindings for Hafnium hypervisor APIs.              |
| `odp-ffa`              | Rust bindings for the Arm Firmware Framework (FF-A).    |
| `platform/ihv1-sp`     | Secure-partition binary for the IHV1 reference platform. |
| `platform/qemu-sp`     | Secure-partition binary for the QEMU virt platform.     |

The two `platform/*` crates are `aarch64-unknown-none` binaries. Everything
else compiles both on the host (for tests / `cargo check`) and on
`aarch64-unknown-none[-softfloat]`.

## 2. Toolchain and prerequisites

- Rust toolchain is pinned via `rust-toolchain.toml`. Required components:
  `rust-src`, `llvm-tools-preview`, `rustfmt`, `rust-analyzer`.
- Required targets: `aarch64-unknown-none` and `aarch64-unknown-none-softfloat`.
  Install with `rustup target add aarch64-unknown-none aarch64-unknown-none-softfloat`.
- Required cargo subcommands (used by the precommit hook and CI):
  - `cargo-deny ^0.19`
  - `cargo-hack ^0.6`
  - `cargo-binstall` (recommended for fast installs of the above)

Install via:

```sh
cargo binstall cargo-deny@^0.19 -y --force --locked
cargo binstall cargo-hack@^0.6  -y --force --locked
```

## 3. Build, test, lint — the canonical commands

These mirror `.husky/pre-commit` and `.github/workflows/check.yml`. **Run the
precommit script before considering any change done**:

```sh
.husky/pre-commit
```

Individual commands an agent will typically run:

```sh
# Format check (must pass; rustfmt config: max_width = 120)
cargo fmt --check

# Clippy on the secure-partition binaries (aarch64 target)
cargo clippy --target=aarch64-unknown-none -p ihv1-ec-sp -p qemu-ec-sp -- -D warnings

# Clippy on the rest of the workspace (host target)
cargo clippy -- -D warnings

# Host build / type-check across the feature powerset (excluding aarch64-only crates)
cargo hack check --feature-powerset --exclude=ihv1-ec-sp --exclude=qemu-ec-sp

# Cross build feature powerset for the aarch64 target
cargo hack check --feature-powerset --target=aarch64-unknown-none

# Tests run on the host target
cargo test --target "$(rustc -vV | sed -n 's/host: //p')" -q

# Per-platform binary builds
( cd platform/qemu-sp && cargo build --target=aarch64-unknown-none )
( cd platform/ihv1-sp && cargo build --target=aarch64-unknown-none )

# Supply-chain checks
cargo deny --all-features --target=aarch64-unknown-none check
```

Documentation build (nightly, as in CI):

```sh
RUSTDOCFLAGS=--cfg docsrs cargo +nightly doc --no-deps --all-features --target=aarch64-unknown-none
```

To convert a secure-partition ELF to a raw binary image:

```sh
cargo objcopy --target=aarch64-unknown-none -- -O binary output.bin
```

## 4. Workspace conventions

- **Edition**: `2021`. Workspace `version = "0.1.0"`.
- **Lint floor** (set in workspace `Cargo.toml`):
  ```toml
  [workspace.lints.clippy]
  suspicious  = "deny"
  correctness = "deny"
  perf        = "deny"
  style       = "deny"
  ```
  New crates added to the workspace should inherit lints with
  `[lints] workspace = true` and not silently relax these categories.
- **Formatting**: rustfmt with `max_width = 120` (see `rustfmt.toml`).
- **Line endings**: LF. Do not introduce CRLF.
- **Dependencies**: prefer adding to `[workspace.dependencies]` and referencing
  them from member crates with `dep = { workspace = true }`. Match existing
  `default-features = false` patterns when the crate is used in `no_std` code.
- **Release profile** is tuned for size (`opt-level = 's'`, `lto = "fat"`,
  `codegen-units = 1`, `debug = true`, `strip = false`). Don't change without
  cause — secure partitions are size-sensitive and `debug = true` is kept on
  purpose for post-mortem debugging.

## 5. no_std and target-specific notes

- The `platform/*-sp` binaries and the lower-level crates target
  `aarch64-unknown-none[-softfloat]` and are `#![no_std]`. They must not gain a
  transitive dependency on `std`. The `no-std.yml` workflow guards this with
  `cargo check --all-features --target aarch64-unknown-none-softfloat`.
- `cargo hack --feature-powerset` is used in CI to enforce **additive
  features**. Any new feature must not break other feature combinations.
- Memory map and image layout for a given platform live under
  `platform/<plat>/linker/` and must match the DTS configuration of the host
  firmware. See the root `README.md` for the expected fields
  (`load-address`, `entrypoint-offset`, `image-size`, plus the `MEMORY`
  block).

## 6. Working with FF-A / Hafnium code

- FF-A definitions live in `odp-ffa`; Hafnium hypervisor calls in `hafnium`.
  Keep these crates free of platform-specific assumptions — platform glue
  belongs under `platform/<plat>` or `aarch64-haf`.
- `ec-service-lib` is the right place for code that should be shared between
  the IHV1 and QEMU secure partitions.
- Host-side unit tests should use `espi-device-stub` (and `mockall` / `rstest`
  where appropriate) instead of the real `espi-device` so they can run on the
  host target.

## 7. Pull-request workflow for agents

1. Branch from `main`. Keep changes scoped.
2. Run `.husky/pre-commit` locally; do not push if it fails.
3. Commit messages: short imperative subject, wrapped body if needed.
4. CI (`check.yml`, `nostd.yml`, `cargo-vet.yml`, `cargo-vet-pr-comment.yml`)
   must be green. CI runs: `fmt`, `clippy`, `test`, `doc` (nightly), `hack`
   (feature powerset on host and aarch64), `deny`, `msrv` (1.85), and
   `no-std` (`aarch64-unknown-none-softfloat`).
5. `CODEOWNERS` requires review from
   `@OpenDevicePartnership/odp-platform-maintainers`. Any change under
   `**/supply-chain` additionally requires `@OpenDevicePartnership/crate-auditors`
   (these are `cargo vet` audit records — only modify them when intentionally
   updating audits).
6. Licensing: contributions are under the repository's
   [MIT license](LICENSE). See `CONTRIBUTING.md`.

## 8. Things to avoid

- Do not add `unsafe` blocks without a `// SAFETY:` comment explaining the
  invariants.
- Do not pull in `std`-only crates into `no_std` members.
- Do not weaken the workspace clippy lint levels.
- Do not commit changes under `supply-chain/` casually — those are
  `cargo vet` audit records gated by a separate codeowner group.
- Do not force-push shared branches.
- Do not introduce CRLF line endings.

## 9. Quick reference

| Need to…                          | Command                                                                 |
|-----------------------------------|-------------------------------------------------------------------------|
| Run everything CI runs            | `.husky/pre-commit`                                                     |
| Format check                      | `cargo fmt --check`                                                     |
| Lint host code                    | `cargo clippy -- -D warnings`                                           |
| Lint SP binaries                  | `cargo clippy --target=aarch64-unknown-none -p ihv1-ec-sp -p qemu-ec-sp -- -D warnings` |
| Test (host)                       | `cargo test`                                                            |
| Feature powerset (host)           | `cargo hack check --feature-powerset --exclude=ihv1-ec-sp --exclude=qemu-ec-sp` |
| Feature powerset (aarch64)        | `cargo hack check --feature-powerset --target=aarch64-unknown-none`     |
| Build QEMU SP                     | `cd platform/qemu-sp && cargo build --target=aarch64-unknown-none`      |
| Build IHV1 SP                     | `cd platform/ihv1-sp && cargo build --target=aarch64-unknown-none`      |
| Supply-chain check                | `cargo deny --all-features --target=aarch64-unknown-none check`         |
| Doc build (nightly)               | `cargo +nightly doc --no-deps --all-features --target=aarch64-unknown-none` |
