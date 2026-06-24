// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![deny(clippy::undocumented_unsafe_blocks)]
#![deny(unsafe_op_in_unsafe_fn)]

#[cfg(target_os = "none")]
mod baremetal;

#[cfg(any(target_os = "none", test))]
mod battery;

#[cfg(not(target_os = "none"))]
fn main() {
    println!("qemu-sp stub");
}

#[cfg(target_os = "none")]
fn main() -> ! {
    use core::cell::RefCell;
    use ec_service_lib::MessageHandler;
    use ec_service_lib::services::{EcRelay, MctpSerialTransport, Thermal};
    use odp_ffa::Function;

    log::info!("QEMU Secure Partition - build time: {}", env!("BUILD_TIME"));

    let version = odp_ffa::Version::new().exec().unwrap();
    log::info!("FFA version: {}.{}", version.major(), version.minor());

    // Shared EC relay: one physical PL011 channel into the EC firmware,
    // borrowed by every relay-backed FFA service (currently Thermal;
    // Battery will migrate in a later task).
    //
    // SAFETY: 0x09040000 is the secure PL011 MMIO region exposed to
    // this SP by the SPMC, declared as an `ec_uart` device-region in
    // the SP DTS (added/uncommented in a follow-up task). The SP
    // manifest grants R/W access of >= 0x40 bytes at this base.
    let pl011 = unsafe { qemu_sp_uart::Pl011Uart::new(0x09040000) };
    let transport = MctpSerialTransport::new(pl011);
    let relay = RefCell::new(EcRelay::new(transport));

    MessageHandler::new()
        .append(Thermal::new(&relay))
        .append(ec_service_lib::services::FwMgmt::new())
        .append(ec_service_lib::services::Notify::new())
        .append(battery::Battery::new())
        .run_message_loop()
        .expect("Error in run_message_loop");

    unreachable!()
}
