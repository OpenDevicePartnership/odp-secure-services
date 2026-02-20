// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![deny(clippy::undocumented_unsafe_blocks)]
#![deny(unsafe_op_in_unsafe_fn)]

#[cfg(target_os = "none")]
mod baremetal;

mod battery;

#[cfg(not(target_os = "none"))]
fn main() {
    println!("qemu-sp stub");
}

#[cfg(target_os = "none")]
#[embassy_executor::main(executor = "embassy_aarch64_haf::Executor")]
async fn embassy_main(_spawner: embassy_executor::Spawner) {
    use ec_service_lib::service_list;

    log::info!("QEMU Secure Partition - build time: {}", env!("BUILD_TIME"));

    service_list![
        ec_service_lib::services::Thermal::new(),
        ec_service_lib::services::FwMgmt::new(),
        ec_service_lib::services::Notify::new(),
        battery::Battery::new()
    ]
    .run_message_loop(async |_| Ok(()))
    .await
    .expect("Error in run_message_loop");
}
