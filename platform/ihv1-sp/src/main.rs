// This project is dual-licensed under Apache 2.0 and MIT terms.
// See LICENSE-APACHE and LICENSE-MIT for details.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![deny(clippy::undocumented_unsafe_blocks)]
#![deny(unsafe_op_in_unsafe_fn)]

#[cfg(target_os = "none")]
mod baremetal;

mod services;

#[cfg(not(target_os = "none"))]
fn main() {
    println!("ihv1-sp stub");
}

#[cfg(target_os = "none")]
fn main() -> ! {
    use ec_service_lib::service_list;
    use odp_ffa::Function;

    log::info!("IHV1 Secure Partition - build time: {}", env!("BUILD_TIME"));

    let version = odp_ffa::Version::new().exec().unwrap();
    log::info!("FFA version: {}.{}", version.major(), version.minor());

    service_list![ec_service_lib::services::Thermal::new()]
        .run_message_loop(|_| Ok(()))
        .expect("Error in run_message_loop");

    unreachable!()
}
