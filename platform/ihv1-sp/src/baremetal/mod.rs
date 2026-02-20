mod interrupt;
mod panic;

use aarch64_rt::entry;
use ec_service_lib::sp_logger::SpLogger;

entry!(aarch64_rt_main);
fn aarch64_rt_main(_arg0: u64, _arg1: u64, _arg2: u64, _arg3: u64) -> ! {
    log::set_logger(&SpLogger).unwrap();
    log::set_max_level(log::LevelFilter::Trace);
    crate::main();
}
