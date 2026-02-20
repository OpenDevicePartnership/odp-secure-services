use aarch64_cpu::registers::{DAIF, ESR_EL1, FAR_EL1, Readable, Writeable};
use hafnium::{hf_interrupt_deactivate, hf_interrupt_get};
use log::debug;

pub trait HafInterruptHandler {
    fn handle(&self, haf_interrupt_id: hafnium::InterruptId);
}

#[macro_export]
macro_rules! haf_interrupt_handler_impl {
    (static $name:ident: $t: ty = $val:expr) => {
        static $name: $t = $val;

        #[unsafe(no_mangle)]
        #[inline]
        fn _haf_interrupt_handler_handle(haf_interrupt_id: hafnium::InterruptId) {
            <$t as $crate::HafInterruptHandler>::handle(&$name, haf_interrupt_id);
        }
    };
}

unsafe extern "C" {
    fn _haf_interrupt_handler_handle(haf_interrupt_id: hafnium::InterruptId);
}

#[unsafe(no_mangle)]
unsafe extern "C" fn irq_current(_elr: u64, _spsr: u64) -> bool {
    let interrupt_id = match hf_interrupt_get() {
        Some(interrupt_id) => interrupt_id,
        None => panic!("No pending interrupts"),
    };

    if let Err(e) = hf_interrupt_deactivate(interrupt_id) {
        panic!("Failed to deactivate interrupt {:?}: {}", interrupt_id, e);
    }

    // Call the interrupt handler defined by the user
    // This is a type that implements HafInterruptHandler
    unsafe {
        _haf_interrupt_handler_handle(interrupt_id);
    }

    false
}

#[unsafe(no_mangle)]
extern "C" fn sync_exception_current(elr: u64, _spsr: u64) {
    panic!(
        "Unexpected sync_exception_current, esr={:#x}, far={:#x}, elr={:#x}",
        ESR_EL1.get(),
        FAR_EL1.get(),
        elr
    );
}

#[unsafe(no_mangle)]
extern "C" fn fiq_current(_elr: u64, _spsr: u64) {
    panic!("Unexpected fiq_current");
}

#[unsafe(no_mangle)]
extern "C" fn serr_current(_elr: u64, _spsr: u64) {
    panic!("Unexpected serr_current");
}

#[unsafe(no_mangle)]
extern "C" fn sync_lower(_elr: u64, _spsr: u64) {
    panic!("Unexpected sync_lower");
}

#[unsafe(no_mangle)]
extern "C" fn irq_lower(_elr: u64, _spsr: u64) {
    panic!("Unexpected irq_lower");
}

#[unsafe(no_mangle)]
extern "C" fn fiq_lower(_elr: u64, _spsr: u64) {
    panic!("Unexpected fiq_lower");
}

#[unsafe(no_mangle)]
extern "C" fn serr_lower(_elr: u64, _spsr: u64) {
    panic!("Unexpected serr_lower");
}

pub fn enable_arch_interrupts() {
    debug!("enable_interrupts");
    DAIF.write(DAIF::D::Unmasked + DAIF::A::Unmasked + DAIF::I::Unmasked + DAIF::F::Unmasked);
}

pub fn disable_arch_interrupts() {
    debug!("disable_interrupts");
    DAIF.write(DAIF::D::Masked + DAIF::A::Masked + DAIF::I::Masked + DAIF::F::Masked);
}
