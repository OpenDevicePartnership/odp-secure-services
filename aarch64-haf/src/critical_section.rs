use aarch64_cpu::registers::{DAIF, Readable, Writeable};
use core::sync::atomic::{Ordering, compiler_fence};
use critical_section::{Impl, RawRestoreState};
struct AArch64CriticalSection;
critical_section::set_impl!(AArch64CriticalSection);

unsafe impl Impl for AArch64CriticalSection {
    unsafe fn acquire() -> RawRestoreState {
        compiler_fence(Ordering::SeqCst);
        let diaf = DAIF.get();
        DAIF.write(DAIF::I::Masked + DAIF::F::Masked);
        compiler_fence(Ordering::SeqCst);
        diaf
    }

    unsafe fn release(restore_state: RawRestoreState) {
        compiler_fence(Ordering::SeqCst);
        DAIF.set(restore_state);
        compiler_fence(Ordering::SeqCst);
    }
}
