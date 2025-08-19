use core::marker::PhantomData;
use embassy_executor::{Spawner, raw};
use odp_ffa::Function;

#[unsafe(export_name = "__pender")]
fn pender(context: *mut ()) {
    let context = context as usize;
    log::info!("pender called with context: {:<08X}", context);
    // do we need to execute an FFA_RUN or FFA_INTERRUPT here?
}

pub struct Executor {
    inner: raw::Executor,
    not_send: PhantomData<*mut ()>,
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}

impl Executor {
    /// Create a new Executor.
    pub fn new() -> Self {
        Self {
            inner: raw::Executor::new(core::ptr::null_mut()),
            not_send: PhantomData,
        }
    }

    /// Run the executor.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        log::info!("Executor::run");
        let version = odp_ffa::Version::new().exec().unwrap();
        log::info!("FFA version: {}.{}", version.major(), version.minor());

        init(self.inner.spawner());
        log::info!("Executor::run: init done");

        loop {
            unsafe {
                log::debug!("Executor::run: polling");
                self.inner.poll();
            };
        }
    }
}
