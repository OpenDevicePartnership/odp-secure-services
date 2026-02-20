use aarch64_haf::{haf_interrupt_handler_impl, HafInterruptHandler};

pub struct IHV1InterriptHandler;

impl HafInterruptHandler for IHV1InterriptHandler {
    fn handle(&self, haf_interrupt_id: hafnium::InterruptId) {
        log::info!("IH1 Interrupt: {:?}", haf_interrupt_id);
    }
}
haf_interrupt_handler_impl!(static IRQ_HANDLER: IHV1InterriptHandler = IHV1InterriptHandler);
